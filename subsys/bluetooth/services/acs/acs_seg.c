/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/gatt.h>

#include "acs_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Build a segmentation header byte. */
static inline uint8_t seg_hdr_build(bool first, bool last, uint8_t counter)
{
	return (first ? BIT(ACS_SEG_FIRST_SEGMENT_BIT) : 0U) |
	       (last ? BIT(ACS_SEG_LAST_SEGMENT_BIT) : 0U) |
	       FIELD_PREP(ACS_SEG_COUNTER_MASK, counter);
}

static void acs_seg_notify_work_handler(struct k_work *work);
static void acs_seg_notify_complete_cb(struct bt_conn *conn, void *user_data);

int acs_seg_notify(struct bt_conn *conn, const struct bt_gatt_attr *attr, const uint8_t *data,
		   uint16_t len)
{
	uint8_t pdu[ACS_SEG_CTX_PDU_SIZE];
	uint16_t seg_payload;
	uint16_t offset;
	uint8_t counter;
	int err;

	if (!conn || !attr || !data || len == 0) {
		return -EINVAL;
	}

	seg_payload = MIN(ACS_SEG_PAYLOAD_SIZE(bt_gatt_get_mtu(conn)),
			  (uint16_t)(sizeof(pdu) - ACS_SEG_HDR_SIZE));

	if (seg_payload == 0) {
		return -ENOMEM;
	}

	offset = 0;
	counter = 0;

	LOG_DBG("seg_notify: start len=%u seg_payload=%u", len, seg_payload);

	while (offset < len) {
		uint16_t chunk = MIN(seg_payload, len - offset);
		bool is_first = (offset == 0);
		bool is_last = (offset + chunk >= len);

		pdu[0] = seg_hdr_build(is_first, is_last, counter);
		memcpy(&pdu[1], data + offset, chunk);

		LOG_DBG("seg_notify: send first=%u last=%u counter=%u chunk=%u offset=%u/%u",
			is_first, is_last, counter, chunk, offset, len);
		LOG_HEXDUMP_DBG(pdu, chunk + ACS_SEG_HDR_SIZE, "seg_notify pdu");

		err = bt_gatt_notify(conn, attr, pdu, chunk + ACS_SEG_HDR_SIZE);

		if (err) {
			LOG_ERR("seg_notify: bt_gatt_notify failed: %d", err);
			return err;
		}

		offset += chunk;
		counter = (counter + 1) % ACS_SEG_COUNTER_MAX;
	}

	return 0;
}

static void notify_tx_detach_buf(struct acs_seg_notify_ctx *ctx)
{
	ctx->buf = NULL;
}

static void acs_seg_notify_complete_cb(struct bt_conn *conn, void *user_data)
{
	struct acs_seg_notify_ctx *ctx = user_data;
	uint16_t buf_len;

	if (!ctx || !ctx->tx_in_flight) {
		return; /* disconnect cleanup already claimed this context */
	}

	buf_len = ctx->buf ? ctx->buf->len : 0;
	ctx->tx_in_flight = false;

	if (ctx->tx_offset < buf_len) {
		LOG_DBG("seg_notify_async: chunk complete, more pending offset=%u total_len=%u",
			ctx->tx_offset, buf_len);
		k_work_submit_to_queue(acs_get_wq(), &ctx->tx_work);
		return;
	}

	LOG_DBG("seg_notify_async: transfer complete total_len=%u", buf_len);

	acs_seg_tx_completion_cb_t cb = ctx->completion_cb;
	void *ud = ctx->completion_cb_data;
	const struct bt_gatt_attr *attr = ctx->tx_attr;

	ctx->completion_cb = NULL;
	ctx->completion_cb_data = NULL;

	bt_conn_unref(ctx->tx_conn);
	ctx->tx_conn = NULL;
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = NULL;
	notify_tx_detach_buf(ctx);

	if (cb) {
		cb(conn, attr, 0, ud);
	}
}

static void acs_seg_notify_work_handler(struct k_work *work)
{
	struct acs_seg_notify_ctx *ctx = CONTAINER_OF(work, struct acs_seg_notify_ctx, tx_work);
	uint16_t seg_payload;
	uint16_t buf_len;
	uint16_t remaining;
	uint16_t chunk;
	uint16_t mtu;
	bool is_first;
	bool is_last;
	int err;

	acs_seg_tx_completion_cb_t cb;
	void *ud;
	const struct bt_gatt_attr *attr;
	struct bt_conn *conn;

	__ASSERT_NO_MSG(ctx->tx_conn);
	__ASSERT_NO_MSG(ctx->buf);
	__ASSERT_NO_MSG(ctx->tx_attr);
	__ASSERT_NO_MSG(!ctx->tx_in_flight);

	buf_len = ctx->buf->len;
	remaining = buf_len - ctx->tx_offset;

	__ASSERT_NO_MSG(remaining > 0);

	mtu = bt_gatt_get_mtu(ctx->tx_conn);
	seg_payload = ACS_SEG_PAYLOAD_SIZE(mtu);

	if (seg_payload == 0) {
		LOG_ERR("seg_notify_async work: MTU too small (mtu=%u)", mtu);
		err = -EINVAL;
		goto cleanup;
	}

	chunk = MIN(seg_payload, remaining);
	chunk = MIN(chunk, (uint16_t)(sizeof(ctx->tx_scratch) - ACS_SEG_HDR_SIZE));

	is_first = (ctx->tx_offset == 0);
	is_last = (ctx->tx_offset + chunk >= buf_len);

	ctx->tx_scratch[0] = seg_hdr_build(is_first, is_last, ctx->tx_counter);
	memcpy(&ctx->tx_scratch[1], ctx->buf->data + ctx->tx_offset, chunk);

	LOG_DBG("seg_notify_async: send first=%u last=%u counter=%u chunk=%u offset=%u/%u",
		is_first, is_last, ctx->tx_counter, chunk, ctx->tx_offset, buf_len);
	LOG_HEXDUMP_DBG(ctx->tx_scratch, chunk + ACS_SEG_HDR_SIZE, "seg_notify_async pdu");

	memset(&ctx->notify_params, 0, sizeof(ctx->notify_params));
	ctx->notify_params.attr = ctx->tx_attr;
	ctx->notify_params.func = acs_seg_notify_complete_cb;
	ctx->notify_params.user_data = ctx;
	ctx->notify_params.data = ctx->tx_scratch;
	ctx->notify_params.len = chunk + ACS_SEG_HDR_SIZE;

	ctx->tx_in_flight = true;

	err = bt_gatt_notify_cb(ctx->tx_conn, &ctx->notify_params);
	if (err) {
		ctx->tx_in_flight = false;
		LOG_ERR("seg_notify_async: bt_gatt_notify_cb failed: %d", err);
		goto cleanup;
	}

	ctx->tx_offset += chunk;
	ctx->tx_counter = (ctx->tx_counter + 1) % ACS_SEG_COUNTER_MAX;
	return;

cleanup:
	cb = ctx->completion_cb;
	ud = ctx->completion_cb_data;
	attr = ctx->tx_attr;
	conn = ctx->tx_conn;

	ctx->tx_conn = NULL;
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = NULL;
	ctx->completion_cb = NULL;
	ctx->completion_cb_data = NULL;
	notify_tx_detach_buf(ctx);
	if (cb) {
		cb(conn, attr, err, ud);
	}
	if (conn) {
		bt_conn_unref(conn);
	}
}

void acs_seg_notify_async_init(struct acs_seg_notify_ctx *ctx)
{
	__ASSERT_NO_MSG(ctx);

	memset(ctx, 0, sizeof(*ctx));
	k_work_init(&ctx->tx_work, acs_seg_notify_work_handler);
}

void acs_seg_notify_async_reset(struct acs_seg_notify_ctx *ctx)
{
	struct k_work_sync sync;

	__ASSERT_NO_MSG(ctx != NULL);

	ctx->tx_in_flight = false;

	k_work_cancel_sync(&ctx->tx_work, &sync);
	if (ctx->tx_conn) {
		bt_conn_unref(ctx->tx_conn);
		ctx->tx_conn = NULL;
	}
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = NULL;

	notify_tx_detach_buf(ctx);

	ctx->completion_cb = NULL;
	ctx->completion_cb_data = NULL;
}

int acs_seg_notify_async_send(struct acs_seg_notify_ctx *ctx, struct bt_conn *conn,
			      const struct bt_gatt_attr *attr, struct net_buf *buf,
			      acs_seg_tx_completion_cb_t completion_cb, void *user_data)
{
	__ASSERT_NO_MSG(ctx != NULL);
	__ASSERT_NO_MSG(conn != NULL);
	__ASSERT_NO_MSG(attr != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(buf->len > 0);
	__ASSERT_NO_MSG(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY));

	if (ctx->tx_in_flight || k_work_is_pending(&ctx->tx_work)) {
		LOG_WRN("seg_notify_async_send: TX already in progress");
		return -EBUSY;
	}

	ctx->buf = buf;
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = attr;
	ctx->completion_cb = completion_cb;
	ctx->completion_cb_data = user_data;

	__ASSERT_NO_MSG(ctx->tx_conn == NULL);
	ctx->tx_conn = bt_conn_ref(conn);

	k_work_submit_to_queue(acs_get_wq(), &ctx->tx_work);
	return 0;
}

void acs_seg_rx_init(struct acs_seg_rx_ctx *ctx)
{
	__ASSERT_NO_MSG(ctx);

	memset(ctx, 0, sizeof(*ctx));
}

void acs_seg_rx_reset(struct acs_seg_rx_ctx *ctx)
{
	__ASSERT_NO_MSG(ctx != NULL);

	ctx->rx_in_progress = false;
	ctx->rx_counter = 0;

	if (ctx->buf) {
		net_buf_unref(ctx->buf);
		ctx->buf = NULL;
	}
}

void acs_seg_rx_begin(struct acs_seg_rx_ctx *ctx, struct net_buf *buf)
{
	__ASSERT_NO_MSG(ctx != NULL);
	__ASSERT_NO_MSG(buf != NULL);

	ctx->buf = buf;
}

enum acs_seg_rx_result acs_seg_rx_process(struct acs_seg_rx_ctx *ctx, const uint8_t *data,
					  uint16_t len)
{
	const uint8_t *payload;
	uint16_t payload_len;
	uint8_t seg_hdr;
	uint8_t counter;
	bool is_first;
	bool is_last;

	__ASSERT_NO_MSG(ctx != NULL);
	__ASSERT_NO_MSG(data != NULL);
	__ASSERT_NO_MSG(ctx->buf != NULL);
	__ASSERT_NO_MSG(ctx->buf->size > 0);

	if (len < 1) {
		return ACS_SEG_RX_ERR_LEN;
	}

	seg_hdr = data[0];
	is_first = IS_BIT_SET(seg_hdr, ACS_SEG_FIRST_SEGMENT_BIT);
	is_last = IS_BIT_SET(seg_hdr, ACS_SEG_LAST_SEGMENT_BIT);
	counter = FIELD_GET(ACS_SEG_COUNTER_MASK, seg_hdr);
	payload = &data[1];
	payload_len = len - 1;

	LOG_DBG("seg_rx: first=%u last=%u counter=%u payload_len=%u in_progress=%u buf_len=%u",
		is_first, is_last, counter, payload_len, ctx->rx_in_progress,
		ctx->buf ? ctx->buf->len : 0);
	LOG_HEXDUMP_DBG(data, len, "seg_rx raw");

	if (is_first) {
		ctx->rx_in_progress = false;
		net_buf_reset(ctx->buf);

		if (payload_len > net_buf_tailroom(ctx->buf)) {
			LOG_ERR("seg_rx: first segment overflow (%u > %zu)", payload_len,
				net_buf_tailroom(ctx->buf));
			return ACS_SEG_RX_ERR_OVERFLOW;
		}

		net_buf_add_mem(ctx->buf, payload, payload_len);

		if (is_last) {
			LOG_DBG("seg_rx: complete in single segment total_len=%u", ctx->buf->len);
			LOG_HEXDUMP_DBG(ctx->buf->data, ctx->buf->len, "seg_rx assembled");
			return ACS_SEG_RX_COMPLETE;
		}

		ctx->rx_in_progress = true;
		ctx->rx_counter = (counter + 1) % ACS_SEG_COUNTER_MAX;
		ctx->rx_deadline = sys_timepoint_calc(ACS_SEG_RX_TIMEOUT);
		LOG_DBG("seg_rx: started reassembly total_len=%u next_counter=%u", ctx->buf->len,
			ctx->rx_counter);
		return ACS_SEG_RX_PENDING;
	}

	if (!ctx->rx_in_progress) {
		LOG_WRN("seg_rx: %s segment without preceding First",
			is_last ? "Last" : "Continuation");
		return ACS_SEG_RX_ERR_ORPHAN;
	}

	/* Check inter-segment timeout (§3.6.2) */
	if (sys_timepoint_expired(ctx->rx_deadline)) {
		LOG_WRN("seg_rx: 30-second inter-segment timeout — aborting reassembly");
		ctx->rx_in_progress = false;
		net_buf_reset(ctx->buf);
		return ACS_SEG_RX_ERR_TIMEOUT;
	}

	if (counter != ctx->rx_counter) {
		LOG_ERR("seg_rx: counter mismatch (expected %u, got %u)", ctx->rx_counter, counter);
		ctx->rx_in_progress = false;
		net_buf_reset(ctx->buf);
		return ACS_SEG_RX_ERR_COUNTER;
	}

	if (payload_len > net_buf_tailroom(ctx->buf)) {
		LOG_ERR("seg_rx: segment overflow (%u > %zu)", payload_len,
			net_buf_tailroom(ctx->buf));
		ctx->rx_in_progress = false;
		net_buf_reset(ctx->buf);
		return ACS_SEG_RX_ERR_OVERFLOW;
	}

	net_buf_add_mem(ctx->buf, payload, payload_len);

	if (is_last) {
		ctx->rx_in_progress = false;
		LOG_DBG("seg_rx: reassembly complete total_len=%u", ctx->buf->len);
		LOG_HEXDUMP_DBG(ctx->buf->data, ctx->buf->len, "seg_rx assembled");
		return ACS_SEG_RX_COMPLETE;
	}

	ctx->rx_counter = (counter + 1) % ACS_SEG_COUNTER_MAX;
	ctx->rx_deadline = sys_timepoint_calc(ACS_SEG_RX_TIMEOUT);
	LOG_DBG("seg_rx: appended fragment total_len=%u next_counter=%u", ctx->buf->len,
		ctx->rx_counter);
	return ACS_SEG_RX_PENDING;
}

/* Forward declaration */
static void acs_seg_tx_work_handler(struct k_work *work);

/**
 * @brief Detach the buffer from the seg-TX context.
 *
 * The seg-TX engine borrows the buffer for the duration of the segmented
 * transfer but does not own it.  Callers are responsible for the buffer's
 * lifetime — the plain-CP path frees it in its completion callback, and the
 * protected-resource path lets the owning request context manage it.
 */
static void seg_tx_detach_buf(struct acs_seg_tx_ctx *ctx)
{
	ctx->buf = NULL;
}

/* TX indication confirm callback. */
static void acs_seg_tx_confirm_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params,
				  uint8_t err)
{
	struct acs_seg_tx_ctx *ctx = CONTAINER_OF(params, struct acs_seg_tx_ctx, ind_params);
	uint16_t buf_len;

	if (!ctx->tx_in_flight) {
		return; /* disconnect cleanup already claimed this context */
	}

	buf_len = ctx->buf ? ctx->buf->len : 0;

	ctx->tx_in_flight = false;

	if (err) {
		LOG_ERR("seg_tx: indication confirm error 0x%02x", err);
		goto cleanup_err;
	}

	if (ctx->tx_offset < buf_len) {
		LOG_DBG("seg_tx: indication confirmed, more segments pending offset=%u "
			"total_len=%u",
			ctx->tx_offset, buf_len);
		k_work_submit_to_queue(acs_get_wq(), &ctx->tx_work);
		return;
	}

	LOG_DBG("seg_tx: indication confirmed, transfer complete total_len=%u", buf_len);

	/* Snapshot callbacks before resetting state so completion_cb can re-arm. */
	acs_seg_tx_completion_cb_t cb = ctx->completion_cb;
	void *ud = ctx->completion_cb_data;
	const struct bt_gatt_attr *attr = ctx->tx_attr;

	ctx->completion_cb = NULL;
	ctx->completion_cb_data = NULL;

	bt_conn_unref(ctx->tx_conn);
	ctx->tx_conn = NULL;
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = NULL;
	seg_tx_detach_buf(ctx);

	if (cb) {
		cb(conn, attr, 0, ud);
	}
	return;

cleanup_err:
	acs_seg_tx_completion_cb_t err_cb = ctx->completion_cb;
	void *err_ud = ctx->completion_cb_data;
	const struct bt_gatt_attr *err_attr = ctx->tx_attr;

	if (ctx->tx_conn) {
		bt_conn_unref(ctx->tx_conn);
		ctx->tx_conn = NULL;
	}
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = NULL;
	ctx->completion_cb = NULL;
	ctx->completion_cb_data = NULL;
	seg_tx_detach_buf(ctx);
	if (err_cb) {
		err_cb(conn, err_attr, -EIO, err_ud);
	}
}

static void acs_seg_tx_work_handler(struct k_work *work)
{
	struct acs_seg_tx_ctx *ctx = CONTAINER_OF(work, struct acs_seg_tx_ctx, tx_work);
	uint16_t seg_payload;
	uint16_t buf_len;
	uint16_t remaining;
	uint16_t chunk;
	uint16_t mtu;
	bool is_first;
	bool is_last;
	int err;

	acs_seg_tx_completion_cb_t cb;
	void *ud;
	const struct bt_gatt_attr *attr;
	struct bt_conn *conn;

	__ASSERT_NO_MSG(ctx->tx_conn);
	__ASSERT_NO_MSG(ctx->buf);
	__ASSERT_NO_MSG(ctx->tx_attr);
	__ASSERT_NO_MSG(!ctx->tx_in_flight);

	buf_len = ctx->buf->len;
	remaining = buf_len - ctx->tx_offset;

	__ASSERT_NO_MSG(remaining > 0);

	mtu = bt_gatt_get_mtu(ctx->tx_conn);
	seg_payload = ACS_SEG_PAYLOAD_SIZE(mtu);

	if (seg_payload == 0) {
		LOG_ERR("seg_tx work: MTU too small (mtu=%u)", mtu);
		err = -EINVAL;
		goto cleanup;
	}

	/* Limit chunk to fit in our small PDU scratch buffer */
	chunk = MIN(seg_payload, remaining);
	chunk = MIN(chunk, (uint16_t)(sizeof(ctx->tx_scratch) - ACS_SEG_HDR_SIZE));

	is_first = (ctx->tx_offset == 0);
	is_last = (ctx->tx_offset + chunk >= buf_len);

	/* Build PDU in scratch buffer: [header][payload_chunk] */
	ctx->tx_scratch[0] = seg_hdr_build(is_first, is_last, ctx->tx_counter);
	memcpy(&ctx->tx_scratch[1], ctx->buf->data + ctx->tx_offset, chunk);

	LOG_DBG("seg_tx: send first=%u last=%u counter=%u chunk=%u offset=%u/%u", is_first, is_last,
		ctx->tx_counter, chunk, ctx->tx_offset, buf_len);
	LOG_HEXDUMP_DBG(ctx->tx_scratch, chunk + ACS_SEG_HDR_SIZE, "seg_tx pdu");

	memset(&ctx->ind_params, 0, sizeof(ctx->ind_params));
	ctx->ind_params.attr = ctx->tx_attr;
	ctx->ind_params.func = acs_seg_tx_confirm_cb;
	ctx->ind_params.data = ctx->tx_scratch;
	ctx->ind_params.len = chunk + ACS_SEG_HDR_SIZE;

	ctx->tx_in_flight = true;

	err = bt_gatt_indicate(ctx->tx_conn, &ctx->ind_params);

	if (err) {
		ctx->tx_in_flight = false;
		if (err == -EINVAL) {
			LOG_DBG("seg_tx: bt_gatt_indicate skipped (CCC not enabled)");
		} else {
			LOG_ERR("seg_tx: bt_gatt_indicate failed: %d", err);
		}
		goto cleanup;
	}

	ctx->tx_offset += chunk;
	ctx->tx_counter = (ctx->tx_counter + 1) % ACS_SEG_COUNTER_MAX;
	return;

cleanup:
	cb = ctx->completion_cb;
	ud = ctx->completion_cb_data;
	attr = ctx->tx_attr;
	conn = ctx->tx_conn;

	ctx->tx_conn = NULL;
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = NULL;
	ctx->completion_cb = NULL;
	ctx->completion_cb_data = NULL;
	seg_tx_detach_buf(ctx);
	if (cb) {
		cb(conn, attr, err, ud);
	}
	if (conn) {
		bt_conn_unref(conn);
	}
}

void acs_seg_tx_init(struct acs_seg_tx_ctx *ctx)
{
	__ASSERT_NO_MSG(ctx);

	memset(ctx, 0, sizeof(*ctx));
	k_work_init(&ctx->tx_work, acs_seg_tx_work_handler);
}

void acs_seg_tx_reset(struct acs_seg_tx_ctx *ctx)
{
	struct k_work_sync sync;

	__ASSERT_NO_MSG(ctx != NULL);

	/* Set tx_in_flight = false first, so a racing GATT confirm callback
	 * sees this and bails out before touching other fields.
	 */
	ctx->tx_in_flight = false;

	/* Cancel TX work and release connection ref */
	k_work_cancel_sync(&ctx->tx_work, &sync);
	if (ctx->tx_conn) {
		bt_conn_unref(ctx->tx_conn);
		ctx->tx_conn = NULL;
	}
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = NULL;

	/* Release buffer if present */
	seg_tx_detach_buf(ctx);

	ctx->completion_cb = NULL;
	ctx->completion_cb_data = NULL;
}

int acs_seg_tx_send(struct acs_seg_tx_ctx *ctx, struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, struct net_buf *buf,
		    acs_seg_tx_completion_cb_t completion_cb, void *user_data)
{
	__ASSERT_NO_MSG(ctx != NULL);
	__ASSERT_NO_MSG(conn != NULL);
	__ASSERT_NO_MSG(attr != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(buf->len > 0);
	__ASSERT_NO_MSG(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_INDICATE));

	if (ctx->tx_in_flight || k_work_is_pending(&ctx->tx_work)) {
		LOG_WRN("seg_tx_send: TX already in progress");
		return -EBUSY;
	}

	ctx->buf = buf;
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = attr;
	ctx->completion_cb = completion_cb;
	ctx->completion_cb_data = user_data;

	__ASSERT_NO_MSG(ctx->tx_conn == NULL);
	ctx->tx_conn = bt_conn_ref(conn);

	k_work_submit_to_queue(acs_get_wq(), &ctx->tx_work);
	return 0;
}

enum acs_seg_rx_result acs_channel_rx_feed(struct acs_seg_rx_ctx *rx_ctx, const uint8_t *data,
					   uint16_t len)
{
	enum acs_seg_rx_result res;

	if (len < 1) {
		return ACS_SEG_RX_ERR_LEN;
	}

	if (IS_BIT_SET(data[0], ACS_SEG_FIRST_SEGMENT_BIT) && !rx_ctx->buf) {
		struct net_buf *rx_buf = acs_buf_alloc(K_NO_WAIT);

		if (!rx_buf) {
			LOG_ERR("Channel RX: buffer pool exhausted");
			return ACS_SEG_RX_ERR_OVERFLOW;
		}
		acs_seg_rx_begin(rx_ctx, rx_buf);
	}

	if (!rx_ctx->buf) {
		LOG_ERR("Channel RX: continuation segment without prior first segment");
		return ACS_SEG_RX_ERR_ORPHAN;
	}

	res = acs_seg_rx_process(rx_ctx, data, len);

	if (res != ACS_SEG_RX_COMPLETE && res != ACS_SEG_RX_PENDING) {
		acs_seg_rx_reset(rx_ctx);
	}

	return res;
}
