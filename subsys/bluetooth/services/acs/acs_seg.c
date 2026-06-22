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

static void acs_seg_tx_work_handler(struct k_work *work);

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

	LOG_DBG("start len=%u seg_payload=%u", len, seg_payload);

	while (offset < len) {
		uint16_t chunk = MIN(seg_payload, len - offset);
		bool is_first = (offset == 0);
		bool is_last = (offset + chunk >= len);

		pdu[0] = seg_hdr_build(is_first, is_last, counter);
		memcpy(&pdu[1], data + offset, chunk);

		LOG_DBG("send first=%u last=%u counter=%u chunk=%u offset=%u/%u", is_first, is_last,
			counter, chunk, offset, len);
		LOG_HEXDUMP_DBG(pdu, chunk + ACS_SEG_HDR_SIZE, "seg_notify pdu");

		err = bt_gatt_notify(conn, attr, pdu, chunk + ACS_SEG_HDR_SIZE);

		if (err) {
			LOG_ERR("bt_gatt_notify failed: %d", err);
			return err;
		}

		offset += chunk;
		counter = (counter + 1) % ACS_SEG_COUNTER_MAX;
	}

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

	LOG_DBG("first=%u last=%u counter=%u payload_len=%u in_progress=%u buf_len=%u", is_first,
		is_last, counter, payload_len, ctx->rx_in_progress, ctx->buf ? ctx->buf->len : 0);
	LOG_HEXDUMP_DBG(data, len, "seg_rx raw");

	if (is_first) {
		ctx->rx_in_progress = false;
		net_buf_reset(ctx->buf);

		if (payload_len > net_buf_tailroom(ctx->buf)) {
			LOG_ERR("first segment overflow (%u > %zu)", payload_len,
				net_buf_tailroom(ctx->buf));
			return ACS_SEG_RX_ERR_OVERFLOW;
		}

		net_buf_add_mem(ctx->buf, payload, payload_len);

		if (is_last) {
			LOG_DBG("complete in single segment total_len=%u", ctx->buf->len);
			LOG_HEXDUMP_DBG(ctx->buf->data, ctx->buf->len, "seg_rx assembled");
			return ACS_SEG_RX_COMPLETE;
		}

		ctx->rx_in_progress = true;
		ctx->rx_counter = (counter + 1) % ACS_SEG_COUNTER_MAX;
		ctx->rx_deadline = sys_timepoint_calc(ACS_SEG_RX_TIMEOUT);
		LOG_DBG("started reassembly total_len=%u next_counter=%u", ctx->buf->len,
			ctx->rx_counter);
		return ACS_SEG_RX_PENDING;
	}

	if (!ctx->rx_in_progress) {
		LOG_WRN("%s segment without preceding First", is_last ? "Last" : "Continuation");
		return ACS_SEG_RX_ERR_ORPHAN;
	}

	/* Check inter-segment timeout (§3.6.2) */
	if (sys_timepoint_expired(ctx->rx_deadline)) {
		LOG_WRN("30-second inter-segment timeout - aborting reassembly");
		ctx->rx_in_progress = false;
		net_buf_reset(ctx->buf);
		return ACS_SEG_RX_ERR_TIMEOUT;
	}

	if (counter != ctx->rx_counter) {
		LOG_ERR("counter mismatch (expected %u, got %u)", ctx->rx_counter, counter);
		ctx->rx_in_progress = false;
		net_buf_reset(ctx->buf);
		return ACS_SEG_RX_ERR_COUNTER;
	}

	if (payload_len > net_buf_tailroom(ctx->buf)) {
		LOG_ERR("segment overflow (%u > %zu)", payload_len, net_buf_tailroom(ctx->buf));
		ctx->rx_in_progress = false;
		net_buf_reset(ctx->buf);
		return ACS_SEG_RX_ERR_OVERFLOW;
	}

	net_buf_add_mem(ctx->buf, payload, payload_len);

	if (is_last) {
		ctx->rx_in_progress = false;
		LOG_DBG("reassembly complete total_len=%u", ctx->buf->len);
		LOG_HEXDUMP_DBG(ctx->buf->data, ctx->buf->len, "seg_rx assembled");
		return ACS_SEG_RX_COMPLETE;
	}

	ctx->rx_counter = (counter + 1) % ACS_SEG_COUNTER_MAX;
	ctx->rx_deadline = sys_timepoint_calc(ACS_SEG_RX_TIMEOUT);
	LOG_DBG("appended fragment total_len=%u next_counter=%u", ctx->buf->len, ctx->rx_counter);
	return ACS_SEG_RX_PENDING;
}

/* Forward declaration */
static void acs_seg_tx_work_handler(struct k_work *work);

/**
 * @brief Detach the buffer from the seg-TX context.
 *
 * The seg-TX engine borrows the buffer for the duration of the segmented
 * transfer but does not own it.  Callers are responsible for the buffer's
 * lifetime - the plain-CP path frees it in its completion callback, and the
 * protected-resource path lets the owning request context manage it.
 */
static void seg_tx_detach_buf(struct acs_seg_tx_ctx *ctx)
{
	ctx->buf = NULL;
}

static void seg_tx_cleanup(struct acs_seg_tx_ctx *ctx, int err)
{
	acs_seg_tx_completion_cb_t cb = ctx->completion_cb;
	void *ud = ctx->completion_cb_data;
	const struct bt_gatt_attr *attr = ctx->tx_attr;
	struct bt_conn *conn = ctx->tx_conn;

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

static void seg_tx_transfer_complete(struct acs_seg_tx_ctx *ctx, struct bt_conn *conn, int err)
{
	uint16_t buf_len;

	if (!ctx->tx_in_flight) {
		return;
	}

	buf_len = ctx->buf ? ctx->buf->len : 0;
	ctx->tx_in_flight = false;

	if (err) {
		LOG_ERR("confirm error %d", err);
		seg_tx_cleanup(ctx, -EIO);
		return;
	}

	if (ctx->tx_offset < buf_len) {
		LOG_DBG("chunk complete, more pending offset=%u total_len=%u", ctx->tx_offset,
			buf_len);
		k_work_submit_to_queue(acs_get_wq(), &ctx->tx_work);
		return;
	}

	LOG_DBG("transfer complete total_len=%u", buf_len);
	seg_tx_cleanup(ctx, 0);
}

static void seg_tx_indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params,
			       uint8_t err)
{
	struct acs_seg_tx_ctx *ctx = CONTAINER_OF(params, struct acs_seg_tx_ctx, params.ind);

	seg_tx_transfer_complete(ctx, conn, err ? -EIO : 0);
}

static void seg_tx_notify_cb(struct bt_conn *conn, void *user_data)
{
	struct acs_seg_tx_ctx *ctx = user_data;

	seg_tx_transfer_complete(ctx, conn, 0);
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
		LOG_ERR("MTU too small (mtu=%u)", mtu);
		err = -EINVAL;
		goto cleanup;
	}

	chunk = MIN(seg_payload, remaining);
	chunk = MIN(chunk, (uint16_t)(sizeof(ctx->tx_scratch) - ACS_SEG_HDR_SIZE));

	is_first = (ctx->tx_offset == 0);
	is_last = (ctx->tx_offset + chunk >= buf_len);

	ctx->tx_scratch[0] = seg_hdr_build(is_first, is_last, ctx->tx_counter);
	memcpy(&ctx->tx_scratch[1], ctx->buf->data + ctx->tx_offset, chunk);

	LOG_DBG("send first=%u last=%u counter=%u chunk=%u offset=%u/%u", is_first, is_last,
		ctx->tx_counter, chunk, ctx->tx_offset, buf_len);
	LOG_HEXDUMP_DBG(ctx->tx_scratch, chunk + ACS_SEG_HDR_SIZE, "seg_tx pdu");

	ctx->tx_in_flight = true;

	if (ctx->is_indicate) {
		memset(&ctx->params.ind, 0, sizeof(ctx->params.ind));
		ctx->params.ind.attr = ctx->tx_attr;
		ctx->params.ind.func = seg_tx_indicate_cb;
		ctx->params.ind.data = ctx->tx_scratch;
		ctx->params.ind.len = chunk + ACS_SEG_HDR_SIZE;

		err = bt_gatt_indicate(ctx->tx_conn, &ctx->params.ind);
		if (!err) {
			ctx->tx_offset += chunk;
			ctx->tx_counter = (ctx->tx_counter + 1) % ACS_SEG_COUNTER_MAX;
			return;
		}
		ctx->tx_in_flight = false;
		if (err == -EINVAL) {
			LOG_DBG("indicate skipped (CCC not enabled)");
		} else {
			LOG_ERR("bt_gatt_indicate failed: %d", err);
		}
	} else {
		memset(&ctx->params.ntf, 0, sizeof(ctx->params.ntf));
		ctx->params.ntf.attr = ctx->tx_attr;
		ctx->params.ntf.func = seg_tx_notify_cb;
		ctx->params.ntf.user_data = ctx;
		ctx->params.ntf.data = ctx->tx_scratch;
		ctx->params.ntf.len = chunk + ACS_SEG_HDR_SIZE;

		ctx->tx_offset += chunk;
		ctx->tx_counter = (ctx->tx_counter + 1) % ACS_SEG_COUNTER_MAX;

		err = bt_gatt_notify_cb(ctx->tx_conn, &ctx->params.ntf);
		if (!err) {
			return;
		}
		ctx->tx_in_flight = false;
		ctx->tx_offset -= chunk;
		ctx->tx_counter =
			(ctx->tx_counter + ACS_SEG_COUNTER_MAX - 1U) % ACS_SEG_COUNTER_MAX;
		LOG_ERR("bt_gatt_notify_cb failed: %d", err);
	}

cleanup:
	seg_tx_cleanup(ctx, err);
}

void acs_seg_tx_init(struct acs_seg_tx_ctx *ctx, bool indicate)
{
	__ASSERT_NO_MSG(ctx);

	memset(ctx, 0, sizeof(*ctx));
	ctx->is_indicate = indicate;
	k_work_init(&ctx->tx_work, acs_seg_tx_work_handler);
}

void acs_seg_tx_reset(struct acs_seg_tx_ctx *ctx)
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
	__ASSERT_NO_MSG(bt_gatt_is_subscribed(
		conn, attr, ctx->is_indicate ? BT_GATT_CCC_INDICATE : BT_GATT_CCC_NOTIFY));

	if (ctx->tx_in_flight || k_work_is_pending(&ctx->tx_work)) {
		LOG_WRN("TX already in progress");
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
