/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/gatt.h>

#include "acs_seg.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Build a segmentation header byte. */
static inline uint8_t seg_hdr_build(bool first, bool last, uint8_t counter)
{
	return (first ? ACS_SEG_FIRST_MASK : 0U) | (last ? ACS_SEG_LAST_MASK : 0U) |
	       FIELD_PREP(ACS_SEG_COUNTER_MASK, counter);
}

int acs_seg_notify(struct bt_conn *conn, const struct bt_gatt_attr *attr, const uint8_t *data,
		   uint16_t len)
{
	uint8_t pdu[ACS_SEG_HDR_SIZE + CONFIG_BT_ACS_MAX_SEGMENT_SIZE];
	uint16_t seg_payload;
	uint16_t offset;
	uint8_t counter;
	int err;

	if (!conn || !attr || !data || len == 0) {
		return -EINVAL;
	}

	seg_payload = MIN(ACS_SEG_PAYLOAD_SIZE(bt_gatt_get_mtu(conn)),
			  (uint16_t)CONFIG_BT_ACS_MAX_SEGMENT_SIZE);

	if (seg_payload == 0) {
		return -ENOMEM;
	}

	offset = 0;
	counter = 0;

	while (offset < len) {
		uint16_t chunk = MIN(seg_payload, len - offset);
		bool is_first = (offset == 0);
		bool is_last = (offset + chunk >= len);

		pdu[0] = seg_hdr_build(is_first, is_last, counter);
		memcpy(&pdu[1], data + offset, chunk);

		err = bt_gatt_notify(conn, attr, pdu, chunk + ACS_SEG_HDR_SIZE);

		if (err) {
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
	ctx->rx_in_progress = false;
	ctx->rx_counter = 0;
	ctx->buf = NULL;
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
	is_first = (seg_hdr & ACS_SEG_FIRST_MASK) != 0;
	is_last = (seg_hdr & ACS_SEG_LAST_MASK) != 0;
	counter = FIELD_GET(ACS_SEG_COUNTER_MASK, seg_hdr);
	payload = &data[1];
	payload_len = len - 1;

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
			return ACS_SEG_RX_COMPLETE;
		}

		ctx->rx_in_progress = true;
		ctx->rx_counter = (counter + 1) % ACS_SEG_COUNTER_MAX;
		ctx->rx_deadline = sys_timepoint_calc(ACS_SEG_RX_TIMEOUT);
		return ACS_SEG_RX_FRAGMENT;
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
		return ACS_SEG_RX_COMPLETE;
	}

	ctx->rx_counter = (counter + 1) % ACS_SEG_COUNTER_MAX;
	ctx->rx_deadline = sys_timepoint_calc(ACS_SEG_RX_TIMEOUT);
	return ACS_SEG_RX_FRAGMENT;
}

/* Forward declaration */
static void acs_seg_tx_work_handler(struct k_work *work);

static void seg_tx_release_buf(struct acs_seg_tx_ctx *ctx)
{
	if (ctx->buf) {
		net_buf_unref(ctx->buf);
		ctx->buf = NULL;
	}
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
		k_work_submit(&ctx->tx_work);
		return;
	}

	LOG_DBG("seg_tx: all segments confirmed");

	/* Snapshot callbacks before resetting state so tx_on_complete can re-arm. */
	acs_seg_tx_on_complete_t cb = ctx->tx_on_complete;
	void *ud = ctx->tx_on_complete_data;
	const struct bt_gatt_attr *attr = ctx->tx_attr;

	ctx->tx_on_complete = NULL;
	ctx->tx_on_complete_data = NULL;

	bt_conn_unref(ctx->tx_conn);
	ctx->tx_conn = NULL;
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = NULL;
	seg_tx_release_buf(ctx);

	if (cb) {
		cb(conn, attr, 0, ud);
	}
	return;

cleanup_err:
	acs_seg_tx_on_complete_t err_cb = ctx->tx_on_complete;
	void *err_ud = ctx->tx_on_complete_data;
	const struct bt_gatt_attr *err_attr = ctx->tx_attr;

	if (ctx->tx_conn) {
		bt_conn_unref(ctx->tx_conn);
		ctx->tx_conn = NULL;
	}
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = NULL;
	ctx->tx_on_complete = NULL;
	ctx->tx_on_complete_data = NULL;
	seg_tx_release_buf(ctx);
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

	acs_seg_tx_on_complete_t cb;
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
	cb = ctx->tx_on_complete;
	ud = ctx->tx_on_complete_data;
	attr = ctx->tx_attr;
	conn = ctx->tx_conn;

	ctx->tx_conn = NULL;
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = NULL;
	ctx->tx_on_complete = NULL;
	ctx->tx_on_complete_data = NULL;
	seg_tx_release_buf(ctx);
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
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_in_flight = false;
	ctx->tx_attr = NULL;
	ctx->tx_conn = NULL;
	ctx->buf = NULL;
	ctx->tx_on_complete = NULL;
	ctx->tx_on_complete_data = NULL;
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
	seg_tx_release_buf(ctx);

	ctx->tx_on_complete = NULL;
	ctx->tx_on_complete_data = NULL;
}

int acs_seg_tx_send(struct acs_seg_tx_ctx *ctx, struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, struct net_buf *buf,
		    acs_seg_tx_on_complete_t tx_on_complete, void *user_data)
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
	ctx->tx_on_complete = tx_on_complete;
	ctx->tx_on_complete_data = user_data;

	__ASSERT_NO_MSG(ctx->tx_conn == NULL);
	ctx->tx_conn = bt_conn_ref(conn);

	k_work_submit(&ctx->tx_work);
	return 0;
}
