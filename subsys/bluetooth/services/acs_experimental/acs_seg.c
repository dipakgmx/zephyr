/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "acs_seg.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static inline uint8_t acs_seg_hdr_build(bool first, bool last, uint8_t counter)
{
	return (first ? ACS_SEG_FIRST_MASK : 0U) | (last ? ACS_SEG_LAST_MASK : 0U) |
	       FIELD_PREP(ACS_SEG_COUNTER_MASK, counter);
}

static void acs_seg_tx_work_handler(struct k_work *work);

static void acs_seg_tx_detach_buf(struct acs_seg_tx_ctx *ctx)
{
	ctx->buf = NULL;
}

static void acs_seg_tx_complete(struct acs_seg_tx_ctx *ctx, struct bt_conn *conn,
				const struct bt_gatt_attr *attr, int err)
{
	acs_seg_tx_on_complete_t cb = ctx->tx_on_complete;
	void *user_data = ctx->tx_on_complete_data;

	ctx->tx_on_complete = NULL;
	ctx->tx_on_complete_data = NULL;

	if (cb) {
		cb(conn, attr, err, user_data);
	}
}

static void acs_seg_tx_confirm_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params,
				  uint8_t err)
{
	struct acs_seg_tx_ctx *ctx = CONTAINER_OF(params, struct acs_seg_tx_ctx, ind_params);
	struct bt_conn *tx_conn = ctx->tx_conn;
	const struct bt_gatt_attr *attr = ctx->tx_attr;
	uint16_t buf_len;

	if (!ctx->tx_in_flight) {
		return;
	}

	buf_len = ctx->buf ? ctx->buf->len : 0U;
	ctx->tx_in_flight = false;

	if (err != 0U) {
		LOG_WRN("seg_tx: indication confirm error 0x%02x", err);
		goto cleanup_err;
	}

	if (ctx->offset < buf_len) {
		k_work_submit(&ctx->tx_work);
		return;
	}

	if (tx_conn) {
		bt_conn_unref(tx_conn);
		ctx->tx_conn = NULL;
	}

	ctx->offset = 0U;
	ctx->counter = 0U;
	ctx->tx_attr = NULL;
	acs_seg_tx_detach_buf(ctx);
	memset(&ctx->ind_params, 0, sizeof(ctx->ind_params));
	acs_seg_tx_complete(ctx, conn, attr, 0);
	return;

cleanup_err:
	if (tx_conn) {
		bt_conn_unref(tx_conn);
		ctx->tx_conn = NULL;
	}

	ctx->offset = 0U;
	ctx->counter = 0U;
	ctx->tx_attr = NULL;
	acs_seg_tx_detach_buf(ctx);
	memset(&ctx->ind_params, 0, sizeof(ctx->ind_params));
	acs_seg_tx_complete(ctx, conn, attr, -EIO);
}

static void acs_seg_tx_work_handler(struct k_work *work)
{
	struct acs_seg_tx_ctx *ctx = CONTAINER_OF(work, struct acs_seg_tx_ctx, tx_work);
	struct bt_conn *conn;
	const struct bt_gatt_attr *attr;
	acs_seg_tx_on_complete_t cb;
	void *user_data;
	uint16_t seg_payload;
	uint16_t buf_len;
	uint16_t remaining;
	uint16_t chunk;
	uint16_t mtu;
	bool is_first;
	bool is_last;
	int err;

	__ASSERT_NO_MSG(ctx->tx_conn != NULL);
	__ASSERT_NO_MSG(ctx->buf != NULL);
	__ASSERT_NO_MSG(ctx->tx_attr != NULL);
	__ASSERT_NO_MSG(!ctx->tx_in_flight);

	buf_len = ctx->buf->len;
	remaining = buf_len - ctx->offset;
	__ASSERT_NO_MSG(remaining > 0U);

	mtu = bt_gatt_get_mtu(ctx->tx_conn);
	seg_payload = ACS_SEG_PAYLOAD_SIZE(mtu);
	if (seg_payload == 0U) {
		err = -EINVAL;
		goto cleanup;
	}

	chunk = MIN(seg_payload, remaining);
	chunk = MIN(chunk, (uint16_t)(sizeof(ctx->scratch) - ACS_SEG_HEADER_SIZE));
	is_first = (ctx->offset == 0U);
	is_last = (ctx->offset + chunk >= buf_len);

	ctx->scratch[0] = acs_seg_hdr_build(is_first, is_last, ctx->counter);
	memcpy(&ctx->scratch[ACS_SEG_HEADER_SIZE], ctx->buf->data + ctx->offset, chunk);

	memset(&ctx->ind_params, 0, sizeof(ctx->ind_params));
	ctx->ind_params.attr = ctx->tx_attr;
	ctx->ind_params.func = acs_seg_tx_confirm_cb;
	ctx->ind_params.data = ctx->scratch;
	ctx->ind_params.len = chunk + ACS_SEG_HEADER_SIZE;
	ctx->tx_in_flight = true;

	err = bt_gatt_indicate(ctx->tx_conn, &ctx->ind_params);
	if (err) {
		ctx->tx_in_flight = false;
		goto cleanup;
	}

	ctx->offset += chunk;
	ctx->counter = (ctx->counter + 1U) % ACS_SEG_COUNTER_MAX;
	return;

cleanup:
	conn = ctx->tx_conn;
	attr = ctx->tx_attr;
	cb = ctx->tx_on_complete;
	user_data = ctx->tx_on_complete_data;

	ctx->offset = 0U;
	ctx->counter = 0U;
	ctx->tx_attr = NULL;
	ctx->tx_on_complete = NULL;
	ctx->tx_on_complete_data = NULL;
	ctx->tx_conn = NULL;
	acs_seg_tx_detach_buf(ctx);
	memset(&ctx->ind_params, 0, sizeof(ctx->ind_params));

	if (cb) {
		cb(conn, attr, err, user_data);
	}

	if (conn) {
		bt_conn_unref(conn);
	}
}

int acs_seg_notify(struct bt_conn *conn, const struct bt_gatt_attr *attr, const uint8_t *data,
		   uint16_t len)
{
	uint8_t pdu[ACS_SEG_HEADER_SIZE + CONFIG_BT_ACS_MAX_SEGMENT_SIZE];
	uint16_t seg_payload;
	uint16_t offset;
	uint8_t counter;
	int err;

	if (!conn || !attr || !data || len == 0U) {
		return -EINVAL;
	}

	seg_payload = MIN(ACS_SEG_PAYLOAD_SIZE(bt_gatt_get_mtu(conn)),
			  (uint16_t)CONFIG_BT_ACS_MAX_SEGMENT_SIZE);
	if (seg_payload == 0U) {
		return -ENOMEM;
	}

	offset = 0U;
	counter = 0U;

	while (offset < len) {
		uint16_t chunk = MIN(seg_payload, (uint16_t)(len - offset));
		bool is_first = (offset == 0U);
		bool is_last = (offset + chunk >= len);

		pdu[0] = acs_seg_hdr_build(is_first, is_last, counter);
		memcpy(&pdu[ACS_SEG_HEADER_SIZE], data + offset, chunk);

		err = bt_gatt_notify(conn, attr, pdu, chunk + ACS_SEG_HEADER_SIZE);
		if (err) {
			return err;
		}

		offset += chunk;
		counter = (counter + 1U) % ACS_SEG_COUNTER_MAX;
	}

	return 0;
}

void acs_seg_rx_init(struct acs_seg_rx_ctx *ctx)
{
	__ASSERT_NO_MSG(ctx != NULL);

	memset(ctx, 0, sizeof(*ctx));
	ctx->rx_deadline = sys_timepoint_calc(K_NO_WAIT);
}

void acs_seg_rx_begin(struct acs_seg_rx_ctx *ctx, struct net_buf *buf)
{
	__ASSERT_NO_MSG(ctx != NULL);
	__ASSERT_NO_MSG(buf != NULL);

	ctx->buf = buf;
}

void acs_seg_rx_reset(struct acs_seg_rx_ctx *ctx)
{
	__ASSERT_NO_MSG(ctx != NULL);

	ctx->rx_in_progress = false;
	ctx->rx_counter = 0U;
	ctx->rx_deadline = sys_timepoint_calc(K_NO_WAIT);

	if (ctx->buf) {
		net_buf_unref(ctx->buf);
		ctx->buf = NULL;
	}
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
	__ASSERT_NO_MSG(ctx->buf->size > 0U);

	if (len < ACS_SEG_HEADER_SIZE) {
		return ACS_SEG_RX_ERR_LEN;
	}

	seg_hdr = data[0];
	is_first = (seg_hdr & ACS_SEG_FIRST_MASK) != 0U;
	is_last = (seg_hdr & ACS_SEG_LAST_MASK) != 0U;
	counter = FIELD_GET(ACS_SEG_COUNTER_MASK, seg_hdr);
	payload = &data[ACS_SEG_HEADER_SIZE];
	payload_len = len - ACS_SEG_HEADER_SIZE;

	LOG_DBG("first=%u last=%u counter=%u segment_len=%u",
		is_first ? 1U : 0U,
		is_last ? 1U : 0U,
		counter,
		payload_len);
	LOG_HEXDUMP_DBG(data, len, "seg_rx raw");

	if (is_first) {
		ctx->rx_in_progress = false;
		net_buf_reset(ctx->buf);

		if (payload_len > net_buf_tailroom(ctx->buf)) {
			LOG_WRN("first segment overflow (%u > %zu)", payload_len,
				net_buf_tailroom(ctx->buf));
			return ACS_SEG_RX_ERR_OVERFLOW;
		}

		net_buf_add_mem(ctx->buf, payload, payload_len);

		if (is_last) {
			return ACS_SEG_RX_COMPLETE;
		}

		ctx->rx_in_progress = true;
		ctx->rx_counter = (counter + 1U) % ACS_SEG_COUNTER_MAX;
		ctx->rx_deadline = sys_timepoint_calc(ACS_SEG_RX_TIMEOUT);
		return ACS_SEG_RX_FRAGMENT;
	}

	if (!ctx->rx_in_progress) {
		LOG_WRN("continuation without preceding first segment");
		return ACS_SEG_RX_ERR_ORPHAN;
	}

	if (sys_timepoint_expired(ctx->rx_deadline)) {
		LOG_WRN("inter-segment timeout expired");
		ctx->rx_in_progress = false;
		net_buf_reset(ctx->buf);
		return ACS_SEG_RX_ERR_TIMEOUT;
	}

	if (counter != ctx->rx_counter) {
		LOG_WRN("counter mismatch (expected %u got %u)", ctx->rx_counter, counter);
		ctx->rx_in_progress = false;
		net_buf_reset(ctx->buf);
		return ACS_SEG_RX_ERR_COUNTER;
	}

	if (payload_len > net_buf_tailroom(ctx->buf)) {
		LOG_WRN("segment overflow (%u > %zu)", payload_len,
			net_buf_tailroom(ctx->buf));
		ctx->rx_in_progress = false;
		net_buf_reset(ctx->buf);
		return ACS_SEG_RX_ERR_OVERFLOW;
	}

	net_buf_add_mem(ctx->buf, payload, payload_len);
	LOG_DBG("assembled_len=%u", (uint16_t)ctx->buf->len);

	if (is_last) {
		ctx->rx_in_progress = false;
		return ACS_SEG_RX_COMPLETE;
	}

	ctx->rx_counter = (counter + 1U) % ACS_SEG_COUNTER_MAX;
	ctx->rx_deadline = sys_timepoint_calc(ACS_SEG_RX_TIMEOUT);
	return ACS_SEG_RX_FRAGMENT;
}

void acs_seg_tx_init(struct acs_seg_tx_ctx *ctx)
{
	__ASSERT_NO_MSG(ctx != NULL);

	memset(ctx, 0, sizeof(*ctx));
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

	ctx->offset = 0U;
	ctx->counter = 0U;
	ctx->tx_attr = NULL;
	ctx->tx_on_complete = NULL;
	ctx->tx_on_complete_data = NULL;
	acs_seg_tx_detach_buf(ctx);
	memset(&ctx->ind_params, 0, sizeof(ctx->ind_params));
}

int acs_seg_tx_send(struct acs_seg_tx_ctx *ctx, struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, struct net_buf *buf,
		    acs_seg_tx_on_complete_t tx_on_complete, void *user_data)
{
	__ASSERT_NO_MSG(ctx != NULL);
	__ASSERT_NO_MSG(conn != NULL);
	__ASSERT_NO_MSG(attr != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(buf->len > 0U);

	if (ctx->tx_in_flight || k_work_is_pending(&ctx->tx_work)) {
		LOG_WRN("seg_tx_send busy: in_flight=%u pending=%u tx_conn=%p offset=%zu len=%u",
			ctx->tx_in_flight ? 1U : 0U, k_work_is_pending(&ctx->tx_work) ? 1U : 0U,
			(void *)ctx->tx_conn, ctx->offset, buf->len);
		return -EBUSY;
	}

	ctx->buf = buf;
	ctx->offset = 0U;
	ctx->counter = 0U;
	ctx->tx_attr = attr;
	ctx->tx_on_complete = tx_on_complete;
	ctx->tx_on_complete_data = user_data;
	ctx->tx_conn = bt_conn_ref(conn);

	k_work_submit(&ctx->tx_work);
	return 0;
}
