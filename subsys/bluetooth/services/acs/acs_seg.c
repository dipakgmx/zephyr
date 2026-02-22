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

/* Build a Segmentation_Header value (Table 4.3). */
static inline uint8_t seg_hdr_build(bool first, bool last, uint8_t counter)
{
	return (first ? BIT(ACS_SEG_FIRST_SEGMENT_BIT) : 0U) |
	       (last ? BIT(ACS_SEG_LAST_SEGMENT_BIT) : 0U) |
	       FIELD_PREP(ACS_SEG_COUNTER_MASK, counter);
}

static void acs_seg_tx_work_handler(struct k_work *work);

void acs_seg_rx_init(struct acs_seg_rx_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
}

void acs_seg_rx_reset(struct acs_seg_rx_ctx *ctx)
{
	ctx->rx_in_progress = false;
	ctx->rx_counter = 0;

	if (ctx->buf != NULL) {
		net_buf_unref(ctx->buf);
		ctx->buf = NULL;
	}
}

static void acs_seg_rx_begin(struct acs_seg_rx_ctx *ctx, struct net_buf *buf)
{
	ctx->buf = buf;
}

static enum acs_seg_rx_result acs_seg_rx_process(struct acs_seg_rx_ctx *ctx, const uint8_t *data,
						 uint16_t len)
{
	const uint8_t *payload;
	uint16_t payload_len;
	uint8_t seg_hdr;
	uint8_t counter;
	bool is_first;
	bool is_last;

	seg_hdr = data[0];
	is_first = IS_BIT_SET(seg_hdr, ACS_SEG_FIRST_SEGMENT_BIT);
	is_last = IS_BIT_SET(seg_hdr, ACS_SEG_LAST_SEGMENT_BIT);
	counter = FIELD_GET(ACS_SEG_COUNTER_MASK, seg_hdr);
	payload = &data[1];
	payload_len = len - 1;

	LOG_DBG("first=%u last=%u counter=%u payload_len=%u in_progress=%u buf_len=%u", is_first,
		is_last, counter, payload_len, ctx->rx_in_progress, ctx->buf->len);
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

	/* A continuation received after the inter-segment deadline is invalid (§3.6.2). */
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
	ctx->buf = NULL;
	if (cb != NULL) {
		cb(conn, attr, err, ud);
	}
	if (conn != NULL) {
		bt_conn_unref(conn);
	}
}

static void seg_tx_segment_complete(struct acs_seg_tx_ctx *ctx, int err)
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

/* Complete DON and DOI on the ACS workqueue; complete plain CP inline. */
static void seg_tx_dispatch_completion(struct acs_seg_tx_ctx *ctx, int err)
{
	if (ctx->complete_on_wq) {
		ctx->completion_err = err;
		ctx->completion_pending = true;
		k_work_submit_to_queue(acs_get_wq(), &ctx->tx_work);
		return;
	}

	/* Let the sender finish updating the transfer before handling completion. */
	if (ctx->tx_sending) {
		ctx->completion_err = err;
		ctx->completion_pending = true;
		return;
	}

	seg_tx_segment_complete(ctx, err);
}

static void seg_tx_indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params,
			       uint8_t err)
{
	struct acs_seg_tx_ctx *ctx = CONTAINER_OF(params, struct acs_seg_tx_ctx, params.ind);

	ARG_UNUSED(conn);
	seg_tx_dispatch_completion(ctx, err ? -EIO : 0);
}

static void seg_tx_notify_cb(struct bt_conn *conn, void *user_data)
{
	struct acs_seg_tx_ctx *ctx = user_data;

	ARG_UNUSED(conn);
	seg_tx_dispatch_completion(ctx, 0);
}

static void acs_seg_tx_work_handler(struct k_work *work)
{
	struct acs_seg_tx_ctx *ctx = CONTAINER_OF(work, struct acs_seg_tx_ctx, tx_work);
	uint16_t seg_payload;
	uint16_t buf_len;
	uint16_t remaining;
	uint16_t chunk;
	uint16_t mtu;
	uint8_t *pdu;
	bool is_first;
	bool is_last;
	int err;

	if (ctx->completion_pending) {
		ctx->completion_pending = false;
		seg_tx_segment_complete(ctx, ctx->completion_err);
		return;
	}

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

	is_first = (ctx->tx_offset == 0);
	is_last = (ctx->tx_offset + chunk >= buf_len);

	/* Reuse the byte before each chunk for its segment header. */
	pdu = ctx->buf->data + ctx->tx_offset - ACS_SEG_HDR_SIZE;
	pdu[0] = seg_hdr_build(is_first, is_last, ctx->tx_counter);

	LOG_DBG("send first=%u last=%u counter=%u chunk=%u offset=%u/%u", is_first, is_last,
		ctx->tx_counter, chunk, ctx->tx_offset, buf_len);
	LOG_HEXDUMP_DBG(pdu, chunk + ACS_SEG_HDR_SIZE, "seg_tx pdu");

	/* Update tx_offset before a completion can run on another thread. */
	ctx->tx_in_flight = true;
	ctx->tx_sending = true;
	ctx->tx_offset += chunk;
	ctx->tx_counter = (ctx->tx_counter + 1) % ACS_SEG_COUNTER_MAX;

	if (ctx->is_indicate) {
		ctx->params.ind.data = pdu;
		ctx->params.ind.len = chunk + ACS_SEG_HDR_SIZE;

		err = bt_gatt_indicate(ctx->tx_conn, &ctx->params.ind);
	} else {
		ctx->params.ntf.data = pdu;
		ctx->params.ntf.len = chunk + ACS_SEG_HDR_SIZE;

		err = bt_gatt_notify_cb(ctx->tx_conn, &ctx->params.ntf);
	}

	if (!err) {
		if (is_last && ctx->sent_cb != NULL) {
			ctx->sent_cb(ctx->completion_cb_data);
		}
		ctx->tx_sending = false;
		if (ctx->completion_pending && !ctx->complete_on_wq) {
			ctx->completion_pending = false;
			seg_tx_segment_complete(ctx, ctx->completion_err);
		}
		return;
	}

	ctx->tx_sending = false;
	ctx->tx_in_flight = false;
	if (ctx->is_indicate && err == -EINVAL) {
		LOG_DBG("indicate skipped (CCC not enabled)");
	} else if (ctx->is_indicate) {
		LOG_ERR("bt_gatt_indicate failed: %d", err);
	} else {
		LOG_ERR("bt_gatt_notify_cb failed: %d", err);
	}

cleanup:
	seg_tx_cleanup(ctx, err);
}

void acs_seg_tx_init(struct acs_seg_tx_ctx *ctx, bool indicate, bool complete_on_wq)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->is_indicate = indicate;
	ctx->complete_on_wq = complete_on_wq;
	k_work_init(&ctx->tx_work, acs_seg_tx_work_handler);
}

void acs_seg_tx_set_sent_cb(struct acs_seg_tx_ctx *ctx, acs_seg_tx_sent_cb_t cb)
{
	ctx->sent_cb = cb;
}

void acs_seg_tx_reset(struct acs_seg_tx_ctx *ctx)
{
	struct k_work_sync sync;

	ctx->tx_in_flight = false;
	ctx->tx_sending = false;
	ctx->completion_pending = false;

	k_work_cancel_sync(&ctx->tx_work, &sync);
	if (ctx->tx_conn != NULL) {
		bt_conn_unref(ctx->tx_conn);
		ctx->tx_conn = NULL;
	}
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = NULL;

	ctx->buf = NULL;

	ctx->completion_cb = NULL;
	ctx->completion_cb_data = NULL;
}

int acs_seg_tx_send(struct acs_seg_tx_ctx *ctx, struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, struct net_buf *buf,
		    acs_seg_tx_completion_cb_t completion_cb, void *user_data)
{
	__ASSERT_NO_MSG(conn != NULL);
	__ASSERT_NO_MSG(attr != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(buf->len > 0);

	/* buf remains set for the whole transfer, including callback continuations. */
	if (ctx->buf != NULL) {
		LOG_WRN("TX already in progress");
		return -EBUSY;
	}

	ctx->buf = buf;
	ctx->tx_offset = 0;
	ctx->tx_counter = 0;
	ctx->tx_attr = attr;
	ctx->completion_cb = completion_cb;
	ctx->completion_cb_data = user_data;

	if (ctx->is_indicate) {
		memset(&ctx->params.ind, 0, sizeof(ctx->params.ind));
		ctx->params.ind.attr = attr;
		ctx->params.ind.func = seg_tx_indicate_cb;
	} else {
		memset(&ctx->params.ntf, 0, sizeof(ctx->params.ntf));
		ctx->params.ntf.attr = attr;
		ctx->params.ntf.func = seg_tx_notify_cb;
		ctx->params.ntf.user_data = ctx;
	}

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

	if (IS_BIT_SET(data[0], ACS_SEG_FIRST_SEGMENT_BIT) && rx_ctx->buf == NULL) {
		struct net_buf *rx_buf = acs_buf_alloc(K_NO_WAIT);

		if (rx_buf == NULL) {
			LOG_ERR("Channel RX: buffer pool exhausted");
			return ACS_SEG_RX_ERR_OVERFLOW;
		}
		acs_seg_rx_begin(rx_ctx, rx_buf);
	}

	if (rx_ctx->buf == NULL) {
		LOG_ERR("Channel RX: continuation segment without prior first segment");
		return ACS_SEG_RX_ERR_ORPHAN;
	}

	res = acs_seg_rx_process(rx_ctx, data, len);

	if (res != ACS_SEG_RX_COMPLETE && res != ACS_SEG_RX_PENDING) {
		acs_seg_rx_reset(rx_ctx);
	}

	return res;
}
