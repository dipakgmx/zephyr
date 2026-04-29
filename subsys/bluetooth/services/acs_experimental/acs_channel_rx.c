/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>

#include "acs_internal.h"
#include "acs_wire_constants.h"

NET_BUF_POOL_FIXED_DEFINE(acs_channel_buf_pool,
			  MAX(8, CONFIG_BT_ACS_MAX_CONCURRENT_CONN * 4),
			  ACS_BUF_SIZE, 0, NULL);

/** See @ref acs_channel_buf_alloc. */
struct net_buf *acs_channel_buf_alloc(void)
{
	return net_buf_alloc(&acs_channel_buf_pool, K_NO_WAIT);
}

/** See @ref acs_channel_buf_free. */
void acs_channel_buf_free(struct net_buf *buf)
{
	if (buf) {
		net_buf_unref(buf);
	}
}

/** See @ref acs_cp_channel_frame_from_write. */
int acs_cp_channel_frame_from_write(struct bt_conn *conn, const void *buf, uint16_t len,
				    struct acs_frame *frame)
{
	if (!conn || !buf || !frame || len == 0U) {
		return -EINVAL;
	}

	memset(frame, 0, sizeof(*frame));
	frame->conn = conn;
	frame->payload = buf;
	frame->payload_len = len;
	frame->source_channel = ACS_SOURCE_CP_CHANNEL;
	frame->encrypted = false;
	frame->resource_handle = 0U;
	frame->isc_id = 0U;
	frame->backing_buf = NULL;

	return 0;
}

/** See @ref acs_data_in_channel_frame_from_write. */
int acs_data_in_channel_frame_from_write(struct bt_conn *conn, const void *buf, uint16_t len,
					 struct acs_frame *frame)
{
	const uint8_t *bytes = buf;

	if (!conn || !buf || !frame || len < ACS_DATA_IN_HDR_SIZE) {
		return -EINVAL;
	}

	memset(frame, 0, sizeof(*frame));
	frame->conn = conn;
	frame->payload = buf;
	frame->payload_len = len;
	frame->source_channel = ACS_SOURCE_DATA_IN_CHANNEL;
	frame->encrypted = true;
	frame->isc_id = sys_get_le16(bytes);
	frame->resource_handle = 0U;
	frame->backing_buf = NULL;

	return 0;
}

/**
 * @brief Shared segmented-message reassembly helper for CP and Data In.
 *
 * This file owns the receive-side ACS channel transport work:
 * - allocate the first fragment buffer
 * - feed bytes into the ACS segment parser
 * - normalize a completed message into an @ref acs_frame
 */
static int acs_channel_reassemble(struct bt_conn *conn, struct acs_seg_rx_ctx *rx,
				  enum acs_source_channel source_channel, const void *buf,
				  uint16_t len, struct acs_frame *frame)
{
	enum acs_seg_rx_result res;
	const uint8_t *bytes = buf;
	int err;

	if (!conn || !rx || !buf || !frame || len == 0U) {
		return -EINVAL;
	}

	if ((bytes[0] & ACS_SEG_FIRST_MASK) != 0U && rx->buf == NULL) {
		struct net_buf *rx_buf = acs_channel_buf_alloc();

		if (!rx_buf) {
			return -ENOMEM;
		}
		acs_seg_rx_begin(rx, rx_buf);
	}

	if (!rx->buf) {
		return -EINVAL;
	}

	res = acs_seg_rx_process(rx, buf, len);
	if (res == ACS_SEG_RX_PENDING) {
		return ACS_SEG_RX_PENDING;
	}

	if (res != ACS_SEG_RX_COMPLETE) {
		acs_seg_rx_reset(rx);
		return res;
	}

	if (source_channel == ACS_SOURCE_CP_CHANNEL) {
		err = acs_cp_channel_frame_from_write(conn, rx->buf->data, rx->buf->len, frame);
	} else {
		err = acs_data_in_channel_frame_from_write(conn, rx->buf->data, rx->buf->len,
							       frame);
	}

	if (err) {
		acs_seg_rx_reset(rx);
		return err;
	}

	frame->backing_buf = rx->buf;
	rx->buf = NULL;
	rx->rx_in_progress = false;
	return 0;
}

int acs_cp_channel_reassemble(struct acs_conn_ctx *conn_ctx, const void *buf, uint16_t len,
			      struct acs_frame *frame)
{
	int err;

	err = acs_channel_reassemble(conn_ctx ? conn_ctx->conn : NULL,
				     conn_ctx ? &conn_ctx->cp_rx : NULL,
				     ACS_SOURCE_CP_CHANNEL, buf, len, frame);
	return err;
}

/** See @ref acs_data_in_channel_reassemble. */
int acs_data_in_channel_reassemble(struct acs_conn_ctx *conn_ctx, const void *buf, uint16_t len,
				   struct acs_frame *frame)
{
	int err;

	err = acs_channel_reassemble(conn_ctx ? conn_ctx->conn : NULL,
				     conn_ctx ? &conn_ctx->data_rx : NULL,
				     ACS_SOURCE_DATA_IN_CHANNEL, buf, len, frame);
	return err;
}
