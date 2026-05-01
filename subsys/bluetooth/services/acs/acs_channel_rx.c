/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/net_buf.h>

#include "acs_channel_rx.h"
#include "acs_internal.h"
#include "acs_seg.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static inline bool acs_channel_rx_is_first(const uint8_t *data)
{
	return (data[0] & ACS_SEG_FIRST_MASK) != 0;
}

enum acs_seg_rx_result acs_channel_rx_feed(struct acs_seg_rx_ctx *rx_ctx, const uint8_t *data,
					   uint16_t len)
{
	enum acs_seg_rx_result res;

	if (len < 1) {
		return ACS_SEG_RX_ERR_LEN;
	}

	if (acs_channel_rx_is_first(data) && !rx_ctx->buf) {
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

	if (res != ACS_SEG_RX_COMPLETE && res != ACS_SEG_RX_FRAGMENT) {
		acs_seg_rx_reset(rx_ctx);
	}

	return res;
}
