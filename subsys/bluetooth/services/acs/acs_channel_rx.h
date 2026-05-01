/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_CHANNEL_RX_H_
#define ACS_CHANNEL_RX_H_

#include <stdint.h>

#include "acs_seg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Drive segment reassembly for one ACS RX channel.
 *
 * Allocates a pool buffer on the first segment of a transfer, hands subsequent
 * segments to @ref acs_seg_rx_process, and returns the seg-RX result. On
 * @ref ACS_SEG_RX_COMPLETE the reassembled payload is in @c rx_ctx->buf and the
 * caller is responsible for transferring ownership (typically into an
 * @ref acs_frame::backing_buf) and clearing @c rx_ctx->buf before returning.
 *
 * On any error result the helper resets @c rx_ctx; the caller does not need to.
 *
 * @param rx_ctx  Per-channel reassembly context.
 * @param data    Raw ATT write PDU (segmentation header at byte 0).
 * @param len     Length of @p data.
 *
 * @return @ref acs_seg_rx_result. ACS_SEG_RX_ERR_OVERFLOW is also returned when
 *         the pool is exhausted on first segment.
 */
enum acs_seg_rx_result acs_channel_rx_feed(struct acs_seg_rx_ctx *rx_ctx, const uint8_t *data,
					   uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* ACS_CHANNEL_RX_H_ */
