/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_SEG_H_
#define ACS_SEG_H_

#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/clock.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ATT opcode(1) + handle(2) for notify; opcode(1) + handle(2) for indicate */
#define ACS_SEG_ATT_HDR_SIZE 3

/* Segmentation header occupies the first byte of the characteristic value */
#define ACS_SEG_HDR_SIZE 1

/* Effective payload per ATT PDU = MTU - ATT_HDR - SEG_HDR */
#define ACS_SEG_PAYLOAD_SIZE(mtu)                                                                  \
	((mtu) > (ACS_SEG_ATT_HDR_SIZE + ACS_SEG_HDR_SIZE)                                         \
		 ? ((mtu) - ACS_SEG_ATT_HDR_SIZE - ACS_SEG_HDR_SIZE)                               \
		 : 0u)

/**
 * @brief Segmentation header bit-field masks (Table 4.3)
 *
 *   Bit 0: First Segment
 *   Bit 1: Last Segment
 *   Bits 2-7: Rolling Segment Counter (0-63, wraps)
 */
#define ACS_SEG_FIRST_SEGMENT_BIT 0
#define ACS_SEG_LAST_SEGMENT_BIT  1
#define ACS_SEG_COUNTER_MASK      GENMASK(7, 2)
/* (1U << 6): counter occupies 6 bits (GENMASK(7,2)); wraps at 64. */
#define ACS_SEG_COUNTER_MAX       64

/** Single-segment (complete) PDU header: First=1, Last=1, Counter=0 */
#define ACS_SEG_SINGLE_PDU (BIT(ACS_SEG_FIRST_SEGMENT_BIT) | BIT(ACS_SEG_LAST_SEGMENT_BIT))

/** @brief Timeout between segments of a multi-segment write (30 s, §4.3.2.1 / §4.4.3.21). */
#define ACS_SEG_RX_TIMEOUT K_SECONDS(30)

/** @brief Size of the small PDU scratch buffer embedded in acs_seg_tx_ctx.
 *  Payload portion is driven by the hidden CONFIG_BT_ACS_SEG_TX_SCRATCH_PAYLOAD
 *  knob so RAM-tight targets can shrink every per-connection TX context. */
#define ACS_SEG_CTX_PDU_SIZE (ACS_SEG_HDR_SIZE + CONFIG_BT_ACS_SEG_TX_SCRATCH_PAYLOAD)

/**
 * @brief Completion callback invoked when a full segmented notification or
 *        indication transfer completes (or after the first failure).
 *
 * Called once per logical ACS payload, not once per segment.
 *
 * @param conn       Connection the indication was sent on.
 * @param attr       GATT attribute that was indicated.
 * @param err        0 on success, negative errno on failure.
 * @param user_data  Caller-owned completion context passed through
 *                   @ref acs_seg_tx_send so higher layers can resume or
 *                   clean up the object associated with this transfer
 *                   (for example a staged CP response buffer or an
 *                   in-flight @ref acs_procedure).
 */
typedef void (*acs_seg_tx_completion_cb_t)(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					   int err, void *user_data);

/**
 * @brief Return codes for acs_seg_rx_process().
 */
enum acs_seg_rx_result {
	ACS_SEG_RX_COMPLETE = 0,      /**< Payload fully reassembled in ctx->buf */
	ACS_SEG_RX_PENDING = 1,       /**< Mid-stream segment buffered; not yet complete */
	ACS_SEG_RX_ERR_COUNTER = -1,  /**< Invalid rolling counter */
	ACS_SEG_RX_ERR_OVERFLOW = -2, /**< Segment too large for buffer */
	ACS_SEG_RX_ERR_ORPHAN = -3,   /**< Last/continuation without preceding First */
	ACS_SEG_RX_ERR_LEN = -4,      /**< PDU too short (no segmentation header) */
	ACS_SEG_RX_ERR_TIMEOUT = -5,  /**< Inter-segment timeout expired (§3.6.2) */
};

/**
 * @brief Segmentation context for inbound RX reassembly.
 *
 * Tracks inter-segment state for multi-segment ATT writes on a single
 * channel (CP or Data In).  A pool buffer is borrowed on demand via
 * acs_seg_rx_begin() before the first segment arrives.
 *
 * Timeout enforcement (§3.6.2) uses a deadline-based check: the deadline
 * is stamped on each continuation segment and evaluated lazily when the
 * next segment arrives, so no background work item is needed.
 */
struct acs_seg_rx_ctx {
	k_timepoint_t rx_deadline; /**< Inter-segment deadline (§3.6.2) */
	struct net_buf *buf;       /**< Borrowed from pool; set via acs_seg_rx_begin() */
	uint8_t rx_counter;        /**< Expected rolling counter value */
	bool rx_in_progress;       /**< Multi-segment transfer ongoing */
};

/**
 * @brief Segmentation context for outbound TX indications.
 *
 * Manages segmented GATT indications on a single channel (CP or DOI).
 * A pool buffer and completion callback are installed via acs_seg_tx_send();
 * the TX engine chains segments through a work item until all are confirmed.
 */
struct acs_seg_tx_ctx {
	struct bt_gatt_indicate_params ind_params; /**< Passed to bt_gatt_indicate() */
	struct k_work tx_work;                     /**< Work item for chaining TX segments */
	struct net_buf *buf;                       /**< Borrowed from pool; set before TX */
	const struct bt_gatt_attr *tx_attr;        /**< Characteristic value attribute */
	struct bt_conn *tx_conn;                   /**< Connection ref held during TX */
	acs_seg_tx_completion_cb_t completion_cb;  /**< Completion callback */
	void *completion_cb_data;                  /**< Opaque callback user data */
	uint8_t tx_scratch[ACS_SEG_CTX_PDU_SIZE];  /**< PDU scratch: seg header + one chunk */
	uint16_t tx_offset;                        /**< Bytes already indicated */
	uint8_t tx_counter;                        /**< Rolling segment counter */
	bool tx_in_flight;                         /**< Segment currently awaiting confirmation */
};

/**
 * @brief Segmentation context for outbound TX notifications.
 *
 * Manages segmented GATT notifications on a single channel. The TX engine
 * chains segments through a work item and advances on notify completion
 * callbacks until the full logical ACS payload is sent.
 */
struct acs_seg_notify_ctx {
	struct bt_gatt_notify_params notify_params; /**< Passed to bt_gatt_notify_cb() */
	struct k_work tx_work;                      /**< Work item for chaining TX segments */
	struct net_buf *buf;                        /**< Borrowed from pool; set before TX */
	const struct bt_gatt_attr *tx_attr;         /**< Characteristic value attribute */
	struct bt_conn *tx_conn;                    /**< Connection ref held during TX */
	acs_seg_tx_completion_cb_t completion_cb;   /**< Completion callback */
	void *completion_cb_data;                   /**< Opaque callback user data */
	uint8_t tx_scratch[ACS_SEG_CTX_PDU_SIZE];   /**< PDU scratch: seg header + one chunk */
	uint16_t tx_offset;                         /**< Bytes already notified */
	uint8_t tx_counter;                         /**< Rolling segment counter */
	bool tx_in_flight;                          /**< Segment currently awaiting completion */
};

/**
 * @brief Send a segmented notification.
 *
 * @param conn  Connection to notify on.
 * @param attr  GATT attribute to notify.
 * @param data  Full payload (already encrypted / ready to send).
 * @param len   Payload length in bytes.
 *
 * @return 0 on success, or the first bt_gatt_notify() error code.
 */
int acs_seg_notify(struct bt_conn *conn, const struct bt_gatt_attr *attr, const uint8_t *data,
		   uint16_t len);

/**
 * @brief Initialise a TX notification context.
 *
 * Must be called once before first use. Arms the internal work item used to
 * chain multi-segment notifications through @ref bt_gatt_notify_cb.
 *
 * @param ctx  TX notification context to initialise.
 */
void acs_seg_notify_async_init(struct acs_seg_notify_ctx *ctx);

/**
 * @brief Reset and clean up a TX notification context.
 *
 * Cancels any pending work, releases the held connection reference and clears
 * the borrowed buffer/callback state. Safe to call on an already-clean
 * context.
 *
 * @param ctx  TX notification context to reset.
 */
void acs_seg_notify_async_reset(struct acs_seg_notify_ctx *ctx);

/**
 * @brief Arm a segmented notification and transfer buffer ownership to TX.
 *
 * The TX path borrows @p buf until it completes or fails.
 *
 * @param ctx   TX notification context.
 * @param conn  Connection to notify on (ref taken internally).
 * @param attr  GATT attribute to notify.
 * @param buf   Full payload buffer to send.
 * @param completion_cb Completion callback invoked after all chunks complete.
 * @param user_data Opaque callback user data.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_seg_notify_async_send(struct acs_seg_notify_ctx *ctx, struct bt_conn *conn,
			      const struct bt_gatt_attr *attr, struct net_buf *buf,
			      acs_seg_tx_completion_cb_t completion_cb, void *user_data);

/**
 * @brief Initialise an RX reassembly context.
 *
 * Must be called once before first use.  Does not allocate a buffer; the
 * buffer is installed on-demand via acs_seg_rx_begin().
 *
 * @param ctx  RX context to initialise.
 */
void acs_seg_rx_init(struct acs_seg_rx_ctx *ctx);

/**
 * @brief Reset and clean up an RX reassembly context.
 *
 * Clears reassembly state and releases any installed buffer.
 * Safe to call on an already-clean context.
 *
 * @param ctx  RX context to reset.
 */
void acs_seg_rx_reset(struct acs_seg_rx_ctx *ctx);

/**
 * @brief Install a buffer for RX reassembly.
 *
 * The caller remains responsible for the buffer lifetime until it explicitly
 * transfers or releases it.  The context only uses the buffer for reassembly.
 *
 * @param ctx  RX reassembly context.
 * @param buf  Buffer to use for subsequent RX segments.
 */
void acs_seg_rx_begin(struct acs_seg_rx_ctx *ctx, struct net_buf *buf);

/**
 * @brief Process one inbound ATT write PDU and reassemble into the installed buffer.
 *
 * The caller must install a pool buffer with acs_seg_rx_begin() before the
 * first segment.  On ACS_SEG_RX_COMPLETE the reassembled payload is in
 * ctx->buf->data with length ctx->buf->len.
 *
 * @param ctx      RX reassembly context (ctx->buf must be set).
 * @param data     Raw ATT write buffer (segmentation header at [0]).
 * @param len      Length of @p data.
 *
 * @return ACS_SEG_RX_COMPLETE, ACS_SEG_RX_PENDING, or a negative error code.
 */
enum acs_seg_rx_result acs_seg_rx_process(struct acs_seg_rx_ctx *ctx, const uint8_t *data,
					  uint16_t len);

/**
 * @brief Initialise a TX indication context.
 *
 * Must be called once before first use.  Arms the internal work item used
 * to chain multi-segment indications.
 *
 * @param ctx  TX context to initialise.
 */
void acs_seg_tx_init(struct acs_seg_tx_ctx *ctx);

/**
 * @brief Reset and clean up a TX indication context.
 *
 * Cancels any pending work, releases the held connection reference and
 * buffer.  Safe to call on an already-clean context.
 *
 * @param ctx  TX context to reset.
 */
void acs_seg_tx_reset(struct acs_seg_tx_ctx *ctx);

/**
 * @brief Arm a segmented indication and transfer buffer ownership to TX.
 *
 * The TX path owns @p buf until it completes or fails.
 *
 * @param ctx   TX indication context.
 * @param conn  Connection to indicate on (ref taken internally).
 * @param attr  GATT attribute to indicate.
 * @param buf   Full payload buffer to send.
 * @param completion_cb Completion callback invoked after all segments are confirmed.
 * @param user_data Opaque callback user data.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_seg_tx_send(struct acs_seg_tx_ctx *ctx, struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, struct net_buf *buf,
		    acs_seg_tx_completion_cb_t completion_cb, void *user_data);

/**
 * @brief Drive segment reassembly for one ACS RX channel.
 *
 * Allocates a pool buffer on the first segment of a transfer, hands subsequent
 * segments to @ref acs_seg_rx_process, and returns the seg-RX result. On
 * @ref ACS_SEG_RX_COMPLETE the reassembled payload is in @c rx_ctx->buf and the
 * caller is responsible for transferring ownership (typically into a request
 * context) and clearing @c rx_ctx->buf before returning.
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

#endif /* ACS_SEG_H_ */
