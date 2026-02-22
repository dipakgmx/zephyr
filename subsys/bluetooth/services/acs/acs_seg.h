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

/* ATT opcode and Attribute Handle overhead for notifications and indications. */
#define ACS_SEG_ATT_HDR_SIZE 3

/* Segmentation_Header size (Table 4.3). */
#define ACS_SEG_HDR_SIZE 1

/* Effective segmented payload size within an ATT PDU. */
#define ACS_SEG_PAYLOAD_SIZE(mtu)                                                                  \
	((mtu) > (ACS_SEG_ATT_HDR_SIZE + ACS_SEG_HDR_SIZE)                                         \
		 ? ((mtu) - ACS_SEG_ATT_HDR_SIZE - ACS_SEG_HDR_SIZE)                               \
		 : 0u)

/*
 * Segmentation_Header bit fields (Table 4.3):
 *   bit 0    First Segment
 *   bit 1    Last Segment
 *   bits 2-7 Rolling Segment Counter, 0-63, wraps
 */
#define ACS_SEG_FIRST_SEGMENT_BIT 0
#define ACS_SEG_LAST_SEGMENT_BIT  1
#define ACS_SEG_COUNTER_MASK      GENMASK(7, 2)
/* Modulus of the 6-bit Rolling Segment Counter. */
#define ACS_SEG_COUNTER_MAX       64

/* Timeout between segments of a multi-segment write (30 s, §4.3.2.1 / §4.4.3.21). */
#define ACS_SEG_RX_TIMEOUT K_SECONDS(30)

/* Completion callback called once for a segmented transfer. */
typedef void (*acs_seg_tx_completion_cb_t)(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					   int err, void *user_data);

/* Callback called when the last segment is accepted for sending. */
typedef void (*acs_seg_tx_sent_cb_t)(void *user_data);

/* Segment-reassembly result. */
enum acs_seg_rx_result {
	ACS_SEG_RX_COMPLETE = 0,      /* Payload fully reassembled in ctx->buf */
	ACS_SEG_RX_PENDING = 1,       /* Mid-stream segment buffered; not yet complete */
	ACS_SEG_RX_ERR_COUNTER = -1,  /* Invalid rolling counter */
	ACS_SEG_RX_ERR_OVERFLOW = -2, /* Segment too large for buffer */
	ACS_SEG_RX_ERR_ORPHAN = -3,   /* Last/continuation without preceding First */
	ACS_SEG_RX_ERR_LEN = -4,      /* PDU too short (no segmentation header) */
	ACS_SEG_RX_ERR_TIMEOUT = -5,  /* Inter-segment timeout expired (§3.6.2) */
};

/* Inbound reassembly state for one Control Point or Data In channel. */
struct acs_seg_rx_ctx {
	k_timepoint_t rx_deadline; /* Inter-segment deadline (§3.6.2) */
	struct net_buf *buf;       /* Borrowed buffer */
	uint8_t rx_counter;        /* Expected rolling counter value */
	bool rx_in_progress;       /* Multi-segment transfer ongoing */
};

/* Outbound segmentation state for one channel. */
struct acs_seg_tx_ctx {
	union {
		struct bt_gatt_indicate_params ind;
		struct bt_gatt_notify_params ntf;
	} params;
	struct k_work tx_work; /* Work item for chaining TX segments */
	struct net_buf *buf;   /* Borrowed for the transfer, never owned by the engine */
	const struct bt_gatt_attr *tx_attr;       /* Characteristic value attribute */
	struct bt_conn *tx_conn;                  /* Connection ref held during TX */
	acs_seg_tx_completion_cb_t completion_cb; /* Completion callback */
	void *completion_cb_data;                 /* Callback data */
	acs_seg_tx_sent_cb_t sent_cb;             /* Fired when the last segment is sent */
	uint16_t tx_offset;                       /* Bytes already sent */
	int completion_err;                       /* Outcome recorded by the stack callback */
	uint8_t tx_counter;                       /* Rolling segment counter */
	bool completion_pending;                  /* Callback recorded a completion for tx_work */
	bool complete_on_wq; /* Run completion on the ACS workqueue vs inline */
	bool tx_in_flight;   /* Segment currently awaiting completion */
	bool tx_sending;     /* Sender is updating transfer state */
	bool is_indicate;    /* Indication when true, notification when false */
};

/* Return true when the last segment is waiting for workqueue completion. */
static inline bool acs_seg_tx_complete_pending(const struct acs_seg_tx_ctx *ctx)
{
	return ctx->completion_pending && ctx->completion_err == 0 && ctx->buf != NULL &&
	       ctx->tx_offset >= ctx->buf->len;
}

/* Initialize an RX context. */
void acs_seg_rx_init(struct acs_seg_rx_ctx *ctx);

/* Reset RX state and release its buffer. */
void acs_seg_rx_reset(struct acs_seg_rx_ctx *ctx);

/* Initialize a TX context. */
void acs_seg_tx_init(struct acs_seg_tx_ctx *ctx, bool indicate, bool complete_on_wq);

/* Set the last-segment callback, or clear it with NULL. */
void acs_seg_tx_set_sent_cb(struct acs_seg_tx_ctx *ctx, acs_seg_tx_sent_cb_t cb);

/* Reset TX state and release its connection and buffer. */
void acs_seg_tx_reset(struct acs_seg_tx_ctx *ctx);

/* Return true while a transfer is active. */
static inline bool acs_seg_tx_busy(const struct acs_seg_tx_ctx *ctx)
{
	return ctx->buf != NULL;
}

/* Return the active transfer's callback data, or NULL when idle. */
static inline void *acs_seg_tx_user_data(const struct acs_seg_tx_ctx *ctx)
{
	return ctx->completion_cb_data;
}

/* Start a segmented transfer. The TX context borrows buf until completion. */
int acs_seg_tx_send(struct acs_seg_tx_ctx *ctx, struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, struct net_buf *buf,
		    acs_seg_tx_completion_cb_t completion_cb, void *user_data);

/* Feed one ATT write PDU into rx_ctx and return its reassembly state. */
enum acs_seg_rx_result acs_channel_rx_feed(struct acs_seg_rx_ctx *rx_ctx, const uint8_t *data,
					   uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* ACS_SEG_H_ */
