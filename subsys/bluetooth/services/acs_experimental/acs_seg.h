/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_SEG_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_SEG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/clock.h>

#include "acs_wire_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Effective payload carried in one ATT PDU after ATT + ACS segmentation headers. */
#define ACS_SEG_PAYLOAD_SIZE(mtu)                                                                  \
	((mtu) > (ACS_SEG_ATT_HDR_SIZE + ACS_SEG_HEADER_SIZE)                                      \
		 ? ((mtu) - ACS_SEG_ATT_HDR_SIZE - ACS_SEG_HEADER_SIZE)                            \
		 : 0U)

/* Spec inter-segment timeout for a multi-segment write. */
#define ACS_SEG_RX_TIMEOUT K_SECONDS(30)

/**
 * @brief Result of processing one segmented RX fragment.
 */
enum acs_seg_rx_result {
	ACS_SEG_RX_COMPLETE = 0,
	ACS_SEG_RX_FRAGMENT = 1,
	/*
	 * Keep segmentation-private failures in their own range so they do not
	 * collide with generic -errno values returned by higher layers.
	 */
	ACS_SEG_RX_ERR_COUNTER = -1001,
	ACS_SEG_RX_ERR_OVERFLOW = -1002,
	ACS_SEG_RX_ERR_ORPHAN = -1003,
	ACS_SEG_RX_ERR_LEN = -1004,
	ACS_SEG_RX_ERR_TIMEOUT = -1005,
};

/**
 * @brief Completion callback for segmented indication TX.
 *
 * Called when the full logical ACS message has either been confirmed by the
 * client or aborted with an error.
 */
typedef void (*acs_seg_tx_on_complete_t)(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					 int err, void *user_data);

/**
 * @brief Reassembly context for one ACS RX channel.
 */
struct acs_seg_rx_ctx {
	struct net_buf *buf;
	k_timepoint_t rx_deadline;
	uint8_t rx_counter;
	bool rx_in_progress;
};

/**
 * @brief Minimal TX bookkeeping for an ACS outbound channel.
 *
 * The current scaffold does not transmit segmented output yet, but runtime and
 * outbound-channel code already need a concrete storage type.
 */
struct acs_seg_tx_ctx {
	struct bt_gatt_indicate_params ind_params;
	struct k_work tx_work;
	acs_seg_tx_on_complete_t tx_on_complete;
	void *tx_on_complete_data;
	struct bt_conn *tx_conn;
	const struct bt_gatt_attr *tx_attr;
	struct net_buf *buf;
	size_t offset;
	uint8_t counter;
	bool tx_in_flight;
	uint8_t scratch[ACS_SEG_HEADER_SIZE + CONFIG_BT_ACS_SEG_TX_SCRATCH_PAYLOAD];
};

/**
 * @brief Send a segmented notification using ACS wire framing.
 *
 * The payload must already be in final on-the-wire order for the selected
 * channel.
 */
int acs_seg_notify(struct bt_conn *conn, const struct bt_gatt_attr *attr, const uint8_t *data,
		   uint16_t len);

/** Initialize an RX reassembly context. */
void acs_seg_rx_init(struct acs_seg_rx_ctx *ctx);

/** Begin reassembly using @p buf as the fragment accumulation buffer. */
void acs_seg_rx_begin(struct acs_seg_rx_ctx *ctx, struct net_buf *buf);

/** Reset an RX reassembly context and release any owned buffer. */
void acs_seg_rx_reset(struct acs_seg_rx_ctx *ctx);

/**
 * @brief Consume one raw ACS fragment.
 *
 * The first byte is treated as the segmentation header and the remainder is
 * appended to the reassembly buffer.
 */
enum acs_seg_rx_result acs_seg_rx_process(struct acs_seg_rx_ctx *ctx, const uint8_t *buf,
					  uint16_t len);

/** Initialize a TX context. */
void acs_seg_tx_init(struct acs_seg_tx_ctx *ctx);

/** Reset a TX context and release any owned buffer. */
void acs_seg_tx_reset(struct acs_seg_tx_ctx *ctx);

/**
 * @brief Send one logical ACS indication, segmented as needed.
 *
 * The TX context borrows @p buf for the duration of the transfer but does not
 * take ownership of it.
 */
int acs_seg_tx_send(struct acs_seg_tx_ctx *ctx, struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, struct net_buf *buf,
		    acs_seg_tx_on_complete_t tx_on_complete, void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_SEG_H_ */
