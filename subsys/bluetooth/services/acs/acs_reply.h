/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_REPLY_H_
#define ACS_REPLY_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/net_buf.h>

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize reply state for a newly zeroed connection slot. */
void acs_reply_init_conn(struct bt_acs_conn *conn);

/* Claim a general reply slot, excluding the slot reserved for Abort. */
struct acs_reply *acs_reply_alloc(struct bt_acs_conn *conn);

/* Move rx->buf to reply->request without copying it. */
void acs_reply_take_request_buf(struct acs_reply *reply, struct acs_seg_rx_ctx *rx);

/* Free a reply and release its Control Point lock, if held. */
void acs_reply_free(struct acs_reply *reply);

/* Stop and free all replies during disconnect. */
void acs_reply_cancel_all(struct bt_acs_conn *conn);

/* End a procedure when its last response segment is sent (§4.4.3). */
void acs_reply_response_sent(void *user_data);

/* Stop the active procedure and send the Abort Response Code. */
void acs_abort_request(struct bt_acs_conn *conn);

/* Prepare reply->response. Return NULL when no buffer is available. */
struct net_buf *acs_prepare_reply_buf(struct acs_reply *reply);

/* Send reply->response on reply->channel. */
int acs_reply_submit(struct acs_reply *reply);

/* Build the next response. Return false when the reply can be freed. */
bool acs_reply_continue(struct acs_reply *reply);

#ifdef __cplusplus
}
#endif

#endif /* ACS_REPLY_H_ */
