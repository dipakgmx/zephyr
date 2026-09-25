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

#include "acs_seg.h"
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

/* End a procedure when the last segment of its last message is sent (§4.4.3). */
void acs_reply_response_sent(void *user_data);

/* Stop the active procedure and send the Abort Response Code. */
void acs_abort_request(struct bt_acs_conn *conn);

/*
 * Largest response operand one message carries on any channel. A Data Out
 * message gives up ISC_ID and Nonce_Var, the segment header, the
 * Protected_Resource_Handle, the response opcode and the tag.
 */
#define ACS_MESSAGE_MAX_OPERAND                                                                    \
	(ACS_BUF_SIZE - ACS_CRYPTO_HEADROOM - ACS_SEG_HDR_SIZE - sizeof(uint16_t) - 1U -          \
	 ACS_MAX_AUTH_TAG_SIZE)

/*
 * Append a message to reply and return its buffer, prepared for the reply's
 * channel. Return NULL when no buffer is available.
 */
struct net_buf *acs_reply_add_message(struct acs_reply *reply);

/* Discard every message reply holds. */
void acs_reply_drop_messages(struct acs_reply *reply);

/* Send the reply's messages in order on reply->channel. */
int acs_reply_submit(struct acs_reply *reply);

/* Move on to the message after the one just sent. Return NULL after the last one. */
struct net_buf *acs_reply_next_message(struct acs_reply *reply);

/* Carry out what follows a fully delivered reply: removing the key it invalidated. */
void acs_reply_delivered(struct acs_reply *reply);

#ifdef __cplusplus
}
#endif

#endif /* ACS_REPLY_H_ */
