/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_KEY_EXCHANGE_H_
#define BT_GATT_ACS_KEY_EXCHANGE_H_

#include <zephyr/types.h>
#include <zephyr/net_buf.h>
#include "acs_types.h"

/* Allocate a key-exchange context, or return NULL if none is available. */
struct bt_acs_kex_ctx *acs_key_exchange_alloc(struct bt_acs_conn *acs_conn);

/* Set the AuthValue to an OOB number: big-endian in the last octets, zeros before. */
void acs_key_exchange_set_auth_value(struct bt_acs_kex_ctx *kex, uint32_t number);

/* Append the Key Exchange Response (§4.4.3.10) after the last step's response. */
int acs_key_exchange_add_result(struct acs_reply *reply);

/*
 * Complete the key exchange once its Key Exchange Response is sent: free its
 * context, set Security Established, store a new ECDH key and schedule the
 * Status indication. Does nothing after any other Control Point response.
 * Call only while a key exchange is in progress.
 */
void acs_key_exchange_complete(struct bt_acs_conn *acs_conn);

/* Start an ECDH key exchange: generate the AC Server's ephemeral key pair. */
int acs_key_exchange_ecdh_start(struct bt_acs_conn *acs_conn);

/* Abort the in-flight key exchange and destroy its partial key state. */
void acs_key_exchange_abort(struct bt_acs_conn *acs_conn);

/* Process the ECDH public-key exchange and build its response. */
int acs_key_exchange_ecdh_pubkey(struct bt_acs_conn *acs_conn, struct net_buf *rsp_buf);

/* Derive the ECDH session key and build the KDF response. */
int acs_key_exchange_ecdh_kdf(struct bt_acs_conn *acs_conn, struct net_buf *rsp_buf);

/*
 * Send the server confirmation commitment. The client commitment is verified
 * later in acs_key_exchange_ecdh_confirm_rand().
 */
int acs_key_exchange_ecdh_confirm_code(struct bt_acs_conn *acs_conn, struct net_buf *rsp_buf);

/* Verify the client confirmation and send the server random-number response. */
int acs_key_exchange_ecdh_confirm_rand(struct bt_acs_conn *acs_conn,
				       const uint8_t client_random[ACS_CONFIRM_VALUE_SIZE],
				       struct net_buf *rsp_buf);

/* Perform a standalone KDF key exchange (§4.4.3.17.2.1). */
int acs_key_exchange_kdf(struct bt_acs_conn *acs_conn, struct net_buf *rsp_buf);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/* Checks if the opcode is allowed in the current key exchange state. See §4.4.3.17. */
bool acs_key_exchange_is_step_allowed(struct bt_acs_conn *acs_conn, uint8_t opcode,
				      const struct net_buf_simple *operand);

/* Abort the key exchange if one is running and opcode is one of its procedures. */
void acs_key_exchange_abort_if_active(struct bt_acs_conn *acs_conn, uint8_t opcode);
#else
static inline bool acs_key_exchange_is_step_allowed(struct bt_acs_conn *acs_conn, uint8_t opcode,
						    const struct net_buf_simple *operand)
{
	ARG_UNUSED(acs_conn);
	ARG_UNUSED(opcode);
	ARG_UNUSED(operand);
	return true;
}

static inline void acs_key_exchange_abort_if_active(struct bt_acs_conn *acs_conn, uint8_t opcode)
{
	ARG_UNUSED(acs_conn);
	ARG_UNUSED(opcode);
}
#endif

#endif /* BT_GATT_ACS_KEY_EXCHANGE_H_ */
