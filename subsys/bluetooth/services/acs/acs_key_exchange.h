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
struct bt_acs_kex_ctx *acs_kex_alloc(struct bt_acs_conn *acs_conn);

/* Finish the request phase and start the key-exchange responses. */
void acs_kex_conclude(struct acs_reply *reply);

/* Send the Key Exchange Response indication (Table 4.77). */
int acs_kex_send_result(struct acs_reply *reply);

/* Finish a successful exchange, notify the application, and update Status. */
void acs_kex_finalize_success(struct bt_acs_conn *conn);

/* Start an ECDH key exchange for key_id. */
int acs_key_exchange_ecdh_start(struct bt_acs_conn *acs_conn, uint16_t key_id);

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

/* Return the installed KDF or ECDH key, preferring KDF when both exist. */
struct bt_acs_key_desc_runtime *acs_key_exchange_installed_key(struct bt_acs_conn *acs_conn);

/* Check the state and Key_ID for a key-exchange procedure. */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
bool acs_kex_step_allowed(struct bt_acs_conn *acs_conn, uint8_t opcode,
			  const struct net_buf_simple *operand);

/* Abort an in-progress exchange when one of its procedures fails. */
void acs_kex_abort_failed_procedure(struct bt_acs_conn *acs_conn, uint8_t opcode);
#else
static inline bool acs_kex_step_allowed(struct bt_acs_conn *acs_conn, uint8_t opcode,
					const struct net_buf_simple *operand)
{
	ARG_UNUSED(acs_conn);
	ARG_UNUSED(opcode);
	ARG_UNUSED(operand);
	return true;
}

static inline void acs_kex_abort_failed_procedure(struct bt_acs_conn *acs_conn, uint8_t opcode)
{
	ARG_UNUSED(acs_conn);
	ARG_UNUSED(opcode);
}
#endif

#endif /* BT_GATT_ACS_KEY_EXCHANGE_H_ */
