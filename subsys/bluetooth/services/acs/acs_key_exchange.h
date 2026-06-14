/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_KEY_EXCHANGE_H_
#define BT_GATT_ACS_KEY_EXCHANGE_H_

#include <zephyr/types.h>
#include <zephyr/net_buf.h>
#include "acs_internal.h"

/**
 * @brief Allocate the key-exchange context for a new exchange on @p acs_conn.
 *
 * The transaction state machine lives in this pooled context; the caller sets
 * the initial state once the Start Key Exchange operand is validated.
 *
 * @return The zeroed context now referenced by @c acs_conn->kex, or NULL when
 *         the pool is exhausted.
 */
struct bt_acs_kex_ctx *acs_kex_alloc(struct bt_acs_conn *acs_conn);

/**
 * @brief Conclude the inbound phase of the exchange with a verdict.
 *
 * Arms the completion chain: the next indication confirms drive
 * @ref acs_kex_continue, which sends Key Exchange Response (success/failure)
 * and then commits the keys or tears the exchange down.
 */
void acs_kex_conclude(struct acs_procedure *proc, bool failed);

/** @brief Advance the completion chain on indication confirm (sequence engine hook). */
int acs_kex_continue(struct acs_procedure *proc);

/** @brief Initialize the Key Exchange procedure. */
int acs_key_exchange_ecdh_start(struct bt_acs_conn *acs_conn, uint16_t key_id);

/** @brief Abort the in-flight key exchange and tear down any partial key state. */
void acs_key_exchange_abort(struct bt_acs_conn *acs_conn);

/** @brief Process the Public Key Exchange. */
int acs_key_exchange_ecdh_pubkey(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf);

/** @brief Perform Key Derivation (KDF). */
int acs_key_exchange_ecdh_kdf(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf);

/**
 * @brief Compute and send the server's confirmation code commitment.
 *
 * Client verification is deferred to acs_key_exchange_ecdh_confirm_rand.
 */
int acs_key_exchange_ecdh_confirm_code(struct bt_acs_conn *acs_conn,
				       struct net_buf_simple *rsp_buf);

/** @brief Verify the client's confirmation code and send the server's random number. */
int acs_key_exchange_ecdh_confirm_rand(struct bt_acs_conn *acs_conn,
				       const uint8_t client_random[ACS_CONFIRM_VALUE_SIZE],
				       struct net_buf_simple *rsp_buf);

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
/** @brief Perform standalone KDF key exchange (§4.4.3.17.2.1). */
int acs_key_exchange_kdf(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf);
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF */

struct bt_acs_key_desc_runtime *
acs_key_exchange_established_key(struct bt_acs_conn const *acs_conn);

#endif /* BT_GATT_ACS_KEY_EXCHANGE_H_ */
