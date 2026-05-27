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
 * @brief Initialize the Key Exchange procedure.
 *
 * Prepares the connection context for a new key exchange. This must be the first
 * procedure called in the handshake sequence.
 *
 * @param acs_conn ACS connection context.
 * @param key_id   The Key ID requested by the client (must match advertised descriptors).
 *
 * @return 0 on success.
 * @return -EALREADY if the Key ID is not supported.
 */
int acs_key_exchange_ecdh_start(struct bt_acs_conn *acs_conn, uint16_t key_id);

/**
 * @brief Abort the in-flight key exchange and tear down any partial key state.
 *
 * Frees the transient KEX context and destroys any key material imported for
 * the exchange before it was fully completed.
 *
 * @param acs_conn ACS connection context.
 */
void acs_key_exchange_abort(struct bt_acs_conn *acs_conn);

/**
 * @brief Process the Public Key Exchange.
 *
 * Reads acs_conn->client_pubkey (wire format set by handle_key_exchange_ecdh),
 * generates the server's ephemeral keypair, computes the shared ECDH secret, and
 * appends the server public key in spec wire format to rsp_buf.
 *
 * @param acs_conn ACS connection context (client_pubkey must already be set).
 * @param rsp_buf  Buffer to append the server public key response into.
 *
 * @return 0 on success.
 * @return -EAGAIN if the procedure is called out of order.
 * @return -EIO if the internal crypto engine fails.
 * @return -ENOMEM if rsp_buf does not have enough space.
 */
int acs_key_exchange_ecdh_pubkey(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf);

/**
 * @brief Perform Key Derivation (KDF).
 *
 * Derives the 128-bit symmetric session key from the previously computed ECDH
 * shared secret using the configured Key Derivation Function (e.g., HKDF-SHA256).
 *
 * @param acs_conn ACS connection context.
 * @param rsp_buf  Buffer to store the KDF response (containing Salt and Info if used).
 *
 * @return 0 on success.
 * @return -EAGAIN if the procedure is called out of order.
 * @return -EIO if the key derivation math fails.
 */
int acs_key_exchange_ecdh_kdf(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf);

/**
 * @brief Compute and send the server's confirmation code commitment.
 *
 * acs_conn->client_confirm must already be set by handle_ecdh_confirm_code.
 * Verification of the client's commitment is intentionally deferred to
 * acs_key_exchange_ecdh_confirm_rand (after the client reveals its random number).
 *
 * @param acs_conn ACS connection context.
 * @param rsp_buf  Buffer to append the server confirmation code into.
 *
 * @return 0 on success.
 * @return -EAGAIN if called out of order.
 * @return -EIO if the HMAC computation fails.
 * @return -ENOMEM if rsp_buf does not have enough space.
 */
int acs_key_exchange_ecdh_confirm_code(struct bt_acs_conn *acs_conn,
				       struct net_buf_simple *rsp_buf);

/**
 * @brief Verify the client's confirmation code and send the server's random number.
 *
 * acs_conn->client_random must already be set by handle_ecdh_confirm_rand.
 * Verifies HMAC(ConfirmationKey, client_random) == client_confirm, then appends
 * server_random to rsp_buf. Session key derivation and callbacks are handled
 * by the caller after this function returns successfully.
 *
 * @param acs_conn ACS connection context.
 * @param rsp_buf  Buffer to append the server random number into.
 *
 * @return 0 on success.
 * @return -EAGAIN if called out of order.
 * @return -EACCES if the client's confirmation code fails verification.
 */
int acs_key_exchange_ecdh_confirm_rand(struct bt_acs_conn *acs_conn,
				       struct net_buf_simple *rsp_buf);

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
/**
 * @brief Perform standalone KDF key exchange (§4.4.3.17.2.1).
 *
 * Derives a lower-level child key from the established parent session key using HKDF.
 *
 * @param acs_conn ACS connection context; session_key must be established.
 * @param rsp_buf  Buffer to append the Key Exchange KDF Response operand into.
 *
 * @return 0        on success.
 * @return -EAGAIN  if the procedure is called out of order.
 * @return -EIO     if HKDF child key derivation fails.
 * @return -ENOMEM  if rsp_buf has insufficient tailroom.
 */
int acs_key_exchange_kdf(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf);
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF */

struct bt_acs_key_desc_runtime *acs_key_exchange_established_key(struct bt_acs_conn *acs_conn);

int acs_key_exchange_step_response(struct acs_procedure *proc, uint8_t status);
int acs_key_exchange_step_success_status(struct acs_procedure *proc);

#endif /* BT_GATT_ACS_KEY_EXCHANGE_H_ */
