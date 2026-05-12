/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_CRYPTO_H_
#define ACS_CRYPTO_H_

/*
 * Crypto and key-exchange API used by the runtime and reply layers.
 *
 * Per-connection crypto state lives on bt_acs_conn->crypto:
 *   - current_keys[] bound to key-exchange descriptor records
 *   - record_states[] bound to AES-with-nonce key descriptor records
 *
 * This header exposes resolution helpers and crypto operations on that state
 * plus the pooled key-exchange context lifecycle.
 */

#include <stddef.h>
#include <stdint.h>

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Look up the runtime key-state slot for @p key_id on @p acs_conn.
 *
 * @param acs_conn      Per-connection ACS state.
 * @param key_id        Spec Key_ID to resolve.
 * @param current_key   [out] Matching runtime key-state slot.
 * @return 0 on success, -ENOENT if no slot is reserved for @p key_id,
 *         -EINVAL for invalid arguments.
 */
int acs_crypto_current_key_lookup(struct bt_acs_conn *acs_conn, uint16_t key_id,
				  struct bt_acs_runtime_key_state **current_key);

/**
 * @brief Resolve the current key used by @p isc_id on @p acs_conn.
 *
 * Follows the static descriptor relation ISC -> Key Descriptor ->
 * parent Key_ID (if any) until it reaches the current key that must carry the
 * runtime material and nonce state for this message.
 */
int acs_crypto_current_key_from_isc(struct bt_acs_conn *acs_conn, uint16_t isc_id,
				    struct bt_acs_runtime_key_state **current_key);

/**
 * @brief Look up the runtime record-state slot for @p key_id on @p acs_conn.
 *
 * @param acs_conn      Per-connection ACS state.
 * @param key_id        AES-with-nonce Key_ID to resolve.
 * @param record_state  [out] Matching runtime record-state slot.
 * @return 0 on success, -ENOENT if no slot is reserved for @p key_id,
 *         -EINVAL for invalid arguments.
 */
int acs_crypto_record_state_lookup(struct bt_acs_conn *acs_conn, uint16_t key_id,
				   struct bt_acs_record_state **record_state);

/**
 * @brief Resolve the runtime record-state used by @p isc_id on @p acs_conn.
 */
int acs_crypto_record_state_from_isc(struct bt_acs_conn *acs_conn, uint16_t isc_id,
				     struct bt_acs_record_state **record_state);

/**
 * @brief Lazy-initialise and copy the server fixed nonce for @p key_id.
 *
 * The fixed part is carried per algorithm Key_ID, not once per connection.
 *
 * @param acs_conn  Per-connection ACS state.
 * @param key_id    AES-with-nonce Key_ID whose server nonce should be emitted.
 * @param nonce_buf Caller buffer to receive the nonce.
 * @param len       Size of @p nonce_buf.
 * @return 0 on success, -ENOTSUP if fixed nonce is not configured.
 */
int acs_crypto_get_server_nonce_fixed(struct bt_acs_conn *acs_conn, uint16_t key_id,
				      uint8_t *nonce_buf, size_t len);

/**
 * @brief Derive the session key from the completed key exchange.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_derive_session_key(struct bt_acs_conn *acs_conn);

/**
 * @brief Import a current key into the PSA keystore.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_import_current_key(struct bt_acs_runtime_key_state *current_key);

/** @brief Destroy one current key from the PSA keystore. */
void acs_crypto_destroy_current_key(struct bt_acs_runtime_key_state *current_key);

/** @brief Destroy every imported current key on @p acs_conn. */
void acs_crypto_destroy_connection_keys(struct bt_acs_conn *acs_conn);

/** @brief Import one record-state key into the PSA keystore. */
int acs_crypto_import_record_key(struct bt_acs_record_state *record_state);

/** @brief Destroy one record-state key from the PSA keystore. */
void acs_crypto_destroy_record_key(struct bt_acs_record_state *record_state);

/** @brief Destroy every imported record-state key on @p acs_conn. */
void acs_crypto_destroy_connection_record_keys(struct bt_acs_conn *acs_conn);

/** @brief Rebind record-state key material from the current exchange-key slots. */
int acs_crypto_rebind_record_states(struct bt_acs_conn *acs_conn);

/** @brief Reset nonce counters for record states that derive from @p current_key_id. */
void acs_crypto_reset_record_counters(struct bt_acs_conn *acs_conn, uint16_t current_key_id);

/**
 * @brief Encrypt @p plaintext using the session key.
 *
 * @param acs_conn   Per-connection state (holds key and nonce counters).
 * @param isc_id     ISC identifier for nonce construction.
 * @param plaintext  Input data.
 * @param plain_len  Input length.
 * @param ciphertext Output buffer (must fit ciphertext + auth tag).
 * @param cipher_len [out] Total output length.
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_encrypt(struct bt_acs_conn *acs_conn, uint16_t isc_id, const uint8_t *plaintext,
		       uint16_t plain_len, uint8_t *ciphertext, uint16_t *cipher_len);

/**
 * @brief Decrypt @p ciphertext using the session key.
 *
 * @param acs_conn   Per-connection state.
 * @param isc_id     ISC identifier for nonce construction.
 * @param ciphertext Input (ciphertext + auth tag).
 * @param cipher_len Total input length.
 * @param plaintext  Output buffer.
 * @param plain_len  [out] Plaintext length.
 * @param aad        Additional authenticated data (may be NULL).
 * @param aad_len    Length of @p aad.
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_decrypt(struct bt_acs_conn *acs_conn, uint16_t isc_id, const uint8_t *ciphertext,
		       uint16_t cipher_len, uint8_t *plaintext, uint16_t *plain_len,
		       const uint8_t *aad, uint16_t aad_len);

/** @brief Allocate a transient key-exchange context. Returns NULL if pool exhausted. */
int acs_kex_alloc(struct bt_acs_conn *acs_conn);

/** @brief Return a key-exchange context to the pool. */
void acs_kex_free(struct bt_acs_kex_ctx *kex);

#ifdef __cplusplus
}
#endif

#endif /* ACS_CRYPTO_H_ */
