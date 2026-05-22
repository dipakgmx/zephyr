/*
 * Copyright (c) 2025 Dipak Shetty
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
 *   - key_desc_runtimes[] bound to AES-with-nonce key descriptor records
 *
 * This header exposes resolution helpers and crypto operations on that state
 * plus the pooled key-exchange context lifecycle.
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Resolve the current exchange-key Key_ID backing a descriptor record.
 *
 * Walks from an ISC or algorithm descriptor through the parent relation until
 * it reaches the non-algorithm current key that owns the runtime key material.
 *
 * @param rec             Descriptor record to resolve.
 * @param current_key_id  [out] Resolved current-key Key_ID.
 * @return 0 on success, -ENOENT if the relation cannot be resolved,
 *         -EINVAL for invalid arguments.
 */
int acs_crypto_current_key_id_from_key_desc(const struct bt_acs_key_desc_record *rec,
					    uint16_t *current_key_id);

/**
 * @brief Look up the runtime key-state slot for @p key_id on @p acs_conn.
 *
 * @param acs_conn      Per-connection ACS state.
 * @param key_id        Spec Key_ID to resolve.
 * @param current_key   [out] Matching runtime key-state slot.
 * @return 0 on success, -ENOENT if no slot is reserved for @p key_id,
 *         -EINVAL for invalid arguments.
 */
int acs_crypto_current_key_lookup(const struct bt_acs_conn *acs_conn, uint16_t key_id,
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
 * @brief Look up the key-descriptor runtime slot for @p key_id on @p acs_conn.
 *
 * @param acs_conn      Per-connection ACS state.
 * @param key_id        AES-with-nonce Key_ID to resolve.
 * @param key_desc_runtime  [out] Matching key-descriptor runtime slot.
 * @return 0 on success, -ENOENT if no slot is reserved for @p key_id,
 *         -EINVAL for invalid arguments.
 */
int acs_crypto_key_desc_runtime_lookup(struct bt_acs_conn *acs_conn, uint16_t key_id,
				       struct bt_acs_key_desc_runtime **key_desc_runtime);

/** @brief Bind per-connection runtime slots to the static key-descriptor graph. */
void acs_crypto_init_slots(struct bt_acs_conn *acs_conn);

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

/** @brief Log a warning if a PSA key destroy operation fails. */
void acs_crypto_warn_destroy_key_failure(psa_status_t status, psa_key_id_t key_id, const char *ctx);

/** @brief Destroy every imported current key on @p acs_conn. */
void acs_crypto_destroy_connection_keys(struct bt_acs_conn *acs_conn);

/** @brief Import one key-descriptor runtime key into the PSA keystore. */
int acs_crypto_import_record_key(struct bt_acs_key_desc_runtime *key_desc_runtime);

/** @brief Destroy one key-descriptor runtime key from the PSA keystore. */
void acs_crypto_destroy_record_key(struct bt_acs_key_desc_runtime *key_desc_runtime);

/** @brief Destroy every imported key-descriptor runtime key on @p acs_conn. */
void acs_crypto_destroy_connection_record_keys(struct bt_acs_conn *acs_conn);

/** @brief Refresh key-descriptor runtime key material from the current exchange-key slots. */
int acs_crypto_rebind_key_desc_runtimes(struct bt_acs_conn *acs_conn);

/** @brief Reset nonce counters for key-descriptor runtimes that derive from @p current_key_id. */
void acs_crypto_reset_key_desc_runtime_counters(struct bt_acs_conn *acs_conn,
						uint16_t current_key_id);

/** @brief Reset per-connection crypto runtime state and rebind static slots. */
void acs_crypto_reset(struct bt_acs_conn *acs_conn);

/**
 * @brief Reset crypto runtime state while preserving key-descriptor runtime nonce data.
 *
 * Preserves the bound key-descriptor runtime slots so fixed nonce values, client-nonce
 * state, and nonce counters survive the reset.
 */
void acs_crypto_reset_preserve_record_states(struct bt_acs_conn *acs_conn);

/**
 * @brief Encrypt @p plaintext using the resolved key-descriptor runtime key.
 *
 * @param key_desc_runtime Resolved key-descriptor runtime holding the key and nonce counters.
 * @param plaintext  Input data.
 * @param plain_len  Input length.
 * @param ciphertext Output buffer (must fit ciphertext + auth tag).
 * @param cipher_len [out] Total output length.
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_encrypt(struct bt_acs_key_desc_runtime *key_desc_runtime, const uint8_t *plaintext,
		       uint16_t plain_len, uint8_t *ciphertext, uint16_t *cipher_len);

/**
 * @brief Decrypt @p ciphertext using the resolved key-descriptor runtime key.
 *
 * @param key_desc_runtime Resolved key-descriptor runtime holding the key and nonce counters.
 * @param ciphertext Input (ciphertext + auth tag).
 * @param cipher_len Total input length.
 * @param plaintext  Output buffer.
 * @param plain_len  [out] Plaintext length.
 * @param aad        Additional authenticated data (may be NULL).
 * @param aad_len    Length of @p aad.
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_decrypt(struct bt_acs_key_desc_runtime *key_desc_runtime, const uint8_t *ciphertext,
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
