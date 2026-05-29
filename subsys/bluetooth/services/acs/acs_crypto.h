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
 * Per-connection crypto state lives on bt_acs_conn->crypto.key_runtimes[],
 * a unified array covering both key-exchange and algorithm descriptor records.
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
 * @brief Look up the unified runtime slot for @p key_id on @p acs_conn.
 *
 * Scans key_runtimes[] for any descriptor (exchange or algorithm) whose
 * key_id matches.
 *
 * @param acs_conn      Per-connection ACS state.
 * @param key_id        Spec Key_ID to resolve.
 * @param key_runtime   [out] Matching runtime slot.
 * @return 0 on success, -ENOENT if no slot is reserved for @p key_id,
 *         -EINVAL for invalid arguments.
 */
int acs_crypto_key_runtime_lookup(const struct bt_acs_conn *acs_conn, uint16_t key_id,
				  struct bt_acs_key_desc_runtime **key_runtime);

/**
 * @brief Resolve the exchange key used by @p isc_id on @p acs_conn.
 *
 * Follows the static descriptor relation ISC -> Key Descriptor ->
 * parent Key_ID (if any) until it reaches the exchange key that owns the
 * runtime key material for this message.
 */
int acs_crypto_current_key_from_isc(struct bt_acs_conn *acs_conn, uint16_t isc_id,
				    struct bt_acs_key_desc_runtime **current_key);

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
 * @param acs_conn        ACS connection context.
 * @param client_random   Client random number in wire order (LSO first).
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_derive_session_key(struct bt_acs_conn *acs_conn,
				  const uint8_t client_random[ACS_CONFIRM_VALUE_SIZE]);

/**
 * @brief Import an exchange key into the PSA keystore.
 *
 * Populates both the operational exchange-key handle and its derivation-capable
 * twin from the same caller-supplied key material.
 *
 * @param key_runtime    Runtime slot to populate with PSA key handles.
 * @param key_material   Raw key bytes to import.
 * @param key_len        Length of @p key_material.
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_import_exchange_key(struct bt_acs_key_desc_runtime *key_runtime,
				   const uint8_t *key_material, size_t key_len);

/**
 * @brief Release exchange-key handles without destroying the underlying PSA keys.
 *
 * Zeros the psa_key_id fields in the connection's exchange-key slots so the
 * connection no longer references the PSA slots, but the volatile PSA keys
 * remain alive (owned by the session cache).
 */
void acs_crypto_release_exchange_keys(struct bt_acs_conn *acs_conn);

/** @brief Destroy one key from the PSA keystore and zero its runtime slot. */
void acs_crypto_destroy_key(struct bt_acs_key_desc_runtime *key_runtime);

/** @brief Destroy every imported exchange key on @p acs_conn. */
void acs_crypto_destroy_exchange_keys(struct bt_acs_conn *acs_conn);

/**
 * @brief Import one key-descriptor runtime key into the PSA keystore.
 *
 * @param key_desc_runtime  Algorithm record runtime to import for.
 * @param key_material      Raw key bytes (from the parent exchange key).
 * @param key_len           Length of @p key_material.
 */
int acs_crypto_import_record_key(struct bt_acs_key_desc_runtime *key_desc_runtime,
				 const uint8_t *key_material, size_t key_len);

/** @brief Destroy every imported key-descriptor runtime key on @p acs_conn. */
void acs_crypto_destroy_connection_record_keys(struct bt_acs_conn *acs_conn);

/**
 * @brief Mint algorithm-record children of @p parent on @p acs_conn.
 *
 * For each algorithm record whose declared parent matches @p parent's Key_ID,
 * destroy any stale PSA key on its slot, then psa_copy_key from @p parent
 * under the record's algorithm-specific policy, then reset nonce counters to
 * the fresh-session initial state.  Records pointing at a different parent
 * are untouched.
 *
 * @param acs_conn  Per-connection ACS state.
 * @param parent    Exchange-key runtime with a live PSA key (asserts).
 * @return 0 on success, negative errno on policy/copy failure.
 */
int acs_crypto_bind_algorithm_keys(struct bt_acs_conn *acs_conn,
				   struct bt_acs_key_desc_runtime *parent);

/**
 * @brief Destroy every algorithm-record PSA key and clear their nonce state.
 *
 * Use after tearing down a parent exchange key (CP Invalidate Key for the KDF
 * child, KDF arm of acs_key_exchange_abort, or any full-teardown path).  In
 * current ACS builds all algorithm records share one parent, so the no-filter
 * form is equivalent to per-parent invalidation; introduce a filtered variant
 * if a future descriptor set declares algorithm records with distinct parents.
 */
void acs_crypto_invalidate_algorithm_keys(struct bt_acs_conn *acs_conn);

/**
 * @brief Import an exchange key, rebind algorithm records, and derive nonce state.
 *
 * Destroys any existing PSA key on @p exchange_key before importing.
 */
int acs_crypto_activate_key(struct bt_acs_conn *acs_conn,
			    struct bt_acs_key_desc_runtime *exchange_key,
			    const uint8_t *key_material, size_t key_len);

/** @brief Reset per-connection crypto runtime state and rebind static slots. */
void acs_crypto_reset(struct bt_acs_conn *acs_conn);

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

#ifdef __cplusplus
}
#endif

#endif /* ACS_CRYPTO_H_ */
