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
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_derive_session_key(struct bt_acs_conn *acs_conn);

/**
 * @brief Import an exchange key into the PSA keystore.
 *
 * @param key_runtime    Runtime slot to populate with a PSA key handle.
 * @param key_material   Raw key bytes to import.
 * @param key_len        Length of @p key_material.
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_import_exchange_key(struct bt_acs_key_desc_runtime *key_runtime,
				   const uint8_t *key_material, size_t key_len);

/**
 * @brief Export raw key bytes from a runtime slot's PSA key to a caller buffer.
 *
 * The caller must zeroize the output buffer after use.
 *
 * @param key_runtime  Runtime slot with a live PSA key.
 * @param buf          Output buffer for raw key bytes.
 * @param buf_len      Size of @p buf.
 * @param out_len      Receives the number of bytes written.
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_export_key(const struct bt_acs_key_desc_runtime *key_runtime, uint8_t *buf,
			  size_t buf_len, size_t *out_len);

/**
 * @brief Ensure that an exchange runtime key has a derivation-capable twin.
 *
 * Rebuilds @p key_runtime->derive_key_id from the installed exchange key if
 * needed so later HKDF steps can use psa_key_derivation_input_key().
 */
int acs_crypto_ensure_exchange_derive_key(struct bt_acs_key_desc_runtime *key_runtime);

/**
 * @brief Materialize a derivation-capable exchange key from a PSA derivation op.
 *
 * Outputs a key of type PSA_KEY_TYPE_DERIVE into @p key_runtime->derive_key_id.
 */
int acs_crypto_output_exchange_derive_key(struct bt_acs_key_desc_runtime *key_runtime,
					  psa_key_derivation_operation_t *op, size_t key_len);

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

/** @brief Log a warning if a PSA key destroy operation fails. */
void acs_crypto_warn_destroy_key_failure(psa_status_t status, psa_key_id_t key_id, const char *ctx);

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

/** @brief Destroy one key-descriptor runtime key from the PSA keystore. */
void acs_crypto_destroy_record_key(struct bt_acs_key_desc_runtime *key_desc_runtime);

/** @brief Destroy every imported key-descriptor runtime key on @p acs_conn. */
void acs_crypto_destroy_connection_record_keys(struct bt_acs_conn *acs_conn);

/** @brief Refresh algorithm-record PSA keys from the exchange-key slots. */
int acs_crypto_rebind_algorithm_keys(struct bt_acs_conn *acs_conn);

/**
 * @brief Refresh algorithm-record PSA keys by copying from the parent keystore
 *        entry instead of exporting and re-importing raw key bytes.
 *
 * Behaves like acs_crypto_rebind_algorithm_keys() but mints each record key via
 * psa_copy_key(), so no key material is materialized in plaintext.  Per-record
 * nonce state is reset (tx/rx counters and fixed parts), matching the state a
 * freshly restored session starts from.
 */
int acs_crypto_rebind_algorithm_keys_by_copy(struct bt_acs_conn *acs_conn);

/**
 * @brief Copy a runtime exchange key into a persistent PSA keystore entry.
 *
 * Duplicates the key inside the keystore (no plaintext export) under a
 * persistent policy at @p dst_id.  Replaces an existing entry at @p dst_id.
 *
 * @param parent  Runtime slot holding a live exchange key.
 * @param dst_id  Persistent PSA key id to populate.
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_copy_key_to_persistent(const struct bt_acs_key_desc_runtime *parent,
				      psa_key_id_t dst_id);

/**
 * @brief Copy a persistent PSA key into a fresh volatile runtime exchange key.
 *
 * Destroys any existing key on @p parent, then duplicates @p src_id inside the
 * keystore (no plaintext export), storing the new id in @p parent->psa_key_id.
 *
 * @param src_id  Persistent PSA key id to copy from.
 * @param parent  Runtime slot to populate.
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_copy_persistent_key_to_runtime(psa_key_id_t src_id,
					      struct bt_acs_key_desc_runtime *parent);

/**
 * @brief Import an exchange key, rebind algorithm records, and derive nonce state.
 *
 * Single entry point that replaces the three-call pattern of import + rebind +
 * nonce-derive.  Any existing PSA key on @p exchange_key is destroyed first.
 *
 * @param acs_conn      Per-connection ACS state.
 * @param exchange_key  Runtime slot for the exchange key to populate.
 * @param key_material  Raw key bytes (caller-owned, typically on the stack).
 * @param key_len       Length of @p key_material.
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_activate_key(struct bt_acs_conn *acs_conn,
			    struct bt_acs_key_desc_runtime *exchange_key,
			    const uint8_t *key_material, size_t key_len);

/** @brief Reset per-connection crypto runtime state and rebind static slots. */
void acs_crypto_reset(struct bt_acs_conn *acs_conn);

/**
 * @brief Reset crypto runtime state while preserving algorithm-record nonce data.
 *
 * Preserves the algorithm-record slots so fixed nonce values, client-nonce
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
