/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_CRYPTO_H_
#define ACS_CRYPTO_H_

/*
 * Operations on bt_acs_conn->crypto.key_runtimes[], which covers both
 * key-exchange and algorithm descriptor records.
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Find the runtime slot for an exchange or algorithm Key_ID. */
int acs_crypto_key_runtime_lookup(struct bt_acs_conn *acs_conn, uint16_t key_id,
				  struct bt_acs_key_desc_runtime **key_runtime);

/* Bind per-connection runtime slots to the static key-descriptor graph. */
void acs_crypto_init_slots(struct bt_acs_conn *acs_conn);

/*
 * Copy the server fixed nonce for key_id into nonce_buf. The nonce is carried
 * per Key_ID rather than once per connection.
 */
int acs_crypto_get_server_nonce_fixed(struct bt_acs_conn *acs_conn, uint16_t key_id,
				      uint8_t *nonce_buf, size_t len);

/*
 * Generate the ephemeral ECDH key pair and export its public coordinates to
 * kex.server_pubkey in little-endian wire order.
 */
int acs_crypto_generate_keypair(struct bt_acs_conn *acs_conn);

/*
 * Compute the ECDH shared secret using kex.client_pubkey, import it as
 * kex.derived_key_id, and destroy the ephemeral private key.
 */
int acs_crypto_compute_shared_secret(struct bt_acs_conn *acs_conn);

/*
 * Import an exchange key into both the operational handle and its
 * derivation-capable twin.
 */
int acs_crypto_import_exchange_key(struct bt_acs_key_desc_runtime *key_runtime,
				   const uint8_t *key_material, size_t key_len);

/* Destroy one PSA key and reset its runtime slot. */
void acs_crypto_destroy_key(struct bt_acs_key_desc_runtime *key_runtime);

/* Destroy every imported exchange key on acs_conn. */
void acs_crypto_destroy_exchange_keys(struct bt_acs_conn *acs_conn);

/* Destroy every imported key-descriptor runtime key on acs_conn. */
void acs_crypto_destroy_connection_record_keys(struct bt_acs_conn *acs_conn);

/* Install the parent key for its algorithm records and optionally reset their nonces. */
int acs_crypto_bind_algorithm_keys(struct bt_acs_conn *acs_conn,
				   struct bt_acs_key_desc_runtime *parent, bool reset_nonce_state);

/* Destroy every algorithm key and clear its nonce state. */
void acs_crypto_invalidate_algorithm_keys(struct bt_acs_conn *acs_conn);

/* Destroy the KDF child and its algorithm keys. Keep the ECDH parent. */
void acs_crypto_destroy_kdf_keys(struct bt_acs_conn *acs_conn);

/* Reset the connection's crypto state and set up its key slots again. */
void acs_crypto_reset(struct bt_acs_conn *acs_conn);

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
/* Return -EEXIST if a nonce prefix is already used on any connection. */
int acs_client_nonce_fixed_check_unique(struct bt_acs_conn *acs_conn,
					const struct bt_acs_key_desc_runtime *runtime,
					const uint8_t *candidate, uint8_t fixed_size);
#endif

/* Encrypt buf in wire order. Return -ENOSPC when the nonce counter is exhausted. */
int acs_crypto_encrypt(struct bt_acs_key_desc_runtime *key_desc_runtime, uint8_t *buf,
		       uint16_t plain_len, uint16_t *cipher_len);

/* Decrypt buf in wire order. Return -EACCES if authentication fails. */
int acs_crypto_decrypt(struct bt_acs_key_desc_runtime *key_desc_runtime, uint64_t received_counter,
		       uint8_t *buf, uint16_t buf_len, uint16_t *plain_len);

#ifdef __cplusplus
}
#endif

#endif /* ACS_CRYPTO_H_ */
