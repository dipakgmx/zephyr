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
 * Per-connection crypto state lives on bt_acs_conn (key_state, rx/tx nonce
 * counters, session-key handle). This header exposes the operations on that
 * state plus the pooled key-exchange context lifecycle.
 */

#include <stddef.h>
#include <stdint.h>

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Lazy-initialise and copy the server fixed nonce for @p acs_conn.
 *
 * Must be called before START_KEY_EXCHANGE for SEQ_DIFF_FIXED nonce types
 * (spec §4.4.3.13).
 *
 * @param acs_conn  Per-connection ACS state.
 * @param nonce_buf Caller buffer to receive the nonce.
 * @param len       Size of @p nonce_buf.
 * @return 0 on success, -ENOTSUP if fixed nonce is not configured.
 */
int acs_crypto_get_server_nonce_fixed(struct bt_acs_conn *acs_conn, uint8_t *nonce_buf, size_t len);

/**
 * @brief Derive the session key from the completed key exchange.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_derive_session_key(struct bt_acs_conn *acs_conn);

/**
 * @brief Import the session key into the PSA keystore.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_import_session_key(struct bt_acs_conn *acs_conn);

/** @brief Destroy the session key from the PSA keystore. */
void acs_crypto_destroy_session_key(struct bt_acs_conn *acs_conn);

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
