/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_CRYPTO_H_
#define ACS_CRYPTO_H_

/*
 * Protection of Data In / Data Out payloads with a security algorithm: the
 * nonce, the sequence numbers and the wire byte order (§4.3.2).
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Encrypt buf in wire order. Return -ENOSPC when the nonce counter is exhausted. */
int acs_crypto_encrypt(struct acs_sec_alg *alg, uint8_t *buf, uint16_t plain_len,
		       uint16_t *cipher_len);

/*
 * Decrypt buf in wire order; buf_len must cover the authentication tag.
 * Return -EACCES if authentication fails.
 */
int acs_crypto_decrypt(struct acs_sec_alg *alg, uint64_t received_counter,
		       uint8_t *buf, uint16_t buf_len, uint16_t *plain_len);

#ifdef __cplusplus
}
#endif

#endif /* ACS_CRYPTO_H_ */
