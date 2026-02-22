/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_CRYPTO_CONFIG_H_
#define BT_GATT_ACS_CRYPTO_CONFIG_H_

#include <zephyr/sys/util.h>

#include <psa/crypto_sizes.h>
#include <psa/crypto_values.h>

#ifdef __cplusplus
extern "C" {
#endif

/* HKDF algorithm for ACS session key derivation (HKDF-SHA-256, ACP 1.0 §4.1.1.1). */
#define ACS_PSA_HKDF_ALG PSA_ALG_HKDF(PSA_ALG_SHA_256)

/* Every supported ACS data-protection algorithm uses an AES-128 key. */
#define ACS_AES_KEY_SIZE 16U
#define ACS_AES_KEY_BITS (ACS_AES_KEY_SIZE * BITS_PER_BYTE)

/* ECDH is fixed to Curve P-256 (secp256r1): 32-octet coordinates and shared secret. */
#define ACS_ECDH_COORD_SIZE    32
#define ACS_SHARED_SECRET_SIZE 32

/* HKDF-SHA-256 salt and additional-info sizes in bytes (SHA-256 hash length). */
#define ACS_KDF_SALT_SIZE 32
#define ACS_KDF_INFO_SIZE 32

/* CCM nonce type tag: sequential with a fixed prefix. */
#define ACS_CCM_NONCE_TYPE ACS_NONCE_SEQ_DIFF_FIXED /* 0x02 */

/* AES-CCM fixed nonce size; total nonce size is 13 octets. */
#define ACS_CCM_NONCE_FIXED_SIZE 5

/* Variable (counter) part of the AES-CCM nonce in bytes. */
#define ACS_CCM_NONCE_VAR_SIZE 8

/* Total AES-CCM nonce length in bytes (fixed + variable). */
#define ACS_CCM_NONCE_SIZE (ACS_CCM_NONCE_FIXED_SIZE + ACS_CCM_NONCE_VAR_SIZE) /* 13 */

/* AES authentication tag overhead in bytes (full AES-GCM/GMAC/CCM tag). */
#define ACS_CRYPTO_AUTH_TAG_SIZE                                                                   \
	PSA_AEAD_TAG_LENGTH(PSA_KEY_TYPE_AES, ACS_AES_KEY_BITS,                                    \
			    PSA_ALG_GCM)

/*
 * AES-CCM tag length, falling back to the generic size when CCM is not
 * compiled in.
 */
#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
#define ACS_CCM_MAC_SIZE CONFIG_BT_ACS_CCM_MAC_SIZE
#else
#define ACS_CCM_MAC_SIZE ACS_CRYPTO_AUTH_TAG_SIZE
#endif

/* AES-GCM and GMAC fixed nonce size. */
#define ACS_GCM_NONCE_FIXED_SIZE 4

/* AES-GCM and GMAC nonce counter size. */
#define ACS_GCM_NONCE_VAR_SIZE 8

/* Total AES-GCM nonce length in bytes (fixed + variable). */
#define ACS_GCM_NONCE_SIZE (ACS_GCM_NONCE_FIXED_SIZE + ACS_GCM_NONCE_VAR_SIZE) /* 12 */

/* GMAC nonce/IV constants - identical to GCM (GMAC is GCM with zero-length plaintext). */
#define ACS_GMAC_NONCE_FIXED_SIZE ACS_GCM_NONCE_FIXED_SIZE
#define ACS_GMAC_NONCE_VAR_SIZE   ACS_GCM_NONCE_VAR_SIZE
#define ACS_GMAC_NONCE_SIZE       ACS_GCM_NONCE_SIZE

/* Maximum nonce sizes among the enabled algorithms. */
BUILD_ASSERT(ACS_CCM_NONCE_SIZE >= ACS_GCM_NONCE_SIZE &&
		     ACS_CCM_NONCE_FIXED_SIZE >= ACS_GCM_NONCE_FIXED_SIZE &&
		     ACS_CCM_NONCE_VAR_SIZE == ACS_GCM_NONCE_VAR_SIZE,
	     "AES-CCM must carry the largest nonce for the maxima below to hold");

#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
#define ACS_MAX_NONCE_SIZE        ACS_CCM_NONCE_SIZE
#define ACS_MAX_NONCE_VAR_SIZE    ACS_CCM_NONCE_VAR_SIZE
#define ACS_MAX_NONCE_PREFIX_SIZE ACS_CCM_NONCE_FIXED_SIZE
#elif defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                            \
	defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
#define ACS_MAX_NONCE_SIZE        ACS_GCM_NONCE_SIZE
#define ACS_MAX_NONCE_VAR_SIZE    ACS_GCM_NONCE_VAR_SIZE
#define ACS_MAX_NONCE_PREFIX_SIZE ACS_GCM_NONCE_FIXED_SIZE
#else
#define ACS_MAX_NONCE_SIZE        0
#define ACS_MAX_NONCE_VAR_SIZE    0
#define ACS_MAX_NONCE_PREFIX_SIZE 1
#endif

/* Sequence numbers are held in uint64_t counters end to end. */
BUILD_ASSERT(ACS_MAX_NONCE_VAR_SIZE <= 8, "nonce variable part is limited to 8 octets");

/* Largest fixed nonce among the enabled algorithms. */
#define ACS_MAX_NONCE_FIXED_SIZE ACS_MAX_NONCE_PREFIX_SIZE

/*
 * Every algorithm caps at a 128-bit tag. Must stay a literal constant
 * expression: NET_BUF_POOL_FIXED_DEFINE() uses it as a dimension.
 */
#define ACS_MAX_AUTH_TAG_SIZE 16U

/*
 * In-place encryption headroom: ISC_ID (2) + Nonce_Var. The tag needs
 * tailroom instead, already counted in ACS_BUF_SIZE.
 */
#define ACS_CRYPTO_HEADROOM (2U + ACS_MAX_NONCE_VAR_SIZE)

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_CRYPTO_CONFIG_H_ */
