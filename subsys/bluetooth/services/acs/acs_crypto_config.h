/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_CRYPTO_CONFIG_H_
#define BT_GATT_ACS_CRYPTO_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @brief HMAC-SHA-256 output size in bytes (auth_value, randoms, confirms). */
#define ACS_HMAC_SHA256_SIZE 32

/** @brief Maximum nonce-variable size supported by the current runtime model. */
#define ACS_NONCE_VAR_COUNTER_SIZE 8

/** @brief Total CCM nonce length in bytes (13 bytes per CCM spec). */
#define ACS_NONCE_SIZE 13

/** @brief Default fixed-part size for CCM nonces when no scheme is configured (8 bytes). */
#define ACS_NONCE_FIXED_SIZE 8

#if defined(CONFIG_BT_ACS_CCM_NONCE_SEQ_DIFF_FIXED)
/** @brief CCM nonce type tag: sequential with a fixed prefix. */
#define ACS_CCM_NONCE_TYPE       ACS_NONCE_SEQ_DIFF_FIXED /**< 0x02 */
/** @brief Fixed-prefix length in bytes (1–12, set by CONFIG_BT_ACS_CCM_NONCE_FIXED_SIZE). */
#define ACS_CCM_NONCE_FIXED_SIZE CONFIG_BT_ACS_CCM_NONCE_FIXED_SIZE
/** @brief Variable (counter) part of the CCM nonce in bytes. */
#define ACS_CCM_NONCE_VAR_SIZE   (ACS_NONCE_SIZE - ACS_CCM_NONCE_FIXED_SIZE)
#endif
/** @} */

/**
 * @name Nonce exhaustion thresholds (§3.6.2)
 *
 * When the counter reaches these values the nonce space is exhausted and the
 * session key must be replaced before further messages can be sent or received.
 */
/** @brief Safe 64-bit TX counter ceiling (DIFF_FIXED and other schemes). */
#define ACS_COUNTER_TX_MAX UINT64_C(0xFFFFFFFFFFFFFFFE)
/** @brief Safe 64-bit RX counter ceiling (DIFF_FIXED and other schemes). */
#define ACS_COUNTER_RX_MAX UINT64_C(0xFFFFFFFFFFFFFFFE)
/** @} */

/** @brief AES authentication tag overhead in bytes (AES-GCM and AES-CCM). */
#define ACS_CRYPTO_AUTH_TAG_SIZE 16

/**
 * @brief AES-CCM authentication tag length in bytes.
 *
 * When AES-CCM is not the active protection algorithm, fall back to the
 * generic auth tag size so that CCM-guarded dead code remains compilable.
 */
#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
#define ACS_CCM_MAC_SIZE CONFIG_BT_ACS_CCM_MAC_SIZE
#else
#define ACS_CCM_MAC_SIZE ACS_CRYPTO_AUTH_TAG_SIZE
#endif

/** @brief Fixed-prefix length for AES-GCM nonces in bytes.
 *
 * ACS §4.4.4.15.1.4.4.6 says the fixed part for GCM/GMAC should be used as
 * the IV fixed field with 4 octets.
 */
#define ACS_GCM_NONCE_FIXED_SIZE 4

/** @brief Variable (counter) part of the AES-GCM nonce in bytes.
 *
 * ACS §4.4.4.15.1.4.4.5 says the GCM/GMAC variable part should be used as the
 * IV invocation field with 8 octets. The runtime keeps per-record counters in
 * 64-bit state so this shape is now supported end to end.
 */
#define ACS_GCM_NONCE_VAR_SIZE 8

/** @brief Total AES-GCM nonce length in bytes (fixed + variable). */
#define ACS_GCM_NONCE_SIZE (ACS_GCM_NONCE_FIXED_SIZE + ACS_GCM_NONCE_VAR_SIZE) /**< 12 */

/** @brief AES-GCM authentication tag length in bytes (always 128-bit per spec). */
#define ACS_GCM_MAC_SIZE 16

/** @brief GMAC nonce/IV constants — identical to GCM (GMAC is GCM with zero-length plaintext). */
#define ACS_GMAC_NONCE_FIXED_SIZE ACS_GCM_NONCE_FIXED_SIZE
#define ACS_GMAC_NONCE_VAR_SIZE   ACS_GCM_NONCE_VAR_SIZE
#define ACS_GMAC_NONCE_SIZE       ACS_GCM_NONCE_SIZE
#define ACS_GMAC_MAC_SIZE         ACS_GCM_MAC_SIZE

#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
#define ACS_CCM_NONCE_SIZE_OR_0       ACS_NONCE_SIZE
#define ACS_CCM_NONCE_VAR_SIZE_OR_0   ACS_CCM_NONCE_VAR_SIZE
#define ACS_CCM_NONCE_FIXED_SIZE_OR_0 ACS_CCM_NONCE_FIXED_SIZE
#else
#define ACS_CCM_NONCE_SIZE_OR_0       0
#define ACS_CCM_NONCE_VAR_SIZE_OR_0   0
#define ACS_CCM_NONCE_FIXED_SIZE_OR_0 0
#endif

#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
#define ACS_GCM_NONCE_SIZE_OR_0       ACS_GCM_NONCE_SIZE
#define ACS_GCM_NONCE_VAR_SIZE_OR_0   ACS_GCM_NONCE_VAR_SIZE
#define ACS_GCM_NONCE_FIXED_SIZE_OR_0 ACS_GCM_NONCE_FIXED_SIZE
#else
#define ACS_GCM_NONCE_SIZE_OR_0       0
#define ACS_GCM_NONCE_VAR_SIZE_OR_0   0
#define ACS_GCM_NONCE_FIXED_SIZE_OR_0 0
#endif

#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
#define ACS_GMAC_NONCE_SIZE_OR_0       ACS_GMAC_NONCE_SIZE
#define ACS_GMAC_NONCE_VAR_SIZE_OR_0   ACS_GMAC_NONCE_VAR_SIZE
#define ACS_GMAC_NONCE_FIXED_SIZE_OR_0 ACS_GMAC_NONCE_FIXED_SIZE
#else
#define ACS_GMAC_NONCE_SIZE_OR_0       0
#define ACS_GMAC_NONCE_VAR_SIZE_OR_0   0
#define ACS_GMAC_NONCE_FIXED_SIZE_OR_0 0
#endif

/** @brief Largest nonce size used by any enabled data-protection algorithm. */
#define ACS_MAX_NONCE_SIZE                                                                         \
	((ACS_CCM_NONCE_SIZE_OR_0 > ACS_GCM_NONCE_SIZE_OR_0)                                       \
		 ? ((ACS_CCM_NONCE_SIZE_OR_0 > ACS_GMAC_NONCE_SIZE_OR_0)                           \
			    ? ACS_CCM_NONCE_SIZE_OR_0                                              \
			    : ACS_GMAC_NONCE_SIZE_OR_0)                                            \
		 : ((ACS_GCM_NONCE_SIZE_OR_0 > ACS_GMAC_NONCE_SIZE_OR_0)                           \
			    ? ACS_GCM_NONCE_SIZE_OR_0                                              \
			    : ACS_GMAC_NONCE_SIZE_OR_0))

/** @brief Largest nonce-variable size used by any enabled data-protection algorithm. */
#define ACS_MAX_NONCE_VAR_SIZE                                                                     \
	((ACS_CCM_NONCE_VAR_SIZE_OR_0 > ACS_GCM_NONCE_VAR_SIZE_OR_0)                               \
		 ? ((ACS_CCM_NONCE_VAR_SIZE_OR_0 > ACS_GMAC_NONCE_VAR_SIZE_OR_0)                   \
			    ? ACS_CCM_NONCE_VAR_SIZE_OR_0                                          \
			    : ACS_GMAC_NONCE_VAR_SIZE_OR_0)                                        \
		 : ((ACS_GCM_NONCE_VAR_SIZE_OR_0 > ACS_GMAC_NONCE_VAR_SIZE_OR_0)                   \
			    ? ACS_GCM_NONCE_VAR_SIZE_OR_0                                          \
			    : ACS_GMAC_NONCE_VAR_SIZE_OR_0))

/** @brief Largest nonce-fixed size used by any enabled data-protection algorithm. */
#define ACS_MAX_NONCE_FIXED_SIZE                                                                   \
	((ACS_CCM_NONCE_FIXED_SIZE_OR_0 > ACS_GCM_NONCE_FIXED_SIZE_OR_0)                           \
		 ? ((ACS_CCM_NONCE_FIXED_SIZE_OR_0 > ACS_GMAC_NONCE_FIXED_SIZE_OR_0)               \
			    ? ACS_CCM_NONCE_FIXED_SIZE_OR_0                                        \
			    : ACS_GMAC_NONCE_FIXED_SIZE_OR_0)                                      \
		 : ((ACS_GCM_NONCE_FIXED_SIZE_OR_0 > ACS_GMAC_NONCE_FIXED_SIZE_OR_0)               \
			    ? ACS_GCM_NONCE_FIXED_SIZE_OR_0                                        \
			    : ACS_GMAC_NONCE_FIXED_SIZE_OR_0))

/** @brief Largest authentication tag size used by any enabled data-protection algorithm. */
#define ACS_MAX_AUTH_TAG_SIZE ACS_CRYPTO_AUTH_TAG_SIZE

/**
 * @brief Headroom reserved in protected response buffers for in-place encryption.
 *
 * ISC_ID (2 bytes) + Nonce_Var (variable-part nonce bytes).
 * The auth tag is appended after ciphertext — needs tailroom,
 * already accounted for in ACS_BUF_SIZE.
 */
#define ACS_CRYPTO_HEADROOM (2U + ACS_MAX_NONCE_VAR_SIZE)

/*
 * PSA algorithm and key-usage constants are now defined locally in the crypto
 * implementation files to support multi-algorithm dispatch.
 */

#ifdef __cplusplus

#endif

#endif /* BT_GATT_ACS_CRYPTO_CONFIG_H_ */
