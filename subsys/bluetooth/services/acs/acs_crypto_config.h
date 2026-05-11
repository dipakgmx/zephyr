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
#elif defined(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD)
/** @brief CCM nonce type tag: even/odd sequential scheme — no fixed prefix. */
#define ACS_CCM_NONCE_TYPE       ACS_NONCE_SEQ_EVEN_ODD /**< 0x01 */
/** @brief Fixed-prefix length is zero for the even/odd scheme. */
#define ACS_CCM_NONCE_FIXED_SIZE 0
/** @brief The full 13-byte nonce is the counter for the even/odd scheme. */
#define ACS_CCM_NONCE_VAR_SIZE   ACS_NONCE_SIZE /**< 13 bytes */
#endif
/** @} */

/**
 * @name Nonce exhaustion thresholds (§3.6.2)
 *
 * When the counter reaches these values the nonce space is exhausted and the
 * session key must be replaced before further messages can be sent or received.
 */
#if defined(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD)
/** @brief Last valid even server-TX nonce (EVEN_ODD scheme). */
#define ACS_COUNTER_TX_MAX 0xFFFFFFFEU
/** @brief Last valid odd client-RX nonce (EVEN_ODD scheme). */
#define ACS_COUNTER_RX_MAX 0xFFFFFFFDU
#else
/** @brief Safe 32-bit TX counter ceiling (DIFF_FIXED and other schemes). */
#define ACS_COUNTER_TX_MAX 0xFFFFFFFEU
/** @brief Safe 32-bit RX counter ceiling (DIFF_FIXED and other schemes). */
#define ACS_COUNTER_RX_MAX 0xFFFFFFFEU
#endif
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
 * the IV fixed field with 4 octets. We intentionally advertise 8 here because
 * this implementation currently caps the variable part at 32 bits.
 */
#define ACS_GCM_NONCE_FIXED_SIZE 8

/** @brief Variable (counter) part of the AES-GCM nonce in bytes.
 *
 * The current ACS implementation intentionally caps runtime nonce counters at
 * 32 bits. ACS §4.4.4.15.1.4.4.5 says the GCM/GMAC variable part should be
 * used as the IV invocation field with 8 octets, but we intentionally
 * advertise 4 here so we do not claim support for a larger nonce space than
 * the code actually maintains.
 */
#define ACS_GCM_NONCE_VAR_SIZE 4

/** @brief Total AES-GCM nonce length in bytes (fixed + variable). */
#define ACS_GCM_NONCE_SIZE (ACS_GCM_NONCE_FIXED_SIZE + ACS_GCM_NONCE_VAR_SIZE) /**< 12 */

/** @brief AES-GCM authentication tag length in bytes (always 128-bit per spec). */
#define ACS_GCM_MAC_SIZE 16

/** @brief GMAC nonce/IV constants — identical to GCM (GMAC is GCM with zero-length plaintext). */
#define ACS_GMAC_NONCE_FIXED_SIZE ACS_GCM_NONCE_FIXED_SIZE
#define ACS_GMAC_NONCE_VAR_SIZE   ACS_GCM_NONCE_VAR_SIZE
#define ACS_GMAC_NONCE_SIZE       ACS_GCM_NONCE_SIZE
#define ACS_GMAC_MAC_SIZE         ACS_GCM_MAC_SIZE

/**
 * @brief Total nonce length in bytes for the active cipher.
 */
#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                              \
	defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
#define ACS_ACTIVE_NONCE_SIZE ACS_GCM_NONCE_SIZE /**< 12 bytes */
#else
#define ACS_ACTIVE_NONCE_SIZE ACS_NONCE_SIZE /**< 13 bytes (CCM) */
#endif

/**
 * @brief On-wire nonce variable-part length in bytes for the active cipher.
 */
#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                              \
	defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
#define ACS_ACTIVE_NONCE_VAR_SIZE ACS_GCM_NONCE_VAR_SIZE
#elif defined(CONFIG_BT_ACS_CCM_NONCE_SEQ_DIFF_FIXED) ||                                           \
	defined(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD)
#define ACS_ACTIVE_NONCE_VAR_SIZE ACS_CCM_NONCE_VAR_SIZE
#else
#define ACS_ACTIVE_NONCE_VAR_SIZE 0
#endif

/**
 * @brief Authentication tag length in bytes for the active cipher.
 *
 * GCM always uses a 128-bit (16-byte) tag. CCM tag length is configurable
 * via CONFIG_BT_ACS_CCM_MAC_SIZE (any even value from 4 to 16).
 */
#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
#define ACS_ACTIVE_AUTH_TAG_SIZE ACS_GCM_MAC_SIZE
#else
#define ACS_ACTIVE_AUTH_TAG_SIZE ACS_CCM_MAC_SIZE
#endif

/**
 * @brief Headroom reserved in protected response buffers for in-place encryption.
 *
 * ISC_ID (2 bytes) + Nonce_Var (variable-part nonce bytes).
 * The auth tag is appended after ciphertext — needs tailroom,
 * already accounted for in ACS_BUF_SIZE.
 */
#define ACS_CRYPTO_HEADROOM (2U + ACS_ACTIVE_NONCE_VAR_SIZE)

/*
 * PSA algorithm and key-usage constants are now defined locally in acs_crypto.c
 * (per-algorithm: ACS_PSA_CCM_ALG, ACS_PSA_GCM_ALG, etc.) to support
 * multi-algorithm dispatch.  The legacy single-algorithm aliases
 * (ACS_PSA_AEAD_ALG, ACS_PSA_TAG_LEN) live there too for buffer sizing.
 */

#ifdef __cplusplus

#endif

#endif /* BT_GATT_ACS_CRYPTO_CONFIG_H_ */
