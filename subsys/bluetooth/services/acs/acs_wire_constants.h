/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file acs_wire_constants.h
 * @brief ACS wire-layout sizing macros, pool constants, and ATT error codes.
 *
 * Pure preprocessor constants with no struct dependencies.
 * Included transitively via acs_internal.h — not intended for direct inclusion.
 */

#ifndef BT_GATT_ACS_WIRE_CONSTANTS_H_
#define BT_GATT_ACS_WIRE_CONSTANTS_H_

#include "acs_crypto_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ACS spec wire format size: Key_ID(2) + X_Size(1) + X(N) + Y_Size(1) + Y(N or 0)
 */
#define ACS_ECDH_PUBKEY_MAX_SIZE                                                                   \
	(4 + CONFIG_BT_ACS_ECDH_COORD_SIZE +                                                       \
	 (CONFIG_BT_ACS_ECDH_HAS_Y * CONFIG_BT_ACS_ECDH_COORD_SIZE))

/** @brief TX buffer size for per-connection indication / notification contexts. */
#define ACS_DATA_IND_BUF_SIZE                                                                      \
	(CONFIG_BT_ACS_MAX_SEGMENT_SIZE + ACS_CRYPTO_AUTH_TAG_SIZE + ACS_ACTIVE_NONCE_VAR_SIZE)

/**
 * @brief Shared buffer pool buffer size.
 *
 * Sized to hold the maximum segment plus crypto overhead (auth tag + nonce variable).
 */
#define ACS_BUF_SIZE ACS_DATA_IND_BUF_SIZE

/**
 * @brief Number of buffers in the shared pool.
 *
 * Per concurrent connection:
 *   - 1 for plain CP RX reassembly
 *   - 1 for Data In RX reassembly
 *   - 1 for plain CP / DOI wire TX
 *   - 2 per inflight protected request slot (plaintext input + plaintext response)
 */
#define ACS_BUF_COUNT (CONFIG_BT_MAX_CONN * (3 + (2 * CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN)))

/** @brief ACS Status characteristic size: 1 byte flags + 2 bytes map_id */
#define ACS_STATUS_SIZE 3

/** @brief Generic CP Response Code packet layout */
#define ACS_CP_RSP_HDR_SIZE    3 /**< Opcode(1) + ReqOpcode(1) + RspCode(1) */
#define ACS_CP_RSP_OPCODE_SIZE 1 /**< Response opcode-only prefix */

/** @brief KDF response wire layout fixed-size fields */
#define ACS_KDF_RSP_KEY_ID_SIZE   2
#define ACS_KDF_RSP_SALT_SZ_FIELD 1
#define ACS_KDF_RSP_INFO_SZ_FIELD 1
#define ACS_KDF_RSP_FIXED_SIZE                                                                     \
	(ACS_KDF_RSP_KEY_ID_SIZE + ACS_KDF_RSP_SALT_SZ_FIELD + ACS_KDF_RSP_INFO_SZ_FIELD)

#define ACS_DATA_IN_HDR_SIZE 2 /**< ISC_ID field at front of ACS Data payload */

/** @brief ISC record wire layout field sizes */
#define ACS_ISC_NUM_CTRL_FIELD_SIZE 1
#define ACS_ISC_KEY_ID_FIELD_SIZE   2

/**
 * @brief ACS-specific ATT error: Invalid Key (Section 3.6, ATT Application error 0x80)
 */
#define BT_ACS_ATT_ERR_INVALID_KEY 0x80

/**
 * @brief ACS-specific ATT error: Resource Not Protected (Table 1.2, 0x81)
 */
#define BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED 0x81

/**
 * @brief ACS-specific ATT error: Incorrect Security Configuration (Table 1.2, 0x82)
 */
#define BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG 0x82

/**
 * @brief ACS-specific ATT error: Invalid Rolling Segment Counter (Section 4.2)
 */
#define BT_ACS_ATT_ERR_INVALID_SEG_COUNTER 0x83

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_WIRE_CONSTANTS_H_ */
