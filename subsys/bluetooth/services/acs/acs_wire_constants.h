/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_WIRE_CONSTANTS_H_
#define BT_GATT_ACS_WIRE_CONSTANTS_H_

#include <stdint.h>
#include <zephyr/toolchain.h>

#include "acs_crypto_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Descriptor general schema (Table 4.4). */
struct acs_desc_rec_hdr {
	uint8_t type_id;     /* Record type, per the owning descriptor's table */
	uint16_t type_value; /* ISC_ID, Key_ID, or Restriction_Map_ID / Resource_Handle (LE) */
	uint8_t data_size;   /* Byte length of the Data field that follows; 0 if absent */
} __packed;

/* Buffer size for one protected segment. */
#define ACS_BUF_SIZE                                                                               \
	(CONFIG_BT_ACS_MAX_SEGMENT_SIZE + ACS_MAX_AUTH_TAG_SIZE + ACS_MAX_NONCE_VAR_SIZE + 1U)

/* Buffers needed for each connection and in-flight request. */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
#define ACS_BUF_PER_CONN (3 + (2 * CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN))
#else
#define ACS_BUF_PER_CONN 2
#endif

#define ACS_BUF_COUNT (CONFIG_BT_MAX_CONN * ACS_BUF_PER_CONN)

/* Status characteristic value: 1 octet flags + 2 octets Restriction_Map_ID. */
#define ACS_STATUS_SIZE 3

/* Fixed-size fields of the KDF Response operand (Table 4.76). */
#define ACS_KDF_RSP_KEY_ID_SIZE   2
#define ACS_KDF_RSP_SALT_SZ_FIELD 1
#define ACS_KDF_RSP_INFO_SZ_FIELD 1
#define ACS_KDF_RSP_FIXED_SIZE                                                                     \
	(ACS_KDF_RSP_KEY_ID_SIZE + ACS_KDF_RSP_SALT_SZ_FIELD + ACS_KDF_RSP_INFO_SZ_FIELD)

/* Information_Security_Configuration_ID size (Table 4.9). */
#define ACS_DATA_IN_HDR_SIZE 2

/* Fixed-size fields of an ISC record (Table 4.32). */
#define ACS_ISC_NUM_CTRL_FIELD_SIZE 1
#define ACS_ISC_KEY_ID_FIELD_SIZE   2

/* ATT application error codes defined by this service (Table 1.2). */
#define BT_ACS_ATT_ERR_INVALID_KEY               0x80
#define BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED    0x81
#define BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG 0x82
#define BT_ACS_ATT_ERR_INVALID_SEG_COUNTER       0x83

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_WIRE_CONSTANTS_H_ */
