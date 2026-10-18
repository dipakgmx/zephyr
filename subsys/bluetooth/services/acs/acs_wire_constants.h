/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_WIRE_CONSTANTS_H_
#define BT_GATT_ACS_WIRE_CONSTANTS_H_

#include <stdint.h>
#include <zephyr/toolchain.h>
#include <zephyr/bluetooth/att.h>

#include "acs_crypto_config.h"
#include "acs_seg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Descriptor general schema (Table 4.4). */
struct acs_desc_rec_hdr {
	uint8_t type_id;     /* Record type, per the owning descriptor's table */
	uint16_t type_value; /* ISC_ID, Key_ID, or Restriction_Map_ID / Resource_Handle (LE) */
	uint8_t data_size;   /* Byte length of the Data field that follows; 0 if absent */
} __packed;

/* Information_Security_Configuration_ID size (Table 4.9). */
#define ACS_DATA_IN_HDR_SIZE 2

/* Protected_Resource_Handle size (Table 4.10). */
#define ACS_RESOURCE_HANDLE_SIZE 2

/*
 * Largest application message: an ACS Control Point opcode with its operands,
 * or a protected characteristic value. ACS framing and security fields are not
 * counted.
 */
#define ACS_MESSAGE_MAX_SIZE CONFIG_BT_ACS_MAX_MESSAGE_SIZE

/* Largest operand of one message, after its one-octet opcode. */
#define ACS_MESSAGE_MAX_OPERAND (ACS_MESSAGE_MAX_SIZE - 1U)

/*
 * Fields a protected message adds around the application message (§4.3.2):
 * ISC_ID, Nonce_Var and Protected_Resource_Handle before it, the MAC after it,
 * each at its largest among the enabled algorithms.
 */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
#define ACS_PROTECTED_HDR_SIZE                                                                     \
	(ACS_DATA_IN_HDR_SIZE + ACS_MAX_NONCE_VAR_SIZE + ACS_RESOURCE_HANDLE_SIZE)
#define ACS_PROTECTED_TAG_SIZE ACS_MAX_AUTH_TAG_SIZE
#else
#define ACS_PROTECTED_HDR_SIZE 0U
#define ACS_PROTECTED_TAG_SIZE 0U
#endif

/* Room each message buffer reserves in front of the application message. */
#define ACS_BUF_HEADROOM (ACS_SEG_HDR_SIZE + ACS_PROTECTED_HDR_SIZE)

/*
 * One buffer of the shared RX and TX pool: the largest ACS characteristic
 * value, Segmentation_Header through MAC (§4.1.1).
 */
#define ACS_BUF_SIZE (ACS_BUF_HEADROOM + ACS_MESSAGE_MAX_SIZE + ACS_PROTECTED_TAG_SIZE)

/* An ACS characteristic value is an ATT attribute value (Core Vol 3 Part F §3.2.9). */
BUILD_ASSERT(ACS_BUF_SIZE <= BT_ATT_MAX_ATTRIBUTE_LEN,
	     "CONFIG_BT_ACS_MAX_MESSAGE_SIZE and the ACS framing exceed 512 octets");

/*
 * Longest reassembled payload on either channel: a Data In payload, ISC_ID
 * through the application message. A plain ACS CP message is checked against
 * ACS_MESSAGE_MAX_SIZE once reassembled.
 */
#define ACS_RX_PAYLOAD_MAX_SIZE (ACS_BUF_SIZE - ACS_SEG_HDR_SIZE)

/* Buffers per connection - 4 for Get All Active Descriptors (3 descriptor responses which are
 * prepared for dispatch and the succeeding response code), 1 for an expected abort write that can
 * arrive while the 4 are sent, and 1 for the next procedure.
 */
#define ACS_BUF_PER_CONN 6

/* The pool takes ACS_BUF_COUNT x ACS_BUF_SIZE octets of RAM, plus a net_buf header per buffer. */
#define ACS_BUF_COUNT (CONFIG_BT_MAX_CONN * ACS_BUF_PER_CONN)

/* Status characteristic value: Status_Flags[1 byte] + Current Restriction_Map_ID [2 byte]
 * (§4.2 Table 4.6)
 */
#define ACS_STATUS_SIZE 3

/* Fixed-size fields of the KDF Response operand (Table 4.76). */
#define ACS_KDF_RSP_KEY_ID_SIZE   2
#define ACS_KDF_RSP_SALT_SZ_FIELD 1
#define ACS_KDF_RSP_INFO_SZ_FIELD 1
#define ACS_KDF_RSP_FIXED_SIZE                                                                     \
	(ACS_KDF_RSP_KEY_ID_SIZE + ACS_KDF_RSP_SALT_SZ_FIELD + ACS_KDF_RSP_INFO_SZ_FIELD)

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
