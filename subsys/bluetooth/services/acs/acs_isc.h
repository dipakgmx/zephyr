/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_ISC_H_
#define BT_GATT_ACS_ISC_H_

#include <zephyr/types.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/iterable_sections.h>

/* Spec-Defined Constants (Tables 4.31 & 4.33) */
#define BT_ACS_RECORD_TYPE_ISC_ID 0x00

/**
 * Reserved ISC ID: no information security controls required (spec §4.4.x).
 * Resources mapped to this ID are accessed directly without security controls.
 * Used only as a reference in the restriction map descriptor; shall never
 * appear in an ISC descriptor record.
 */
#define BT_ACS_ISC_UNPROTECTED_ID 0x0000

/**
 * Reserved ISC ID used as a filter to request all ISC records (Table 4.31).
 * Shall never appear in an ISC descriptor record.
 */
#define BT_ACS_ISC_ALL_RECORDS_FILTER 0xFFFF

/* Named ISC record IDs (assigned by AC Server; registered via BT_ACS_ISC_DEFINE()) */
#define ACS_ISC_ID_HIGH_SEC  0x0001 /**< Confidentiality + integrity + auth (GCM/CCM) */
#define ACS_ISC_ID_AUTH      0x0002 /**< Auth-only: Nonce + MAC (ECDH required) */
#define ACS_ISC_ID_UNENC     0x0003 /**< Unencrypted fallback (always present) */
#define ACS_ISC_ID_INTEGRITY 0x0004 /**< Integrity + auth, no confidentiality (GMAC) */
#define ACS_ISC_ID_MAC_ONLY  0x0005 /**< MAC-only authentication, no nonce (CMAC) */

/**
 * @brief ISC descriptor record header (Table 4.5 / §4.1.2).
 *
 * Maps the generic descriptor record layout (Table 4.5) for ISC records.
 * The Type_ID values for ISC records are defined in Table 4.31;
 * the Data field that follows is structured per Table 4.32.
 */
struct acs_isc_rec_hdr {
	/** Record type (Table 4.31); always @ref BT_ACS_RECORD_TYPE_ISC_ID (0x00) for ISC records.
	 */
	uint8_t type_id;
	/** Type_Value field (Table 4.5, little-endian); for ISC records, carries the ISC ID
	 * assigned by the AC Server. */
	uint16_t type_value;
	/** Byte length of the Data field that follows: num_controls + controls[] + optional Key_ID
	 * (Table 4.32). */
	uint8_t data_size;
} __packed;

/**
 * @brief Fixed header of the ISC Data field (Table 4.32).
 *
 * Immediately followed by Information_Security_Controls[num_controls]
 * and optionally Key_ID(2 LE) if controls require a key.
 */
struct acs_isc_data_hdr {
	uint8_t num_controls; /**< Number_Of_Information_Security_Controls */
} __packed;

/**
 * @enum acs_sec_control_type
 * @brief Information_Security_Controls field values (Table 4.33).
 *
 */
enum acs_sec_control_type {
	/** Nonce */
	ACS_CTRL_NONCE = 0x00,
	/** Authenticated Protected Resource Request Or Response */
	ACS_CTRL_AUTH = 0x01,
	/** Encrypted Protected Resource Request Or Response */
	ACS_CTRL_ENC = 0x02,
	/**< Authenticated And Encrypted Protected Resource Request Or Response */
	ACS_CTRL_AUTH_ENC = 0x03,
	/**< Authenticated And Encrypted Protected Resource Request Or Response With Associated Data
	 */
	ACS_CTRL_AUTH_ENC_AD = 0x04,
	/** Unencrypted Protected Resource Request Or Response */
	ACS_CTRL_UNENC = 0x05,
	/** MAC */
	ACS_CTRL_MAC = 0x06
};

/**
 * @brief Information Security Configuration record (Table 4.32).
 *
 * Each ISC record describes a set of security controls and an associated key.
 * Registered via BT_ACS_ISC_DEFINE(); discovered at runtime via the
 * bt_acs_isc_record iterable section.
 */
struct bt_acs_isc_record {
	uint16_t isc_id;
	uint8_t num_controls;
	uint8_t controls[CONFIG_BT_ACS_ISC_MAX_CONTROLS];
	uint16_t key_id;
};

#define BT_ACS_ISC_DEFINE(_name, ...)                                                              \
	STRUCT_SECTION_ITERABLE(bt_acs_isc_record, _name) = {__VA_ARGS__}

/** @brief Look up an ISC record by ISC ID. */
const struct bt_acs_isc_record *acs_isc_lookup(uint16_t isc_id);

/**
 * @brief Parses the filter request and builds the ISC Descriptor Response payload.
 * @param operand      The request operand buffer (filter pulled via net_buf_simple).
 * @param buf          Buffer to store the generated response TLV records.
 * @return 0 on success, negative error code on failure.
 * @return -EINVAL if the operand format is invalid
 * @return -ENOENT if no matching records were found
 * @return -ENOMEM if the output buffer is too small
 */
int acs_isc_build_response(struct net_buf_simple *operand, struct net_buf_simple *buf);

#endif /* BT_GATT_ACS_ISC_H_ */
