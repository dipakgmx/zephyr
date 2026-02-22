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

#include "acs_wire_constants.h"
#include "acs_types.h"

/* ISC record Type_ID (Table 4.31). */
#define BT_ACS_RECORD_TYPE_ISC_ID 0x00

/*
 * Reserved ISC ID used as a filter to request all ISC records (Table 4.31).
 * This value does not appear in an ISC descriptor record.
 */
#define BT_ACS_ISC_ALL_RECORDS_FILTER 0xFFFF

/*
 * Information_Security_Controls field values (Table 4.33). Each names how the
 * Protected Resource Request Or Response is protected.
 */
enum acs_sec_control_type {
	ACS_CTRL_NONCE = 0x00,       /* Nonce */
	ACS_CTRL_AUTH = 0x01,        /* Authenticated */
	ACS_CTRL_ENC = 0x02,         /* Encrypted */
	ACS_CTRL_AUTH_ENC = 0x03,    /* Authenticated and encrypted */
	ACS_CTRL_AUTH_ENC_AD = 0x04, /* Authenticated and encrypted, associated data */
	ACS_CTRL_UNENC = 0x05,       /* Unencrypted */
	ACS_CTRL_MAC = 0x06          /* MAC */
};

/*
 * Information Security Configuration record (Table 4.32): a set of security
 * controls and the key they use.
 */
struct bt_acs_isc_record {
	uint16_t isc_id;
	uint8_t num_controls;
	uint8_t controls[CONFIG_BT_ACS_ISC_MAX_CONTROLS];
	uint16_t key_id;
};

#define BT_ACS_ISC_DEFINE(_name, ...)                                                              \
	STRUCT_SECTION_ITERABLE(bt_acs_isc_record, _name) = {__VA_ARGS__}

/* Find an ISC record by ISC_ID, or return NULL when it is not registered. */
const struct bt_acs_isc_record *acs_isc_lookup(uint16_t isc_id);

/* Validate ISC IDs and the number of controls in every registered record. */
int acs_isc_validate_records(void);

/* Build the ISC Descriptor Response records selected by operand. */
int acs_isc_build_response(struct net_buf_simple *operand, struct net_buf *buf);

/* Resolve an ISC to its key slot. Return -ENOENT if either record is missing. */
int acs_resolve_isc_slot(struct bt_acs_conn *acs_conn, uint16_t isc_id,
			 struct bt_acs_key_desc_runtime **key_runtime);

/* Resolve an ISC key. Return -EACCES if its slot has no installed key. */
int acs_resolve_isc_key(struct bt_acs_conn *acs_conn, uint16_t isc_id,
			struct bt_acs_key_desc_runtime **key_runtime);

#endif /* BT_GATT_ACS_ISC_H_ */
