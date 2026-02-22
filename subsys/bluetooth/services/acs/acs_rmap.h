/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_RMAP_H_
#define BT_GATT_ACS_RMAP_H_

#include <zephyr/types.h>
#include <zephyr/net_buf.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_wire_constants.h"

/* Restriction Map record Type_ID values (Table 4.19). */
#define ACS_RMAP_TYPE_ID             0x00 /* Restriction_Map_ID record */
#define ACS_RMAP_TYPE_DEFAULT_ISC    0x01 /* Default_ISC_ID record */
#define ACS_RMAP_TYPE_PROTECTED_CHAR 0x02 /* Protected Characteristic record */
#define ACS_RMAP_TYPE_PROTECTED_CP   0x03 /* Protected Control Point record */

/* Resource_Handle_Filter value selecting every resource (Table 4.17). */
#define ACS_RMAP_FILTER_ALL 0xFFFF

/* Characteristic operation checked by the local access policy. */
enum acs_direction {
	ACS_DIRECTION_READ,
	ACS_DIRECTION_WRITE,
	ACS_DIRECTION_NOTIFY,
	ACS_DIRECTION_INDICATE,
};

/* Get Restriction Map Descriptor operand (Table 4.17 / §4.4.4.3). */
struct acs_rmap_get_descriptor_req {
	uint16_t map_id;                 /* Restriction_Map_ID */
	uint16_t resource_handle_filter; /* Resource_Handle_Filter */
} __packed;

/* One ATT opcode -> ISC_ID entry in a Protected record (Table 4.20). */
struct acs_rmap_data_entry {
	uint16_t opcode; /* ATT opcode, little-endian */
	uint16_t isc_id; /* ISC_ID, little-endian */
} __packed;

/* Restriction Map ID List response entry (Table 4.22). */
struct acs_rmap_id_list_entry {
	uint16_t map_id; /* Restriction_Map_ID, little-endian */
	uint16_t isc_id; /* Information_Security_Configuration_ID, little-endian */
} __packed;

/* Find a resolved restriction map by Restriction_Map_ID, or return NULL. */
const struct bt_acs_rmap_runtime *acs_rmap_lookup(uint16_t map_id);

/* Find a protected-resource entry by ACS Resource Handle. */
int acs_rmap_find_entry_by_resource_handle(const struct bt_acs_rmap_runtime *runtime,
					   uint16_t resource_handle,
					   const struct bt_acs_rmap_entry **entry);

/* Return true when an attribute operation must use ACS protected transport. */
bool acs_rmap_attr_requires_protected_transport(const struct bt_acs_rmap_runtime *runtime,
						uint16_t attr_handle, enum acs_direction direction);

/* Return true when entry binds opcode to isc_id. */
bool acs_rmap_entry_matches(const struct bt_acs_rmap_entry *entry, uint16_t opcode,
			    uint16_t isc_id);

/* Return true when a plain write must use ACS protected transport. */
bool acs_rmap_write_requires_protected_transport(const struct bt_acs_rmap_runtime *runtime,
						 uint16_t attr_handle, bool opcode_valid,
						 uint8_t opcode);

/* Return the ISC_ID for a protected Control Point opcode, or NONE. */
uint16_t acs_rmap_cp_opcode_isc(const struct bt_acs_rmap_runtime *runtime, uint16_t cp_handle,
				uint8_t opcode);

/* Return Protected Resource Uses feature bits for the registered resources. */
uint32_t acs_rmap_protected_resource_feature_bits(void);

/* Return the Multiple Restriction Maps Supported state (Table 4.60 bit 2). */
bool acs_rmap_multiple_supported(void);

/* Validate registered maps and build their runtime handle indexes. */
int acs_rmap_init_runtime(void);

/* Append the Restriction Map Descriptor records from the already-resolved map. */
int acs_rmap_build_descriptor_response(const struct bt_acs_rmap_runtime *runtime,
				       const struct acs_rmap_get_descriptor_req *req,
				       struct net_buf *buf);

/* Return whether any active descriptor is protected and its common ISC_ID. */
bool acs_rmap_descriptor_protection(const struct bt_acs_rmap_runtime *active_rmap,
				    uint16_t *isc_id);

#endif /* BT_GATT_ACS_RMAP_H_ */
