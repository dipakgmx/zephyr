/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_RMAP_H_
#define BT_GATT_ACS_RMAP_H_

#include <zephyr/types.h>
#include <zephyr/net_buf.h>

/* Record Type_ID values (Table 4.19) */
#define ACS_RMAP_TYPE_ID             0x00 /**< Restriction_Map_ID record */
#define ACS_RMAP_TYPE_DEFAULT_ISC    0x01 /**< Default_ISC_ID record */
#define ACS_RMAP_TYPE_PROTECTED_CHAR 0x02 /**< Protected Characteristic record */
#define ACS_RMAP_TYPE_PROTECTED_CP   0x03 /**< Protected Control Point record */

/** Resource accessed via their own service (§3.5.3) */
#define BT_ACS_RMAP_ID_NONE 0x0000U /**< Resource accessed via their own service */

/** Special handle_filter value: report the full map (all resource handles). */
#define ACS_RMAP_FILTER_ALL 0xFFFF

/**
 * @brief Operand for the Get Restriction Map Descriptor opcode (Table 4.17 / §4.4.4.3).
 */
struct acs_rmap_get_descriptor_req {
	uint16_t map_id;                 /**< Restriction_Map_ID */
	uint16_t resource_handle_filter; /**< Resource_Handle_Filter */
} __packed;

/**
 * @brief Wire header for every restriction map record (Table 4.19 / §4.1.1.2).
 *
 *   type_id    - record type (0x00–0x03)
 *   type_value - Restriction_Map_ID, ISC_ID, or Resource_Handle (LE)
 *   data_size  - byte count of the Data field; 0 if absent
 */
struct acs_rmap_rec_hdr {
	uint8_t type_id;
	uint16_t type_value; /**< little-endian */
	uint8_t data_size;
} __packed;

/**
 * @brief One entry in the Data field of a Protected record (Table 4.20).
 *
 * Each entry maps one ATT opcode to an ISC_ID.
 */
struct acs_rmap_data_entry {
	uint16_t opcode; /**< ATT opcode, little-endian */
	uint16_t isc_id; /**< ISC_ID, little-endian */
} __packed;

/**
 * @brief Entry in the Restriction Map ID List response (Table 4.22).
 */
struct acs_rmap_id_list_entry {
	uint16_t map_id; /**< Restriction_Map_ID, little-endian */
	uint16_t isc_id; /**< Information_Security_Configuration_ID, little-endian */
} __packed;

/** @brief Kind of protected resource found in a restriction map. */
enum acs_rmap_resource_kind {
	ACS_RMAP_RESOURCE_NONE,
	ACS_RMAP_RESOURCE_CHAR,
	ACS_RMAP_RESOURCE_CP,
};

/** @brief Protected CCCD gate resolved from a protected characteristic entry. */
struct acs_rmap_cccd_gate {
	uint16_t cccd_handle;
	uint16_t char_handle;
	bool notify_protected;
	bool indicate_protected;
};

/** @brief Look up a restriction map by ID. */
int acs_rmap_lookup(uint16_t map_id, struct bt_acs_restriction_map *out);

/** @brief Find a protected resource entry in a restriction map by handle. */
int acs_rmap_find_protected(uint16_t map_id, uint16_t resource_handle,
			    enum acs_rmap_resource_kind *kind,
			    const struct bt_acs_rmap_protected **entry);

/** @brief Check whether a characteristic operation requires ACS security. */
bool acs_rmap_char_is_protected(uint16_t map_id, uint16_t att_handle,
				enum bt_acs_direction direction);

/** @brief Check whether a Control Point opcode requires ACS security. */
bool acs_rmap_cp_opcode_is_protected(uint16_t map_id, uint16_t cp_handle, uint8_t opcode);

/**
 * @brief Collect protected CCCD gates derived from registered protected chars.
 *
 * Walks all registered protected characteristic entries, derives CCCD gates
 * for notification/indication-protected characteristics, and writes up to
 * @p capacity entries into @p gates.
 *
 * Duplicate CCCD handles are merged. If more gates are discovered than fit in
 * @p capacity, excess gates are skipped.
 *
 * @param gates      Output array of CCCD gate entries.
 * @param capacity   Number of elements available in @p gates.
 * @param count      Filled with the number of gate entries written.
 */
void acs_rmap_collect_protected_cccds(struct acs_rmap_cccd_gate *gates, size_t capacity,
				      size_t *count);

/**
 * @brief Build the Restriction Map Descriptor Response payload.
 *
 * @details function invokes the application-provided restriction_map_get callback to retrieve
 * the relevant restriction map records. It then serializes the matching TLV (Type-Length-Value)
 * records into the provided buffer @p buf. The buffer must already contain the opcode byte.
 *
 * @param req         Pointer to the parsed request operand structure containing the map ID and
 * handle filter.
 * @param buf         Buffer to write the serialized TLV records into (opcode byte already present).
 *
 * @retval 0        Success; response payload written to @p buf.
 * @retval -ENOENT  No matching restriction map found, or no record matched the handle filter.
 * @retval -ENOMEM  Output buffer @p buf is too small to hold the response.
 */
int acs_rmap_build_descriptor_response(const struct acs_rmap_get_descriptor_req *req,
				       struct net_buf_simple *buf);

/**
 * @brief Build the Restriction Map ID List Response payload.
 *
 * Iterates restriction map IDs 1–10 via the application callback and serialises
 * each found map as an acs_rmap_id_list_entry into @p buf.
 *
 * @param buf Buffer to write entries into (opcode byte already present).
 *
 * @return 0 on success
 * @return -ENOMEM if the output buffer is too small
 */
int acs_rmap_build_id_list_response(struct net_buf_simple *buf);

/**
 * @brief Resolve ATT handles for all bt_acs_rmap_char_reg iterable section entries.
 *
 * Called internally by bt_acs_init() after bt_enable() so that the GATT
 * attribute table is fully built before handle lookup.
 *
 * @return 0 if all handles resolved, -ENOENT if any UUID was not found.
 */
int acs_rmap_resolve_handles(void);

#endif /* BT_GATT_ACS_RMAP_H_ */
