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
 *   type_id    — record type (0x00–0x03)
 *   type_value — Restriction_Map_ID, ISC_ID, or Resource_Handle (LE)
 *   data_size  — byte count of the Data field; 0 if absent
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

/**
 * @brief Callback type for iterating over protected resource entries in a restriction map.
 *
 * @param prot  The protected resource entry.
 * @param user_data  User-supplied context pointer.
 * @return true to continue iteration, false to stop.
 */
typedef bool (*bt_acs_rmap_protected_cb_t)(const struct bt_acs_rmap_protected *prot,
					   void *user_data);

/**
 * @brief Look up a restriction map by ID.
 *
 * Scans the bt_acs_restriction_map iterable section for a map whose
 * @c map_id equals @p map_id and copies it to @p out.
 *
 * @param map_id  Restriction_Map_ID to search for.
 * @param out     Destination for the found map (must not be NULL).
 *
 * @retval 0        Found; @p out has been populated.
 * @retval -ENOENT  No map with @p map_id is registered.
 */
int acs_rmap_lookup(uint16_t map_id, struct bt_acs_restriction_map *out);

/**
 * @brief Iterate the protected characteristics of a restriction map.
 *
 * Calls @p cb for each entry in @p map->chars[]. Stops early if @p cb
 * returns @c false.
 *
 * @param map       Restriction map to iterate (must not be NULL).
 * @param cb        Callback invoked per entry; return @c false to stop.
 * @param user_data Opaque pointer forwarded to every @p cb invocation.
 */
void acs_rmap_foreach_char(const struct bt_acs_restriction_map *map,
			   bool (*cb)(const struct bt_acs_rmap_protected *prot, void *user_data),
			   void *user_data);

/**
 * @brief Iterate the protected Control Point entries of a restriction map.
 *
 * Calls @p cb for each entry in @p map->cps[] and for iterable section
 * entries registered with BT_ACS_RMAP_PROTECT_CP_IN_MAP().  Stops early
 * if @p cb returns @c false.
 *
 * @param map        Restriction map to iterate (must not be NULL).
 * @param cb         Callback invoked per entry; return @c false to stop.
 * @param user_data  User-supplied context pointer passed to @p cb.
 */
void acs_rmap_foreach_cp(const struct bt_acs_restriction_map *map, bt_acs_rmap_protected_cb_t cb,
			 void *user_data);

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
