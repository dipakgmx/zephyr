/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_RHANDLE_H_
#define BT_GATT_ACS_RHANDLE_H_

#include <zephyr/types.h>
#include <zephyr/net_buf.h>
#include <zephyr/bluetooth/uuid.h>

/* Attribute_Type values (Table 4.26) */
#define ACS_RHANDLE_ATTR_PRIMARY_SVC   0x00
#define ACS_RHANDLE_ATTR_SECONDARY_SVC 0x01
#define ACS_RHANDLE_ATTR_CHAR_VALUE    0x02

/**
 * @brief Build the Resource Handle UUID Map response payload.
 *
 * Iterates the full GATT database and serialises every primary service and its
 * characteristic values into @p buf using the Table 4.25 wire format.
 *
 * @param buf Buffer to write records into (response opcode byte already present).
 *
 * @return 0 on success
 * @return -ENOMEM if the output buffer is too small
 */
int acs_rhandle_build_map_response(struct net_buf_simple *buf);

/**
 * @brief Look up the service and characteristic UUIDs for a resource handle.
 *
 * Iterates the GATT database to find @p resource_handle, then serialises the
 * owning service UUID and the characteristic UUID as:
 *   UUID_Size(1) + Service_UUID(var) + UUID_Size(1) + Char_UUID(var)
 *
 * @param resource_handle ATT handle to look up.
 * @param buf             Buffer to write the result into (response opcode already present).
 *
 * @return 0 on success
 * @return -ENOENT if the handle is not found in the GATT database
 * @return -ENOMEM if the output buffer is too small
 */
int acs_rhandle_lookup_svc_char(uint16_t resource_handle, struct net_buf_simple *buf);

/**
 * @brief Find the ATT value handle for a characteristic identified by UUID.
 *
 * Iterates the full GATT database and returns the handle of the characteristic
 * value attribute whose declaration UUID matches @p char_uuid.
 *
 * @param char_uuid UUID of the characteristic to search for.
 *
 * @return ATT handle (> 0) on success
 * @return 0 if the characteristic was not found
 */
uint16_t acs_rhandle_find_char_handle(const struct bt_uuid *char_uuid);

/**
 * @brief Find the CCCD handle for a characteristic given its value handle.
 *
 * Scans the GATT attributes immediately following @p char_value_handle for a
 * Client Characteristic Configuration Descriptor (UUID 0x2902).  The scan
 * stops as soon as a new service or characteristic declaration is encountered.
 *
 * @param char_value_handle ATT handle of the characteristic value attribute.
 *
 * @return ATT handle of the CCCD (> 0) if found
 * @return 0 if no CCCD is present for this characteristic
 */
uint16_t acs_rhandle_find_cccd_for_char(uint16_t char_value_handle);

#endif /* BT_GATT_ACS_RHANDLE_H_ */
