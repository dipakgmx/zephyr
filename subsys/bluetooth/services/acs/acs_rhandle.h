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

/** @brief Build the Resource Handle UUID Map response payload (Table 4.25). */
int acs_rhandle_build_map_response(struct net_buf_simple *buf);

/** @brief Look up the service and characteristic UUIDs for a resource handle. */
int acs_rhandle_lookup_svc_char(uint16_t resource_handle, struct net_buf_simple *buf);

/** @brief Find the ATT value handle for a characteristic identified by UUID. */
uint16_t acs_rhandle_find_char_handle(const struct bt_uuid *char_uuid);

/** @brief Find the CCCD handle for a characteristic given its value handle. */
uint16_t acs_rhandle_find_cccd_for_char(uint16_t char_value_handle);

struct acs_char_attr_ctx {
	uint16_t value_handle;
	const struct bt_gatt_attr *decl;
	const struct bt_gatt_attr *value;
};

uint8_t acs_find_char_attrs_cb(const struct bt_gatt_attr *attr, uint16_t handle, void *user_data);

#endif /* BT_GATT_ACS_RHANDLE_H_ */
