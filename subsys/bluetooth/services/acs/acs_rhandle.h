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

struct bt_gatt_attr;

/* Attribute_Type values (Table 4.26). */
#define ACS_RHANDLE_ATTR_PRIMARY_SVC   0x00
#define ACS_RHANDLE_ATTR_SECONDARY_SVC 0x01
#define ACS_RHANDLE_ATTR_CHAR_VALUE    0x02

/* One numbered resource visited during a GATT-table enumeration. */
struct acs_rhandle_resource {
	uint8_t attr_type;               /* Attribute_Type, Table 4.26 */
	uint16_t resource_handle;        /* AC Server-assigned Resource Handle */
	uint16_t attr_handle;            /* GATT Attribute Handle of this attribute */
	const struct bt_gatt_attr *attr; /* GATT attribute backing this resource */
	uint8_t props;                   /* Characteristic properties, 0 for a service */
	const struct bt_uuid *uuid;      /* This resource's own UUID */
	const struct bt_uuid *svc_uuid;  /* UUID of the enclosing service */
};

/* Visitor for one numbered resource. */
typedef uint8_t (*acs_rhandle_visit_t)(const struct acs_rhandle_resource *res, void *user_data);

/* Visit service declarations and characteristic values in Resource Handle order. */
void acs_rhandle_walk(acs_rhandle_visit_t visit, void *user_data);

/* Build the Resource Handle UUID Map response payload (Table 4.25). */
int acs_rhandle_build_map_response(struct net_buf *buf);

/* Append the service and characteristic UUIDs for a Resource Handle to buf. */
int acs_rhandle_lookup_svc_char(uint16_t resource_handle, struct net_buf *buf);

/* Find the GATT Attribute Handle assigned to a Resource Handle. */
uint16_t acs_rhandle_find_attr_handle(uint16_t resource_handle);

/* Find the Resource Handle assigned to a GATT Attribute Handle. */
uint16_t acs_rhandle_find_resource_handle(uint16_t attr_handle);

/* Find the Resource and GATT Attribute Handles for a characteristic UUID. */
int acs_rhandle_find_char_attr_handles(const struct bt_uuid *char_uuid, uint16_t *resource_handle,
				       uint16_t *attr_handle);

/* Resolve a characteristic value and properties. Either output may be NULL. */
int acs_resolve_char(uint16_t value_handle, const struct bt_gatt_attr **value_out,
		     uint8_t *props_out);

#endif /* BT_GATT_ACS_RHANDLE_H_ */
