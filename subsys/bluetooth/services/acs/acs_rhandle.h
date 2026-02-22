/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_RHANDLE_H_
#define BT_GATT_ACS_RHANDLE_H_

#include <zephyr/types.h>
#include <zephyr/bluetooth/uuid.h>

struct bt_gatt_attr;

/* Attribute_Type values in the Resource Handle UUID Map (Table 4.26). */
#define ACS_RHANDLE_ATTR_PRIMARY_SVC   0x00
#define ACS_RHANDLE_ATTR_SECONDARY_SVC 0x01
#define ACS_RHANDLE_ATTR_CHAR_VALUE    0x02

/* Connects an ACS resource to its GATT attribute and metadata. Pointers belong to GATT. */
struct acs_rhandle_resource {
	uint8_t attr_type;               /* Attribute_Type, Table 4.26 */
	uint8_t props;                   /* Characteristic properties, 0 for a service */
	uint16_t resource_handle;        /* AC Server-assigned Resource Handle */
	uint16_t attr_handle;            /* GATT Attribute Handle of this attribute */
	const struct bt_gatt_attr *attr; /* GATT attribute backing this resource */
	const struct bt_uuid *uuid;      /* This resource's own UUID */
	const struct bt_uuid *svc_uuid;  /* UUID of the enclosing service */
};

/*
 * Resolve a request's Resource Handle to a characteristic.
 * Copies the result on success (0); -ENOENT leaves it unchanged.
 */
int acs_rhandle_get(uint16_t resource_handle, struct acs_rhandle_resource *resource);

/*
 * Find a characteristic's handles and metadata during initialization.
 * Copies the first UUID match in GATT order (0); -ENOENT leaves the result unchanged.
 */
int acs_rhandle_find_by_uuid(const struct bt_uuid *uuid, struct acs_rhandle_resource *resource);

/* Return BT_GATT_ITER_CONTINUE to visit the next resource, or BT_GATT_ITER_STOP. */
typedef uint8_t (*acs_rhandle_visit_t)(const struct acs_rhandle_resource *resource,
				       void *user_data);

/*
 * Iterator for walking the GATT databases and picking services and characteristic values.
 * For each selected resource, it calls the supplied @visit function with its UUID, GATT handle,
 * properties, and enclosing service.
 */
void acs_rhandle_foreach(acs_rhandle_visit_t visit, void *user_data);

#endif /* BT_GATT_ACS_RHANDLE_H_ */
