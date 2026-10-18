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

/* Attribute_Type values in the Resource Handle To UUID Map (Table 4.26). */
enum acs_rhandle_attr_type {
	ACS_RHANDLE_ATTR_PRIMARY_SVC = 0x00,   /* Primary Service */
	ACS_RHANDLE_ATTR_SECONDARY_SVC = 0x01, /* Secondary Service */
	ACS_RHANDLE_ATTR_CHAR_VALUE = 0x02,    /* Characteristic Value */
};

/* Resource record linking an ACS handle to its GATT attribute and UUID. */
struct acs_rhandle_resource_record {
	uint8_t attr_type;  /* Attribute_Type, Table 4.26 (see @acs_rhandle_attr_type) */
	uint8_t properties; /* BT_GATT_CHRC_* bits from struct @bt_gatt_chrc, 0 for a service */
	uint16_t resource_handle;            /* AC Server-assigned Resource Handle */
	const struct bt_gatt_attr *attr;     /* GATT attribute backing this resource */
	const struct bt_uuid *resource_uuid; /* UUID of the service or characteristic represented */
	const struct bt_uuid
		*service_uuid; /* Enclosing service UUID; equals resource_uuid for services */
};

/*
 * Resolve a request's Resource Handle to a characteristic.
 * Copies the result on success (0); -ENOENT leaves it unchanged.
 */
int acs_rhandle_get(uint16_t resource_handle, struct acs_rhandle_resource_record *resource_record);

/* Return the ACS Resource Handle of characteristic value attr, or 0 when it has none. */
uint16_t acs_rhandle_from_attr(const struct bt_gatt_attr *attr);

/* Return BT_GATT_ITER_CONTINUE to visit the next resource, or BT_GATT_ITER_STOP. */
typedef uint8_t (*acs_rhandle_visit_t)(const struct acs_rhandle_resource_record *resource,
				       void *user_data);

/**
 * Iterate services and characteristics assigned ACS resource handles.
 */
void acs_rhandle_foreach_resource(acs_rhandle_visit_t visit, void *user_data);

/*
 * Check that the response for Get Resource Handle To UUID Map procedure can fit in one message
 * of CONFIG_BT_ACS_MAX_MESSAGE_SIZE octets.
 * Return -EMSGSIZE when it does not.
 */
int acs_rhandle_check_uuid_map_size(void);

#endif /* BT_GATT_ACS_RHANDLE_H_ */
