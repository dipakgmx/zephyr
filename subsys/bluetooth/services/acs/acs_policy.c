/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/logging/log.h>

#include "acs_internal.h"
#include "acs_rmap.h"
#include "common/bt_str.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/*
 * Checks shared by every plain GATT operation. Returns the active restriction map when the map
 * decides the operation; otherwise returns NULL with permitted set to the verdict.
 */
static const struct bt_acs_restriction_map *
acs_policy_map(struct bt_conn *conn, const struct bt_gatt_attr *attr, bool *permitted)
{
	const struct bt_acs_restriction_map *active_rmap;

	*permitted = false;

	if (!acs_is_initialized()) {
		LOG_WRN("operation before ACS initialization completed");
		return NULL;
	}

	if (acs_conn_lookup(conn) == NULL) {
		LOG_WRN("operation without connection context");
		return NULL;
	}

	if (!acs_security_switch_get()) {
		*permitted = true;
		return NULL;
	}

	active_rmap = acs_rmap_get_active_map();
	if (active_rmap == NULL) {
		LOG_ERR("operation without an active restriction map");
		return NULL;
	}

	/* ACS's own attributes are not blocked here: its Control Point and Data handlers apply
	 * the map after reading the payload, and what ACS sends (Control Point responses,
	 * Status, Data Out) is already the protected route.
	 */
	if (acs_attr_is_own(attr)) {
		*permitted = true;
		return NULL;
	}

	return active_rmap;
}

/* Checks whether a plain GATT read or write (operation BT_GATT_CHRC_READ or _WRITE) may go
 * ahead. False when the active restriction map protects the operation, which must then go
 * through the ACS Data In path instead.
 */
static bool acs_gatt_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       uint8_t operation)
{
	const struct bt_acs_restriction_map *active_rmap;
	bool permitted;

	active_rmap = acs_policy_map(conn, attr, &permitted);
	if (active_rmap == NULL) {
		return permitted;
	}

	if (acs_rmap_attr_isc(active_rmap, attr, operation, NULL) != BT_ACS_ISC_ID_NONE) {
		LOG_WRN("plain %s on protected %s denied; use Data In",
			(operation == BT_GATT_CHRC_READ) ? "read" : "write",
			bt_uuid_str(attr->uuid));
		return false;
	}

	return true;
}

static bool acs_gatt_read_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	return acs_gatt_authorize(conn, attr, BT_GATT_CHRC_READ);
}

static bool acs_gatt_write_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	return acs_gatt_authorize(conn, attr, BT_GATT_CHRC_WRITE);
}

int acs_policy_register_gatt_auth_cb(void)
{
	static const struct bt_gatt_authorization_cb acs_gatt_auth_cb = {
		.read_authorize = acs_gatt_read_authorize,
		.write_authorize = acs_gatt_write_authorize,
	};

	return bt_gatt_authorization_cb_register(&acs_gatt_auth_cb);
}
