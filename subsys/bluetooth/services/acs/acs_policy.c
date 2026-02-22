/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/logging/log.h>

#include "acs_internal.h"
#include "acs_rmap.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static uint8_t acs_attr_uuid_lookup_cb(const struct bt_gatt_attr *attr, uint16_t handle,
				       void *user_data)
{
	const struct bt_uuid **out = user_data;

	*out = attr->uuid;
	return BT_GATT_ITER_STOP;
}

static bool acs_direction_is_outbound(enum acs_direction direction)
{
	return direction == ACS_DIRECTION_NOTIFY || direction == ACS_DIRECTION_INDICATE;
}

static bool acs_policy_is_permitted(struct bt_conn *conn, uint16_t attr_handle,
				    enum acs_direction direction)
{
	const struct bt_acs_rmap_runtime *active_rmap;
	struct bt_acs_conn const *acs_conn;

	if (!acs_is_initialized()) {
		LOG_WRN("operation on handle 0x%04x before ACS initialization completed",
			attr_handle);
		return acs_direction_is_outbound(direction);
	}

	acs_conn = acs_conn_lookup(conn);
	if (acs_conn == NULL) {
		LOG_WRN("operation on handle 0x%04x without connection context", attr_handle);
		return acs_direction_is_outbound(direction);
	}

	if (!acs_security_switch_get()) {
		return true;
	}

	active_rmap = acs_rmap_active();
	if (active_rmap == NULL) {
		LOG_ERR("operation on handle 0x%04x without an active restriction map",
			attr_handle);
		return false;
	}

	/* ACS's own CP handler enforces procedure policy after parsing the payload. */
	if (acs_handle_is_own(attr_handle)) {
		return true;
	}

	if (acs_rmap_attr_requires_protected_transport(active_rmap, attr_handle, direction)) {
		char uuid_str[BT_UUID_STR_LEN];
		const struct bt_uuid *attr_uuid = NULL;

		bt_gatt_foreach_attr(attr_handle, attr_handle, acs_attr_uuid_lookup_cb, &attr_uuid);
		if (attr_uuid != NULL) {
			bt_uuid_to_str(attr_uuid, uuid_str, sizeof(uuid_str));
		} else {
			snprintf(uuid_str, sizeof(uuid_str), "<unknown>");
		}
		LOG_WRN("operation denied by ACS: handle 0x%04x (%s) direction: %d", attr_handle,
			uuid_str, direction);
		return false;
	}

	return true;
}

static bool acs_gatt_read_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	uint16_t handle = bt_gatt_attr_get_handle(attr);

	return acs_policy_is_permitted(conn, handle, ACS_DIRECTION_READ);
}

static bool acs_gatt_write_data_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					  const void *buf, uint16_t len, uint16_t offset,
					  uint8_t flags)
{
	const struct bt_acs_rmap_runtime *active_rmap;
	uint16_t handle = bt_gatt_attr_get_handle(attr);
	bool opcode_valid;
	uint8_t opcode;

	if (!acs_is_initialized() || acs_conn_lookup(conn) == NULL) {
		return false;
	}

	if (!acs_security_switch_get()) {
		return true;
	}

	active_rmap = acs_rmap_active();
	if (active_rmap == NULL) {
		return false;
	}

	if (acs_handle_is_own(handle)) {
		return true;
	}

	opcode_valid = buf != NULL && len > 0U && offset == 0U &&
		       (flags & (BT_GATT_WRITE_FLAG_PREPARE | BT_GATT_WRITE_FLAG_EXECUTE)) == 0U;
	opcode = opcode_valid ? ((const uint8_t *)buf)[0] : 0U;
	if (acs_rmap_write_requires_protected_transport(active_rmap, handle, opcode_valid,
							opcode)) {
		LOG_WRN("plain write on protected handle 0x%04x denied; use Data In", handle);
		return false;
	}

	return true;
}

/* Convert a characteristic declaration to the value handle used by the map. */
static uint16_t acs_attr_char_handle(const struct bt_gatt_attr *attr)
{
	uint16_t handle = bt_gatt_attr_value_handle(attr);

	return (handle != 0) ? handle : bt_gatt_attr_get_handle(attr);
}

static bool acs_gatt_notify_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	uint16_t handle = acs_attr_char_handle(attr);

	if (acs_policy_is_permitted(conn, handle, ACS_DIRECTION_NOTIFY)) {
		return true;
	}

	LOG_WRN("plain notification on protected handle 0x%04x dropped; send it over Data Out "
		"with bt_acs_notify_uuid() or bt_acs_notify_cb()",
		handle);
	return false;
}

static bool acs_gatt_indicate_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	uint16_t handle = acs_attr_char_handle(attr);

	if (acs_policy_is_permitted(conn, handle, ACS_DIRECTION_INDICATE)) {
		return true;
	}

	LOG_WRN("plain indication on protected handle 0x%04x dropped; send it over Data Out "
		"with bt_acs_indicate()",
		handle);
	return false;
}

static const struct bt_gatt_authorization_cb acs_gatt_auth_cb = {
	.read_authorize = acs_gatt_read_authorize,
	.notify_authorize = acs_gatt_notify_authorize,
	.indicate_authorize = acs_gatt_indicate_authorize,
	.write_data_authorize = acs_gatt_write_data_authorize,
};

int acs_policy_register_gatt_auth_cb(void)
{
	return bt_gatt_authorization_cb_register(&acs_gatt_auth_cb);
}
