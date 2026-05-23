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
#include "acs_rhandle.h"
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#include "acs_rmap.h"
#endif

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)

static uint8_t acs_attr_uuid_lookup_cb(const struct bt_gatt_attr *attr, uint16_t handle,
				       void *user_data)
{
	const struct bt_uuid **out = user_data;

	*out = attr->uuid;
	return BT_GATT_ITER_STOP;
}

static const char *acs_direction_str(enum bt_acs_direction direction)
{
	switch (direction) {
	case BT_ACS_DIRECTION_READ:
		return "READ";
	case BT_ACS_DIRECTION_WRITE:
		return "WRITE";
	case BT_ACS_DIRECTION_NOTIFY:
		return "NOTIFY";
	case BT_ACS_DIRECTION_INDICATE:
		return "INDICATE";
	default:
		return "UNKNOWN";
	}
}

#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

bool bt_acs_policy_is_permitted(struct bt_conn *conn, uint16_t att_handle,
				enum bt_acs_direction direction)
{
	struct bt_acs_conn const *acs_conn;

	if (!acs_is_initialized()) {
		return true;
	}

	acs_conn = acs_conn_lookup(conn);
	if (!acs_conn) {
		LOG_WRN("operation on handle 0x%04x without connection context", att_handle);
		return true;
	}

	if (acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED) {
		return true;
	}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	if (acs_rmap_char_is_protected(acs_conn->restriction_map_id, att_handle, direction)) {
		char uuid_str[BT_UUID_STR_LEN];
		const struct bt_uuid *attr_uuid = NULL;

		bt_gatt_foreach_attr(att_handle, att_handle, acs_attr_uuid_lookup_cb, &attr_uuid);
		if (attr_uuid) {
			bt_uuid_to_str(attr_uuid, uuid_str, sizeof(uuid_str));
		} else {
			snprintf(uuid_str, sizeof(uuid_str), "<unknown>");
		}
		LOG_WRN("operation denied by ACS: handle 0x%04x (%s) direction: %s", att_handle,
			uuid_str, acs_direction_str(direction));
		return false;
	}
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

	return true;
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)

/* CCCD gate table — maps protected CCCD handles to their char's permission entries. */
static struct acs_rmap_cccd_gate acs_protected_cccds[CONFIG_BT_ACS_MAX_PROTECTED_CCCD_GATES];
static uint8_t acs_protected_cccd_count;

void acs_policy_resolve_protected_cccds(void)
{
	size_t gate_count = 0;

	acs_rmap_collect_protected_cccds(acs_protected_cccds, ARRAY_SIZE(acs_protected_cccds),
					 &gate_count);
	acs_protected_cccd_count = (uint8_t)gate_count;
}

#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

static bool acs_gatt_read_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	uint16_t handle = bt_gatt_attr_get_handle(attr);

	return bt_acs_policy_is_permitted(conn, handle, BT_ACS_DIRECTION_READ);
}

static bool acs_gatt_write_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	uint16_t handle = bt_gatt_attr_get_handle(attr);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	/* Gate CCCD writes for notify/indicate-protected chars until ACS security is established.
	 */
	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CCC)) {
		for (uint8_t i = 0; i < acs_protected_cccd_count; i++) {
			if (acs_protected_cccds[i].cccd_handle == handle) {
				if (acs_protected_cccds[i].notify_protected &&
				    !bt_acs_policy_is_permitted(conn,
								acs_protected_cccds[i].char_handle,
								BT_ACS_DIRECTION_NOTIFY)) {
					return false;
				}
				if (acs_protected_cccds[i].indicate_protected &&
				    !bt_acs_policy_is_permitted(conn,
								acs_protected_cccds[i].char_handle,
								BT_ACS_DIRECTION_INDICATE)) {
					return false;
				}
				return true;
			}
		}
		/* CCCD not in the protected set — allow freely */
		return true;
	}
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

	return bt_acs_policy_is_permitted(conn, handle, BT_ACS_DIRECTION_WRITE);
}

static const struct bt_gatt_authorization_cb acs_gatt_auth_cb = {
	.read_authorize = acs_gatt_read_authorize,
	.write_authorize = acs_gatt_write_authorize,
};

int acs_policy_register_gatt_auth_cb(void)
{
	return bt_gatt_authorization_cb_register(&acs_gatt_auth_cb);
}
