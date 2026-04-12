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

struct check_protected_ctx {
	uint16_t att_handle;
	enum bt_acs_direction direction;
	/* tri-state: 0 = not found, 1 = found+protected, 2 = found+not protected */
	uint8_t result;
};

static bool check_protected_cb(const struct bt_acs_rmap_protected *prot, void *user_data)
{
	struct check_protected_ctx *ctx = user_data;

	if (prot->resource_handle != ctx->att_handle) {
		return true;
	}

	for (uint8_t j = 0; j < prot->num_ops; j++) {
		uint16_t op = prot->ops[j].opcode;
		bool opcode_matches = false;

		switch (ctx->direction) {
		case BT_ACS_DIRECTION_READ:
			opcode_matches = (op == BT_ACS_RMAP_OP_ATT_READ_REQ ||
					  op == BT_ACS_RMAP_OP_ATT_READ_BLOB_REQ ||
					  op == BT_ACS_RMAP_OP_ATT_READ_MULT_REQ ||
					  op == BT_ACS_RMAP_OP_ATT_READ_MULT_VL_REQ);
			break;
		case BT_ACS_DIRECTION_WRITE:
			opcode_matches = (op == BT_ACS_RMAP_OP_ATT_WRITE_REQ ||
					  op == BT_ACS_RMAP_OP_ATT_WRITE_CMD ||
					  op == BT_ACS_RMAP_OP_ATT_SIGNED_WRITE_CMD ||
					  op == BT_ACS_RMAP_OP_ATT_PREPARE_WRITE_REQ ||
					  op == BT_ACS_RMAP_OP_ATT_EXECUTE_WRITE_REQ);
			break;
		case BT_ACS_DIRECTION_NOTIFY:
			opcode_matches = (op == BT_ACS_RMAP_OP_ATT_NOTIFY ||
					  op == BT_ACS_RMAP_OP_ATT_NOTIFY_MULT);
			break;
		case BT_ACS_DIRECTION_INDICATE:
			opcode_matches = (op == BT_ACS_RMAP_OP_ATT_INDICATE);
			break;
		}

		if (opcode_matches) {
			ctx->result = (prot->ops[j].isc_id != BT_ACS_ISC_ID_NONE) ? 1u : 2u;
			return false; /* stop */
		}
	}

	/* Handle in map but no opcode for this direction — not blocked */
	ctx->result = 2u;
	return false; /* stop */
}

/* Check if a handle is protected in the active restriction map for the given direction. */
static bool acs_handle_is_protected(uint16_t restriction_map_id, uint16_t att_handle,
				    enum bt_acs_direction direction)
{
	struct bt_acs_restriction_map map = {0};
	struct check_protected_ctx ctx = {
		.att_handle = att_handle,
		.direction = direction,
		.result = 0,
	};

	if (acs_rmap_lookup(restriction_map_id, &map) != 0) {
		return false;
	}

	acs_rmap_foreach_char(&map, check_protected_cb, &ctx);
	return ctx.result == 1u;
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
	if (acs_handle_is_protected(acs_conn->restriction_map_id, att_handle, direction)) {
		char uuid_str[BT_UUID_STR_LEN];
		const struct bt_uuid *attr_uuid = NULL;

		bt_gatt_foreach_attr(att_handle, att_handle, acs_attr_uuid_lookup_cb, &attr_uuid);
		if (attr_uuid) {
			bt_uuid_to_str(attr_uuid, uuid_str, sizeof(uuid_str));
		} else {
			strncpy(uuid_str, "<unknown>", sizeof(uuid_str));
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
struct acs_protected_cccd_gate {
	uint16_t cccd_handle;
	uint16_t char_handle;
	bool notify_protected;
	bool indicate_protected;
};

static struct acs_protected_cccd_gate acs_protected_cccds[CONFIG_BT_ACS_MAX_PROTECTED_CCCD_GATES];
static uint8_t acs_protected_cccd_count;

static bool cccd_resolve_char_cb(const struct bt_acs_rmap_protected *prot, void *user_data)
{
	bool protect_notify = false;
	bool protect_indicate = false;

	ARG_UNUSED(user_data);

	if (prot->resource_handle == 0) {
		return true;
	}

	for (uint8_t j = 0; j < prot->num_ops; j++) {
		uint16_t op = prot->ops[j].opcode;

		if (op == BT_ACS_RMAP_OP_ATT_NOTIFY || op == BT_ACS_RMAP_OP_ATT_NOTIFY_MULT) {
			protect_notify = true;
		} else if (op == BT_ACS_RMAP_OP_ATT_INDICATE) {
			protect_indicate = true;
		}
	}

	if (!protect_notify && !protect_indicate) {
		return true;
	}

	uint16_t cccd = acs_rhandle_find_cccd_for_char(prot->resource_handle);

	if (cccd == 0) {
		LOG_WRN("No CCCD found for protected char 0x%04x", prot->resource_handle);
		return true;
	}

	/* Avoid duplicates (same char may appear in multiple maps) */
	for (uint8_t k = 0; k < acs_protected_cccd_count; k++) {
		if (acs_protected_cccds[k].cccd_handle == cccd) {
			acs_protected_cccds[k].notify_protected |= protect_notify;
			acs_protected_cccds[k].indicate_protected |= protect_indicate;
			return true;
		}
	}

	if (acs_protected_cccd_count >= CONFIG_BT_ACS_MAX_PROTECTED_CCCD_GATES) {
		LOG_WRN("Protected CCCD gate table full (max %d) — char 0x%04x left ungated",
			CONFIG_BT_ACS_MAX_PROTECTED_CCCD_GATES, prot->resource_handle);
		return true;
	}

	acs_protected_cccds[acs_protected_cccd_count].cccd_handle = cccd;
	acs_protected_cccds[acs_protected_cccd_count].char_handle = prot->resource_handle;
	acs_protected_cccds[acs_protected_cccd_count].notify_protected = protect_notify;
	acs_protected_cccds[acs_protected_cccd_count].indicate_protected = protect_indicate;
	acs_protected_cccd_count++;
	LOG_DBG("CCCD 0x%04x gated for protected char 0x%04x (notify=%u indicate=%u)", cccd,
		prot->resource_handle, protect_notify, protect_indicate);
	return true;
}

void acs_policy_resolve_protected_cccds(void)
{
	acs_protected_cccd_count = 0;

	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		acs_rmap_foreach_char(map, cccd_resolve_char_cb, NULL);
	}
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

const struct bt_gatt_authorization_cb acs_gatt_auth_cb = {
	.read_authorize = acs_gatt_read_authorize,
	.write_authorize = acs_gatt_write_authorize,
};
