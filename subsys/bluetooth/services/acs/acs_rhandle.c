/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include "acs_rhandle.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

uint8_t acs_find_char_attrs_cb(const struct bt_gatt_attr *attr, uint16_t handle, void *user_data)
{
	struct acs_char_attr_ctx *ctx = user_data;

	if (handle == ctx->value_handle - 1U) {
		ctx->decl = attr;
	} else if (handle == ctx->value_handle) {
		ctx->value = attr;
	}
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t uuid_wire_size(const struct bt_uuid *uuid)
{
	return uuid->type == BT_UUID_TYPE_16 ? 2u : 16u;
}

/* Append UUID_Size(1) + UUID(var) to buf. */
static int buf_add_uuid(struct net_buf_simple *buf, const struct bt_uuid *uuid)
{
	uint8_t sz = uuid_wire_size(uuid);

	if (net_buf_simple_tailroom(buf) < (1 + sz)) {
		return -ENOMEM;
	}

	net_buf_simple_add_u8(buf, sz);

	if (uuid->type == BT_UUID_TYPE_16) {
		net_buf_simple_add_le16(buf, BT_UUID_16(uuid)->val);
	} else {
		net_buf_simple_add_mem(buf, BT_UUID_128(uuid)->val, 16);
	}

	return 0;
}

struct rhandle_build_ctx {
	struct net_buf_simple *buf;
	bool prev_was_chrc;              /* previous attr was a CHRC declaration */
	const struct bt_uuid *char_uuid; /* UUID saved from CHRC decl user_data */
	uint8_t *num_sub_ptr; /* pointer into buf for Number_Of_Sub-Attributes backfill */
	uint8_t sub_count;    /* running count of chars under current service */
	int err;
};

static int write_service_record(struct rhandle_build_ctx *ctx, uint8_t attr_type, uint16_t handle,
				const struct bt_uuid *svc_uuid)
{
	uint8_t uuid_sz;

	if (ctx->num_sub_ptr != NULL) {
		*ctx->num_sub_ptr = ctx->sub_count;
	}
	ctx->sub_count = 0;
	ctx->num_sub_ptr = NULL;

	uuid_sz = uuid_wire_size(svc_uuid);

	if (net_buf_simple_tailroom(ctx->buf) < (1 + 2 + 1 + uuid_sz + 1)) {
		return -ENOMEM;
	}

	net_buf_simple_add_u8(ctx->buf, attr_type);
	net_buf_simple_add_le16(ctx->buf, handle);
	buf_add_uuid(ctx->buf, svc_uuid);

	ctx->num_sub_ptr = net_buf_simple_add(ctx->buf, 1);
	*ctx->num_sub_ptr = 0;

	return 0;
}

static int write_char_record(struct rhandle_build_ctx *ctx, uint16_t handle,
			     const struct bt_uuid *char_uuid)
{
	uint8_t uuid_sz = uuid_wire_size(char_uuid);

	if (net_buf_simple_tailroom(ctx->buf) < (1 + 2 + 1 + uuid_sz)) {
		return -ENOMEM;
	}

	net_buf_simple_add_u8(ctx->buf, ACS_RHANDLE_ATTR_CHAR_VALUE);
	net_buf_simple_add_le16(ctx->buf, handle);
	buf_add_uuid(ctx->buf, char_uuid);

	ctx->sub_count++;
	return 0;
}

static uint8_t rhandle_build_cb(const struct bt_gatt_attr *attr, uint16_t handle, void *user_data)
{
	struct rhandle_build_ctx *ctx = user_data;

	if (ctx->err) {
		return BT_GATT_ITER_STOP;
	}

	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY)) {
		const struct bt_uuid *svc_uuid = (const struct bt_uuid *)attr->user_data;

		ctx->prev_was_chrc = false;
		ctx->err =
			write_service_record(ctx, ACS_RHANDLE_ATTR_PRIMARY_SVC, handle, svc_uuid);
		if (ctx->err) {
			return BT_GATT_ITER_STOP;
		}

		char uuid_str[BT_UUID_STR_LEN];

		bt_uuid_to_str(svc_uuid, uuid_str, sizeof(uuid_str));
		LOG_DBG("RH map: Primary Service handle=0x%04x uuid=%s", handle, uuid_str);
	} else if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_SECONDARY)) {
		const struct bt_uuid *svc_uuid = (const struct bt_uuid *)attr->user_data;

		ctx->prev_was_chrc = false;
		ctx->err =
			write_service_record(ctx, ACS_RHANDLE_ATTR_SECONDARY_SVC, handle, svc_uuid);
		if (ctx->err) {
			return BT_GATT_ITER_STOP;
		}

		char uuid_str[BT_UUID_STR_LEN];

		bt_uuid_to_str(svc_uuid, uuid_str, sizeof(uuid_str));
		LOG_DBG("RH map: Secondary Service handle=0x%04x uuid=%s", handle, uuid_str);
	} else if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC)) {
		const struct bt_gatt_chrc *chrc = (const struct bt_gatt_chrc *)attr->user_data;

		ctx->prev_was_chrc = true;
		ctx->char_uuid = chrc->uuid;
	} else {
		if (ctx->prev_was_chrc) {
			ctx->err = write_char_record(ctx, handle, ctx->char_uuid);
			if (ctx->err) {
				return BT_GATT_ITER_STOP;
			}

			char uuid_str[BT_UUID_STR_LEN];

			bt_uuid_to_str(ctx->char_uuid, uuid_str, sizeof(uuid_str));
			LOG_DBG("RH map: Char Value handle=0x%04x uuid=%s", handle, uuid_str);
		}
		ctx->prev_was_chrc = false;
	}

	return BT_GATT_ITER_CONTINUE;
}

int acs_rhandle_build_map_response(struct net_buf_simple *buf)
{
	struct rhandle_build_ctx ctx = {
		.buf = buf,
		.prev_was_chrc = false,
		.char_uuid = NULL,
		.num_sub_ptr = NULL,
		.sub_count = 0,
		.err = 0,
	};

	bt_gatt_foreach_attr(0x0001, 0xFFFF, rhandle_build_cb, &ctx);

	if (ctx.num_sub_ptr != NULL) {
		*ctx.num_sub_ptr = ctx.sub_count;
	}

	return ctx.err;
}

struct rhandle_find_uuid_ctx {
	const struct bt_uuid *target_uuid;
	uint16_t found_handle;
	bool prev_was_chrc;
	const struct bt_uuid *char_uuid;
};

static uint8_t rhandle_find_uuid_cb(const struct bt_gatt_attr *attr, uint16_t handle,
				    void *user_data)
{
	struct rhandle_find_uuid_ctx *ctx = user_data;

	if (ctx->found_handle != 0) {
		return BT_GATT_ITER_STOP;
	}

	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC)) {
		const struct bt_gatt_chrc *chrc = (const struct bt_gatt_chrc *)attr->user_data;

		ctx->prev_was_chrc = true;
		ctx->char_uuid = chrc->uuid;
	} else {
		if (ctx->prev_was_chrc && !bt_uuid_cmp(ctx->char_uuid, ctx->target_uuid)) {
			ctx->found_handle = handle;
			return BT_GATT_ITER_STOP;
		}
		ctx->prev_was_chrc = false;
	}

	return BT_GATT_ITER_CONTINUE;
}

uint16_t acs_rhandle_find_char_handle(const struct bt_uuid *char_uuid)
{
	struct rhandle_find_uuid_ctx ctx = {
		.target_uuid = char_uuid,
		.found_handle = 0,
		.prev_was_chrc = false,
		.char_uuid = NULL,
	};

	bt_gatt_foreach_attr(0x0001, 0xFFFF, rhandle_find_uuid_cb, &ctx);
	return ctx.found_handle;
}

static uint8_t cccd_find_cb(const struct bt_gatt_attr *attr, uint16_t handle, void *user_data)
{
	uint16_t *found = user_data;

	/* Stop at any service or characteristic declaration boundary. */
	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY) ||
	    !bt_uuid_cmp(attr->uuid, BT_UUID_GATT_SECONDARY) ||
	    !bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC)) {
		return BT_GATT_ITER_STOP;
	}

	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CCC)) {
		*found = handle;
		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_CONTINUE;
}

uint16_t acs_rhandle_find_cccd_for_char(uint16_t char_value_handle)
{
	uint16_t found = 0;

	bt_gatt_foreach_attr(char_value_handle + 1, char_value_handle + 8, cccd_find_cb, &found);
	return found;
}

struct rhandle_lookup_ctx {
	uint16_t target_handle;
	const struct bt_uuid *current_svc_uuid; /* UUID of the current service being iterated */
	const struct bt_uuid *found_svc_uuid;   /* service UUID at time of match */
	const struct bt_uuid *found_char_uuid;  /* char UUID at time of match */
	bool prev_was_chrc;
	const struct bt_uuid *char_uuid; /* saved from CHRC decl */
	bool found;
};

static uint8_t rhandle_lookup_cb(const struct bt_gatt_attr *attr, uint16_t handle, void *user_data)
{
	struct rhandle_lookup_ctx *ctx = user_data;

	if (ctx->found) {
		return BT_GATT_ITER_STOP;
	}

	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY) ||
	    !bt_uuid_cmp(attr->uuid, BT_UUID_GATT_SECONDARY)) {
		ctx->current_svc_uuid = (const struct bt_uuid *)attr->user_data;
		ctx->prev_was_chrc = false;
	} else if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC)) {
		const struct bt_gatt_chrc *chrc = (const struct bt_gatt_chrc *)attr->user_data;

		ctx->prev_was_chrc = true;
		ctx->char_uuid = chrc->uuid;
	} else {
		if (ctx->prev_was_chrc) {
			if (handle == ctx->target_handle) {
				ctx->found_svc_uuid = ctx->current_svc_uuid;
				ctx->found_char_uuid = ctx->char_uuid;
				ctx->found = true;
				return BT_GATT_ITER_STOP;
			}
		}
		ctx->prev_was_chrc = false;
	}

	return BT_GATT_ITER_CONTINUE;
}

int acs_rhandle_lookup_svc_char(uint16_t resource_handle, struct net_buf_simple *buf)
{
	struct rhandle_lookup_ctx ctx;
	int err;

	ctx.target_handle = resource_handle;
	ctx.current_svc_uuid = NULL;
	ctx.found_svc_uuid = NULL;
	ctx.found_char_uuid = NULL;
	ctx.prev_was_chrc = false;
	ctx.char_uuid = NULL;
	ctx.found = false;

	bt_gatt_foreach_attr(0x0001, 0xFFFF, rhandle_lookup_cb, &ctx);

	if (!ctx.found) {
		LOG_WRN("RH lookup: handle 0x%04x not found", resource_handle);
		return -ENOENT;
	}

	err = buf_add_uuid(buf, ctx.found_svc_uuid);
	if (err) {
		return err;
	}

	err = buf_add_uuid(buf, ctx.found_char_uuid);
	if (err) {
		return err;
	}

	return 0;
}
