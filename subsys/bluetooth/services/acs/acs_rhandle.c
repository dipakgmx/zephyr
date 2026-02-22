/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include "acs_cp_operands.h"
#include "acs_internal.h"
#include "acs_reply.h"
#include "acs_rhandle.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* State of one Resource Handle enumeration. */
struct rhandle_walk_ctx {
	acs_rhandle_visit_t visit;
	void *user_data;
	const struct bt_uuid *svc_uuid;  /* Current service UUID */
	const struct bt_uuid *char_uuid; /* Pending characteristic UUID */
	uint8_t char_props;              /* Pending characteristic properties */
	uint16_t next_resource_handle;
	bool prev_was_chrc;
};

/* Resource Handle UUID Map response state (Table 4.25). */
struct rhandle_build_ctx {
	struct net_buf *buf;
	uint8_t *num_sub_ptr; /* Number_Of_Sub-Attributes output position */
	uint8_t sub_count;    /* Characteristics in the current service */
	int err;
};

/* Characteristic lookup by UUID. */
struct rhandle_find_uuid_ctx {
	const struct bt_uuid *target_uuid;
	uint16_t resource_handle;
	uint16_t attr_handle;
};

/* Resource lookup by Resource Handle. */
struct rhandle_lookup_ctx {
	uint16_t target_resource_handle;
	const struct bt_uuid *svc_uuid;
	const struct bt_uuid *char_uuid;
	uint16_t attr_handle;
	bool found;
};

/* Resource Handle lookup by GATT Attribute Handle. */
struct rhandle_find_att_ctx {
	uint16_t target_attr_handle;
	uint16_t resource_handle;
};

/* Characteristic declaration and value lookup state. */
struct acs_char_attr_ctx {
	uint16_t value_handle;
	const struct bt_gatt_attr *decl;
	const struct bt_gatt_attr *value;
};

static uint8_t acs_find_char_attrs_cb(const struct bt_gatt_attr *attr, uint16_t handle,
				      void *user_data)
{
	struct acs_char_attr_ctx *ctx = user_data;

	if (handle == ctx->value_handle - 1U) {
		ctx->decl = attr;
	} else if (handle == ctx->value_handle) {
		ctx->value = attr;
	}
	return BT_GATT_ITER_CONTINUE;
}

int acs_resolve_char(uint16_t value_handle, const struct bt_gatt_attr **value_out,
		     uint8_t *props_out)
{
	struct acs_char_attr_ctx ctx = {
		.value_handle = value_handle,
		.decl = NULL,
		.value = NULL,
	};

	bt_gatt_foreach_attr(value_handle - 1U, value_handle, acs_find_char_attrs_cb, &ctx);
	if (ctx.value == NULL) {
		return -ENOENT;
	}

	if (value_out != NULL) {
		*value_out = ctx.value;
	}
	if (props_out != NULL) {
		*props_out =
			(ctx.decl != NULL && ctx.decl->user_data != NULL)
				? ((const struct bt_gatt_chrc *)ctx.decl->user_data)->properties
				: 0U;
	}
	return 0;
}

static uint8_t uuid_wire_size(const struct bt_uuid *uuid)
{
	switch (uuid->type) {
	case BT_UUID_TYPE_16:
		return 2u;
	case BT_UUID_TYPE_32:
		return 4u;
	default:
		return 16u;
	}
}

/* Append UUID_Size(1) + UUID(var). The caller guarantees the tailroom. */
static void buf_add_uuid(struct net_buf *buf, const struct bt_uuid *uuid)
{
	net_buf_add_u8(buf, uuid_wire_size(uuid));

	switch (uuid->type) {
	case BT_UUID_TYPE_16:
		net_buf_add_le16(buf, BT_UUID_16(uuid)->val);
		break;
	case BT_UUID_TYPE_32:
		net_buf_add_le32(buf, BT_UUID_32(uuid)->val);
		break;
	default:
		net_buf_add_mem(buf, BT_UUID_128(uuid)->val, 16);
		break;
	}
}

/* Number service declarations and characteristic values in GATT order. */
static uint8_t rhandle_walk_cb(const struct bt_gatt_attr *attr, uint16_t handle, void *user_data)
{
	struct rhandle_walk_ctx *ctx = user_data;
	struct acs_rhandle_resource res;

	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY) ||
	    !bt_uuid_cmp(attr->uuid, BT_UUID_GATT_SECONDARY)) {
		ctx->prev_was_chrc = false;
		ctx->svc_uuid = (const struct bt_uuid *)attr->user_data;
		res.attr_type = !bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY)
					? ACS_RHANDLE_ATTR_PRIMARY_SVC
					: ACS_RHANDLE_ATTR_SECONDARY_SVC;
		res.uuid = ctx->svc_uuid;
		res.props = 0U;
	} else if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC)) {
		const struct bt_gatt_chrc *chrc = attr->user_data;

		ctx->prev_was_chrc = true;
		ctx->char_uuid = chrc->uuid;
		ctx->char_props = chrc->properties;
		return BT_GATT_ITER_CONTINUE;
	} else if (ctx->prev_was_chrc) {
		ctx->prev_was_chrc = false;
		res.attr_type = ACS_RHANDLE_ATTR_CHAR_VALUE;
		res.uuid = ctx->char_uuid;
		res.props = ctx->char_props;
	} else {
		return BT_GATT_ITER_CONTINUE;
	}

	res.resource_handle = ctx->next_resource_handle++;
	res.attr_handle = handle;
	res.attr = attr;
	res.svc_uuid = ctx->svc_uuid;

	return ctx->visit(&res, ctx->user_data);
}

void acs_rhandle_walk(acs_rhandle_visit_t visit, void *user_data)
{
	struct rhandle_walk_ctx ctx = {
		.visit = visit,
		.user_data = user_data,
		.svc_uuid = NULL,
		.char_uuid = NULL,
		.char_props = 0U,
		.next_resource_handle = 1U,
		.prev_was_chrc = false,
	};

	bt_gatt_foreach_attr(BT_ATT_FIRST_ATTRIBUTE_HANDLE, BT_ATT_LAST_ATTRIBUTE_HANDLE,
			     rhandle_walk_cb, &ctx);
}

static int write_service_record(struct rhandle_build_ctx *ctx,
				const struct acs_rhandle_resource *res)
{
	if (ctx->num_sub_ptr != NULL) {
		*ctx->num_sub_ptr = ctx->sub_count;
	}
	ctx->sub_count = 0;
	ctx->num_sub_ptr = NULL;

	if (net_buf_tailroom(ctx->buf) < (1 + 2 + 1 + uuid_wire_size(res->uuid) + 1)) {
		return -ENOMEM;
	}

	net_buf_add_u8(ctx->buf, res->attr_type);
	net_buf_add_le16(ctx->buf, res->resource_handle);
	buf_add_uuid(ctx->buf, res->uuid);

	ctx->num_sub_ptr = net_buf_add(ctx->buf, 1);
	*ctx->num_sub_ptr = 0;

	return 0;
}

static int write_char_record(struct rhandle_build_ctx *ctx, const struct acs_rhandle_resource *res)
{
	if (net_buf_tailroom(ctx->buf) < (1 + 2 + 1 + uuid_wire_size(res->uuid))) {
		return -ENOMEM;
	}

	net_buf_add_u8(ctx->buf, ACS_RHANDLE_ATTR_CHAR_VALUE);
	net_buf_add_le16(ctx->buf, res->resource_handle);
	buf_add_uuid(ctx->buf, res->uuid);

	ctx->sub_count++;
	return 0;
}

static uint8_t rhandle_build_visit(const struct acs_rhandle_resource *res, void *user_data)
{
	struct rhandle_build_ctx *ctx = user_data;
	char uuid_str[BT_UUID_STR_LEN];

	if (res->attr_type == ACS_RHANDLE_ATTR_CHAR_VALUE) {
		ctx->err = write_char_record(ctx, res);
	} else {
		ctx->err = write_service_record(ctx, res);
	}

	if (ctx->err) {
		return BT_GATT_ITER_STOP;
	}

	bt_uuid_to_str(res->uuid, uuid_str, sizeof(uuid_str));
	LOG_DBG("%s %s (resource=0x%04x att=0x%04x)",
		res->attr_type == ACS_RHANDLE_ATTR_CHAR_VALUE ? "  Char" : "Service", uuid_str,
		res->resource_handle, res->attr_handle);

	return BT_GATT_ITER_CONTINUE;
}

int acs_rhandle_build_map_response(struct net_buf *buf)
{
	struct rhandle_build_ctx ctx = {
		.buf = buf,
		.num_sub_ptr = NULL,
		.sub_count = 0,
		.err = 0,
	};

	acs_rhandle_walk(rhandle_build_visit, &ctx);

	if (ctx.num_sub_ptr != NULL) {
		*ctx.num_sub_ptr = ctx.sub_count;
	}

	return ctx.err;
}

static uint8_t rhandle_find_uuid_visit(const struct acs_rhandle_resource *res, void *user_data)
{
	struct rhandle_find_uuid_ctx *ctx = user_data;

	if (res->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE ||
	    bt_uuid_cmp(res->uuid, ctx->target_uuid) != 0) {
		return BT_GATT_ITER_CONTINUE;
	}

	ctx->resource_handle = res->resource_handle;
	ctx->attr_handle = res->attr_handle;
	return BT_GATT_ITER_STOP;
}

int acs_rhandle_find_char_attr_handles(const struct bt_uuid *char_uuid, uint16_t *resource_handle,
				       uint16_t *attr_handle)
{
	struct rhandle_find_uuid_ctx ctx = {
		.target_uuid = char_uuid,
		.resource_handle = 0,
		.attr_handle = 0,
	};

	acs_rhandle_walk(rhandle_find_uuid_visit, &ctx);

	if (ctx.resource_handle == 0) {
		return -ENOENT;
	}

	if (resource_handle != NULL) {
		*resource_handle = ctx.resource_handle;
	}
	if (attr_handle != NULL) {
		*attr_handle = ctx.attr_handle;
	}

	return 0;
}

static uint8_t rhandle_lookup_visit(const struct acs_rhandle_resource *res, void *user_data)
{
	struct rhandle_lookup_ctx *ctx = user_data;

	if (res->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE ||
	    res->resource_handle != ctx->target_resource_handle) {
		return BT_GATT_ITER_CONTINUE;
	}

	ctx->svc_uuid = res->svc_uuid;
	ctx->char_uuid = res->uuid;
	ctx->attr_handle = res->attr_handle;
	ctx->found = true;
	return BT_GATT_ITER_STOP;
}

int acs_rhandle_lookup_svc_char(uint16_t resource_handle, struct net_buf *buf)
{
	struct rhandle_lookup_ctx ctx = {
		.target_resource_handle = resource_handle,
		.found = false,
	};

	acs_rhandle_walk(rhandle_lookup_visit, &ctx);

	if (!ctx.found) {
		LOG_WRN("RH lookup: handle 0x%04x not found", resource_handle);
		return -ENOENT;
	}

	if (net_buf_tailroom(buf) <
	    (size_t)(1U + uuid_wire_size(ctx.svc_uuid) + 1U + uuid_wire_size(ctx.char_uuid))) {
		return -ENOMEM;
	}

	buf_add_uuid(buf, ctx.svc_uuid);
	buf_add_uuid(buf, ctx.char_uuid);

	return 0;
}

uint16_t acs_rhandle_find_attr_handle(uint16_t resource_handle)
{
	struct rhandle_lookup_ctx ctx = {
		.target_resource_handle = resource_handle,
		.found = false,
	};

	acs_rhandle_walk(rhandle_lookup_visit, &ctx);

	return ctx.found ? ctx.attr_handle : 0;
}

static uint8_t rhandle_find_att_visit(const struct acs_rhandle_resource *res, void *user_data)
{
	struct rhandle_find_att_ctx *ctx = user_data;

	if (res->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE ||
	    res->attr_handle != ctx->target_attr_handle) {
		return BT_GATT_ITER_CONTINUE;
	}

	ctx->resource_handle = res->resource_handle;
	return BT_GATT_ITER_STOP;
}

uint16_t acs_rhandle_find_resource_handle(uint16_t attr_handle)
{
	struct rhandle_find_att_ctx ctx = {
		.target_attr_handle = attr_handle,
		.resource_handle = 0,
	};

	acs_rhandle_walk(rhandle_find_att_visit, &ctx);

	return ctx.resource_handle;
}

struct acs_cp_result acs_cp_handle_get_resource_handle_uuid_map(struct acs_reply *reply,
								struct net_buf_simple *payload)
{
	ARG_UNUSED(payload);

	int err = acs_rhandle_build_map_response(reply->response);

	if (err) {
		return acs_cp_status(errno_to_acs_status(err));
	}

	return acs_cp_reply();
}

struct acs_cp_result acs_cp_handle_get_svc_char_uuids(struct acs_reply *reply,
						      struct net_buf_simple *buf)
{
	uint16_t resource_handle = net_buf_simple_pull_le16(buf);
	int err;

	err = acs_rhandle_lookup_svc_char(resource_handle, reply->response);
	if (err == -ENOENT) {
		return acs_cp_status(BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	} else if (err) {
		return acs_cp_status(errno_to_acs_status(err));
	}

	return acs_cp_reply();
}
