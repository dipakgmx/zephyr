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
#include "acs_rmap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/*
 * ACS identifies a resource by its Resource Handle; GATT uses Attribute Handles.
 * This file maps ACS Resource Handles to GATT attributes and builds the
 * Resource Handle UUID Map response. acs_rmap.c uses these handles to identify
 * the resources covered by its protection rules.
 *
 * acs_rhandle_foreach() numbers protected characteristic values and their enclosing services in
 * GATT order:
 *
 * GATT attribute              Attribute Handle    ACS Resource Handle
 * Service with protected char 0x0010              1
 * Characteristic declaration  0x0011              (skipped)
 * Protected char value        0x0012              2
 * CCC descriptor              0x0013              (skipped)
 * Unprotected char value      0x0015              (skipped)
 *
 * Each call reads the GATT table again; no resource list is stored. Numbering
 * stays the same while the GATT table is unchanged. acs_rhandle_foreach() visits
 * services and characteristics; the lookup functions return characteristics only.
 */

/* Information kept while reading the GATT table one attribute at a time. */
struct rhandle_walk_ctx {
	acs_rhandle_visit_t visit; /* Function called for each service or characteristic value */
	void *user_data;           /* Caller data passed to that function */
	struct acs_rhandle_resource service; /* Current service, emitted on its first resource */
	uint16_t next_resource_handle;  /* Next ACS Resource Handle to assign, starting at 1 */
	bool service_emitted;           /* Current service already passed to the visitor */
};

/* Holds the requested characteristic UUID and the lookup result. */
struct rhandle_uuid_lookup {
	const struct bt_uuid *uuid;
	struct acs_rhandle_resource result; /* attr is NULL until a match is found. */
};

/* Holds the requested ACS Resource Handle and the lookup result. */
struct rhandle_handle_lookup {
	uint16_t handle;
	struct acs_rhandle_resource result; /* attr is NULL until a match is found. */
};

/* Tracks the response buffer and the current service's characteristic count. */
struct rhandle_build_ctx {
	struct net_buf *buf;
	uint8_t *characteristic_count; /* Count byte in the current service record */
	int err; /* Error for the response handler to return after the callback stops. */
};

static bool rhandle_characteristic_needs_handle(uint16_t attr_handle, const struct bt_uuid *uuid)
{
	if (bt_uuid_cmp(uuid, BT_UUID_GATT_ACS_CP) == 0) {
		return true;
	}

#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	return acs_rmap_resource_needs_handle(attr_handle, uuid);
#else
	return false;
#endif
}

/* Assign ACS Resource Handles to protected characteristic values and their enclosing services. */
static uint8_t rhandle_walk_cb(const struct bt_gatt_attr *attr, uint16_t handle, void *user_data)
{
	struct rhandle_walk_ctx *ctx = user_data;
	struct acs_rhandle_resource res = {
		.attr = attr,
		.attr_handle = handle,
	};

	if (bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY) == 0 ||
	    bt_uuid_cmp(attr->uuid, BT_UUID_GATT_SECONDARY) == 0) {
		/* A service declaration stores the service UUID in user_data. */
		ctx->service = res;
		ctx->service.attr_type = bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY) == 0
						? ACS_RHANDLE_ATTR_PRIMARY_SVC
						: ACS_RHANDLE_ATTR_SECONDARY_SVC;
		ctx->service.uuid = (const struct bt_uuid *)attr->user_data;
		ctx->service.props = 0U;
		ctx->service_emitted = false;
		return BT_GATT_ITER_CONTINUE;
	} else if (bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC) == 0) {
		const struct bt_gatt_chrc *chrc = attr->user_data;

		/* Resolve the value directly from its characteristic declaration. */
		res.attr = bt_gatt_attr_next(attr);
		if (res.attr == NULL) {
			return BT_GATT_ITER_CONTINUE;
		}

		res.attr_handle = bt_gatt_attr_value_handle(attr);
		res.attr_type = ACS_RHANDLE_ATTR_CHAR_VALUE;
		res.uuid = chrc->uuid;
		res.props = chrc->properties;

		if (!rhandle_characteristic_needs_handle(res.attr_handle, res.uuid)) {
			return BT_GATT_ITER_CONTINUE;
		}
	} else {
		return BT_GATT_ITER_CONTINUE;
	}

	if (!ctx->service_emitted) {
		ctx->service.resource_handle = ctx->next_resource_handle++;
		ctx->service.svc_uuid = ctx->service.uuid;
		ctx->service_emitted = true;

		if (ctx->visit(&ctx->service, ctx->user_data) == BT_GATT_ITER_STOP) {
			return BT_GATT_ITER_STOP;
		}
	}

	res.resource_handle = ctx->next_resource_handle++;
	res.svc_uuid = ctx->service.uuid;

	/* The callback receives a temporary resource; copy it to keep it afterwards. */
	return ctx->visit(&res, ctx->user_data);
}


void acs_rhandle_foreach(acs_rhandle_visit_t visit, void *user_data)
{
	struct rhandle_walk_ctx ctx = {
		.visit = visit,
		.user_data = user_data,
		.next_resource_handle = 1U,
	};

	bt_gatt_foreach_attr(BT_ATT_FIRST_ATTRIBUTE_HANDLE, BT_ATT_LAST_ATTRIBUTE_HANDLE,
			     rhandle_walk_cb, &ctx);
}

/* Copy the first characteristic with the requested UUID and stop the search. */
static uint8_t rhandle_find_uuid_visit(const struct acs_rhandle_resource *res, void *user_data)
{
	struct rhandle_uuid_lookup *ctx = user_data;

	if (res->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE || bt_uuid_cmp(res->uuid, ctx->uuid)) {
		return BT_GATT_ITER_CONTINUE;
	}

	ctx->result = *res;
	return BT_GATT_ITER_STOP;
}

int acs_rhandle_find_by_uuid(const struct bt_uuid *uuid, struct acs_rhandle_resource *resource)
{
	struct rhandle_uuid_lookup ctx = {
		.uuid = uuid,
	};

	acs_rhandle_foreach(rhandle_find_uuid_visit, &ctx);
	if (ctx.result.attr == NULL) {
		return -ENOENT;
	}

	*resource = ctx.result;
	return 0;
}

static uint8_t rhandle_find_handle_visit(const struct acs_rhandle_resource *res, void *user_data)
{
	struct rhandle_handle_lookup *ctx = user_data;

	if (res->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE || res->resource_handle != ctx->handle) {
		return BT_GATT_ITER_CONTINUE;
	}

	ctx->result = *res;
	return BT_GATT_ITER_STOP;
}

int acs_rhandle_get(uint16_t resource_handle, struct acs_rhandle_resource *resource)
{
	struct rhandle_handle_lookup ctx = {
		.handle = resource_handle,
	};

	acs_rhandle_foreach(rhandle_find_handle_visit, &ctx);
	if (ctx.result.attr == NULL) {
		return -ENOENT;
	}

	*resource = ctx.result;
	return 0;
}

/* Number of UUID bytes, excluding the one-byte size field. */
static uint8_t uuid_wire_size(const struct bt_uuid *uuid)
{
	switch (uuid->type) {
	case BT_UUID_TYPE_16:
		return BT_UUID_SIZE_16;
	case BT_UUID_TYPE_32:
		return BT_UUID_SIZE_32;
	default:
		return BT_UUID_SIZE_128;
	}
}

/* Write the UUID byte count, then the UUID. The caller must check buffer space. */
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
		net_buf_add_mem(buf, BT_UUID_128(uuid)->val, BT_UUID_SIZE_128);
		break;
	}
}

/*
 * Resource Handle UUID Map response records; field sizes in bytes (Tables 4.25/4.26):
 *   Service:        Type(1), Handle(2), UUID_Size(1), UUID, Characteristic_Count(1)
 *   Characteristic: Type(1), Handle(2), UUID_Size(1), UUID
 * Characteristic_Count is the spec's Number_Of_Sub-Attributes field.
 */
static uint8_t rhandle_build_visit(const struct acs_rhandle_resource *res, void *user_data)
{
	struct rhandle_build_ctx *ctx = user_data;
	bool is_service = res->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE;
	size_t record_size = 1U + 2U + 1U + uuid_wire_size(res->uuid);
	char uuid_str[BT_UUID_STR_LEN];

	if (is_service) {
		record_size++; /* Only service records carry a characteristic count. */
	}

	/* Check space for the whole record before writing any of it. */
	if (net_buf_tailroom(ctx->buf) < record_size) {
		ctx->err = -ENOMEM;
		return BT_GATT_ITER_STOP;
	}

	net_buf_add_u8(ctx->buf, res->attr_type);
	net_buf_add_le16(ctx->buf, res->resource_handle);
	buf_add_uuid(ctx->buf, res->uuid);

	if (is_service) {
		ctx->characteristic_count = net_buf_add(ctx->buf, 1);
		*ctx->characteristic_count = 0;
	} else if (ctx->characteristic_count != NULL) {
		(*ctx->characteristic_count)++;
	}

	bt_uuid_to_str(res->uuid, uuid_str, sizeof(uuid_str));
	LOG_DBG("%s %s (resource=0x%04x att=0x%04x)", is_service ? "Service" : "  Char", uuid_str,
		res->resource_handle, res->attr_handle);

	return BT_GATT_ITER_CONTINUE;
}

uint8_t acs_cp_handle_get_resource_handle_uuid_map(struct acs_reply *reply,
						   struct net_buf_simple *payload)
{
	struct rhandle_build_ctx ctx = {
		.buf = reply->response,
	};

	ARG_UNUSED(payload);

	acs_rhandle_foreach(rhandle_build_visit, &ctx);

	return (ctx.err == 0) ? BT_ACS_CP_RESPONSE_SUCCESS
			      : BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
}

/* Write the service UUID, then the characteristic UUID, each preceded by its byte count. */
uint8_t acs_cp_handle_get_svc_char_uuids(struct acs_reply *reply, struct net_buf_simple *buf)
{
	uint16_t resource_handle = net_buf_simple_pull_le16(buf);
	struct net_buf *response = reply->response;
	struct acs_rhandle_resource resource;

	if (acs_rhandle_get(resource_handle, &resource) != 0) {
		LOG_WRN("Resource handle 0x%04x not found", resource_handle);
		return BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE;
	}

	if (net_buf_tailroom(response) <
	    (size_t)(1U + uuid_wire_size(resource.svc_uuid) + 1U + uuid_wire_size(resource.uuid))) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	buf_add_uuid(response, resource.svc_uuid);
	buf_add_uuid(response, resource.uuid);
	return BT_ACS_CP_RESPONSE_SUCCESS;
}
