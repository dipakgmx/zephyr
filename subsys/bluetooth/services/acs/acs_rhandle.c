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
#include "common/bt_str.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Information kept while reading the GATT table one attribute at a time. */
struct rhandle_walk_data {
	acs_rhandle_visit_t visit; /* Function called for each service or characteristic value */
	void *user_data;           /* Caller data passed to that function */
	const struct bt_gatt_attr *service_attr; /* Declaration of the current service */
	bool service_visited;          /* Whether the visitor has received the current service. */
	uint16_t next_resource_handle; /* Next ACS Resource Handle to assign, from 1 */
};

/* Holds the requested value attribute and its Resource Handle. */
struct rhandle_find_attr_data {
	const struct bt_gatt_attr *attr;
	uint16_t resource_handle; /* 0 until a match is found. */
};

/* Holds the requested ACS Resource Handle and the lookup result. */
struct rhandle_find_handle_data {
	uint16_t handle;
	struct acs_rhandle_resource_record result; /* attr is NULL until a match is found. */
};

/* Tracks the response buffer and the current service's child count field. */
struct uuid_map_build_data {
	struct net_buf *buf;
	/* Number_Of_Sub-Attributes of the current service (see §4.4.4.9.1) */
	uint8_t *service_child_count_field;
};

/* Checks whether a characteristic value attribute needs an ACS Resource Handle. */
static bool rhandle_characteristic_needs_handle(const struct bt_gatt_attr *attr,
						const struct bt_uuid *uuid)
{
	if (bt_uuid_cmp(uuid, BT_UUID_GATT_ACS_CP) == 0) {
		return true;
	}

#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	return acs_rmap_resource_needs_handle(attr, uuid);
#else
	return false;
#endif
}

/* Assign ACS Resource Handles to protected characteristic values and their enclosing services. */
static uint8_t rhandle_walk_cb(const struct bt_gatt_attr *attr, uint16_t handle, void *user_data)
{
	struct rhandle_walk_data *data = user_data;
	struct acs_rhandle_resource_record res = {0};

	ARG_UNUSED(handle);

	if (bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY) == 0 ||
	    bt_uuid_cmp(attr->uuid, BT_UUID_GATT_SECONDARY) == 0) {
		/* Reported with its first selected characteristic, if any. */
		data->service_attr = attr;
		data->service_visited = false;
		return BT_GATT_ITER_CONTINUE;
	} else if (bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC) == 0) {
		const struct bt_gatt_chrc *chrc = attr->user_data;

		/* The value directly follows its declaration (Core Vol 3 Part G 3.3). */
		res.attr = attr + 1;
		res.attr_type = ACS_RHANDLE_ATTR_CHAR_VALUE;
		res.resource_uuid = chrc->uuid;
		res.properties = chrc->properties;

		if (!rhandle_characteristic_needs_handle(res.attr, res.resource_uuid)) {
			return BT_GATT_ITER_CONTINUE;
		}
	} else {
		return BT_GATT_ITER_CONTINUE;
	}

	/* GATT places every characteristic inside a service. */
	__ASSERT_NO_MSG(data->service_attr != NULL);

	if (!data->service_visited) {
		/* A service declaration stores the service UUID in user_data. */
		struct acs_rhandle_resource_record service = {
			.attr = data->service_attr,
			.resource_uuid = data->service_attr->user_data,
		};

		service.attr_type = bt_uuid_cmp(data->service_attr->uuid, BT_UUID_GATT_PRIMARY) == 0
					    ? ACS_RHANDLE_ATTR_PRIMARY_SVC
					    : ACS_RHANDLE_ATTR_SECONDARY_SVC;

		service.resource_handle = data->next_resource_handle++;
		data->service_visited = true;
		service.service_uuid = service.resource_uuid;

		if (data->visit(&service, data->user_data) == BT_GATT_ITER_STOP) {
			return BT_GATT_ITER_STOP;
		}
	}

	res.resource_handle = data->next_resource_handle++;
	res.service_uuid = data->service_attr->user_data;

	/* The callback receives a temporary resource; copy it to keep it afterwards. */
	return data->visit(&res, data->user_data);
}

void acs_rhandle_foreach_resource(acs_rhandle_visit_t visit, void *user_data)
{
	struct rhandle_walk_data data = {
		.visit = visit,
		.user_data = user_data,
		.next_resource_handle = 1U,
	};

	bt_gatt_foreach_attr(BT_ATT_FIRST_ATTRIBUTE_HANDLE, BT_ATT_LAST_ATTRIBUTE_HANDLE,
			     rhandle_walk_cb, &data);
}

/* Record the Resource Handle of the requested value attribute and stop the search. */
static uint8_t rhandle_find_attr_visit(const struct acs_rhandle_resource_record *res,
				       void *user_data)
{
	struct rhandle_find_attr_data *data = user_data;

	if (res->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE || res->attr != data->attr) {
		return BT_GATT_ITER_CONTINUE;
	}

	data->resource_handle = res->resource_handle;
	return BT_GATT_ITER_STOP;
}

uint16_t acs_rhandle_from_attr(const struct bt_gatt_attr *attr)
{
	struct rhandle_find_attr_data data = {
		.attr = attr,
	};

	acs_rhandle_foreach_resource(rhandle_find_attr_visit, &data);
	return data.resource_handle;
}

static uint8_t rhandle_find_handle_visit(const struct acs_rhandle_resource_record *res,
					 void *user_data)
{
	struct rhandle_find_handle_data *data = user_data;

	if (res->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE || res->resource_handle != data->handle) {
		return BT_GATT_ITER_CONTINUE;
	}

	data->result = *res;
	return BT_GATT_ITER_STOP;
}

int acs_rhandle_get(uint16_t resource_handle, struct acs_rhandle_resource_record *resource_record)
{
	struct rhandle_find_handle_data data = {
		.handle = resource_handle,
	};

	acs_rhandle_foreach_resource(rhandle_find_handle_visit, &data);
	if (data.result.attr == NULL) {
		return -ENOENT;
	}

	*resource_record = data.result;
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
	case BT_UUID_TYPE_128:
		return BT_UUID_SIZE_128;
	default:
		__ASSERT_NO_MSG(false);
		return 0U;
	}
}

/* Write the UUID byte count, then the UUID */
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
	case BT_UUID_TYPE_128:
		net_buf_add_mem(buf, BT_UUID_128(uuid)->val, BT_UUID_SIZE_128);
		break;
	default:
		__ASSERT_NO_MSG(false);
		break;
	}
}

/*
 * Size of one Resource Handle To UUID Map record (Tables 4.25/4.26), its own
 * fields only; a service's nested records are sized by their own calls.
 *   Service:        Attribute_Type(1), Resource_Handle(2), UUID_Size(1), UUID,
 *                   Number_Of_Sub-Attributes(1)
 *   Characteristic: Attribute_Type(1), Resource_Handle(2), UUID_Size(1), UUID
 */
static size_t uuid_map_record_size(const struct acs_rhandle_resource_record *res)
{
	size_t record_size = sizeof(uint8_t) + sizeof(uint16_t) + sizeof(uint8_t) +
			     uuid_wire_size(res->resource_uuid);

	/* Only service records carry a characteristic count. */
	if (res->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE) {
		record_size += sizeof(uint8_t);
	}

	return record_size;
}

static uint8_t uuid_map_build_visit(const struct acs_rhandle_resource_record *res, void *user_data)
{
	struct uuid_map_build_data *data = user_data;
	bool is_service = res->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE;

	/* acs_rhandle_check_uuid_map_size() sized this response at init. */
	__ASSERT_NO_MSG(net_buf_tailroom(data->buf) >= uuid_map_record_size(res));

	net_buf_add_u8(data->buf, res->attr_type);
	net_buf_add_le16(data->buf, res->resource_handle);
	buf_add_uuid(data->buf, res->resource_uuid);

	if (is_service) {
		/* Reserve the count field; update it as child records are emitted. */
		data->service_child_count_field = net_buf_add(data->buf, 1U);
		*data->service_child_count_field = 0U;
	} else {
		/* acs_rhandle_foreach_resource() emits a characteristic's service first. */
		__ASSERT_NO_MSG(data->service_child_count_field != NULL);
		(*data->service_child_count_field)++;
	}

	LOG_DBG("%s %s (resource=0x%04x)", is_service ? "Service" : "  Char",
		bt_uuid_str(res->resource_uuid), res->resource_handle);

	return BT_GATT_ITER_CONTINUE;
}

uint8_t acs_cp_handle_get_resource_handle_uuid_map(struct acs_reply *reply,
						   struct net_buf_simple *payload)
{
	struct uuid_map_build_data data = {
		.buf = reply->response,
	};

	ARG_UNUSED(payload);

	acs_rhandle_foreach_resource(uuid_map_build_visit, &data);

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

/* Visitor for acs_rhandle_foreach_resource() that sums the size of each record in the UUID map. */
static uint8_t uuid_map_size_visit(const struct acs_rhandle_resource_record *res, void *user_data)
{
	size_t *size = user_data;

	*size += uuid_map_record_size(res);
	return BT_GATT_ITER_CONTINUE;
}

int acs_rhandle_check_uuid_map_size(void)
{
	size_t size = 1U; /* Response opcode */

	acs_rhandle_foreach_resource(uuid_map_size_visit, &size);

	if (size > ACS_MESSAGE_MAX_SIZE) {
		LOG_ERR("Resource Handle UUID Map Response needs %zu octets, "
			"CONFIG_BT_ACS_MAX_MESSAGE_SIZE is %d",
			size, ACS_MESSAGE_MAX_SIZE);
		return -EMSGSIZE;
	}

	return 0;
}

/* Write the service UUID, then the characteristic UUID, each preceded by its byte count. */
uint8_t acs_cp_handle_get_svc_char_uuids(struct acs_reply *reply, struct net_buf_simple *buf)
{
	uint16_t resource_handle = net_buf_simple_pull_le16(buf);
	struct net_buf *response = reply->response;
	struct acs_rhandle_resource_record resource;

	if (acs_rhandle_get(resource_handle, &resource) != 0) {
		LOG_WRN("Resource handle 0x%04x not found", resource_handle);
		return BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE;
	}

	if (net_buf_tailroom(response) < (size_t)(1U + uuid_wire_size(resource.service_uuid) + 1U +
						  uuid_wire_size(resource.resource_uuid))) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	buf_add_uuid(response, resource.service_uuid);
	buf_add_uuid(response, resource.resource_uuid);
	return BT_ACS_CP_RESPONSE_SUCCESS;
}
