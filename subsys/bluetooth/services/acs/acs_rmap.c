/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/acs.h>

#include "host/att_internal.h"

#include "acs_cp_operands.h"
#include "acs_rmap.h"
#include "acs_rhandle.h"
#include "acs_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/*
 * ATT opcodes each protected characteristic operation covers. A record
 * advertises an opcode when the characteristic has the property that offers it.
 */
static const struct {
	uint8_t operation;  /* BT_GATT_CHRC_* named in the record */
	uint8_t prop;       /* characteristic property offering the opcode */
	uint8_t att_opcode; /* ATT opcode (Core Vol 3 Part F 3.4) */
} rmap_char_opcodes[] = {
	{BT_GATT_CHRC_READ, BT_GATT_CHRC_READ, BT_ATT_OP_READ_REQ},
	{BT_GATT_CHRC_READ, BT_GATT_CHRC_READ, BT_ATT_OP_READ_BLOB_REQ},
	{BT_GATT_CHRC_WRITE, BT_GATT_CHRC_WRITE, BT_ATT_OP_WRITE_REQ},
	{BT_GATT_CHRC_WRITE, BT_GATT_CHRC_WRITE, BT_ATT_OP_PREPARE_WRITE_REQ},
	{BT_GATT_CHRC_WRITE, BT_GATT_CHRC_WRITE, BT_ATT_OP_EXEC_WRITE_REQ},
	{BT_GATT_CHRC_WRITE, BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_ATT_OP_WRITE_CMD},
	{BT_GATT_CHRC_NOTIFY, BT_GATT_CHRC_NOTIFY, BT_ATT_OP_NOTIFY},
	{BT_GATT_CHRC_INDICATE, BT_GATT_CHRC_INDICATE, BT_ATT_OP_INDICATE},
};

/* Protected Resource Uses feature bits, fixed once the resources are bound. */
static uint32_t rmap_feature_bits;

/* Whether more than one map is registered. */
static bool rmap_multiple_maps;

/* Restriction map shared by all connections, NULL before init. */
static atomic_ptr_t rmap_active_map;

/* Operations a characteristic offers, given its properties. */
static uint8_t rmap_offered_operations(uint8_t props)
{
	uint8_t operations = 0U;

	ARRAY_FOR_EACH_PTR(rmap_char_opcodes, entry) {
		if ((props & entry->prop) != 0U) {
			operations |= entry->operation;
		}
	}

	return operations;
}

const struct bt_acs_restriction_map *acs_rmap_lookup(uint16_t map_id)
{
	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		if (map->map_id == map_id) {
			return map;
		}
	}

	return NULL;
}

const struct bt_acs_rmap_resource *
acs_rmap_resource_by_handle(const struct bt_acs_restriction_map *map, uint16_t resource_handle)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (res->map == map && res->bound.resource_handle == resource_handle) {
			return res;
		}
	}

	return NULL;
}

const struct bt_acs_rmap_resource *
acs_rmap_resource_by_attr(const struct bt_acs_restriction_map *map, const struct bt_gatt_attr *attr)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (res->map == map && res->bound.value_attr == attr) {
			return res;
		}
	}

	return NULL;
}

uint16_t acs_rmap_char_isc(const struct bt_acs_rmap_resource *resource, uint8_t operation)
{
	if (resource->kind == BT_ACS_RMAP_RESOURCE_CHAR &&
	    (resource->operations & operation) != 0U) {
		return resource->isc_id;
	}

	return BT_ACS_ISC_ID_NONE;
}

/* Return true when a Protected Control Point record lists procedure. */
static bool rmap_lists_procedure(const struct bt_acs_rmap_resource *resource, uint16_t procedure)
{
	for (uint8_t i = 0; i < resource->num_procedures; i++) {
		if (resource->procedures[i] == procedure) {
			return true;
		}
	}

	return false;
}

bool acs_rmap_resource_needs_handle(const struct bt_gatt_attr *attr, const struct bt_uuid *uuid)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (bt_uuid_cmp(res->char_uuid, uuid) == 0) {
			return true;
		}
	}

	/* ACS attributes are governed by their own procedure policy, not a map default. */
	if (acs_attr_is_own(attr)) {
		return false;
	}

	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		if (map->default_isc_id != BT_ACS_ISC_ID_NONE) {
			return true;
		}
	}

	return false;
}

/*
 * Whether the map's default ISC governs an attribute no record lists. It covers
 * characteristic values, the resources a restriction map record names (Table
 * 4.19); declarations and descriptors, CCCDs included, are never governed.
 */
static bool rmap_default_protects(const struct bt_acs_restriction_map *map,
				  const struct bt_gatt_attr *attr)
{
	if (map->default_isc_id == BT_ACS_ISC_ID_NONE ||
	    bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY) == 0 ||
	    bt_uuid_cmp(attr->uuid, BT_UUID_GATT_SECONDARY) == 0) {
		return false;
	}

	/*
	 * Every other attribute follows its service declaration (Core Vol 3 Part G
	 * 3.1), and a value directly follows its characteristic declaration (3.3).
	 */
	return bt_uuid_cmp((attr - 1)->uuid, BT_UUID_GATT_CHRC) == 0;
}

uint16_t acs_rmap_attr_isc(const struct bt_acs_restriction_map *map,
			   const struct bt_gatt_attr *attr, uint8_t operation,
			   uint16_t *resource_handle)
{
	const struct bt_acs_rmap_resource *res = acs_rmap_resource_by_attr(map, attr);

	if (res == NULL) {
		return rmap_default_protects(map, attr) ? map->default_isc_id : BT_ACS_ISC_ID_NONE;
	}

	if (resource_handle != NULL) {
		*resource_handle = res->bound.resource_handle;
	}

	return acs_rmap_char_isc(res, operation);
}

uint16_t acs_rmap_cp_opcode_isc_id(const struct bt_acs_restriction_map *map, uint8_t opcode)
{
	const struct bt_acs_rmap_resource *res = acs_rmap_resource_by_attr(map, acs_attr_cp());

	/* Never protected (§4.4.3.3). */
	if (opcode == BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST) {
		return BT_ACS_ISC_ID_NONE;
	}

	if (res != NULL && res->kind == BT_ACS_RMAP_RESOURCE_CP &&
	    rmap_lists_procedure(res, opcode)) {
		return res->isc_id;
	}

	/* A procedure the map does not list takes its default ISC (§4.4.3.2). */
	return map->default_isc_id;
}

bool acs_rmap_is_active_descriptor_protected(const struct bt_acs_restriction_map *active_map,
					     uint16_t *isc_id)
{
	static const uint8_t descriptor_opcodes[] = {
		BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR,
		BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR,
	};
	uint16_t request_isc;

	__ASSERT_NO_MSG(isc_id != NULL);
	if (active_map == NULL) {
		/* No map means no usable ISC; plain descriptor access is still denied. */
		*isc_id = BT_ACS_ISC_ID_NONE;
		return true;
	}

	/* The restriction map is protected by its own ISC (§4.4.3.3). */
	request_isc = active_map->map_isc_id;

	for (size_t i = 0U; i < ARRAY_SIZE(descriptor_opcodes); i++) {
		uint16_t desc_isc = acs_rmap_cp_opcode_isc_id(active_map, descriptor_opcodes[i]);

		if (desc_isc == BT_ACS_ISC_ID_NONE) {
			continue;
		}
		if (request_isc != BT_ACS_ISC_ID_NONE && request_isc != desc_isc) {
			/* Conflicting ISCs: protected, but no single ISC can serve it. */
			*isc_id = BT_ACS_ISC_ID_NONE;
			return true;
		}

		request_isc = desc_isc;
	}

	*isc_id = request_isc;
	return request_isc != BT_ACS_ISC_ID_NONE;
}

uint32_t acs_rmap_protected_resource_feature_bits(void)
{
	return rmap_feature_bits;
}

bool acs_rmap_multiple_supported(void)
{
	return rmap_multiple_maps;
}

const struct bt_acs_restriction_map *acs_rmap_get_active_map(void)
{
	return atomic_ptr_get(&rmap_active_map);
}

uint16_t acs_rmap_get_active_map_id(void)
{
	const struct bt_acs_restriction_map *map = acs_rmap_get_active_map();

	return (map != NULL) ? map->map_id : BT_ACS_RMAP_ID_NONE;
}

int acs_rmap_activate_map(uint16_t map_id)
{
	const struct bt_acs_restriction_map *map = acs_rmap_lookup(map_id);
	uint16_t descriptor_isc;

	if (map == NULL) {
		return -ENOENT;
	}

	(void)atomic_ptr_set(&rmap_active_map, (atomic_ptr_val_t)map);

	LOG_INF("Active restriction map 0x%04x: map_isc=0x%04x, descriptors %s", map->map_id,
		map->map_isc_id,
		acs_rmap_is_active_descriptor_protected(map, &descriptor_isc)
			? "protected (Data In)"
			: "unprotected (plain ACS CP)");

	return 0;
}

/* Number of ATT opcodes a Protected Characteristic record lists for operations and props. */
static uint8_t rmap_char_entry_count(uint8_t operations, uint8_t props)
{
	uint8_t count = 0U;

	ARRAY_FOR_EACH_PTR(rmap_char_opcodes, entry) {
		if ((operations & entry->operation) != 0U && (props & entry->prop) != 0U) {
			count++;
		}
	}

	return count;
}

/*
 * Append a Protected Characteristic record (Tables 4.19, 4.20): every ATT
 * opcode of operations that props offer, each bound to isc_id.
 */
static int rmap_append_char_record(struct net_buf *buf, uint16_t resource_handle,
				   uint8_t operations, uint8_t props, uint16_t isc_id)
{
	uint8_t count = rmap_char_entry_count(operations, props);
	int err;

	if (count == 0U) {
		return -ENOENT;
	}

	err = acs_desc_add_record_header(buf, ACS_RMAP_TYPE_PROTECTED_CHAR, resource_handle,
					 count * sizeof(struct acs_rmap_data_entry));
	if (err) {
		return err;
	}

	ARRAY_FOR_EACH_PTR(rmap_char_opcodes, entry) {
		if ((operations & entry->operation) != 0U && (props & entry->prop) != 0U) {
			net_buf_add_le16(buf, entry->att_opcode);
			net_buf_add_le16(buf, isc_id);
		}
	}

	return 0;
}

/* Append a Protected Control Point record (Tables 4.19, 4.20). */
static int rmap_append_cp_record(struct net_buf *buf, const struct bt_acs_rmap_resource *resource)
{
	int err = acs_desc_add_record_header(
		buf, ACS_RMAP_TYPE_PROTECTED_CP, resource->bound.resource_handle,
		resource->num_procedures * sizeof(struct acs_rmap_data_entry));

	if (err) {
		return err;
	}

	for (uint8_t i = 0; i < resource->num_procedures; i++) {
		net_buf_add_le16(buf, resource->procedures[i]);
		net_buf_add_le16(buf, resource->isc_id);
	}

	return 0;
}

/* Append the record a map lists for one of its resources. */
static int rmap_append_resource_record(struct net_buf *buf,
				       const struct bt_acs_rmap_resource *resource)
{
	if (resource->kind == BT_ACS_RMAP_RESOURCE_CP) {
		return rmap_append_cp_record(buf, resource);
	}

	return rmap_append_char_record(buf, resource->bound.resource_handle, resource->operations,
				       resource->bound.props, resource->isc_id);
}

/*
 * Append the record of a characteristic the map omits: every operation its
 * properties allow, at the map's default ISC.
 */
static int rmap_append_default_record(struct net_buf *buf,
				      const struct acs_rhandle_resource_record *characteristic,
				      uint16_t default_isc_id)
{
	return rmap_append_char_record(buf, characteristic->resource_handle,
				       Z_BT_ACS_RMAP_CHAR_OPERATIONS, characteristic->properties,
				       default_isc_id);
}

/* Every record of map: its ID, its default ISC, then each resource it lists. */
static int rmap_append_whole_map(const struct bt_acs_restriction_map *map, struct net_buf *buf)
{
	int err;

	err = acs_desc_add_record_header(buf, ACS_RMAP_TYPE_ID, map->map_id, 0U);
	if (err) {
		return err;
	}

	err = acs_desc_add_record_header(buf, ACS_RMAP_TYPE_DEFAULT_ISC, map->default_isc_id, 0U);
	if (err) {
		return err;
	}

	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (res->map != map) {
			continue;
		}

		err = rmap_append_resource_record(buf, res);
		if (err) {
			return err;
		}
	}

	return 0;
}

/* The record describing one resource: listed by map, or derived from its default ISC. */
static int rmap_append_one_resource(const struct bt_acs_restriction_map *map,
				    uint16_t resource_handle, struct net_buf *buf)
{
	const struct bt_acs_rmap_resource *res = acs_rmap_resource_by_handle(map, resource_handle);
	struct acs_rhandle_resource_record characteristic;

	if (res != NULL) {
		return rmap_append_resource_record(buf, res);
	}

	/* An omitted resource is protected only when the map's default ISC is nonzero. */
	if (map->default_isc_id == BT_ACS_ISC_ID_NONE ||
	    acs_rhandle_get(resource_handle, &characteristic) != 0) {
		LOG_WRN("no record matched handle filter 0x%04x", resource_handle);
		return -ENOENT;
	}

	return rmap_append_default_record(buf, &characteristic, map->default_isc_id);
}

uint8_t acs_rmap_build_descriptor_response(const struct bt_acs_restriction_map *map,
					   uint16_t handle_filter, struct net_buf *buf)
{
	int err = (handle_filter == ACS_RMAP_FILTER_ALL)
			  ? rmap_append_whole_map(map, buf)
			  : rmap_append_one_resource(map, handle_filter, buf);

	if (err == -ENOENT) {
		return BT_ACS_CP_RESPONSE_NO_RECORDS_FOUND;
	}

	return (err == 0) ? BT_ACS_CP_RESPONSE_SUCCESS : BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
}

/* Append every registered Restriction_Map_ID with its ISC (Table 4.22). */
static int rmap_build_id_list_response(struct net_buf *buf)
{
	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		if (net_buf_tailroom(buf) < sizeof(struct acs_rmap_id_list_entry)) {
			LOG_ERR("Get Restriction Map ID List: buffer too small (map_id=0x%04x)",
				map->map_id);
			return -ENOMEM;
		}

		net_buf_add_le16(buf, map->map_id);
		net_buf_add_le16(buf, map->map_isc_id);
	}

	return 0;
}

uint8_t acs_cp_handle_get_restriction_map_id_list(struct acs_reply *reply,
						  struct net_buf_simple *payload)
{
	int err;

	ARG_UNUSED(payload);

	err = rmap_build_id_list_response(reply->response);

	return (err == 0) ? BT_ACS_CP_RESPONSE_SUCCESS : BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
}

uint8_t acs_cp_handle_get_restriction_map_descriptor(struct acs_reply *reply,
						     struct net_buf_simple *buf)
{
	uint16_t map_id = net_buf_simple_pull_le16(buf);
	uint16_t handle_filter = net_buf_simple_pull_le16(buf);
	const struct bt_acs_restriction_map *map = acs_rmap_lookup(map_id);

	if (map == NULL) {
		return BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE;
	}

	return acs_rmap_build_descriptor_response(map, handle_filter, reply->response);
}

uint8_t acs_cp_handle_activate_restriction_map(struct acs_reply *reply, struct net_buf_simple *buf)
{
	uint16_t map_id = net_buf_simple_pull_le16(buf);
	const struct bt_acs_restriction_map *map = acs_rmap_lookup(map_id);

	/* A protected map is activated only over ACS Data In (§4.4.3.4). */
	if (map != NULL && reply->channel == ACS_REPLY_CP &&
	    map->map_isc_id != BT_ACS_ISC_ID_NONE) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	}

	if (!acs_rmap_multiple_supported()) {
		LOG_ERR("single restriction map registered");
		return BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED;
	}

	if (acs_rmap_get_active_map_id() == map_id) {
		return BT_ACS_CP_RESPONSE_SUCCESS;
	}

	if (acs_rmap_activate_map(map_id) != 0) {
		LOG_WRN("map ID 0x%04x not found", map_id);
		return BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE;
	}

	/* The active map changes Status for every peer. */
	acs_status_schedule_all();

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

/* Map IDs are unique, and map 0 is reserved (§4.4.3.2). */
static int rmap_check_maps(void)
{
	size_t count;

	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		if (map->map_id == BT_ACS_RMAP_ID_NONE) {
			LOG_ERR("reserved restriction map 0 is not registrable");
			return -EINVAL;
		}
		if (acs_rmap_lookup(map->map_id) != map) {
			LOG_ERR("duplicate restriction map ID 0x%04x", map->map_id);
			return -EEXIST;
		}
	}

	STRUCT_SECTION_COUNT(bt_acs_restriction_map, &count);
	rmap_multiple_maps = count > 1U;

	return 0;
}

/*
 * A record names its map and, for a characteristic, operations the access
 * policy can enforce; a Control Point lists its procedures. Clear the GATT
 * binding for rmap_bind().
 */
static int rmap_check_resources(void)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		bool valid = res->map != NULL && res->char_uuid != NULL;

		if (res->kind == BT_ACS_RMAP_RESOURCE_CHAR) {
			valid = valid && res->operations != 0U &&
				(res->operations & ~Z_BT_ACS_RMAP_CHAR_OPERATIONS) == 0U;
		} else {
			valid = valid && res->procedures != NULL && res->num_procedures > 0U;
		}

		if (!valid) {
			LOG_ERR("invalid protected-resource registration");
			return -EINVAL;
		}

		res->bound = (struct bt_acs_rmap_binding){0};
	}

	return 0;
}

/*
 * Bind every resource declaring the UUID of this characteristic value. A UUID
 * that names a second value cannot identify a resource: report it and stop.
 */
static uint8_t rmap_bind_characteristic(const struct acs_rhandle_resource_record *characteristic,
					void *user_data)
{
	const struct bt_acs_rmap_resource **ambiguous = user_data;

	if (characteristic->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE) {
		return BT_GATT_ITER_CONTINUE;
	}

	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (bt_uuid_cmp(res->char_uuid, characteristic->resource_uuid) != 0) {
			continue;
		}
		if (res->bound.value_attr != NULL) {
			*ambiguous = res;
			return BT_GATT_ITER_STOP;
		}

		res->bound = (struct bt_acs_rmap_binding){
			.resource_handle = characteristic->resource_handle,
			.value_attr = characteristic->attr,
			.props = characteristic->properties,
		};
	}

	return BT_GATT_ITER_CONTINUE;
}

/* Bind every resource to its characteristic; a missing or ambiguous UUID fails init. */
static int rmap_bind(void)
{
	const struct bt_acs_rmap_resource *ambiguous = NULL;
	char uuid_str[BT_UUID_STR_LEN];
	bool missing = false;

	acs_rhandle_foreach_resource(rmap_bind_characteristic, &ambiguous);

	if (ambiguous != NULL) {
		bt_uuid_to_str(ambiguous->char_uuid, uuid_str, sizeof(uuid_str));
		LOG_ERR("uuid=%s names more than one characteristic value; a protected resource "
			"must be unambiguous",
			uuid_str);
		return -EEXIST;
	}

	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (res->bound.value_attr != NULL) {
			continue;
		}

		missing = true;
		bt_uuid_to_str(res->char_uuid, uuid_str, sizeof(uuid_str));
		LOG_ERR("could not resolve handle for %s uuid=%s",
			res->kind == BT_ACS_RMAP_RESOURCE_CP ? "control point" : "char", uuid_str);
	}

	return missing ? -ENOENT : 0;
}

/* A characteristic record protects only operations its characteristic offers. */
static int rmap_check_operations(void)
{
	char uuid_str[BT_UUID_STR_LEN];

	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		uint8_t unoffered;

		if (res->kind != BT_ACS_RMAP_RESOURCE_CHAR) {
			continue;
		}

		unoffered = res->operations & ~rmap_offered_operations(res->bound.props);
		if (unoffered == 0U) {
			continue;
		}

		bt_uuid_to_str(res->char_uuid, uuid_str, sizeof(uuid_str));
		LOG_ERR("map 0x%04x: uuid=%s does not offer protected operations 0x%02x",
			res->map->map_id, uuid_str, unoffered);
		return -EINVAL;
	}

	return 0;
}

/* A map describes each resource with at most one record (§4.4.3.2). */
static int rmap_check_listed_once(void)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (acs_rmap_resource_by_handle(res->map, res->bound.resource_handle) != res) {
			LOG_ERR("map 0x%04x lists resource 0x%04x more than once", res->map->map_id,
				res->bound.resource_handle);
			return -EEXIST;
		}
	}

	return 0;
}

/* An ACS CP record does not list Get Restriction Map ID List (§4.4.3.3). */
static int rmap_check_acs_cp_opcodes(void)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (res->bound.value_attr != acs_attr_cp()) {
			continue;
		}

		if (rmap_lists_procedure(res, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST)) {
			LOG_ERR("map 0x%04x: Get Restriction Map ID List cannot be protected",
				res->map->map_id);
			return -EINVAL;
		}
	}

	return 0;
}

/* Octets of the record map lists for resource (rmap_append_resource_record()). */
static size_t rmap_resource_record_size(const struct bt_acs_rmap_resource *resource)
{
	uint8_t entries = resource->num_procedures;

	if (resource->kind == BT_ACS_RMAP_RESOURCE_CHAR) {
		entries = rmap_char_entry_count(resource->operations, resource->bound.props);
	}

	return sizeof(struct acs_desc_rec_hdr) + entries * sizeof(struct acs_rmap_data_entry);
}

/* Restriction Map Descriptor Response with every record of map (rmap_append_whole_map()). */
static size_t rmap_whole_map_response_size(const struct bt_acs_restriction_map *map)
{
	/* Response opcode, Restriction_Map_ID record and Default_ISC_ID record */
	size_t size = 1U + 2U * sizeof(struct acs_desc_rec_hdr);

	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (res->map == map) {
			size += rmap_resource_record_size(res);
		}
	}

	return size;
}

/*
 * Each descriptor response is one message. A client can request any registered
 * map, active or not.
 */
static int rmap_check_response_sizes(void)
{
	size_t count;
	size_t size;

	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		size = rmap_whole_map_response_size(map);
		if (size > ACS_MESSAGE_MAX_SIZE) {
			LOG_ERR("map 0x%04x: Restriction Map Descriptor Response needs %zu octets, "
				"CONFIG_BT_ACS_MAX_MESSAGE_SIZE is %d",
				map->map_id, size, ACS_MESSAGE_MAX_SIZE);
			return -EMSGSIZE;
		}
	}

	STRUCT_SECTION_COUNT(bt_acs_restriction_map, &count);
	size = 1U + count * sizeof(struct acs_rmap_id_list_entry);
	if (size > ACS_MESSAGE_MAX_SIZE) {
		LOG_ERR("Restriction Map ID List Response needs %zu octets, "
			"CONFIG_BT_ACS_MAX_MESSAGE_SIZE is %d",
			size, ACS_MESSAGE_MAX_SIZE);
		return -EMSGSIZE;
	}

	return 0;
}

/*
 * Whether some map protects an ACS Control Point procedure: its own descriptor,
 * a Protected Control Point record, or a default ISC covering the procedures it
 * does not list. Such a procedure is written to ACS Data In and answered on ACS
 * Data Out Indicate (§4.4.3).
 */
static bool rmap_protects_cp_procedure(void)
{
	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		if (map->map_isc_id != BT_ACS_ISC_ID_NONE ||
		    map->default_isc_id != BT_ACS_ISC_ID_NONE) {
			return true;
		}
	}

	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (res->kind == BT_ACS_RMAP_RESOURCE_CP && res->isc_id != BT_ACS_ISC_ID_NONE) {
			return true;
		}
	}

	return false;
}

/*
 * Protected Resource Uses bits (Table 4.60 bits 11-14), which also decide the
 * ACS Data characteristics the server must have (Table 4.1 C.1-C.3). The ACS
 * Control Point is a protected resource when a map protects one of its
 * procedures: written through ACS Data In, answered by indication.
 */
static uint32_t rmap_build_feature_bits(void)
{
	static const struct {
		uint8_t operation;
		uint32_t feature;
	} uses[] = {
		{BT_GATT_CHRC_READ, BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_READ_REQUEST},
		{BT_GATT_CHRC_WRITE, BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_WRITE_REQUEST},
		{BT_GATT_CHRC_NOTIFY, BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_NOTIFICATION},
		{BT_GATT_CHRC_INDICATE, BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_INDICATION},
	};
	uint32_t bits = 0U;

	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		ARRAY_FOR_EACH(uses, i) {
			if (acs_rmap_char_isc(res, uses[i].operation) != BT_ACS_ISC_ID_NONE) {
				bits |= uses[i].feature;
			}
		}
	}

	if (rmap_protects_cp_procedure()) {
		bits |= BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_WRITE_REQUEST |
			BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_INDICATION;
	}

	return bits;
}

static void rmap_log_maps(void)
{
	char uuid_str[BT_UUID_STR_LEN];

	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		LOG_DBG("rmap 0x%04x map_isc=0x%04x default_isc=0x%04x", map->map_id,
			map->map_isc_id, map->default_isc_id);

		STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
			if (res->map != map) {
				continue;
			}

			bt_uuid_to_str(res->char_uuid, uuid_str, sizeof(uuid_str));
			LOG_DBG("  %s resource=0x%04x uuid=%s isc=0x%04x",
				res->kind == BT_ACS_RMAP_RESOURCE_CP ? "CP" : "char",
				res->bound.resource_handle, uuid_str, res->isc_id);
			if (res->kind == BT_ACS_RMAP_RESOURCE_CHAR) {
				LOG_DBG("    operations=0x%02x", res->operations);
			}
			for (uint8_t i = 0; i < res->num_procedures; i++) {
				LOG_DBG("    procedure=0x%04x", res->procedures[i]);
			}
		}
	}
}

int acs_rmap_init(void)
{
	int err;

	err = rmap_check_maps();
	if (err) {
		return err;
	}

	err = rmap_check_resources();
	if (err) {
		return err;
	}

	err = rmap_bind();
	if (err) {
		return err;
	}

	err = rmap_check_operations();
	if (err) {
		return err;
	}

	err = rmap_check_listed_once();
	if (err) {
		return err;
	}

	err = rmap_check_acs_cp_opcodes();
	if (err) {
		return err;
	}

	err = rmap_check_response_sizes();
	if (err != 0) {
		return err;
	}

	rmap_feature_bits = rmap_build_feature_bits();
	rmap_log_maps();

	return 0;
}
