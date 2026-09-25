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

#include "acs_cp_operands.h"
#include "acs_rmap.h"
#include "acs_rhandle.h"
#include "acs_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* ATT opcode allowed by each characteristic property (Core Vol 3 Part F §3.4). */
static const struct {
	uint8_t prop;
	uint16_t opcode;
} rmap_prop_opcodes[] = {
	{BT_GATT_CHRC_READ, BT_ACS_RMAP_OP_ATT_READ_REQ},
	{BT_GATT_CHRC_WRITE, BT_ACS_RMAP_OP_ATT_WRITE_REQ},
	{BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_ACS_RMAP_OP_ATT_WRITE_CMD},
	{BT_GATT_CHRC_NOTIFY, BT_ACS_RMAP_OP_ATT_NOTIFY},
	{BT_GATT_CHRC_INDICATE, BT_ACS_RMAP_OP_ATT_INDICATE},
};

/* Protected Resource Uses feature bits, fixed once the resources are bound. */
static uint32_t rmap_feature_bits;

/* Whether more than one map is registered. */
static bool rmap_multiple_maps;

/* Restriction Map ID of the map shared by all connections, 0 before init. */
static atomic_t rmap_active_map_id;

/*
 * Direction of an ATT opcode in a Protected Characteristic record. Return false
 * for an opcode the access policy cannot enforce.
 */
static bool rmap_att_op_direction(uint16_t opcode, enum acs_direction *direction)
{
	switch (opcode) {
	case BT_ACS_RMAP_OP_ATT_READ_REQ:
	case BT_ACS_RMAP_OP_ATT_READ_BLOB_REQ:
		*direction = ACS_DIRECTION_READ;
		return true;
	case BT_ACS_RMAP_OP_ATT_WRITE_REQ:
	case BT_ACS_RMAP_OP_ATT_WRITE_CMD:
	case BT_ACS_RMAP_OP_ATT_SIGNED_WRITE_CMD:
	case BT_ACS_RMAP_OP_ATT_PREPARE_WRITE_REQ:
	case BT_ACS_RMAP_OP_ATT_EXECUTE_WRITE_REQ:
		*direction = ACS_DIRECTION_WRITE;
		return true;
	case BT_ACS_RMAP_OP_ATT_NOTIFY:
		*direction = ACS_DIRECTION_NOTIFY;
		return true;
	case BT_ACS_RMAP_OP_ATT_INDICATE:
		*direction = ACS_DIRECTION_INDICATE;
		return true;
	default:
		return false;
	}
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
acs_rmap_resource_by_attr_handle(const struct bt_acs_restriction_map *map, uint16_t attr_handle)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (res->map == map && res->bound.attr_handle == attr_handle) {
			return res;
		}
	}

	return NULL;
}

const struct bt_acs_rmap_resource *
acs_rmap_resource_by_uuid(const struct bt_acs_restriction_map *map, const struct bt_uuid *uuid)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (res->map == map && res->kind == BT_ACS_RMAP_RESOURCE_CHAR &&
		    bt_uuid_cmp(res->char_uuid, uuid) == 0) {
			return res;
		}
	}

	return NULL;
}

const struct bt_acs_rmap_op_isc *acs_rmap_find_op(const struct bt_acs_rmap_resource *resource,
						  uint16_t opcode)
{
	for (uint8_t i = 0; i < resource->num_ops; i++) {
		if (resource->ops[i].opcode == opcode) {
			return &resource->ops[i];
		}
	}

	return NULL;
}

bool acs_rmap_resource_needs_handle(uint16_t attr_handle, const struct bt_uuid *uuid)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (bt_uuid_cmp(res->char_uuid, uuid) == 0) {
			return true;
		}
	}

	/* ACS attributes are governed by their own procedure policy, not a map default. */
	if (acs_handle_is_own(attr_handle)) {
		return false;
	}

	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		if (map->default_isc_id != BT_ACS_ISC_ID_NONE) {
			return true;
		}
	}

	return false;
}

static bool rmap_has_protected_opcode(const struct bt_acs_rmap_resource *resource)
{
	for (uint8_t i = 0; i < resource->num_ops; i++) {
		if (resource->ops[i].isc_id != BT_ACS_ISC_ID_NONE) {
			return true;
		}
	}

	return false;
}

/* Return true when a Protected Characteristic protects an operation in direction. */
static bool rmap_protects_direction(const struct bt_acs_rmap_resource *resource,
				    enum acs_direction direction)
{
	enum acs_direction op_direction;

	for (uint8_t i = 0; i < resource->num_ops; i++) {
		if (resource->ops[i].isc_id != BT_ACS_ISC_ID_NONE &&
		    rmap_att_op_direction(resource->ops[i].opcode, &op_direction) &&
		    op_direction == direction) {
			return true;
		}
	}

	return false;
}

bool acs_rmap_attr_requires_protected_transport(const struct bt_acs_restriction_map *map,
						uint16_t attr_handle, enum acs_direction direction)
{
	const struct bt_acs_rmap_resource *res = acs_rmap_resource_by_attr_handle(map, attr_handle);

	/* Unlisted handles use the map's default ISC (§3.5.3). */
	if (res == NULL) {
		return map->default_isc_id != BT_ACS_ISC_ID_NONE;
	}

	if (res->kind == BT_ACS_RMAP_RESOURCE_CP) {
		return rmap_has_protected_opcode(res);
	}

	return rmap_protects_direction(res, direction);
}

bool acs_rmap_write_requires_protected_transport(const struct bt_acs_restriction_map *map,
						 uint16_t attr_handle, bool opcode_valid,
						 uint8_t opcode)
{
	const struct bt_acs_rmap_resource *res = acs_rmap_resource_by_attr_handle(map, attr_handle);
	const struct bt_acs_rmap_op_isc *op;

	/* Unlisted handles use the map's default ISC (§3.5.3). */
	if (res == NULL) {
		return map->default_isc_id != BT_ACS_ISC_ID_NONE;
	}

	if (res->kind == BT_ACS_RMAP_RESOURCE_CHAR) {
		return rmap_protects_direction(res, ACS_DIRECTION_WRITE);
	}

	/* Without a usable opcode, require protection if any CP operation does. */
	if (!opcode_valid) {
		return rmap_has_protected_opcode(res);
	}

	/* Missing opcodes and ISC 0 allow plain writes. */
	op = acs_rmap_find_op(res, opcode);

	return op != NULL && op->isc_id != BT_ACS_ISC_ID_NONE;
}

uint16_t acs_rmap_cp_opcode_isc_id(const struct bt_acs_restriction_map *map, uint16_t cp_handle,
				   uint8_t opcode)
{
	const struct bt_acs_rmap_resource *res = acs_rmap_resource_by_attr_handle(map, cp_handle);
	const struct bt_acs_rmap_op_isc *op;

	/* Only explicitly listed Control Point opcodes are protected. */
	if (res == NULL || res->kind != BT_ACS_RMAP_RESOURCE_CP) {
		return BT_ACS_ISC_ID_NONE;
	}

	op = acs_rmap_find_op(res, opcode);

	return op != NULL ? op->isc_id : BT_ACS_ISC_ID_NONE;
}

/* Active descriptors: restriction map, ISC, and key descriptor (§4.4.3.1). */
bool acs_rmap_descriptor_protection(const struct bt_acs_restriction_map *active_map,
				    uint16_t *isc_id)
{
	static const uint8_t descriptor_opcodes[] = {
		BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR,
		BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR,
	};
	const struct bt_acs_rmap_resource *cp;
	uint16_t isc;

	__ASSERT_NO_MSG(isc_id != NULL);
	if (active_map == NULL) {
		/* No map means no usable ISC; plain descriptor access is still denied. */
		*isc_id = BT_ACS_ISC_ID_NONE;
		return true;
	}

	isc = active_map->map_isc_id;
	cp = acs_rmap_resource_by_attr_handle(active_map, acs_cp_attr_handle());
	if (cp == NULL || cp->kind != BT_ACS_RMAP_RESOURCE_CP) {
		*isc_id = isc;
		return isc != BT_ACS_ISC_ID_NONE;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(descriptor_opcodes); i++) {
		const struct bt_acs_rmap_op_isc *op = acs_rmap_find_op(cp, descriptor_opcodes[i]);

		if (op == NULL || op->isc_id == BT_ACS_ISC_ID_NONE) {
			continue;
		}
		if (isc != BT_ACS_ISC_ID_NONE && isc != op->isc_id) {
			/* Conflicting ISCs require protection but have no common ISC. */
			*isc_id = BT_ACS_ISC_ID_NONE;
			return true;
		}

		isc = op->isc_id;
	}

	*isc_id = isc;
	return isc != BT_ACS_ISC_ID_NONE;
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
	return acs_rmap_lookup(acs_rmap_get_active_map_id());
}

uint16_t acs_rmap_get_active_map_id(void)
{
	return (uint16_t)atomic_get(&rmap_active_map_id);
}

int acs_rmap_activate_map(uint16_t map_id)
{
	const struct bt_acs_restriction_map *map = acs_rmap_lookup(map_id);
	uint16_t descriptor_isc;

	if (map == NULL) {
		return -ENOENT;
	}

	atomic_set(&rmap_active_map_id, map_id);

	LOG_INF("Active restriction map 0x%04x: map_isc=0x%04x, descriptors %s", map->map_id,
		map->map_isc_id,
		acs_rmap_descriptor_protection(map, &descriptor_isc)
			? "protected (Data In)"
			: "unprotected (plain ACS CP)");

	return 0;
}

/* Append a Protected Characteristic or Control Point record (Tables 4.19, 4.20). */
static int rmap_append_protected_record(struct net_buf *buf, uint8_t type_id,
					uint16_t resource_handle,
					const struct bt_acs_rmap_op_isc *ops, uint8_t num_ops)
{
	uint8_t data_size = (uint8_t)(num_ops * sizeof(struct acs_rmap_data_entry));
	int err = acs_desc_add_record_header(buf, type_id, resource_handle, data_size);

	if (err) {
		return err;
	}

	for (uint8_t i = 0; i < num_ops; i++) {
		net_buf_add_le16(buf, ops[i].opcode);
		net_buf_add_le16(buf, ops[i].isc_id);
	}

	return 0;
}

/* Append the record a map lists for one of its resources. */
static int rmap_append_resource_record(struct net_buf *buf,
				       const struct bt_acs_rmap_resource *resource)
{
	uint8_t type_id = (resource->kind == BT_ACS_RMAP_RESOURCE_CP)
				  ? ACS_RMAP_TYPE_PROTECTED_CP
				  : ACS_RMAP_TYPE_PROTECTED_CHAR;

	return rmap_append_protected_record(buf, type_id, resource->bound.resource_handle,
					    resource->ops, resource->num_ops);
}

/*
 * Append the record of a characteristic the map omits: every operation its
 * properties allow, at the map's default ISC.
 */
static int rmap_append_default_record(struct net_buf *buf,
				      const struct acs_rhandle_resource *characteristic,
				      uint16_t default_isc_id)
{
	struct bt_acs_rmap_op_isc ops[ARRAY_SIZE(rmap_prop_opcodes)];
	uint8_t num_ops = 0U;

	if (characteristic->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE) {
		return -ENOENT;
	}

	ARRAY_FOR_EACH(rmap_prop_opcodes, i) {
		if ((characteristic->props & rmap_prop_opcodes[i].prop) != 0U) {
			ops[num_ops].opcode = rmap_prop_opcodes[i].opcode;
			ops[num_ops].isc_id = default_isc_id;
			num_ops++;
		}
	}

	if (num_ops == 0U) {
		return -ENOENT;
	}

	return rmap_append_protected_record(buf, ACS_RMAP_TYPE_PROTECTED_CHAR,
					    characteristic->resource_handle, ops, num_ops);
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
	struct acs_rhandle_resource characteristic;

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
	uint16_t map_id;

	ARG_UNUSED(reply);

	if (!acs_rmap_multiple_supported()) {
		LOG_ERR("single restriction map registered");
		return BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED;
	}

	map_id = net_buf_simple_pull_le16(buf);

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
 * A characteristic record lists only opcodes the access policy can enforce.
 * Clear the GATT binding for rmap_bind().
 */
static int rmap_check_resources(void)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		enum acs_direction direction;

		if (res->map == NULL || res->char_uuid == NULL ||
		    (res->num_ops > 0U && res->ops == NULL)) {
			LOG_ERR("invalid protected-resource registration");
			return -EINVAL;
		}

		for (uint8_t i = 0; i < res->num_ops; i++) {
			if (res->kind == BT_ACS_RMAP_RESOURCE_CHAR &&
			    !rmap_att_op_direction(res->ops[i].opcode, &direction)) {
				LOG_ERR("map 0x%04x: ATT opcode 0x%04x cannot be protected",
					res->map->map_id, res->ops[i].opcode);
				return -EINVAL;
			}
		}

		res->bound = (struct bt_acs_rmap_binding){0};
	}

	return 0;
}

/*
 * Bind every resource declaring the UUID of this characteristic value. A UUID
 * that names a second value cannot identify a resource: report it and stop.
 */
static uint8_t rmap_bind_characteristic(const struct acs_rhandle_resource *characteristic,
					void *user_data)
{
	const struct bt_acs_rmap_resource **ambiguous = user_data;

	if (characteristic->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE) {
		return BT_GATT_ITER_CONTINUE;
	}

	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (bt_uuid_cmp(res->char_uuid, characteristic->uuid) != 0) {
			continue;
		}
		if (res->bound.attr_handle != 0U) {
			*ambiguous = res;
			return BT_GATT_ITER_STOP;
		}

		res->bound = (struct bt_acs_rmap_binding){
			.resource_handle = characteristic->resource_handle,
			.attr_handle = characteristic->attr_handle,
			.value_attr = characteristic->attr,
			.props = characteristic->props,
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

	acs_rhandle_foreach(rmap_bind_characteristic, &ambiguous);

	if (ambiguous != NULL) {
		bt_uuid_to_str(ambiguous->char_uuid, uuid_str, sizeof(uuid_str));
		LOG_ERR("uuid=%s names more than one characteristic value; a protected resource "
			"must be unambiguous",
			uuid_str);
		return -EEXIST;
	}

	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (res->bound.attr_handle != 0U) {
			continue;
		}

		missing = true;
		bt_uuid_to_str(res->char_uuid, uuid_str, sizeof(uuid_str));
		LOG_ERR("could not resolve handle for %s uuid=%s",
			res->kind == BT_ACS_RMAP_RESOURCE_CP ? "control point" : "char", uuid_str);
	}

	return missing ? -ENOENT : 0;
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

#define RMAP_OPCODE_ENTRY(_arg, _opcode) _opcode,

/*
 * An ACS CP record lists no procedure a client must reach before it holds a
 * key. BT_ACS_RMAP_ACS_CP_DEFINE() asserts this at build time; this check
 * covers BT_ACS_RMAP_CP_DEFINE() naming the ACS CP UUID.
 */
static int rmap_check_acs_cp_opcodes(void)
{
	static const uint16_t unprotectable[] = {
		Z_BT_ACS_CP_UNPROTECTABLE_OPCODES(_, RMAP_OPCODE_ENTRY)};

	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (res->bound.attr_handle != acs_cp_attr_handle()) {
			continue;
		}

		for (uint8_t i = 0; i < res->num_ops; i++) {
			for (size_t j = 0; j < ARRAY_SIZE(unprotectable); j++) {
				if (res->ops[i].opcode == unprotectable[j]) {
					LOG_ERR("map 0x%04x: ACS CP opcode 0x%02x is unprotectable",
						res->map->map_id, res->ops[i].opcode);
					return -EINVAL;
				}
			}
		}
	}

	return 0;
}

static uint32_t rmap_build_feature_bits(void)
{
	uint32_t bits = 0U;

	STRUCT_SECTION_FOREACH(bt_acs_rmap_resource, res) {
		if (res->kind == BT_ACS_RMAP_RESOURCE_CP) {
			if (!acs_handle_is_own(res->bound.attr_handle) &&
			    rmap_has_protected_opcode(res)) {
				bits |= BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_WRITE_REQUEST |
					BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_INDICATION;
			}
			continue;
		}

		if (rmap_protects_direction(res, ACS_DIRECTION_READ)) {
			bits |= BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_READ_REQUEST;
		}
		if (rmap_protects_direction(res, ACS_DIRECTION_WRITE)) {
			bits |= BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_WRITE_REQUEST;
		}
		if (rmap_protects_direction(res, ACS_DIRECTION_NOTIFY)) {
			bits |= BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_NOTIFICATION;
		}
		if (rmap_protects_direction(res, ACS_DIRECTION_INDICATE)) {
			bits |= BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_INDICATION;
		}
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
			LOG_DBG("  %s resource=0x%04x att=0x%04x uuid=%s",
				res->kind == BT_ACS_RMAP_RESOURCE_CP ? "CP" : "char",
				res->bound.resource_handle, res->bound.attr_handle, uuid_str);
			for (uint8_t i = 0; i < res->num_ops; i++) {
				LOG_DBG("    op=0x%04x -> isc=0x%04x", res->ops[i].opcode,
					res->ops[i].isc_id);
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

	err = rmap_check_listed_once();
	if (err) {
		return err;
	}

	err = rmap_check_acs_cp_opcodes();
	if (err) {
		return err;
	}

	rmap_feature_bits = rmap_build_feature_bits();
	rmap_log_maps();

	return 0;
}
