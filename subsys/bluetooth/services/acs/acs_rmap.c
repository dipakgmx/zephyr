/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_cp.h"
#include "acs_rmap.h"
#include "acs_rhandle.h"
#include "acs_isc.h"
#include "acs_key_desc.h"
#include "acs_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

struct rmap_desc_char_ctx {
	struct net_buf_simple *buf;
	struct acs_rmap_rec_hdr hdr;
	uint16_t handle_filter;
	bool matched;
	int err;
};

typedef bool (*acs_rmap_entry_cb_t)(const struct bt_acs_rmap_protected *entry, void *user_data);

static bool rmap_desc_entry_cb(const struct bt_acs_rmap_protected *entry, void *user_data)
{
	__ASSERT_NO_MSG(user_data != NULL);
	struct rmap_desc_char_ctx *ctx = user_data;

	if (ctx->handle_filter != ACS_RMAP_FILTER_ALL &&
	    ctx->handle_filter != entry->resource_handle) {
		return true;
	}

	ctx->matched = true;

	uint8_t data_size = entry->num_ops * (uint8_t)sizeof(struct acs_rmap_data_entry);

	if (net_buf_simple_tailroom(ctx->buf) < (int)(sizeof(ctx->hdr) + data_size)) {
		ctx->err = -ENOMEM;
		return false;
	}

	/* type_id is pre-set by the caller (PROTECTED_CHAR or PROTECTED_CP) */
	ctx->hdr.type_value = sys_cpu_to_le16(entry->resource_handle);
	ctx->hdr.data_size = data_size;
	net_buf_simple_add_mem(ctx->buf, &ctx->hdr, sizeof(ctx->hdr));

	LOG_DBG("RM record: type_id=0x%02x handle=0x%04x ops=%u", ctx->hdr.type_id,
		entry->resource_handle, entry->num_ops);

	for (uint8_t j = 0; j < entry->num_ops; j++) {
		struct acs_rmap_data_entry de = {
			.opcode = sys_cpu_to_le16(entry->ops[j].opcode),
			.isc_id = sys_cpu_to_le16(entry->ops[j].isc_id),
		};

		net_buf_simple_add_mem(ctx->buf, &de, sizeof(de));
	}
	return true;
}

int acs_rmap_lookup(uint16_t map_id, struct bt_acs_restriction_map *out)
{
	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		if (map->map_id == map_id) {
			*out = *map;
			return 0;
		}
	}
	return -ENOENT;
}

/**
 * @brief Iterate over all registered protected characteristic entries for a given restriction map,
 * invoking a callback for each.
 *
 * @param map The restriction map whose protected characteristic entries should be iterated over.
 * @param cb Callback function
 * @param user_data Pointer passed to callback
 */
static void acs_rmap_foreach_char(const struct bt_acs_restriction_map *map, acs_rmap_entry_cb_t cb,
				  void *user_data)
{
	__ASSERT_NO_MSG(cb);
	__ASSERT_NO_MSG(map);

	STRUCT_SECTION_FOREACH(bt_acs_rmap_char_reg, reg) {
		if (reg->map_id == map->map_id && reg->entry != NULL) {
			if (!cb(reg->entry, user_data)) {
				return;
			}
		}
	}
}

static void acs_rmap_foreach_cp(const struct bt_acs_restriction_map *map, acs_rmap_entry_cb_t cb,
				void *user_data)
{
	__ASSERT_NO_MSG(cb);
	__ASSERT_NO_MSG(map);

	STRUCT_SECTION_FOREACH_ALTERNATE(bt_acs_rmap_cp_reg, bt_acs_rmap_char_reg, reg) {
		if (reg->map_id == map->map_id && reg->entry != NULL) {
			if (!cb(reg->entry, user_data)) {
				return;
			}
		}
	}
}

struct acs_att_direction_ops {
	const uint16_t *ops;
	size_t count;
};

static const uint16_t dir_read_ops[] = {
	BT_ACS_RMAP_OP_ATT_READ_REQ,
	BT_ACS_RMAP_OP_ATT_READ_BLOB_REQ,
	BT_ACS_RMAP_OP_ATT_READ_MULT_REQ,
	BT_ACS_RMAP_OP_ATT_READ_MULT_VL_REQ,
};

static const uint16_t dir_write_ops[] = {
	BT_ACS_RMAP_OP_ATT_WRITE_REQ,         BT_ACS_RMAP_OP_ATT_WRITE_CMD,
	BT_ACS_RMAP_OP_ATT_SIGNED_WRITE_CMD,  BT_ACS_RMAP_OP_ATT_PREPARE_WRITE_REQ,
	BT_ACS_RMAP_OP_ATT_EXECUTE_WRITE_REQ,
};

static const uint16_t dir_notify_ops[] = {
	BT_ACS_RMAP_OP_ATT_NOTIFY,
	BT_ACS_RMAP_OP_ATT_NOTIFY_MULT,
};

static const uint16_t dir_indicate_ops[] = {
	BT_ACS_RMAP_OP_ATT_INDICATE,
};

static const struct acs_att_direction_ops direction_ops[] = {
	[BT_ACS_DIRECTION_READ] =
		{
			.ops = dir_read_ops,
			.count = ARRAY_SIZE(dir_read_ops),
		},
	[BT_ACS_DIRECTION_WRITE] =
		{
			.ops = dir_write_ops,
			.count = ARRAY_SIZE(dir_write_ops),
		},
	[BT_ACS_DIRECTION_NOTIFY] =
		{
			.ops = dir_notify_ops,
			.count = ARRAY_SIZE(dir_notify_ops),
		},
	[BT_ACS_DIRECTION_INDICATE] =
		{
			.ops = dir_indicate_ops,
			.count = ARRAY_SIZE(dir_indicate_ops),
		},
};

static bool acs_att_opcode_matches_direction(uint16_t opcode, enum bt_acs_direction direction)
{
	const uint16_t *ops;
	size_t count;

	__ASSERT_NO_MSG((size_t)direction < ARRAY_SIZE(direction_ops));

	if ((size_t)direction >= ARRAY_SIZE(direction_ops)) {
		return false;
	}

	ops = direction_ops[direction].ops;
	count = direction_ops[direction].count;

	for (size_t i = 0; i < count; i++) {
		if (ops[i] == opcode) {
			return true;
		}
	}

	return false;
}

int acs_rmap_find_protected(uint16_t map_id, uint16_t resource_handle,
			    enum acs_rmap_resource_kind *kind,
			    const struct bt_acs_rmap_protected **entry)
{
	__ASSERT_NO_MSG(kind != NULL);
	__ASSERT_NO_MSG(entry != NULL);

	STRUCT_SECTION_FOREACH(bt_acs_rmap_char_reg, reg) {
		if (reg->map_id == map_id && reg->entry != NULL &&
		    reg->entry->resource_handle == resource_handle) {
			*kind = ACS_RMAP_RESOURCE_CHAR;
			*entry = reg->entry;
			return 0;
		}
	}

	STRUCT_SECTION_FOREACH_ALTERNATE(bt_acs_rmap_cp_reg, bt_acs_rmap_char_reg, reg) {
		if (reg->map_id == map_id && reg->entry != NULL &&
		    reg->entry->resource_handle == resource_handle) {
			*kind = ACS_RMAP_RESOURCE_CP;
			*entry = reg->entry;
			return 0;
		}
	}

	return -ENOENT;
}

bool acs_rmap_cp_opcode_is_protected(uint16_t map_id, uint16_t cp_handle, uint8_t opcode)
{
	const struct bt_acs_rmap_protected *entry;
	enum acs_rmap_resource_kind kind;

	if (acs_rmap_find_protected(map_id, cp_handle, &kind, &entry) != 0 ||
	    kind != ACS_RMAP_RESOURCE_CP) {
		struct bt_acs_restriction_map map;

		if (acs_rmap_lookup(map_id, &map) == 0 &&
		    map.default_isc_id != BT_ACS_ISC_ID_NONE) {
			return true;
		}
		return false;
	}

	for (uint8_t j = 0; j < entry->num_ops; j++) {
		if (entry->ops[j].opcode == (uint16_t)opcode) {
			return entry->ops[j].isc_id != BT_ACS_ISC_ID_NONE;
		}
	}

	return false;
}

bool acs_rmap_char_is_protected(uint16_t map_id, uint16_t att_handle,
				enum bt_acs_direction direction)
{
	const struct bt_acs_rmap_protected *entry;
	enum acs_rmap_resource_kind kind;

	if (acs_rmap_find_protected(map_id, att_handle, &kind, &entry) != 0 ||
	    kind != ACS_RMAP_RESOURCE_CHAR) {
		struct bt_acs_restriction_map map;

		if (acs_rmap_lookup(map_id, &map) == 0 &&
		    map.default_isc_id != BT_ACS_ISC_ID_NONE) {
			return true;
		}
		return false;
	}

	for (uint8_t j = 0; j < entry->num_ops; j++) {
		if (acs_att_opcode_matches_direction(entry->ops[j].opcode, direction) &&
		    entry->ops[j].isc_id != BT_ACS_ISC_ID_NONE) {
			return true;
		}
	}

	return false;
}

void acs_rmap_collect_protected_cccds(struct acs_rmap_cccd_gate *gates, size_t capacity,
				      size_t *count)
{
	size_t gate_count = 0;

	__ASSERT_NO_MSG(gates != NULL);
	__ASSERT_NO_MSG(count != NULL);

	STRUCT_SECTION_FOREACH(bt_acs_rmap_char_reg, reg) {
		bool protect_notify = false;
		bool protect_indicate = false;
		uint16_t cccd;

		if (reg->entry == NULL || reg->entry->resource_handle == 0) {
			continue;
		}

		for (uint8_t j = 0; j < reg->entry->num_ops; j++) {
			uint16_t op = reg->entry->ops[j].opcode;

			if (op == BT_ACS_RMAP_OP_ATT_NOTIFY ||
			    op == BT_ACS_RMAP_OP_ATT_NOTIFY_MULT) {
				protect_notify = true;
			} else if (op == BT_ACS_RMAP_OP_ATT_INDICATE) {
				protect_indicate = true;
			}
		}

		if (!protect_notify && !protect_indicate) {
			continue;
		}

		cccd = acs_rhandle_find_cccd_for_char(reg->entry->resource_handle);
		if (cccd == 0) {
			LOG_WRN("No CCCD found for protected char 0x%04x",
				reg->entry->resource_handle);
			continue;
		}

		for (size_t k = 0; k < gate_count; k++) {
			if (gates[k].cccd_handle == cccd) {
				gates[k].notify_protected |= protect_notify;
				gates[k].indicate_protected |= protect_indicate;
				goto next_reg;
			}
		}

		if (gate_count >= capacity) {
			LOG_WRN("Protected CCCD gate table full (max %zu) - char 0x%04x left "
				"ungated",
				capacity, reg->entry->resource_handle);
			continue;
		}

		gates[gate_count].cccd_handle = cccd;
		gates[gate_count].char_handle = reg->entry->resource_handle;
		gates[gate_count].notify_protected = protect_notify;
		gates[gate_count].indicate_protected = protect_indicate;
		gate_count++;

next_reg:
		continue;
	}

	*count = gate_count;
}

int acs_rmap_build_descriptor_response(const struct acs_rmap_get_descriptor_req *req,
				       struct net_buf_simple *buf)
{
	struct rmap_desc_char_ctx char_ctx;
	struct bt_acs_restriction_map map;
	struct acs_rmap_rec_hdr hdr;
	uint16_t handle_filter;
	uint16_t map_id;
	bool matched;

	map_id = req->map_id;
	handle_filter = req->resource_handle_filter;

	if (acs_rmap_lookup(map_id, &map) != 0) {
		LOG_WRN("Restriction map 0x%04x not found", map_id);
		return -ENOENT;
	}

	hdr.type_id = ACS_RMAP_TYPE_ID;
	hdr.type_value = sys_cpu_to_le16(map.map_id);
	hdr.data_size = 0;

	if (net_buf_simple_tailroom(buf) < (int)sizeof(hdr)) {
		LOG_ERR("Not enough buffer space for RM_ID record");
		return -ENOMEM;
	}

	LOG_DBG("RM record: type_id=0x00 (RM_ID) type_value=0x%04x", map.map_id);
	net_buf_simple_add_mem(buf, &hdr, sizeof(hdr));

	/* Non-zero map IDs include exactly one Default Security Configuration record. */
	if (map.map_id != 0) {
		if (net_buf_simple_tailroom(buf) < (int)sizeof(hdr)) {
			LOG_ERR("Not enough buffer space for Default ISC record");
			return -ENOMEM;
		}

		hdr.type_id = ACS_RMAP_TYPE_DEFAULT_ISC;
		hdr.type_value = sys_cpu_to_le16(map.default_isc_id);
		hdr.data_size = 0;

		LOG_DBG("RM record: type_id=0x01 (Default ISC) type_value=0x%04x",
			map.default_isc_id);
		net_buf_simple_add_mem(buf, &hdr, sizeof(hdr));
	}

	char_ctx.buf = buf;
	char_ctx.hdr = hdr;
	char_ctx.handle_filter = handle_filter;
	char_ctx.matched = false;
	char_ctx.err = 0;

	/* Protected Characteristic records (Type_ID 0x02) */
	char_ctx.hdr.type_id = ACS_RMAP_TYPE_PROTECTED_CHAR;
	acs_rmap_foreach_char(&map, rmap_desc_entry_cb, &char_ctx);

	if (char_ctx.err) {
		return char_ctx.err;
	}

	/* Protected Control Point records (Type_ID 0x03) */
	char_ctx.hdr.type_id = ACS_RMAP_TYPE_PROTECTED_CP;
	acs_rmap_foreach_cp(&map, rmap_desc_entry_cb, &char_ctx);

	if (char_ctx.err) {
		return char_ctx.err;
	}

	matched = char_ctx.matched;

	if (handle_filter != ACS_RMAP_FILTER_ALL && !matched) {
		if (map.default_isc_id == 0) {
			LOG_WRN("RM descriptor: no record matched handle filter 0x%04x",
				handle_filter);
			return -ENOENT;
		}
		LOG_DBG("RM record: handle 0x%04x covered by default ISC 0x%04x", handle_filter,
			map.default_isc_id);
	}

	return 0;
}

int acs_rmap_build_id_list_response(struct net_buf_simple *buf)
{
	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		if (net_buf_simple_tailroom(buf) < (int)sizeof(struct acs_rmap_id_list_entry)) {
			LOG_ERR("Get Restriction Map ID List: buffer too small (map_id=0x%04x)",
				map->map_id);
			return -ENOMEM;
		}

		struct acs_rmap_id_list_entry entry = {
			.map_id = sys_cpu_to_le16(map->map_id),
			.isc_id = sys_cpu_to_le16(map->map_isc_id),
		};

		LOG_DBG("ID list entry: map_id=0x%04x isc_id=0x%04x", map->map_id, map->map_isc_id);
		net_buf_simple_add_mem(buf, &entry, sizeof(entry));
	}

	return 0;
}

static void rmap_dump_entry(const char *kind, const struct bt_acs_rmap_protected *p)
{
	const struct bt_uuid *uuid = NULL;
	char uuid_str[BT_UUID_STR_LEN] = "<unknown>";

	STRUCT_SECTION_FOREACH(bt_acs_rmap_char_reg, reg) {
		if (reg->entry == p) {
			uuid = reg->char_uuid;
			break;
		}
	}
	if (!uuid) {
		STRUCT_SECTION_FOREACH_ALTERNATE(bt_acs_rmap_cp_reg, bt_acs_rmap_char_reg, reg) {
			if (reg->entry == p) {
				uuid = reg->char_uuid;
				break;
			}
		}
	}

	if (uuid) {
		bt_uuid_to_str(uuid, uuid_str, sizeof(uuid_str));
	}

	LOG_DBG("protected %s handle=0x%04x uuid=%s num_ops=%u", kind, p->resource_handle, uuid_str,
		p->num_ops);
	for (uint8_t j = 0; j < p->num_ops; j++) {
		LOG_DBG("  op=0x%04x -> isc=0x%04x", p->ops[j].opcode, p->ops[j].isc_id);
	}
}

static bool rmap_dump_char_cb(const struct bt_acs_rmap_protected *p, void *user_data)
{
	ARG_UNUSED(user_data);
	rmap_dump_entry("char", p);
	return true;
}

static bool rmap_dump_cp_cb(const struct bt_acs_rmap_protected *p, void *user_data)
{
	ARG_UNUSED(user_data);
	rmap_dump_entry("CP  ", p);
	return true;
}

int acs_rmap_resolve_handles(void)
{
	char uuid_str[BT_UUID_STR_LEN];
	int failed = 0;

	STRUCT_SECTION_FOREACH(bt_acs_rmap_char_reg, reg) {
		uint16_t handle = acs_rhandle_find_char_handle(reg->char_uuid);

		if (handle == 0) {
			bt_uuid_to_str(reg->char_uuid, uuid_str, sizeof(uuid_str));
			LOG_WRN("could not resolve handle for char uuid=%s", uuid_str);
			failed++;
		} else {
			reg->entry->resource_handle = handle;
		}
	}

	STRUCT_SECTION_FOREACH_ALTERNATE(bt_acs_rmap_cp_reg, bt_acs_rmap_char_reg, reg) {
		uint16_t handle = acs_rhandle_find_char_handle(reg->char_uuid);

		if (handle == 0) {
			bt_uuid_to_str(reg->char_uuid, uuid_str, sizeof(uuid_str));
			LOG_WRN("could not resolve handle for CP uuid=%s", uuid_str);
			failed++;
		} else {
			reg->entry->resource_handle = handle;
		}
	}

	if (failed > 0) {
		LOG_ERR("%d resource(s) could not be resolved", failed);
		return -ENOENT;
	}

	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		LOG_DBG("map_id=0x%04x map_isc=0x%04x default_isc=0x%04x", map->map_id,
			map->map_isc_id, map->default_isc_id);

		acs_rmap_foreach_char(map, rmap_dump_char_cb, NULL);
		acs_rmap_foreach_cp(map, rmap_dump_cp_cb, NULL);
	}

	return 0;
}

int acs_cp_handle_get_restriction_map_id_list(struct acs_reply *reply)
{
	int err = acs_rmap_build_id_list_response(&reply->response->b);

	if (err) {
		return errno_to_acs_status(err);
	}
	return ACS_CP_RESULT_STAGED_REPLY;
}

int acs_cp_handle_get_restriction_map_descriptor(struct acs_reply *reply,
						 struct net_buf_simple *buf)
{
	struct acs_rmap_get_descriptor_req desc_req;
	struct bt_acs_restriction_map map = {0};
	int err;

	desc_req.map_id = net_buf_simple_pull_le16(buf);
	desc_req.resource_handle_filter = net_buf_simple_pull_le16(buf);

	if (acs_rmap_lookup(desc_req.map_id, &map) != 0) {
		return BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE;
	}

	if (map.map_isc_id != 0 && reply->channel == ACS_REPLY_CP) {
		LOG_WRN("Get Restriction Map Descriptor: protected map 0x%04x requires Data In",
			desc_req.map_id);
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	}

	err = acs_rmap_build_descriptor_response(&desc_req, &reply->response->b);
	if (err) {
		return errno_to_acs_status(err);
	}
	return ACS_CP_RESULT_STAGED_REPLY;
}

int acs_cp_handle_activate_restriction_map(struct acs_reply *reply, struct net_buf_simple *buf)
{
	uint16_t map_id;
	struct bt_acs_restriction_map map;

	if (!IS_ENABLED(CONFIG_BT_ACS_MULTIPLE_RESTRICTION_MAPS)) {
		LOG_ERR("Activate Restriction Map: feature not supported");
		return BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED;
	}

	map_id = net_buf_simple_pull_le16(buf);
	memset(&map, 0, sizeof(map));

	if (acs_rmap_lookup(map_id, &map) != 0) {
		LOG_WRN("Activate Restriction Map: map ID 0x%04x not found", map_id);
		return BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE;
	}

	if (map.map_isc_id != 0 && reply->channel == ACS_REPLY_CP) {
		LOG_WRN("Activate Restriction Map: map 0x%04x is protected (ISC 0x%04x) - "
			"plain CP is not applicable",
			map_id, map.map_isc_id);
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	}

	reply->conn->restriction_map_id = map_id;
	LOG_DBG("Restriction map 0x%04x activated", map_id);

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

#if IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS)

static const uint8_t all_records_filter[2] = {0xFF, 0xFF};

int acs_all_active_step_isc(struct acs_reply *reply)
{
	struct net_buf *buf;
	struct net_buf_simple operand;

	buf = acs_prepare_reply_buf(reply);
	if (!buf) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf,
		       BT_ACS_CP_OPCODE_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR_RESPONSE);
	net_buf_simple_init_with_data(&operand, (void *)all_records_filter,
				      sizeof(all_records_filter));

	if (acs_isc_build_response(&operand, &buf->b) != 0) {
		LOG_ERR("Get All Active Descriptors: ISC build failed");
		acs_cp_rsp_status(reply, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		reply->step = ACS_REPLY_DONE;
		return 0;
	}
	return acs_cp_send_reply(reply);
}

int acs_all_active_step_key(struct acs_reply *reply)
{
	struct net_buf *buf;
	struct net_buf_simple operand;
	int err;

	buf = acs_prepare_reply_buf(reply);
	if (!buf) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_KEY_DESCRIPTOR_RESPONSE);
	net_buf_simple_init_with_data(&operand, (void *)all_records_filter,
				      sizeof(all_records_filter));

	err = acs_key_desc_build_response(&operand, &buf->b, reply->conn);

	if (err == -ENOENT) {
		acs_cp_rsp_status(reply, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
				  BT_ACS_CP_RESPONSE_SUCCESS);
		reply->step = ACS_REPLY_DONE;
		return 0;
	}
	if (err) {
		LOG_ERR("Get All Active Descriptors: Key build failed (%d)", err);
		acs_cp_rsp_status(reply, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		reply->step = ACS_REPLY_DONE;
		return 0;
	}
	return acs_cp_send_reply(reply);
}

int acs_all_active_step_rc(struct acs_reply *reply)
{
	int err = acs_cp_rsp_status(reply, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
				    BT_ACS_CP_RESPONSE_SUCCESS);

	reply->step = ACS_REPLY_DONE;
	return err;
}

int acs_cp_all_active_get(struct acs_reply *reply)
{
	struct acs_rmap_get_descriptor_req rm_operand;
	int err;

	rm_operand.map_id = reply->conn->restriction_map_id;
	rm_operand.resource_handle_filter = ACS_RMAP_FILTER_ALL;
	err = acs_rmap_build_descriptor_response(&rm_operand, &reply->response->b);
	if (err != 0) {
		LOG_ERR("Get All Active Descriptors: RMAP build failed (%d)", err);
		return errno_to_acs_status(err);
	}

	/* Remaining descriptor payloads chain on the confirm-side sequence. */
	reply->step = ACS_REPLY_DESCS_ISC;
	return ACS_CP_RESULT_STAGED_REPLY;
}

#endif /* CONFIG_BT_ACS_DESCRIPTORS */
