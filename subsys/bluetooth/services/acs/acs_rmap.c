/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_rmap.h"
#include "acs_rhandle.h"
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

static bool rmap_desc_entry_cb(const struct bt_acs_rmap_protected *entry, void *user_data)
{
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

void acs_rmap_foreach_char(const struct bt_acs_restriction_map *map,
			   bool (*cb)(const struct bt_acs_rmap_protected *prot, void *user_data),
			   void *user_data)
{
	if (!map || !cb) {
		return;
	}

	/* Iterate explicit chars array if provided */
	for (uint8_t i = 0; i < map->num_chars; i++) {
		if (!cb(map->chars[i], user_data)) {
			return;
		}
	}

	/* Also iterate iterable section entries matching this map_id.
	 * This supports BT_ACS_RMAP_PROTECT_CHAR_IN_MAP() which registers
	 * characteristics by map_id instead of an explicit pointer array.
	 * Skip CP entries (is_cp == true) — those are handled by acs_rmap_foreach_cp().
	 */
	STRUCT_SECTION_FOREACH(bt_acs_rmap_char_reg, reg) {
		if (reg->map_id == map->map_id && reg->entry && !reg->is_cp) {
			if (!cb(reg->entry, user_data)) {
				return;
			}
		}
	}
}

void acs_rmap_foreach_cp(const struct bt_acs_restriction_map *map, bt_acs_rmap_protected_cb_t cb,
			 void *user_data)
{
	__ASSERT_NO_MSG(cb);
	__ASSERT_NO_MSG(map);

	/* Iterate explicit cps array if provided */
	for (size_t i = 0; i < map->num_cps; i++) {
		if (!cb(map->cps[i], user_data)) {
			return;
		}
	}

	/* Also iterate iterable section CP entries matching this map_id.
	 * This supports BT_ACS_RMAP_PROTECT_CP_IN_MAP().
	 */
	STRUCT_SECTION_FOREACH(bt_acs_rmap_char_reg, reg) {
		if (reg->map_id == map->map_id && reg->entry && reg->is_cp &&
		    !cb(reg->entry, user_data)) {
			return;
		}
	}
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
		return -ERANGE;
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
		if (map.default_isc_id != 0) {
			if (net_buf_simple_tailroom(buf) < (int)sizeof(hdr)) {
				return -ENOMEM;
			}

			hdr.type_id = ACS_RMAP_TYPE_PROTECTED_CHAR;
			hdr.type_value = sys_cpu_to_le16(handle_filter);
			hdr.data_size = 0;
			net_buf_simple_add_mem(buf, &hdr, sizeof(hdr));

			LOG_DBG("RM record: handle 0x%04x mapped to default ISC 0x%04x",
				handle_filter, map.default_isc_id);
		} else {
			LOG_WRN("RM descriptor: no record matched handle filter 0x%04x",
				handle_filter);
			return -ENOENT;
		}
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

static const struct bt_uuid *rmap_lookup_uuid(const struct bt_acs_rmap_protected *p)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_char_reg, reg) {
		if (reg->entry == p) {
			return reg->char_uuid;
		}
	}
	return NULL;
}

static void rmap_dump_entry(const char *kind, const struct bt_acs_rmap_protected *p)
{
	const struct bt_uuid *uuid = rmap_lookup_uuid(p);
	char uuid_str[BT_UUID_STR_LEN] = "<unknown>";

	if (uuid) {
		bt_uuid_to_str(uuid, uuid_str, sizeof(uuid_str));
	}

	LOG_DBG("rmap:   protected %s handle=0x%04x uuid=%s num_ops=%u", kind, p->resource_handle,
		uuid_str, p->num_ops);
	for (uint8_t j = 0; j < p->num_ops; j++) {
		LOG_DBG("rmap:     op=0x%04x -> isc=0x%04x", p->ops[j].opcode, p->ops[j].isc_id);
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
			LOG_WRN("rmap: could not resolve handle for char uuid=%s", uuid_str);
			failed++;
		} else {
			reg->entry->resource_handle = handle;
		}
	}

	if (failed > 0) {
		LOG_ERR("rmap: %d characteristic(s) could not be resolved", failed);
		return -ENOENT;
	}

	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		LOG_DBG("rmap: map_id=0x%04x map_isc=0x%04x default_isc=0x%04x", map->map_id,
			map->map_isc_id, map->default_isc_id);

		acs_rmap_foreach_char(map, rmap_dump_char_cb, NULL);
		acs_rmap_foreach_cp(map, rmap_dump_cp_cb, NULL);
	}

	return 0;
}
