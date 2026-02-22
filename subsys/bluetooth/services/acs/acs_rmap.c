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
#include <zephyr/bluetooth/services/acs.h>

#include "acs_cp_operands.h"
#include "acs_rmap.h"
#include "acs_rhandle.h"
#include "acs_isc.h"
#include "acs_key_desc.h"
#include "acs_internal.h"
#include "acs_reply.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

STRUCT_SECTION_START_EXTERN(bt_acs_rmap_entry);
STRUCT_SECTION_END_EXTERN(bt_acs_rmap_entry);

/* State for deriving protected-characteristic records from the GATT table. */
struct rmap_desc_char_ctx {
	struct net_buf *buf;
	struct acs_desc_rec_hdr hdr;
	uint16_t handle_filter;
	uint16_t isc_id; /* ISC applied to records derived from the GATT table */
	bool matched;
	int err;
};

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

/* Build a Protected Characteristic record from the GATT table. */
static uint8_t rmap_emit_derived_entry(const struct acs_rhandle_resource *res, void *user_data)
{
	struct rmap_desc_char_ctx *ctx = user_data;
	struct acs_rmap_data_entry ops[ARRAY_SIZE(rmap_prop_opcodes)];
	uint8_t num_ops = 0;
	uint8_t data_size;

	if (res->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE) {
		return BT_GATT_ITER_CONTINUE;
	}
	if (ctx->handle_filter != ACS_RMAP_FILTER_ALL &&
	    ctx->handle_filter != res->resource_handle) {
		return BT_GATT_ITER_CONTINUE;
	}

	for (size_t i = 0; i < ARRAY_SIZE(rmap_prop_opcodes); i++) {
		if ((res->props & rmap_prop_opcodes[i].prop) != 0U) {
			ops[num_ops].opcode = sys_cpu_to_le16(rmap_prop_opcodes[i].opcode);
			ops[num_ops].isc_id = sys_cpu_to_le16(ctx->isc_id);
			num_ops++;
		}
	}

	if (num_ops == 0U) {
		return BT_GATT_ITER_CONTINUE;
	}

	ctx->matched = true;
	data_size = num_ops * (uint8_t)sizeof(ops[0]);

	if (net_buf_tailroom(ctx->buf) < (int)(sizeof(ctx->hdr) + data_size)) {
		ctx->err = -ENOMEM;
		return BT_GATT_ITER_STOP;
	}

	ctx->hdr.type_id = ACS_RMAP_TYPE_PROTECTED_CHAR;
	ctx->hdr.type_value = sys_cpu_to_le16(res->resource_handle);
	ctx->hdr.data_size = data_size;
	net_buf_add_mem(ctx->buf, &ctx->hdr, sizeof(ctx->hdr));
	net_buf_add_mem(ctx->buf, ops, data_size);

	return BT_GATT_ITER_CONTINUE;
}

/* Build one Protected Characteristic or Control Point record. */
static int rmap_emit_desc_entry(struct rmap_desc_char_ctx *ctx,
				const struct bt_acs_rmap_entry *entry)
{
	const struct bt_acs_rmap_protected *record = entry->record;
	uint8_t data_size;

	if (ctx->handle_filter != ACS_RMAP_FILTER_ALL &&
	    ctx->handle_filter != entry->resource_handle) {
		return 0;
	}

	ctx->matched = true;
	data_size = record->num_ops * (uint8_t)sizeof(struct acs_rmap_data_entry);

	if (net_buf_tailroom(ctx->buf) < (int)(sizeof(ctx->hdr) + data_size)) {
		return -ENOMEM;
	}

	ctx->hdr.type_value = sys_cpu_to_le16(entry->resource_handle);
	ctx->hdr.data_size = data_size;
	net_buf_add_mem(ctx->buf, &ctx->hdr, sizeof(ctx->hdr));

	LOG_DBG("RM record: type_id=0x%02x handle=0x%04x ops=%u", ctx->hdr.type_id,
		entry->resource_handle, record->num_ops);

	for (uint8_t j = 0; j < record->num_ops; j++) {
		struct acs_rmap_data_entry de = {
			.opcode = sys_cpu_to_le16(record->ops[j].opcode),
			.isc_id = sys_cpu_to_le16(record->ops[j].isc_id),
		};

		net_buf_add_mem(ctx->buf, &de, sizeof(de));
	}
	return 0;
}

static struct bt_acs_rmap_runtime *rmap_lookup_mutable(uint16_t map_id)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_runtime, runtime) {
		if (runtime->map->map_id == map_id) {
			return runtime;
		}
	}

	return NULL;
}

static struct bt_acs_rmap_entry *rmap_entry_start(void)
{
	return STRUCT_SECTION_START(bt_acs_rmap_entry);
}

static size_t rmap_entry_count(void)
{
	size_t count;

	STRUCT_SECTION_COUNT(bt_acs_rmap_entry, &count);
	return count;
}

static const struct bt_acs_rmap_entry *rmap_slice_entry(const struct bt_acs_rmap_runtime *runtime,
							uint16_t index)
{
	return &rmap_entry_start()[runtime->first + index];
}

const struct bt_acs_rmap_runtime *acs_rmap_lookup(uint16_t map_id)
{
	return rmap_lookup_mutable(map_id);
}

bool acs_rmap_entry_matches(const struct bt_acs_rmap_entry *entry, uint16_t opcode, uint16_t isc_id)
{
	const struct bt_acs_rmap_protected *record = entry->record;

	for (uint8_t i = 0; i < record->num_ops; i++) {
		if (record->ops[i].opcode == opcode && record->ops[i].isc_id == isc_id) {
			return true;
		}
	}

	return false;
}

static bool acs_rmap_entry_has_any_protected_opcode(const struct bt_acs_rmap_entry *entry)
{
	const struct bt_acs_rmap_protected *record = entry->record;

	for (uint8_t i = 0; i < record->num_ops; i++) {
		if (record->ops[i].isc_id != BT_ACS_ISC_ID_NONE) {
			return true;
		}
	}

	return false;
}

static uint32_t rmap_compute_protected_resource_feature_bits(void)
{
	uint32_t bits = 0;

	STRUCT_SECTION_FOREACH(bt_acs_rmap_entry, entry) {
		const struct bt_acs_rmap_protected *record = entry->record;

		if (record == NULL || record->map_id == BT_ACS_RMAP_ID_NONE) {
			continue;
		}

		if (record->kind == BT_ACS_RMAP_RESOURCE_CP) {
			if (!acs_handle_is_own(entry->attr_handle) &&
			    acs_rmap_entry_has_any_protected_opcode(entry)) {
				bits |= BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_WRITE_REQUEST |
					BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_INDICATION;
			}
			continue;
		}

		if ((entry->protected_dir_mask & BIT(ACS_DIRECTION_READ)) != 0U) {
			bits |= BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_READ_REQUEST;
		}
		if ((entry->protected_dir_mask & BIT(ACS_DIRECTION_WRITE)) != 0U) {
			bits |= BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_WRITE_REQUEST;
		}
		if ((entry->protected_dir_mask & BIT(ACS_DIRECTION_NOTIFY)) != 0U) {
			bits |= BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_NOTIFICATION;
		}
		if ((entry->protected_dir_mask & BIT(ACS_DIRECTION_INDICATE)) != 0U) {
			bits |= BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_INDICATION;
		}
	}

	return bits;
}

/* Cached Protected Resource Uses feature bits. */
static uint32_t acs_protected_resource_feature_bits;
static bool acs_multiple_rmaps;

uint32_t acs_rmap_protected_resource_feature_bits(void)
{
	return acs_protected_resource_feature_bits;
}

bool acs_rmap_multiple_supported(void)
{
	return acs_multiple_rmaps;
}

/* Current server-wide restriction map; reset by bt_acs_init(). */
static atomic_ptr_t acs_active_rmap;

const struct bt_acs_rmap_runtime *acs_rmap_active(void)
{
	return atomic_ptr_get(&acs_active_rmap);
}

uint16_t acs_rmap_active_id(void)
{
	const struct bt_acs_rmap_runtime *runtime = acs_rmap_active();

	return runtime != NULL ? runtime->map->map_id : BT_ACS_RMAP_ID_NONE;
}

static void rmap_activate_runtime(const struct bt_acs_rmap_runtime *runtime)
{
	uint16_t descriptor_isc;
	bool descriptor_protected;

	descriptor_protected = acs_rmap_descriptor_protection(runtime, &descriptor_isc);
	ARG_UNUSED(descriptor_isc);
	atomic_ptr_set(&acs_active_rmap, (void *)runtime);

	LOG_INF("Active restriction map 0x%04x: map_isc=0x%04x, %u protected resource(s), "
		"descriptors %s",
		runtime->map->map_id, runtime->map->map_isc_id, runtime->count,
		descriptor_protected ? "protected (Data In)" : "unprotected (plain ACS CP)");
}

int acs_rmap_activate(uint16_t map_id)
{
	const struct bt_acs_rmap_runtime *runtime = acs_rmap_lookup(map_id);

	if (runtime == NULL) {
		return -ENOENT;
	}

	rmap_activate_runtime(runtime);

	return 0;
}

int acs_rmap_find_entry_by_resource_handle(const struct bt_acs_rmap_runtime *runtime,
					   uint16_t resource_handle,
					   const struct bt_acs_rmap_entry **entry)
{
	uint16_t high;
	uint16_t low = 0U;

	if (runtime == NULL) {
		return -ENOENT;
	}

	high = runtime->count;
	while (low < high) {
		uint16_t mid = low + ((high - low) / 2U);
		const struct bt_acs_rmap_entry *candidate = rmap_slice_entry(runtime, mid);

		if (candidate->resource_handle < resource_handle) {
			low = mid + 1U;
			continue;
		}
		if (candidate->resource_handle > resource_handle) {
			high = mid;
			continue;
		}

		*entry = candidate;
		return 0;
	}

	return -ENOENT;
}

static int acs_rmap_find_entry_by_attr_handle(const struct bt_acs_rmap_runtime *runtime,
					      uint16_t attr_handle,
					      const struct bt_acs_rmap_entry **entry)
{
	uint16_t low = 0U;
	uint16_t high;

	if (runtime == NULL) {
		return -ENOENT;
	}

	if (!IN_RANGE(attr_handle, runtime->att_min, runtime->att_max)) {
		return -ENOENT;
	}

	high = runtime->count;

	while (low < high) {
		uint16_t mid = low + ((high - low) / 2U);
		const struct bt_acs_rmap_entry *candidate = rmap_slice_entry(runtime, mid);

		if (candidate->attr_handle < attr_handle) {
			low = mid + 1U;
			continue;
		}
		if (candidate->attr_handle > attr_handle) {
			high = mid;
			continue;
		}

		*entry = candidate;
		return 0;
	}

	return -ENOENT;
}

uint16_t acs_rmap_cp_opcode_isc(const struct bt_acs_rmap_runtime *runtime, uint16_t cp_handle,
				uint8_t opcode)
{
	const struct bt_acs_rmap_entry *entry;
	const struct bt_acs_rmap_protected *record;

	/* Only explicitly listed Control Point opcodes are protected. */
	if (acs_rmap_find_entry_by_attr_handle(runtime, cp_handle, &entry) != 0 ||
	    entry->record->kind != BT_ACS_RMAP_RESOURCE_CP) {
		return BT_ACS_ISC_ID_NONE;
	}

	record = entry->record;
	for (uint8_t j = 0; j < record->num_ops; j++) {
		if (record->ops[j].opcode == (uint16_t)opcode) {
			return record->ops[j].isc_id;
		}
	}

	return BT_ACS_ISC_ID_NONE;
}

bool acs_rmap_write_requires_protected_transport(const struct bt_acs_rmap_runtime *runtime,
						 uint16_t attr_handle, bool opcode_valid,
						 uint8_t opcode)
{
	const struct bt_acs_rmap_entry *entry;
	const struct bt_acs_rmap_protected *record;

	if (runtime == NULL) {
		return true;
	}

	if (acs_rmap_find_entry_by_attr_handle(runtime, attr_handle, &entry) != 0) {
		return runtime->map->default_isc_id != BT_ACS_ISC_ID_NONE;
	}

	if (entry->record->kind == BT_ACS_RMAP_RESOURCE_CHAR) {
		return (entry->protected_dir_mask & BIT(ACS_DIRECTION_WRITE)) != 0U;
	}

	if (entry->record->kind != BT_ACS_RMAP_RESOURCE_CP) {
		return true;
	}

	record = entry->record;
	for (uint8_t i = 0U; i < record->num_ops; i++) {
		if (record->ops[i].isc_id != BT_ACS_ISC_ID_NONE &&
		    (!opcode_valid || record->ops[i].opcode == opcode)) {
			return true;
		}
	}

	return false;
}

bool acs_rmap_attr_requires_protected_transport(const struct bt_acs_rmap_runtime *runtime,
						uint16_t attr_handle, enum acs_direction direction)
{
	const struct bt_acs_rmap_entry *entry;

	if (runtime == NULL) {
		return true;
	}

	if (acs_rmap_find_entry_by_attr_handle(runtime, attr_handle, &entry) != 0) {
		/* Unlisted handles use the map's default ISC (§3.5.3). */
		return runtime->map->default_isc_id != BT_ACS_ISC_ID_NONE;
	}

	if (entry->record->kind == BT_ACS_RMAP_RESOURCE_CP) {
		return acs_rmap_entry_has_any_protected_opcode(entry);
	}

	return (entry->protected_dir_mask & BIT(direction)) != 0U;
}

int acs_rmap_build_descriptor_response(const struct bt_acs_rmap_runtime *runtime,
				       const struct acs_rmap_get_descriptor_req *req,
				       struct net_buf *buf)
{
	struct rmap_desc_char_ctx char_ctx;
	const struct bt_acs_restriction_map *map;
	struct acs_desc_rec_hdr hdr = {0};
	uint16_t handle_filter;
	uint16_t map_id;
	int err;

	map_id = req->map_id;
	handle_filter = req->resource_handle_filter;

	/* Map 0 has no runtime and only a Restriction Map ID record (Section 4.4.3.2). */
	if (map_id == BT_ACS_RMAP_ID_NONE) {
		if (handle_filter != ACS_RMAP_FILTER_ALL) {
			return -ENOENT;
		}
		if (net_buf_tailroom(buf) < (int)sizeof(hdr)) {
			return -ENOMEM;
		}

		hdr.type_id = ACS_RMAP_TYPE_ID;
		hdr.type_value = sys_cpu_to_le16(BT_ACS_RMAP_ID_NONE);
		net_buf_add_mem(buf, &hdr, sizeof(hdr));
		return 0;
	}

	if (runtime == NULL) {
		LOG_WRN("Restriction map 0x%04x not found", map_id);
		return -ENOENT;
	}
	if (runtime->map->map_id != map_id) {
		return -EINVAL;
	}
	map = runtime->map;

	/* A handle filter includes only its resource record. */
	if (handle_filter == ACS_RMAP_FILTER_ALL) {
		hdr.type_id = ACS_RMAP_TYPE_ID;
		hdr.type_value = sys_cpu_to_le16(map->map_id);

		if (net_buf_tailroom(buf) < (int)sizeof(hdr)) {
			LOG_ERR("Not enough buffer space for RM_ID record");
			return -ENOMEM;
		}

		LOG_DBG("RM record: type_id=0x00 (RM_ID) type_value=0x%04x", map->map_id);
		net_buf_add_mem(buf, &hdr, sizeof(hdr));

		if (net_buf_tailroom(buf) < (int)sizeof(hdr)) {
			LOG_ERR("Not enough buffer space for Default ISC record");
			return -ENOMEM;
		}

		hdr.type_id = ACS_RMAP_TYPE_DEFAULT_ISC;
		hdr.type_value = sys_cpu_to_le16(map->default_isc_id);

		LOG_DBG("RM record: type_id=0x01 (Default ISC) type_value=0x%04x",
			map->default_isc_id);
		net_buf_add_mem(buf, &hdr, sizeof(hdr));
	}

	char_ctx.buf = buf;
	char_ctx.hdr = hdr;
	char_ctx.handle_filter = handle_filter;
	char_ctx.matched = false;
	char_ctx.err = 0;

	/* An empty map describes the GATT table using its default ISC. */
	if (runtime->count == 0U) {
		char_ctx.isc_id = map->default_isc_id;
		acs_rhandle_walk(rmap_emit_derived_entry, &char_ctx);
		err = char_ctx.err;
	} else {
		for (uint16_t i = 0U; i < runtime->count; i++) {
			const struct bt_acs_rmap_entry *entry = rmap_slice_entry(runtime, i);

			char_ctx.hdr.type_id = entry->record->kind == BT_ACS_RMAP_RESOURCE_CP
						       ? ACS_RMAP_TYPE_PROTECTED_CP
						       : ACS_RMAP_TYPE_PROTECTED_CHAR;
			err = rmap_emit_desc_entry(&char_ctx, entry);
			if (err) {
				return err;
			}
		}
	}

	if (err) {
		return err;
	}

	/* Build a record from the default ISC when the handle is not listed. */
	if (handle_filter != ACS_RMAP_FILTER_ALL && !char_ctx.matched && runtime->count > 0U) {
		char_ctx.isc_id = map->default_isc_id;
		acs_rhandle_walk(rmap_emit_derived_entry, &char_ctx);
		if (char_ctx.err) {
			return char_ctx.err;
		}
	}

	if (handle_filter != ACS_RMAP_FILTER_ALL && !char_ctx.matched) {
		LOG_WRN("RM descriptor: no record matched handle filter 0x%04x", handle_filter);
		return -ENOENT;
	}

	return 0;
}

/* Append every registered Restriction_Map_ID to buf. */
static int acs_rmap_build_id_list_response(struct net_buf *buf)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_runtime, runtime) {
		const struct bt_acs_restriction_map *map = runtime->map;

		if (net_buf_tailroom(buf) < (int)sizeof(struct acs_rmap_id_list_entry)) {
			LOG_ERR("Get Restriction Map ID List: buffer too small (map_id=0x%04x)",
				map->map_id);
			return -ENOMEM;
		}

		struct acs_rmap_id_list_entry entry = {
			.map_id = sys_cpu_to_le16(map->map_id),
			.isc_id = sys_cpu_to_le16(map->map_isc_id),
		};

		LOG_DBG("ID list entry: map_id=0x%04x isc_id=0x%04x", map->map_id, map->map_isc_id);
		net_buf_add_mem(buf, &entry, sizeof(entry));
	}

	return 0;
}

static void rmap_dump_entry(const struct bt_acs_rmap_entry *entry)
{
	const struct bt_acs_rmap_protected *record = entry->record;
	char uuid_str[BT_UUID_STR_LEN] = "<unknown>";
	const char *kind;

	bt_uuid_to_str(record->char_uuid, uuid_str, sizeof(uuid_str));
	kind = record->kind == BT_ACS_RMAP_RESOURCE_CP ? "CP" : "char";

	LOG_DBG("protected %s resource=0x%04x att=0x%04x uuid=%s num_ops=%u", kind,
		entry->resource_handle, entry->attr_handle, uuid_str, record->num_ops);
	for (uint8_t j = 0; j < record->num_ops; j++) {
		LOG_DBG("  op=0x%04x -> isc=0x%04x", record->ops[j].opcode, record->ops[j].isc_id);
	}
}

#define ACS_CP_UNPROTECTABLE_ENTRY(_arg, _opcode) _opcode,

static const uint16_t acs_cp_unprotectable_opcodes[] = {
	Z_BT_ACS_CP_UNPROTECTABLE_OPCODES(0, ACS_CP_UNPROTECTABLE_ENTRY)};

/* Reject protected procedures needed to establish security. */
static int rmap_acs_cp_opcodes_valid(const struct bt_acs_rmap_protected *record)
{
	if (record->kind != BT_ACS_RMAP_RESOURCE_CP ||
	    bt_uuid_cmp(record->char_uuid, BT_UUID_GATT_ACS_CP) != 0) {
		return 0;
	}

	for (uint8_t i = 0; i < record->num_ops; i++) {
		for (size_t j = 0; j < ARRAY_SIZE(acs_cp_unprotectable_opcodes); j++) {
			if (record->ops[i].opcode != acs_cp_unprotectable_opcodes[j]) {
				continue;
			}

			LOG_ERR("map 0x%04x protects ACS CP opcode 0x%02x, which must stay "
				"reachable on the plain ACS CP",
				record->map_id, record->ops[i].opcode);
			return -EINVAL;
		}
	}

	return 0;
}

static int rmap_protected_dir_mask(const struct bt_acs_rmap_protected *record, uint8_t *mask)
{
	if (record->kind == BT_ACS_RMAP_RESOURCE_CP) {
		*mask = 0U;
		return 0;
	}

	*mask = 0U;

	for (uint8_t i = 0; i < record->num_ops; i++) {
		uint16_t opcode = record->ops[i].opcode;
		enum acs_direction direction;

		/* Type 0x02 accepts only direct characteristic operations. */
		switch (opcode) {
		case BT_ACS_RMAP_OP_ATT_READ_REQ:
		case BT_ACS_RMAP_OP_ATT_READ_BLOB_REQ:
			direction = ACS_DIRECTION_READ;
			break;
		case BT_ACS_RMAP_OP_ATT_WRITE_REQ:
		case BT_ACS_RMAP_OP_ATT_WRITE_CMD:
		case BT_ACS_RMAP_OP_ATT_SIGNED_WRITE_CMD:
		case BT_ACS_RMAP_OP_ATT_PREPARE_WRITE_REQ:
		case BT_ACS_RMAP_OP_ATT_EXECUTE_WRITE_REQ:
			direction = ACS_DIRECTION_WRITE;
			break;
		case BT_ACS_RMAP_OP_ATT_NOTIFY:
			direction = ACS_DIRECTION_NOTIFY;
			break;
		case BT_ACS_RMAP_OP_ATT_INDICATE:
			direction = ACS_DIRECTION_INDICATE;
			break;
		default:
			/* Reject operations the local policy cannot enforce. */
			LOG_ERR("unsupported ATT opcode 0x%04x in a protected record", opcode);
			return -EINVAL;
		}

		if (record->ops[i].isc_id != BT_ACS_ISC_ID_NONE) {
			*mask |= BIT(direction);
		}
	}

	return 0;
}

static bool rmap_id_is_duplicate(const struct bt_acs_rmap_runtime *runtime)
{
	STRUCT_SECTION_FOREACH(bt_acs_rmap_runtime, candidate) {
		if (candidate == runtime) {
			return false;
		}
		if (candidate->map->map_id == runtime->map->map_id) {
			return true;
		}
	}

	return false;
}

static bool rmap_entry_after(const struct bt_acs_rmap_entry *a, const struct bt_acs_rmap_entry *b)
{
	uint16_t a_map = a->record->map_id;
	uint16_t b_map = b->record->map_id;

	if (a_map != b_map) {
		return a_map > b_map;
	}
	if (a->attr_handle != b->attr_handle) {
		return a->attr_handle > b->attr_handle;
	}
	if (a->resource_handle != b->resource_handle) {
		return a->resource_handle > b->resource_handle;
	}

	return a->record->kind > b->record->kind;
}

static void rmap_sort_entries(void)
{
	struct bt_acs_rmap_entry *entries = rmap_entry_start();
	size_t count = rmap_entry_count();

	for (size_t i = 1U; i < count; i++) {
		struct bt_acs_rmap_entry entry = entries[i];
		size_t j = i;

		while (j > 0U && rmap_entry_after(&entries[j - 1U], &entry)) {
			entries[j] = entries[j - 1U];
			j--;
		}
		entries[j] = entry;
	}
}

/* Bind map entries to the first GATT resource with their UUID. */
static uint8_t rmap_bind_entry_handles(const struct acs_rhandle_resource *res, void *user_data)
{
	uint16_t *unresolved = user_data;

	if (res->attr_type != ACS_RHANDLE_ATTR_CHAR_VALUE) {
		return BT_GATT_ITER_CONTINUE;
	}

	STRUCT_SECTION_FOREACH(bt_acs_rmap_entry, entry) {
		if (entry->attr_handle != 0U || entry->record == NULL) {
			continue;
		}
		if (bt_uuid_cmp(entry->record->char_uuid, res->uuid) != 0) {
			continue;
		}

		entry->resource_handle = res->resource_handle;
		entry->attr_handle = res->attr_handle;
		entry->value_attr = res->attr;
		entry->props = res->props;
		(*unresolved)--;
	}

	return (*unresolved == 0U) ? BT_GATT_ITER_STOP : BT_GATT_ITER_CONTINUE;
}

int acs_rmap_init_runtime(void)
{
	char uuid_str[BT_UUID_STR_LEN];
	struct bt_acs_rmap_entry *entries = rmap_entry_start();
	size_t entry_count = rmap_entry_count();
	uint16_t unresolved = 0U;
	size_t map_count;
	int failed = 0;

	if (entry_count > UINT16_MAX) {
		LOG_ERR("too many restriction-map entries (%zu)", entry_count);
		return -E2BIG;
	}

	STRUCT_SECTION_COUNT(bt_acs_rmap_runtime, &map_count);
	acs_multiple_rmaps = map_count > 1U;

	STRUCT_SECTION_FOREACH(bt_acs_rmap_runtime, runtime) {
		runtime->first = 0U;
		runtime->count = 0U;
		runtime->att_min = UINT16_MAX;
		runtime->att_max = 0U;
		if (rmap_id_is_duplicate(runtime)) {
			LOG_ERR("duplicate restriction map ID 0x%04x", runtime->map->map_id);
			failed++;
		}
	}

	STRUCT_SECTION_FOREACH(bt_acs_rmap_entry, entry) {
		const struct bt_acs_rmap_protected *record = entry->record;

		entry->resource_handle = 0U;
		entry->attr_handle = 0U;
		entry->value_attr = NULL;
		entry->props = 0U;

		if (record == NULL || record->char_uuid == NULL ||
		    (record->num_ops > 0U && record->ops == NULL)) {
			LOG_ERR("invalid protected-resource registration");
			failed++;
			continue;
		}

		if (record->num_ops > (UINT8_MAX / sizeof(struct acs_rmap_data_entry))) {
			LOG_ERR("map 0x%04x protected resource has too many ops (%u) to encode",
				record->map_id, record->num_ops);
			failed++;
			continue;
		}

		if (!rmap_lookup_mutable(record->map_id)) {
			LOG_ERR("protected resource references unknown map 0x%04x", record->map_id);
			failed++;
			continue;
		}

		if (rmap_acs_cp_opcodes_valid(record) != 0) {
			failed++;
			continue;
		}

		if (rmap_protected_dir_mask(record, &entry->protected_dir_mask) != 0) {
			failed++;
			continue;
		}

		unresolved++;
	}

	if (failed > 0) {
		LOG_ERR("%d restriction-map registration error(s)", failed);
		return -ENOENT;
	}

	acs_rhandle_walk(rmap_bind_entry_handles, &unresolved);

	STRUCT_SECTION_FOREACH(bt_acs_rmap_entry, entry) {
		const struct bt_acs_rmap_protected *record = entry->record;

		if (entry->attr_handle != 0U) {
			continue;
		}

		bt_uuid_to_str(record->char_uuid, uuid_str, sizeof(uuid_str));
		LOG_WRN("could not resolve handle for %s uuid=%s",
			record->kind == BT_ACS_RMAP_RESOURCE_CP ? "CP" : "char", uuid_str);
		failed++;
	}

	if (failed > 0) {
		LOG_ERR("%d restriction-map resolution error(s)", failed);
		return -ENOENT;
	}

	rmap_sort_entries();

	for (size_t i = 0U; i < entry_count; i++) {
		struct bt_acs_rmap_entry *entry = &entries[i];
		struct bt_acs_rmap_runtime *runtime = rmap_lookup_mutable(entry->record->map_id);

		__ASSERT_NO_MSG(runtime != NULL);

		if (i > 0U && entries[i - 1U].record->map_id == entry->record->map_id) {
			const struct bt_acs_rmap_entry *prev = &entries[i - 1U];

			if (prev->resource_handle == entry->resource_handle) {
				LOG_ERR("duplicate map 0x%04x resource handle 0x%04x",
					entry->record->map_id, entry->resource_handle);
				failed++;
				continue;
			}
			if (prev->resource_handle > entry->resource_handle) {
				LOG_ERR("map 0x%04x resource handles are not ordered with GATT "
					"handles",
					entry->record->map_id);
				failed++;
				continue;
			}
		}

		if (runtime->count == 0U) {
			runtime->first = (uint16_t)i;
		}

		runtime->count++;
		runtime->att_min = MIN(runtime->att_min, entry->attr_handle);
		runtime->att_max = MAX(runtime->att_max, entry->attr_handle);
	}

	if (failed > 0) {
		LOG_ERR("%d restriction-map indexing error(s)", failed);
		return -ENOENT;
	}

	STRUCT_SECTION_FOREACH(bt_acs_rmap_runtime, runtime) {
		const struct bt_acs_restriction_map *map = runtime->map;

		LOG_DBG("map_id=0x%04x map_isc=0x%04x default_isc=0x%04x entries=%u first=%u",
			map->map_id, map->map_isc_id, map->default_isc_id, runtime->count,
			runtime->first);

		for (uint16_t i = 0U; i < runtime->count; i++) {
			rmap_dump_entry(rmap_slice_entry(runtime, i));
		}
	}

	acs_protected_resource_feature_bits = rmap_compute_protected_resource_feature_bits();

	return 0;
}

struct acs_cp_result acs_cp_handle_get_restriction_map_id_list(struct acs_reply *reply,
							       struct net_buf_simple *payload)
{
	ARG_UNUSED(payload);

	int err = acs_rmap_build_id_list_response(reply->response);

	if (err) {
		return acs_cp_status(errno_to_acs_status(err));
	}
	return acs_cp_reply();
}

struct acs_cp_result acs_cp_handle_get_restriction_map_descriptor(struct acs_reply *reply,
								  struct net_buf_simple *buf)
{
	struct acs_rmap_get_descriptor_req desc_req;
	const struct bt_acs_rmap_runtime *runtime;
	int err;

	desc_req.map_id = net_buf_simple_pull_le16(buf);
	desc_req.resource_handle_filter = net_buf_simple_pull_le16(buf);

	/* Send errors on the same channel as the request (Appendix A.5). */
	runtime = acs_rmap_lookup(desc_req.map_id);
	if (runtime == NULL) {
		return acs_cp_status(BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	}

	err = acs_rmap_build_descriptor_response(runtime, &desc_req, reply->response);
	if (err) {
		return acs_cp_status(errno_to_acs_status(err));
	}
	return acs_cp_reply();
}

struct acs_cp_result acs_cp_handle_activate_restriction_map(struct acs_reply *reply,
							    struct net_buf_simple *buf)
{
	const struct bt_acs_rmap_runtime *runtime;
	uint16_t map_id;

	if (!acs_rmap_multiple_supported()) {
		LOG_ERR("Activate Restriction Map: only one restriction map registered");
		return acs_cp_status(BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	map_id = net_buf_simple_pull_le16(buf);

	runtime = acs_rmap_lookup(map_id);
	if (runtime == NULL) {
		LOG_WRN("Activate Restriction Map: map ID 0x%04x not found", map_id);
		return acs_cp_status(BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	}

	/* The active map changes Status for every peer. */
	if (acs_rmap_active_id() != map_id) {
		rmap_activate_runtime(runtime);
		acs_status_schedule_all();
	}

	return acs_cp_status(BT_ACS_CP_RESPONSE_SUCCESS);
}

/* Active descriptors: restriction map, ISC, and key descriptor (Section 4.4.3.1). */
bool acs_rmap_descriptor_protection(const struct bt_acs_rmap_runtime *active_rmap, uint16_t *isc_id)
{
	static const uint8_t descriptor_opcodes[] = {
		BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR,
		BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR,
	};
	const struct bt_acs_rmap_entry *cp_entry = NULL;
	uint16_t cp_attr_handle = acs_cp_attr_handle();
	uint16_t isc;
	bool protected = false;

	__ASSERT_NO_MSG(isc_id != NULL);
	if (active_rmap == NULL) {
		*isc_id = BT_ACS_ISC_ID_NONE;
		return true;
	}

	isc = active_rmap->map->map_isc_id;
	protected = isc != BT_ACS_ISC_ID_NONE;
	if (acs_rmap_find_entry_by_attr_handle(active_rmap, cp_attr_handle, &cp_entry) != 0 ||
	    cp_entry->record->kind != BT_ACS_RMAP_RESOURCE_CP) {
		*isc_id = isc;
		return protected;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(descriptor_opcodes); i++) {
		uint16_t op_isc = BT_ACS_ISC_ID_NONE;

		for (uint8_t j = 0U; j < cp_entry->record->num_ops; j++) {
			if (cp_entry->record->ops[j].opcode == descriptor_opcodes[i]) {
				op_isc = cp_entry->record->ops[j].isc_id;
				break;
			}
		}

		if (op_isc == BT_ACS_ISC_ID_NONE) {
			continue;
		}
		protected = true;

		if (isc != BT_ACS_ISC_ID_NONE && isc != op_isc) {
			*isc_id = BT_ACS_ISC_ID_NONE;
			return true;
		}

		isc = op_isc;
	}

	*isc_id = isc;
	return protected;
}
