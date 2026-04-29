/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/sys/util.h>

#include "acs_internal.h"
#include "zephyr/sys/byteorder.h"

LOG_MODULE_REGISTER(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static bool acs_runtime_initialized;
static struct acs_conn_ctx acs_runtime_conns[CONFIG_BT_MAX_CONN];
static const struct bt_acs_cb *acs_app_cb;

#ifdef CONFIG_BT_ACS_ACTIVE_RMAP_ID
#define ACS_DEFAULT_ACTIVE_MAP_ID CONFIG_BT_ACS_ACTIVE_RMAP_ID
#else
#define ACS_DEFAULT_ACTIVE_MAP_ID 0U
#endif

struct acs_uuid_resolve_ctx {
	const struct bt_uuid *uuid;
	uint16_t handle;
};

struct acs_attr_lookup_ctx {
	const struct bt_gatt_attr *attr;
	uint16_t handle;
};

static const struct bt_acs_restriction_map *acs_find_map(uint16_t map_id);
static struct bt_acs_rmap_char_reg *acs_find_registration(uint16_t handle);
static uint16_t acs_find_char_value_handle(const struct bt_uuid *uuid);
static uint16_t acs_attr_handle_lookup(const struct bt_gatt_attr *attr);
static int acs_resolve_registered_handles(void);
static int acs_registration_isc_for_direction(const struct bt_acs_rmap_char_reg *reg,
					      enum bt_acs_direction direction);
static bool acs_gatt_read_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr);
static bool acs_gatt_write_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr);
static void acs_runtime_cp_complete_work_handler(struct k_work *work);
static int acs_runtime_unwrap_data_in(struct acs_conn_ctx *conn_ctx, struct acs_frame *frame);
static bool acs_runtime_is_abort_request(const struct acs_frame *frame);
static void acs_runtime_abort_kex_if_active(struct acs_conn_ctx *conn_ctx);
static void acs_runtime_abort_active_proc(struct acs_conn_ctx *conn_ctx);
static int acs_runtime_send_abort_response(struct acs_conn_ctx *conn_ctx, bool secure,
					   uint16_t resource_handle, uint16_t isc_id,
					   uint8_t response_code);
static int acs_runtime_handle_abort_frame(struct acs_conn_ctx *conn_ctx,
					  const struct acs_frame *frame);

static bool acs_runtime_client_nonce_matches_wire(const struct acs_crypto_ctx *crypto,
						  const uint8_t *nonce, size_t nonce_len)
{
	if (!crypto || !nonce || crypto->client_nonce_fixed_len != nonce_len) {
		return false;
	}

	for (size_t i = 0U; i < nonce_len; i++) {
		if (crypto->client_nonce_fixed[nonce_len - 1U - i] != nonce[i]) {
			return false;
		}
	}

	return true;
}

static void acs_runtime_notify_security_invalidated(struct acs_conn_ctx *conn_ctx)
{
	if (!conn_ctx || !conn_ctx->conn) {
		return;
	}

	if ((conn_ctx->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED) != 0U && acs_app_cb &&
	    acs_app_cb->security_invalidated) {
		acs_app_cb->security_invalidated(conn_ctx->conn);
	}
}

static int acs_runtime_sync_persist(struct acs_conn_ctx *conn_ctx)
{
	if (!conn_ctx || !conn_ctx->conn) {
		return -EINVAL;
	}

	if (!conn_ctx->kex.parent_key_valid && !conn_ctx->crypto.session_key_valid &&
	    conn_ctx->crypto.client_nonce_fixed_len == 0U &&
	    conn_ctx->crypto.server_nonce_fixed_len == 0U) {
		return acs_persist_delete_conn(conn_ctx->conn);
	}

	return acs_persist_save_conn(conn_ctx);
}

static const struct bt_gatt_authorization_cb acs_gatt_auth_cb = {
	.read_authorize = acs_gatt_read_authorize,
	.write_authorize = acs_gatt_write_authorize,
};

enum acs_crypto_mode acs_runtime_crypto_mode_default(void)
{
	if (!IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)) {
		return ACS_CRYPTO_MODE_NONE;
	}

	if (IS_ENABLED(CONFIG_BT_ACS_FEAT_CONFIDENTIALITY)) {
		return ACS_CRYPTO_MODE_AEAD;
	}

	if (IS_ENABLED(CONFIG_BT_ACS_FEAT_INTEGRITY)) {
		return ACS_CRYPTO_MODE_INTEGRITY_ONLY;
	}

	return ACS_CRYPTO_MODE_AUTH_ONLY;
}

const struct bt_acs_cb *acs_runtime_callbacks(void)
{
	return acs_app_cb;
}

/**
 * @brief Return a connection slot to its post-init state.
 *
 * This helper is the single teardown point for the runtime's per-connection
 * resources. Future implementations can extend it with procedure abort
 * notifications, key-state invalidation, or deferred work cancellation.
 */
static void acs_runtime_reset_conn(struct acs_conn_ctx *conn_ctx)
{
	struct k_work_sync sync;

	if (!conn_ctx) {
		return;
	}

	(void)acs_persist_save_conn(conn_ctx);

	if ((conn_ctx->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED) != 0U && acs_app_cb &&
	    acs_app_cb->security_invalidated && conn_ctx->conn) {
		acs_app_cb->security_invalidated(conn_ctx->conn);
	}

	if (conn_ctx->conn) {
		bt_conn_unref(conn_ctx->conn);
	}

	acs_crypto_reset(&conn_ctx->crypto);
	acs_kex_reset(&conn_ctx->kex);
	acs_seg_rx_reset(&conn_ctx->cp_rx);
	acs_seg_rx_reset(&conn_ctx->data_rx);
	k_work_cancel_sync(&conn_ctx->cp_complete_work, &sync);
	acs_seg_tx_reset(&conn_ctx->cp_tx);
	acs_seg_tx_reset(&conn_ctx->doi_tx);
	acs_procedure_engine_reset(&conn_ctx->active_proc);
	memset(conn_ctx, 0, sizeof(*conn_ctx));
}

/** See @ref acs_runtime_lookup_conn. */
struct acs_conn_ctx *acs_runtime_lookup_conn(struct bt_conn *conn)
{
	struct acs_conn_ctx *conn_ctx;
	uint8_t index;

	if (!conn) {
		return NULL;
	}

	index = bt_conn_index(conn);
	if (index >= ARRAY_SIZE(acs_runtime_conns)) {
		return NULL;
	}

	conn_ctx = &acs_runtime_conns[index];
	return conn_ctx->conn == conn ? conn_ctx : NULL;
}

/** See @ref acs_runtime_acquire_conn. */
struct acs_conn_ctx *acs_runtime_acquire_conn(struct bt_conn *conn)
{
	struct acs_conn_ctx *conn_ctx;
	uint8_t index;

	if (!conn) {
		return NULL;
	}

	index = bt_conn_index(conn);
	if (index >= ARRAY_SIZE(acs_runtime_conns)) {
		return NULL;
	}

	conn_ctx = &acs_runtime_conns[index];
	if (conn_ctx->conn == conn) {
		return conn_ctx;
	}

	memset(conn_ctx, 0, sizeof(*conn_ctx));
	conn_ctx->conn = bt_conn_ref(conn);
	acs_crypto_init(&conn_ctx->crypto);
	acs_kex_reset(&conn_ctx->kex);
	acs_seg_rx_init(&conn_ctx->cp_rx);
	acs_seg_rx_init(&conn_ctx->data_rx);
	acs_seg_tx_init(&conn_ctx->cp_tx);
	acs_seg_tx_init(&conn_ctx->doi_tx);
	k_work_init(&conn_ctx->cp_complete_work, acs_runtime_cp_complete_work_handler);
	atomic_set(&conn_ctx->proc_busy, 0);
	conn_ctx->status_flags = BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	conn_ctx->active_map_id = ACS_DEFAULT_ACTIVE_MAP_ID;
	acs_persist_restore_conn(conn_ctx);
	return conn_ctx;
}

bool acs_runtime_client_nonce_conflicts(struct bt_conn *exclude_conn, const uint8_t *nonce,
					size_t nonce_len)
{
	for (size_t i = 0U; i < ARRAY_SIZE(acs_runtime_conns); i++) {
		struct acs_conn_ctx *conn_ctx = &acs_runtime_conns[i];

		if (!conn_ctx->conn || conn_ctx->conn == exclude_conn) {
			continue;
		}

		if (acs_runtime_client_nonce_matches_wire(&conn_ctx->crypto, nonce, nonce_len)) {
			return true;
		}
	}

	return false;
}

static void acs_runtime_cp_complete_work_handler(struct k_work *work)
{
	struct acs_conn_ctx *conn_ctx =
		CONTAINER_OF(work, struct acs_conn_ctx, cp_complete_work);

	LOG_DBG("ACS CP complete work: releasing procedure busy state");
	acs_procedure_engine_reset(&conn_ctx->active_proc);
	atomic_set(&conn_ctx->proc_busy, 0);
}

static bool acs_runtime_is_abort_request(const struct acs_frame *frame)
{
	return frame && frame->payload != NULL && frame->payload_len == 1U &&
	       frame->payload[0] == BT_ACS_CP_OPCODE_ABORT;
}

static void acs_runtime_abort_kex_if_active(struct acs_conn_ctx *conn_ctx)
{
	if (!conn_ctx) {
		return;
	}

	if (conn_ctx->kex.state != ACS_KEX_IDLE && conn_ctx->kex.state != ACS_KEX_COMPLETE) {
		acs_kex_reset(&conn_ctx->kex);
	}
}

static void acs_runtime_abort_active_proc(struct acs_conn_ctx *conn_ctx)
{
	if (!conn_ctx) {
		return;
	}

	if (conn_ctx->active_proc.status != ACS_PROC_IDLE) {
		acs_procedure_engine_abort(&conn_ctx->active_proc, -ECANCELED);
		acs_procedure_engine_reset(&conn_ctx->active_proc);
	}

	acs_runtime_abort_kex_if_active(conn_ctx);
}

static int acs_runtime_send_abort_response(struct acs_conn_ctx *conn_ctx, bool secure,
					   uint16_t resource_handle, uint16_t isc_id,
					   uint8_t response_code)
{
	struct acs_procedure *proc;
	int err;

	if (!conn_ctx || !conn_ctx->conn) {
		return -EINVAL;
	}

	proc = &conn_ctx->active_proc;
	memset(proc, 0, sizeof(*proc));
	proc->conn = conn_ctx->conn;
	proc->resource_handle = resource_handle;
	proc->isc_id = isc_id;
	proc->flags = secure ? ACS_PROC_FLAG_SECURE_TRANSPORT : 0U;

	err = acs_cp_domain_send_response_code(proc, BT_ACS_CP_OPCODE_ABORT, response_code);
	if (err != ACS_PROC_STEP_WAIT_IND_CONFIRM) {
		acs_procedure_engine_reset(proc);
		atomic_set(&conn_ctx->proc_busy, 0);
	}

	return err;
}

static int acs_runtime_handle_abort_frame(struct acs_conn_ctx *conn_ctx, const struct acs_frame *frame)
{
	bool cp_lane_busy;
	bool doi_lane_busy;
	bool transport_busy;
	bool secure = frame->source_channel == ACS_SOURCE_DATA_IN_CHANNEL || frame->encrypted;

	if (!conn_ctx || !frame) {
		return -EINVAL;
	}

	cp_lane_busy = conn_ctx->cp_tx.tx_in_flight || conn_ctx->cp_tx.tx_conn != NULL ||
		       k_work_is_pending(&conn_ctx->cp_tx.tx_work);
	doi_lane_busy = conn_ctx->doi_tx.tx_in_flight || conn_ctx->doi_tx.tx_conn != NULL ||
			k_work_is_pending(&conn_ctx->doi_tx.tx_work);
	transport_busy = cp_lane_busy || doi_lane_busy;

	if (!atomic_get(&conn_ctx->proc_busy) && conn_ctx->active_proc.status == ACS_PROC_IDLE &&
	    !transport_busy) {
		return acs_runtime_send_abort_response(conn_ctx, secure, frame->resource_handle,
						       frame->isc_id, 0x07U);
	}

	if (transport_busy) {
		conn_ctx->abort_pending = true;
		conn_ctx->abort_flags = secure ? ACS_PROC_FLAG_SECURE_TRANSPORT : 0U;
		conn_ctx->abort_resource_handle = frame->resource_handle;
		conn_ctx->abort_isc_id = frame->isc_id;
		return ACS_PROC_STEP_WAIT_IND_CONFIRM;
	}

	acs_runtime_abort_active_proc(conn_ctx);
	atomic_set(&conn_ctx->proc_busy, 1);

	return acs_runtime_send_abort_response(conn_ctx, secure, frame->resource_handle,
					       frame->isc_id, 0x01U);
}

static int acs_runtime_unwrap_data_in(struct acs_conn_ctx *conn_ctx, struct acs_frame *frame)
{
	struct net_buf *plain_buf;
	struct net_buf *wire_buf;
	size_t nonce_var_len;
	size_t tag_len;
	size_t plain_len;
	uint16_t handle;
	int err;

	if (!conn_ctx || !frame || !frame->backing_buf || !frame->encrypted) {
		return -EINVAL;
	}

	if (frame->payload_len < ACS_DATA_IN_HDR_SIZE) {
		return -EINVAL;
	}

	if (!acs_crypto_has_session(&conn_ctx->crypto)) {
		LOG_WRN("ACS Data In rejected: no active security session");
		return -EACCES;
	}

	if (frame->isc_id != conn_ctx->crypto.active_isc_id) {
		LOG_WRN("ACS Data In rejected: isc mismatch rx=0x%04x active=0x%04x",
			frame->isc_id, conn_ctx->crypto.active_isc_id);
		return -EPERM;
	}

	nonce_var_len = acs_crypto_nonce_variable_size(&conn_ctx->crypto);
	tag_len = acs_crypto_auth_tag_size(&conn_ctx->crypto);
	if (frame->payload_len < ACS_DATA_IN_HDR_SIZE + nonce_var_len + tag_len) {
		LOG_WRN("ACS Data In rejected: encrypted payload too short len=%u", frame->payload_len);
		return -EINVAL;
	}

	plain_buf = acs_channel_buf_alloc();
	if (!plain_buf) {
		return -ENOMEM;
	}

	wire_buf = frame->backing_buf;
	plain_len = plain_buf->size;
	err = acs_crypto_decrypt(&conn_ctx->crypto, &frame->payload[ACS_DATA_IN_HDR_SIZE],
				 nonce_var_len, &frame->payload[ACS_DATA_IN_HDR_SIZE + nonce_var_len],
				 tag_len,
				 &frame->payload[ACS_DATA_IN_HDR_SIZE + nonce_var_len + tag_len],
				 frame->payload_len - ACS_DATA_IN_HDR_SIZE - nonce_var_len - tag_len,
				 plain_buf->data, &plain_len);
	if (err) {
		LOG_WRN("ACS Data In decrypt failed: %d", err);
		acs_channel_buf_free(plain_buf);
		return err;
	}

	if (plain_len < sizeof(uint16_t)) {
		acs_channel_buf_free(plain_buf);
		return -EINVAL;
	}

	net_buf_add(plain_buf, plain_len);
	handle = sys_get_le16(plain_buf->data);
	frame->resource_handle = handle;
	frame->payload = plain_buf->data + sizeof(uint16_t);
	frame->payload_len = plain_len - sizeof(uint16_t);
	frame->backing_buf = plain_buf;
	acs_channel_buf_free(wire_buf);

	LOG_DBG("ACS Data In unwrapped: isc=0x%04x handle=0x%04x payload_len=%u", frame->isc_id,
		frame->resource_handle, frame->payload_len);

	return 0;
}

/**
 * @brief Drive a normalized frame through router and procedure execution.
 *
 * The runtime owns the frame buffer lifetime and the single active procedure
 * slot in the connection context. Procedure completion releases the slot
 * immediately; confirm-driven flows keep ownership until a later callback.
 */
static int acs_runtime_dispatch_frame(const struct acs_frame *frame)
{
	struct acs_route route;
	struct acs_conn_ctx *conn_ctx;
	struct acs_procedure *proc;
	int err;

	/* TODO: acs_runtime_handle_cp_write() and acs_runtime_handle_data_in()
	 * already acquire conn_ctx before calling this helper. Consider passing
	 * conn_ctx into acs_runtime_dispatch_frame() directly instead of resolving
	 * it again from frame->conn.
	 */
	conn_ctx = acs_runtime_acquire_conn(frame->conn);
	if (!conn_ctx) {
		return -ENOMEM;
	}

	if (acs_runtime_is_abort_request(frame)) {
		err = acs_runtime_handle_abort_frame(conn_ctx, frame);
		goto out;
	}

	if (frame->source_channel == ACS_SOURCE_CP_CHANNEL &&
	    (conn_ctx->cp_tx.tx_in_flight || conn_ctx->cp_tx.tx_conn != NULL ||
	     k_work_is_pending(&conn_ctx->cp_tx.tx_work))) {
		LOG_WRN("CP dispatch rejected: lane busy in_flight=%u pending=%u tx_conn=%p proc_busy=%d",
			conn_ctx->cp_tx.tx_in_flight ? 1U : 0U,
			k_work_is_pending(&conn_ctx->cp_tx.tx_work) ? 1U : 0U,
			(void *)conn_ctx->cp_tx.tx_conn, (int)atomic_get(&conn_ctx->proc_busy));
		return -EBUSY;
	}

	if (!atomic_cas(&conn_ctx->proc_busy, 0, 1)) {
		return -EBUSY;
	}

	err = acs_classify_frame(frame, &route);
	if (err != 0) {
		LOG_WRN("unable to classify frame: %d", err);
		atomic_set(&conn_ctx->proc_busy, 0);
		goto out;
	}

	proc = &conn_ctx->active_proc;
	memset(proc, 0, sizeof(*proc));
	err = acs_build_procedure_for_route(frame, &route, proc);
	if (err) {
		LOG_WRN("procedure build failed: %d", err);
		atomic_set(&conn_ctx->proc_busy, 0);
		goto out;
	}

	err = acs_procedure_engine_start(proc, frame);
	if (err < 0) {
		acs_procedure_engine_abort(proc, err);
		acs_procedure_engine_reset(proc);
		atomic_set(&conn_ctx->proc_busy, 0);
		goto out;
	}

	if (err == ACS_PROC_STEP_WAIT_IND_CONFIRM) {
		goto out;
	}

	if (err != ACS_PROC_RES_COMPLETE) {
		__ASSERT(false, "unexpected procedure step result %d", err);
		err = -EINVAL;
		acs_procedure_engine_abort(proc, err);
	}

	acs_procedure_engine_reset(proc);
	atomic_set(&conn_ctx->proc_busy, 0);
out:
	if (frame->backing_buf) {
		acs_channel_buf_free(frame->backing_buf);
	}

	return err;
}

/** See @ref acs_runtime_init. */
int acs_runtime_init(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(acs_runtime_conns); i++) {
		acs_runtime_reset_conn(&acs_runtime_conns[i]);
	}

	acs_runtime_initialized = true;
	LOG_INF("ACS experimental scaffold initialized");
	return 0;
}

/** See @ref acs_runtime_handle_cp_write. */
int acs_runtime_handle_cp_write(struct bt_conn *conn, const void *buf, uint16_t len)
{
	struct acs_frame frame;
	struct acs_conn_ctx *conn_ctx;
	int err;

	if (!acs_runtime_initialized) {
		return -EACCES;
	}

	conn_ctx = acs_runtime_acquire_conn(conn);
	if (!conn_ctx) {
		return -ENOMEM;
	}

	err = acs_cp_channel_reassemble(conn_ctx, buf, len, &frame);
	if (err) {
		return err == ACS_SEG_RX_PENDING ? 0 : err;
	}

	return acs_runtime_dispatch_frame(&frame);
}

/** See @ref acs_runtime_handle_data_in. */
int acs_runtime_handle_data_in(struct bt_conn *conn, const void *buf, uint16_t len)
{
	struct acs_frame frame;
	struct acs_conn_ctx *conn_ctx;
	int err;

	if (!acs_runtime_initialized) {
		return -EACCES;
	}

	conn_ctx = acs_runtime_acquire_conn(conn);
	if (!conn_ctx) {
		return -ENOMEM;
	}

	err = acs_data_in_channel_reassemble(conn_ctx, buf, len, &frame);
	if (err) {
		return err == ACS_SEG_RX_PENDING ? 0 : err;
	}

	err = acs_runtime_unwrap_data_in(conn_ctx, &frame);
	if (err) {
		if (frame.backing_buf) {
			acs_channel_buf_free(frame.backing_buf);
		}
		return err;
	}

	return acs_runtime_dispatch_frame(&frame);
}

/** See @ref acs_runtime_handle_disconnect. */
void acs_runtime_handle_disconnect(struct bt_conn *conn)
{
	struct acs_conn_ctx *conn_ctx = acs_runtime_lookup_conn(conn);

	if (!conn_ctx) {
		return;
	}

	acs_runtime_reset_conn(conn_ctx);
}

static uint8_t acs_resolve_uuid_handle_cb(const struct bt_gatt_attr *attr, uint16_t handle,
					  void *user_data)
{
	struct acs_uuid_resolve_ctx *ctx = user_data;
	const struct bt_gatt_chrc *chrc;

	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC)) {
		chrc = attr->user_data;
		if (chrc && !bt_uuid_cmp(chrc->uuid, ctx->uuid)) {
			ctx->handle = chrc->value_handle ? chrc->value_handle : (uint16_t)(handle + 1U);
			return BT_GATT_ITER_STOP;
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

static uint16_t acs_find_char_value_handle(const struct bt_uuid *uuid)
{
	struct acs_uuid_resolve_ctx ctx = {
		.uuid = uuid,
		.handle = 0U,
	};

	bt_gatt_foreach_attr(1U, BT_ATT_LAST_ATTRIBUTE_HANDLE, acs_resolve_uuid_handle_cb, &ctx);
	return ctx.handle;
}

static uint8_t acs_lookup_attr_handle_cb(const struct bt_gatt_attr *attr, uint16_t handle, void *user_data)
{
	struct acs_attr_lookup_ctx *ctx = user_data;

	if (attr == ctx->attr) {
		ctx->handle = handle;
		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_CONTINUE;
}

static uint16_t acs_attr_handle_lookup(const struct bt_gatt_attr *attr)
{
	struct acs_attr_lookup_ctx ctx = {
		.attr = attr,
		.handle = 0U,
	};

	if (!attr) {
		return 0U;
	}

	bt_gatt_foreach_attr(1U, BT_ATT_LAST_ATTRIBUTE_HANDLE, acs_lookup_attr_handle_cb, &ctx);
	return ctx.handle;
}

static int acs_resolve_registered_handles(void)
{
	struct bt_acs_rmap_char_reg *reg;

	ARG_UNUSED(reg);
	STRUCT_SECTION_FOREACH(bt_acs_rmap_char_reg, reg) {
		uint16_t handle;

		if (!reg->entry || !reg->char_uuid) {
			continue;
		}

		handle = acs_find_char_value_handle(reg->char_uuid);
		if (handle == 0U) {
			LOG_WRN("failed to resolve handle for UUID");
			return -ENOENT;
		}

		reg->entry->resource_handle = handle;
	}

	return 0;
}

static const struct bt_acs_restriction_map *acs_find_map(uint16_t map_id)
{
	const struct bt_acs_restriction_map *map;

	ARG_UNUSED(map);
	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		if (map->map_id == map_id) {
			return map;
		}
	}

	return NULL;
}

static struct bt_acs_rmap_char_reg *acs_find_registration(uint16_t handle)
{
	struct bt_acs_rmap_char_reg *reg;

	ARG_UNUSED(reg);
	STRUCT_SECTION_FOREACH(bt_acs_rmap_char_reg, reg) {
		if (reg->entry && reg->entry->resource_handle == handle) {
			return reg;
		}
	}

	return NULL;
}

static int acs_registration_isc_for_direction(const struct bt_acs_rmap_char_reg *reg,
					      enum bt_acs_direction direction)
{
	uint16_t opcode;
	uint8_t i;

	if (!reg || !reg->entry || !reg->entry->ops) {
		return -ENOENT;
	}

	switch (direction) {
	case BT_ACS_DIRECTION_READ:
		opcode = BT_ACS_RMAP_OP_ATT_READ;
		break;
	case BT_ACS_DIRECTION_WRITE:
		opcode = BT_ACS_RMAP_OP_ATT_WRITE;
		break;
	case BT_ACS_DIRECTION_NOTIFY:
		opcode = BT_ACS_RMAP_OP_ATT_NOTIFY;
		break;
	case BT_ACS_DIRECTION_INDICATE:
		opcode = BT_ACS_RMAP_OP_ATT_INDICATE;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0U; i < reg->entry->num_ops; i++) {
		if (reg->entry->ops[i].opcode == opcode) {
			return reg->entry->ops[i].isc_id;
		}
	}

	return -ENOENT;
}

static bool acs_gatt_read_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	uint16_t handle = acs_attr_handle_lookup(attr);
	bool permitted = bt_acs_policy_is_permitted(conn, handle, BT_ACS_DIRECTION_READ);

	if (!permitted) {
		LOG_WRN("ACS GATT read denied: handle=0x%04x", handle);
	}

	return permitted;
}

static bool acs_gatt_write_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	uint16_t handle = acs_attr_handle_lookup(attr);
	bool permitted = bt_acs_policy_is_permitted(conn, handle, BT_ACS_DIRECTION_WRITE);

	if (!permitted) {
		LOG_WRN("ACS GATT write denied: handle=0x%04x", handle);
	}

	return permitted;
}

int bt_acs_set_oob_number(struct bt_conn *conn, const uint8_t *oob, uint16_t len)
{
	struct acs_conn_ctx *conn_ctx;

	if (!conn || !oob || len == 0U || len > sizeof(((struct acs_conn_ctx *)0)->input_oob)) {
		return -EINVAL;
	}

	conn_ctx = acs_runtime_acquire_conn(conn);
	if (!conn_ctx) {
		return -ENOMEM;
	}

	memset(conn_ctx->input_oob, 0, sizeof(conn_ctx->input_oob));
	memcpy(conn_ctx->input_oob + sizeof(conn_ctx->input_oob) - len, oob, len);
	conn_ctx->input_oob_len = len;

	return 0;
}

int bt_acs_init(const struct bt_acs_cb *cb)
{
	int err;

	if (acs_runtime_initialized) {
		return -EALREADY;
	}

	acs_app_cb = cb;

	err = acs_runtime_init();
	if (err) {
		LOG_WRN("ACS runtime init failed: %d", err);
		return err;
	}

	err = acs_service_cache_attrs();
	if (err) {
		LOG_WRN("ACS service attr cache failed: %d", err);
		return err;
	}

	err = acs_resolve_registered_handles();
	if (err) {
		LOG_WRN("ACS handle resolution failed: %d", err);
		return err;
	}

	if (IS_ENABLED(CONFIG_BT_ACS_GATT_AUTHORIZATION)) {
		err = bt_gatt_authorization_cb_register(&acs_gatt_auth_cb);
		if (err && err != -ENOSYS) {
			LOG_WRN("ACS GATT authorization registration failed: %d", err);
			return err;
		}
	}

	LOG_DBG("ACS init complete: authorization=%d", IS_ENABLED
		(CONFIG_BT_ACS_GATT_AUTHORIZATION));

	return 0;
}

int bt_acs_invalidate_security(struct bt_conn *conn)
{
	struct acs_conn_ctx *conn_ctx;

	if (!acs_runtime_initialized || !conn) {
		return -EINVAL;
	}

	conn_ctx = acs_runtime_lookup_conn(conn);
	if (!conn_ctx) {
		return -ENOTCONN;
	}

	acs_runtime_notify_security_invalidated(conn_ctx);
	acs_crypto_session_clear(&conn_ctx->crypto);
	acs_kex_reset(&conn_ctx->kex);
	conn_ctx->status_flags &= (uint8_t)~BT_ACS_STATUS_SECURITY_ESTABLISHED;
	memset(conn_ctx->input_oob, 0, sizeof(conn_ctx->input_oob));
	conn_ctx->input_oob_len = 0U;
	(void)acs_persist_delete_conn(conn);

	return 0;
}

int acs_runtime_invalidate_all_security(void)
{
	int err;

	if (!acs_runtime_initialized) {
		return -EINVAL;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(acs_runtime_conns); i++) {
		struct acs_conn_ctx *conn_ctx = &acs_runtime_conns[i];

		if (!conn_ctx->conn) {
			continue;
		}

		acs_runtime_notify_security_invalidated(conn_ctx);
		acs_crypto_session_clear(&conn_ctx->crypto);
		acs_kex_reset(&conn_ctx->kex);
		conn_ctx->status_flags &= (uint8_t)~BT_ACS_STATUS_SECURITY_ESTABLISHED;
		memset(conn_ctx->input_oob, 0, sizeof(conn_ctx->input_oob));
		conn_ctx->input_oob_len = 0U;
	}

	err = acs_persist_delete_all();
	return err;
}

int acs_runtime_invalidate_key(struct bt_conn *conn, uint16_t key_id)
{
	struct acs_conn_ctx *conn_ctx;

	if (!acs_runtime_initialized || !conn) {
		return -EINVAL;
	}

	conn_ctx = acs_runtime_lookup_conn(conn);
	if (!conn_ctx) {
		return -ENOTCONN;
	}

	if (key_id == 0xFFFFU) {
		if (!conn_ctx->kex.parent_key_valid && !conn_ctx->kex.session_key_valid) {
			return -EALREADY;
		}

		acs_runtime_notify_security_invalidated(conn_ctx);
		acs_crypto_session_clear(&conn_ctx->crypto);
		acs_kex_reset(&conn_ctx->kex);
		conn_ctx->status_flags &= (uint8_t)~BT_ACS_STATUS_SECURITY_ESTABLISHED;
		return acs_runtime_sync_persist(conn_ctx);
	}

	if (key_id == 0x0001U) {
		if (!conn_ctx->kex.parent_key_valid) {
			return -EALREADY;
		}

		acs_runtime_notify_security_invalidated(conn_ctx);
		acs_crypto_session_clear(&conn_ctx->crypto);
		acs_kex_reset(&conn_ctx->kex);
		conn_ctx->status_flags &= (uint8_t)~BT_ACS_STATUS_SECURITY_ESTABLISHED;
		return acs_runtime_sync_persist(conn_ctx);
	}

	if (key_id == 0x0002U) {
		if (!conn_ctx->kex.session_key_valid) {
			return -EALREADY;
		}

		acs_runtime_notify_security_invalidated(conn_ctx);
		acs_crypto_session_clear(&conn_ctx->crypto);
		memset(conn_ctx->kex.session_key, 0, sizeof(conn_ctx->kex.session_key));
		conn_ctx->kex.session_key_valid = false;
		conn_ctx->status_flags &= (uint8_t)~BT_ACS_STATUS_SECURITY_ESTABLISHED;
		conn_ctx->kex.state = conn_ctx->kex.parent_key_valid ? ACS_KEX_COMPLETE :
							 ACS_KEX_IDLE;
		return acs_runtime_sync_persist(conn_ctx);
	}

	return -ENOENT;
}

uint8_t bt_acs_status_get(struct bt_conn *conn)
{
	struct acs_conn_ctx *conn_ctx;

	if (!acs_runtime_initialized) {
		return 0U;
	}

	conn_ctx = acs_runtime_lookup_conn(conn);
	if (!conn_ctx) {
		return BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	}

	return conn_ctx->status_flags;
}

int bt_acs_set_restriction_map(struct bt_conn *conn, uint16_t map_id)
{
	struct acs_conn_ctx *conn_ctx;

	if (!conn) {
		return -EINVAL;
	}

	if (!acs_find_map(map_id)) {
		return -ENOENT;
	}

	conn_ctx = acs_runtime_acquire_conn(conn);
	if (!conn_ctx) {
		return -ENOMEM;
	}

	conn_ctx->active_map_id = map_id;
	(void)acs_persist_save_conn(conn_ctx);
	return 0;
}

bool bt_acs_policy_is_permitted(struct bt_conn *conn, uint16_t att_handle,
				enum bt_acs_direction direction)
{
	struct bt_acs_rmap_char_reg *reg;
	struct acs_conn_ctx *conn_ctx;
	int isc_id;

	if (att_handle == 0U) {
		return true;
	}

	reg = acs_find_registration(att_handle);
	if (!reg) {
		return true;
	}

	if (reg->is_cp && !bt_uuid_cmp(reg->char_uuid, BT_UUID_GATT_ACS_CP)) {
		LOG_DBG("ACS policy: allowing ACS CP transport access on handle=0x%04x",
			att_handle);
		return true;
	}

	conn_ctx = acs_runtime_lookup_conn(conn);
	if (conn_ctx &&
	    (conn_ctx->status_flags & BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED) == 0U) {
		return true;
	}

	isc_id = acs_registration_isc_for_direction(reg, direction);
	if (isc_id < 0) {
		LOG_WRN("ACS policy: no ISC for handle=0x%04x dir=%d", att_handle, direction);
		return false;
	}

	if (reg->map_id != 0U) {
		if (!conn_ctx || conn_ctx->active_map_id != reg->map_id) {
			LOG_WRN("ACS policy: map mismatch handle=0x%04x active=0x%04x required=0x%04x",
				att_handle, conn_ctx ? conn_ctx->active_map_id : 0U, reg->map_id);
			return false;
		}
	}

	if (isc_id == BT_ACS_ISC_ID_NONE || isc_id == BT_ACS_ISC_ID_UNENC) {
		return true;
	}

	if (!conn_ctx) {
		LOG_WRN("ACS policy: secure access denied without connection context handle=0x%04x",
			att_handle);
		return false;
	}

	if ((conn_ctx->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED) == 0U) {
		LOG_WRN("ACS policy: secure access denied, security not established handle=0x%04x isc=0x%04x",
			att_handle, isc_id);
	}

	return (conn_ctx->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED) != 0U;
}

static void acs_runtime_disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(reason);
	acs_runtime_handle_disconnect(conn);
}

BT_CONN_CB_DEFINE(acs_runtime_conn_cb) = {
	.disconnected = acs_runtime_disconnected,
};
