/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_internal.h"
#include "acs_seg.h"
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#include "acs_rmap.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static int acs_runtime_classify_frame(const struct acs_frame *frame, uint16_t map_id,
				      struct acs_route *route)
{
	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(route != NULL);

	if (frame->source_channel == ACS_SRC_CP) {
		route->kind = ACS_ROUTE_ACS_CP;
		return 0;
	}

#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	{
		enum acs_rmap_resource_kind hit;
		const struct bt_acs_rmap_protected *entry;

		if (acs_rmap_find_protected(map_id, frame->resource_handle, &hit, &entry) != 0) {
			LOG_WRN("data-in handle 0x%04x not in restriction map %u",
				frame->resource_handle, map_id);
			return -ENOENT;
		}
		ARG_UNUSED(entry);

		switch (hit) {
		case ACS_RMAP_RESOURCE_CP:
			route->kind = ACS_ROUTE_PROTECTED_SERVICE_CP;
			break;
		case ACS_RMAP_RESOURCE_CHAR:
			route->kind = ACS_ROUTE_PROTECTED_CHAR;
			break;
		default:
			LOG_WRN("data-in handle 0x%04x not in restriction map %u",
				frame->resource_handle, map_id);
			return -ENOENT;
		}

		route->encrypted = true;
		route->resource_handle = frame->resource_handle;
		route->isc_id = frame->isc_id;

		return 0;
	}
#else
	ARG_UNUSED(map_id);
	LOG_WRN("data-in received but authorization disabled");
	return -ENOENT;
#endif
}

static void acs_procedure_take_request_buf(struct acs_procedure *proc, struct bt_acs_conn *acs_conn,
					   const struct acs_frame *frame)
{
	proc->buffers.request_buf = acs_conn->data_rx.buf;
	acs_conn->data_rx.buf = NULL;
	net_buf_pull(proc->buffers.request_buf,
		     (size_t)(frame->payload - proc->buffers.request_buf->data));
	proc->buffers.request_buf->len = frame->payload_len;
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
int acs_runtime_dispatch_protected_cp_frame(struct acs_frame *frame, struct bt_acs_conn *acs_conn)
{
	struct acs_procedure *req_ctx;
	int err;

	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->data_rx.buf != NULL);

	/* Spec-mandated: reject Data In write if DOI CCC not configured. */
	err = acs_doi_ccc_check(frame->conn);
	if (err) {
		LOG_WRN("DOI indications not enabled for protected CP handle 0x%04x",
			frame->resource_handle);
		return ACS_DATA_ERR_CCC_IMPROPER_CONF;
	}

	LOG_DBG("routing handle 0x%04x to CP dispatcher (respond via DOI)", frame->resource_handle);

	req_ctx = acs_procedure_alloc(acs_conn, frame->resource_handle, frame->isc_id);
	if (!req_ctx) {
		LOG_WRN("no free CP request context for handle 0x%04x", frame->resource_handle);
		return -ENOMEM;
	}

	acs_procedure_take_request_buf(req_ctx, acs_conn, frame);

	err = acs_cp_dispatch(frame, acs_conn, req_ctx);

	/* The CP handler ran synchronously and consumed the input bytes already.
	 * Drop the decrypted_request buffer now so it returns to the pool while
	 * we wait for the indication to confirm — acs_req_free will see NULL
	 * and skip the free at refcount-zero.
	 */
	if (req_ctx->buffers.request_buf) {
		acs_buf_free(req_ctx->buffers.request_buf);
		req_ctx->buffers.request_buf = NULL;
	}

	/* If a multi-step reply sequence is active, the sequence now owns the
	 * ALLOC ref and will release it in acs_seq_clear(). Otherwise, drop it
	 * here — the single response is already queued or completed.
	 */
	if (req_ctx->seq_state == ACS_CP_SEQ_IDLE) {
		acs_procedure_release_owner(req_ctx);
	}
	return err;
}

#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
static enum acs_req_access acs_rmap_resolve_request_access(uint16_t map_id, uint16_t handle,
							   uint16_t isc_id, uint16_t payload_len)
{
	const struct bt_acs_rmap_protected *entry;
	enum acs_rmap_resource_kind kind;
	bool can_read = false;
	bool can_write = false;

	if (acs_rmap_find_protected(map_id, handle, &kind, &entry) != 0) {
		return ACS_REQ_ACCESS_UNKNOWN;
	}

	for (uint8_t i = 0; i < entry->num_ops; i++) {
		if (entry->ops[i].isc_id != isc_id) {
			continue;
		}

		switch (entry->ops[i].opcode) {
		case BT_ACS_RMAP_OP_ATT_READ_REQ:
		case BT_ACS_RMAP_OP_ATT_READ_BLOB_REQ:
		case BT_ACS_RMAP_OP_ATT_READ_MULT_REQ:
		case BT_ACS_RMAP_OP_ATT_READ_MULT_VL_REQ:
			can_read = true;
			break;
		case BT_ACS_RMAP_OP_ATT_WRITE_REQ:
		case BT_ACS_RMAP_OP_ATT_WRITE_CMD:
		case BT_ACS_RMAP_OP_ATT_SIGNED_WRITE_CMD:
		case BT_ACS_RMAP_OP_ATT_PREPARE_WRITE_REQ:
		case BT_ACS_RMAP_OP_ATT_EXECUTE_WRITE_REQ:
			can_write = true;
			break;
		default:
			break;
		}
	}

	if (can_write && !can_read) {
		return ACS_REQ_ACCESS_WRITE;
	}
	if (can_read && !can_write) {
		return ACS_REQ_ACCESS_READ;
	}
	if (can_read && can_write) {
		/* Same ISC_ID covers both read and write — use payload as tiebreaker. */
		return (payload_len > 0) ? ACS_REQ_ACCESS_WRITE : ACS_REQ_ACCESS_READ;
	}

	return ACS_REQ_ACCESS_UNKNOWN;
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

int acs_runtime_dispatch_protected_char_frame(struct acs_frame *frame, struct bt_acs_conn *acs_conn)
{
	struct acs_procedure *req_ctx;
	int err;

	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->data_rx.buf != NULL);

	err = acs_require_data_out_subscription(frame->conn, frame->resource_handle,
						frame->payload_len);
	if (err == -EINVAL) {
		LOG_WRN("required data-out CCC not enabled for handle 0x%04x",
			frame->resource_handle);
		return ACS_DATA_ERR_CCC_IMPROPER_CONF;
	}
	if (err) {
		LOG_WRN("unable to resolve data-out path for handle 0x%04x (%d)",
			frame->resource_handle, err);
		return err;
	}

	req_ctx = acs_procedure_alloc(acs_conn, frame->resource_handle, frame->isc_id);
	if (!req_ctx) {
		LOG_WRN("no free request context for handle 0x%04x", frame->resource_handle);
		return -ENOMEM;
	}

	acs_procedure_take_request_buf(req_ctx, acs_conn, frame);

#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	req_ctx->route.access = acs_rmap_resolve_request_access(acs_conn->restriction_map_id,
								frame->resource_handle,
								frame->isc_id, frame->payload_len);
	if (req_ctx->route.access == ACS_REQ_ACCESS_UNKNOWN) {
		LOG_ERR("no read/write op in rmap for handle 0x%04x isc 0x%04x",
			frame->resource_handle, frame->isc_id);
		acs_procedure_release_owner(req_ctx);
		return -ENOENT;
	}
#endif

	acs_request_queue_submit(acs_conn, req_ctx);
	return 0;
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

int acs_runtime_dispatch_frame(struct acs_frame *frame, struct bt_acs_conn *acs_conn)
{
	struct acs_route route = {0};
	uint16_t map_id;
	int err;

	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);

	map_id = (frame->source_channel == ACS_SRC_CP) ? 0U : acs_conn->restriction_map_id;

	err = acs_runtime_classify_frame(frame, map_id, &route);
	if (err == -ENOENT) {
		LOG_WRN("handle 0x%04x not in restriction map", frame->resource_handle);
		return ACS_DATA_ERR_RESOURCE_NOT_PROTECTED;
	}
	if (err) {
		LOG_ERR("frame classification failed: %d", err);
		return err;
	}

	switch (route.kind) {
	case ACS_ROUTE_ACS_CP:
		return acs_cp_dispatch(frame, acs_conn, NULL);
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	case ACS_ROUTE_PROTECTED_SERVICE_CP:
		return acs_runtime_dispatch_protected_cp_frame(frame, acs_conn);
	case ACS_ROUTE_PROTECTED_CHAR:
		return acs_runtime_dispatch_protected_char_frame(frame, acs_conn);
#endif
	default:
		return -EINVAL;
	}
}

ssize_t acs_cp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
		     uint16_t len, uint16_t offset, uint8_t flags)
{
	int ret;
	struct bt_acs_conn *acs_conn;
	enum acs_seg_rx_result seg_rx_result;

	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0) {
		LOG_WRN("unexpected CP write offset %u", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len < 2) {
		LOG_WRN("CP write PDU too short (%u bytes)", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (!acs_is_initialized()) {
		LOG_ERR("CP write received before ACS init");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* Spec-mandated: reject CP write if CP indications not enabled (ATT CCC_IMPROPER_CONF). */
	ret = acs_cp_ccc_check(conn);

	if (ret == -EINVAL) {
		LOG_WRN("CP indications not enabled by client");
		return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
	}
	if (ret) {
		LOG_ERR("CP CCC check failed: %d", ret);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	acs_conn = acs_conn_lookup(conn);

	if (!acs_conn) {
		LOG_ERR("no ACS connection state for conn %p", (void *)conn);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	seg_rx_result = acs_channel_rx_feed(&acs_conn->cp_rx, buf, len);

	switch (seg_rx_result) {
	case ACS_SEG_RX_COMPLETE: {
		struct acs_frame frame = acs_frame_from_cp_rx(conn, acs_conn->cp_rx.buf);
		bool is_abort = false;

#if IS_ENABLED(CONFIG_BT_ACS_ABORT)
		/*
		 * Abort command must preempt any in-progress procedure per §4.4.4: it bypasses the
		 * plain_cp_proc lock and the handler owns the lock semantics itself.
		 */
		is_abort = (frame.payload_len > 0 && frame.payload[0] == BT_ACS_CP_OPCODE_ABORT);
#endif
		if (!is_abort && !atomic_cas(&acs_conn->plain_cp_proc.plain_cp.locked, 0, 1)) {
			acs_seg_rx_reset(&acs_conn->cp_rx);
			LOG_WRN("procedure already in progress");
			return BT_GATT_ERR(BT_ATT_ERR_PROCEDURE_IN_PROGRESS);
		}
		(void)acs_runtime_dispatch_frame(&frame, acs_conn);
		acs_seg_rx_reset(&acs_conn->cp_rx);
		break;
	}
	case ACS_SEG_RX_PENDING:
		break;
	case ACS_SEG_RX_ERR_COUNTER:
		LOG_WRN("invalid CP segment counter");
		return BT_GATT_ERR(BT_ACS_ATT_ERR_INVALID_SEG_COUNTER);
	case ACS_SEG_RX_ERR_OVERFLOW:
		LOG_WRN("CP receive overflow: accumulated payload exceeds buffer");
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	case ACS_SEG_RX_ERR_TIMEOUT:
		LOG_WRN("CP receive inter-segment timeout expired");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	case ACS_SEG_RX_ERR_ORPHAN:
		LOG_WRN("CP continuation segment without prior first segment");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	case ACS_SEG_RX_ERR_LEN:
		LOG_WRN("invalid CP segment length");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	default:
		LOG_ERR("unexpected CP seg_rx result %d", (int)seg_rx_result);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return len;
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/* ATT Write Long not handled; ACS segmentation covers large payloads. */
ssize_t acs_data_in_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags)
{
	struct bt_acs_conn *acs_conn;
	enum acs_seg_rx_result res;
	int err;
	uint16_t attr_handle = attr ? attr->handle : 0U;

	ARG_UNUSED(flags);

	if (offset != 0) {
		LOG_ERR("data-in write with non-zero offset: %u", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len < 1) {
		LOG_ERR("data-in write with invalid length: %u", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (!acs_is_initialized()) {
		LOG_ERR("data-in write received before ACS init");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	acs_conn = acs_conn_lookup(conn);

	if (!acs_conn) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	res = acs_channel_rx_feed(&acs_conn->data_rx, buf, len);

	switch (res) {
	case ACS_SEG_RX_COMPLETE: {
		struct net_buf_simple simple;

		net_buf_simple_init_with_data(&simple, acs_conn->data_rx.buf->data,
					      acs_conn->data_rx.buf->len);

		err = acs_data_in_unwrap_and_route(acs_conn, &simple);

		/*
		 * On success the buffer is either owned by the request context
		 * (regular protected resource) or already freed (protected CP).
		 * On error, release the buffer back to the pool.
		 */
		if (err && acs_conn->data_rx.buf) {
			acs_seg_rx_reset(&acs_conn->data_rx);
		}

		if (err == ACS_DATA_ERR_NOT_AUTHORIZED) {
			LOG_WRN("data-in write to handle 0x%04x rejected: no security established",
				attr_handle);
			return BT_GATT_ERR(BT_ATT_ERR_AUTHORIZATION);
		}
		if (err == ACS_DATA_ERR_INVALID_KEY) {
			LOG_ERR("data-in write to handle 0x%04x failed with invalid key; inner "
				"protected resource handle is unavailable unless decryption "
				"succeeds",
				attr_handle);
			return BT_GATT_ERR(BT_ACS_ATT_ERR_INVALID_KEY);
		}
		if (err == ACS_DATA_ERR_CCC_IMPROPER_CONF) {
			LOG_WRN("data-in write to handle 0x%04x rejected: required CCC not "
				"configured",
				attr_handle);
			return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
		}
		if (err == ACS_DATA_ERR_RESOURCE_NOT_PROTECTED) {
			LOG_WRN("data-in write to handle 0x%04x rejected: resource handle not "
				"protected "
				"by active map",
				attr_handle);
			return BT_GATT_ERR(BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED);
		}
		if (err == ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG) {
			LOG_WRN("data-in write to handle 0x%04x rejected: ISC_ID not found or "
				"security "
				"config mismatch",
				attr_handle);
			return BT_GATT_ERR(BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG);
		}
		if (err == -ENOMEM) {
			LOG_WRN("data-in write to handle 0x%04x rejected: no free request context "
				"for "
				"protected resource",
				attr_handle);
			return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
		}
		if (err) {
			LOG_ERR("data-in processing failed: %d", err);
			return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
		}
		break;
	}
	case ACS_SEG_RX_PENDING:
		break;
	case ACS_SEG_RX_ERR_COUNTER:
		return BT_GATT_ERR(BT_ACS_ATT_ERR_INVALID_SEG_COUNTER);
	case ACS_SEG_RX_ERR_OVERFLOW:
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	case ACS_SEG_RX_ERR_TIMEOUT:
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	case ACS_SEG_RX_ERR_ORPHAN:
	case ACS_SEG_RX_ERR_LEN:
	default:
		LOG_ERR("data-in processing failed");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return len;
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */
