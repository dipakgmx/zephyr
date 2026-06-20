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
				      enum acs_route_kind *kind)
{
	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(kind != NULL);

	if (frame->source_channel == ACS_SRC_CP) {
		*kind = ACS_ROUTE_ACS_CP;
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
			*kind = ACS_ROUTE_PROTECTED_SERVICE_CP;
			return 0;
		case ACS_RMAP_RESOURCE_CHAR:
			*kind = ACS_ROUTE_PROTECTED_CHAR;
			return 0;
		default:
			LOG_WRN("handle 0x%04x not in restriction map %u", frame->resource_handle,
				map_id);
			return -ENOENT;
		}
	}
#else
	ARG_UNUSED(map_id);
	LOG_WRN("data-in received but authorization disabled");
	return -ENOENT;
#endif
}

/*
 * Transfer the reassembled Data-In buffer from the channel RX context into the
 * request. The unwrap helper already pulled the transport header and trimmed
 * the buffer in place, so data/len describe the inner plaintext payload.
 */
static void acs_reply_take_request_buf(struct acs_reply *reply, struct bt_acs_conn *acs_conn)
{
	reply->request = acs_conn->data_rx.buf;
	acs_conn->data_rx.buf = NULL;
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
int acs_runtime_dispatch_protected_cp_frame(const struct acs_frame *frame,
					    struct bt_acs_conn *acs_conn)
{
	struct acs_reply *reply;
	int err;

	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->data_rx.buf != NULL);

	err = acs_doi_ccc_check(acs_conn->conn);
	if (err) {
		LOG_WRN("DOI indications not enabled for protected CP handle 0x%04x",
			frame->resource_handle);
		return ACS_DATA_ERR_CCC_IMPROPER_CONF;
	}

	LOG_DBG("routing handle 0x%04x to CP dispatcher (respond via DOI)", frame->resource_handle);

	reply = acs_reply_alloc(acs_conn);
	if (!reply) {
		LOG_WRN("no free reply slot for protected CP handle 0x%04x",
			frame->resource_handle);
		return -ENOMEM;
	}

	reply->channel = ACS_REPLY_DOI;
	reply->resource_handle = frame->resource_handle;
	reply->isc_id = frame->isc_id;
	acs_reply_take_request_buf(reply, acs_conn);

	err = acs_cp_dispatch(frame, acs_conn, reply);

	if (reply->request) {
		acs_buf_free(reply->request);
		reply->request = NULL;
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
		/* Same ISC_ID covers both read and write - use payload as tiebreaker. */
		return (payload_len > 0) ? ACS_REQ_ACCESS_WRITE : ACS_REQ_ACCESS_READ;
	}

	return ACS_REQ_ACCESS_UNKNOWN;
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

int acs_runtime_dispatch_protected_char_frame(const struct acs_frame *frame,
					      struct bt_acs_conn *acs_conn)
{
	struct acs_reply *reply;
	int err;

	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->data_rx.buf != NULL);

	err = acs_require_data_out_subscription(acs_conn->conn, frame->resource_handle,
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

	reply = acs_reply_alloc(acs_conn);
	if (!reply) {
		LOG_WRN("no free reply slot for handle 0x%04x", frame->resource_handle);
		return -ENOMEM;
	}

	reply->resource_handle = frame->resource_handle;
	reply->isc_id = frame->isc_id;
	acs_reply_take_request_buf(reply, acs_conn);

#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	reply->access = acs_rmap_resolve_request_access(acs_conn->restriction_map_id,
							frame->resource_handle, frame->isc_id,
							frame->payload_len);
	if (reply->access == ACS_REQ_ACCESS_UNKNOWN) {
		LOG_ERR("no read/write op in rmap for handle 0x%04x isc 0x%04x",
			frame->resource_handle, frame->isc_id);
		acs_reply_free(reply);
		return -ENOENT;
	}
#endif

	acs_request_queue_submit(acs_conn, reply);
	return 0;
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

int acs_runtime_dispatch_frame(const struct acs_frame *frame, struct bt_acs_conn *acs_conn)
{
	enum acs_route_kind kind;
	uint16_t map_id;
	int err;

	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);

	/* Plain CP writes are always unencrypted - no restriction map lookup needed. */
	map_id = (frame->source_channel == ACS_SRC_CP) ? BT_ACS_RMAP_ID_NONE
						       : acs_conn->restriction_map_id;

	err = acs_runtime_classify_frame(frame, map_id, &kind);
	if (err != 0) {
		LOG_WRN("handle 0x%04x not in restriction map", frame->resource_handle);
		return ACS_DATA_ERR_RESOURCE_NOT_PROTECTED;
	}

	switch (kind) {
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

/* Map a segmentation-RX error to the ATT error returned for the failing write. */
static ssize_t acs_seg_rx_err_to_att(enum acs_seg_rx_result res, const char *channel)
{
	switch (res) {
	case ACS_SEG_RX_ERR_COUNTER:
		LOG_WRN("%s: invalid segment counter", channel);
		return BT_GATT_ERR(BT_ACS_ATT_ERR_INVALID_SEG_COUNTER);
	case ACS_SEG_RX_ERR_OVERFLOW:
		LOG_WRN("%s: receive overflow: accumulated payload exceeds buffer", channel);
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	case ACS_SEG_RX_ERR_TIMEOUT:
		LOG_WRN("%s: inter-segment timeout expired", channel);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	case ACS_SEG_RX_ERR_ORPHAN:
		LOG_WRN("%s: continuation segment without prior first segment", channel);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	case ACS_SEG_RX_ERR_LEN:
		LOG_WRN("%s: invalid segment length", channel);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	default:
		LOG_ERR("%s: unexpected seg_rx result %d", channel, (int)res);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
}

ssize_t acs_cp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
		     uint16_t len, uint16_t offset, uint8_t flags)
{
	int ret;
	struct bt_acs_conn *acs_conn;
	enum acs_seg_rx_result seg_rx_result;

	ARG_UNUSED(flags);

	if (offset != 0) {
		LOG_WRN("unexpected CP write offset %u", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	/* Segmentation_Header + Payload min size */
	if (len < ACS_SEG_HDR_SIZE + 1) {
		LOG_WRN("CP write PDU too short (%u bytes)", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (!acs_is_initialized()) {
		LOG_ERR("CP write received before ACS init");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* See §4.4.5 General error handling procedures */
	ret = acs_cp_ccc_check(conn);

	if (ret == -EINVAL) {
		LOG_WRN("CP indications not enabled by client");
		return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
	}

	acs_conn = acs_conn_lookup(conn);

	if (!acs_conn) {
		LOG_ERR("no ACS connection state for conn %p", (void *)conn);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	seg_rx_result = acs_channel_rx_feed(&acs_conn->cp_rx, buf, len);

	switch (seg_rx_result) {
	case ACS_SEG_RX_COMPLETE: {
		struct acs_frame frame = acs_frame_from_cp_rx(acs_conn->cp_rx.buf);
		bool is_abort = false;

#if IS_ENABLED(CONFIG_BT_ACS_ABORT)
		/*
		 * Abort command must preempt any in-progress procedure per §4.4.4: it bypasses the
		 * cp_locked gate and the handler owns the lock semantics itself.
		 */
		is_abort = (frame.payload_len > 0 && frame.payload[0] == BT_ACS_CP_OPCODE_ABORT);
#endif

		if (!is_abort && !atomic_cas(&acs_conn->cp_locked, 0, 1)) {
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
	default:
		return acs_seg_rx_err_to_att(seg_rx_result, "CP");
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

	if (len < ACS_SEG_HDR_SIZE + 1) {
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
		/* Unwrap consumes the transport header and trims to the inner
		 * plaintext directly on the reassembled buffer, so a later
		 * ownership transfer into a request context is a plain handoff.
		 */
		err = acs_data_in_unwrap_and_route(acs_conn, &acs_conn->data_rx.buf->b);

		/*
		 * On success the buffer is either owned by the request context
		 * (regular protected resource) or already freed (protected CP).
		 * On error, release the buffer back to the pool.
		 */
		if (err && acs_conn->data_rx.buf) {
			acs_seg_rx_reset(&acs_conn->data_rx);
		}

		switch (err) {
		case 0:
			break;
		case ACS_DATA_ERR_NOT_AUTHORIZED:
			LOG_WRN("data-in write to handle 0x%04x rejected: no security established",
				attr_handle);
			return BT_GATT_ERR(BT_ATT_ERR_AUTHORIZATION);
		case ACS_DATA_ERR_INVALID_KEY:
			LOG_ERR("data-in write to handle 0x%04x failed with invalid key; inner "
				"protected resource handle is unavailable unless decryption "
				"succeeds",
				attr_handle);
			return BT_GATT_ERR(BT_ACS_ATT_ERR_INVALID_KEY);
		case ACS_DATA_ERR_CCC_IMPROPER_CONF:
			LOG_WRN("data-in write to handle 0x%04x rejected: required CCC not "
				"configured",
				attr_handle);
			return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
		case ACS_DATA_ERR_RESOURCE_NOT_PROTECTED:
			LOG_WRN("data-in write to handle 0x%04x rejected: resource handle not "
				"protected by active map",
				attr_handle);
			return BT_GATT_ERR(BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED);
		case ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG:
			LOG_WRN("data-in write to handle 0x%04x rejected: ISC_ID not found or "
				"security config mismatch",
				attr_handle);
			return BT_GATT_ERR(BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG);
		case -ENOMEM:
			LOG_WRN("data-in write to handle 0x%04x rejected: no free request context "
				"for protected resource",
				attr_handle);
			return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
		default:
			LOG_ERR("data-in processing failed: %d", err);
			return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
		}
		break;
	}
	case ACS_SEG_RX_PENDING:
		break;
	default:
		return acs_seg_rx_err_to_att(res, "data-in");
	}

	return len;
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */
