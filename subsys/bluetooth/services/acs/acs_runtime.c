/*
 * Copyright (c) 2026 Dipak Shetty
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

#include "acs_channel_rx.h"
#include "acs_internal.h"
#include "acs_protected_resource_router.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

int acs_runtime_dispatch_cp_frame(struct acs_frame *frame, struct bt_acs_conn *acs_conn,
				  struct acs_procedure *prot_req)
{
	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);

	acs_cp_dispatch(frame, acs_conn, prot_req);
	return 0;
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
int acs_runtime_dispatch_protected_cp_frame(struct acs_frame *frame, struct bt_acs_conn *acs_conn)
{
	struct acs_procedure *req_ctx;
	uint16_t data_offset;
	int sub_err;

	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->data_rx.buf != NULL);

	/* Spec-mandated: reject Data In write if DOI CCC not configured. */
	sub_err = acs_doi_ccc_check(frame->conn);
	if (sub_err == -EINVAL) {
		LOG_WRN("Data In: DOI indications not enabled for protected CP handle 0x%04x",
			frame->resource_handle);
		return ACS_DATA_ERR_CCC_IMPROPER_CONF;
	}
	if (sub_err) {
		LOG_WRN("Data In: DOI unavailable for protected CP handle 0x%04x (%d)",
			frame->resource_handle, sub_err);
		return sub_err;
	}

	LOG_DBG("Data In: routing handle 0x%04x to CP dispatcher (respond via DOI)",
		frame->resource_handle);

	data_offset = (uint16_t)(frame->payload - acs_conn->data_rx.buf->data);
	req_ctx = acs_procedure_alloc(acs_conn, frame->resource_handle, frame->isc_id,
					      data_offset, frame->payload_len);
	if (!req_ctx) {
		LOG_WRN("Data In: no free CP request context for handle 0x%04x",
			frame->resource_handle);
		return -ENOMEM;
	}

	req_ctx->decrypted_request = acs_conn->data_rx.buf;
	acs_conn->data_rx.buf = NULL;

	(void)acs_runtime_dispatch_cp_frame(frame, acs_conn, req_ctx);

	/* The CP handler ran synchronously and consumed the input bytes already.
	 * Drop the decrypted_request buffer now so it returns to the pool while
	 * we wait for the indication to confirm — acs_req_free will see NULL
	 * and skip the free at refcount-zero.
	 */
	if (req_ctx->decrypted_request) {
		acs_buf_free(req_ctx->decrypted_request);
		req_ctx->decrypted_request = NULL;
	}

	/* If a multi-step reply sequence is active, the sequence now owns the
	 * ALLOC ref and will release it in acs_seq_clear(). Otherwise, drop it
	 * here — the single response is already queued or completed.
	 */
	if (!req_ctx->reply_seq.desc) {
		acs_procedure_release_owner(req_ctx);
	}
	return 0;
}

int acs_runtime_dispatch_protected_char_frame(struct acs_frame *frame,
					      struct bt_acs_conn *acs_conn)
{
	struct acs_procedure *req_ctx;
	uint16_t data_offset;
	int sub_err;

	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->data_rx.buf != NULL);

	sub_err = acs_require_data_out_subscription(frame->conn, frame->resource_handle,
						    frame->payload_len);
	if (sub_err == -EINVAL) {
		LOG_WRN("Data In: required Data Out CCC not enabled for handle 0x%04x",
			frame->resource_handle);
		return ACS_DATA_ERR_CCC_IMPROPER_CONF;
	}
	if (sub_err) {
		LOG_WRN("Data In: unable to resolve Data Out path for handle 0x%04x (%d)",
			frame->resource_handle, sub_err);
		return sub_err;
	}

	data_offset = (uint16_t)(frame->payload - acs_conn->data_rx.buf->data);
	req_ctx = acs_procedure_alloc(acs_conn, frame->resource_handle, frame->isc_id,
					      data_offset, frame->payload_len);
	if (!req_ctx) {
		LOG_WRN("Data In: no free request context for handle 0x%04x",
			frame->resource_handle);
		return -ENOMEM;
	}

	req_ctx->decrypted_request = acs_conn->data_rx.buf;
	acs_conn->data_rx.buf = NULL;

	k_work_submit(&req_ctx->work);
	return 0;
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

int acs_runtime_dispatch_frame(struct acs_frame *frame, struct bt_acs_conn *acs_conn)
{
	struct acs_route route;
	uint16_t map_id;
	int err;

	if (!frame || !acs_conn) {
		return -EINVAL;
	}

	map_id = (frame->source_channel == ACS_SRC_CP) ? 0U : acs_conn->restriction_map_id;

	err = acs_classify_frame(frame, map_id, &route);
	if (err == -ENOENT) {
		LOG_WRN("Runtime dispatch: handle 0x%04x not in restriction map",
			frame->resource_handle);
		return ACS_DATA_ERR_RESOURCE_NOT_PROTECTED;
	}
	if (err) {
		LOG_ERR("Runtime dispatch: classify_frame failed (%d)", err);
		return err;
	}

	switch (route.kind) {
	case ACS_ROUTE_ACS_CP:
		return acs_runtime_dispatch_cp_frame(frame, acs_conn, NULL);
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
		LOG_WRN("CP write: unexpected offset %u", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len < 2) {
		LOG_WRN("CP write: PDU too short (%u bytes)", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (!acs_is_initialized()) {
		LOG_ERR("CP write: ACS not initialized");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* Spec-mandated: reject CP write if CP indications not enabled (ATT CCC_IMPROPER_CONF). */
	ret = acs_cp_ccc_check(conn);

	if (ret == -EINVAL) {
		LOG_WRN("CP write: CP indications not enabled by client");
		return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
	}
	if (ret) {
		LOG_ERR("CP write: CCC check failed (%d)", ret);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	acs_conn = acs_conn_lookup(conn);

	if (!acs_conn) {
		LOG_ERR("CP write: no ACS connection state for conn %p", (void *)conn);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	seg_rx_result = acs_channel_rx_feed(&acs_conn->cp_rx, buf, len);

	switch (seg_rx_result) {
	case ACS_SEG_RX_COMPLETE: {
		struct net_buf *rx_buf = acs_conn->cp_rx.buf;
		struct acs_frame frame = {
			.conn = conn,
			.resource_handle = 0,
			.isc_id = 0,
			.payload = rx_buf->data,
			.payload_len = rx_buf->len,
			.source_channel = ACS_SRC_CP,
			.encrypted = false,
			.backing_buf = rx_buf,
		};
		bool is_abort = false;

#if IS_ENABLED(CONFIG_BT_ACS_ABORT)
		/*
		 * Abort command must preempt any in-progress procedure per §4.4.4: it bypasses the
		 * plain_cp_proc lock and the handler owns the lock semantics itself.
		 */
		is_abort = (frame.payload_len > 0 &&
			    frame.payload[0] == BT_ACS_CP_OPCODE_ABORT);
#endif
		if (!is_abort && !atomic_cas(&acs_conn->plain_cp_proc.locked, 0, 1)) {
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
		LOG_WRN("CP RX: invalid segment counter (out-of-sequence PDU)");
		return BT_GATT_ERR(BT_ACS_ATT_ERR_INVALID_SEG_COUNTER);
	case ACS_SEG_RX_ERR_OVERFLOW:
		LOG_WRN("CP RX: overflow — accumulated payload exceeds buffer");
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	case ACS_SEG_RX_ERR_TIMEOUT:
		LOG_WRN("CP RX: inter-segment timeout expired");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	case ACS_SEG_RX_ERR_ORPHAN:
		LOG_WRN("CP RX: continuation segment without prior first segment");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	case ACS_SEG_RX_ERR_LEN:
		LOG_WRN("CP RX: invalid segment length");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	default:
		LOG_ERR("CP RX: unexpected seg_rx result %d", (int)seg_rx_result);
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
	int proc_err;
	uint16_t attr_handle;

	ARG_UNUSED(flags);

	if (offset != 0) {
		LOG_ERR("Data In write with non-zero offset: %u", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len < 1) {
		LOG_ERR("Data In write with invalid length: %u", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (!acs_is_initialized()) {
		LOG_ERR("Data In write received but ACS not initialized");
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

		proc_err = acs_data_in_unwrap_and_route(conn, acs_conn, &simple);
		attr_handle = bt_gatt_attr_get_handle(attr);

		/*
		 * On success the buffer is either owned by the request context
		 * (regular protected resource) or already freed (protected CP).
		 * On error, release the buffer back to the pool.
		 */
		if (proc_err && acs_conn->data_rx.buf) {
			acs_seg_rx_reset(&acs_conn->data_rx);
		}

		if (proc_err == ACS_DATA_ERR_NOT_AUTHORIZED) {
			LOG_WRN("Data In ATT write to handle 0x%04x: no security established",
				attr_handle);
			return BT_GATT_ERR(BT_ATT_ERR_AUTHORIZATION);
		}
		if (proc_err == ACS_DATA_ERR_INVALID_KEY) {
			LOG_ERR("Data In ATT write to handle 0x%04x failed with Invalid Key; inner "
				"protected resource handle is unavailable unless decryption "
				"succeeds",
				attr_handle);
			return BT_GATT_ERR(BT_ACS_ATT_ERR_INVALID_KEY);
		}
		if (proc_err == ACS_DATA_ERR_CCC_IMPROPER_CONF) {
			LOG_WRN("Data In ATT write to handle 0x%04x: required CCC not configured",
				attr_handle);
			return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
		}
		if (proc_err == ACS_DATA_ERR_RESOURCE_NOT_PROTECTED) {
			LOG_WRN("Data In ATT write to handle 0x%04x: resource handle not protected "
				"by active map",
				attr_handle);
			return BT_GATT_ERR(BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED);
		}
		if (proc_err == ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG) {
			LOG_WRN("Data In ATT write to handle 0x%04x: ISC_ID not found or security "
				"config mismatch",
				attr_handle);
			return BT_GATT_ERR(BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG);
		}
		if (proc_err == -ENOMEM) {
			LOG_WRN("Data In ATT write to handle 0x%04x: no free request context for "
				"protected resource",
				attr_handle);
			return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
		}
		if (proc_err) {
			LOG_ERR("Data In processing failed: %d", proc_err);
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
		LOG_ERR("Data In processing failed");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return len;
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */
