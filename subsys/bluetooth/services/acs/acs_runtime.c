/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_internal.h"
#include "acs_runtime.h"
#include "acs_reply.h"
#include "acs_cp.h"
#include "acs_request.h"
#include "acs_seg.h"
#include "acs_rhandle.h"
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#include "acs_rmap.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* The classifier and its dispatch helpers serve the Data In path only. */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
/* A CP resource is keyed by its procedure opcode: the first plaintext byte. */
static bool acs_rmap_cp_entry_matches(const struct bt_acs_rmap_entry *entry,
				      const struct acs_frame *frame)
{
	return frame->payload_len > 0U &&
	       acs_rmap_entry_matches(entry, frame->payload[0], frame->isc_id);
}

/* A payload after Protected_Resource_Handle makes this a secure write (Table 4.10). */
static uint16_t acs_protected_char_att_opcode(const struct acs_frame *frame)
{
	return frame->payload_len > 0 ? BT_ACS_RMAP_OP_ATT_WRITE_REQ : BT_ACS_RMAP_OP_ATT_READ_REQ;
}

static enum acs_route_kind acs_route_from_att_opcode(uint16_t opcode)
{
	return (opcode == BT_ACS_RMAP_OP_ATT_WRITE_REQ) ? ACS_ROUTE_PROTECTED_WRITE
							: ACS_ROUTE_PROTECTED_READ;
}

#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

static int acs_runtime_classify_route(const struct acs_frame *frame,
				      const struct bt_acs_rmap_runtime *runtime,
				      enum acs_route_kind *kind, uint16_t *attr_handle,
				      const struct bt_gatt_attr **value_attr, uint8_t *value_props)
{
	*value_attr = NULL;
	*value_props = 0U;
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	{
		const struct bt_acs_rmap_entry *rmap_entry;
		uint16_t opcode;

		if (runtime == NULL) {
			LOG_ERR("data-in frame received without an active restriction map");
			return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
		}

		/* ACS CP opcode/ISC validation happens in the CP procedure. */
		if (frame->resource_handle == acs_cp_resource_handle()) {
			*attr_handle = acs_cp_attr_handle();
			*value_attr = acs_attr_cp();
			*kind = ACS_ROUTE_PROTECTED_ACS_CP;
			return 0;
		}

		if (acs_rmap_find_entry_by_resource_handle(runtime, frame->resource_handle,
							   &rmap_entry) != 0) {
			uint16_t default_isc = runtime->map->default_isc_id;

			if (default_isc != BT_ACS_ISC_ID_NONE) {
				if (default_isc != frame->isc_id) {
					return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
				}
				*attr_handle = acs_rhandle_find_attr_handle(frame->resource_handle);
				if (*attr_handle == 0U) {
					LOG_WRN("data-in handle 0x%04x has no backing attribute",
						frame->resource_handle);
					return ACS_DATA_ERR_RESOURCE_NOT_PROTECTED;
				}
				*kind = acs_route_from_att_opcode(
					acs_protected_char_att_opcode(frame));
				return 0;
			}
			LOG_WRN("data-in handle 0x%04x not in restriction map %u",
				frame->resource_handle, runtime->map->map_id);
			return ACS_DATA_ERR_RESOURCE_NOT_PROTECTED;
		}

		switch (rmap_entry->record->kind) {
		case BT_ACS_RMAP_RESOURCE_CP:
			if (!acs_rmap_cp_entry_matches(rmap_entry, frame)) {
				return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
			}
			*attr_handle = rmap_entry->attr_handle;
			*value_attr = rmap_entry->value_attr;
			*value_props = rmap_entry->props;
			*kind = ACS_ROUTE_PROTECTED_EXTERNAL_CP;
			return 0;
		case BT_ACS_RMAP_RESOURCE_CHAR:
			opcode = acs_protected_char_att_opcode(frame);
			if (!acs_rmap_entry_matches(rmap_entry, opcode, frame->isc_id)) {
				return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
			}
			*attr_handle = rmap_entry->attr_handle;
			*value_attr = rmap_entry->value_attr;
			*value_props = rmap_entry->props;
			*kind = acs_route_from_att_opcode(opcode);
			return 0;
		default:
			LOG_WRN("handle 0x%04x not in restriction map %u", frame->resource_handle,
				runtime->map->map_id);
			return ACS_DATA_ERR_RESOURCE_NOT_PROTECTED;
		}
	}
#else
	ARG_UNUSED(runtime);
	if (frame->resource_handle == acs_cp_resource_handle()) {
		*kind = ACS_ROUTE_PROTECTED_ACS_CP;
		*attr_handle = acs_cp_attr_handle();
		*value_attr = acs_attr_cp();
		return 0;
	}
	LOG_WRN("data-in received but authorization disabled");
	return ACS_DATA_ERR_RESOURCE_NOT_PROTECTED;
#endif
}

static struct acs_reply *
alloc_reply_for_request(struct bt_acs_conn *acs_conn, const struct acs_frame *frame,
			uint16_t attr_handle, const struct bt_gatt_attr *value_attr,
			uint8_t value_props, enum acs_reply_channel channel,
			enum acs_req_access access)
{
	struct acs_reply *reply = acs_reply_alloc(acs_conn);

	if (reply == NULL) {
		LOG_WRN("no free reply slot for resource 0x%04x", frame->resource_handle);
		return NULL;
	}

	reply->resource_handle = frame->resource_handle;
	reply->attr_handle = attr_handle;
	reply->value_attr = value_attr;
	reply->value_props = value_props;
	reply->isc_id = frame->isc_id;
	reply->key_runtime = frame->key_runtime;
	reply->channel = channel;
	reply->access = access;

	/* Already unwrapped and trimmed, so this is the inner plaintext. */
	acs_reply_take_request_buf(reply, &acs_conn->data_rx);

	return reply;
}

static int acs_runtime_dispatch_protected_acs_cp_frame(const struct acs_frame *frame,
						       struct bt_acs_conn *acs_conn)
{
	struct acs_reply *reply;
	int err;

	err = acs_doi_ccc_check(acs_conn->conn);
	if (err) {
		LOG_WRN("DOI indications not enabled for protected CP handle 0x%04x",
			frame->resource_handle);
		return ACS_DATA_ERR_CCC_IMPROPER_CONF;
	}

	LOG_DBG("routing handle 0x%04x to CP dispatcher (respond via DOI)", frame->resource_handle);

	reply = alloc_reply_for_request(acs_conn, frame, acs_cp_attr_handle(), acs_attr_cp(), 0U,
					ACS_REPLY_DOI, ACS_REQ_ACCESS_UNKNOWN);
	if (reply == NULL) {
		return ACS_DATA_ERR_NO_RESOURCES;
	}

	return acs_cp_queue_protected(frame, acs_conn, reply);
}

static int acs_runtime_dispatch_protected_attr_frame(
	const struct acs_frame *frame, struct bt_acs_conn *acs_conn, uint16_t attr_handle,
	const struct bt_gatt_attr *value_attr, uint8_t value_props, enum acs_req_access access)
{
	struct acs_reply *reply;
	int err;

	if (value_attr == NULL && acs_resolve_char(attr_handle, &value_attr, &value_props) != 0) {
		LOG_WRN("unable to resolve protected resource 0x%04x backing attr 0x%04x",
			frame->resource_handle, attr_handle);
		return -EIO;
	}

	/* Protected characteristic responses use Data Out Notify (§4.3.2). */
	err = acs_don_ccc_check(acs_conn->conn);
	if (err == -EINVAL) {
		LOG_WRN("required data-out CCC not enabled for resource 0x%04x attr 0x%04x",
			frame->resource_handle, attr_handle);
		return ACS_DATA_ERR_CCC_IMPROPER_CONF;
	}
	if (err) {
		LOG_WRN("unable to resolve data-out path for resource 0x%04x attr 0x%04x (%d)",
			frame->resource_handle, attr_handle, err);
		return err;
	}

	reply = alloc_reply_for_request(acs_conn, frame, attr_handle, value_attr, value_props,
					ACS_REPLY_DON, access);
	if (reply == NULL) {
		return ACS_DATA_ERR_NO_RESOURCES;
	}

	acs_request_queue_submit(acs_conn, reply);
	return 0;
}

/* Another service's Control Point sends its own response. */
static int acs_runtime_dispatch_external_cp_frame(const struct acs_frame *frame,
						  struct bt_acs_conn *acs_conn,
						  uint16_t attr_handle,
						  const struct bt_gatt_attr *value_attr,
						  uint8_t value_props)
{
	struct acs_reply *reply;

	if (value_attr == NULL && acs_resolve_char(attr_handle, &value_attr, &value_props) != 0) {
		LOG_WRN("unable to resolve external CP resource 0x%04x backing attr 0x%04x",
			frame->resource_handle, attr_handle);
		return -EIO;
	}

	reply = alloc_reply_for_request(acs_conn, frame, attr_handle, value_attr, value_props,
					ACS_REPLY_CP, ACS_REQ_ACCESS_CP_WRITE);
	if (reply == NULL) {
		return ACS_DATA_ERR_NO_RESOURCES;
	}

	acs_request_queue_submit(acs_conn, reply);
	return 0;
}

int acs_runtime_dispatch_frame(const struct acs_frame *frame, struct bt_acs_conn *acs_conn)
{
	const struct bt_gatt_attr *value_attr;
	enum acs_route_kind kind;
	uint16_t attr_handle;
	uint8_t value_props;
	int err;

	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);

	err = acs_runtime_classify_route(frame, acs_rmap_active(), &kind, &attr_handle, &value_attr,
					 &value_props);
	if (err != 0) {
		return err;
	}

	switch (kind) {
	case ACS_ROUTE_PROTECTED_ACS_CP:
		return acs_runtime_dispatch_protected_acs_cp_frame(frame, acs_conn);
	case ACS_ROUTE_PROTECTED_EXTERNAL_CP:
		return acs_runtime_dispatch_external_cp_frame(frame, acs_conn, attr_handle,
							      value_attr, value_props);
	case ACS_ROUTE_PROTECTED_WRITE:
		return acs_runtime_dispatch_protected_attr_frame(frame, acs_conn, attr_handle,
								 value_attr, value_props,
								 ACS_REQ_ACCESS_WRITE);
	case ACS_ROUTE_PROTECTED_READ:
		return acs_runtime_dispatch_protected_attr_frame(
			frame, acs_conn, attr_handle, value_attr, value_props, ACS_REQ_ACCESS_READ);
	default:
		return -EINVAL;
	}
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

/* Convert a segmentation error to the matching Data In error. */
static int acs_seg_rx_err_to_errno(enum acs_seg_rx_result res)
{
	switch (res) {
	case ACS_SEG_RX_ERR_COUNTER:
		return ACS_DATA_ERR_INVALID_SEG_COUNTER;
	case ACS_SEG_RX_ERR_OVERFLOW:
		return ACS_DATA_ERR_NO_RESOURCES;
	case ACS_SEG_RX_ERR_TIMEOUT:
	case ACS_SEG_RX_ERR_ORPHAN:
	case ACS_SEG_RX_ERR_LEN:
	default:
		return -EIO;
	}
}

/* Convert an ACS write result to an ATT status. */
static ssize_t acs_write_err_to_att(int err, uint16_t handle)
{
	uint8_t att_err;

	switch (err) {
	case 0:
		return 0;
	case ACS_CP_RESULT_INSUFFICIENT_AUTH:
		att_err = BT_ATT_ERR_AUTHORIZATION;
		break;
	case ACS_CP_RESULT_PROCEDURE_IN_PROGRESS:
		att_err = BT_ATT_ERR_PROCEDURE_IN_PROGRESS;
		break;
	case ACS_CP_RESULT_INVALID_LENGTH:
		att_err = BT_ATT_ERR_INVALID_ATTRIBUTE_LEN;
		break;
	case ACS_DATA_ERR_INVALID_KEY:
		att_err = BT_ACS_ATT_ERR_INVALID_KEY;
		break;
	case ACS_DATA_ERR_CCC_IMPROPER_CONF:
		att_err = BT_ATT_ERR_CCC_IMPROPER_CONF;
		break;
	case ACS_DATA_ERR_RESOURCE_NOT_PROTECTED:
		att_err = BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED;
		break;
	case ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG:
		att_err = BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG;
		break;
	case ACS_DATA_ERR_INVALID_SEG_COUNTER:
		att_err = BT_ACS_ATT_ERR_INVALID_SEG_COUNTER;
		break;
	case ACS_DATA_ERR_NO_RESOURCES:
		att_err = BT_ATT_ERR_INSUFFICIENT_RESOURCES;
		break;
	default:
		LOG_ERR("handle 0x%04x: unmapped error %d", handle, err);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	LOG_WRN("handle 0x%04x: err %d rejected with ATT 0x%02x %s", handle, err, att_err,
		bt_att_err_to_str(att_err));
	return BT_GATT_ERR(att_err);
}

ssize_t acs_cp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
		     uint16_t len, uint16_t offset, uint8_t flags)
{
	struct bt_acs_conn *acs_conn;
	enum acs_seg_rx_result seg_rx_result;

	ARG_UNUSED(flags);

	if (offset != 0) {
		LOG_WRN("unexpected CP write offset %u", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	/* A Control Point PDU contains a segment header and at least one payload byte. */
	if (len < ACS_SEG_HDR_SIZE + 1) {
		LOG_WRN("CP write PDU too short (%u bytes)", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (!acs_is_initialized()) {
		LOG_ERR("CP write received before ACS init");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* The client must enable ACS Control Point indications (ACS 1.0 §4.4.5). */
	if (acs_cp_ccc_check(conn) != 0) {
		LOG_WRN("CP indications not enabled by client");
		return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
	}

	acs_conn = acs_conn_lookup(conn);

	if (acs_conn == NULL) {
		LOG_ERR("no ACS connection state for conn %p", (void *)conn);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	seg_rx_result = acs_channel_rx_feed(&acs_conn->cp_rx, buf, len);

	switch (seg_rx_result) {
	case ACS_SEG_RX_COMPLETE: {
		ssize_t att_err = acs_write_err_to_att(acs_cp_queue_plain(acs_conn), 0);

		return att_err ? att_err : len;
	}
	case ACS_SEG_RX_PENDING:
		break;
	default:
		return acs_write_err_to_att(acs_seg_rx_err_to_errno(seg_rx_result), 0);
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
	uint16_t attr_handle = attr != NULL ? bt_gatt_attr_get_handle(attr) : 0U;

	ARG_UNUSED(flags);

	if (offset != 0) {
		LOG_WRN("data-in write with non-zero offset: %u", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len < ACS_SEG_HDR_SIZE + 1) {
		LOG_WRN("data-in write with invalid length: %u", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (!acs_is_initialized()) {
		LOG_ERR("data-in write received before ACS init");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	acs_conn = acs_conn_lookup(conn);

	if (acs_conn == NULL) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	res = acs_channel_rx_feed(&acs_conn->data_rx, buf, len);

	switch (res) {
	case ACS_SEG_RX_COMPLETE: {
		err = acs_data_in_unwrap_and_route(acs_conn, acs_conn->data_rx.buf);

		/* Release the buffer only if routing did not take it. */
		if (err && acs_conn->data_rx.buf != NULL) {
			acs_seg_rx_reset(&acs_conn->data_rx);
		}

		if (err) {
			return acs_write_err_to_att(err, attr_handle);
		}
		break;
	}
	case ACS_SEG_RX_PENDING:
		break;
	default:
		return acs_write_err_to_att(acs_seg_rx_err_to_errno(res), attr_handle);
	}

	return len;
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */
