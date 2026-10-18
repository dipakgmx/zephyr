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
#include "acs_data_in.h"
#include "acs_runtime.h"
#include "acs_reply.h"
#include "acs_cp.h"
#include "acs_request.h"
#include "acs_seg.h"
#include "acs_rhandle.h"
#include "acs_rmap.h"
#include "host/gatt_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* The classifier and its dispatch helpers serve the Data In path only. */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)

/* Route a protected payload takes once decrypted. */
enum acs_route_kind {
	ACS_ROUTE_PROTECTED_ACS_CP = 0, /* ACS Control Point procedure, answered on DOI */
	ACS_ROUTE_PROTECTED_READ,       /* Characteristic read, answered on DON */
	ACS_ROUTE_PROTECTED_WRITE,      /* Characteristic write, answered on DON */
};

/* Classifier result: the route and, for a characteristic route, the value it acts on. */
struct acs_route {
	enum acs_route_kind kind;              /* Route of the request */
	const struct bt_gatt_attr *value_attr; /* Target value attribute, NULL for the ACS CP */
	uint8_t value_props;                   /* Target characteristic properties */
};

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
/*
 * The target keeps its Core access and link security permissions on the ACS
 * path, checked as for a plain ATT request (Core Vol 3 Part F 3.2.5).
 */
static uint8_t acs_runtime_check_perm(struct bt_conn *conn, const struct acs_route *route)
{
	uint16_t mask = (route->kind == ACS_ROUTE_PROTECTED_READ) ? BT_GATT_PERM_READ_MASK
								  : BT_GATT_PERM_WRITE_MASK;

	return bt_gatt_check_perm(conn, route->value_attr, mask);
}
#endif

/*
 * Checks run in the Core order for a plain ATT request: whether ACS protects the
 * resource at all, the target's access and link security permissions, then its
 * ACS authorization (operation protected, matching ISC).
 */
static uint8_t acs_runtime_classify_route(struct bt_conn *conn, const struct acs_frame *frame,
					  const struct bt_acs_restriction_map *map,
					  struct acs_route *route)
{
	*route = (struct acs_route){.kind = ACS_ROUTE_PROTECTED_ACS_CP};
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	{
		const struct bt_acs_rmap_resource *resource;
		uint8_t att_err;
		uint16_t isc;

		if (map == NULL) {
			LOG_ERR("data-in frame received without an active restriction map");
			return BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG;
		}

		/* ACS CP opcode/ISC validation happens in the CP procedure. */
		if (frame->resource_handle == acs_cp_resource_handle()) {
			return BT_ATT_ERR_SUCCESS;
		}

		resource = acs_rmap_resource_by_handle(map, frame->resource_handle);
		if (resource == NULL) {
			struct acs_rhandle_resource_record characteristic;

			/* An unlisted resource takes the map's default ISC (§4.4.3.2). */
			isc = map->default_isc_id;
			if (isc == BT_ACS_ISC_ID_NONE) {
				LOG_WRN("data-in handle 0x%04x not in restriction map %u",
					frame->resource_handle, map->map_id);
				return BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED;
			}
			if (acs_rhandle_get(frame->resource_handle, &characteristic) != 0) {
				LOG_WRN("data-in handle 0x%04x has no backing attribute",
					frame->resource_handle);
				return BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED;
			}
			route->value_attr = characteristic.attr;
			route->value_props = characteristic.properties;
			/* An empty characteristic payload is a secure read (Table 4.10). */
			route->kind = frame->payload_len > 0U ? ACS_ROUTE_PROTECTED_WRITE
							      : ACS_ROUTE_PROTECTED_READ;
		} else {
			/* A nonempty payload is a secure write (Table 4.10). */
			bool is_write = frame->payload_len > 0U;

			isc = acs_rmap_char_isc(resource,
						is_write ? BT_GATT_CHRC_WRITE : BT_GATT_CHRC_READ);
			route->kind =
				is_write ? ACS_ROUTE_PROTECTED_WRITE : ACS_ROUTE_PROTECTED_READ;
			route->value_attr = resource->bound.value_attr;
			route->value_props = resource->bound.props;
		}

		att_err = acs_runtime_check_perm(conn, route);
		if (att_err != BT_ATT_ERR_SUCCESS) {
			return att_err;
		}

		/* An operation the record does not list is unprotected (§3.1). */
		if (isc == BT_ACS_ISC_ID_NONE) {
			LOG_WRN("data-in handle 0x%04x: operation not protected",
				frame->resource_handle);
			return BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED;
		}
		if (isc != frame->isc_id) {
			return BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG;
		}

		return BT_ATT_ERR_SUCCESS;
	}
#else
	ARG_UNUSED(conn);
	ARG_UNUSED(map);
	if (frame->resource_handle == acs_cp_resource_handle()) {
		return BT_ATT_ERR_SUCCESS;
	}
	LOG_WRN("data-in received but authorization disabled");
	return BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED;
#endif
}

static struct acs_reply *alloc_reply_for_request(struct bt_acs_conn *acs_conn,
						 const struct acs_frame *frame,
						 const struct acs_route *route,
						 enum acs_reply_channel channel)
{
	struct acs_reply *reply = acs_reply_alloc(acs_conn);

	if (reply == NULL) {
		LOG_WRN("no free reply slot for resource 0x%04x", frame->resource_handle);
		return NULL;
	}

	reply->resource_handle = frame->resource_handle;
	reply->isc_id = frame->isc_id;
	reply->value_attr = route->value_attr;
	reply->value_props = route->value_props;
	reply->channel = channel;

	/* Already unwrapped and trimmed, so this is the inner plaintext. */
	reply->request = net_buf_take(&acs_conn->data_rx.buf);

	return reply;
}

static uint8_t acs_runtime_dispatch_protected_acs_cp_frame(const struct acs_frame *frame,
							   struct bt_acs_conn *acs_conn,
							   const struct acs_route *route)
{
	struct acs_reply *reply;

	if (acs_doi_ccc_check(acs_conn->conn) != 0) {
		LOG_WRN("DOI indications not enabled for protected CP handle 0x%04x",
			frame->resource_handle);
		return BT_ATT_ERR_CCC_IMPROPER_CONF;
	}

	LOG_DBG("routing handle 0x%04x to CP dispatcher (respond via DOI)", frame->resource_handle);

	reply = alloc_reply_for_request(acs_conn, frame, route, ACS_REPLY_DOI);
	if (reply == NULL) {
		return BT_ATT_ERR_INSUFFICIENT_RESOURCES;
	}

	return acs_cp_queue_protected(reply);
}

static uint8_t acs_runtime_dispatch_protected_attr_frame(const struct acs_frame *frame,
							 struct bt_acs_conn *acs_conn,
							 const struct acs_route *route)
{
	struct acs_reply *reply;

	/* Protected characteristic responses use Data Out Notify (§4.3.2). */
	if (acs_don_ccc_check(acs_conn->conn) != 0) {
		LOG_WRN("required data-out CCC not enabled for resource 0x%04x",
			frame->resource_handle);
		return BT_ATT_ERR_CCC_IMPROPER_CONF;
	}

	reply = alloc_reply_for_request(acs_conn, frame, route, ACS_REPLY_DON);
	if (reply == NULL) {
		return BT_ATT_ERR_INSUFFICIENT_RESOURCES;
	}

	acs_request_queue_submit(reply);
	return BT_ATT_ERR_SUCCESS;
}

uint8_t acs_runtime_dispatch_frame(const struct acs_frame *frame, struct bt_acs_conn *acs_conn)
{
	struct acs_route route;
	uint8_t att_err;

	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);

	att_err = acs_runtime_classify_route(acs_conn->conn, frame, acs_rmap_get_active_map(),
					     &route);
	if (att_err != BT_ATT_ERR_SUCCESS) {
		return att_err;
	}

	switch (route.kind) {
	case ACS_ROUTE_PROTECTED_ACS_CP:
		return acs_runtime_dispatch_protected_acs_cp_frame(frame, acs_conn, &route);
	case ACS_ROUTE_PROTECTED_READ:
	case ACS_ROUTE_PROTECTED_WRITE:
		return acs_runtime_dispatch_protected_attr_frame(frame, acs_conn, &route);
	default:
		return BT_ATT_ERR_UNLIKELY;
	}
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

/* ATT error for a failed reassembly. */
static uint8_t acs_seg_rx_att_err(enum acs_seg_rx_result res)
{
	switch (res) {
	case ACS_SEG_RX_ERR_COUNTER:
		return BT_ACS_ATT_ERR_INVALID_SEG_COUNTER;
	case ACS_SEG_RX_ERR_OVERFLOW:
		return BT_ATT_ERR_INVALID_ATTRIBUTE_LEN;
	case ACS_SEG_RX_ERR_NO_BUF:
		return BT_ATT_ERR_INSUFFICIENT_RESOURCES;
	default:
		return BT_ATT_ERR_UNLIKELY;
	}
}

/* Complete an ACS write: accept all len octets, or reject the write with att_err. */
static ssize_t acs_write_result(uint8_t att_err, uint16_t len)
{
	if (att_err == BT_ATT_ERR_SUCCESS) {
		return len;
	}

	LOG_WRN("write rejected with ATT 0x%02x %s", att_err, bt_att_err_to_str(att_err));
	return BT_GATT_ERR(att_err);
}

/* Checks shared by the ACS Control Point and Data In writes. */
static uint8_t acs_write_check(struct bt_conn *conn, uint16_t len, uint16_t offset,
			       struct bt_acs_conn **acs_conn)
{
	if (offset != 0U) {
		LOG_WRN("unexpected write offset %u", offset);
		return BT_ATT_ERR_INVALID_OFFSET;
	}

	/* A PDU contains a segment header and at least one payload byte. */
	if (len < ACS_SEG_HDR_SIZE + 1U) {
		LOG_WRN("write PDU too short (%u bytes)", len);
		return BT_ATT_ERR_INVALID_ATTRIBUTE_LEN;
	}

	if (!acs_is_initialized()) {
		LOG_ERR("write received before ACS init");
		return BT_ATT_ERR_UNLIKELY;
	}

	*acs_conn = acs_conn_lookup(conn);
	if (*acs_conn == NULL) {
		LOG_ERR("no ACS connection state for conn %p", (void *)conn);
		return BT_ATT_ERR_UNLIKELY;
	}

	return BT_ATT_ERR_SUCCESS;
}

ssize_t acs_cp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
		     uint16_t len, uint16_t offset, uint8_t flags)
{
	struct bt_acs_conn *acs_conn;
	enum acs_seg_rx_result seg_rx_result;
	uint8_t att_err;

	ARG_UNUSED(flags);

	att_err = acs_write_check(conn, len, offset, &acs_conn);
	if (att_err != BT_ATT_ERR_SUCCESS) {
		return BT_GATT_ERR(att_err);
	}

	seg_rx_result = acs_seg_rx_feed(&acs_conn->cp_rx, buf, len);

	switch (seg_rx_result) {
	case ACS_SEG_RX_COMPLETE:
		return acs_write_result(acs_cp_queue_plain(acs_conn), len);
	case ACS_SEG_RX_PENDING:
		return len;
	default:
		return acs_write_result(acs_seg_rx_att_err(seg_rx_result), len);
	}
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
ssize_t acs_data_in_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags)
{
	struct bt_acs_conn *acs_conn;
	enum acs_seg_rx_result res;
	uint8_t att_err;

	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	att_err = acs_write_check(conn, len, offset, &acs_conn);
	if (att_err != BT_ATT_ERR_SUCCESS) {
		return BT_GATT_ERR(att_err);
	}

	res = acs_seg_rx_feed(&acs_conn->data_rx, buf, len);

	switch (res) {
	case ACS_SEG_RX_COMPLETE:
		att_err = acs_data_in_unwrap_and_route(acs_conn);

		/* Release the buffer only if routing did not take it. */
		if (att_err != BT_ATT_ERR_SUCCESS && acs_conn->data_rx.buf != NULL) {
			acs_seg_rx_reset(&acs_conn->data_rx);
		}

		return acs_write_result(att_err, len);
	case ACS_SEG_RX_PENDING:
		return len;
	default:
		return acs_write_result(acs_seg_rx_att_err(res), len);
	}
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */
