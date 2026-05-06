/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_cp.h"
#include "acs_internal.h"
#include "acs_rmap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

int acs_cp_handle_get_restriction_map_id_list(struct acs_procedure *proc)
{
	struct net_buf *rsp_buf;
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	int build_err;

	rsp_buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);
	if (!rsp_buf) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_RESTRICTION_MAP_ID_LIST_RESPONSE);
	build_err = acs_rmap_build_id_list_response(&rsp_buf->b);
	if (build_err) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST,
					 errno_to_acs_status(build_err));
	} else {
		int err = acs_cp_send_reply(proc);

		if (err) {
			LOG_WRN("CP indicate failed for opcode 0x%02x: %d",
				BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST, err);
		}
		return err;
	}
}

int acs_cp_handle_get_restriction_map_descriptor(struct acs_procedure *proc,
						 struct net_buf_simple *buf)
{
	struct acs_rmap_get_descriptor_req desc_req;
	struct net_buf *rsp_buf;
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	int build_err;

	if (buf->len != sizeof(struct acs_rmap_get_descriptor_req)) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	/* Pull all operand data before response buffer init to avoid aliasing. */
	desc_req.map_id = net_buf_simple_pull_le16(buf);
	desc_req.resource_handle_filter = net_buf_simple_pull_le16(buf);

	rsp_buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);
	if (!rsp_buf) {
		LOG_WRN("buffer pool exhausted");
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_RESTRICTION_MAP_DESCRIPTOR_RESPONSE);

	build_err = acs_rmap_build_descriptor_response(&desc_req, &rsp_buf->b);
	if (build_err) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR,
					 errno_to_acs_status(build_err));
	} else {
		int err = acs_cp_send_reply(proc);

		if (err) {
			LOG_WRN("CP indicate failed for opcode 0x%02x: %d",
				BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR, err);
		}
		return err;
	}
}

int acs_cp_handle_activate_restriction_map(struct acs_procedure *proc, struct net_buf_simple *buf)
{
	uint16_t map_id;
	struct bt_acs_restriction_map map;

	if (!IS_ENABLED(CONFIG_BT_ACS_MULTIPLE_RESTRICTION_MAPS)) {
		LOG_ERR("Activate Restriction Map: feature not supported");
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP,
					 BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	if (buf->len < 2) {
		LOG_ERR("Activate Restriction Map: operand too short (%u)", buf->len);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	map_id = net_buf_simple_pull_le16(buf);
	memset(&map, 0, sizeof(map));

	if (acs_rmap_lookup(map_id, &map) != 0) {
		LOG_WRN("Activate Restriction Map: map ID 0x%04x not found", map_id);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP,
					 BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	}

	/* Protected maps (ISC ID != 0) require ACS security to be established.
	 * On the data channel (proc != NULL) security was verified during
	 * decryption.  On the plain CP the peer must have completed key exchange.
	 */
	if (map.map_isc_id != 0 && proc == NULL &&
	    proc->acs_conn->key_state != BT_ACS_KEY_EXCHANGE_COMPLETE) {
		LOG_WRN("Activate Restriction Map: map 0x%04x is protected (ISC 0x%04x) — "
			"security not established",
			map_id, map.map_isc_id);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	proc->acs_conn->restriction_map_id = map_id;
	LOG_DBG("Restriction map 0x%04x activated", map_id);

	return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP,
				 BT_ACS_CP_RESPONSE_SUCCESS);
}
