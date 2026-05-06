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
#include "acs_isc.h"
#include "acs_key_desc.h"
#include "acs_rmap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/*
 * Get All Active Descriptors — multi-step chained indication sequence.
 *
 * The ACS spec requires RMAP, ISC, KEY, and RC to be sent as separate confirmed
 * indications in order.  BLE only allows one in-flight indication at a time, so
 * each step is gated on the ATT confirm of the previous one.
 *
 * acs_cp_all_active_get sends RMAP first, then the table-driven sequence
 * advances ISC → KEY → RC on each confirm until the sequence completes.
 */

static int all_active_step_isc(struct acs_procedure *proc)
{
	static const uint8_t filter[2] = {0xFF, 0xFF};
	struct net_buf *buf;
	struct net_buf_simple operand;
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);

	buf = acs_prepare_reply_buf(proc, reply_mode.channel, reply_mode.encrypted);
	if (!buf) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf,
		       BT_ACS_CP_OPCODE_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR_RESPONSE);
	net_buf_simple_init_with_data(&operand, (void *)filter, sizeof(filter));

	if (acs_isc_build_response(&operand, &buf->b) != 0) {
		LOG_ERR("Get All Active Descriptors: ISC build failed");
		acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		acs_seq_clear(proc);
		return 0;
	}
	return acs_cp_send_reply(proc);
}

static int all_active_step_key(struct acs_procedure *proc)
{
	static const uint8_t filter[2] = {0xFF, 0xFF};
	struct net_buf *buf;
	struct net_buf_simple operand;
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	int err;

	buf = acs_prepare_reply_buf(proc, reply_mode.channel, reply_mode.encrypted);
	if (!buf) {
		return -ENOMEM;
	}

	uint8_t server_nonce[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE];

	err = acs_crypto_get_server_nonce_fixed(proc->acs_conn, server_nonce,
						sizeof(server_nonce));
	if (err) {
		return err;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_KEY_DESCRIPTOR_RESPONSE);
	net_buf_simple_init_with_data(&operand, (void *)filter, sizeof(filter));

	err = acs_key_desc_build_response(&operand, &buf->b, server_nonce);

	if (err == -ENOENT) {
		acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
				  BT_ACS_CP_RESPONSE_SUCCESS);
		acs_seq_clear(proc);
		return 0;
	}
	if (err) {
		LOG_ERR("Get All Active Descriptors: Key build failed (%d)", err);
		acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		acs_seq_clear(proc);
		return 0;
	}
	return acs_cp_send_reply(proc);
}

static int all_active_step_rc(struct acs_procedure *proc)
{
	/* Send the terminal Response Code first — acs_cp_send_reply adds a TX
	 * ref that keeps the request alive.  Clear the sequence afterwards so
	 * the indication-complete callback sees reply_seq.desc == NULL and
	 * does not try to advance further.
	 */
	int err = acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
				    BT_ACS_CP_RESPONSE_SUCCESS);

	acs_seq_clear(proc);
	return err;
}

static const acs_seq_step_fn all_active_steps[] = {
	all_active_step_isc,
	all_active_step_key,
	all_active_step_rc,
};

static const struct acs_seq_desc all_active_seq = {
	.steps = all_active_steps,
	.step_count = ARRAY_SIZE(all_active_steps),
};

int acs_cp_all_active_get(struct acs_procedure *proc)
{
	struct net_buf *buf;
	struct acs_rmap_get_descriptor_req rm_operand;
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	int err;

	buf = acs_prepare_reply_buf(proc, reply_mode.channel, reply_mode.encrypted);
	if (buf == NULL) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_RESTRICTION_MAP_DESCRIPTOR_RESPONSE);

	rm_operand.map_id = proc->acs_conn->restriction_map_id;
	rm_operand.resource_handle_filter = ACS_RMAP_FILTER_ALL;
	err = acs_rmap_build_descriptor_response(&rm_operand, &buf->b);
	if (err != 0) {
		LOG_ERR("Get All Active Descriptors: RMAP build failed (%d)", err);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	acs_seq_begin(proc, &all_active_seq);

	err = acs_cp_send_reply(proc);
	if (err) {
		acs_seq_abort(proc);
		LOG_WRN("RMAP indication send failed (%d)", err);
	}
	return err;
}
