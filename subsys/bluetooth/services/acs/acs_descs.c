/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/net_buf.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_cp_operands.h"
#include "acs_descs.h"
#include "acs_internal.h"
#include "acs_reply.h"
#include "acs_cp.h"
#include "acs_isc.h"
#include "acs_key_desc.h"
#include "acs_rmap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static const uint8_t all_records_filter[2] = {0xFF, 0xFF};

/* End the descriptor sequence with a Response Code. */
static int acs_all_active_finish(struct acs_reply *reply, uint8_t code)
{
	int err = acs_cp_rsp_status(reply, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS, code);

	reply->step = ACS_REPLY_DONE;
	return err;
}

int acs_all_active_step_isc(struct acs_reply *reply)
{
	struct net_buf *buf;
	struct net_buf_simple operand;

	buf = acs_prepare_reply_buf(reply);
	if (buf == NULL) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf,
		       BT_ACS_CP_OPCODE_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR_RESPONSE);
	net_buf_simple_init_with_data(&operand, (void *)all_records_filter,
				      sizeof(all_records_filter));

	if (acs_isc_build_response(&operand, buf) != 0) {
		LOG_ERR("Get All Active Descriptors: ISC build failed");
		return acs_all_active_finish(reply, BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	return acs_reply_submit(reply);
}

int acs_all_active_step_key(struct acs_reply *reply)
{
	struct net_buf *buf;
	struct net_buf_simple operand;
	int err;

	buf = acs_prepare_reply_buf(reply);
	if (buf == NULL) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_KEY_DESCRIPTOR_RESPONSE);
	net_buf_simple_init_with_data(&operand, (void *)all_records_filter,
				      sizeof(all_records_filter));

	err = acs_key_desc_build_response(&operand, buf, reply->conn);

	if (err == -ENOENT) {
		return acs_all_active_finish(reply, BT_ACS_CP_RESPONSE_SUCCESS);
	}
	if (err) {
		LOG_ERR("Get All Active Descriptors: Key build failed (%d)", err);
		return acs_all_active_finish(reply, BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	return acs_reply_submit(reply);
}

int acs_all_active_finish_success(struct acs_reply *reply)
{
	return acs_all_active_finish(reply, BT_ACS_CP_RESPONSE_SUCCESS);
}

struct acs_cp_result acs_cp_all_active_get(struct acs_reply *reply, struct net_buf_simple *payload)
{
	ARG_UNUSED(payload);

	struct acs_rmap_get_descriptor_req rm_operand;
	int err;

	rm_operand.map_id = reply->rmap != NULL ? reply->rmap->map->map_id : 0U;
	rm_operand.resource_handle_filter = ACS_RMAP_FILTER_ALL;
	err = acs_rmap_build_descriptor_response(reply->rmap, &rm_operand, reply->response);
	if (err != 0) {
		LOG_ERR("Get All Active Descriptors: RMAP build failed (%d)", err);
		return acs_cp_status(errno_to_acs_status(err));
	}

	/* Remaining descriptor payloads chain on the confirm-side sequence. */
	reply->step = ACS_REPLY_DESCS_ISC;
	return acs_cp_reply();
}
