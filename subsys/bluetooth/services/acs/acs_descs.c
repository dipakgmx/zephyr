/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/net_buf.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_cp_operands.h"
#include "acs_cp_handlers.h"
#include "acs_internal.h"
#include "acs_reply.h"
#include "acs_isc.h"
#include "acs_key_desc.h"
#include "acs_rmap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Append a descriptor response message that starts with rsp_opcode. */
static struct net_buf *add_descriptor_response(struct acs_reply *reply, uint8_t rsp_opcode)
{
	struct net_buf *buf = acs_reply_add_message(reply);

	if (buf != NULL) {
		net_buf_add_u8(buf, rsp_opcode);
	}

	return buf;
}

/*
 * Get All Active Descriptors (§4.4.3.1): the Restriction Map, Information
 * Security Configuration and Key Descriptor procedures in that order, each
 * answered with its own response. The dispatcher then adds the Success
 * Response Code. Key descriptors are always supported where this builds.
 */
uint8_t acs_cp_all_active_get(struct acs_reply *reply, struct net_buf_simple *payload)
{
	struct net_buf *buf;
	uint8_t rc;

	ARG_UNUSED(payload);

	buf = add_descriptor_response(reply, BT_ACS_CP_OPCODE_RESTRICTION_MAP_DESCRIPTOR_RESPONSE);
	if (buf == NULL) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}
	rc = acs_rmap_build_descriptor_response(reply->rmap, ACS_RMAP_FILTER_ALL, buf);
	if (rc != BT_ACS_CP_RESPONSE_SUCCESS) {
		LOG_ERR("Get All Active Descriptors: restriction map failed (0x%02x)", rc);
		return rc;
	}

	buf = add_descriptor_response(
		reply, BT_ACS_CP_OPCODE_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR_RESPONSE);
	if (buf == NULL) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}
	rc = acs_isc_build_response(BT_ACS_ISC_ALL_RECORDS_FILTER, buf);
	if (rc != BT_ACS_CP_RESPONSE_SUCCESS) {
		LOG_ERR("Get All Active Descriptors: ISC descriptor failed (0x%02x)", rc);
		return rc;
	}

	buf = add_descriptor_response(reply, BT_ACS_CP_OPCODE_KEY_DESCRIPTOR_RESPONSE);
	if (buf == NULL) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}
	rc = acs_key_desc_build_response(BT_ACS_GET_KEY_DESC_ALL_RECORDS_FILTER, buf,
					 reply->conn);
	if (rc != BT_ACS_CP_RESPONSE_SUCCESS) {
		LOG_ERR("Get All Active Descriptors: key descriptor failed (0x%02x)", rc);
		return rc;
	}

	return BT_ACS_CP_RESPONSE_SUCCESS;
}
