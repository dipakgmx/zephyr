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
#include "acs_rhandle.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
void acs_cp_handle_get_key_descriptor(struct acs_cp_ctx *ctx, struct net_buf_simple *buf)
{
	struct net_buf *rsp_buf;
	int build_err;

	rsp_buf = acs_cp_rsp_alloc(ctx);
	if (!rsp_buf) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}
	uint8_t server_nonce[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE];

	build_err = acs_crypto_get_server_nonce_fixed(ctx->acs_conn, server_nonce,
						      sizeof(server_nonce));
	if (build_err) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR,
				  errno_to_acs_status(build_err));
		return;
	}

	net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_KEY_DESCRIPTOR_RESPONSE);
	build_err = acs_key_desc_build_response(buf, &rsp_buf->b, server_nonce);
	if (build_err) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR,
				  errno_to_acs_status(build_err));
	} else {
		int err = acs_cp_rsp_send(ctx);

		if (err) {
			LOG_WRN("CP indicate failed for opcode 0x%02x: %d",
				BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR, err);
		}
	}
}
#endif /* CONFIG_BT_ACS_ANY_KEY_EXCHANGE */

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
void acs_cp_handle_get_isc_descriptor(struct acs_cp_ctx *ctx, struct net_buf_simple *buf)
{
	struct net_buf *rsp_buf;
	int build_err;

	rsp_buf = acs_cp_rsp_alloc(ctx);
	if (!rsp_buf) {
		acs_cp_rsp_status(
			ctx, BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR,
			BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}
	net_buf_add_u8(rsp_buf,
		       BT_ACS_CP_OPCODE_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR_RESPONSE);
	build_err = acs_isc_build_response(buf, &rsp_buf->b);
	if (build_err) {
		acs_cp_rsp_status(
			ctx, BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR,
			errno_to_acs_status(build_err));
	} else {
		int err = acs_cp_rsp_send(ctx);

		if (err) {
			LOG_WRN("CP indicate failed for opcode 0x%02x: %d",
				BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR,
				err);
		}
	}
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

#if IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP)
void acs_cp_handle_get_resource_handle_uuid_map(struct acs_cp_ctx *ctx)
{
	struct net_buf *rsp_buf;
	int build_err;

	rsp_buf = acs_cp_rsp_alloc(ctx);
	if (!rsp_buf) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}
	net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_RESOURCE_HANDLE_UUID_MAP_RESPONSE);
	build_err = acs_rhandle_build_map_response(&rsp_buf->b);
	if (build_err) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP,
				  errno_to_acs_status(build_err));
	} else {
		int err = acs_cp_rsp_send(ctx);

		if (err) {
			LOG_WRN("CP indicate failed for opcode 0x%02x: %d",
				BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP, err);
		}
	}
}
#endif /* CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP */

void acs_cp_handle_get_svc_char_uuids(struct acs_cp_ctx *ctx, struct net_buf_simple *buf)
{
	uint16_t resource_handle;
	struct net_buf *rsp_buf;
	int err;

	if (buf->len < 2) {
		acs_cp_rsp_status(
			ctx, BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE,
			BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	resource_handle = net_buf_simple_pull_le16(buf);

	rsp_buf = acs_cp_rsp_alloc(ctx);
	if (!rsp_buf) {
		acs_cp_rsp_status(
			ctx, BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE,
			BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}
	net_buf_add_u8(rsp_buf,
		       BT_ACS_CP_OPCODE_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE_RESPONSE);

	err = acs_rhandle_lookup_svc_char(resource_handle, &rsp_buf->b);

	if (err == -ENOENT) {
		acs_cp_rsp_status(
			ctx, BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE,
			BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	} else if (err) {
		acs_cp_rsp_status(
			ctx, BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE,
			errno_to_acs_status(err));
	} else {
		int send_err = acs_cp_rsp_send(ctx);

		if (send_err) {
			LOG_WRN("CP indicate failed for opcode 0x%02x: %d",
				BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE,
				send_err);
		}
	}
}
