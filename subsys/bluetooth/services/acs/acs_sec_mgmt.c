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

#include "acs_internal.h"
#include "acs_cp_handlers.h"
#include "acs_key_desc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)

static inline bool is_active_key_id(uint16_t key_id)
{
	if (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) && key_id == ACS_KEY_ID_CCM) {
		return true;
	}
	if (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) && key_id == ACS_KEY_ID_GCM) {
		return true;
	}
	if (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC) && key_id == ACS_KEY_ID_CMAC) {
		return true;
	}
	if (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) && key_id == ACS_KEY_ID_GMAC) {
		return true;
	}
	if (IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH) && key_id == ACS_KEY_ID_ECDH) {
		return true;
	}
	if (IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB) && key_id == ACS_KEY_ID_OOB) {
		return true;
	}
	if (IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) && key_id == ACS_KEY_ID_KDF) {
		return true;
	}
	return false;
}

static int invalidate_self_step(struct acs_cp_ctx *ctx)
{
	acs_seq_clear(ctx);
	bt_acs_invalidate_security(ctx->conn);
	return 0;
}

static const acs_seq_step_fn invalidate_self_steps[] = {
	invalidate_self_step,
};

static const struct acs_seq_desc invalidate_self_seq = {
	.steps = invalidate_self_steps,
	.step_count = ARRAY_SIZE(invalidate_self_steps),
};

void acs_sec_mgmt_invalidate_all(struct acs_cp_ctx *ctx)
{
	struct bt_acs_conn const *acs_conn = ctx->acs_conn;
	uint8_t req_idx;
	int count = 0;

	if (!acs_conn || acs_conn->key_state != BT_ACS_KEY_EXCHANGE_COMPLETE) {
		LOG_WRN("Invalidate All: rejected — sender has no established ACS security");
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		return;
	}

	req_idx = bt_conn_index(ctx->conn);

	for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		struct bt_acs_conn *ac = acs_conn_by_index(i);

		if (!ac || !ac->conn || i == req_idx) {
			continue;
		}

		if (ac->key_state == BT_ACS_KEY_EXCHANGE_COMPLETE) {
			LOG_DBG("Invalidating security for conn %p", (void *)ac->conn);
			bt_acs_invalidate_security(ac->conn);
			count++;
		}
	}

#if defined(CONFIG_BT_SETTINGS)
	acs_session_clear_all(ctx->conn);
#endif /* CONFIG_BT_SETTINGS */

	LOG_DBG("Invalidated security for %d other connection(s); deferring self", count);

	/* Defer self-invalidation until after the success response is confirmed. */
	acs_seq_begin(ctx, &invalidate_self_seq);

	acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY,
			  BT_ACS_CP_RESPONSE_SUCCESS);
}

void acs_sec_mgmt_invalidate_key(struct acs_cp_ctx *ctx, struct net_buf_simple *buf)
{
	struct acs_cp_invalidate_key_req invalidate_req;
	uint16_t key_id;
	uint8_t response_code;
	int ret;

	if (buf->len < sizeof(struct acs_cp_invalidate_key_req)) {
		LOG_ERR("Invalid Invalidate Key operand length: need %zu, have %u",
			sizeof(struct acs_cp_invalidate_key_req), buf->len);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_INVALIDATE_KEY,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	/* Pull operand data before any response buffer init. */
	memcpy(&invalidate_req, net_buf_simple_pull_mem(buf, sizeof(invalidate_req)),
	       sizeof(invalidate_req));
	key_id = sys_le16_to_cpu(invalidate_req.key_id);

	if (!ctx->acs_conn) {
		LOG_ERR("Invalidate Key received for unknown connection");
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_INVALIDATE_KEY,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}

	if (key_id == 0xFFFF) {
		if (ctx->acs_conn->key_state == BT_ACS_KEY_EXCHANGE_IDLE) {
			LOG_ERR("Invalidate All Keys not allowed with no security established");
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
		} else {
			bt_acs_invalidate_security(ctx->conn);
			response_code = BT_ACS_CP_RESPONSE_SUCCESS;
		}
	} else if (is_active_key_id(key_id)) {
		if (ctx->acs_conn->key_state != BT_ACS_KEY_EXCHANGE_COMPLETE) {
			LOG_ERR("Invalidate Key ID 0x%04x not allowed with no security established",
				key_id);
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
		} else {
			ret = bt_acs_invalidate_security(ctx->conn);
			if (ret) {
				LOG_ERR("Failed to invalidate security for key ID 0x%04x: %d",
					key_id, ret);
				response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
			} else {
				response_code = BT_ACS_CP_RESPONSE_SUCCESS;
				LOG_DBG("Key ID 0x%04x invalidated", key_id);
			}
		}
	} else {
		LOG_ERR("Invalidate Key received with unknown Key ID 0x%04x", key_id);
		response_code = BT_ACS_CP_RESPONSE_NO_RECORDS_FOUND;
	}

	acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_INVALIDATE_KEY, response_code);
}

#endif /* CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY */

#if IS_ENABLED(CONFIG_BT_ACS_ABORT)

void acs_sec_mgmt_abort(struct acs_cp_ctx *ctx)
{
	struct bt_acs_conn *acs_conn = ctx->acs_conn;
	bool kex_in_progress;
	bool data_ops_pending;

	if (!acs_conn) {
		LOG_WRN("Abort requested for unknown ACS connection");
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ABORT,
				  BT_ACS_CP_RESPONSE_ABORT_UNSUCCESSFUL);
		return;
	}

	kex_in_progress = (acs_conn->key_state != BT_ACS_KEY_EXCHANGE_IDLE &&
			   acs_conn->key_state != BT_ACS_KEY_EXCHANGE_COMPLETE);
	data_ops_pending = false;

	for (uint8_t i = 0; i < CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN; i++) {
		if (atomic_ptr_get(&acs_conn->pending_reqs[i]) != NULL) {
			data_ops_pending = true;
			break;
		}
	}

	if (!kex_in_progress && !data_ops_pending) {
		LOG_WRN("Abort requested with no in-progress procedure (key_state=%u)",
			(unsigned int)acs_conn->key_state);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ABORT,
				  BT_ACS_CP_RESPONSE_ABORT_UNSUCCESSFUL);
		return;
	}

	if (kex_in_progress) {
		enum bt_acs_key_exchange_state old_state = acs_conn->key_state;

		acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
		if (acs_conn->kex) {
			if (acs_conn->kex->ecdh_key_id != 0) {
				psa_destroy_key(acs_conn->kex->ecdh_key_id);
			}
			acs_kex_free(acs_conn->kex);
			acs_conn->kex = NULL;
		}
		LOG_DBG("Abort: cleared key_state from %u", (unsigned int)old_state);
	}

	acs_prot_resource_req_abort_all(acs_conn);

	acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ABORT, BT_ACS_CP_RESPONSE_SUCCESS);
}

#endif /* CONFIG_BT_ACS_ABORT */

#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)

void acs_sec_mgmt_set_security_switch(struct acs_cp_ctx *ctx, struct net_buf_simple *buf)
{
	struct acs_cp_sec_switch_req switch_req;
	uint8_t switch_state;

	if (buf->len < sizeof(struct acs_cp_sec_switch_req)) {
		LOG_ERR("Set Security Controls Switch operand too short: %u", buf->len);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	if (!ctx->acs_conn) {
		LOG_ERR("Set Security Controls Switch for unknown ACS connection");
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}

	/* Pull operand data before any response buffer init. */
	memcpy(&switch_req, net_buf_simple_pull_mem(buf, sizeof(switch_req)), sizeof(switch_req));
	switch_state = switch_req.switch_state & 0x01;

	if (switch_state) {
		ctx->acs_conn->status_flags |= BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	} else {
		ctx->acs_conn->status_flags &= ~BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	}

	acs_status_indicate(ctx->conn);

	LOG_DBG("Security controls switch set to %u", switch_state);

	acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH,
			  BT_ACS_CP_RESPONSE_SUCCESS);
}

#endif /* CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_URI)

void acs_sec_mgmt_get_key_uri(struct acs_cp_ctx *ctx, struct net_buf_simple *buf)
{
	struct acs_cp_get_key_uri_req key_uri_req;
	struct acs_cp_key_uri_rsp_hdr *hdr;
	const struct bt_acs_cb *cb;
	struct net_buf_simple *rsp_buf;
	uint8_t *uri_ptr;
	uint16_t uri_max;
	uint16_t uri_len;
	uint16_t key_id;
	int arm_err;
	int err;

	if (buf->len < sizeof(struct acs_cp_get_key_uri_req)) {
		LOG_ERR("Get Key URI operand too short: %u", buf->len);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_GET_KEY_URI,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	/* Pull operand data before any response buffer init. */
	memcpy(&key_uri_req, net_buf_simple_pull_mem(buf, sizeof(key_uri_req)),
	       sizeof(key_uri_req));
	key_id = sys_le16_to_cpu(key_uri_req.key_id);
	cb = acs_cb_get();

	if (!cb || !cb->key_uri_get) {
		LOG_WRN("Get Key URI for key_id=0x%04x but no callback is registered", key_id);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_GET_KEY_URI,
				  BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
		return;
	}

	if (!ctx->acs_conn) {
		LOG_ERR("Get Key URI for key_id=0x%04x with no ACS connection context", key_id);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_GET_KEY_URI,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}

	{
		struct net_buf *nbuf = acs_cp_rsp_alloc(ctx);

		if (!nbuf) {
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_GET_KEY_URI,
					  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
			return;
		}
		net_buf_add_u8(nbuf, BT_ACS_CP_OPCODE_KEY_URI_RESPONSE);
		rsp_buf = &nbuf->b;
	}

	hdr = net_buf_simple_add(rsp_buf, sizeof(struct acs_cp_key_uri_rsp_hdr));
	hdr->key_id = sys_cpu_to_le16(key_id);

	uri_max = MIN(net_buf_simple_tailroom(rsp_buf), (uint16_t)CONFIG_BT_ACS_KEY_URI_MAX_LEN);
	uri_ptr = rsp_buf->data + rsp_buf->len;
	uri_len = 0;

	err = cb->key_uri_get(ctx->conn, key_id, uri_ptr, uri_max, &uri_len);

	if (err || uri_len == 0) {
		LOG_DBG("key_uri_get key_id=0x%04x err=%d", key_id, err);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_GET_KEY_URI,
				  BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
		return;
	}

	net_buf_simple_add(rsp_buf, uri_len);

	LOG_DBG("Key URI response: key_id=0x%04x uri_len=%u", key_id, uri_len);

	arm_err = acs_cp_rsp_send(ctx);

	if (arm_err) {
		LOG_WRN("Key URI response send failed for key_id=0x%04x: %d", key_id, arm_err);
	}
}

#endif /* CONFIG_BT_ACS_KEY_URI */

#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)

void acs_sec_mgmt_initiate_pairing(struct acs_cp_ctx *ctx)
{
	int err;

	err = bt_conn_set_security(ctx->conn, BT_SECURITY_L2 | BT_SECURITY_FORCE_PAIR);

	if (err) {
		LOG_ERR("Initiate Pairing: bt_conn_set_security failed: %d", err);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_INITIATE_PAIRING,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}

	acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_INITIATE_PAIRING, BT_ACS_CP_RESPONSE_SUCCESS);
}

#endif /* CONFIG_BT_ACS_INITIATE_PAIRING */
