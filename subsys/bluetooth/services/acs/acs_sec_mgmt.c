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

static inline bool is_active_algorithm_key_id(uint16_t key_id)
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
	return false;
}

static inline bool is_active_key_id(uint16_t key_id)
{
	if (is_active_algorithm_key_id(key_id)) {
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

/**
 * @brief Invalidate only the KDF child key, keeping the ECDH parent alive.
 *
 * Destroys the current PSA session key (the child), zeros crypto.session_key,
 * resets nonce counters, and clears kdf_child_active.  The ECDH parent in
 * ecdh_parent_key is untouched — the peer can Start Key Exchange(KDF) to
 * derive a new child without repeating the full ECDH handshake.
 */
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
static void invalidate_kdf_child(struct bt_acs_conn *acs_conn)
{
	acs_crypto_destroy_session_key(acs_conn);

	/* Restore the ECDH parent into crypto.session_key so a subsequent
	 * Start Key Exchange(KDF) + Key Exchange KDF can use it as IKM for
	 * bt_acs_crypto_derive_kdf_child_key().  Re-import it into PSA so
	 * the key handle is valid if any code path touches psa_key_id before
	 * the new child derivation overwrites it. */
	memcpy(acs_conn->crypto.session_key, acs_conn->ecdh_parent_key,
	       CONFIG_BT_ACS_SESSION_KEY_SIZE);
	acs_conn->crypto.tx_nonce_counter = 0;
	acs_conn->crypto.rx_nonce_counter = 0;
	acs_conn->kdf_child_active = false;
	acs_conn->status_flags &= ~BT_ACS_STATUS_SECURITY_ESTABLISHED;

	acs_crypto_import_session_key(acs_conn);
}
#endif

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

	if (ctx->acs_conn->key_state != BT_ACS_KEY_EXCHANGE_COMPLETE) {
		LOG_ERR("Invalidate Key ID 0x%04x: no security established", key_id);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_INVALIDATE_KEY,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		return;
	}

	if (key_id == 0xFFFF) {
		bt_acs_invalidate_security(ctx->conn);
		response_code = BT_ACS_CP_RESPONSE_SUCCESS;
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	} else if (key_id == ACS_KEY_ID_KDF) {
		if (!ctx->acs_conn->kdf_child_active) {
			/* Spec: "key ID that is already invalid → Procedure Not Applicable" */
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
		} else {
			invalidate_kdf_child(ctx->acs_conn);
#if defined(CONFIG_BT_SETTINGS) && !IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
			acs_session_store(ctx->conn, ctx->acs_conn);
#endif
			response_code = BT_ACS_CP_RESPONSE_SUCCESS;
			LOG_DBG("KDF child key invalidated (parent retained)");
		}
	} else if (is_active_algorithm_key_id(key_id) && ctx->acs_conn->kdf_child_active) {
		/* Algorithm keys (GCM, CCM, …) use the KDF child as their actual key
		 * material.  Invalidating them is equivalent to invalidating the child. */
		invalidate_kdf_child(ctx->acs_conn);
#if defined(CONFIG_BT_SETTINGS) && !IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
		acs_session_store(ctx->conn, ctx->acs_conn);
#endif
		response_code = BT_ACS_CP_RESPONSE_SUCCESS;
		LOG_DBG("Algorithm key 0x%04x invalidated (KDF child destroyed, parent retained)",
			key_id);
#endif
	} else if (is_active_key_id(key_id)) {
		ret = bt_acs_invalidate_security(ctx->conn);
		if (ret) {
			LOG_ERR("Failed to invalidate security for key ID 0x%04x: %d",
				key_id, ret);
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
		} else {
			response_code = BT_ACS_CP_RESPONSE_SUCCESS;
			LOG_DBG("Key ID 0x%04x invalidated (full teardown)", key_id);
		}
	} else {
		LOG_ERR("Invalidate Key received with unknown Key ID 0x%04x", key_id);
		response_code = BT_ACS_CP_RESPONSE_NO_RECORDS_FOUND;
	}

	acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_INVALIDATE_KEY, response_code);
}

#endif /* CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY */

#if IS_ENABLED(CONFIG_BT_ACS_ABORT)

/**
 * @brief Handle the ACS Abort Control Point procedure.
 *
 * ACS spec §4.4.4 (Abort) mandates a transactional model:
 *   - Success: stop every in-progress procedure AND suppress its response.
 *   - Error / Unsuccessful: leave the in-progress procedure running unchanged.
 *
 * The implementation therefore follows a three-phase pattern:
 *   1. Probe   — read-only scan of what would need to be torn down.
 *   2. Decide  — if nothing to abort, or if a response is already mid-flight
 *                (cannot be unsent), return UNSUCCESSFUL without mutating state.
 *   3. Commit  — only after both checks pass, tear down sequences, KEX, and
 *                data-op state, then send the ABORT SUCCESS response.
 *
 * ABORT bypasses the cp_proc.locked gate in acs_cp_write() (ABORT must be
 * able to preempt an in-progress procedure), so this handler also owns the
 * lock bookkeeping for its own response.
 */
void acs_sec_mgmt_abort(struct acs_cp_ctx *ctx)
{
	struct bt_acs_conn *acs_conn = ctx->acs_conn;
	struct k_work_sync sync;
	bool plain_cp_active;
	bool kex_in_progress;
	bool data_ops_pending;
	bool has_work;
	bool can_commit;

	plain_cp_active = (atomic_get(&acs_conn->cp_proc.locked) == 1);

	kex_in_progress = (acs_conn->key_state != BT_ACS_KEY_EXCHANGE_IDLE &&
			   acs_conn->key_state != BT_ACS_KEY_EXCHANGE_COMPLETE);

	data_ops_pending = false;
	for (uint8_t i = 0; i < CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN; i++) {
		if (atomic_ptr_get(&acs_conn->pending_reqs[i]) != NULL) {
			data_ops_pending = true;
			break;
		}
	}

	has_work = plain_cp_active || kex_in_progress || data_ops_pending;

	if (!has_work) {
		LOG_WRN("Abort requested with no in-progress procedure");
		/* Take the lock for our own response, since no proc holds it. */
		atomic_set(&acs_conn->cp_proc.locked, 1);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ABORT,
				  BT_ACS_CP_RESPONSE_ABORT_UNSUCCESSFUL);
		return;
	}

	/* If any CP-bound indication is already handed to the stack we
	 * cannot un-send it — spec §4.4.4: reply UNSUCCESSFUL and leave
	 * the aborted procedure running.
	 */
	can_commit = !acs_conn->cp_tx.tx_in_flight;
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	can_commit = can_commit && !acs_conn->indicate_tx.tx_in_flight;
#endif

	if (!can_commit) {
		LOG_WRN("response in flight — cannot abort");
		/* Do NOT touch any state. The aborted procedure continues. */
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ABORT,
				  BT_ACS_CP_RESPONSE_ABORT_UNSUCCESSFUL);
		return;
	}

	/*
	 * Plain CP: cancel any staged/pending response.  Sync-cancel the seg-TX work first so a
	 * racing work handler cannot fire between our free and the new ABORT response allocation.
	 */
	if (plain_cp_active) {
		k_work_cancel_sync(&acs_conn->cp_tx.tx_work, &sync);
		acs_seq_clear(ctx);
		if (acs_conn->cp_proc.response) {
			acs_buf_free(acs_conn->cp_proc.response);
			acs_conn->cp_proc.response = NULL;
		}
		/* lock stays held — ABORT now owns it for its own response */
	} else {
		atomic_set(&acs_conn->cp_proc.locked, 1);
	}

	/* Tear down KEX state (local only — always safe once probed). */
	if (kex_in_progress) {
		acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
		if (acs_conn->kex) {
			if (acs_conn->kex->ecdh_key_id != 0) {
				psa_destroy_key(acs_conn->kex->ecdh_key_id);
			}
			acs_kex_free(acs_conn->kex);
			acs_conn->kex = NULL;
		}
	}

	/* Drain any pending protected-resource requests. */
	if (data_ops_pending) {
		acs_prot_resource_req_abort_all(acs_conn);
	}

	/* Send success response — lock released on confirm as usual. */
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
