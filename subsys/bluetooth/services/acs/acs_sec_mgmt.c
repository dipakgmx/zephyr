/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <mbedtls/platform_util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_internal.h"
#include "acs_cp_handlers.h"
#include "acs_key_desc.h"
#include "acs_key_exchange.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)

#define ACS_ACTIVE_ALGORITHM_KEY_ID_MASK                                                           \
	((IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) ? BIT(ACS_KEY_ID_CCM) : 0U) |          \
	 (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ? BIT(ACS_KEY_ID_GCM) : 0U) |          \
	 (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC) ? BIT(ACS_KEY_ID_CMAC) : 0U) |        \
	 (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) ? BIT(ACS_KEY_ID_GMAC) : 0U))

#define ACS_ACTIVE_KEY_ID_MASK                                                                     \
	(ACS_ACTIVE_ALGORITHM_KEY_ID_MASK |                                                        \
	 (IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH) ? BIT(ACS_KEY_ID_ECDH) : 0U) |               \
	 (IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB) ? BIT(ACS_KEY_ID_OOB) : 0U) |                 \
	 (IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) ? BIT(ACS_KEY_ID_KDF) : 0U))

static inline bool is_active_algorithm_key_id(uint16_t key_id)
{
	return key_id < (sizeof(unsigned long) * 8U) &&
	       ((ACS_ACTIVE_ALGORITHM_KEY_ID_MASK & BIT(key_id)) != 0U);
}

static inline bool is_active_key_id(uint16_t key_id)
{
	return key_id < (sizeof(unsigned long) * 8U) &&
	       ((ACS_ACTIVE_KEY_ID_MASK & BIT(key_id)) != 0U);
}

static bool current_key_is_valid(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	struct bt_acs_runtime_key_state *current_key;

	if (acs_crypto_current_key_lookup(acs_conn, key_id, &current_key) != 0) {
		return false;
	}

	return current_key->psa_key_id != 0U;
}

static int invalidate_self_step(struct acs_procedure *proc)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;

	acs_seq_clear(proc);
	bt_acs_invalidate_security(acs_conn->conn);
	return 0;
}

static const acs_seq_step_fn invalidate_self_steps[] = {
	invalidate_self_step,
};

static const struct acs_seq_desc invalidate_self_seq = {
	.steps = invalidate_self_steps,
	.step_count = ARRAY_SIZE(invalidate_self_steps),
};

int acs_sec_mgmt_invalidate_all(struct acs_procedure *proc)
{
	struct bt_acs_conn const *acs_conn = proc->acs_conn;
	uint8_t req_idx;
	int count = 0;

	if (!acs_conn || !acs_session_established(acs_conn)) {
		LOG_WRN("Invalidate All: rejected — sender has no established ACS security");
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	req_idx = bt_conn_index(proc->acs_conn->conn);

	for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		struct bt_acs_conn *ac = acs_conn_by_index(i);

		if (!ac || !ac->conn || i == req_idx) {
			continue;
		}

		if (acs_session_established(ac)) {
			LOG_DBG("Invalidating security for conn %p", (void *)ac->conn);
			bt_acs_invalidate_security(ac->conn);
			count++;
		}
	}

	acs_session_cache_clear_all_except(bt_conn_get_dst(proc->acs_conn->conn));

#if defined(CONFIG_BT_SETTINGS)
	acs_session_clear_all(proc->acs_conn->conn);
#endif /* CONFIG_BT_SETTINGS */

	LOG_DBG("Invalidated security for %d other connection(s); deferring self", count);

	/* Defer self-invalidation until after the success response is confirmed. */
	acs_seq_begin(proc, &invalidate_self_seq);

	return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY,
				 BT_ACS_CP_RESPONSE_SUCCESS);
}

/**
 * @brief Invalidate only the KDF child key, keeping the ECDH parent alive.
 *
 * Destroys the current KDF key handle and clears its runtime key context.
 * The ECDH parent current key remains intact, so the peer can Start
 * Key Exchange(KDF) to derive a new child without repeating the full ECDH
 * handshake.
 */
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
static void invalidate_kdf_child(struct bt_acs_conn *acs_conn)
{
	struct bt_acs_runtime_key_state *kdf_key;
	int err = acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_KDF, &kdf_key);

	if (err) {
		LOG_ERR("Missing KDF runtime key state");
		return;
	}

	acs_crypto_destroy_current_key(kdf_key);
	mbedtls_platform_zeroize(kdf_key->key, sizeof(kdf_key->key));
	acs_crypto_rebind_key_desc_runtimes(acs_conn);
	acs_crypto_reset_key_desc_runtime_counters(acs_conn, ACS_KEY_ID_KDF);
	acs_conn->status_flags &= ~BT_ACS_STATUS_SECURITY_ESTABLISHED;
}
#endif

int acs_sec_mgmt_invalidate_key(struct acs_procedure *proc, struct net_buf_simple *buf)
{
	struct acs_cp_invalidate_key_req invalidate_req;
	uint16_t key_id;
	uint8_t response_code;
	int ret;

	if (buf->len < sizeof(struct acs_cp_invalidate_key_req)) {
		LOG_ERR("Invalid Invalidate Key operand length: need %zu, have %u",
			sizeof(struct acs_cp_invalidate_key_req), buf->len);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_INVALIDATE_KEY,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	/* Pull operand data before any response buffer init. */
	memcpy(&invalidate_req, net_buf_simple_pull_mem(buf, sizeof(invalidate_req)),
	       sizeof(invalidate_req));
	key_id = sys_le16_to_cpu(invalidate_req.key_id);

	if (!proc->acs_conn) {
		LOG_ERR("Invalidate Key received for unknown connection");
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_INVALIDATE_KEY,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	if (key_id == BT_ACS_GET_KEY_DESC_ALL_RECORDS_FILTER) {
		if (acs_key_exchange_established_key(proc->acs_conn) == NULL) {
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_INVALIDATE_KEY,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		}

		bt_acs_invalidate_security(proc->acs_conn->conn);
		response_code = BT_ACS_CP_RESPONSE_SUCCESS;
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	} else if (key_id == ACS_KEY_ID_KDF) {
		struct bt_acs_runtime_key_state *kdf_key;

		ret = acs_crypto_current_key_lookup(proc->acs_conn, ACS_KEY_ID_KDF, &kdf_key);
		if (ret) {
			LOG_ERR("Missing KDF runtime key state");
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
		} else if (kdf_key->psa_key_id == 0U) {
			/* Spec §4.4.3.12: "key ID that is already invalid → Procedure Not
			 * Applicable" */
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
		} else {
			invalidate_kdf_child(proc->acs_conn);
#if defined(CONFIG_BT_SETTINGS)
			acs_session_store(proc->acs_conn->conn, proc->acs_conn);
#endif
			{
				const struct bt_acs_cb *cb = acs_cb_get();

				if (cb && cb->security_invalidated) {
					cb->security_invalidated(proc->acs_conn->conn);
				}
			}
			acs_status_indicate(proc->acs_conn->conn);
			response_code = BT_ACS_CP_RESPONSE_SUCCESS;
			LOG_DBG("KDF child key invalidated");
		}
	} else if (is_active_algorithm_key_id(key_id)) {
		struct bt_acs_runtime_key_state *kdf_key;

		ret = acs_crypto_current_key_lookup(proc->acs_conn, ACS_KEY_ID_KDF, &kdf_key);
		if (ret == 0 && kdf_key->psa_key_id != 0U) {
			/* Algorithm keys (GCM, CCM, …) use the KDF child as their actual key
			 * material.  Invalidating them is equivalent to invalidating the child. */
			invalidate_kdf_child(proc->acs_conn);
#if defined(CONFIG_BT_SETTINGS)
			acs_session_store(proc->acs_conn->conn, proc->acs_conn);
#endif
			{
				const struct bt_acs_cb *cb = acs_cb_get();

				if (cb && cb->security_invalidated) {
					cb->security_invalidated(proc->acs_conn->conn);
				}
			}
			acs_status_indicate(proc->acs_conn->conn);
			response_code = BT_ACS_CP_RESPONSE_SUCCESS;
			LOG_DBG("Algorithm key 0x%04x invalidated (KDF child destroyed, parent "
				"retained)",
				key_id);
		} else if (ret) {
			LOG_ERR("Missing KDF runtime key state");
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
		} else {
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
		}
#endif
	} else if (is_active_key_id(key_id)) {
		if (!current_key_is_valid(proc->acs_conn, key_id)) {
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
		} else {
			ret = bt_acs_invalidate_security(proc->acs_conn->conn);
			if (ret) {
				LOG_ERR("Failed to invalidate security for key ID 0x%04x: %d",
					key_id, ret);
				response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
			} else {
				response_code = BT_ACS_CP_RESPONSE_SUCCESS;
				LOG_DBG("Key ID 0x%04x invalidated (full teardown)", key_id);
			}
		}
	} else {
		LOG_ERR("Invalidate Key received with unknown Key ID 0x%04x", key_id);
		response_code = BT_ACS_CP_RESPONSE_NO_RECORDS_FOUND;
	}

	return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_INVALIDATE_KEY, response_code);
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
 * ABORT bypasses the plain_cp_proc.plain_cp.locked gate in acs_cp_write() (ABORT must be
 * able to preempt an in-progress procedure), so this handler also owns the
 * lock bookkeeping for its own response.
 */
int acs_sec_mgmt_abort(struct acs_procedure *proc)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;
	struct k_work_sync sync;
	bool plain_cp_active;
	bool kex_in_progress;
	bool data_ops_pending;
	bool has_work;
	bool can_commit;

	plain_cp_active = (atomic_get(&acs_conn->plain_cp_proc.plain_cp.locked) == 1);

	kex_in_progress = acs_kex_in_progress(acs_conn);

	data_ops_pending = false;
	for (uint8_t i = 0; i < CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN; i++) {
		if (atomic_ptr_get(&acs_conn->inflight_reqs[i]) != NULL) {
			data_ops_pending = true;
			break;
		}
	}

	has_work = plain_cp_active || kex_in_progress || data_ops_pending;

	if (!has_work) {
		LOG_WRN("Abort requested with no in-progress procedure");
		/* Take the lock for our own response, since no proc holds it. */
		atomic_set(&acs_conn->plain_cp_proc.plain_cp.locked, 1);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ABORT,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	/* If a CP indication is already handed to the BLE stack we cannot
	 * un-send it, but we CAN suppress all subsequent indications once it
	 * confirms.  Set a deferred flag so the confirm callback tears down the
	 * procedure and sends ABORT SUCCESS when the channel is free.
	 *
	 * We must not try to send a response here — the TX channel is occupied
	 * and the send would fail with -EBUSY.
	 */
	can_commit = !acs_conn->cp_tx.tx_in_flight;
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	can_commit = can_commit && !acs_conn->indicate_tx.tx_in_flight;
#endif

	if (!can_commit) {
		LOG_DBG("Abort deferred — indication in flight, will commit on confirm");
		acs_conn->plain_cp_proc.plain_cp.abort_pending = true;
		return 0;
	}

	/*
	 * Plain CP: cancel any staged/pending response.  Sync-cancel the seg-TX work first so a
	 * racing work handler cannot fire between our free and the new ABORT response allocation.
	 */
	if (plain_cp_active) {
		k_work_cancel_sync(&acs_conn->cp_tx.tx_work, &sync);
		acs_seq_clear(proc);
		if (acs_conn->plain_cp_proc.buffers.response_buf) {
			acs_buf_free(acs_conn->plain_cp_proc.buffers.response_buf);
			acs_conn->plain_cp_proc.buffers.response_buf = NULL;
		}
		/* lock stays held — ABORT now owns it for its own response */
	} else {
		atomic_set(&acs_conn->plain_cp_proc.plain_cp.locked, 1);
	}

	/* Tear down KEX state (local only — always safe once probed). */
	if (kex_in_progress) {
		acs_key_exchange_abort(acs_conn);
	}

	/* Drain any pending protected-resource requests. */
	if (data_ops_pending) {
		acs_procedure_abort_all(acs_conn);
	}

	/* Send success response — lock released on confirm as usual. */
	return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ABORT, BT_ACS_CP_RESPONSE_SUCCESS);
}

#endif /* CONFIG_BT_ACS_ABORT */

#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)

int acs_sec_mgmt_set_security_switch(struct acs_procedure *proc, struct net_buf_simple *buf)
{
	struct acs_cp_sec_switch_req switch_req;
	uint8_t switch_state;

	if (buf->len < sizeof(struct acs_cp_sec_switch_req)) {
		LOG_ERR("Set Security Controls Switch operand too short: %u", buf->len);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	if (!proc->acs_conn) {
		LOG_ERR("Set Security Controls Switch for unknown ACS connection");
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	/* Pull operand data before any response buffer init. */
	memcpy(&switch_req, net_buf_simple_pull_mem(buf, sizeof(switch_req)), sizeof(switch_req));
	switch_state = switch_req.switch_state & 0x01;

	if (switch_state) {
		proc->acs_conn->status_flags |= BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	} else {
		proc->acs_conn->status_flags &= ~BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	}

	acs_status_indicate(proc->acs_conn->conn);

	LOG_DBG("Security controls switch set to %u", switch_state);

	return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH,
				 BT_ACS_CP_RESPONSE_SUCCESS);
}

#endif /* CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_URI)

int acs_sec_mgmt_get_key_uri(struct acs_procedure *proc, struct net_buf_simple *buf)
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
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_KEY_URI,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	/* Pull operand data before any response buffer init. */
	memcpy(&key_uri_req, net_buf_simple_pull_mem(buf, sizeof(key_uri_req)),
	       sizeof(key_uri_req));
	key_id = sys_le16_to_cpu(key_uri_req.key_id);
	cb = acs_cb_get();

	if (!cb || !cb->key_uri_get) {
		LOG_WRN("Get Key URI for key_id=0x%04x but no callback is registered", key_id);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_KEY_URI,
					 BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	}

	if (!proc->acs_conn) {
		LOG_ERR("Get Key URI for key_id=0x%04x with no ACS connection context", key_id);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_KEY_URI,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	{
		struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
		struct net_buf *nbuf = acs_prepare_reply_buf(proc, reply_mode.encrypted);

		if (!nbuf) {
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_KEY_URI,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
		net_buf_add_u8(nbuf, BT_ACS_CP_OPCODE_KEY_URI_RESPONSE);
		rsp_buf = &nbuf->b;
	}

	hdr = net_buf_simple_add(rsp_buf, sizeof(struct acs_cp_key_uri_rsp_hdr));
	hdr->key_id = sys_cpu_to_le16(key_id);

	uri_max = MIN(net_buf_simple_tailroom(rsp_buf), (uint16_t)CONFIG_BT_ACS_KEY_URI_MAX_LEN);
	uri_ptr = rsp_buf->data + rsp_buf->len;
	uri_len = 0;

	err = cb->key_uri_get(proc->acs_conn->conn, key_id, uri_ptr, uri_max, &uri_len);

	if (err || uri_len == 0) {
		LOG_DBG("key_uri_get key_id=0x%04x err=%d", key_id, err);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_KEY_URI,
					 BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	}

	net_buf_simple_add(rsp_buf, uri_len);

	LOG_DBG("Key URI response: key_id=0x%04x uri_len=%u", key_id, uri_len);

	arm_err = acs_cp_send_reply(proc);

	if (arm_err) {
		LOG_WRN("Key URI response send failed for key_id=0x%04x: %d", key_id, arm_err);
	}
	return arm_err;
}

#endif /* CONFIG_BT_ACS_KEY_URI */

#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)

int acs_sec_mgmt_initiate_pairing(struct acs_procedure *proc)
{
	int err;

	err = bt_conn_set_security(proc->acs_conn->conn, BT_SECURITY_L2 | BT_SECURITY_FORCE_PAIR);

	if (err) {
		LOG_ERR("Initiate Pairing: bt_conn_set_security failed: %d", err);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_INITIATE_PAIRING,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_INITIATE_PAIRING,
				 BT_ACS_CP_RESPONSE_SUCCESS);
}

#endif /* CONFIG_BT_ACS_INITIATE_PAIRING */
