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

int acs_sec_mgmt_invalidate_all(struct acs_reply *reply)
{
	struct bt_acs_conn const *acs_conn = reply->conn;
	uint8_t req_idx;
	int count = 0;

	if (!acs_conn || !acs_session_established(acs_conn)) {
		LOG_WRN("Invalidate All: rejected - sender has no established ACS security");
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	}

	req_idx = bt_conn_index(reply->conn->conn);

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

	acs_session_cache_clear_all_except(bt_conn_get_dst(reply->conn->conn));

#if defined(CONFIG_BT_SETTINGS)
	acs_session_clear_all_except(reply->conn->conn);
#endif /* CONFIG_BT_SETTINGS */

	LOG_DBG("Invalidated security for %d other connection(s); deferring self", count);

	/* Defer self-invalidation until after the success response is confirmed. */
	reply->step = ACS_REPLY_INVALIDATE;

	return BT_ACS_CP_RESPONSE_SUCCESS;
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
	struct bt_acs_key_desc_runtime *kdf_key;

	if (acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_KDF, &kdf_key) != 0) {
		LOG_ERR("Missing KDF runtime key state");
		return;
	}

	acs_crypto_destroy_key(kdf_key);
	acs_crypto_invalidate_algorithm_keys(acs_conn);
	acs_conn->status_flags &= ~BT_ACS_STATUS_SECURITY_ESTABLISHED;
}
#endif

int acs_sec_mgmt_invalidate_key(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct acs_cp_invalidate_key_req invalidate_req;
	uint16_t key_id;
	uint8_t response_code;
	int ret;

	if (buf->len < sizeof(struct acs_cp_invalidate_key_req)) {
		LOG_ERR("Invalid Invalidate Key operand length: need %zu, have %u",
			sizeof(struct acs_cp_invalidate_key_req), buf->len);
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	}

	/* Copy the packed operand out of the request buffer for aligned access. */
	memcpy(&invalidate_req, net_buf_simple_pull_mem(buf, sizeof(invalidate_req)),
	       sizeof(invalidate_req));
	key_id = sys_le16_to_cpu(invalidate_req.key_id);

	if (!reply->conn) {
		LOG_ERR("Invalidate Key received for unknown connection");
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	if (key_id == BT_ACS_GET_KEY_DESC_ALL_RECORDS_FILTER) {
		if (acs_key_exchange_established_key(reply->conn) == NULL) {
			return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
		}

		bt_acs_invalidate_security(reply->conn->conn);
		response_code = BT_ACS_CP_RESPONSE_SUCCESS;
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	} else if (key_id == ACS_KEY_ID_KDF) {
		struct bt_acs_key_desc_runtime *kdf_key;

		ret = acs_crypto_key_runtime_lookup(reply->conn, ACS_KEY_ID_KDF, &kdf_key);
		if (ret) {
			LOG_ERR("Missing KDF runtime key state");
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
		} else if (kdf_key->psa_key_id == 0U) {
			/* Spec §4.4.3.12: "key ID that is already invalid → Procedure Not
			 * Applicable" */
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
		} else {
			invalidate_kdf_child(reply->conn);
#if defined(CONFIG_BT_SETTINGS)
			acs_session_store(reply->conn->conn, reply->conn);
#endif
			{
				const struct bt_acs_cb *cb = acs_cb_get();

				if (cb && cb->security_invalidated) {
					cb->security_invalidated(reply->conn->conn);
				}
			}
			acs_status_indicate(reply->conn->conn);
			response_code = BT_ACS_CP_RESPONSE_SUCCESS;
			LOG_DBG("KDF child key invalidated");
		}
	} else if (is_active_algorithm_key_id(key_id)) {
		struct bt_acs_key_desc_runtime *kdf_key;

		ret = acs_crypto_key_runtime_lookup(reply->conn, ACS_KEY_ID_KDF, &kdf_key);
		if (ret == 0 && kdf_key->psa_key_id != 0U) {
			/* Algorithm keys (GCM, CCM, …) use the KDF child as their actual key
			 * material.  Invalidating them is equivalent to invalidating the child. */
			invalidate_kdf_child(reply->conn);
#if defined(CONFIG_BT_SETTINGS)
			acs_session_store(reply->conn->conn, reply->conn);
#endif
			{
				const struct bt_acs_cb *cb = acs_cb_get();

				if (cb && cb->security_invalidated) {
					cb->security_invalidated(reply->conn->conn);
				}
			}
			acs_status_indicate(reply->conn->conn);
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
		struct bt_acs_key_desc_runtime *current_key;

		if (acs_crypto_key_runtime_lookup(reply->conn, key_id, &current_key) != 0 ||
		    current_key->psa_key_id == 0U) {
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
		} else {
			ret = bt_acs_invalidate_security(reply->conn->conn);
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

	return response_code;
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
 *   1. Probe   - read-only scan of what would need to be torn down.
 *   2. Decide  - if nothing to abort, or if a response is already mid-flight
 *                (cannot be unsent), return UNSUCCESSFUL without mutating state.
 *   3. Commit  - only after both checks pass, tear down sequences, KEX, and
 *                data-op state, then send the ABORT SUCCESS response.
 *
 * ABORT bypasses the cp_locked gate in acs_cp_write() (ABORT must be
 * able to preempt an in-progress procedure), so this handler also owns the
 * lock bookkeeping for its own response.
 */
int acs_sec_mgmt_abort(struct acs_reply *reply)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	bool plain_cp_active;
	bool kex_in_progress;
	bool has_work;
	bool can_commit;

	plain_cp_active = (atomic_get(&acs_conn->cp_locked) == 1);
	kex_in_progress = acs_kex_in_progress(acs_conn);
	has_work = plain_cp_active || kex_in_progress;

	if (!has_work) {
		LOG_WRN("Abort requested with no in-progress procedure");
		if (reply->channel == ACS_REPLY_CP) {
			atomic_set(&acs_conn->cp_locked, 1);
		}
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	}

	can_commit = !acs_conn->cp_tx.tx_in_flight;

	if (!can_commit) {
		if (reply->channel != ACS_REPLY_CP) {
			return BT_ACS_CP_RESPONSE_ABORT_UNSUCCESSFUL;
		}
		LOG_DBG("Abort deferred - indication in flight, will commit on confirm");
		acs_conn->cp_abort_pending = true;
		return ACS_CP_RESULT_NO_REPLY;
	}

	acs_abort_commit(acs_conn);

	if (reply->channel == ACS_REPLY_CP) {
		atomic_set(&acs_conn->cp_locked, 1);
	}

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

#endif /* CONFIG_BT_ACS_ABORT */

#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)

int acs_sec_mgmt_set_security_switch(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct acs_cp_sec_switch_req switch_req;
	uint8_t switch_state;

	if (buf->len < sizeof(struct acs_cp_sec_switch_req)) {
		LOG_ERR("Set Security Controls Switch operand too short: %u", buf->len);
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	}

	if (!reply->conn) {
		LOG_ERR("Set Security Controls Switch for unknown ACS connection");
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	/* Copy the packed operand out of the request buffer for aligned access. */
	memcpy(&switch_req, net_buf_simple_pull_mem(buf, sizeof(switch_req)), sizeof(switch_req));

	if (switch_req.switch_state & 0xFE) {
		LOG_WRN("Set Security Controls Switch: padding bits non-zero (0x%02x)",
			switch_req.switch_state);
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	}

	switch_state = switch_req.switch_state & 0x01;

	acs_security_switch_set(switch_state != 0);

	LOG_DBG("Security controls switch set to %u", switch_state);

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

#endif /* CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_URI)

int acs_sec_mgmt_get_key_uri(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct acs_cp_get_key_uri_req key_uri_req;
	struct acs_cp_key_uri_rsp_hdr *hdr;
	const struct bt_acs_cb *cb;
	struct net_buf_simple *rsp_buf;
	uint8_t *uri_ptr;
	uint16_t uri_max;
	uint16_t uri_len;
	uint16_t key_id;
	int err;

	if (buf->len < sizeof(struct acs_cp_get_key_uri_req)) {
		LOG_ERR("Get Key URI operand too short: %u", buf->len);
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	}

	/* Copy the packed operand out of the request buffer for aligned access. */
	memcpy(&key_uri_req, net_buf_simple_pull_mem(buf, sizeof(key_uri_req)),
	       sizeof(key_uri_req));
	key_id = sys_le16_to_cpu(key_uri_req.key_id);
	cb = acs_cb_get();

	if (!cb || !cb->key_uri_get) {
		LOG_WRN("Get Key URI for key_id=0x%04x but no callback is registered", key_id);
		return BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE;
	}

	if (!reply->conn) {
		LOG_ERR("Get Key URI for key_id=0x%04x with no ACS connection context", key_id);
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	rsp_buf = &reply->response->b;

	hdr = net_buf_simple_add(rsp_buf, sizeof(struct acs_cp_key_uri_rsp_hdr));
	hdr->key_id = sys_cpu_to_le16(key_id);

	uri_max = MIN(net_buf_simple_tailroom(rsp_buf), (uint16_t)CONFIG_BT_ACS_KEY_URI_MAX_LEN);
	uri_ptr = rsp_buf->data + rsp_buf->len;
	uri_len = 0;

	err = cb->key_uri_get(reply->conn->conn, key_id, uri_ptr, uri_max, &uri_len);

	if (err || uri_len == 0) {
		LOG_DBG("key_uri_get key_id=0x%04x err=%d", key_id, err);
		return BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE;
	}

	net_buf_simple_add(rsp_buf, uri_len);

	LOG_DBG("Key URI response: key_id=0x%04x uri_len=%u", key_id, uri_len);

	return ACS_CP_RESULT_STAGED_REPLY;
}

#endif /* CONFIG_BT_ACS_KEY_URI */

#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)

int acs_sec_mgmt_initiate_pairing(struct acs_reply *reply)
{
	int err;

	err = bt_conn_set_security(reply->conn->conn, BT_SECURITY_L2 | BT_SECURITY_FORCE_PAIR);

	if (err) {
		LOG_ERR("Initiate Pairing: bt_conn_set_security failed: %d", err);
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

#endif /* CONFIG_BT_ACS_INITIATE_PAIRING */
