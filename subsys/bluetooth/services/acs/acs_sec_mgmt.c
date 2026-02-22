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
#include "acs_reply.h"
#include "acs_crypto.h"
#include "acs_cp_handlers.h"
#include "acs_key_desc.h"
#include "acs_key_exchange.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#define ACS_ACTIVE_ALGORITHM_KEY_ID_MASK                                                           \
	((IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) ? BIT(ACS_KEY_ID_CCM) : 0U) |          \
	 (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ? BIT(ACS_KEY_ID_GCM) : 0U) |          \
	 (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC) ? BIT(ACS_KEY_ID_CMAC) : 0U) |        \
	 (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) ? BIT(ACS_KEY_ID_GMAC) : 0U))

#define ACS_ACTIVE_KEY_ID_MASK                                                                     \
	(ACS_ACTIVE_ALGORITHM_KEY_ID_MASK | BIT(ACS_KEY_ID_ECDH) | BIT(ACS_KEY_ID_KDF))

static inline bool is_active_algorithm_key_id(uint16_t key_id)
{
	return key_id < BITS_PER_LONG && ((ACS_ACTIVE_ALGORITHM_KEY_ID_MASK & BIT(key_id)) != 0U);
}

static inline bool is_active_key_id(uint16_t key_id)
{
	return key_id < BITS_PER_LONG && ((ACS_ACTIVE_KEY_ID_MASK & BIT(key_id)) != 0U);
}

/* Remove all security after the encrypted response is sent. */
static void remove_security_after_reply(struct acs_reply *reply)
{
	atomic_set_bit(&reply->conn->state, ACS_STATE_INVALIDATE_PENDING);
	reply->invalidate_key_id = 0U;
	reply->step = ACS_REPLY_INVALIDATE;
}

/* Remove the named child key after its protected response is sent. */
static void remove_key_after_reply(struct acs_reply *reply, uint16_t key_id)
{
	reply->invalidate_key_id = key_id;
	reply->step = ACS_REPLY_INVALIDATE;
}

/* State for invalidating every connection except the requester. */
struct invalidate_others_ctx {
	const struct bt_conn *requester;
	int count;
};

static void invalidate_other_conn(struct bt_conn *conn, void *data)
{
	struct invalidate_others_ctx *ctx = data;
	struct bt_acs_conn *ac = acs_conn_lookup(conn);

	if (ac == NULL || conn == ctx->requester) {
		return;
	}

	/* Remove key material from every other connection (§4.4.3.11). */
	if (atomic_test_bit(&ac->state, ACS_STATE_SECURITY_ESTABLISHED) ||
	    acs_key_exchange_installed_key(ac) != NULL || acs_kex_in_progress(ac)) {
		LOG_DBG("Invalidating security for conn %p", (void *)conn);
		bt_acs_invalidate_security(conn);
		ctx->count++;
	}
}

struct acs_cp_result acs_sec_mgmt_invalidate_all(struct acs_reply *reply,
						 struct net_buf_simple *payload)
{
	ARG_UNUSED(payload);

	struct invalidate_others_ctx ctx = {
		.requester = reply->conn->conn,
		.count = 0,
	};

	bt_conn_foreach(BT_CONN_TYPE_LE, invalidate_other_conn, &ctx);

	acs_key_store_clear_all_except(reply->conn->conn);
	LOG_DBG("Invalidated security for %d other connection(s); requester follows response",
		ctx.count);
	remove_security_after_reply(reply);
	return acs_cp_status(BT_ACS_CP_RESPONSE_SUCCESS);
}

static void invalidate_kdf_security(struct bt_acs_conn *acs_conn)
{
	const struct bt_acs_cb *cb = acs_cb_get();

	acs_crypto_destroy_kdf_keys(acs_conn);
	atomic_set_bit_to(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED, false);
	acs_key_store(acs_conn);

	if (cb != NULL && cb->security_invalidated != NULL) {
		cb->security_invalidated(acs_conn->conn);
	}

	acs_status_schedule(acs_conn->conn);
}

/* Remove only the named algorithm key; keep its parent and siblings (§4.4.3.12). */
static void invalidate_algorithm_key(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	struct bt_acs_key_desc_runtime *alg_key;

	if (acs_crypto_key_runtime_lookup(acs_conn, key_id, &alg_key) != 0) {
		LOG_ERR("Missing runtime key state for algorithm Key_ID 0x%04x", key_id);
		return;
	}

	acs_crypto_destroy_key(alg_key);
}

void acs_sec_mgmt_remove_child_key(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	if (key_id == ACS_KEY_ID_KDF) {
		invalidate_kdf_security(acs_conn);
	} else {
		invalidate_algorithm_key(acs_conn, key_id);
	}
}

struct acs_cp_result acs_sec_mgmt_invalidate_key(struct acs_reply *reply,
						 struct net_buf_simple *buf)
{
	struct acs_cp_invalidate_key_req invalidate_req;
	uint16_t key_id;
	uint8_t response_code;
	int ret;

	/* Copy the packed operand out of the request buffer for aligned access. */
	memcpy(&invalidate_req, net_buf_simple_pull_mem(buf, sizeof(invalidate_req)),
	       sizeof(invalidate_req));
	key_id = sys_le16_to_cpu(invalidate_req.key_id);

	if (key_id == BT_ACS_GET_KEY_DESC_ALL_RECORDS_FILTER) {
		if (acs_key_exchange_installed_key(reply->conn) == NULL) {
			return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		}

		remove_security_after_reply(reply);
		response_code = BT_ACS_CP_RESPONSE_SUCCESS;
	} else if (key_id == ACS_KEY_ID_KDF || is_active_algorithm_key_id(key_id)) {
		/* Keep the exchanged parent when removing a child key (§4.4.3.12). */
		struct bt_acs_key_desc_runtime *key;

		ret = acs_crypto_key_runtime_lookup(reply->conn, key_id, &key);
		if (ret) {
			LOG_ERR("Missing runtime key state for Key_ID 0x%04x", key_id);
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
		} else if (key->psa_key_id == 0U) {
			/* §4.4.3.12: an already-invalid Key_ID is Procedure Not Applicable. */
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
		} else if (reply->channel != ACS_REPLY_CP) {
			/* The DOI reply is encrypted with this key or a child of it. */
			remove_key_after_reply(reply, key_id);
			response_code = BT_ACS_CP_RESPONSE_SUCCESS;
			LOG_DBG("Key 0x%04x will be removed after the protected response", key_id);
		} else {
			acs_sec_mgmt_remove_child_key(reply->conn, key_id);
			response_code = BT_ACS_CP_RESPONSE_SUCCESS;
			LOG_DBG("Key 0x%04x invalidated (exchanged parent retained)", key_id);
		}
	} else if (is_active_key_id(key_id)) {
		struct bt_acs_key_desc_runtime *current_key;

		if (acs_crypto_key_runtime_lookup(reply->conn, key_id, &current_key) != 0 ||
		    current_key->psa_key_id == 0U) {
			response_code = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
		} else {
			remove_security_after_reply(reply);
			response_code = BT_ACS_CP_RESPONSE_SUCCESS;
			LOG_DBG("Key ID 0x%04x will be removed after the response", key_id);
		}
	} else {
		LOG_ERR("Invalidate Key received with unknown Key ID 0x%04x", key_id);
		response_code = BT_ACS_CP_RESPONSE_NO_RECORDS_FOUND;
	}

	return acs_cp_status(response_code);
}

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
struct acs_cp_result acs_cp_handle_set_client_nonce_fixed(struct acs_reply *reply,
							  struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	const struct bt_acs_key_desc_record *key_desc;
	struct bt_acs_key_desc_runtime *runtime;
	uint8_t candidate[ACS_MAX_NONCE_PREFIX_SIZE];
	uint16_t key_id;
	uint8_t fixed_size;
	int ret;

	if (acs_conn == NULL) {
		LOG_ERR("Request to set client nonce fixed received for unknown connection");
		return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	key_id = sys_get_le16(buf->data);
	key_desc = acs_key_desc_lookup(key_id);
	if (key_desc == NULL) {
		LOG_WRN("Set client nonce fixed: unknown Key_ID 0x%04x", key_id);
		return acs_cp_status(BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	}

	if (!acs_key_desc_has_nonce_record(key_desc) ||
	    key_desc->aes.nonce_type != ACS_NONCE_SEQ_DIFF_FIXED) {
		LOG_WRN("Set client nonce fixed: Key_ID 0x%04x does not support SEQ_DIFF_FIXED",
			key_id);
		return acs_cp_status(BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	/* §4.4.3.18: reject while key exchange is ongoing or already successful. */
	if (acs_kex_in_progress(acs_conn)) {
		LOG_WRN("Set client nonce fixed: key exchange active");
		return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	if (acs_crypto_key_runtime_lookup(acs_conn, key_id, &runtime) != 0) {
		return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	if (runtime->psa_key_id != 0U) {
		LOG_WRN("Set client nonce fixed: Key_ID 0x%04x already has an active key", key_id);
		return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	fixed_size = acs_key_desc_nonce_fixed_size(key_desc);
	if (buf->len != sizeof(struct acs_cp_set_client_nonce_fixed_req) + fixed_size) {
		LOG_WRN("Set client nonce fixed: size mismatch (got %u, expected %u)", buf->len,
			(unsigned int)(sizeof(struct acs_cp_set_client_nonce_fixed_req) +
				       fixed_size));
		return acs_cp_status(BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	ret = acs_server_nonce_fixed_ensure(runtime);
	if (ret) {
		LOG_ERR("Set client nonce fixed: server nonce generation failed: %d", ret);
		return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	memcpy(candidate, buf->data + sizeof(struct acs_cp_set_client_nonce_fixed_req), fixed_size);
	sys_mem_swap(candidate, fixed_size);

	ret = acs_client_nonce_fixed_check_unique(acs_conn, runtime, candidate, fixed_size);
	if (ret) {
		if (ret == -EEXIST) {
			LOG_WRN("Set client nonce fixed: value collides with existing nonce fixed");
			return acs_cp_status(BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		}
		return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	memcpy(runtime->client_nonce_fixed, candidate, fixed_size);
	runtime->client_nonce_set = true;
	LOG_DBG("Stored client nonce fixed for Key_ID 0x%04x (%u bytes)", key_id, fixed_size);

	return acs_cp_status(BT_ACS_CP_RESPONSE_SUCCESS);
}
#endif /* CONFIG_BT_ACS_HAS_NONCE_FIXED */

#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)

struct acs_cp_result acs_sec_mgmt_set_security_switch(struct acs_reply *reply,
						      struct net_buf_simple *buf)
{
	struct acs_cp_sec_switch_req switch_req;
	uint8_t switch_state;

	/* Copy the packed operand out of the request buffer for aligned access. */
	memcpy(&switch_req, net_buf_simple_pull_mem(buf, sizeof(switch_req)), sizeof(switch_req));

	if (switch_req.switch_state & 0xFE) {
		LOG_WRN("Set Security Controls Switch: padding bits non-zero (0x%02x)",
			switch_req.switch_state);
		return acs_cp_status(BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	switch_state = switch_req.switch_state & 0x01;

	acs_security_switch_set(switch_state != 0);

	LOG_DBG("Security controls switch set to %u", switch_state);

	return acs_cp_status(BT_ACS_CP_RESPONSE_SUCCESS);
}

#endif /* CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH */

#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)

struct acs_cp_result acs_sec_mgmt_initiate_pairing(struct acs_reply *reply,
						   struct net_buf_simple *payload)
{
	ARG_UNUSED(payload);

	int err;

	err = bt_conn_set_security(reply->conn->conn, BT_SECURITY_L2 | BT_SECURITY_FORCE_PAIR);

	if (err) {
		LOG_ERR("Initiate Pairing: bt_conn_set_security failed: %d", err);
		return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	return acs_cp_status(BT_ACS_CP_RESPONSE_SUCCESS);
}

#endif /* CONFIG_BT_ACS_INITIATE_PAIRING */
