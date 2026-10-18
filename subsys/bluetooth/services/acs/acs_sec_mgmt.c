/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_internal.h"
#include "acs_reply.h"
#include "acs_keys.h"
#include "acs_cp_handlers.h"
#include "acs_key_desc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/*
 * Remove key_id once the reply is delivered: a protected response is encrypted
 * with that key or one derived from it. Removing the ECDH key ends all
 * security, so a disconnect before then still completes it.
 */
static void remove_key_after_reply(struct acs_reply *reply, uint16_t key_id)
{
	if (key_id == ACS_KEY_ID_ECDH) {
		atomic_set_bit(&reply->acs_conn->state, ACS_STATE_INVALIDATE_PENDING);
	}

	reply->invalidate_key_id = key_id;
}

/* Whether a security algorithm key other than except_key_id is installed. */
static bool alg_key_installed(struct bt_acs_conn *acs_conn, uint16_t except_key_id)
{
	uint16_t key_ids[ACS_KEY_COUNT];
	uint8_t count = acs_keys_list(acs_conn, key_ids);

	for (uint8_t i = 0U; i < count; i++) {
		if (key_ids[i] != ACS_KEY_ID_ECDH && key_ids[i] != ACS_KEY_ID_KDF &&
		    key_ids[i] != except_key_id) {
			return true;
		}
	}

	return false;
}

/*
 * Whether removing key_id ends the established security: Security Established
 * stays 1 only while a valid key remains (§4.2.2), so it ends with the ECDH or
 * KDF key, or with the last security algorithm key.
 */
static bool removal_ends_security(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	return key_id == ACS_KEY_ID_ECDH || key_id == ACS_KEY_ID_KDF ||
	       !alg_key_installed(acs_conn, key_id);
}

/* Invalidate security on conn unless it is the requester passed in data. */
static void invalidate_other_conn(struct bt_conn *conn, void *data)
{
	struct bt_acs_conn *acs_conn = acs_conn_lookup(conn);

	if (acs_conn == NULL || conn == data) {
		return;
	}

	/* Remove key material from every other connection (§4.4.3.11). */
	if (atomic_test_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED) ||
	    acs_keys_installed(acs_conn, ACS_KEY_ID_ECDH) || acs_kex_in_progress(acs_conn)) {
		LOG_DBG("Invalidating security for conn %p", (void *)conn);
		acs_security_invalidate(acs_conn);
	}
}

uint8_t acs_sec_mgmt_invalidate_all(struct acs_reply *reply, struct net_buf_simple *payload)
{
	ARG_UNUSED(payload);

	bt_conn_foreach(BT_CONN_TYPE_LE, invalidate_other_conn, reply->acs_conn->conn);

	/*
	 * The requester's security ends now, so Status reads 0 once the Success
	 * response is out (§4.4.3.11); its keys go after that response is sent.
	 */
	acs_keys_forget_all_except(reply->acs_conn);
	acs_security_end(reply->acs_conn);
	remove_key_after_reply(reply, ACS_KEY_ID_ECDH);
	return BT_ACS_CP_RESPONSE_SUCCESS;
}

void acs_sec_mgmt_remove_key(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	bool ends_security;

	if (key_id == ACS_KEY_ID_ECDH) {
		acs_security_invalidate(acs_conn);
		return;
	}

	/*
	 * The KDF key takes every algorithm key with it and keeps the ECDH key for a
	 * new KDF exchange; an algorithm key goes alone (§4.4.3.12).
	 */
	ends_security = removal_ends_security(acs_conn, key_id);
	acs_keys_remove(acs_conn, key_id);

	if (ends_security) {
		acs_security_end(acs_conn);
		acs_status_schedule(acs_conn);
	}
}

uint8_t acs_sec_mgmt_invalidate_key(struct acs_reply *reply, struct net_buf_simple *buf)
{
	uint16_t key_id = net_buf_simple_pull_le16(buf);

	/*
	 * 0xFFFF invalidates all keys, whatever is installed (§4.4.3.12). Every key
	 * derives from the ECDH key, so that is removing the ECDH key.
	 */
	if (key_id == BT_ACS_GET_KEY_DESC_ALL_RECORDS_FILTER) {
		acs_security_end(reply->acs_conn);
		remove_key_after_reply(reply, ACS_KEY_ID_ECDH);
		return BT_ACS_CP_RESPONSE_SUCCESS;
	}

	if (acs_key_desc_lookup(key_id) == NULL) {
		LOG_ERR("Invalidate Key received with unknown Key ID 0x%04x", key_id);
		return BT_ACS_CP_RESPONSE_NO_RECORDS_FOUND;
	}

	/* §4.4.3.12: an already-invalid Key_ID is Procedure Not Applicable. */
	if (!acs_keys_installed(reply->acs_conn, key_id)) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	}

	/*
	 * When the removal ends the established security, it ends here, so Status
	 * reads 0 once the Success response is out. A key the response is encrypted
	 * with is removed after that response is sent.
	 */
	if (removal_ends_security(reply->acs_conn, key_id)) {
		acs_security_end(reply->acs_conn);
	}

	if (key_id == ACS_KEY_ID_ECDH || reply->channel != ACS_REPLY_CP) {
		remove_key_after_reply(reply, key_id);
		LOG_DBG("Key 0x%04x will be removed after the response", key_id);
	} else {
		acs_sec_mgmt_remove_key(reply->acs_conn, key_id);
		LOG_DBG("Key 0x%04x invalidated", key_id);
	}

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
uint8_t acs_cp_handle_set_client_nonce_fixed(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->acs_conn;
	const struct bt_acs_key_desc_record *key_desc;
	struct acs_sec_alg *alg;
	uint16_t key_id;
	uint8_t fixed_size;
	int ret;

	key_id = sys_get_le16(buf->data);
	key_desc = acs_key_desc_lookup(key_id);
	if (key_desc == NULL) {
		LOG_WRN("Set client nonce fixed: unknown Key_ID 0x%04x", key_id);
		return BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE;
	}

	/* Every nonce-bearing record uses Sequence Number Different Fixed Parts. */
	if (!acs_key_desc_has_nonce_record(key_desc)) {
		LOG_WRN("Set client nonce fixed: Key_ID 0x%04x does not support SEQ_DIFF_FIXED",
			key_id);
		return BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED;
	}

	/* §4.4.3.18: reject while key exchange is ongoing or already successful. */
	alg = acs_keys_alg(acs_conn, key_id);
	if (acs_kex_in_progress(acs_conn) || acs_sec_alg_ready(alg)) {
		LOG_WRN("Set client nonce fixed: Key_ID 0x%04x is exchanging or exchanged", key_id);
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	}

	fixed_size = acs_key_desc_nonce_fixed_size(key_desc);
	if (buf->len != sizeof(struct acs_cp_set_client_nonce_fixed_req) + fixed_size) {
		LOG_WRN("Set client nonce fixed: size mismatch (got %u, expected %u)", buf->len,
			(unsigned int)(sizeof(struct acs_cp_set_client_nonce_fixed_req) +
				       fixed_size));
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	}

	ret = acs_keys_set_client_nonce_fixed(
		acs_conn, alg, buf->data + sizeof(struct acs_cp_set_client_nonce_fixed_req));
	if (ret == -EEXIST) {
		LOG_WRN("Set client nonce fixed: value collides with existing nonce fixed");
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	} else if (ret) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	LOG_DBG("Stored client nonce fixed for Key_ID 0x%04x (%u bytes)", key_id, fixed_size);
	return BT_ACS_CP_RESPONSE_SUCCESS;
}
#endif /* CONFIG_BT_ACS_HAS_NONCE_FIXED */

#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)

uint8_t acs_sec_mgmt_set_security_switch(struct acs_reply *reply, struct net_buf_simple *buf)
{
	uint8_t switch_state = net_buf_simple_pull_u8(buf);

	ARG_UNUSED(reply);

	if (switch_state & 0xFE) {
		LOG_WRN("Set Security Controls Switch: padding bits non-zero (0x%02x)",
			switch_state);
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	}

	acs_security_switch_set(switch_state != 0);

	LOG_DBG("Security controls switch set to %u", switch_state);

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

#endif /* CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH */

#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)

uint8_t acs_sec_mgmt_initiate_pairing(struct acs_reply *reply, struct net_buf_simple *payload)
{
	ARG_UNUSED(payload);

	int err;

	err = bt_conn_set_security(reply->acs_conn->conn, BT_SECURITY_L2 | BT_SECURITY_FORCE_PAIR);

	if (err) {
		LOG_ERR("Initiate Pairing: bt_conn_set_security failed: %d", err);
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

#endif /* CONFIG_BT_ACS_INITIATE_PAIRING */
