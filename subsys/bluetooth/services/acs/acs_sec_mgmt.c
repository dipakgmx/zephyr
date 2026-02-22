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
		atomic_set_bit(&reply->conn->state, ACS_STATE_INVALIDATE_PENDING);
	}

	reply->invalidate_key_id = key_id;
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
	    acs_keys_installed(ac, ACS_KEY_ID_ECDH) || acs_kex_in_progress(ac)) {
		LOG_DBG("Invalidating security for conn %p", (void *)conn);
		bt_acs_invalidate_security(conn);
		ctx->count++;
	}
}

uint8_t acs_sec_mgmt_invalidate_all(struct acs_reply *reply, struct net_buf_simple *payload)
{
	ARG_UNUSED(payload);

	struct invalidate_others_ctx ctx = {
		.requester = reply->conn->conn,
		.count = 0,
	};

	bt_conn_foreach(BT_CONN_TYPE_LE, invalidate_other_conn, &ctx);

	acs_keys_forget_all_except(reply->conn);
	LOG_DBG("Invalidated security for %d other connection(s); requester follows response",
		ctx.count);
	remove_key_after_reply(reply, ACS_KEY_ID_ECDH);
	return BT_ACS_CP_RESPONSE_SUCCESS;
}

void acs_sec_mgmt_remove_key(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	const struct bt_acs_cb *cb = acs_cb_get();

	switch (key_id) {
	case ACS_KEY_ID_ECDH:
		(void)bt_acs_invalidate_security(acs_conn->conn);
		break;
	case ACS_KEY_ID_KDF:
		/* The session keys go; the ECDH key stays for a new KDF exchange. */
		acs_keys_remove(acs_conn, ACS_KEY_ID_KDF);
		atomic_clear_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED);
		if (cb != NULL && cb->security_invalidated != NULL) {
			cb->security_invalidated(acs_conn->conn);
		}
		acs_status_schedule(acs_conn->conn);
		break;
	default:
		/* Only the named algorithm key; its parent and siblings stay (§4.4.3.12). */
		acs_keys_remove(acs_conn, key_id);
		break;
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
		remove_key_after_reply(reply, ACS_KEY_ID_ECDH);
		return BT_ACS_CP_RESPONSE_SUCCESS;
	}

	if (acs_key_desc_lookup(key_id) == NULL) {
		LOG_ERR("Invalidate Key received with unknown Key ID 0x%04x", key_id);
		return BT_ACS_CP_RESPONSE_NO_RECORDS_FOUND;
	}

	/* §4.4.3.12: an already-invalid Key_ID is Procedure Not Applicable. */
	if (!acs_keys_installed(reply->conn, key_id)) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	}

	if (key_id == ACS_KEY_ID_ECDH || reply->channel != ACS_REPLY_CP) {
		remove_key_after_reply(reply, key_id);
		LOG_DBG("Key 0x%04x will be removed after the response", key_id);
	} else {
		acs_sec_mgmt_remove_key(reply->conn, key_id);
		LOG_DBG("Key 0x%04x invalidated", key_id);
	}

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
uint8_t acs_cp_handle_set_client_nonce_fixed(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
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
		LOG_WRN("Set client nonce fixed: Key_ID 0x%04x is exchanging or exchanged",
			key_id);
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

	switch_state &= 0x01;

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

	err = bt_conn_set_security(reply->conn->conn, BT_SECURITY_L2 | BT_SECURITY_FORCE_PAIR);

	if (err) {
		LOG_ERR("Initiate Pairing: bt_conn_set_security failed: %d", err);
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

#endif /* CONFIG_BT_ACS_INITIATE_PAIRING */
