/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/random/random.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_cp.h"
#include "acs_internal.h"
#include "acs_key_desc.h"
#include "acs_key_exchange.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static struct bt_acs_runtime_key_state *acs_cp_kex_established_key(struct bt_acs_conn *acs_conn)
{
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	struct bt_acs_runtime_key_state *kdf_key;

	if (acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_KDF, &kdf_key) == 0 &&
	    kdf_key->psa_key_id != 0U) {
		return kdf_key;
	}
#endif

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	{
		struct bt_acs_runtime_key_state *ecdh_key;

		if (acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_ECDH, &ecdh_key) == 0 &&
		    ecdh_key->psa_key_id != 0U) {
			return ecdh_key;
		}
	}
#endif

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	{
		struct bt_acs_runtime_key_state *oob_key;

		if (acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_OOB, &oob_key) == 0 &&
		    oob_key->psa_key_id != 0U) {
			return oob_key;
		}
	}
#endif

	return NULL;
}

int acs_cp_kex_get_current_key_list(struct acs_procedure *proc)
{
	/* count(1 byte) + up to ACS_KEY_ID_COUNT x Key_ID(2 bytes) */
	uint8_t buf[sizeof(uint8_t) + ACS_KEY_ID_COUNT * sizeof(uint16_t)];
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	uint8_t count = 0;
	uint16_t pos = sizeof(uint8_t); /* byte 0 reserved for count */

	if (proc->acs_conn->crypto.key_state == BT_ACS_KEY_EXCHANGE_COMPLETE) {
		for (size_t i = 0; i < ARRAY_SIZE(proc->acs_conn->crypto.current_keys); i++) {
			const struct bt_acs_runtime_key_state *current_key =
				&proc->acs_conn->crypto.current_keys[i];

			if (current_key->psa_key_id == 0U) {
				continue;
			}

			sys_put_le16(acs_runtime_key_id(current_key), &buf[pos]);
			pos += sizeof(uint16_t);
			count++;
		}
	}

	buf[0] = count;

	{
		struct net_buf *rsp_buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);

		if (!rsp_buf) {
			LOG_WRN("buffer pool exhausted");
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
		net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_CURRENT_KEY_LIST_RESPONSE);
		net_buf_add_mem(rsp_buf, buf, pos);
		return acs_cp_send_reply(proc);
	}
}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH) || IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)

/* --- Key exchange success sequence steps --- */

static int kex_step_success_response(struct acs_procedure *proc)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	uint8_t payload[3];
	uint16_t key_id;

	if (!acs_conn || !acs_conn->crypto.kex) {
		return -EINVAL;
	}

	key_id = sys_le16_to_cpu(acs_conn->crypto.kex->start_kex.key_id);
	acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_PENDING_STATUS;
	sys_put_le16(key_id, &payload[0]);
	payload[2] = 0x00;

	struct net_buf *buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);

	if (!buf) {
		return -ENOMEM;
	}
	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_RESPONSE);
	net_buf_add_mem(buf, payload, sizeof(payload));
	return acs_cp_send_reply(proc);
}

static int kex_step_status(struct acs_procedure *proc)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;

	acs_kex_free(acs_conn->crypto.kex);
	acs_conn->crypto.kex = NULL;
	acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_COMPLETE;

	/* The KEY_EXCHANGE_RESPONSE indication has been confirmed by the peer, so
	 * both sides agree the exchange succeeded.  Set the flag and notify the
	 * application now — doing it earlier (in the handler that built the response)
	 * would race against a connection drop before the peer acknowledged it. */
	acs_conn->status_flags |= BT_ACS_STATUS_SECURITY_ESTABLISHED;
	{
		const struct bt_acs_cb *cb = acs_cb_get();
		struct bt_acs_runtime_key_state *established_key =
			acs_cp_kex_established_key(acs_conn);

		if (cb && cb->security_established && established_key) {
			cb->security_established(proc->acs_conn->conn, established_key->key,
						 CONFIG_BT_ACS_SESSION_KEY_SIZE);
		}
	}

#if defined(CONFIG_BT_SETTINGS)
/* Persist only after KEY_EXCHANGE_RESPONSE has been confirmed: storing
 * earlier would leave NVS with the new key if the connection drops
 * before the response is delivered — the remote would discard the
 * incomplete exchange and still expect the old key on reconnect.
 *
 * In session mode (CONFIG_BT_ACS_KDF_SESSION_KEY), the KDF child key
 * intentionally lives only for this connection.  The ECDH parent key
 * was already stored when the ECDH exchange completed (with zero nonce
 * counters, since the parent is never used for AEAD directly).  Storing
 * again here would needlessly overwrite it with a child key that will be
 * discarded at disconnect anyway.  Skip the store; the peer must redo
 * the KDF exchange on every reconnect.
 *
 * In persistent mode (default), store unconditionally: acs_session_store
 * handles the two-key layout, writing the exchanged parent key
 * and the child key with its nonce counters into separate fields so both
 * can be restored on reconnect without repeating either exchange. */
#if IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
	{
		struct bt_acs_runtime_key_state *kdf_key;

		if (acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_KDF, &kdf_key) != 0 ||
		    kdf_key->psa_key_id == 0U) {
			acs_session_store(proc->acs_conn->conn, acs_conn);
		}
	}
#else
	acs_session_store(proc->acs_conn->conn, acs_conn);
#endif
#endif

	acs_seq_clear(proc);
	acs_status_indicate(acs_conn->conn);
	return 0;
}

static void kex_on_abort(struct acs_procedure *proc)
{
	struct bt_acs_conn *acs_conn = proc ? proc->acs_conn : NULL;

	if (!acs_conn || !acs_conn->crypto.kex) {
		return;
	}

	acs_kex_free(acs_conn->crypto.kex);
	acs_conn->crypto.kex = NULL;
	if (acs_conn->crypto.key_state == BT_ACS_KEY_EXCHANGE_PENDING_RESPONSE ||
	    acs_conn->crypto.key_state == BT_ACS_KEY_EXCHANGE_PENDING_STATUS) {
		acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
	}
}

static const acs_seq_step_fn kex_success_steps[] = {
	kex_step_success_response,
	kex_step_status,
};

static const struct acs_seq_desc kex_success_seq = {
	.steps = kex_success_steps,
	.step_count = ARRAY_SIZE(kex_success_steps),
	.on_abort = kex_on_abort,
};

/* --- Key exchange failure sequence steps --- */

static int kex_step_fail_response(struct acs_procedure *proc)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	uint8_t payload[3];
	uint16_t key_id;

	if (!acs_conn || !acs_conn->crypto.kex) {
		return -EINVAL;
	}

	key_id = sys_le16_to_cpu(acs_conn->crypto.kex->start_kex.key_id);
	sys_put_le16(key_id, &payload[0]);
	payload[2] = 0x01;

	struct net_buf *buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);

	if (!buf) {
		return -ENOMEM;
	}
	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_RESPONSE);
	net_buf_add_mem(buf, payload, sizeof(payload));
	return acs_cp_send_reply(proc);
}

static int kex_step_fail_cleanup(struct acs_procedure *proc)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;

	acs_kex_free(acs_conn->crypto.kex);
	acs_conn->crypto.kex = NULL;
	acs_seq_clear(proc);
	return 0;
}

static const acs_seq_step_fn kex_fail_steps[] = {
	kex_step_fail_response,
	kex_step_fail_cleanup,
};

static const struct acs_seq_desc kex_fail_seq = {
	.steps = kex_fail_steps,
	.step_count = ARRAY_SIZE(kex_fail_steps),
	.on_abort = kex_on_abort,
};

int acs_cp_kex_exchange_kdf(struct acs_procedure *proc, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;
	struct acs_kdf_req req_data;
	uint16_t key_id;
	struct net_buf *rsp_buf;
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	int err;
	int arm_err;

	if (buf->len != sizeof(struct acs_kdf_req)) {
		LOG_WRN("Key exchange KDF operand invalid length: %u", buf->len);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	/* Pull all operand data before response buffer init to avoid aliasing. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));
	key_id = sys_le16_to_cpu(req_data.key_id);

	/* §4.4.3.17.2.1: Key Exchange KDF is only valid after Start Key Exchange
	 * for the same Key_ID. Reject if no KEX context exists or the Key_ID differs.
	 */

	if (!acs_conn->crypto.kex ||
	    key_id != sys_le16_to_cpu(acs_conn->crypto.kex->start_kex.key_id)) {
		LOG_WRN("Key exchange KDF operand invalid Key_ID: 0x%04x, expected 0x%04x", key_id,
			acs_conn->crypto.kex
				? sys_le16_to_cpu(acs_conn->crypto.kex->start_kex.key_id)
				: 0);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	rsp_buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);
	if (!rsp_buf) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF_RESPONSE);

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	if (key_id == ACS_KEY_ID_KDF) {
		err = acs_key_exchange_kdf(acs_conn, &rsp_buf->b);

		/* Standalone KDF is only applicable once the higher-level parent key is
		 * established and the KEX state machine has been advanced by Start Key Exchange.
		 */
		if (err == -EAGAIN) {
			LOG_WRN("KDF key exchange: wrong state (state %d)",
				acs_conn->crypto.key_state);
			acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(proc, &kex_fail_seq);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		} else if (err) {
			LOG_ERR("KDF key exchange: internal error (err %d)", err);
			acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(proc, &kex_fail_seq);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}

		/* Nonce counters are reset inside bt_acs_crypto_derive_kdf_child_key()
		 * with the correct SEQ_EVEN_ODD logic — no separate reset needed here.
		 *
		 * SECURITY_ESTABLISHED flag and the security_established callback are
		 * deferred to kex_step_status, which runs after the KEY_EXCHANGE_RESPONSE
		 * indication is confirmed by the peer.  Setting them here would tell the
		 * application the session is live before the peer has acknowledged it. */
		acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_PENDING_RESPONSE;

		/* KDF_RESPONSE must be delivered before KEY_EXCHANGE_RESPONSE (§4.4.3.10). */
		acs_seq_begin(proc, &kex_success_seq);
		arm_err = acs_cp_send_reply(proc);

		if (arm_err) {
			LOG_WRN("KDF standalone response indication arm failed: %d", arm_err);
			acs_seq_abort(proc);
		}
		return arm_err;
	}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	/* ECDH KDF path: KEY_EXCHANGE_RESPONSE follows after Confirmation Random Number. */
	/* In the ECDH flow, this KDF step is only valid after the peer public key has
	 * been processed and the shared secret is available for derivation.
	 */
	if (acs_conn->crypto.key_state != BT_ACS_KEY_EXCHANGE_PUBKEY_EXCHANGED) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	err = acs_key_exchange_ecdh_kdf(acs_conn, &rsp_buf->b);

	if (err == -EAGAIN) {
		LOG_WRN("ECDH KDF: wrong state (state %d)", acs_conn->crypto.key_state);
		acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
		acs_seq_begin(proc, &kex_fail_seq);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	} else if (err) {
		LOG_ERR("ECDH KDF: internal error (err %d)", err);
		acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
		acs_seq_begin(proc, &kex_fail_seq);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	arm_err = acs_cp_send_reply(proc);
	if (arm_err) {
		LOG_WRN("ECDH KDF response arm failed: %d", arm_err);
	}
	return arm_err;
#else
	return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
				 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */
}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF || CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

int acs_cp_kex_start(struct acs_procedure *proc, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;
	struct acs_cp_start_key_exchange_req req_data;
	uint16_t key_id;
	uint8_t method;
	uint8_t action;
	bool method_action_valid;
	const struct bt_acs_cb *cb;
	int err;
	uint32_t oob_num;
	uint16_t oob_len;
	uint8_t oob_buf[ACS_HMAC_SHA256_SIZE];
	int oob_err;

#if defined(CONFIG_BT_SETTINGS)
	/* Decline if session cache is full and this peer has no existing slot. */
	if (!acs_session_cache_has_room(bt_conn_get_dst(proc->acs_conn->conn))) {
		LOG_WRN("start_kex: session cache full");
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
#endif /* CONFIG_BT_SETTINGS */

	if (buf->len < sizeof(struct acs_cp_start_key_exchange_req)) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	/* Pull all operand data before any response buffer init. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));
	key_id = sys_le16_to_cpu(req_data.key_id);
	method = req_data.confirmation_method;
	action = req_data.confirmation_action;
	method_action_valid = true;
	cb = acs_cb_get();

	/* Validate the selected method/action against Tables 4.50–4.52 and against
	 * what this server advertises in the ACS Feature Response.  Spec §4.4.4.18.2:
	 * "the Selected_Confirmation_Method field shall be set to an AC Server-supported
	 * confirmation method."  Unsupported methods/actions are rejected as Invalid
	 * Operand so the client learns to pick a different value. */
	switch (method) {
	case BT_ACS_CONFIRM_METHOD_NONE:
		/* Always supported (no advertised capability needed).  Spec §4.4.4.18.3:
		 * action MUST be 0xFF when method is None. */
		if (action != BT_ACS_CONFIRM_ACTION_NOT_APPLICABLE) {
			method_action_valid = false;
		}
		break;

	case BT_ACS_CONFIRM_METHOD_OUTPUT_OOB:
		/* Table 4.51: valid actions are Beep (0x01) and Output Numeric (0x03).
		 * Each action must be individually advertised in the Feature Response's
		 * Confirmation_Output_OOB_Number_Capabilities field. */
		if (action == BT_ACS_CONFIRM_ACTION_OUTPUT_BEEP) {
			method_action_valid = IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_BEEP);
		} else if (action == BT_ACS_CONFIRM_ACTION_OUTPUT_NUMERIC) {
			method_action_valid = IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC);
		} else {
			method_action_valid = false;
		}
		break;

	case BT_ACS_CONFIRM_METHOD_INPUT_OOB:
		/* Table 4.52: valid actions are Push (0x00) and Input Numeric (0x02).
		 * Each action must be individually advertised in the Feature Response's
		 * Confirmation_Input_OOB_Number_Capabilities field. */
		if (action == BT_ACS_CONFIRM_ACTION_INPUT_PUSH) {
			method_action_valid = IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_PUSH);
		} else if (action == BT_ACS_CONFIRM_ACTION_INPUT_NUMERIC) {
			method_action_valid = IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_NUMERIC);
		} else {
			method_action_valid = false;
		}
		break;

	case BT_ACS_CONFIRM_METHOD_STATIC_OOB:
		/* Spec §4.4.4.18.3 (after Table 4.52): Static OOB action MUST be 0xFF.
		 * The method itself is only supported when the server advertises at least
		 * one bit in Confirmation_Static_OOB_Number_Capabilities — otherwise the
		 * client has no way to know the static number and the exchange would fail. */
		if (action != BT_ACS_CONFIRM_ACTION_NOT_APPLICABLE) {
			method_action_valid = false;
		} else if (!IS_ENABLED(CONFIG_BT_ACS_OOB_STATIC_NUM_NUMBER) &&
			   !IS_ENABLED(CONFIG_BT_ACS_OOB_STATIC_NUM_ON_DEVICE)) {
			method_action_valid = false;
		}
		break;

	default:
		/* Table 4.50: values 0x04–0xFF are RFU. */
		method_action_valid = false;
		break;
	}

	if (!method_action_valid) {
		LOG_WRN("Start Key Exchange: unsupported method/action (method=0x%02x "
			"action=0x%02x)",
			method, action);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	err = 0;

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	if (key_id == ACS_KEY_ID_KDF) {
		struct bt_acs_runtime_key_state *kdf_key;

		/* KDF standalone: requires parent key and method=None. */
		if (method != BT_ACS_CONFIRM_METHOD_NONE ||
		    action != BT_ACS_CONFIRM_ACTION_NOT_APPLICABLE) {
			LOG_WRN("invalid confirmation method/action (method=0x%02x action=0x%02x)",
				method, action);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
						 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		}
		if (acs_conn->crypto.key_state != BT_ACS_KEY_EXCHANGE_COMPLETE) {
			LOG_WRN("no parent key available (key_state=%d, prior ECDH exchange "
				"required)",
				acs_conn->crypto.key_state);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		}
		/* A KDF child key derived from the ECDH parent is already installed.
		 * The peer must invalidate it before deriving a new one; silently
		 * re-deriving would orphan the old child's nonce counters and create
		 * ambiguity about which key is current in NVS. */
		if (acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_KDF, &kdf_key) != 0) {
			LOG_ERR("Missing KDF runtime key state");
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
		if (kdf_key->psa_key_id != 0U) {
			LOG_WRN("KDF child key already active — invalidate before re-exchange");
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		}
		acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_STARTED;
	} else
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF */
	{
		err = acs_key_exchange_ecdh_start(acs_conn, key_id);
	}

	if (err != 0) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					 errno_to_acs_status(err));
	}

	if (!acs_conn->crypto.kex) {
		if (acs_kex_alloc(acs_conn) != 0) {
			LOG_ERR("No free KEX context");
			acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
	}

	/* Clear auth_value before populating it in the switch cases below */
	memset(acs_conn->crypto.kex->auth_value, 0, sizeof(acs_conn->crypto.kex->auth_value));

	switch (method) {
	case BT_ACS_CONFIRM_METHOD_OUTPUT_OOB:
		/* Generate random OOB number; store right-aligned big-endian in auth_value. */
		sys_rand_get(&oob_num, sizeof(oob_num));
#if IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC)
		oob_num = (oob_num % CONFIG_BT_ACS_CONFIRMATION_OUTPUT_MAX_VALUE) + 1;
#else
		oob_num = (oob_num % 9) + 1;
#endif
		sys_put_be32(
			oob_num,
			&acs_conn->crypto.kex->auth_value[ACS_HMAC_SHA256_SIZE - sizeof(oob_num)]);
		if (cb && cb->output_oob_number) {
			cb->output_oob_number(proc->acs_conn->conn, action, oob_num);
		}
		break;
	case BT_ACS_CONFIRM_METHOD_INPUT_OOB:
		if (cb && cb->input_oob_request) {
			cb->input_oob_request(proc->acs_conn->conn, action);
		}
		break;
	case BT_ACS_CONFIRM_METHOD_STATIC_OOB:
		if (cb && cb->static_oob_get) {
			oob_len = 0;
			oob_err = cb->static_oob_get(proc->acs_conn->conn, oob_buf, &oob_len);
			if (oob_err || oob_len == 0 || oob_len > ACS_HMAC_SHA256_SIZE) {
				LOG_WRN("start_key_exchange: static_oob_get failed: %d", oob_err);
				acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
				return acs_cp_rsp_status(
					proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
			}
			/* Right-align big-endian in the 256-bit auth_value. */
			memcpy(&acs_conn->crypto.kex->auth_value[ACS_HMAC_SHA256_SIZE - oob_len],
			       oob_buf, oob_len);
		}
		break;
	default:
		break;
	}

	/* Store request data only after OOB processing succeeds */
	acs_conn->crypto.kex->start_kex = req_data;

	return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
				 BT_ACS_CP_RESPONSE_SUCCESS);
}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
int acs_cp_kex_exchange_ecdh(struct acs_procedure *proc, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;
	uint16_t key_id;
	int err;
	int arm_err;

	if (acs_conn->crypto.key_state != BT_ACS_KEY_EXCHANGE_STARTED) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	if (buf->len < 2) {
		LOG_WRN("ECDH key exchange operand too short: %u", buf->len);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	key_id = sys_get_le16(buf->data);

	/* Key_ID must match the one negotiated during Start Key Exchange. */
	if (!acs_conn->crypto.kex ||
	    key_id != sys_le16_to_cpu(acs_conn->crypto.kex->start_kex.key_id)) {
		LOG_WRN("ECDH key exchange with invalid Key_ID: 0x%04X", key_id);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	/* OOB path: no pubkey on the wire; fetch pre-shared key from app callback. */
	if (key_id == ACS_KEY_ID_OOB) {
		const struct bt_acs_cb *cb = acs_cb_get();

		if (!cb || !cb->oob_key_get) {
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
		err = cb->oob_key_get(proc->acs_conn->conn, acs_conn->crypto.kex->shared_secret,
				      &acs_conn->crypto.kex->key_mat_len);
		if (err || acs_conn->crypto.kex->key_mat_len == 0) {
			LOG_ERR("OOB key_get failed: %d", err);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
		acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_PUBKEY_EXCHANGED;
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					 BT_ACS_CP_RESPONSE_SUCCESS);
	}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_OOB */

	if (buf->len != sizeof(acs_conn->crypto.kex->client_pubkey)) {
		LOG_WRN("ECDH key exchange operand invalid length: %u", buf->len);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	/* Pull all operand data before response buffer init to avoid aliasing. */
	memcpy(&acs_conn->crypto.kex->client_pubkey, net_buf_simple_pull_mem(buf, buf->len),
	       sizeof(acs_conn->crypto.kex->client_pubkey));

	{
		struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
		struct net_buf *rsp_buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);

		if (!rsp_buf) {
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
		net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_RESPONSE);

		err = acs_key_exchange_ecdh_pubkey(acs_conn, &rsp_buf->b);

		if (err == -EBADMSG || err == -EINVAL) {
			LOG_ERR("ECDH pubkey: invalid client public key (err %d)", err);
			acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(proc, &kex_fail_seq);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 BT_ACS_CP_RESPONSE_INVALID_PUBLIC_KEY);
		} else if (err == -EAGAIN) {
			LOG_WRN("ECDH pubkey: wrong state (state %d)", acs_conn->crypto.key_state);
			acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(proc, &kex_fail_seq);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		} else if (err) {
			LOG_ERR("ECDH pubkey: internal error (err %d)", err);
			acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(proc, &kex_fail_seq);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		} else {
			arm_err = acs_cp_send_reply(proc);
			if (arm_err) {
				LOG_WRN("ECDH pubkey response arm failed: %d", arm_err);
			}
			return arm_err;
		}
	}
}

int acs_cp_kex_ecdh_confirm_code(struct acs_procedure *proc, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;
	struct acs_cp_ecdh_confirm_code_req req_data;
	uint16_t key_id;
	struct net_buf *rsp_buf;
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	int err;

	if (acs_conn->crypto.key_state != BT_ACS_KEY_EXCHANGE_PUBKEY_EXCHANGED &&
	    acs_conn->crypto.key_state != BT_ACS_KEY_EXCHANGE_KDF_DONE) {
		LOG_ERR("confirm_code: invalid state %d", acs_conn->crypto.key_state);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	if (buf->len != sizeof(struct acs_cp_ecdh_confirm_code_req)) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	/* Pull all operand data before response buffer init to avoid aliasing. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));

	if (!acs_conn->crypto.kex ||
	    sys_le16_to_cpu(req_data.key_id) !=
		    sys_le16_to_cpu(acs_conn->crypto.kex->start_kex.key_id)) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	/* Capture key_id before any kex free. */
	key_id = sys_le16_to_cpu(req_data.key_id);
	ARG_UNUSED(key_id); /* used only for log; kex stays alive */

	memcpy(acs_conn->crypto.kex->client_confirm, req_data.confirm_code, ACS_HMAC_SHA256_SIZE);

	rsp_buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);
	if (!rsp_buf) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_CODE_RESPONSE);

	err = acs_key_exchange_ecdh_confirm_code(acs_conn, &rsp_buf->b);

	if (err) {
		/* Keep kex alive long enough to emit KEY_EXCHANGE_RESPONSE(failed). */
		acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
		acs_seq_begin(proc, &kex_fail_seq);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	} else {
		err = acs_cp_send_reply(proc);
		if (err) {
			LOG_WRN("confirm_code: arm failed: %d", err);
		}
		return err;
	}
}

int acs_cp_kex_ecdh_confirm_rand(struct acs_procedure *proc, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;
	struct acs_cp_ecdh_confirm_rand_req req_data;
	struct net_buf *rsp_buf;
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	int err;

	if (acs_conn->crypto.key_state != BT_ACS_KEY_EXCHANGE_CONFIRM_CODE) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	if (buf->len != sizeof(struct acs_cp_ecdh_confirm_rand_req)) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	/* Pull all operand data before response buffer init to avoid aliasing. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));

	if (!acs_conn->crypto.kex ||
	    sys_le16_to_cpu(req_data.key_id) !=
		    sys_le16_to_cpu(acs_conn->crypto.kex->start_kex.key_id)) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	if (memcmp(req_data.random, acs_conn->crypto.kex->server_random, ACS_HMAC_SHA256_SIZE) ==
	    0) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	memcpy(acs_conn->crypto.kex->client_random, req_data.random, ACS_HMAC_SHA256_SIZE);

	rsp_buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);
	if (!rsp_buf) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	net_buf_add_u8(rsp_buf,
		       BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_RANDOM_NUMBER_RESPONSE);

	err = acs_key_exchange_ecdh_confirm_rand(acs_conn, &rsp_buf->b);

	if (err == -EACCES) {
		/* Confirmation code mismatch: chain KEX_RESPONSE(failed). */
		acs_seq_begin(proc, &kex_fail_seq);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					 BT_ACS_CP_RESPONSE_INVALID_KEY_EXCHANGE_CONFIRMATION_CODE);
	} else if (err) {
		acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
		acs_seq_begin(proc, &kex_fail_seq);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	/* If KDF was applied, the ECDH exchange key is already in the union
	 * buffer; publish it as the current ECDH key. This is still the parent
	 * key for the ACS descriptor chain. A separate standalone
	 * START_KEY_EXCHANGE(KEY_ID=KDF) / KEY_EXCHANGE_KDF pair may still
	 * derive the child algorithm key afterward when the descriptor graph
	 * requires it.
	 *
	 * Otherwise derive the ECDH/OOB session key directly from the raw
	 * shared secret. */
	if (acs_conn->crypto.kex->kdf_applied) {
		struct bt_acs_runtime_key_state *ecdh_key;

		if (acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_ECDH, &ecdh_key) != 0) {
			LOG_ERR("Missing ECDH runtime key state");
			acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(proc, &kex_fail_seq);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}

		memcpy(ecdh_key->key, acs_conn->crypto.kex->ecdh_key,
		       CONFIG_BT_ACS_SESSION_KEY_SIZE);
		ecdh_key->tx_nonce_counter = 0;
#if defined(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD)
		ecdh_key->rx_nonce_counter = 1;
#else
		ecdh_key->rx_nonce_counter = 0;
#endif
		err = acs_crypto_import_current_key(ecdh_key);
		if (err) {
			LOG_ERR("Failed to import ECDH current key: %d", err);
			acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(proc, &kex_fail_seq);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
	} else {
		err = acs_crypto_derive_session_key(acs_conn);
		if (err) {
			LOG_ERR("Failed to derive session key: %d", err);
			acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(proc, &kex_fail_seq);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
	}

	acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_PENDING_RESPONSE;
	acs_seq_begin(proc, &kex_success_seq);

	err = acs_cp_send_reply(proc);
	if (err) {
		LOG_WRN("confirm_rand: arm failed: %d", err);
		acs_seq_abort(proc);
	}
	return err;
}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */
