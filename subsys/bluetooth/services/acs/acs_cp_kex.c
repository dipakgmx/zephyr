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

#define ACS_OUTPUT_NUMERIC_BUCKET_COUNT 9U

int acs_cp_kex_get_current_key_list(struct acs_procedure *proc)
{
	/* count(1 byte) + up to ACS_KEY_ID_COUNT x Key_ID(2 bytes) */
	uint8_t buf[sizeof(uint8_t) + ACS_KEY_ID_COUNT * sizeof(uint16_t)];
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	uint8_t count = 0;
	uint16_t pos = sizeof(uint8_t); /* byte 0 reserved for count */

	for (size_t i = 0; i < ARRAY_SIZE(proc->acs_conn->crypto.current_keys); i++) {
		const struct bt_acs_runtime_key_state *current_key =
			&proc->acs_conn->crypto.current_keys[i];

		if (!acs_current_key_installed(current_key)) {
			continue;
		}

		sys_put_le16(acs_runtime_key_id(current_key), &buf[pos]);
		pos += sizeof(uint16_t);
		count++;
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
		acs_key_exchange_abort(acs_conn);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	/* Pull all operand data before response buffer init to avoid aliasing. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));
	key_id = sys_le16_to_cpu(req_data.key_id);

	/* §4.4.3.17.2.1: Key Exchange KDF is only valid after Start Key Exchange
	 * for the same Key_ID. Reject if no KEX context exists or the Key_ID differs.
	 */

	if (!acs_kex_in_progress(acs_conn) ||
	    key_id != sys_le16_to_cpu(acs_conn->crypto.kex->start_kex.key_id)) {
		LOG_WRN("Key exchange KDF operand invalid Key_ID: 0x%04x, expected 0x%04x", key_id,
			acs_conn->crypto.kex
				? sys_le16_to_cpu(acs_conn->crypto.kex->start_kex.key_id)
				: 0);
		acs_key_exchange_abort(acs_conn);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	rsp_buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);
	if (!rsp_buf) {
		acs_key_exchange_abort(acs_conn);
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
			proc->seq_state = ACS_CP_SEQ_KEX_FAIL_RSP;
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		} else if (err) {
			LOG_ERR("KDF key exchange: internal error (err %d)", err);
			proc->seq_state = ACS_CP_SEQ_KEX_FAIL_RSP;
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
		/* KDF_RESPONSE must be delivered before KEY_EXCHANGE_RESPONSE (§4.4.3.10). */
		proc->seq_state = ACS_CP_SEQ_KEX_SUCCESS_RSP;
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
	if (!acs_kex_expects(acs_conn, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF)) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	err = acs_key_exchange_ecdh_kdf(acs_conn, &rsp_buf->b);

	if (err == -EAGAIN) {
		proc->seq_state = ACS_CP_SEQ_KEX_FAIL_RSP;
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	} else if (err) {
		LOG_ERR("ECDH KDF: internal error (err %d)", err);
		proc->seq_state = ACS_CP_SEQ_KEX_FAIL_RSP;
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	arm_err = acs_cp_send_reply(proc);
	if (arm_err) {
		LOG_WRN("ECDH KDF response arm failed: %d", arm_err);
		acs_key_exchange_abort(acs_conn);
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
	uint8_t oob_buf[ACS_CONFIRM_VALUE_SIZE];
	int oob_err;

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

	if (acs_kex_in_progress(acs_conn)) {
		LOG_WRN("Start Key Exchange rejected - another key exchange is already active");
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	if (key_id == ACS_KEY_ID_KDF) {
		struct bt_acs_runtime_key_state *parent_key;
		struct bt_acs_runtime_key_state *kdf_key;

		/* KDF standalone: requires parent key and method=None. */
		if (method != BT_ACS_CONFIRM_METHOD_NONE ||
		    action != BT_ACS_CONFIRM_ACTION_NOT_APPLICABLE) {
			LOG_WRN("invalid confirmation method/action (method=0x%02x action=0x%02x)",
				method, action);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
						 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		}
		if (acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_ECDH, &parent_key) != 0 ||
		    !acs_current_key_installed(parent_key)) {
			LOG_WRN("no ECDH parent key available, prior exchange required");
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
	} else
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF */
	{
		if (!acs_conn->crypto.kex) {
			if (acs_kex_alloc(acs_conn) != 0) {
				LOG_ERR("No free KEX context");
				return acs_cp_rsp_status(
					proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
			}
		}

		err = acs_key_exchange_ecdh_start(acs_conn, key_id);
	}

	if (err != 0) {
		acs_key_exchange_abort(acs_conn);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					 errno_to_acs_status(err));
	}

	if (!acs_conn->crypto.kex) {
		if (acs_kex_alloc(acs_conn) != 0) {
			LOG_ERR("No free KEX context");
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
	}

	/* Store request data before callbacks so bt_acs_set_oob_number() can
	 * validate the confirmation method.  If OOB processing fails below the
	 * kex context is aborted, making the cached value irrelevant.
	 */
	acs_conn->crypto.kex->start_kex = req_data;

	/* Clear auth_value before populating it in the switch cases below */
	memset(acs_conn->crypto.kex->auth_value, 0, sizeof(acs_conn->crypto.kex->auth_value));

	switch (method) {
	case BT_ACS_CONFIRM_METHOD_OUTPUT_OOB:
		/* Generate random OOB number; store right-aligned big-endian in auth_value. */
		sys_rand_get(&oob_num, sizeof(oob_num));
#if IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC)
		oob_num = (oob_num % CONFIG_BT_ACS_CONFIRMATION_OUTPUT_MAX_VALUE) + 1;
#else
		oob_num = (oob_num % ACS_OUTPUT_NUMERIC_BUCKET_COUNT) + 1;
#endif
		sys_put_be32(oob_num,
			     &acs_conn->crypto.kex
				      ->auth_value[ACS_CONFIRM_VALUE_SIZE - sizeof(oob_num)]);
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
			if (oob_err || oob_len == 0 || oob_len > ACS_CONFIRM_VALUE_SIZE) {
				LOG_WRN("start_key_exchange: static_oob_get failed: %d", oob_err);
				acs_key_exchange_abort(acs_conn);
				return acs_cp_rsp_status(
					proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
			}
			/* Right-align big-endian in the 256-bit auth_value. */
			memcpy(&acs_conn->crypto.kex->auth_value[ACS_CONFIRM_VALUE_SIZE - oob_len],
			       oob_buf, oob_len);
		}
		break;
	default:
		break;
	}

	acs_conn->crypto.kex->next_expected_opcode = (key_id == ACS_KEY_ID_KDF)
							     ? BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF
							     : BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH;

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

	if (!acs_kex_expects(acs_conn, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH)) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	key_id = sys_get_le16(buf->data);

	/* Key_ID must match the one negotiated during Start Key Exchange. */
	if (!acs_kex_in_progress(acs_conn) ||
	    key_id != sys_le16_to_cpu(acs_conn->crypto.kex->start_kex.key_id)) {
		LOG_WRN("ECDH key exchange with invalid Key_ID: 0x%04X", key_id);
		acs_key_exchange_abort(acs_conn);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	/* OOB path: no pubkey on the wire; fetch pre-shared key from app callback. */
	if (key_id == ACS_KEY_ID_OOB) {
		const struct bt_acs_cb *cb = acs_cb_get();

		if (!cb || !cb->oob_key_get) {
			acs_key_exchange_abort(acs_conn);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
		err = cb->oob_key_get(proc->acs_conn->conn, acs_conn->crypto.kex->shared_secret,
				      &acs_conn->crypto.kex->key_mat_len);
		if (err || acs_conn->crypto.kex->key_mat_len == 0) {
			LOG_ERR("OOB key_get failed: %d", err);
			acs_key_exchange_abort(acs_conn);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
		acs_conn->crypto.kex->next_expected_opcode = BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE;
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					 BT_ACS_CP_RESPONSE_SUCCESS);
	}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_OOB */

	if (buf->len != sizeof(acs_conn->crypto.kex->client_pubkey)) {
		LOG_WRN("ECDH key exchange operand invalid length: %u", buf->len);
		acs_key_exchange_abort(acs_conn);
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
			acs_key_exchange_abort(acs_conn);
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
		net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_RESPONSE);

		err = acs_key_exchange_ecdh_pubkey(acs_conn, &rsp_buf->b);

		if (err == -EBADMSG || err == -EINVAL) {
			LOG_ERR("ECDH pubkey: invalid client public key (err %d)", err);
			proc->seq_state = ACS_CP_SEQ_KEX_FAIL_RSP;
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 BT_ACS_CP_RESPONSE_INVALID_PUBLIC_KEY);
		} else if (err == -EAGAIN) {
			proc->seq_state = ACS_CP_SEQ_KEX_FAIL_RSP;
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		} else if (err) {
			LOG_ERR("ECDH pubkey: internal error (err %d)", err);
			proc->seq_state = ACS_CP_SEQ_KEX_FAIL_RSP;
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		} else {
			arm_err = acs_cp_send_reply(proc);
			if (arm_err) {
				LOG_WRN("ECDH pubkey response arm failed: %d", arm_err);
				acs_key_exchange_abort(acs_conn);
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

	if (!acs_kex_accepts_opcode(acs_conn, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE)) {
		LOG_ERR("confirm_code: invalid KEX ordering");
		acs_key_exchange_abort(acs_conn);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	if (buf->len != sizeof(struct acs_cp_ecdh_confirm_code_req)) {
		acs_key_exchange_abort(acs_conn);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	/* Pull all operand data before response buffer init to avoid aliasing. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));

	if (!acs_kex_in_progress(acs_conn) ||
	    sys_le16_to_cpu(req_data.key_id) !=
		    sys_le16_to_cpu(acs_conn->crypto.kex->start_kex.key_id)) {
		acs_key_exchange_abort(acs_conn);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	/* Capture key_id before any kex free. */
	key_id = sys_le16_to_cpu(req_data.key_id);
	ARG_UNUSED(key_id); /* used only for log; kex stays alive */

	memcpy(acs_conn->crypto.kex->client_confirm, req_data.confirm_code, ACS_CONFIRM_VALUE_SIZE);

	rsp_buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);
	if (!rsp_buf) {
		acs_key_exchange_abort(acs_conn);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_CODE_RESPONSE);

	err = acs_key_exchange_ecdh_confirm_code(acs_conn, &rsp_buf->b);

	if (err) {
		/* Keep kex alive long enough to emit KEY_EXCHANGE_RESPONSE(failed). */
		proc->seq_state = ACS_CP_SEQ_KEX_FAIL_RSP;
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	} else {
		err = acs_cp_send_reply(proc);
		if (err) {
			LOG_WRN("confirm_code: arm failed: %d", err);
			acs_key_exchange_abort(acs_conn);
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

	if (!acs_kex_expects(acs_conn, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND)) {
		acs_key_exchange_abort(acs_conn);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	if (buf->len != sizeof(struct acs_cp_ecdh_confirm_rand_req)) {
		acs_key_exchange_abort(acs_conn);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	/* Pull all operand data before response buffer init to avoid aliasing. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));

	if (!acs_kex_in_progress(acs_conn) ||
	    sys_le16_to_cpu(req_data.key_id) !=
		    sys_le16_to_cpu(acs_conn->crypto.kex->start_kex.key_id)) {
		acs_key_exchange_abort(acs_conn);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	if (memcmp(req_data.random, acs_conn->crypto.kex->server_random, ACS_CONFIRM_VALUE_SIZE) ==
	    0) {
		acs_key_exchange_abort(acs_conn);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	memcpy(acs_conn->crypto.kex->client_random, req_data.random, ACS_CONFIRM_VALUE_SIZE);

	rsp_buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);
	if (!rsp_buf) {
		acs_key_exchange_abort(acs_conn);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	net_buf_add_u8(rsp_buf,
		       BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_RANDOM_NUMBER_RESPONSE);

	err = acs_key_exchange_ecdh_confirm_rand(acs_conn, &rsp_buf->b);

	if (err == -EACCES) {
		/* Confirmation code mismatch: chain KEX_RESPONSE(failed). */
		proc->seq_state = ACS_CP_SEQ_KEX_FAIL_RSP;
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					 BT_ACS_CP_RESPONSE_INVALID_KEY_EXCHANGE_CONFIRMATION_CODE);
	} else if (err) {
		proc->seq_state = ACS_CP_SEQ_KEX_FAIL_RSP;
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
			proc->seq_state = ACS_CP_SEQ_KEX_FAIL_RSP;
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}

		memcpy(ecdh_key->key, acs_conn->crypto.kex->ecdh_key,
		       CONFIG_BT_ACS_SESSION_KEY_SIZE);
		err = acs_crypto_import_current_key(ecdh_key);
		if (err == 0) {
			err = acs_crypto_rebind_key_desc_runtimes(acs_conn);
			if (err == 0) {
				acs_crypto_reset_key_desc_runtime_counters(acs_conn,
									   ACS_KEY_ID_ECDH);
			}
		}
		if (err) {
			LOG_ERR("Failed to import ECDH current key: %d", err);
			proc->seq_state = ACS_CP_SEQ_KEX_FAIL_RSP;
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
	} else {
		err = acs_crypto_derive_session_key(acs_conn);
		if (err) {
			LOG_ERR("Failed to derive session key: %d", err);
			proc->seq_state = ACS_CP_SEQ_KEX_FAIL_RSP;
			return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
						 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
	}

	proc->seq_state = ACS_CP_SEQ_KEX_SUCCESS_RSP;

	err = acs_cp_send_reply(proc);
	if (err) {
		LOG_WRN("confirm_rand: arm failed: %d", err);
		acs_seq_abort(proc);
	}
	return err;
}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */
