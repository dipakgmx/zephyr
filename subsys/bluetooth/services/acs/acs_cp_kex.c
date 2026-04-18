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

void acs_cp_kex_get_current_key_list(struct acs_cp_ctx *ctx)
{
	/* count(1 byte) + up to ACS_KEY_ID_COUNT x Key_ID(2 bytes) */
	uint8_t buf[sizeof(uint8_t) + ACS_KEY_ID_COUNT * sizeof(uint16_t)];
	uint8_t count = 0;
	uint16_t pos = sizeof(uint8_t); /* byte 0 reserved for count */

	if (ctx->acs_conn->key_state == BT_ACS_KEY_EXCHANGE_COMPLETE) {
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
		sys_put_le16(ACS_KEY_ID_ECDH, &buf[pos]);
		pos += sizeof(uint16_t);
		count++;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
		sys_put_le16(ACS_KEY_ID_OOB, &buf[pos]);
		pos += sizeof(uint16_t);
		count++;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
		/* The KDF child key is only valid once the peer has completed the KDF
		 * exchange for this connection.  Reporting it before that point would
		 * let the peer attempt AEAD with a key that does not yet exist on the
		 * server side, causing spurious decryption failures. */
		if (ctx->acs_conn->kdf_child_active) {
			sys_put_le16(ACS_KEY_ID_KDF, &buf[pos]);
			pos += sizeof(uint16_t);
			count++;
		}
#endif
	}

	buf[0] = count;

	{
		struct net_buf *rsp_buf = acs_cp_rsp_alloc(ctx);

		if (!rsp_buf) {
			LOG_WRN("buffer pool exhausted");
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST,
					  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
			return;
		}
		net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_CURRENT_KEY_LIST_RESPONSE);
		net_buf_add_mem(rsp_buf, buf, pos);
		acs_cp_rsp_send(ctx);
	}
}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH) || IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)

/* --- Key exchange success sequence steps --- */

static int kex_step_success_response(struct acs_cp_ctx *ctx)
{
	struct bt_acs_conn *acs_conn = ctx->acs_conn;
	uint8_t payload[3];
	uint16_t key_id;

	if (!acs_conn || !acs_conn->kex) {
		return -EINVAL;
	}

	key_id = sys_le16_to_cpu(acs_conn->kex->start_kex.key_id);
	acs_conn->key_state = BT_ACS_KEY_EXCHANGE_PENDING_STATUS;
	sys_put_le16(key_id, &payload[0]);
	payload[2] = 0x00;

	struct net_buf *buf = acs_cp_rsp_alloc(ctx);

	if (!buf) {
		return -ENOMEM;
	}
	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_RESPONSE);
	net_buf_add_mem(buf, payload, sizeof(payload));
	return acs_cp_rsp_send(ctx);
}

static int kex_step_status(struct acs_cp_ctx *ctx)
{
	struct bt_acs_conn *acs_conn = ctx->acs_conn;

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	/* Mark the connection so key-list and AEAD gating know the child key is live.
	 * Must be set before kex context is freed since we read start_kex.key_id. */
	if (acs_conn->kex && sys_le16_to_cpu(acs_conn->kex->start_kex.key_id) == ACS_KEY_ID_KDF) {
		acs_conn->kdf_child_active = true;
	}
#endif

	acs_kex_free(acs_conn->kex);
	acs_conn->kex = NULL;
	acs_conn->key_state = BT_ACS_KEY_EXCHANGE_COMPLETE;

	/* The KEY_EXCHANGE_RESPONSE indication has been confirmed by the peer, so
	 * both sides agree the exchange succeeded.  Set the flag and notify the
	 * application now — doing it earlier (in the handler that built the response)
	 * would race against a connection drop before the peer acknowledged it. */
	acs_conn->status_flags |= BT_ACS_STATUS_SECURITY_ESTABLISHED;
	{
		const struct bt_acs_cb *cb = acs_cb_get();

		if (cb && cb->security_established) {
			cb->security_established(ctx->conn, acs_conn->crypto.session_key,
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
	 * handles the two-key layout, writing the parent key (from ecdh_parent_key)
	 * and the child key with its nonce counters into separate fields so both
	 * can be restored on reconnect without repeating either exchange. */
#if IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
	if (!acs_conn->kdf_child_active)
#endif
	{
		acs_session_store(ctx->conn, acs_conn);
	}
#endif

	acs_seq_clear(ctx);
	acs_status_indicate(ctx->conn);
	return 0;
}

static void kex_on_abort(struct acs_cp_ctx *ctx)
{
	struct bt_acs_conn *acs_conn = ctx ? ctx->acs_conn : NULL;

	if (!acs_conn || !acs_conn->kex) {
		return;
	}

	acs_kex_free(acs_conn->kex);
	acs_conn->kex = NULL;
	if (acs_conn->key_state == BT_ACS_KEY_EXCHANGE_PENDING_RESPONSE ||
	    acs_conn->key_state == BT_ACS_KEY_EXCHANGE_PENDING_STATUS) {
		acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
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

static int kex_step_fail_response(struct acs_cp_ctx *ctx)
{
	struct bt_acs_conn *acs_conn = ctx->acs_conn;
	uint8_t payload[3];
	uint16_t key_id;

	if (!acs_conn || !acs_conn->kex) {
		return -EINVAL;
	}

	key_id = sys_le16_to_cpu(acs_conn->kex->start_kex.key_id);
	sys_put_le16(key_id, &payload[0]);
	payload[2] = 0x01;

	struct net_buf *buf = acs_cp_rsp_alloc(ctx);

	if (!buf) {
		return -ENOMEM;
	}
	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_RESPONSE);
	net_buf_add_mem(buf, payload, sizeof(payload));
	return acs_cp_rsp_send(ctx);
}

static int kex_step_fail_cleanup(struct acs_cp_ctx *ctx)
{
	struct bt_acs_conn *acs_conn = ctx->acs_conn;

	acs_kex_free(acs_conn->kex);
	acs_conn->kex = NULL;
	acs_seq_clear(ctx);
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

void acs_cp_kex_exchange_kdf(struct acs_cp_ctx *ctx, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = ctx->acs_conn;
	struct acs_kdf_req req_data;
	uint16_t key_id;
	struct net_buf *rsp_buf;
	int err;
	int arm_err;

	if (buf->len != sizeof(struct acs_kdf_req)) {
		LOG_WRN("Key exchange KDF operand invalid length: %u", buf->len);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	/* Pull all operand data before response buffer init to avoid aliasing. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));
	key_id = sys_le16_to_cpu(req_data.key_id);

	/* §4.4.3.17.2.1: Key Exchange KDF is only valid after Start Key Exchange
	 * for the same Key_ID. Reject if no KEX context exists or the Key_ID differs.
	 */

	if (!acs_conn->kex || key_id != sys_le16_to_cpu(acs_conn->kex->start_kex.key_id)) {
		LOG_WRN("Key exchange KDF operand invalid Key_ID: 0x%04x, expected 0x%04x", key_id,
			acs_conn->kex ? sys_le16_to_cpu(acs_conn->kex->start_kex.key_id) : 0);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		return;
	}

	rsp_buf = acs_cp_rsp_alloc(ctx);
	if (!rsp_buf) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}
	net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF_RESPONSE);

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	if (key_id == ACS_KEY_ID_KDF) {
		err = acs_key_exchange_kdf(acs_conn, &rsp_buf->b);

		/* Standalone KDF is only applicable once the higher-level parent key is
		 * established and the KEX state machine has been advanced by Start Key Exchange.
		 */
		if (err == -EAGAIN) {
			LOG_WRN("KDF key exchange: wrong state (state %d)", acs_conn->key_state);
			acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(ctx, &kex_fail_seq);
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
					  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
			return;
		} else if (err) {
			LOG_ERR("KDF key exchange: internal error (err %d)", err);
			acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(ctx, &kex_fail_seq);
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
					  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
			return;
		}

		/* Nonce counters are reset inside bt_acs_crypto_derive_kdf_child_key()
		 * with the correct SEQ_EVEN_ODD logic — no separate reset needed here.
		 *
		 * SECURITY_ESTABLISHED flag and the security_established callback are
		 * deferred to kex_step_status, which runs after the KEY_EXCHANGE_RESPONSE
		 * indication is confirmed by the peer.  Setting them here would tell the
		 * application the session is live before the peer has acknowledged it. */
		acs_conn->key_state = BT_ACS_KEY_EXCHANGE_PENDING_RESPONSE;

		/* KDF_RESPONSE must be delivered before KEY_EXCHANGE_RESPONSE (§4.4.3.10). */
		acs_seq_begin(ctx, &kex_success_seq);
		arm_err = acs_cp_rsp_send(ctx);

		if (arm_err) {
			LOG_WRN("KDF standalone response indication arm failed: %d", arm_err);
			acs_seq_abort(ctx);
		}
		return;
	}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	/* ECDH KDF path: KEY_EXCHANGE_RESPONSE follows after Confirmation Random Number. */
	/* In the ECDH flow, this KDF step is only valid after the peer public key has
	 * been processed and the shared secret is available for derivation.
	 */
	if (acs_conn->key_state != BT_ACS_KEY_EXCHANGE_PUBKEY_EXCHANGED) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		return;
	}

	err = acs_key_exchange_ecdh_kdf(acs_conn, &rsp_buf->b);

	if (err == -EAGAIN) {
		LOG_WRN("ECDH KDF: wrong state (state %d)", acs_conn->key_state);
		acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
		acs_seq_begin(ctx, &kex_fail_seq);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		return;
	} else if (err) {
		LOG_ERR("ECDH KDF: internal error (err %d)", err);
		acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
		acs_seq_begin(ctx, &kex_fail_seq);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}

	arm_err = acs_cp_rsp_send(ctx);
	if (arm_err) {
		LOG_WRN("ECDH KDF response arm failed: %d", arm_err);
	}
#else
	acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
			  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */
}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF || CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

void acs_cp_kex_start(struct acs_cp_ctx *ctx, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = ctx->acs_conn;
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
	if (!acs_session_cache_has_room(bt_conn_get_dst(ctx->conn))) {
		LOG_WRN("start_kex: session cache full");
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}
#endif /* CONFIG_BT_SETTINGS */

	if (buf->len < sizeof(struct acs_cp_start_key_exchange_req)) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	/* Pull all operand data before any response buffer init. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));
	key_id = sys_le16_to_cpu(req_data.key_id);
	method = req_data.confirmation_method;
	action = req_data.confirmation_action;
	method_action_valid = true;
	cb = acs_cb_get();

	/* Validate method/action combinations (Tables 4.50–4.52); RFU → invalid operand. */
	switch (method) {
	case BT_ACS_CONFIRM_METHOD_NONE:
		if (action != BT_ACS_CONFIRM_ACTION_NOT_APPLICABLE) {
			method_action_valid = false;
		}
		break;
	case BT_ACS_CONFIRM_METHOD_OUTPUT_OOB:
		if (action != BT_ACS_CONFIRM_ACTION_OUTPUT_BEEP &&
		    action != BT_ACS_CONFIRM_ACTION_OUTPUT_NUMERIC) {
			method_action_valid = false;
		}
		break;
	case BT_ACS_CONFIRM_METHOD_INPUT_OOB:
		if (action != BT_ACS_CONFIRM_ACTION_INPUT_PUSH &&
		    action != BT_ACS_CONFIRM_ACTION_INPUT_NUMERIC) {
			method_action_valid = false;
		}
		break;
	case BT_ACS_CONFIRM_METHOD_STATIC_OOB:
		if (action != BT_ACS_CONFIRM_ACTION_NOT_APPLICABLE) {
			method_action_valid = false;
		}
		break;
	default:
		method_action_valid = false;
		break;
	}

	if (!method_action_valid) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	err = 0;

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	if (key_id == ACS_KEY_ID_KDF) {
		/* KDF standalone: requires parent key and method=None. */
		if (method != BT_ACS_CONFIRM_METHOD_NONE ||
		    action != BT_ACS_CONFIRM_ACTION_NOT_APPLICABLE) {
			LOG_WRN("invalid confirmation method/action (method=0x%02x action=0x%02x)",
				method, action);
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
			return;
		}
		if (acs_conn->key_state != BT_ACS_KEY_EXCHANGE_COMPLETE) {
			LOG_WRN("no parent key available (key_state=%d, prior ECDH exchange "
				"required)",
				acs_conn->key_state);
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
			return;
		}
		/* A KDF child key derived from the ECDH parent is already installed.
		 * The peer must invalidate it before deriving a new one; silently
		 * re-deriving would orphan the old child's nonce counters and create
		 * ambiguity about which key is current in NVS. */
		if (acs_conn->kdf_child_active) {
			LOG_WRN("KDF child key already active — invalidate before re-exchange");
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
			return;
		}
		acs_conn->key_state = BT_ACS_KEY_EXCHANGE_STARTED;
	} else
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF */
	{
		err = acs_key_exchange_ecdh_start(acs_conn, key_id);
	}

	if (err != 0) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
				  errno_to_acs_status(err));
		return;
	}

	if (!acs_conn->kex) {
		acs_conn->kex = acs_kex_alloc();
		if (!acs_conn->kex) {
			LOG_ERR("No free KEX context");
			acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
			return;
		}
	}

	/* Clear auth_value before populating it in the switch cases below */
	memset(acs_conn->kex->auth_value, 0, sizeof(acs_conn->kex->auth_value));

	switch (method) {
	case BT_ACS_CONFIRM_METHOD_OUTPUT_OOB:
		/* Generate random OOB number; store right-aligned big-endian in auth_value. */
		sys_rand_get(&oob_num, sizeof(oob_num));
#if IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC)
		oob_num = (oob_num % CONFIG_BT_ACS_CONFIRMATION_OUTPUT_MAX_VALUE) + 1;
#else
		oob_num = (oob_num % 9) + 1;
#endif
		sys_put_be32(oob_num,
			     &acs_conn->kex->auth_value[ACS_HMAC_SHA256_SIZE - sizeof(oob_num)]);
		if (cb && cb->output_oob_number) {
			cb->output_oob_number(ctx->conn, action, oob_num);
		}
		break;
	case BT_ACS_CONFIRM_METHOD_INPUT_OOB:
		if (cb && cb->input_oob_request) {
			cb->input_oob_request(ctx->conn, action);
		}
		break;
	case BT_ACS_CONFIRM_METHOD_STATIC_OOB:
		if (cb && cb->static_oob_get) {
			oob_len = 0;
			oob_err = cb->static_oob_get(ctx->conn, oob_buf, &oob_len);
			if (oob_err || oob_len == 0 || oob_len > ACS_HMAC_SHA256_SIZE) {
				LOG_WRN("start_key_exchange: static_oob_get failed: %d", oob_err);
				acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
				acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
						  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
				return;
			}
			/* Right-align big-endian in the 256-bit auth_value. */
			memcpy(&acs_conn->kex->auth_value[ACS_HMAC_SHA256_SIZE - oob_len], oob_buf,
			       oob_len);
		}
		break;
	default:
		break;
	}

	/* Store request data only after OOB processing succeeds */
	acs_conn->kex->start_kex = req_data;

	acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE, BT_ACS_CP_RESPONSE_SUCCESS);
}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
void acs_cp_kex_exchange_ecdh(struct acs_cp_ctx *ctx, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = ctx->acs_conn;
	uint16_t key_id;
	int err;
	int arm_err;

	if (acs_conn->key_state != BT_ACS_KEY_EXCHANGE_STARTED) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		return;
	}

	if (buf->len < 2) {
		LOG_WRN("ECDH key exchange operand too short: %u", buf->len);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	key_id = sys_get_le16(buf->data);

	/* Key_ID must match the one negotiated during Start Key Exchange. */
	if (!acs_conn->kex || key_id != sys_le16_to_cpu(acs_conn->kex->start_kex.key_id)) {
		LOG_WRN("ECDH key exchange with invalid Key_ID: 0x%04X", key_id);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		return;
	}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	/* OOB path: no pubkey on the wire; fetch pre-shared key from app callback. */
	if (key_id == ACS_KEY_ID_OOB) {
		const struct bt_acs_cb *cb = acs_cb_get();

		if (!cb || !cb->oob_key_get) {
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
			return;
		}
		err = cb->oob_key_get(ctx->conn, acs_conn->kex->shared_secret,
				      &acs_conn->kex->key_mat_len);
		if (err || acs_conn->kex->key_mat_len == 0) {
			LOG_ERR("OOB key_get failed: %d", err);
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
			return;
		}
		acs_conn->key_state = BT_ACS_KEY_EXCHANGE_PUBKEY_EXCHANGED;
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
				  BT_ACS_CP_RESPONSE_SUCCESS);
		return;
	}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_OOB */

	if (buf->len != sizeof(acs_conn->kex->client_pubkey)) {
		LOG_WRN("ECDH key exchange operand invalid length: %u", buf->len);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	/* Pull all operand data before response buffer init to avoid aliasing. */
	memcpy(&acs_conn->kex->client_pubkey, net_buf_simple_pull_mem(buf, buf->len),
	       sizeof(acs_conn->kex->client_pubkey));

	{
		struct net_buf *rsp_buf = acs_cp_rsp_alloc(ctx);

		if (!rsp_buf) {
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
			return;
		}
		net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_RESPONSE);

		err = acs_key_exchange_ecdh_pubkey(acs_conn, &rsp_buf->b);

		if (err == -EBADMSG || err == -EINVAL) {
			LOG_ERR("ECDH pubkey: invalid client public key (err %d)", err);
			acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(ctx, &kex_fail_seq);
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					  BT_ACS_CP_RESPONSE_INVALID_PUBLIC_KEY);
		} else if (err == -EAGAIN) {
			LOG_WRN("ECDH pubkey: wrong state (state %d)", acs_conn->key_state);
			acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(ctx, &kex_fail_seq);
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		} else if (err) {
			LOG_ERR("ECDH pubkey: internal error (err %d)", err);
			acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(ctx, &kex_fail_seq);
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
					  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		} else {
			arm_err = acs_cp_rsp_send(ctx);
			if (arm_err) {
				LOG_WRN("ECDH pubkey response arm failed: %d", arm_err);
			}
		}
	}
}

void acs_cp_kex_ecdh_confirm_code(struct acs_cp_ctx *ctx, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = ctx->acs_conn;
	struct acs_cp_ecdh_confirm_code_req req_data;
	uint16_t key_id;
	struct net_buf *rsp_buf;
	int err;

	if (acs_conn->key_state != BT_ACS_KEY_EXCHANGE_PUBKEY_EXCHANGED &&
	    acs_conn->key_state != BT_ACS_KEY_EXCHANGE_KDF_DONE) {
		LOG_ERR("confirm_code: invalid state %d", acs_conn->key_state);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		return;
	}

	if (buf->len != sizeof(struct acs_cp_ecdh_confirm_code_req)) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	/* Pull all operand data before response buffer init to avoid aliasing. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));

	if (!acs_conn->kex ||
	    sys_le16_to_cpu(req_data.key_id) != sys_le16_to_cpu(acs_conn->kex->start_kex.key_id)) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		return;
	}

	/* Capture key_id before any kex free. */
	key_id = sys_le16_to_cpu(req_data.key_id);
	ARG_UNUSED(key_id); /* used only for log; kex stays alive */

	memcpy(acs_conn->kex->client_confirm, req_data.confirm_code, ACS_HMAC_SHA256_SIZE);

	rsp_buf = acs_cp_rsp_alloc(ctx);
	if (!rsp_buf) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}
	net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_CODE_RESPONSE);

	err = acs_key_exchange_ecdh_confirm_code(acs_conn, &rsp_buf->b);

	if (err) {
		/* Keep kex alive long enough to emit KEY_EXCHANGE_RESPONSE(failed). */
		acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
		acs_seq_begin(ctx, &kex_fail_seq);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	} else {
		err = acs_cp_rsp_send(ctx);
		if (err) {
			LOG_WRN("confirm_code: arm failed: %d", err);
		}
	}
}

void acs_cp_kex_ecdh_confirm_rand(struct acs_cp_ctx *ctx, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = ctx->acs_conn;
	struct acs_cp_ecdh_confirm_rand_req req_data;
	struct net_buf *rsp_buf;
	int err;

	if (acs_conn->key_state != BT_ACS_KEY_EXCHANGE_CONFIRM_CODE) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		return;
	}

	if (buf->len != sizeof(struct acs_cp_ecdh_confirm_rand_req)) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	/* Pull all operand data before response buffer init to avoid aliasing. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));

	if (!acs_conn->kex ||
	    sys_le16_to_cpu(req_data.key_id) != sys_le16_to_cpu(acs_conn->kex->start_kex.key_id)) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		return;
	}

	if (memcmp(req_data.random, acs_conn->kex->server_random, ACS_HMAC_SHA256_SIZE) == 0) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	memcpy(acs_conn->kex->client_random, req_data.random, ACS_HMAC_SHA256_SIZE);

	rsp_buf = acs_cp_rsp_alloc(ctx);
	if (!rsp_buf) {
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}
	net_buf_add_u8(rsp_buf,
		       BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_RANDOM_NUMBER_RESPONSE);

	err = acs_key_exchange_ecdh_confirm_rand(acs_conn, &rsp_buf->b);

	if (err == -EACCES) {
		/* Confirmation code mismatch: chain KEX_RESPONSE(failed). */
		acs_seq_begin(ctx, &kex_fail_seq);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
				  BT_ACS_CP_RESPONSE_INVALID_KEY_EXCHANGE_CONFIRMATION_CODE);
		return;
	} else if (err) {
		acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
		acs_seq_begin(ctx, &kex_fail_seq);
		acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}

	/* If KDF was applied, the derived key is already in the union buffer;
	 * copy it as the session key.  Otherwise derive via HKDF from the
	 * raw shared secret. */
	if (acs_conn->kex->kdf_applied) {
		memcpy(acs_conn->crypto.session_key, acs_conn->kex->ecdh_key,
		       CONFIG_BT_ACS_SESSION_KEY_SIZE);
	} else {
		err = acs_crypto_derive_session_key(acs_conn);
		if (err) {
			LOG_ERR("Failed to derive session key: %d", err);
			acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			acs_seq_begin(ctx, &kex_fail_seq);
			acs_cp_rsp_status(ctx, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
					  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
			return;
		}
	}

	acs_conn->key_state = BT_ACS_KEY_EXCHANGE_PENDING_RESPONSE;
	acs_seq_begin(ctx, &kex_success_seq);

	err = acs_cp_rsp_send(ctx);
	if (err) {
		LOG_WRN("confirm_rand: arm failed: %d", err);
		acs_seq_abort(ctx);
	}
}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */
