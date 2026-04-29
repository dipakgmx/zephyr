/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include "acs_internal.h"
#include "acs_wire_constants.h"
#include "zephyr/sys/byteorder.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static void acs_data_out_complete_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				     int err, void *user_data)
{
	struct acs_conn_ctx *conn_ctx;
	struct acs_procedure *proc;
	int step_result = -EIO;
	int abort_err;

	ARG_UNUSED(attr);

	proc = user_data;
	conn_ctx = acs_runtime_lookup_conn(conn);

	if (!conn_ctx) {
		return;
	}

	if (err != 0U) {
		acs_procedure_engine_abort(proc, -EIO);
	} else {
		if (conn_ctx->abort_pending) {
			bool secure = (conn_ctx->abort_flags & ACS_PROC_FLAG_SECURE_TRANSPORT) != 0U;

			conn_ctx->abort_pending = false;
			acs_procedure_engine_abort(proc, -ECANCELED);
			acs_procedure_engine_reset(proc);
			if (conn_ctx->kex.state != ACS_KEX_IDLE &&
			    conn_ctx->kex.state != ACS_KEX_COMPLETE) {
				acs_kex_reset(&conn_ctx->kex);
			}

			memset(proc, 0, sizeof(*proc));
			proc->conn = conn_ctx->conn;
			proc->resource_handle = conn_ctx->abort_resource_handle;
			proc->isc_id = conn_ctx->abort_isc_id;
			proc->flags = secure ? ACS_PROC_FLAG_SECURE_TRANSPORT : 0U;

			abort_err = acs_cp_domain_send_response_code(proc, BT_ACS_CP_OPCODE_ABORT,
								     0x01U);
			if (abort_err == ACS_PROC_STEP_WAIT_IND_CONFIRM) {
				return;
			}

			if (abort_err < 0) {
				acs_procedure_engine_abort(proc, abort_err);
			}

			k_work_submit(&conn_ctx->cp_complete_work);
			return;
		}

		step_result = acs_procedure_engine_on_confirm(proc);
		if (step_result != ACS_PROC_STEP_WAIT_IND_CONFIRM && step_result < 0) {
			acs_procedure_engine_abort(proc, step_result);
		}
	}

	if (step_result != ACS_PROC_STEP_WAIT_IND_CONFIRM) {
		k_work_submit(&conn_ctx->cp_complete_work);
	}
}

static int acs_single_pdu_len_ok(struct bt_conn *conn, uint16_t payload_len)
{
	uint16_t max_payload = ACS_SEG_PAYLOAD_SIZE(bt_gatt_get_mtu(conn));

	return payload_len <= max_payload ? 0 : -EMSGSIZE;
}

static int acs_send_notify(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   struct net_buf *plaintext)
{
	uint8_t pdu[ACS_SEG_HEADER_SIZE + CONFIG_BT_ACS_MAX_SEGMENT_SIZE];

	if (acs_single_pdu_len_ok(conn, plaintext->len) != 0) {
		return -EMSGSIZE;
	}

	pdu[0] = ACS_SEG_SINGLE_PDU;
	memcpy(&pdu[ACS_SEG_HEADER_SIZE], plaintext->data, plaintext->len);

	return bt_gatt_notify(conn, attr, pdu, plaintext->len + ACS_SEG_HEADER_SIZE);
}

static int acs_send_indicate(struct acs_conn_ctx *conn_ctx, const struct bt_gatt_attr *attr,
			     struct net_buf *plaintext, struct acs_seg_tx_ctx *tx)
{
	if (!bt_gatt_is_subscribed(conn_ctx->conn, attr, BT_GATT_CCC_INDICATE)) {
		LOG_WRN("ACS CP indicate rejected: client not subscribed");
		return -EINVAL;
	}

	return acs_seg_tx_send(tx, conn_ctx->conn, attr, plaintext, acs_data_out_complete_cb,
			       &conn_ctx->active_proc);
}

static int acs_build_secure_reply(struct acs_procedure *proc, struct net_buf *plaintext,
				  struct net_buf **out_wire)
{
	struct acs_conn_ctx *conn_ctx;
	struct net_buf *wire;
	uint8_t nonce_var[sizeof(uint64_t)];
	uint8_t tag[ACS_CRYPTO_AUTH_TAG_MAX_SIZE];
	uint8_t ciphertext[ACS_BUF_SIZE];
	uint8_t plain_wire[ACS_BUF_SIZE];
	size_t nonce_var_len = sizeof(nonce_var);
	size_t tag_len = sizeof(tag);
	size_t ciphertext_len = sizeof(ciphertext);
	size_t plain_wire_len;
	int err;

	if (!proc || !plaintext || !out_wire) {
		return -EINVAL;
	}

	conn_ctx = acs_runtime_lookup_conn(proc->conn);
	if (!conn_ctx) {
		return -ENOTCONN;
	}

	if (proc->resource_handle == 0U) {
		return -EINVAL;
	}

	plain_wire_len = sizeof(uint16_t) + plaintext->len;
	if (plain_wire_len > sizeof(plain_wire)) {
		return -EMSGSIZE;
	}

	sys_put_le16(proc->resource_handle, plain_wire);
	memcpy(&plain_wire[sizeof(uint16_t)], plaintext->data, plaintext->len);

	wire = acs_channel_buf_alloc();
	if (!wire) {
		return -ENOMEM;
	}

	if (net_buf_tailroom(wire) <
	    (sizeof(uint16_t) + acs_crypto_nonce_variable_size(&conn_ctx->crypto) +
	     acs_crypto_auth_tag_size(&conn_ctx->crypto) + plain_wire_len)) {
		acs_channel_buf_free(wire);
		return -ENOBUFS;
	}

	net_buf_add_le16(wire, proc->isc_id);
	err = acs_crypto_encrypt(&conn_ctx->crypto, plain_wire, plain_wire_len, nonce_var,
				 &nonce_var_len, tag, &tag_len, ciphertext, &ciphertext_len);
	if (err != 0) {
		acs_channel_buf_free(wire);
		return err;
	}

	net_buf_add_mem(wire, nonce_var, nonce_var_len);
	net_buf_add_mem(wire, tag, tag_len);
	net_buf_add_mem(wire, ciphertext, ciphertext_len);
	*out_wire = wire;
	return 0;
}

int acs_data_out_channel_send(struct acs_procedure *proc, struct acs_reply *reply)
{
	struct acs_conn_ctx *conn_ctx;
	const struct bt_gatt_attr *attr;
	int err;

	if (!proc || !reply || !proc->conn || !reply->plaintext) {
		LOG_WRN("ACS send rejected: invalid reply state");
		return -EINVAL;
	}

	conn_ctx = acs_runtime_lookup_conn(proc->conn);
	if (!conn_ctx) {
		return -ENOTCONN;
	}

	if (proc->pending_reply.plaintext && proc->pending_reply.plaintext != reply->plaintext) {
		acs_channel_buf_free(proc->pending_reply.plaintext);
		proc->pending_reply.plaintext = NULL;
	}

	switch (reply->channel) {
	case ACS_REPLY_CP:
		attr = acs_service_attr_cp();
		__ASSERT_NO_MSG(attr != NULL);
		if (!attr) {
			return -ENOENT;
		}
		err = acs_send_indicate(conn_ctx, attr, reply->plaintext, &conn_ctx->cp_tx);
		if (err) {
			LOG_WRN("ACS CP indicate failed: %d", err);
			return err;
		}
		proc->pending_reply = *reply;
		return ACS_PROC_STEP_WAIT_IND_CONFIRM;
	case ACS_REPLY_DON:
		if (!IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION) &&
		    !IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_READ)) {
			return -ENOTSUP;
		}
		attr = acs_service_attr_don();
		if (!attr) {
			return -ENOENT;
		}
		if (reply->encrypted) {
			struct net_buf *wire;

			err = acs_build_secure_reply(proc, reply->plaintext, &wire);
			if (err) {
				LOG_WRN("ACS DON secure build failed: %d", err);
				return err;
			}

			acs_channel_buf_free(reply->plaintext);
			reply->plaintext = wire;
			err = acs_seg_notify(proc->conn, attr, wire->data, wire->len);
		} else {
			err = acs_send_notify(proc->conn, attr, reply->plaintext);
		}
		if (err) {
			LOG_WRN("ACS DON notify failed: %d", err);
			return err;
		}
		proc->pending_reply = *reply;
		return ACS_PROC_RES_COMPLETE;
	case ACS_REPLY_DOI:
		if (!IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)) {
			return -ENOTSUP;
		}
		attr = acs_service_attr_doi();
		if (!attr) {
			return -ENOENT;
		}
		if (reply->encrypted) {
			struct net_buf *wire;

			err = acs_build_secure_reply(proc, reply->plaintext, &wire);
			if (err) {
				LOG_WRN("ACS DOI secure build failed: %d", err);
				return err;
			}

			acs_channel_buf_free(reply->plaintext);
			reply->plaintext = wire;
		}
		err = acs_send_indicate(conn_ctx, attr, reply->plaintext, &conn_ctx->doi_tx);
		if (err) {
			LOG_WRN("ACS DOI indicate failed: %d", err);
			return err;
		}
		proc->pending_reply = *reply;
		return ACS_PROC_STEP_WAIT_IND_CONFIRM;
	case ACS_REPLY_STATUS:
	default:
		LOG_WRN("ACS send rejected: unsupported reply channel %d", reply->channel);
		return -ENOTSUP;
	}
}
