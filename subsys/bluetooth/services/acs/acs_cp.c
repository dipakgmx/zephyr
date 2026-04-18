/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_cp.h"
#include "acs_internal.h"
#include "acs_cp_handlers.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static void acs_cp_on_indicate_done(struct bt_conn *conn, const struct bt_gatt_attr *attr, int err,
				    void *user_data);

struct net_buf *acs_cp_rsp_alloc(struct acs_cp_ctx *ctx)
{
	struct net_buf *buf;

	__ASSERT_NO_MSG(ctx != NULL);
	__ASSERT_NO_MSG(ctx->acs_conn != NULL);

	if (ctx->prot_req) {
		/* Protected path: lazy alloc with crypto headroom */
		if (!ctx->prot_req->response) {
			ctx->prot_req->response = acs_buf_alloc(K_NO_WAIT);
			if (!ctx->prot_req->response) {
				LOG_ERR("CP rsp_begin: buffer pool exhausted");
				return NULL;
			}
		} else {
			net_buf_reset(ctx->prot_req->response);
		}
		buf = ctx->prot_req->response;
		net_buf_reserve(buf, ACS_CRYPTO_HEADROOM);
		net_buf_add_le16(buf, ctx->prot_req->resource_handle);
	} else {
		struct acs_cp_proc_ctx *proc = &ctx->acs_conn->cp_proc;

		if (!proc->response) {
			proc->response = acs_buf_alloc(K_NO_WAIT);
			if (!proc->response) {
				LOG_ERR("buffer pool exhausted");
				return NULL;
			}
		} else {
			net_buf_reset(proc->response);
		}
		buf = proc->response;
	}
	return buf;
}

int acs_cp_rsp_send(struct acs_cp_ctx *ctx)
{
	struct bt_acs_prot_resource_req *req = ctx->prot_req;
	struct bt_conn *conn = ctx->conn;
	struct bt_acs_conn *acs_conn = ctx->acs_conn;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);

	if (req) {
		uint16_t resource_handle = req->resource_handle;

		__ASSERT_NO_MSG(req->response != NULL);
		__ASSERT_NO_MSG(req->response->len > 0);
		err = acs_prot_resource_rsp_indicate(req);
		if (err) {
			acs_seq_abort(ctx);
			LOG_WRN("Protected CP DOI queue failed for handle 0x%04x: %d",
				resource_handle, err);
		}
		return err;
	}

	struct net_buf *rsp_buf = acs_conn->cp_proc.response;

	acs_conn->cp_proc.response = NULL;

	__ASSERT_NO_MSG(rsp_buf != NULL);
	__ASSERT_NO_MSG(rsp_buf->len > 0);

	/* Pass the buffer via user_data so the completion callback can free
	 * it.  The seg-TX engine borrows the buffer but does not own it.
	 */
	err = acs_seg_tx_send(&acs_conn->cp_tx, conn, ctx->attr, rsp_buf, acs_cp_on_indicate_done,
			      rsp_buf);
	if (err) {
		acs_seq_abort(ctx);
		atomic_set(&acs_conn->cp_proc.locked, 0);
		acs_buf_free(rsp_buf);
		LOG_WRN("response indication failed: %d", err);
	}
	return err;
}

int acs_cp_rsp_status(struct acs_cp_ctx *ctx, uint8_t req_opcode, uint8_t code)
{
	struct net_buf *buf = acs_cp_rsp_alloc(ctx);

	if (!buf) {
		acs_seq_abort(ctx);
		if (!ctx->prot_req) {
			atomic_set(&ctx->acs_conn->cp_proc.locked, 0);
		}
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_RESPONSE_CODE);
	net_buf_add_u8(buf, req_opcode);
	net_buf_add_u8(buf, code);

	return acs_cp_rsp_send(ctx);
}

/**
 * @brief GATT indication confirm callback for CP procedure completion
 *
 * Invoked by the GATT stack when the client confirms a CP indication.
 * Advances any active CP sequence and releases
 * the plain-CP locked flag once the sequence is complete.
 *
 * @param conn      Connection that confirmed the indication.
 * @param attr      CP characteristic attribute.
 * @param err       0 on confirm, negative errno if the indication failed.
 * @param user_data Unused.
 */
static void acs_cp_on_indicate_done(struct bt_conn *conn, const struct bt_gatt_attr *attr, int err,
				    void *user_data)
{
	struct net_buf *rsp_buf = user_data;
	struct bt_acs_conn *acs_conn = acs_conn_lookup(conn);

	/* The seg-TX engine does not own the buffer — free it here. */
	acs_buf_free(rsp_buf);

	__ASSERT_NO_MSG(acs_conn != NULL);

	if (err) {
		struct acs_cp_ctx ctx = {
			.prot_req = NULL,
			.conn = conn,
			.attr = attr,
			.acs_conn = acs_conn,
		};

		LOG_WRN("Plain CP indication failed: %d", err);
		acs_seq_abort(&ctx);
		acs_conn->cp_proc.abort_pending = false;
		atomic_set(&acs_conn->cp_proc.locked, 0);
		return;
	}

	/* Deferred abort: an Abort opcode arrived while this indication was
	 * in-flight.  The TX channel is now free (tx_in_flight cleared before
	 * this callback), so tear down the procedure and send ABORT SUCCESS. */
	if (acs_conn->cp_proc.abort_pending) {
		struct acs_cp_ctx ctx = {
			.prot_req = NULL,
			.conn = conn,
			.attr = attr,
			.acs_conn = acs_conn,
		};
		struct k_work_sync sync;

		acs_conn->cp_proc.abort_pending = false;

		k_work_cancel_sync(&acs_conn->cp_tx.tx_work, &sync);
		acs_seq_clear(&ctx);
		if (acs_conn->cp_proc.response) {
			acs_buf_free(acs_conn->cp_proc.response);
			acs_conn->cp_proc.response = NULL;
		}

		/* Tear down KEX if in progress. */
		if (acs_conn->key_state != BT_ACS_KEY_EXCHANGE_IDLE &&
		    acs_conn->key_state != BT_ACS_KEY_EXCHANGE_COMPLETE) {
			acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
			if (acs_conn->kex) {
				if (acs_conn->kex->ecdh_key_id != 0) {
					psa_destroy_key(acs_conn->kex->ecdh_key_id);
				}
				acs_kex_free(acs_conn->kex);
				acs_conn->kex = NULL;
			}
		}

		/* Drain pending protected-resource requests. */
		acs_prot_resource_req_abort_all(acs_conn);

		LOG_DBG("Deferred abort committed — sending ABORT SUCCESS");
		/* Lock stays held — ABORT now owns it; released on confirm. */
		acs_cp_rsp_status(&ctx, BT_ACS_CP_OPCODE_ABORT, BT_ACS_CP_RESPONSE_SUCCESS);
		return;
	}

	acs_seq_on_cp_confirm(conn, attr);

	if (acs_conn->cp_proc.reply_seq.desc == NULL) {
		atomic_set(&acs_conn->cp_proc.locked, 0);
	}
}

/**
 * @brief Check if this is the first segment of a multi-segment write.
 */
static inline bool is_first_segment(const uint8_t *data)
{
	return (data[0] & ACS_SEG_FIRST_MASK) != 0;
}

ssize_t acs_cp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
		     uint16_t len, uint16_t offset, uint8_t flags)
{
	int ret;
	struct bt_acs_conn *acs_conn;
	enum acs_seg_rx_result seg_rx_result;

	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0) {
		LOG_WRN("CP write: unexpected offset %u", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len < 2) {
		LOG_WRN("CP write: PDU too short (%u bytes)", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (!acs_is_initialized()) {
		LOG_ERR("CP write: ACS not initialized");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* Spec-mandated: reject CP write if CP indications not enabled (ATT CCC_IMPROPER_CONF). */
	ret = acs_cp_ccc_check(conn);

	if (ret == -EINVAL) {
		LOG_WRN("CP write: CP indications not enabled by client");
		return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
	}
	if (ret) {
		LOG_ERR("CP write: CCC check failed (%d)", ret);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	acs_conn = acs_conn_lookup(conn);

	if (!acs_conn) {
		LOG_ERR("CP write: no ACS connection state for conn %p", (void *)conn);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* Allocate buffer from pool on first segment if not already allocated */
	if (is_first_segment(buf) && !acs_conn->cp_rx.buf) {
		struct net_buf *rx_buf = acs_buf_alloc(K_NO_WAIT);

		if (!rx_buf) {
			LOG_ERR("CP: Failed to allocate buffer from pool");
			return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
		}
		acs_seg_rx_begin(&acs_conn->cp_rx, rx_buf);
	}

	if (!acs_conn->cp_rx.buf) {
		LOG_ERR("CP: No buffer for continuation segment");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	seg_rx_result = acs_seg_rx_process(&acs_conn->cp_rx, buf, len);

	switch (seg_rx_result) {
	case ACS_SEG_RX_COMPLETE: {
		struct net_buf_simple simple;
		bool is_abort = false;

#if IS_ENABLED(CONFIG_BT_ACS_ABORT)
		/*
		 * Abort command must preempt any in-progress procedure per §4.4.4: it bypasses the
		 * cp_proc lock and the handler owns the lock semantics itself.
		 */
		is_abort = (acs_conn->cp_rx.buf->len > 0 &&
			    acs_conn->cp_rx.buf->data[0] == BT_ACS_CP_OPCODE_ABORT);
#endif
		if (!is_abort && !atomic_cas(&acs_conn->cp_proc.locked, 0, 1)) {
			acs_seg_rx_reset(&acs_conn->cp_rx);
			LOG_WRN("procedure already in progress");
			return BT_GATT_ERR(BT_ATT_ERR_PROCEDURE_IN_PROGRESS);
		}
		net_buf_simple_init_with_data(&simple, acs_conn->cp_rx.buf->data,
					      acs_conn->cp_rx.buf->len);
		acs_cp_dispatch(NULL, acs_conn, &simple);
		acs_seg_rx_reset(&acs_conn->cp_rx);
		break;
	}
	case ACS_SEG_RX_FRAGMENT:
		break;
	case ACS_SEG_RX_ERR_COUNTER:
		LOG_WRN("CP RX: invalid segment counter (out-of-sequence PDU)");
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return BT_GATT_ERR(BT_ACS_ATT_ERR_INVALID_SEG_COUNTER);
	case ACS_SEG_RX_ERR_OVERFLOW:
		LOG_WRN("CP RX: overflow — accumulated payload exceeds buffer (%zu bytes)",
			acs_conn->cp_rx.buf ? acs_conn->cp_rx.buf->size : 0U);
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	case ACS_SEG_RX_ERR_TIMEOUT:
		LOG_WRN("CP RX: inter-segment timeout expired");
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	case ACS_SEG_RX_ERR_ORPHAN:
		LOG_WRN("CP RX: continuation segment without prior first segment");
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	case ACS_SEG_RX_ERR_LEN:
		LOG_WRN("CP RX: invalid segment length");
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	default:
		LOG_ERR("CP RX: unexpected seg_rx result %d", (int)seg_rx_result);
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return len;
}

/* Dispatch reassembled CP payload: payload[0]=opcode, payload[1..]=operand. */
void acs_cp_dispatch(struct bt_acs_prot_resource_req *prot_req, struct bt_acs_conn *acs_conn,
		     struct net_buf_simple *payload)
{
	struct acs_cp_ctx ctx;
	uint8_t opcode;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->conn != NULL);

	ctx = (struct acs_cp_ctx){
		.prot_req = prot_req,
		.conn = acs_conn->conn,
		.attr = acs_conn->attr_cp,
		.acs_conn = acs_conn,
	};

	opcode = net_buf_simple_pull_u8(payload);
	LOG_DBG("CP dispatch: opcode 0x%02x, operand len %u", opcode, payload->len);

	switch (opcode) {
	case BT_ACS_CP_OPCODE_GET_FEATURE:
		acs_cp_handle_get_feature(&ctx, payload);
		break;

#if IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)
	case BT_ACS_CP_OPCODE_ATT_MTU:
		acs_cp_handle_att_mtu(&ctx);
		break;
#endif /* CONFIG_BT_ACS_ATT_MTU */

#if (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) &&                                          \
     IS_ENABLED(CONFIG_BT_ACS_CCM_NONCE_SEQ_DIFF_FIXED)) ||                                        \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                       \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED:
		acs_cp_handle_set_client_nonce_fixed(&ctx, payload);
		break;
#endif /* CCM with fixed nonce sequence or any GCM/GMAC */

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR:
		acs_cp_handle_get_restriction_map_descriptor(&ctx, payload);
		break;

	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST:
		acs_cp_handle_get_restriction_map_id_list(&ctx);
		break;

	case BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP:
		acs_cp_handle_activate_restriction_map(&ctx, payload);
		break;
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
	case BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR:
		acs_cp_handle_get_key_descriptor(&ctx, payload);
		break;
#endif /* CONFIG_BT_ACS_ANY_KEY_EXCHANGE */

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	case BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR:
		acs_cp_handle_get_isc_descriptor(&ctx, payload);
		break;
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

#if IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP)
	case BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP:
		acs_cp_handle_get_resource_handle_uuid_map(&ctx);
		break;
#endif /* CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP */

	case BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE:
		acs_cp_handle_get_svc_char_uuids(&ctx, payload);
		break;

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION) && IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS)
	case BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS:
		acs_cp_all_active_get(&ctx);
		break;
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION && CONFIG_BT_ACS_DESCRIPTORS */

#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
	case BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST:
		acs_cp_kex_get_current_key_list(&ctx);
		break;

	case BT_ACS_CP_OPCODE_START_KEY_EXCHANGE:
		acs_cp_kex_start(&ctx, payload);
		break;
#endif /* CONFIG_BT_ACS_ANY_KEY_EXCHANGE */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH:
		acs_cp_kex_exchange_ecdh(&ctx, payload);
		break;

	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE:
		acs_cp_kex_ecdh_confirm_code(&ctx, payload);
		break;

	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND:
		acs_cp_kex_ecdh_confirm_rand(&ctx, payload);
		break;
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) || IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF:
		acs_cp_kex_exchange_kdf(&ctx, payload);
		break;
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF || CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

#if IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)
	case BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY:
		acs_sec_mgmt_invalidate_all(&ctx);
		break;

	case BT_ACS_CP_OPCODE_INVALIDATE_KEY:
		acs_sec_mgmt_invalidate_key(&ctx, payload);
		break;
#endif /* CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY */

#if IS_ENABLED(CONFIG_BT_ACS_ABORT)
	case BT_ACS_CP_OPCODE_ABORT:
		acs_sec_mgmt_abort(&ctx);
		break;
#endif /* CONFIG_BT_ACS_ABORT */

#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
	case BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH:
		acs_sec_mgmt_set_security_switch(&ctx, payload);
		break;
#endif /* CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_URI)
	case BT_ACS_CP_OPCODE_GET_KEY_URI:
		acs_sec_mgmt_get_key_uri(&ctx, payload);
		break;
#endif /* CONFIG_BT_ACS_KEY_URI */

#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)
	case BT_ACS_CP_OPCODE_INITIATE_PAIRING:
		acs_sec_mgmt_initiate_pairing(&ctx);
		break;
#endif /* CONFIG_BT_ACS_INITIATE_PAIRING */

	default:
		LOG_WRN("Unsupported opcode: 0x%02x", opcode);
		acs_cp_rsp_status(&ctx, opcode, BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
		break;
	}
}
