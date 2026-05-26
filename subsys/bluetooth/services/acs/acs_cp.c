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
#include "acs_key_exchange.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Definition below; non-static so the data-out channel layer can pass it
 * as the seg-TX completion callback for plain-CP indications.
 */
void acs_cp_completion_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, int err,
			  void *user_data);

static uint16_t acs_cp_min_operand_size(uint8_t opcode)
{
	switch (opcode) {
#if (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) &&                                          \
     IS_ENABLED(CONFIG_BT_ACS_CCM_NONCE_SEQ_DIFF_FIXED)) ||                                        \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                       \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED:
		return sizeof(uint16_t);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	case BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP:
		return sizeof(uint16_t);
#endif
	case BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE:
		return sizeof(uint16_t);
#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
	case BT_ACS_CP_OPCODE_START_KEY_EXCHANGE:
		return sizeof(struct acs_cp_start_key_exchange_req);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH:
		return sizeof(uint16_t);
	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE:
		return sizeof(struct acs_cp_ecdh_confirm_code_req);
	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND:
		return sizeof(struct acs_cp_ecdh_confirm_rand_req);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) || IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF:
		return sizeof(struct acs_kdf_req);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
	case BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR:
		return sizeof(uint16_t);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	case BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR:
		return sizeof(uint16_t);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR:
		return sizeof(uint16_t);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)
	case BT_ACS_CP_OPCODE_INVALIDATE_KEY:
		return sizeof(uint16_t);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_URI)
	case BT_ACS_CP_OPCODE_GET_KEY_URI:
		return sizeof(uint16_t);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
	case BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH:
		return sizeof(uint8_t);
#endif
	default:
		return 0U;
	}
}

int acs_cp_send_reply(struct acs_procedure *proc)
{
	struct acs_reply_mode mode;
	struct acs_reply reply;
	int err;

	__ASSERT_NO_MSG(proc != NULL);
	__ASSERT_NO_MSG(proc->acs_conn != NULL);
	__ASSERT_NO_MSG(proc->buffers.response_buf != NULL);
	__ASSERT_NO_MSG(proc->buffers.response_buf->len > 0);

	mode = acs_proc_reply_mode(proc);

	reply.channel = mode.channel;
	reply.plaintext = proc->buffers.response_buf;

	err = acs_tx_submit(proc, &reply);
	if (err) {
		acs_seq_abort(proc);
		if (proc->kind == ACS_PROC_KIND_PROTECTED_REQ) {
			LOG_WRN("Protected CP DOI queue failed for handle 0x%04x: %d",
				proc->route.resource_handle, err);
		} else {
			LOG_WRN("Plain CP response indication failed: %d", err);
		}
	}
	return err;
}

int acs_cp_rsp_status(struct acs_procedure *proc, uint8_t req_opcode, uint8_t code)
{
	struct net_buf *buf;
	struct acs_reply reply;
	struct acs_reply_mode mode;
	int err;

	__ASSERT_NO_MSG(proc != NULL);
	__ASSERT_NO_MSG(proc->acs_conn != NULL);

	mode = acs_proc_reply_mode(proc);
	buf = acs_prepare_reply_buf(proc, mode.encrypted);
	if (!buf) {
		acs_seq_abort(proc);
		if (proc->kind == ACS_PROC_KIND_PLAIN_CP) {
			atomic_set(&proc->acs_conn->plain_cp_proc.plain_cp.locked, 0);
		}
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_RESPONSE_CODE);
	net_buf_add_u8(buf, req_opcode);
	net_buf_add_u8(buf, code);

	reply = (struct acs_reply){
		.channel = mode.channel,
		.plaintext = buf,
	};

	err = acs_tx_submit(proc, &reply);
	if (err) {
		/* Status replies are still replies - same failure contract as
		 * acs_cp_send_reply: tear down any active sequence so we don't
		 * leak deferred ALLOC refs or leave stale step state behind.
		 * The plain-CP busy-gate release for submit failure is handled
		 * inside acs_tx_submit_plain_cp itself, so we don't double up.
		 */
		acs_seq_abort(proc);
		if (proc->kind == ACS_PROC_KIND_PROTECTED_REQ) {
			LOG_WRN("Protected CP status indication failed for handle 0x%04x: %d",
				proc->route.resource_handle, err);
		} else {
			LOG_WRN("Plain CP status indication failed: %d", err);
		}
	}

	return err;
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
void acs_cp_completion_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, int err,
			  void *user_data)
{
	struct net_buf *rsp_buf = user_data;
	struct bt_acs_conn *acs_conn = acs_conn_lookup(conn);

	/* The seg-TX engine does not own the buffer - free it here. */
	acs_buf_free(rsp_buf);

	__ASSERT_NO_MSG(acs_conn != NULL);

	if (err) {
		LOG_WRN("Plain CP indication failed: %d", err);
		acs_seq_abort(&acs_conn->plain_cp_proc);
		acs_conn->plain_cp_proc.plain_cp.abort_pending = false;
		atomic_set(&acs_conn->plain_cp_proc.plain_cp.locked, 0);
		return;
	}

	/* Deferred abort: an Abort opcode arrived while this indication was
	 * in-flight.  The TX channel is now free (tx_in_flight cleared before
	 * this callback), so tear down the procedure and send ABORT SUCCESS. */
	if (acs_conn->plain_cp_proc.plain_cp.abort_pending) {
		struct k_work_sync sync;

		acs_conn->plain_cp_proc.plain_cp.abort_pending = false;

		k_work_cancel_sync(&acs_conn->cp_tx.tx_work, &sync);
		acs_seq_clear(&acs_conn->plain_cp_proc);
		if (acs_conn->plain_cp_proc.buffers.response_buf) {
			acs_buf_free(acs_conn->plain_cp_proc.buffers.response_buf);
			acs_conn->plain_cp_proc.buffers.response_buf = NULL;
		}

		/* Tear down KEX if in progress. */
		if (acs_kex_in_progress(acs_conn)) {
			acs_key_exchange_abort(acs_conn);
		}

		/* Drain pending protected-resource requests. */
		acs_procedure_abort_all(acs_conn);

		LOG_DBG("Deferred abort committed - sending ABORT SUCCESS");
		/* Lock stays held - ABORT now owns it; released on confirm. */
		if (acs_cp_rsp_status(&acs_conn->plain_cp_proc, BT_ACS_CP_OPCODE_ABORT,
				      BT_ACS_CP_RESPONSE_SUCCESS)) {
			LOG_ERR("Deferred ABORT response send failed");
			atomic_set(&acs_conn->plain_cp_proc.plain_cp.locked, 0);
		}
		return;
	}

	acs_seq_on_confirm(&acs_conn->plain_cp_proc);

	if (acs_conn->plain_cp_proc.seq_state == ACS_CP_SEQ_IDLE) {
		atomic_set(&acs_conn->plain_cp_proc.plain_cp.locked, 0);
	}
}

/* Dispatch reassembled CP payload: frame->payload[0]=opcode, [1..]=operand. */
int acs_cp_dispatch(const struct acs_frame *frame, struct bt_acs_conn *acs_conn,
		    struct acs_procedure *prot_req)
{
	struct acs_procedure *proc;
	struct net_buf_simple payload_simple;
	struct net_buf_simple *payload = &payload_simple;
	uint8_t opcode;

	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(frame->payload != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->conn != NULL);

	net_buf_simple_init_with_data(&payload_simple, (void *)frame->payload, frame->payload_len);

	/* Protected path passes its own context; plain CP uses the one embedded in acs_conn. */
	proc = (prot_req != NULL) ? prot_req : &acs_conn->plain_cp_proc;

	opcode = net_buf_simple_pull_u8(payload);
	LOG_DBG("CP dispatch: opcode 0x%02x, operand len %u", opcode, payload->len);

	if (payload->len < acs_cp_min_operand_size(opcode)) {
		LOG_WRN("CP dispatch: opcode 0x%02x operand too short (%u)", opcode, payload->len);
		return acs_cp_rsp_status(proc, opcode, BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	switch (opcode) {
	case BT_ACS_CP_OPCODE_GET_FEATURE:
		return acs_cp_handle_get_feature(proc, payload);

#if IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)
	case BT_ACS_CP_OPCODE_ATT_MTU:
		return acs_cp_handle_att_mtu(proc);
#endif /* CONFIG_BT_ACS_ATT_MTU */

#if (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) &&                                          \
     IS_ENABLED(CONFIG_BT_ACS_CCM_NONCE_SEQ_DIFF_FIXED)) ||                                        \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                       \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED:
		return acs_cp_handle_set_client_nonce_fixed(proc, payload);
#endif /* CCM with fixed nonce sequence or any GCM/GMAC */

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR:
		return acs_cp_handle_get_restriction_map_descriptor(proc, payload);

	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST:
		return acs_cp_handle_get_restriction_map_id_list(proc);

	case BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP:
		return acs_cp_handle_activate_restriction_map(proc, payload);
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
	case BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR:
		return acs_cp_handle_get_key_descriptor(proc, payload);
#endif /* CONFIG_BT_ACS_ANY_KEY_EXCHANGE */

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	case BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR:
		return acs_cp_handle_get_isc_descriptor(proc, payload);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

#if IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP)
	case BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP:
		return acs_cp_handle_get_resource_handle_uuid_map(proc);
#endif /* CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP */

	case BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE:
		return acs_cp_handle_get_svc_char_uuids(proc, payload);

#if IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS)
	case BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS:
		return acs_cp_all_active_get(proc);
#endif /* CONFIG_BT_ACS_DESCRIPTORS */

#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
	case BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST:
		return acs_cp_kex_get_current_key_list(proc);

	case BT_ACS_CP_OPCODE_START_KEY_EXCHANGE:
		return acs_cp_kex_start(proc, payload);
#endif /* CONFIG_BT_ACS_ANY_KEY_EXCHANGE */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH:
		return acs_cp_kex_exchange_ecdh(proc, payload);

	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE:
		return acs_cp_kex_ecdh_confirm_code(proc, payload);

	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND:
		return acs_cp_kex_ecdh_confirm_rand(proc, payload);
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) || IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF:
		return acs_cp_kex_exchange_kdf(proc, payload);
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF || CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

#if IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)
	case BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY:
		return acs_sec_mgmt_invalidate_all(proc);

	case BT_ACS_CP_OPCODE_INVALIDATE_KEY:
		return acs_sec_mgmt_invalidate_key(proc, payload);
#endif /* CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY */

#if IS_ENABLED(CONFIG_BT_ACS_ABORT)
	case BT_ACS_CP_OPCODE_ABORT:
		return acs_sec_mgmt_abort(proc);
#endif /* CONFIG_BT_ACS_ABORT */

#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
	case BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH:
		return acs_sec_mgmt_set_security_switch(proc, payload);
#endif /* CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_URI)
	case BT_ACS_CP_OPCODE_GET_KEY_URI:
		return acs_sec_mgmt_get_key_uri(proc, payload);
#endif /* CONFIG_BT_ACS_KEY_URI */

#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)
	case BT_ACS_CP_OPCODE_INITIATE_PAIRING:
		return acs_sec_mgmt_initiate_pairing(proc);
#endif /* CONFIG_BT_ACS_INITIATE_PAIRING */

	default:
		LOG_WRN("Unsupported opcode: 0x%02x", opcode);
		return acs_cp_rsp_status(proc, opcode, BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}
}

void acs_seq_clear(struct acs_procedure *proc)
{
	bool was_active;

	__ASSERT_NO_MSG(proc != NULL);

	was_active = proc->seq_state != ACS_CP_SEQ_IDLE;
	proc->seq_state = ACS_CP_SEQ_IDLE;

	if (was_active && proc->kind == ACS_PROC_KIND_PROTECTED_REQ) {
		acs_procedure_release_owner(proc);
	}
}

void acs_seq_abort(struct acs_procedure *proc)
{
	__ASSERT_NO_MSG(proc != NULL);
	if (proc->seq_state == ACS_CP_SEQ_IDLE) {
		return;
	}

	switch (proc->seq_state) {
	case ACS_CP_SEQ_KEX_SUCCESS_RSP:
	case ACS_CP_SEQ_KEX_SUCCESS_STATUS:
	case ACS_CP_SEQ_KEX_FAIL_RSP:
	case ACS_CP_SEQ_KEX_FAIL_CLEANUP:
		if (proc->acs_conn && acs_kex_in_progress(proc->acs_conn)) {
			acs_key_exchange_abort(proc->acs_conn);
		}
		break;
	default:
		break;
	}

	acs_seq_clear(proc);
}

static int acs_seq_dispatch_step(struct acs_procedure *proc)
{
	switch (proc->seq_state) {
#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
	case ACS_CP_SEQ_KEX_SUCCESS_RSP:
		proc->seq_state = ACS_CP_SEQ_KEX_SUCCESS_STATUS;
		return acs_key_exchange_step_response(proc, 0x00);
	case ACS_CP_SEQ_KEX_SUCCESS_STATUS:
		return acs_key_exchange_step_success_status(proc);
	case ACS_CP_SEQ_KEX_FAIL_RSP:
		proc->seq_state = ACS_CP_SEQ_KEX_FAIL_CLEANUP;
		return acs_key_exchange_step_response(proc, 0x01);
	case ACS_CP_SEQ_KEX_FAIL_CLEANUP:
		acs_key_exchange_abort(proc->acs_conn);
		acs_seq_clear(proc);
		return 0;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS)
	case ACS_CP_SEQ_ALL_ACTIVE_ISC:
		proc->seq_state = ACS_CP_SEQ_ALL_ACTIVE_KEY;
		return acs_all_active_step_isc(proc);
	case ACS_CP_SEQ_ALL_ACTIVE_KEY:
		proc->seq_state = ACS_CP_SEQ_ALL_ACTIVE_RC;
		return acs_all_active_step_key(proc);
	case ACS_CP_SEQ_ALL_ACTIVE_RC:
		return acs_all_active_step_rc(proc);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)
	case ACS_CP_SEQ_INVALIDATE_SELF: {
		struct bt_conn *conn = proc->acs_conn->conn;

		acs_seq_clear(proc);
		bt_acs_invalidate_security(conn);
		return 0;
	}
#endif
	default:
		acs_seq_clear(proc);
		return 0;
	}
}

void acs_seq_on_confirm(struct acs_procedure *proc)
{
	int err;

	__ASSERT_NO_MSG(proc != NULL);
	__ASSERT_NO_MSG(proc->acs_conn != NULL);
	if (proc->seq_state == ACS_CP_SEQ_IDLE) {
		return;
	}

	enum acs_cp_seq_state prev_state = proc->seq_state;

	err = acs_seq_dispatch_step(proc);
	if (err) {
		LOG_WRN("Reply sequence step %u failed: %d", prev_state, err);
		acs_seq_abort(proc);
	}
}
