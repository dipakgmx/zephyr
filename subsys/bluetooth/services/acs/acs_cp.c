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

/* Definition below; non-static so the data-out channel layer can pass it
 * as the seg-TX completion callback for plain-CP indications.
 */
void acs_cp_on_indicate_done(struct bt_conn *conn, const struct bt_gatt_attr *attr, int err,
			     void *user_data);

int acs_cp_send_reply(acs_procedure *proc)
{
	struct acs_reply_mode mode;
	struct acs_reply reply;
	int err;

	__ASSERT_NO_MSG(proc != NULL);
	__ASSERT_NO_MSG(proc->acs_conn != NULL);
	__ASSERT_NO_MSG(proc->response != NULL);
	__ASSERT_NO_MSG(proc->response->len > 0);

	mode = acs_proc_reply_mode(proc);

	reply = (struct acs_reply){
		.channel = mode.channel,
		.plaintext = proc->response,
		.encrypted = mode.encrypted,
		.needs_confirm = mode.needs_confirm,
	};

	err = acs_tx_submit(proc, &reply);
	if (err) {
		acs_seq_abort(proc);
		if (!proc->is_singleton) {
			LOG_WRN("Protected CP DOI queue failed for handle 0x%04x: %d",
				proc->resource_handle, err);
		} else {
			LOG_WRN("Plain CP response indication failed: %d", err);
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
void acs_cp_on_indicate_done(struct bt_conn *conn, const struct bt_gatt_attr *attr, int err,
			     void *user_data)
{
	struct net_buf *rsp_buf = user_data;
	struct bt_acs_conn *acs_conn = acs_conn_lookup(conn);

	/* The seg-TX engine does not own the buffer — free it here. */
	acs_buf_free(rsp_buf);

	__ASSERT_NO_MSG(acs_conn != NULL);

	if (err) {
		acs_procedure *proc = &acs_conn->plain_cp_proc;

		LOG_WRN("Plain CP indication failed: %d", err);
		acs_seq_abort(proc);
		acs_conn->plain_cp_proc.abort_pending = false;
		atomic_set(&acs_conn->plain_cp_proc.locked, 0);
		return;
	}

	/* Deferred abort: an Abort opcode arrived while this indication was
	 * in-flight.  The TX channel is now free (tx_in_flight cleared before
	 * this callback), so tear down the procedure and send ABORT SUCCESS. */
	if (acs_conn->plain_cp_proc.abort_pending) {
		acs_procedure *proc = &acs_conn->plain_cp_proc;
		struct k_work_sync sync;

		acs_conn->plain_cp_proc.abort_pending = false;

		k_work_cancel_sync(&acs_conn->cp_tx.tx_work, &sync);
		acs_seq_clear(proc);
		if (acs_conn->plain_cp_proc.response) {
			acs_buf_free(acs_conn->plain_cp_proc.response);
			acs_conn->plain_cp_proc.response = NULL;
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
		acs_procedure_abort_all(acs_conn);

		LOG_DBG("Deferred abort committed — sending ABORT SUCCESS");
		/* Lock stays held — ABORT now owns it; released on confirm. */
		acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ABORT, BT_ACS_CP_RESPONSE_SUCCESS);
		return;
	}

	acs_seq_on_confirm(&acs_conn->plain_cp_proc);

	if (acs_conn->plain_cp_proc.reply_seq.desc == NULL) {
		atomic_set(&acs_conn->plain_cp_proc.locked, 0);
	}
}

/* Dispatch reassembled CP payload: frame->payload[0]=opcode, [1..]=operand. */
void acs_cp_dispatch(struct acs_frame *frame, struct bt_acs_conn *acs_conn,
		     struct bt_acs_prot_resource_req *prot_req)
{
	acs_procedure *proc;
	struct net_buf_simple payload_simple;
	struct net_buf_simple *payload = &payload_simple;
	uint8_t opcode;

	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(frame->payload != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->conn != NULL);

	net_buf_simple_init_with_data(&payload_simple, (void *)frame->payload, frame->payload_len);

	proc = prot_req ? prot_req : &acs_conn->plain_cp_proc;

	opcode = net_buf_simple_pull_u8(payload);
	LOG_DBG("CP dispatch: opcode 0x%02x, operand len %u", opcode, payload->len);

	switch (opcode) {
	case BT_ACS_CP_OPCODE_GET_FEATURE:
		acs_cp_handle_get_feature(proc, payload);
		break;

#if IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)
	case BT_ACS_CP_OPCODE_ATT_MTU:
		acs_cp_handle_att_mtu(proc);
		break;
#endif /* CONFIG_BT_ACS_ATT_MTU */

#if (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) &&                                          \
     IS_ENABLED(CONFIG_BT_ACS_CCM_NONCE_SEQ_DIFF_FIXED)) ||                                        \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                       \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED:
		acs_cp_handle_set_client_nonce_fixed(proc, payload);
		break;
#endif /* CCM with fixed nonce sequence or any GCM/GMAC */

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR:
		acs_cp_handle_get_restriction_map_descriptor(proc, payload);
		break;

	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST:
		acs_cp_handle_get_restriction_map_id_list(proc);
		break;

	case BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP:
		acs_cp_handle_activate_restriction_map(proc, payload);
		break;
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
	case BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR:
		acs_cp_handle_get_key_descriptor(proc, payload);
		break;
#endif /* CONFIG_BT_ACS_ANY_KEY_EXCHANGE */

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	case BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR:
		acs_cp_handle_get_isc_descriptor(proc, payload);
		break;
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

#if IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP)
	case BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP:
		acs_cp_handle_get_resource_handle_uuid_map(proc);
		break;
#endif /* CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP */

	case BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE:
		acs_cp_handle_get_svc_char_uuids(proc, payload);
		break;

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION) && IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS)
	case BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS:
		acs_cp_all_active_get(proc);
		break;
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION && CONFIG_BT_ACS_DESCRIPTORS */

#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
	case BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST:
		acs_cp_kex_get_current_key_list(proc);
		break;

	case BT_ACS_CP_OPCODE_START_KEY_EXCHANGE:
		acs_cp_kex_start(proc, payload);
		break;
#endif /* CONFIG_BT_ACS_ANY_KEY_EXCHANGE */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH:
		acs_cp_kex_exchange_ecdh(proc, payload);
		break;

	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE:
		acs_cp_kex_ecdh_confirm_code(proc, payload);
		break;

	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND:
		acs_cp_kex_ecdh_confirm_rand(proc, payload);
		break;
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) || IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF:
		acs_cp_kex_exchange_kdf(proc, payload);
		break;
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF || CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

#if IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)
	case BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY:
		acs_sec_mgmt_invalidate_all(proc);
		break;

	case BT_ACS_CP_OPCODE_INVALIDATE_KEY:
		acs_sec_mgmt_invalidate_key(proc, payload);
		break;
#endif /* CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY */

#if IS_ENABLED(CONFIG_BT_ACS_ABORT)
	case BT_ACS_CP_OPCODE_ABORT:
		acs_sec_mgmt_abort(proc);
		break;
#endif /* CONFIG_BT_ACS_ABORT */

#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
	case BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH:
		acs_sec_mgmt_set_security_switch(proc, payload);
		break;
#endif /* CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_URI)
	case BT_ACS_CP_OPCODE_GET_KEY_URI:
		acs_sec_mgmt_get_key_uri(proc, payload);
		break;
#endif /* CONFIG_BT_ACS_KEY_URI */

#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)
	case BT_ACS_CP_OPCODE_INITIATE_PAIRING:
		acs_sec_mgmt_initiate_pairing(proc);
		break;
#endif /* CONFIG_BT_ACS_INITIATE_PAIRING */

	default:
		LOG_WRN("Unsupported opcode: 0x%02x", opcode);
		acs_cp_rsp_status(proc, opcode, BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
		break;
	}
}
