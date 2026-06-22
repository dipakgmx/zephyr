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
#include "acs_rmap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static int acs_cp_min_operand_size(uint8_t opcode, uint16_t *size)
{
	int err = 0;
	*size = 0U;

	switch (opcode) {
	case BT_ACS_CP_OPCODE_GET_FEATURE:
		break;
#if IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)
	case BT_ACS_CP_OPCODE_ATT_MTU:
		break;
#endif
#if (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) &&                                          \
     IS_ENABLED(CONFIG_BT_ACS_CCM_NONCE_SEQ_DIFF_FIXED)) ||                                        \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                       \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED:
		*size = sizeof(struct acs_cp_set_client_nonce_fixed_req);
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	case BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP:
		*size = sizeof(struct acs_cp_activate_restriction_map_req);
		break;
	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR:
		*size = sizeof(struct acs_rmap_get_descriptor_req);
		break;
	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST:
		break;
#endif
	case BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE:
		*size = sizeof(struct acs_cp_get_svc_char_uuids_req);
		break;
#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
	case BT_ACS_CP_OPCODE_START_KEY_EXCHANGE:
		*size = sizeof(struct acs_cp_start_key_exchange_req);
		break;
	case BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR:
		*size = sizeof(struct acs_cp_get_key_descriptor_req);
		break;
	case BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST:
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH:
		*size = sizeof(struct acs_ecdh_pubkey);
		break;
	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE:
		*size = sizeof(struct acs_cp_ecdh_confirm_code_req);
		break;
	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND:
		*size = sizeof(struct acs_cp_ecdh_confirm_rand_req);
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) || IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF:
		*size = sizeof(struct acs_kdf_req);
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	case BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR:
		*size = sizeof(struct acs_cp_get_isc_descriptor_req);
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP)
	case BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP:
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS)
	case BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS:
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)
	case BT_ACS_CP_OPCODE_INVALIDATE_KEY:
		*size = sizeof(struct acs_cp_invalidate_key_req);
		break;
	case BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY:
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_ABORT)
	case BT_ACS_CP_OPCODE_ABORT:
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
	case BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH:
		*size = sizeof(struct acs_cp_sec_switch_req);
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)
	case BT_ACS_CP_OPCODE_INITIATE_PAIRING:
		break;
#endif
	default:
		err = -ENOENT;
		break;
	}

	return err;
}

int acs_cp_send_reply(struct acs_reply *reply)
{
	int err;

	__ASSERT_NO_MSG(reply != NULL);
	__ASSERT_NO_MSG(reply->conn != NULL);
	__ASSERT_NO_MSG(reply->response != NULL);
	__ASSERT_NO_MSG(reply->response->len > 0);

	err = acs_reply_submit(reply);
	if (err != 0) {
		reply->step = ACS_REPLY_DONE;
		if (reply->channel != ACS_REPLY_CP) {
			LOG_WRN("Protected CP indication failed for handle "
				"0x%04x: %d",
				reply->resource_handle, err);
		} else {
			LOG_WRN("Plain CP indication failed: %d", err);
		}
	}
	return err;
}

int acs_cp_rsp_status(struct acs_reply *reply, uint8_t req_opcode, uint8_t code)
{
	struct net_buf *buf;

	__ASSERT_NO_MSG(reply != NULL);
	__ASSERT_NO_MSG(reply->conn != NULL);

	buf = acs_prepare_reply_buf(reply);
	if (!buf) {
		reply->step = ACS_REPLY_DONE;
		if (reply->channel == ACS_REPLY_CP) {
			atomic_set(&reply->conn->cp_locked, 0);
		}
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_RESPONSE_CODE);
	net_buf_add_u8(buf, req_opcode);
	net_buf_add_u8(buf, code);

	return acs_cp_send_reply(reply);
}

#if IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)
int acs_cp_handle_att_mtu(struct acs_reply *reply)
{
	uint16_t mtu = bt_gatt_get_mtu(reply->conn->conn) - 3U;
	net_buf_add_le16(reply->response, mtu);
	return ACS_CP_RESULT_STAGED_REPLY;
}
#endif /* CONFIG_BT_ACS_ATT_MTU */

static int acs_cp_run_handler(uint8_t opcode, struct acs_reply *reply,
			      struct net_buf_simple *payload)
{
	switch (opcode) {
	case BT_ACS_CP_OPCODE_GET_FEATURE:
		return acs_cp_handle_get_feature(reply, payload);

#if IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)
	case BT_ACS_CP_OPCODE_ATT_MTU:
		return acs_cp_handle_att_mtu(reply);
#endif

#if (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) &&                                          \
     IS_ENABLED(CONFIG_BT_ACS_CCM_NONCE_SEQ_DIFF_FIXED)) ||                                        \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                       \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED:
		return acs_cp_handle_set_client_nonce_fixed(reply, payload);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR:
		return acs_cp_handle_get_restriction_map_descriptor(reply, payload);

	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST:
		return acs_cp_handle_get_restriction_map_id_list(reply);

	case BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP:
		return acs_cp_handle_activate_restriction_map(reply, payload);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
	case BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR:
		return acs_cp_handle_get_key_descriptor(reply, payload);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	case BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR:
		return acs_cp_handle_get_isc_descriptor(reply, payload);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP)
	case BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP:
		return acs_cp_handle_get_resource_handle_uuid_map(reply);
#endif

	case BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE:
		return acs_cp_handle_get_svc_char_uuids(reply, payload);

#if IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS)
	case BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS:
		return acs_cp_all_active_get(reply);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
	case BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST:
		return acs_cp_kex_get_current_key_list(reply);

	case BT_ACS_CP_OPCODE_START_KEY_EXCHANGE:
		return acs_cp_kex_start(reply, payload);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH:
		return acs_cp_kex_exchange_ecdh(reply, payload);

	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE:
		return acs_cp_kex_ecdh_confirm_code(reply, payload);

	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND:
		return acs_cp_kex_ecdh_confirm_rand(reply, payload);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) || IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF:
		return acs_cp_kex_exchange_kdf(reply, payload);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)
	case BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY:
		return acs_sec_mgmt_invalidate_all(reply);

	case BT_ACS_CP_OPCODE_INVALIDATE_KEY:
		return acs_sec_mgmt_invalidate_key(reply, payload);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_ABORT)
	case BT_ACS_CP_OPCODE_ABORT:
		return acs_sec_mgmt_abort(reply);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
	case BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH:
		return acs_sec_mgmt_set_security_switch(reply, payload);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)
	case BT_ACS_CP_OPCODE_INITIATE_PAIRING:
		return acs_sec_mgmt_initiate_pairing(reply);
#endif

	default:
		LOG_WRN("Unsupported opcode: 0x%02x", opcode);
		return BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED;
	}
}

#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
static bool acs_cp_opcode_is_kex(uint8_t opcode)
{
	switch (opcode) {
	case BT_ACS_CP_OPCODE_START_KEY_EXCHANGE:
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH:
	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE:
	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND:
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF:
		return true;
	default:
		return false;
	}
}
#endif

static int acs_cp_stage_response(struct acs_reply *reply, uint8_t opcode)
{
	uint8_t rsp_opcode = 0U;
	struct net_buf *rsp_buf;

	switch (opcode) {
	case BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS:
	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR:
		rsp_opcode = BT_ACS_CP_OPCODE_RESTRICTION_MAP_DESCRIPTOR_RESPONSE;
		break;
	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST:
		rsp_opcode = BT_ACS_CP_OPCODE_RESTRICTION_MAP_ID_LIST_RESPONSE;
		break;
	case BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP:
		rsp_opcode = BT_ACS_CP_OPCODE_RESOURCE_HANDLE_UUID_MAP_RESPONSE;
		break;
	case BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE:
		rsp_opcode =
			BT_ACS_CP_OPCODE_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE_RESPONSE;
		break;
	case BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR:
		rsp_opcode =
			BT_ACS_CP_OPCODE_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR_RESPONSE;
		break;
	case BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR:
		rsp_opcode = BT_ACS_CP_OPCODE_KEY_DESCRIPTOR_RESPONSE;
		break;
	case BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST:
		rsp_opcode = BT_ACS_CP_OPCODE_CURRENT_KEY_LIST_RESPONSE;
		break;
	case BT_ACS_CP_OPCODE_GET_FEATURE:
		rsp_opcode = BT_ACS_CP_OPCODE_ACS_FEATURE_RESPONSE;
		break;
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH:
		rsp_opcode = BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_RESPONSE;
		break;
	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE:
		rsp_opcode = BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_CODE_RESPONSE;
		break;
	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND:
		rsp_opcode = BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_RANDOM_NUMBER_RESPONSE;
		break;
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF:
		rsp_opcode = BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF_RESPONSE;
		break;
	case BT_ACS_CP_OPCODE_ATT_MTU:
		rsp_opcode = BT_ACS_CP_OPCODE_ATT_MTU_RESPONSE;
		break;
	default:
		break;
	}

	if (rsp_opcode == 0U) {
		return 0;
	}

	rsp_buf = acs_prepare_reply_buf(reply);
	if (!rsp_buf) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	net_buf_add_u8(rsp_buf, rsp_opcode);
	return 0;
}

int acs_cp_dispatch(const struct acs_frame *frame, struct bt_acs_conn *acs_conn,
		    struct acs_reply *prot_req)
{
	struct acs_reply *reply;
	struct net_buf_simple payload_simple;
	struct net_buf_simple *payload = &payload_simple;
	uint8_t opcode;
	uint16_t min_operand_size;
	int ret;
	int err;

	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(frame->payload != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->conn != NULL);

	net_buf_simple_init_with_data(&payload_simple, (void *)frame->payload, frame->payload_len);

	if (prot_req != NULL) {
		reply = prot_req;
	} else {
		reply = acs_reply_alloc(acs_conn);
		if (!reply) {
			LOG_ERR("CP reply alloc failed");
			atomic_set(&acs_conn->cp_locked, 0);
			return -ENOMEM;
		}
		reply->channel = ACS_REPLY_CP;
	}

	opcode = net_buf_simple_pull_u8(payload);
	LOG_DBG("CP dispatch: opcode 0x%02x, operand len %u", opcode, payload->len);

	err = acs_cp_min_operand_size(opcode, &min_operand_size);
	if (err == 0 && payload->len < min_operand_size) {
		LOG_WRN("CP dispatch: opcode 0x%02x operand too short (%u)", opcode, payload->len);
		ret = BT_ACS_CP_RESPONSE_INVALID_OPERAND;
#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
		if (acs_cp_opcode_is_kex(opcode) && acs_kex_in_progress(acs_conn)) {
			acs_key_exchange_abort(acs_conn);
		}
#endif
	} else {
		ret = acs_cp_stage_response(reply, opcode);
		if (ret == 0) {
			ret = acs_cp_run_handler(opcode, reply, payload);
		}
#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
		else if (acs_cp_opcode_is_kex(opcode) && acs_kex_in_progress(acs_conn)) {
			acs_key_exchange_abort(acs_conn);
		}
#endif
	}

	if (ret == ACS_CP_RESULT_NO_REPLY) {
		acs_reply_free(reply);
		/* cp_locked is only held by the plain CP path; protected path doesn't acquire it.
		 */
		if (prot_req == NULL) {
			atomic_set(&acs_conn->cp_locked, 0);
		}
		return 0;
	}

	if (ret == ACS_CP_RESULT_STAGED_REPLY) {
		err = acs_cp_send_reply(reply);
	} else {
		err = acs_cp_rsp_status(reply, opcode,
					(ret < 0) ? errno_to_acs_status(ret) : (uint8_t)ret);
	}

	if (err) {
		acs_reply_free(reply);
		if (prot_req == NULL) {
			atomic_set(&acs_conn->cp_locked, 0);
		}
#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
		if (acs_cp_opcode_is_kex(opcode) && acs_kex_in_progress(acs_conn)) {
			acs_key_exchange_abort(acs_conn);
		}
#endif
	}
	return err;
}
