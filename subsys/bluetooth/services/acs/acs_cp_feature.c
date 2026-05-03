/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_cp.h"
#include "acs_internal.h"
#include "acs_key_desc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static const struct bt_acs_feature_rsp acs_features = {
	.features = (
		/* Mandatory: Spec requires at least one of these two to be 1 */
		(IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS) ? BT_ACS_FEATURE_DESCRIPTORS_SUPPORTED : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP)
			 ? BT_ACS_FEATURE_RESOURCE_HANDLE_TO_UUID_MAP_SUPPORTED
			 : 0) |

		/* Optional Features */
		(IS_ENABLED(CONFIG_BT_ACS_MULTIPLE_RESTRICTION_MAPS)
			 ? BT_ACS_FEATURE_MULTIPLE_RESTRICTION_MAPS_SUPPORTED
			 : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_ATT_MTU) ? BT_ACS_FEATURE_ATT_MTU_SUPPORTED : 0) |

		/* Security Management Procedures (Table 4.13: C.6, C.8, C.9) */
		(IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)
			 ? BT_ACS_FEATURE_INVALIDATE_ESTABLISHED_SECURITY_SUPPORTED
			 : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
			 ? BT_ACS_FEATURE_SET_SECURITY_CONTROLS_SWITCH_SUPPORTED
			 : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)
			 ? BT_ACS_FEATURE_INITIATION_OF_PAIRING_SUPPORTED
			 : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_KEY_URI) ? BT_ACS_FEATURE_KEY_URI_SUPPORTED : 0) |

		/* Key Exchange Methods */
		(IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
			 ? BT_ACS_FEATURE_OOB_KEY_EXCHANGE_SUPPORTED
			 : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
			 ? BT_ACS_FEATURE_ECDH_KEY_EXCHANGE_SUPPORTED
			 : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
			 ? BT_ACS_FEATURE_KDF_KEY_EXCHANGE_SUPPORTED
			 : 0) |

		/* Resource Protection Types */
		(IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_WRITE)
			 ? BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_WRITE_REQUEST
			 : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_READ)
			 ? BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_READ_REQUEST
			 : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
			 ? BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_NOTIFICATION
			 : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
			 ? BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_INDICATION
			 : 0) |

		/* Key Formats (Applying selection to both Server and Client bits) */
		(IS_ENABLED(CONFIG_BT_ACS_KEY_FORMAT_UNCOMPRESSED)
			 ? (BT_ACS_FEATURE_KEY_FORMAT_AC_SERVER_UNCOMPRESSED_PLAIN_SUPPORTED |
			    BT_ACS_FEATURE_KEY_FORMAT_AC_CLIENT_UNCOMPRESSED_PLAIN_SUPPORTED)
			 : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_KEY_FORMAT_X509)
			 ? (BT_ACS_FEATURE_KEY_FORMAT_AC_SERVER_X509_SUPPORTED |
			    BT_ACS_FEATURE_KEY_FORMAT_AC_CLIENT_X509_SUPPORTED)
			 : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_KEY_FORMAT_MANUFACTURER)
			 ? (BT_ACS_FEATURE_KEY_FORMAT_AC_SERVER_MANUFACTURER_SPECIFIC_SUPPORTED |
			    BT_ACS_FEATURE_KEY_FORMAT_AC_CLIENT_MANUFACTURER_SPECIFIC_SUPPORTED)
			 : 0)),

	.protection_methods =
		((IS_ENABLED(CONFIG_BT_ACS_FEAT_CONFIDENTIALITY)
			  ? BT_ACS_PROTECTION_CONFIDENTIALITY_SUPPORTED
			  : 0) |
		 (IS_ENABLED(CONFIG_BT_ACS_FEAT_INTEGRITY) ? BT_ACS_PROTECTION_INTEGRITY_SUPPORTED
							   : 0) |
		 (IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
			  ? BT_ACS_PROTECTION_AUTHENTICATION_SUPPORTED
			  : 0) |
		 (IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
			  ? BT_ACS_PROTECTION_AUTHORIZATION_SUPPORTED
			  : 0)),

	.oob_key_exchange_capabilities = (IS_ENABLED(CONFIG_BT_ACS_OOB_TRANSPORT_ON_DEVICE)
						  ? BT_ACS_OOB_KEY_EXCHANGE_ON_DEVICE
						  : 0),

	.confirmation_static_oob_number_capabilities =
		((IS_ENABLED(CONFIG_BT_ACS_OOB_STATIC_NUM_NUMBER) ? BT_ACS_OOB_KEY_EXCHANGE_NUMBER
								  : 0) |
		 (IS_ENABLED(CONFIG_BT_ACS_OOB_STATIC_NUM_ON_DEVICE)
			  ? BT_ACS_OOB_KEY_EXCHANGE_ON_DEVICE
			  : 0)),

	/* Numeric confirmation input capabilities */
	.confirmation_input_oob_number_max_value = CONFIG_BT_ACS_CONFIRMATION_INPUT_MAX_VALUE,
	.confirmation_input_oob_number_capabilities =
		((IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_PUSH)
			  ? BT_ACS_CONFIRMATION_INPUT_OOB_PUSH
			  : 0) |
		 (IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_NUMERIC)
			  ? BT_ACS_CONFIRMATION_INPUT_OOB_INPUT_NUMERIC
			  : 0)),

	/* Numeric confirmation output capabilities */
	.confirmation_output_oob_number_max_value = CONFIG_BT_ACS_CONFIRMATION_OUTPUT_MAX_VALUE,
	.confirmation_output_oob_number_capabilities =
		((IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_BEEP)
			  ? BT_ACS_CONFIRMATION_OUTPUT_OOB_BEEP
			  : 0) |
		 (IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC)
			  ? BT_ACS_CONFIRMATION_OUTPUT_OOB_OUTPUT_NUMERIC
			  : 0)),
};

void acs_cp_handle_get_feature(const struct acs_exec_owner *owner, struct net_buf_simple *buf)
{
	struct net_buf *rsp_buf;
	struct acs_reply_mode reply_mode = acs_owner_reply_mode(owner);
	int err;

	if (buf->len != 0) {
		acs_cp_rsp_status(owner, BT_ACS_CP_OPCODE_GET_FEATURE,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	LOG_DBG("get_feature: feat=0x%02x prot=0x%02x", acs_features.features,
		acs_features.protection_methods);

	rsp_buf = acs_prepare_reply_buf(owner, reply_mode.channel, reply_mode.encrypted);
	if (!rsp_buf) {
		acs_cp_rsp_status(owner, BT_ACS_CP_OPCODE_GET_FEATURE,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}
	net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_ACS_FEATURE_RESPONSE);
	net_buf_add_mem(rsp_buf, &acs_features, sizeof(acs_features));

	err = acs_cp_send_reply(owner);
	if (err) {
		LOG_WRN("get_feature: indication arm failed: %d", err);
	}
}

#if IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)
void acs_cp_handle_att_mtu(const struct acs_exec_owner *owner)
{
	struct net_buf *rsp_buf;
	struct acs_reply_mode reply_mode = acs_owner_reply_mode(owner);
	uint16_t mtu;

	rsp_buf = acs_prepare_reply_buf(owner, reply_mode.channel, reply_mode.encrypted);
	if (!rsp_buf) {
		acs_cp_rsp_status(owner, BT_ACS_CP_OPCODE_ATT_MTU,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}
	mtu = bt_gatt_get_mtu(owner->acs_conn->conn) - 3U;
	net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_ATT_MTU_RESPONSE);
	net_buf_add_le16(rsp_buf, mtu);
	acs_cp_send_reply(owner);
}
#endif /* CONFIG_BT_ACS_ATT_MTU */

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
void acs_cp_handle_set_client_nonce_fixed(const struct acs_exec_owner *owner, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = owner->acs_conn;
	struct acs_cp_set_client_nonce_req req_data;
	uint16_t key_id;
	bool key_id_valid;
	const uint8_t *nonce_value;
	uint8_t this_idx;

	if (!acs_conn) {
		LOG_ERR("Request to set client nonce fixed received for unknown connection");
		acs_cp_rsp_status(owner, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		return;
	}

	/* Operand (§4.4.4.36, Table 4.77): Key_ID(2) + AC_Client_Nonce_Fixed_Value */
	if (buf->len != sizeof(struct acs_cp_set_client_nonce_req)) {
		LOG_WRN("Set client nonce fixed operand invalid length: %u", buf->len);
		acs_cp_rsp_status(owner, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	/* Pull all operand data before any response buffer init to avoid aliasing. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));

	key_id = sys_le16_to_cpu(req_data.key_id);

	/* §4.4.4.36.1: Key_ID must reference an AES algorithm record whose
	 * Nonce_Type is SEQ_DIFF_FIXED (Table 4.45). Only cipher key_ids
	 * that use a fixed nonce part are valid here.
	 */
	key_id_valid =
		(IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) && key_id == ACS_KEY_ID_GCM) ||
		(IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) &&
		 IS_ENABLED(CONFIG_BT_ACS_CCM_NONCE_SEQ_DIFF_FIXED) && key_id == ACS_KEY_ID_CCM) ||
		(IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) && key_id == ACS_KEY_ID_GMAC);

	if (!key_id_valid) {
		LOG_WRN("Set client nonce fixed: Key_ID 0x%04X is not a SEQ_DIFF_FIXED cipher",
			key_id);
		acs_cp_rsp_status(owner, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
				  BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
		return;
	}

	/* §4.4.3.18: reject while a key exchange is in progress (state between
	 * STARTED and PENDING_STATUS).  Allow in IDLE (normal case) and in
	 * COMPLETE (restored session — client wants to re-key with a fresh nonce).
	 * TODO: The exchange complete flag needs to be pruned. It is incorrect!!!
	 */
	if (acs_conn->key_state != BT_ACS_KEY_EXCHANGE_IDLE &&
	    acs_conn->key_state != BT_ACS_KEY_EXCHANGE_COMPLETE) {
		LOG_WRN("Set client nonce fixed rejected: key exchange in progress (state %d)",
			acs_conn->key_state);
		acs_cp_rsp_status(owner, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
				  BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		return;
	}

	nonce_value = req_data.nonce_fixed;

	/* §4.4.3.18: reject if equal to any AC_Server_Nonce_Fixed_Value */
	if (memcmp(nonce_value, acs_conn->crypto.server_nonce_fixed,
		   CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE) == 0) {
		LOG_WRN("Client nonce fixed equals server nonce fixed");
		acs_cp_rsp_status(owner, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
				  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		return;
	}

	/* §4.4.3.18: reject if equal to any AC_Client_Nonce_Fixed_Value
	 * already stored on the AC Server.
	 *
	 * For the current connection's own slot: skip if client_nonce_set is
	 * false — the value is inherited from a previous session (RAM
	 * carry-over or NVS restore) and the client is re-establishing it.
	 * For other connections: always check (active nonce regardless of how
	 * it was set).
	 */
	this_idx = bt_conn_index(owner->acs_conn->conn);

	for (uint8_t idx = 0; idx < CONFIG_BT_MAX_CONN; idx++) {
		struct bt_acs_conn const *other = acs_conn_by_index(idx);

		if (!other || !other->conn) {
			continue;
		}
		if (idx == this_idx && !other->crypto.client_nonce_set) {
			continue;
		}
		if (memcmp(nonce_value, other->crypto.client_nonce_fixed,
			   CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE) == 0) {
			LOG_WRN("Client nonce fixed conflicts with %s connection",
				idx == this_idx ? "current" : "another");
			acs_cp_rsp_status(owner, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
					  BT_ACS_CP_RESPONSE_INVALID_OPERAND);
			return;
		}
	}

	memcpy(acs_conn->crypto.client_nonce_fixed, nonce_value,
	       CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
	acs_conn->crypto.client_nonce_set = true;

	LOG_DBG("Client nonce fixed part stored");

	acs_cp_rsp_status(owner, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED, BT_ACS_CP_RESPONSE_SUCCESS);
}
#endif /* BT_ACS_HAS_NONCE_FIXED */
