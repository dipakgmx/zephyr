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

int acs_cp_handle_get_feature(struct acs_procedure *proc, struct net_buf_simple *buf)
{
	struct net_buf *rsp_buf;
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	int err;

	if (buf->len != 0) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_FEATURE,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	LOG_DBG("feat=0x%02x prot=0x%02x", acs_features.features, acs_features.protection_methods);

	rsp_buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);
	if (!rsp_buf) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_GET_FEATURE,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_ACS_FEATURE_RESPONSE);
	net_buf_add_mem(rsp_buf, &acs_features, sizeof(acs_features));

	err = acs_cp_send_reply(proc);
	if (err) {
		LOG_WRN("get_feature: indication arm failed: %d", err);
	}
	return err;
}

#if IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)
int acs_cp_handle_att_mtu(struct acs_procedure *proc)
{
	struct net_buf *rsp_buf;
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	uint16_t mtu;

	rsp_buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);
	if (!rsp_buf) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_ATT_MTU,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	mtu = bt_gatt_get_mtu(proc->acs_conn->conn) - 3U;
	net_buf_add_u8(rsp_buf, BT_ACS_CP_OPCODE_ATT_MTU_RESPONSE);
	net_buf_add_le16(rsp_buf, mtu);
	return acs_cp_send_reply(proc);
}
#endif /* CONFIG_BT_ACS_ATT_MTU */

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
int acs_cp_handle_set_client_nonce_fixed(struct acs_procedure *proc, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;
	struct bt_acs_record_state *record_state;
	const struct bt_acs_key_desc_record *key_desc;
	uint8_t server_nonce_fixed[ACS_NONCE_SIZE];
	uint16_t key_id;
	const uint8_t *nonce_value;
	uint8_t nonce_fixed_size;
	uint8_t this_idx;
	int err;

	if (!acs_conn) {
		LOG_ERR("Request to set client nonce fixed received for unknown connection");
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	/* Operand (§4.4.4.36, Table 4.77): Key_ID(2) + AC_Client_Nonce_Fixed_Value */
	if (buf->len < sizeof(key_id)) {
		LOG_WRN("Set client nonce fixed operand invalid length: %u", buf->len);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	key_id = sys_get_le16(buf->data);
	key_desc = acs_key_desc_lookup(key_id);
	if (!key_desc) {
		LOG_WRN("Set client nonce fixed: unknown Key_ID 0x%04x", key_id);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
					 BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	}

	if (!acs_key_desc_has_nonce_record(key_desc) ||
	    key_desc->aes.nonce_type != ACS_NONCE_SEQ_DIFF_FIXED) {
		LOG_WRN("Set client nonce fixed: Key_ID 0x%04x does not support SEQ_DIFF_FIXED",
			key_id);
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
					 BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	if (acs_kex_in_progress(acs_conn) || acs_session_established(acs_conn)) {
		LOG_WRN("Set client nonce fixed rejected: key exchange already in progress or "
			"established");
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	err = acs_crypto_record_state_lookup(acs_conn, key_id, &record_state);
	if (err) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	nonce_fixed_size = acs_key_desc_nonce_fixed_size(key_desc);
	if (buf->len != (uint16_t)(sizeof(key_id) + nonce_fixed_size)) {
		LOG_WRN("Set client nonce fixed operand invalid length: %u (expected %u)", buf->len,
			(uint16_t)(sizeof(key_id) + nonce_fixed_size));
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	net_buf_simple_pull_le16(buf);
	nonce_value = net_buf_simple_pull_mem(buf, nonce_fixed_size);

	/* §4.4.3.18: reject if equal to any AC_Server_Nonce_Fixed_Value */
	err = acs_crypto_get_server_nonce_fixed(acs_conn, key_id, server_nonce_fixed,
						nonce_fixed_size);
	if (err) {
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
					 BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	if (memcmp(nonce_value, server_nonce_fixed, nonce_fixed_size) == 0) {
		LOG_WRN("Client nonce fixed equals server nonce fixed");
		return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
					 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
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
	this_idx = bt_conn_index(proc->acs_conn->conn);

	for (uint8_t idx = 0; idx < CONFIG_BT_MAX_CONN; idx++) {
		struct bt_acs_conn const *other = acs_conn_by_index(idx);

		if (!other || !other->conn) {
			continue;
		}

		for (size_t rec_idx = 0; rec_idx < ARRAY_SIZE(other->crypto.record_states);
		     rec_idx++) {
			struct bt_acs_record_state const *other_record =
				&other->crypto.record_states[rec_idx];
			uint8_t other_fixed_size;

			if (!other_record->key_desc) {
				continue;
			}

			other_fixed_size = acs_key_desc_nonce_fixed_size(other_record->key_desc);
			if (other_fixed_size != nonce_fixed_size) {
				continue;
			}

			if (memcmp(nonce_value, other_record->server_nonce_fixed,
				   nonce_fixed_size) == 0) {
				LOG_WRN("Client nonce fixed conflicts with server fixed value on "
					"%s connection",
					idx == this_idx ? "current" : "another");
				return acs_cp_rsp_status(proc,
							 BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
							 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
			}

			if (idx == this_idx && !other_record->client_nonce_set) {
				continue;
			}

			if (memcmp(nonce_value, other_record->client_nonce_fixed,
				   nonce_fixed_size) == 0) {
				LOG_WRN("Client nonce fixed conflicts with %s connection",
					idx == this_idx ? "current" : "another");
				return acs_cp_rsp_status(proc,
							 BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
							 BT_ACS_CP_RESPONSE_INVALID_OPERAND);
			}
		}
	}

	memcpy(record_state->client_nonce_fixed, nonce_value, nonce_fixed_size);
	record_state->client_nonce_set = true;

	LOG_DBG("Client nonce fixed part stored");

	return acs_cp_rsp_status(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
				 BT_ACS_CP_RESPONSE_SUCCESS);
}
#endif /* BT_ACS_HAS_NONCE_FIXED */
