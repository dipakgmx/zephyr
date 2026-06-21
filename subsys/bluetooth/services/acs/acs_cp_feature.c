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
#include "acs_crypto.h"
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

	.oob_key_exchange_capabilities = 0U,

	.confirmation_static_oob_number_capabilities = 0U,

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

int acs_cp_handle_get_feature(struct acs_reply *reply, struct net_buf_simple *buf)
{
	ARG_UNUSED(buf);
	LOG_DBG("feat=0x%08x prot=0x%04x", acs_features.features, acs_features.protection_methods);
	net_buf_add_mem(reply->response, &acs_features, sizeof(acs_features));
	return ACS_CP_RESULT_STAGED_REPLY;
}

#if IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)
int acs_cp_handle_att_mtu(struct acs_reply *reply)
{
	uint16_t mtu = bt_gatt_get_mtu(reply->conn->conn) - 3U;
	net_buf_add_le16(reply->response, mtu);
	return ACS_CP_RESULT_STAGED_REPLY;
}
#endif /* CONFIG_BT_ACS_ATT_MTU */

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
static bool acs_client_nonce_fixed_is_unique(struct bt_acs_conn *acs_conn,
					     const struct bt_acs_key_desc_runtime *runtime,
					     uint8_t fixed_size)
{
	for (size_t i = ACS_KEY_ID_COUNT; i < ACS_KEY_RUNTIME_COUNT; i++) {
		const struct bt_acs_key_desc_runtime *other = &acs_conn->crypto.key_runtimes[i];
		if (!other->key_desc) {
			continue;
		}
		if (memcmp(runtime->client_nonce_fixed, other->server_nonce_fixed, fixed_size) ==
		    0) {
			return false;
		}
		if (other != runtime && other->client_nonce_set &&
		    memcmp(runtime->client_nonce_fixed, other->client_nonce_fixed, fixed_size) ==
			    0) {
			return false;
		}
	}
	return true;
}

int acs_cp_handle_set_client_nonce_fixed(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	const struct bt_acs_key_desc_record *key_desc;
	struct bt_acs_key_desc_runtime *runtime;
	uint16_t key_id;
	uint8_t fixed_size;

	if (!acs_conn) {
		LOG_ERR("Request to set client nonce fixed received for unknown connection");
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}
	key_id = sys_get_le16(buf->data);
	key_desc = acs_key_desc_lookup(key_id);
	if (!key_desc) {
		LOG_WRN("Set client nonce fixed: unknown Key_ID 0x%04x", key_id);
		return BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE;
	}

	if (!acs_key_desc_has_nonce_record(key_desc) ||
	    key_desc->aes.nonce_type != ACS_NONCE_SEQ_DIFF_FIXED) {
		LOG_WRN("Set client nonce fixed: Key_ID 0x%04x does not support SEQ_DIFF_FIXED",
			key_id);
		return BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED;
	}

	/* §4.4.3.18: reject while key exchange is ongoing or already successful. */
	if (acs_kex_in_progress(acs_conn)) {
		LOG_WRN("Set client nonce fixed: key exchange active");
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	}

	if (acs_crypto_key_runtime_lookup(acs_conn, key_id, &runtime) != 0) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	if (runtime->psa_key_id != 0U) {
		LOG_WRN("Set client nonce fixed: Key_ID 0x%04x already has an active key", key_id);
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	}

	fixed_size = acs_key_desc_nonce_fixed_size(key_desc);
	if (buf->len != sizeof(struct acs_cp_set_client_nonce_fixed_req) + fixed_size) {
		LOG_WRN("Set client nonce fixed: size mismatch (got %u, expected %u)", buf->len,
			(unsigned int)(sizeof(struct acs_cp_set_client_nonce_fixed_req) +
				       fixed_size));
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	}

	memcpy(runtime->client_nonce_fixed,
	       buf->data + sizeof(struct acs_cp_set_client_nonce_fixed_req), fixed_size);
	sys_mem_swap(runtime->client_nonce_fixed, fixed_size);

	if (!acs_client_nonce_fixed_is_unique(acs_conn, runtime, fixed_size)) {
		LOG_WRN("Set client nonce fixed: value collides with existing nonce fixed");
		memset(runtime->client_nonce_fixed, 0, sizeof(runtime->client_nonce_fixed));
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	}

	runtime->client_nonce_set = true;
	LOG_DBG("Stored client nonce fixed for Key_ID 0x%04x (%u bytes)", key_id, fixed_size);

	return BT_ACS_CP_RESPONSE_SUCCESS;
}
#endif /* BT_ACS_HAS_NONCE_FIXED */
