/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/sys/byteorder.h>

#include "acs_cp_operands.h"
#include "acs_internal.h"
#include "acs_reply.h"
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#include "acs_rmap.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static const struct bt_acs_feature_rsp acs_features = {
	.features = (
		/* Mandatory and authorization-dependent features (Table 4.60). */
		BT_ACS_FEATURE_RESOURCE_HANDLE_TO_UUID_MAP_SUPPORTED |
		BT_ACS_FEATURE_ATT_MTU_SUPPORTED |
		(IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION) ? BT_ACS_FEATURE_DESCRIPTORS_SUPPORTED
							      : 0) |

		/* Security Management Procedures (Table 4.13: C.6, C.8, C.9). */
		(IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
			 ? BT_ACS_FEATURE_INVALIDATE_ESTABLISHED_SECURITY_SUPPORTED
			 : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
			 ? BT_ACS_FEATURE_SET_SECURITY_CONTROLS_SWITCH_SUPPORTED
			 : 0) |
		(IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)
			 ? BT_ACS_FEATURE_INITIATION_OF_PAIRING_SUPPORTED
			 : 0) |
		/* Authentication provides both ECDH and KDF key exchange. */
		(IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
			 ? (BT_ACS_FEATURE_ECDH_KEY_EXCHANGE_SUPPORTED |
			    BT_ACS_FEATURE_KDF_KEY_EXCHANGE_SUPPORTED)
			 : 0) |

		/* Resource Protection Types are added at runtime. */

		/* Only the uncompressed plain public key format is implemented. */
		(BT_ACS_FEATURE_KEY_FORMAT_AC_SERVER_UNCOMPRESSED_PLAIN_SUPPORTED |
		 BT_ACS_FEATURE_KEY_FORMAT_AC_CLIENT_UNCOMPRESSED_PLAIN_SUPPORTED)),

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

	/* Numeric confirmation input capabilities. */
	.confirmation_input_oob_number_max_value = CONFIG_BT_ACS_CONFIRMATION_INPUT_MAX_VALUE,
	.confirmation_input_oob_number_capabilities =
		((IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_PUSH)
			  ? BT_ACS_CONFIRMATION_INPUT_OOB_PUSH
			  : 0) |
		 (IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_NUMERIC)
			  ? BT_ACS_CONFIRMATION_INPUT_OOB_INPUT_NUMERIC
			  : 0)),

	/* Numeric confirmation output capabilities. */
	.confirmation_output_oob_number_max_value = CONFIG_BT_ACS_CONFIRMATION_OUTPUT_MAX_VALUE,
	.confirmation_output_oob_number_capabilities =
		((IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_BEEP)
			  ? BT_ACS_CONFIRMATION_OUTPUT_OOB_BEEP
			  : 0) |
		 (IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC)
			  ? BT_ACS_CONFIRMATION_OUTPUT_OOB_OUTPUT_NUMERIC
			  : 0)),
};

BUILD_ASSERT(sizeof(struct bt_acs_feature_rsp) == 22U,
	     "Feature Response layout changed, update the serialization below");

static uint8_t acs_feature_response[sizeof(struct bt_acs_feature_rsp)];

void acs_cp_feature_init(void)
{
	struct bt_acs_feature_rsp rsp = acs_features;

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	rsp.features |= acs_rmap_protected_resource_feature_bits();

	if (acs_rmap_multiple_supported()) {
		rsp.features |= BT_ACS_FEATURE_MULTIPLE_RESTRICTION_MAPS_SUPPORTED;
	}
#endif

	LOG_DBG("feat=0x%08x prot=0x%04x", rsp.features, rsp.protection_methods);

	sys_put_le32(rsp.features, &acs_feature_response[0]);
	sys_put_le16(rsp.protection_methods, &acs_feature_response[4]);
	sys_put_le16(rsp.oob_key_exchange_capabilities, &acs_feature_response[6]);
	sys_put_le16(rsp.confirmation_static_oob_number_capabilities, &acs_feature_response[8]);
	sys_put_le32(rsp.confirmation_input_oob_number_max_value, &acs_feature_response[10]);
	sys_put_le16(rsp.confirmation_input_oob_number_capabilities, &acs_feature_response[14]);
	sys_put_le32(rsp.confirmation_output_oob_number_max_value, &acs_feature_response[16]);
	sys_put_le16(rsp.confirmation_output_oob_number_capabilities, &acs_feature_response[20]);
}

struct acs_cp_result acs_cp_handle_get_feature(struct acs_reply *reply, struct net_buf_simple *buf)
{
	ARG_UNUSED(buf);

	if (net_buf_tailroom(reply->response) < sizeof(acs_feature_response)) {
		LOG_ERR("no room for the Feature Response");
		return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	net_buf_add_mem(reply->response, acs_feature_response, sizeof(acs_feature_response));

	return acs_cp_reply();
}
