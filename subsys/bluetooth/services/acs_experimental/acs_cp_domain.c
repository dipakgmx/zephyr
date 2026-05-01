/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/sys/util.h>

#include "acs_internal.h"
#include "acs_cp_wire.h"
#include "acs_key_desc.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

enum acs_all_active_step {
	ACS_ALL_ACTIVE_STEP_RMAP = 0,
	ACS_ALL_ACTIVE_STEP_ISC = 1,
	ACS_ALL_ACTIVE_STEP_KEY = 2,
	ACS_ALL_ACTIVE_STEP_DONE = 3,
};

struct acs_uuid_map_ctx {
	struct net_buf *buf;
	uint8_t *current_sub_count;
	bool failed;
};

struct acs_svc_char_lookup_ctx {
	uint16_t target_handle;
	const struct bt_uuid *current_svc_uuid;
	const struct bt_uuid *found_svc_uuid;
	const struct bt_uuid *found_char_uuid;
	const struct bt_uuid *char_uuid;
	bool prev_was_chrc;
	bool found;
};

static int acs_cp_all_active_on_confirm(struct acs_procedure *proc);
static uint16_t acs_cp_algorithm_key_id(void);
static int acs_cp_kex_final_on_confirm(struct acs_procedure *proc);
static const struct acs_proc_ops acs_cp_all_active_ops = {
	.start = NULL,
	.on_confirm = acs_cp_all_active_on_confirm,
	.destroy = NULL,
};
static const struct acs_proc_ops acs_cp_kex_final_ops = {
	.start = NULL,
	.on_confirm = acs_cp_kex_final_on_confirm,
	.destroy = NULL,
};

#define ACS_FEATURE_SET_SECURITY_CONTROLS_SWITCH_SUPPORTED            BIT(0)
#define ACS_FEATURE_DESCRIPTORS_SUPPORTED                             BIT(1)
#define ACS_FEATURE_MULTIPLE_RESTRICTION_MAPS_SUPPORTED               BIT(2)
#define ACS_FEATURE_RESOURCE_HANDLE_TO_UUID_MAP_SUPPORTED             BIT(3)
#define ACS_FEATURE_INITIATION_OF_PAIRING_SUPPORTED                   BIT(4)
#define ACS_FEATURE_OOB_KEY_EXCHANGE_SUPPORTED                        BIT(5)
#define ACS_FEATURE_ECDH_KEY_EXCHANGE_SUPPORTED                       BIT(6)
#define ACS_FEATURE_KDF_KEY_EXCHANGE_SUPPORTED                        BIT(7)
#define ACS_FEATURE_KEY_URI_SUPPORTED                                 BIT(8)
#define ACS_FEATURE_INVALIDATE_ESTABLISHED_SECURITY_SUPPORTED         BIT(9)
#define ACS_FEATURE_ATT_MTU_SUPPORTED                                 BIT(10)
#define ACS_FEATURE_PROTECTED_RESOURCE_USES_WRITE_REQUEST             BIT(11)
#define ACS_FEATURE_PROTECTED_RESOURCE_USES_READ_REQUEST              BIT(12)
#define ACS_FEATURE_PROTECTED_RESOURCE_USES_NOTIFICATION              BIT(13)
#define ACS_FEATURE_PROTECTED_RESOURCE_USES_INDICATION                BIT(14)
#define ACS_FEATURE_KEY_FORMAT_AC_SERVER_UNCOMPRESSED_PLAIN_SUPPORTED BIT(17)
#define ACS_FEATURE_KEY_FORMAT_AC_CLIENT_UNCOMPRESSED_PLAIN_SUPPORTED BIT(18)

#define ACS_PROTECTION_CONFIDENTIALITY_SUPPORTED BIT(0)
#define ACS_PROTECTION_INTEGRITY_SUPPORTED       BIT(1)
#define ACS_PROTECTION_AUTHENTICATION_SUPPORTED  BIT(2)
#define ACS_PROTECTION_AUTHORIZATION_SUPPORTED   BIT(3)

#define ACS_OOB_KEY_EXCHANGE_NUMBER    BIT(5)
#define ACS_OOB_KEY_EXCHANGE_ON_DEVICE BIT(15)

#define ACS_CONFIRMATION_INPUT_OOB_PUSH          BIT(0)
#define ACS_CONFIRMATION_INPUT_OOB_INPUT_NUMERIC BIT(2)
#define ACS_CONFIRMATION_OUTPUT_OOB_BEEP         BIT(1)
#define ACS_CONFIRMATION_OUTPUT_OOB_OUTPUT_NUMERIC BIT(3)

static const struct acs_feature_rsp acs_features = {
	.features =
		(IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
			 ? ACS_FEATURE_SET_SECURITY_CONTROLS_SWITCH_SUPPORTED
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS) ? ACS_FEATURE_DESCRIPTORS_SUPPORTED : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_MULTIPLE_RESTRICTION_MAPS)
			 ? ACS_FEATURE_MULTIPLE_RESTRICTION_MAPS_SUPPORTED
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP)
			 ? ACS_FEATURE_RESOURCE_HANDLE_TO_UUID_MAP_SUPPORTED
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)
			 ? ACS_FEATURE_INITIATION_OF_PAIRING_SUPPORTED
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
			 ? ACS_FEATURE_OOB_KEY_EXCHANGE_SUPPORTED
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
			 ? ACS_FEATURE_ECDH_KEY_EXCHANGE_SUPPORTED
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
			 ? ACS_FEATURE_KDF_KEY_EXCHANGE_SUPPORTED
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_KEY_URI) ? ACS_FEATURE_KEY_URI_SUPPORTED : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)
			 ? ACS_FEATURE_INVALIDATE_ESTABLISHED_SECURITY_SUPPORTED
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_ATT_MTU) ? ACS_FEATURE_ATT_MTU_SUPPORTED : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_WRITE)
			 ? ACS_FEATURE_PROTECTED_RESOURCE_USES_WRITE_REQUEST
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_READ)
			 ? ACS_FEATURE_PROTECTED_RESOURCE_USES_READ_REQUEST
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
			 ? ACS_FEATURE_PROTECTED_RESOURCE_USES_NOTIFICATION
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
			 ? ACS_FEATURE_PROTECTED_RESOURCE_USES_INDICATION
			 : 0U) |
		ACS_FEATURE_KEY_FORMAT_AC_SERVER_UNCOMPRESSED_PLAIN_SUPPORTED |
		ACS_FEATURE_KEY_FORMAT_AC_CLIENT_UNCOMPRESSED_PLAIN_SUPPORTED,
	.protection_methods =
		(IS_ENABLED(CONFIG_BT_ACS_FEAT_CONFIDENTIALITY)
			 ? ACS_PROTECTION_CONFIDENTIALITY_SUPPORTED
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_FEAT_INTEGRITY) ? ACS_PROTECTION_INTEGRITY_SUPPORTED : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
			 ? ACS_PROTECTION_AUTHENTICATION_SUPPORTED
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
			 ? ACS_PROTECTION_AUTHORIZATION_SUPPORTED
			 : 0U),
	.oob_key_exchange_capabilities =
		(IS_ENABLED(CONFIG_BT_ACS_OOB_TRANSPORT_ON_DEVICE)
			 ? ACS_OOB_KEY_EXCHANGE_ON_DEVICE
			 : 0U),
	.confirmation_static_oob_number_capabilities =
		(IS_ENABLED(CONFIG_BT_ACS_OOB_STATIC_NUM_NUMBER) ? ACS_OOB_KEY_EXCHANGE_NUMBER : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_OOB_STATIC_NUM_ON_DEVICE)
			 ? ACS_OOB_KEY_EXCHANGE_ON_DEVICE
			 : 0U),
	.confirmation_input_oob_number_max_value = CONFIG_BT_ACS_CONFIRMATION_INPUT_MAX_VALUE,
	.confirmation_input_oob_number_capabilities =
		(IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_PUSH)
			 ? ACS_CONFIRMATION_INPUT_OOB_PUSH
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_NUMERIC)
			 ? ACS_CONFIRMATION_INPUT_OOB_INPUT_NUMERIC
			 : 0U),
	.confirmation_output_oob_number_max_value = CONFIG_BT_ACS_CONFIRMATION_OUTPUT_MAX_VALUE,
	.confirmation_output_oob_number_capabilities =
		(IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_BEEP)
			 ? ACS_CONFIRMATION_OUTPUT_OOB_BEEP
			 : 0U) |
		(IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC)
			 ? ACS_CONFIRMATION_OUTPUT_OOB_OUTPUT_NUMERIC
			 : 0U),
};

static int acs_cp_send_response_code(struct acs_procedure *proc, uint8_t req_opcode,
				     uint8_t response_code)
{
	struct net_buf *buf;
	enum acs_reply_channel channel;
	bool encrypted;

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_RESPONSE_CODE);
	net_buf_add_u8(buf, req_opcode);
	net_buf_add_u8(buf, response_code);

	channel = (proc->flags & ACS_PROC_FLAG_SECURE_TRANSPORT) != 0U ? ACS_REPLY_DOI :
								      ACS_REPLY_CP;
	encrypted = (proc->flags & ACS_PROC_FLAG_SECURE_TRANSPORT) != 0U;

	return acs_data_out_channel_send(proc, &(struct acs_reply){
					 .channel = channel,
					 .plaintext = buf,
					 .encrypted = encrypted,
					 .needs_confirm = true,
				       });
}

int acs_cp_domain_send_response_code(struct acs_procedure *proc, uint8_t req_opcode,
				     uint8_t response_code)
{
	return acs_cp_send_response_code(proc, req_opcode, response_code);
}

static inline bool acs_cp_send_failed(int err)
{
	return err < 0;
}

static uint8_t acs_cp_status_from_errno(int err)
{
	switch (err) {
	case -EINVAL:
		return ACS_CP_RESPONSE_INVALID_OPERAND;
	case -ENOENT:
		return ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE;
	case -EALREADY:
	case -EAGAIN:
		return ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	default:
		return ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}
}

static bool acs_cp_map_requires_secure_transport(const struct bt_acs_restriction_map *map)
{
	return map && map->map_isc_id != BT_ACS_ISC_ID_NONE;
}

static uint8_t acs_cp_append_current_key_ids(struct net_buf *buf,
					     const struct acs_conn_ctx *conn_ctx)
{
	uint8_t count = 0U;

	if (!buf || !conn_ctx) {
		return 0U;
	}

	if (conn_ctx->kex.parent_key_valid) {
		net_buf_add_le16(buf, ACS_KEY_ID_ECDH);
		count++;
	}

	if (conn_ctx->kex.session_key_valid) {
		net_buf_add_le16(buf, ACS_KEY_ID_KDF);
		count++;
	}

	return count;
}

static int acs_cp_send_payload(struct acs_procedure *proc, uint8_t rsp_opcode, struct net_buf *buf)
{
	size_t payload_len;
	enum acs_reply_channel channel;
	bool encrypted;

	if (net_buf_tailroom(buf) < 1U) {
		acs_channel_buf_free(buf);
		return -ENOMEM;
	}

	payload_len = buf->len;
	memmove(buf->data + 1, buf->data, payload_len);
	buf->data[0] = rsp_opcode;
	buf->len = payload_len + 1U;

	channel = (proc->flags & ACS_PROC_FLAG_SECURE_TRANSPORT) != 0U ? ACS_REPLY_DOI :
								      ACS_REPLY_CP;
	encrypted = (proc->flags & ACS_PROC_FLAG_SECURE_TRANSPORT) != 0U;

	return acs_data_out_channel_send(proc, &(struct acs_reply){
					 .channel = channel,
					 .plaintext = buf,
					 .encrypted = encrypted,
					 .needs_confirm = true,
				       });
}

static int acs_cp_send_key_exchange_result(struct acs_procedure *proc, uint16_t key_id,
					   uint8_t result_code)
{
	struct net_buf *buf;
	enum acs_reply_channel channel;
	bool encrypted;

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_RESPONSE);
	net_buf_add_le16(buf, key_id);
	net_buf_add_u8(buf, result_code);

	channel = (proc->flags & ACS_PROC_FLAG_SECURE_TRANSPORT) != 0U ? ACS_REPLY_DOI :
								      ACS_REPLY_CP;
	encrypted = (proc->flags & ACS_PROC_FLAG_SECURE_TRANSPORT) != 0U;

	return acs_data_out_channel_send(proc, &(struct acs_reply){
					 .channel = channel,
					 .plaintext = buf,
					 .encrypted = encrypted,
					 .needs_confirm = true,
				       });
}

static uint16_t acs_cp_active_map_id(struct bt_conn *conn)
{
	struct acs_conn_ctx *conn_ctx = acs_runtime_lookup_conn(conn);

	return conn_ctx ? conn_ctx->active_map_id : 0U;
}

static const struct bt_acs_restriction_map *acs_cp_find_map(uint16_t map_id)
{
	const struct bt_acs_restriction_map *map;

	ARG_UNUSED(map);
	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		if (map->map_id == map_id) {
			return map;
		}
	}

	return NULL;
}

static bool acs_cp_filter_matches(uint16_t filter_handle, uint16_t resource_handle)
{
	return filter_handle == ACS_RMAP_FILTER_ALL || filter_handle == resource_handle;
}

static int acs_cp_append_mapping_record(struct net_buf *buf, uint8_t record_type,
					const struct bt_acs_rmap_protected *entry)
{
	size_t needed;

	if (!entry) {
		return -EINVAL;
	}

	needed = 4U + ((size_t)entry->num_ops * sizeof(struct bt_acs_rmap_op_isc));
	if (net_buf_tailroom(buf) < needed) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, record_type);
	net_buf_add_le16(buf, entry->resource_handle);
	net_buf_add_u8(buf, entry->num_ops * sizeof(struct bt_acs_rmap_op_isc));

	for (uint8_t i = 0U; i < entry->num_ops; i++) {
		net_buf_add_le16(buf, entry->ops[i].opcode);
		net_buf_add_le16(buf, entry->ops[i].isc_id);
	}

	return 0;
}

static int acs_cp_append_rmap_descriptor_records(struct net_buf *buf,
						 const struct bt_acs_restriction_map *map,
						 uint16_t filter_handle, bool *any_records)
{
	struct bt_acs_rmap_char_reg *reg;
	int err;

	ARG_UNUSED(reg);
	if (net_buf_tailroom(buf) < 4U) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, ACS_RMAP_RECORD_RESTRICTION_MAP_ID);
	net_buf_add_le16(buf, map->map_id);
	net_buf_add_u8(buf, 0U);
	*any_records = true;

	if (map->default_isc_id != BT_ACS_ISC_ID_NONE) {
		if (net_buf_tailroom(buf) < 4U) {
			return -ENOMEM;
		}

		net_buf_add_u8(buf, ACS_RMAP_RECORD_DEFAULT_ISC);
		net_buf_add_le16(buf, map->default_isc_id);
		net_buf_add_u8(buf, 0U);
	}

	STRUCT_SECTION_FOREACH(bt_acs_rmap_char_reg, reg) {
		if (reg->map_id != map->map_id || !reg->entry ||
		    !acs_cp_filter_matches(filter_handle, reg->entry->resource_handle)) {
			continue;
		}

		err = acs_cp_append_mapping_record(buf,
						   reg->is_cp ? ACS_RMAP_RECORD_PROTECTED_CP :
								ACS_RMAP_RECORD_PROTECTED_CHARACTERISTIC,
						   reg->entry);
		if (err) {
			return err;
		}

		*any_records = true;
	}

	return 0;
}

static uint8_t acs_cp_security_control_count(void)
{
	uint8_t count = 0U;

	if (IS_ENABLED(CONFIG_BT_ACS_ANY_DATA_PROTECTION) &&
	    !IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)) {
		count++;
	}
	if (IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)) {
		count++;
	}
	if (IS_ENABLED(CONFIG_BT_ACS_FEAT_CONFIDENTIALITY)) {
		count++;
	} else if (IS_ENABLED(CONFIG_BT_ACS_FEAT_INTEGRITY)) {
		count++;
	} else if (!IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)) {
		count++;
	}

	return count;
}

static bool acs_cp_isc_uses_key(void)
{
	return IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION) ||
	       IS_ENABLED(CONFIG_BT_ACS_FEAT_INTEGRITY) ||
	       IS_ENABLED(CONFIG_BT_ACS_FEAT_CONFIDENTIALITY);
}

static int acs_cp_append_isc_descriptor(struct net_buf *buf)
{
	uint8_t *size_ptr;
	uint8_t controls = acs_cp_security_control_count();
	uint16_t key_id = acs_cp_isc_uses_key() ? acs_cp_algorithm_key_id() : 0U;
	size_t needed = 4U + 1U + controls + (key_id ? 2U : 0U);

	if (net_buf_tailroom(buf) < needed) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, ACS_ISC_RECORD_TYPE_ID);
	net_buf_add_le16(buf, BT_ACS_ISC_ID_DEFAULT);
	size_ptr = net_buf_add(buf, 1);
	size_ptr[0] = 0U;
	net_buf_add_u8(buf, controls);
	size_ptr[0]++;

	if (IS_ENABLED(CONFIG_BT_ACS_ANY_DATA_PROTECTION) &&
	    !IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)) {
		net_buf_add_u8(buf, ACS_ISC_CTRL_NONCE);
		size_ptr[0]++;
	}

	if (IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)) {
		net_buf_add_u8(buf, ACS_ISC_CTRL_MAC);
		size_ptr[0]++;
	}

	if (IS_ENABLED(CONFIG_BT_ACS_FEAT_CONFIDENTIALITY)) {
		net_buf_add_u8(buf, ACS_ISC_CTRL_AUTHENTICATED_ENCRYPTED_AD);
		size_ptr[0]++;
	} else if (IS_ENABLED(CONFIG_BT_ACS_FEAT_INTEGRITY)) {
		net_buf_add_u8(buf, ACS_ISC_CTRL_AUTHENTICATED);
		size_ptr[0]++;
	} else if (!IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)) {
		net_buf_add_u8(buf, ACS_ISC_CTRL_UNENCRYPTED);
		size_ptr[0]++;
	}

	if (key_id != 0U) {
		net_buf_add_le16(buf, key_id);
		size_ptr[0] += 2U;
	}

	return 0;
}

static uint8_t acs_cp_kdf_id(void)
{
	if (IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA512) ||
	    IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA512_WITH_INFO)) {
		return ACS_KDF_SHA512;
	}

	if (IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA384) ||
	    IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA384_WITH_INFO)) {
		return ACS_KDF_SHA384;
	}

	return ACS_KDF_SHA256;
}

static uint8_t acs_cp_curve_id(void)
{
	if (IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_P384)) {
		return ACS_CURVE_P384;
	}

	if (IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_P521)) {
		return ACS_CURVE_P512;
	}

	if (IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_CURVE25519)) {
		return ACS_CURVE_CURVE25519;
	}

	return ACS_CURVE_P256;
}

static uint8_t acs_cp_algorithm_type(void)
{
	if (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)) {
		return ACS_KEY_REC_AES_CMAC;
	}

	if (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)) {
		return ACS_KEY_REC_AES_CCM;
	}

	if (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)) {
		return ACS_KEY_REC_AES_GMAC;
	}

	return ACS_KEY_REC_AES_GCM;
}

static uint16_t acs_cp_algorithm_key_id(void)
{
	switch (acs_cp_algorithm_type()) {
	case ACS_KEY_REC_AES_CMAC:
		return ACS_KEY_ID_AES_CMAC;
	case ACS_KEY_REC_AES_CCM:
		return ACS_KEY_ID_AES_CCM;
	case ACS_KEY_REC_AES_EAX:
		return ACS_KEY_ID_AES_EAX;
	case ACS_KEY_REC_AES_GCM:
		return ACS_KEY_ID_AES_GCM;
	case ACS_KEY_REC_AES_GMAC:
	default:
		return ACS_KEY_ID_AES_GMAC;
	}
}

static uint16_t acs_cp_algorithm_parent_key_id(void)
{
	return IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) ? ACS_KEY_ID_KDF : ACS_KEY_ID_ECDH;
}

static bool acs_cp_algorithm_supports_client_fixed_nonce(uint16_t key_id)
{
	if (key_id != acs_cp_algorithm_key_id()) {
		return false;
	}

	if (acs_cp_algorithm_type() == ACS_KEY_REC_AES_CCM) {
		return IS_ENABLED(CONFIG_BT_ACS_CCM_NONCE_SEQ_DIFF_FIXED);
	}

	return IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED);
}

static uint8_t acs_cp_ccm_mac_size(void)
{
#if defined(CONFIG_BT_ACS_CCM_MAC_SIZE)
	return CONFIG_BT_ACS_CCM_MAC_SIZE;
#else
	return 8U;
#endif
}

static int acs_cp_append_key_record_header(struct net_buf *buf, uint8_t type_id, uint16_t key_id,
					   uint8_t data_size)
{
	if (net_buf_tailroom(buf) < (size_t)(4U + data_size)) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, type_id);
	net_buf_add_le16(buf, key_id);
	net_buf_add_u8(buf, data_size);
	return 0;
}

static int acs_cp_append_key_descriptor(struct net_buf *buf, uint16_t filter_id)
{
	const uint8_t kdf_id = acs_cp_kdf_id();
	const uint8_t key_format = IS_ENABLED(CONFIG_BT_ACS_KEY_FORMAT_X509) ?
					 ACS_KEY_FMT_X509 :
					 ACS_KEY_FMT_PLAIN;
	const uint8_t curve_id = acs_cp_curve_id();
	const uint8_t algo_type = acs_cp_algorithm_type();
	const uint16_t algo_key_id = acs_cp_algorithm_key_id();
	const uint16_t algo_parent = acs_cp_algorithm_parent_key_id();
	uint8_t algo_data_size;

	if ((filter_id == ACS_KEY_DESC_FILTER_ALL || filter_id == ACS_KEY_ID_ECDH) &&
	    IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)) {
		if (acs_cp_append_key_record_header(buf, ACS_KEY_REC_ECDH, ACS_KEY_ID_ECDH, 4U) != 0) {
			return -ENOMEM;
		}

		net_buf_add_u8(buf, key_format);
		net_buf_add_u8(buf, key_format);
		net_buf_add_u8(buf, curve_id);
		net_buf_add_u8(buf, kdf_id);
	}

	if ((filter_id == ACS_KEY_DESC_FILTER_ALL || filter_id == ACS_KEY_ID_KDF) &&
	    IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)) {
		if (acs_cp_append_key_record_header(buf, ACS_KEY_REC_KDF, ACS_KEY_ID_KDF, 3U) != 0) {
			return -ENOMEM;
		}

		net_buf_add_le16(buf, ACS_KEY_ID_ECDH);
		net_buf_add_u8(buf, kdf_id);
	}

	if (filter_id != ACS_KEY_DESC_FILTER_ALL && filter_id != algo_key_id) {
		return 0;
	}

	if (algo_type == ACS_KEY_REC_AES_CMAC) {
		if (acs_cp_append_key_record_header(buf, algo_type, algo_key_id, 4U) != 0) {
			return -ENOMEM;
		}

		net_buf_add_le16(buf, algo_parent);
		net_buf_add_u8(buf, ACS_MSG_PROFILE_DEFINED);
		net_buf_add_u8(buf, 16U);
		return 0;
	}

	algo_data_size = 7U + CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE;
	if (acs_cp_append_key_record_header(buf, algo_type, algo_key_id, algo_data_size) !=
	    0) {
		return -ENOMEM;
	}

	net_buf_add_le16(buf, algo_parent);
	net_buf_add_u8(buf, ACS_MSG_PROFILE_DEFINED);
	if (algo_type == ACS_KEY_REC_AES_CCM) {
		net_buf_add_u8(buf, acs_cp_ccm_mac_size());
		net_buf_add_u8(buf, IS_ENABLED(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD) ?
					   ACS_NONCE_SEQ_EVEN_ODD :
					   ACS_NONCE_SEQ_DIFF_FIXED);
		net_buf_add_u8(buf, (uint8_t)(13U - CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE));
	} else {
		net_buf_add_u8(buf, 16U);
		net_buf_add_u8(buf, ACS_NONCE_SEQ_DIFF_FIXED);
		net_buf_add_u8(buf, 8U);
	}
	net_buf_add_u8(buf, CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
	for (int i = 0; i < CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE; i++) {
		net_buf_add_u8(buf, 0U);
	}

	return 0;
}

static bool acs_cp_uuid_to_wire(const struct bt_uuid *uuid, struct net_buf *buf)
{
	if (!uuid || !buf) {
		return false;
	}

	switch (uuid->type) {
	case BT_UUID_TYPE_16: {
		const struct bt_uuid_16 *uuid16 = BT_UUID_16(uuid);

		if (net_buf_tailroom(buf) < 3U) {
			return false;
		}

		net_buf_add_u8(buf, 2U);
		net_buf_add_le16(buf, uuid16->val);
		return true;
	}
	case BT_UUID_TYPE_32: {
		const struct bt_uuid_32 *uuid32 = BT_UUID_32(uuid);

		if (net_buf_tailroom(buf) < 5U) {
			return false;
		}

		net_buf_add_u8(buf, 4U);
		net_buf_add_le32(buf, uuid32->val);
		return true;
	}
	case BT_UUID_TYPE_128: {
		const struct bt_uuid_128 *uuid128 = BT_UUID_128(uuid);

		if (net_buf_tailroom(buf) < 17U) {
			return false;
		}

		net_buf_add_u8(buf, 16U);
		net_buf_add_mem(buf, uuid128->val, 16U);
		return true;
	}
	default:
		return false;
	}
}

static uint8_t acs_cp_uuid_map_walk_cb(const struct bt_gatt_attr *attr, uint16_t handle,
				       void *user_data)
{
	struct acs_uuid_map_ctx *ctx = user_data;

	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY)) {
		const struct bt_uuid *svc_uuid = attr->user_data;

		if (net_buf_tailroom(ctx->buf) < 4U) {
			ctx->failed = true;
			return BT_GATT_ITER_STOP;
		}

			net_buf_add_u8(ctx->buf, ACS_RHANDLE_ATTR_PRIMARY_SVC);
		net_buf_add_le16(ctx->buf, handle);
		if (!acs_cp_uuid_to_wire(svc_uuid, ctx->buf)) {
			ctx->failed = true;
			return BT_GATT_ITER_STOP;
		}

		ctx->current_sub_count = net_buf_add(ctx->buf, 1);
		ctx->current_sub_count[0] = 0U;
		return BT_GATT_ITER_CONTINUE;
	}

	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC) && ctx->current_sub_count) {
		const struct bt_gatt_chrc *chrc = attr->user_data;
		uint16_t value_handle;

		if (net_buf_tailroom(ctx->buf) < 4U) {
			ctx->failed = true;
			return BT_GATT_ITER_STOP;
		}

		value_handle = chrc->value_handle ? chrc->value_handle : (uint16_t)(handle + 1U);

		net_buf_add_u8(ctx->buf, 0x02);
		net_buf_add_le16(ctx->buf, value_handle);
		if (!acs_cp_uuid_to_wire(chrc->uuid, ctx->buf)) {
			ctx->failed = true;
			return BT_GATT_ITER_STOP;
		}

		ctx->current_sub_count[0]++;
	}

	return BT_GATT_ITER_CONTINUE;
}

static int acs_cp_append_resource_handle_uuid_map(struct net_buf *buf)
{
	struct acs_uuid_map_ctx ctx = {
		.buf = buf,
		.current_sub_count = NULL,
		.failed = false,
	};

	bt_gatt_foreach_attr(1U, BT_ATT_LAST_ATTRIBUTE_HANDLE, acs_cp_uuid_map_walk_cb, &ctx);

	if (ctx.failed) {
		return -ENOMEM;
	}

	return buf->len > 0U ? 0 : -ENOENT;
}

static uint8_t acs_cp_lookup_svc_char_cb(const struct bt_gatt_attr *attr, uint16_t handle,
					 void *user_data)
{
	struct acs_svc_char_lookup_ctx *ctx = user_data;

	if (ctx->found) {
		return BT_GATT_ITER_STOP;
	}

	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_PRIMARY) ||
	    !bt_uuid_cmp(attr->uuid, BT_UUID_GATT_SECONDARY)) {
		ctx->current_svc_uuid = attr->user_data;
		ctx->prev_was_chrc = false;
		return BT_GATT_ITER_CONTINUE;
	}

	if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CHRC)) {
		const struct bt_gatt_chrc *chrc = attr->user_data;

		ctx->prev_was_chrc = true;
		ctx->char_uuid = chrc ? chrc->uuid : NULL;
		return BT_GATT_ITER_CONTINUE;
	}

	if (ctx->prev_was_chrc && handle == ctx->target_handle) {
		ctx->found = true;
		ctx->found_svc_uuid = ctx->current_svc_uuid;
		ctx->found_char_uuid = ctx->char_uuid;
		return BT_GATT_ITER_STOP;
	}

	ctx->prev_was_chrc = false;
	return BT_GATT_ITER_CONTINUE;
}

static int acs_cp_append_service_char_uuids(struct net_buf *buf, uint16_t resource_handle)
{
	struct acs_svc_char_lookup_ctx ctx = {
		.target_handle = resource_handle,
	};

	bt_gatt_foreach_attr(1U, BT_ATT_LAST_ATTRIBUTE_HANDLE, acs_cp_lookup_svc_char_cb, &ctx);
	if (!ctx.found || !ctx.found_svc_uuid || !ctx.found_char_uuid) {
		return -ENOENT;
	}

	if (!acs_cp_uuid_to_wire(ctx.found_svc_uuid, buf)) {
		return -ENOMEM;
	}

	if (!acs_cp_uuid_to_wire(ctx.found_char_uuid, buf)) {
		return -ENOMEM;
	}

	return 0;
}

static int acs_cp_send_rmap_descriptor_response(struct acs_procedure *proc, uint16_t map_id,
						uint16_t filter_handle)
{
	const struct bt_acs_restriction_map *map = acs_cp_find_map(map_id);
	struct net_buf *buf;
	bool any_records = false;
	int err;

	if (!map) {
		return -ENOENT;
	}

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return -ENOMEM;
	}

	err = acs_cp_append_rmap_descriptor_records(buf, map, filter_handle, &any_records);
	if (err || !any_records) {
		acs_channel_buf_free(buf);
		return err ? err : -ENOENT;
	}

	return acs_cp_send_payload(proc, BT_ACS_CP_OPCODE_RESTRICTION_MAP_DESCRIPTOR_RESPONSE,
				   buf);
}

static int acs_cp_send_isc_descriptor_response(struct acs_procedure *proc, uint16_t filter_id)
{
	struct net_buf *buf = acs_channel_buf_alloc();

	if (filter_id != ACS_ISC_ALL_RECORDS_FILTER && filter_id != BT_ACS_ISC_ID_DEFAULT) {
		return -ENOENT;
	}

	if (!buf) {
		return -ENOMEM;
	}

	if (acs_cp_append_isc_descriptor(buf) != 0) {
		acs_channel_buf_free(buf);
		return -ENOMEM;
	}

	return acs_cp_send_payload(
		proc, BT_ACS_CP_OPCODE_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR_RESPONSE, buf);
}

static int acs_cp_send_key_descriptor_response(struct acs_procedure *proc, uint16_t filter_id)
{
	struct net_buf *buf = acs_channel_buf_alloc();
	bool empty;

	if (!buf) {
		return -ENOMEM;
	}

	if (acs_cp_append_key_descriptor(buf, filter_id) != 0) {
		acs_channel_buf_free(buf);
		return -ENOMEM;
	}

	empty = (buf->len == 0U);
	if (empty) {
		acs_channel_buf_free(buf);
		return -ENOENT;
	}

	return acs_cp_send_payload(proc, BT_ACS_CP_OPCODE_KEY_DESCRIPTOR_RESPONSE, buf);
}

static int acs_cp_all_active_on_confirm(struct acs_procedure *proc)
{
	uint16_t map_id = acs_cp_active_map_id(proc->conn);

	switch (proc->step) {
	case ACS_ALL_ACTIVE_STEP_RMAP:
		proc->step = ACS_ALL_ACTIVE_STEP_ISC;
		return acs_cp_send_isc_descriptor_response(proc, ACS_ISC_ALL_RECORDS_FILTER);
	case ACS_ALL_ACTIVE_STEP_ISC:
		proc->step = ACS_ALL_ACTIVE_STEP_KEY;
		return acs_cp_send_key_descriptor_response(proc, ACS_KEY_DESC_FILTER_ALL);
	case ACS_ALL_ACTIVE_STEP_KEY:
		proc->step = ACS_ALL_ACTIVE_STEP_DONE;
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
						 ACS_CP_RESPONSE_SUCCESS);
		case ACS_ALL_ACTIVE_STEP_DONE:
			LOG_DBG("ACS CP GET_ALL_ACTIVE_DESCRIPTORS complete for map 0x%04x", map_id);
			return ACS_PROC_RES_COMPLETE;
	default:
		return -EINVAL;
	}
}

static int acs_cp_kex_final_on_confirm(struct acs_procedure *proc)
{
	uint16_t key_id;

	if (proc->state_kind != ACS_PROC_STATE_KEX_FINAL) {
		return -EINVAL;
	}

	key_id = proc->state.key_id;

	switch (proc->step) {
	case 0:
		proc->step = 1;
		return acs_cp_send_key_exchange_result(proc, key_id,
						       ACS_KEY_EXCHANGE_RESULT_SUCCESSFUL);
		case 1:
			return ACS_PROC_RES_COMPLETE;
	default:
		return -EINVAL;
	}
}

static int acs_cp_handle_get_feature(struct acs_procedure *proc, const struct acs_frame *frame)
{
	struct net_buf *buf;

	if (frame->payload_len != 1U) {
		LOG_WRN("ACS CP GET_FEATURE invalid operand len=%u", frame->payload_len);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_FEATURE,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_FEATURE,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_ACS_FEATURE_RESPONSE);
	net_buf_add_mem(buf, &acs_features, sizeof(acs_features));

	return acs_data_out_channel_send(proc, &(struct acs_reply){
					 .channel = (proc->flags & ACS_PROC_FLAG_SECURE_TRANSPORT) != 0U ?
							ACS_REPLY_DOI :
							ACS_REPLY_CP,
					 .plaintext = buf,
					 .encrypted = (proc->flags & ACS_PROC_FLAG_SECURE_TRANSPORT) != 0U,
					 .needs_confirm = true,
				       });
}

static int acs_cp_handle_get_restriction_map_id_list(struct acs_procedure *proc,
						     const struct acs_frame *frame)
{
	const struct bt_acs_restriction_map *map;
	struct net_buf *buf;
	bool found = false;

	if (!IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS) ||
	    !IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST,
						 ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	ARG_UNUSED(map);
	if (frame->payload_len != 1U) {
		LOG_WRN("ACS CP GET_RESTRICTION_MAP_ID_LIST invalid operand len=%u",
			frame->payload_len);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, map) {
		if (net_buf_tailroom(buf) < 4U) {
			acs_channel_buf_free(buf);
			return acs_cp_send_response_code(
				proc, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST,
				ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}

		net_buf_add_le16(buf, map->map_id);
		net_buf_add_le16(buf, map->map_isc_id);
		found = true;
	}

	if (!found) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST,
						 ACS_CP_RESPONSE_NO_RECORDS_FOUND);
	}

	return acs_cp_send_payload(proc, BT_ACS_CP_OPCODE_RESTRICTION_MAP_ID_LIST_RESPONSE, buf);
}

static int acs_cp_handle_get_restriction_map_descriptor(struct acs_procedure *proc,
							const struct acs_frame *frame)
{
	const struct acs_cp_map_filter_req *req;
	const struct bt_acs_restriction_map *map;
	struct net_buf *buf;
	uint16_t map_id;
	uint16_t filter_handle;
	bool any_records = false;
	int err;

	if (!IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS) ||
	    !IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)) {
		return acs_cp_send_response_code(
			proc, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR,
			ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	if (frame->payload_len != sizeof(*req)) {
		LOG_WRN("invalid operand len=%u",
			frame->payload_len);
		return acs_cp_send_response_code(proc,
						 BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	req = (const void *)frame->payload;
	map_id = acs_cp_wire_get_le16(req->map_id_le);
	filter_handle = acs_cp_wire_get_le16(req->filter_handle_le);
	map = acs_cp_find_map(map_id);
	if (!map) {
		LOG_WRN("unknown map_id=0x%04x", map_id);
		return acs_cp_send_response_code(
			proc, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR,
			ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	}

	if (acs_cp_map_requires_secure_transport(map) &&
	    (proc->flags & ACS_PROC_FLAG_SECURE_TRANSPORT) == 0U) {
		LOG_WRN("ACS CP GET_RESTRICTION_MAP_DESCRIPTOR requires secure transport for map_id=0x%04x",
			map_id);
		return acs_cp_send_response_code(
			proc, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR,
			ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return acs_cp_send_response_code(proc,
						 BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	err = acs_cp_append_rmap_descriptor_records(buf, map, filter_handle, &any_records);
	if (err == -ENOMEM) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc,
						 BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	if (err) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc,
						 BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	if (!any_records) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc,
						 BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR,
						 ACS_CP_RESPONSE_NO_RECORDS_FOUND);
	}

	return acs_cp_send_payload(proc, BT_ACS_CP_OPCODE_RESTRICTION_MAP_DESCRIPTOR_RESPONSE,
				   buf);
}

static int acs_cp_handle_activate_restriction_map(struct acs_procedure *proc,
						  const struct acs_frame *frame)
{
	const struct acs_cp_map_id_req *req;
	uint16_t map_id;
	int err;

	if (!IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP,
						 ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	if (frame->payload_len != sizeof(*req)) {
		LOG_WRN("ACS CP ACTIVATE_RESTRICTION_MAP invalid operand len=%u", frame->payload_len);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	req = (const void *)frame->payload;
	map_id = acs_cp_wire_get_le16(req->map_id_le);
	{
		const struct bt_acs_restriction_map *map = acs_cp_find_map(map_id);

		if (!map) {
			LOG_WRN("ACS CP ACTIVATE_RESTRICTION_MAP unknown map_id=0x%04x", map_id);
			return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP,
							 ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
		}

		if (acs_cp_map_requires_secure_transport(map) &&
		    (proc->flags & ACS_PROC_FLAG_SECURE_TRANSPORT) == 0U) {
			LOG_WRN("ACS CP ACTIVATE_RESTRICTION_MAP requires secure transport for map_id=0x%04x",
				map_id);
			return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP,
							 ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		}
	}

	err = bt_acs_set_restriction_map(proc->conn, map_id);
	if (err) {
		LOG_WRN("ACS CP ACTIVATE_RESTRICTION_MAP failed: %d", err);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	LOG_WRN("ACS CP activated restriction map 0x%04x", map_id);
	return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP,
					 ACS_CP_RESPONSE_SUCCESS);
}

static int acs_cp_handle_get_information_security_configuration_descriptor(
	struct acs_procedure *proc, const struct acs_frame *frame)
{
	const struct acs_cp_isc_filter_req *req;
	uint16_t filter_id;
	int err;

	if (!IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS)) {
		return acs_cp_send_response_code(
			proc, BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR,
			ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	if (frame->payload_len != sizeof(*req)) {
		LOG_WRN("ACS CP GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR invalid operand len=%u",
			frame->payload_len);
		return acs_cp_send_response_code(
			proc, BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR,
			ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	req = (const void *)frame->payload;
	filter_id = acs_cp_wire_get_le16(req->isc_id_filter_le);
	err = acs_cp_send_isc_descriptor_response(proc, filter_id);
	if (err == -ENOENT) {
		LOG_WRN("ACS CP GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR no records for filter=0x%04x",
			filter_id);
		return acs_cp_send_response_code(
			proc, BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR,
			ACS_CP_RESPONSE_NO_RECORDS_FOUND);
	}
	if (err == -ENOMEM) {
		return acs_cp_send_response_code(
			proc, BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR,
			ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	return err;
}

static int acs_cp_handle_get_key_descriptor(struct acs_procedure *proc,
					    const struct acs_frame *frame)
{
	const struct acs_cp_key_id_req *req;
	uint16_t filter_id;
	int err;

	if (frame->payload_len != sizeof(*req)) {
		LOG_WRN("ACS CP GET_KEY_DESCRIPTOR invalid operand len=%u", frame->payload_len);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	req = (const void *)frame->payload;
	filter_id = acs_cp_wire_get_le16(req->key_id_le);
	err = acs_cp_send_key_descriptor_response(proc, filter_id);
	if (err == -ENOMEM) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}
	if (acs_cp_send_failed(err)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR,
						 ACS_CP_RESPONSE_NO_RECORDS_FOUND);
	}

	return err;
}

static int acs_cp_handle_get_current_key_list(struct acs_procedure *proc,
					      const struct acs_frame *frame)
{
	struct acs_conn_ctx *conn_ctx;
	struct net_buf *buf;
	uint8_t key_count;

	if (frame->payload_len != 1U) {
		LOG_WRN("ACS CP GET_CURRENT_KEY_LIST invalid operand len=%u", frame->payload_len);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	conn_ctx = acs_runtime_lookup_conn(proc->conn);
	if (!conn_ctx) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	net_buf_add_u8(buf, 0U);
	if (net_buf_tailroom(buf) < (2U * sizeof(uint16_t))) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	key_count = acs_cp_append_current_key_ids(buf, conn_ctx);
	buf->data[0] = key_count;

	return acs_cp_send_payload(proc, BT_ACS_CP_OPCODE_CURRENT_KEY_LIST_RESPONSE, buf);
}

static int acs_cp_handle_start_key_exchange(struct acs_procedure *proc,
					    const struct acs_frame *frame)
{
	struct acs_conn_ctx *conn_ctx;
	const struct acs_cp_start_key_exchange_req *req;
	int err;

	if (frame->payload_len != sizeof(*req)) {
		LOG_WRN("ACS CP START_KEY_EXCHANGE invalid operand len=%u", frame->payload_len);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	conn_ctx = acs_runtime_lookup_conn(proc->conn);
	if (!conn_ctx) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	req = (const void *)frame->payload;
	err = acs_kex_start(conn_ctx, acs_cp_wire_get_le16(req->key_id_le),
			    req->confirmation_method, req->confirmation_action);
	if (err) {
		LOG_WRN("ACS CP START_KEY_EXCHANGE rejected: %d", err);
		if (err == -ENOENT || err == -EALREADY || err == -EAGAIN) {
			return acs_cp_send_response_code(
				proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
				ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
		}
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
						 acs_cp_status_from_errno(err));
	}

	return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE,
					 ACS_CP_RESPONSE_SUCCESS);
}

static int acs_cp_mark_security_established(struct acs_conn_ctx *conn_ctx, const uint8_t *session_key,
					    size_t key_len)
{
	const struct bt_acs_cb *cb = acs_runtime_callbacks();
#if CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE > 0
	uint8_t server_nonce_fixed[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE] = { 0 };
#else
	uint8_t server_nonce_fixed[1] = { 0 };
#endif
	size_t nonce_fixed_len = IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED) ?
					 CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE :
					 0U;
	int err;

	err = acs_crypto_set_server_nonce_fixed(&conn_ctx->crypto, server_nonce_fixed, nonce_fixed_len);
	if (err) {
		return err;
	}

	err = acs_crypto_session_install(&conn_ctx->crypto, acs_runtime_crypto_mode_default(),
					 BT_ACS_ISC_ID_DEFAULT, acs_cp_algorithm_key_id(),
					 session_key, key_len);
	if (err) {
		return err;
	}

	conn_ctx->status_flags |= BT_ACS_STATUS_SECURITY_ESTABLISHED;
	(void)acs_persist_save_conn(conn_ctx);
	if (cb && cb->security_established) {
		cb->security_established(conn_ctx->conn, session_key, key_len);
	}

	return 0;
}

static int acs_cp_handle_key_exchange_ecdh(struct acs_procedure *proc, const struct acs_frame *frame)
{
	struct acs_conn_ctx *conn_ctx = acs_runtime_lookup_conn(proc->conn);
	struct net_buf *buf;
	int err;

	if (!conn_ctx || conn_ctx->kex.state != ACS_KEX_STARTED) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	err = acs_kex_build_ecdh_response(conn_ctx, ACS_KEY_ID_ECDH, &frame->payload[1],
					  frame->payload_len - 1U, buf);
	if (err == ACS_KEX_ERR_INVALID_PUBLIC_KEY) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 ACS_CP_RESPONSE_INVALID_PUBLIC_KEY);
	}
	if (err == -EINVAL) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}
	if (err == -ENOENT || err == -EALREADY) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}
	if (err) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	return acs_cp_send_payload(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_RESPONSE, buf);
}

static int acs_cp_handle_key_exchange_kdf(struct acs_procedure *proc, const struct acs_frame *frame)
{
	const struct acs_cp_key_id_req *req;
	struct acs_conn_ctx *conn_ctx = acs_runtime_lookup_conn(proc->conn);
	struct net_buf *buf;
	uint16_t key_id;
	bool send_final = false;
	int err;

	if (frame->payload_len != sizeof(*req)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	if (!conn_ctx || (conn_ctx->kex.state != ACS_KEX_STARTED &&
			  conn_ctx->kex.state != ACS_KEX_PUBKEY_EXCHANGED &&
			  conn_ctx->kex.state != ACS_KEX_COMPLETE &&
			  conn_ctx->kex.state != ACS_KEX_KDF_DONE)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	req = (const void *)frame->payload;
	key_id = acs_cp_wire_get_le16(req->key_id_le);
	buf = acs_channel_buf_alloc();
	if (!buf) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	err = acs_kex_build_kdf_response(conn_ctx, key_id, buf, &send_final);
	if (err == -EINVAL) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}
	if (err == -ENOENT || err == -EALREADY || err == -EAGAIN) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}
	if (err) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	if (send_final) {
		err = acs_cp_mark_security_established(conn_ctx, conn_ctx->kex.session_key,
						       sizeof(conn_ctx->kex.session_key));
		if (err) {
			acs_channel_buf_free(buf);
			return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF,
							 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
		proc->ops = &acs_cp_kex_final_ops;
		proc->state_kind = ACS_PROC_STATE_KEX_FINAL;
		proc->state.key_id = key_id;
		proc->step = 0U;
	}

	return acs_cp_send_payload(proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF_RESPONSE, buf);
}

static int acs_cp_handle_ecdh_confirm_code(struct acs_procedure *proc, const struct acs_frame *frame)
{
	struct acs_conn_ctx *conn_ctx = acs_runtime_lookup_conn(proc->conn);
	struct net_buf *buf;
	int err;

	if (!conn_ctx || conn_ctx->kex.state != ACS_KEX_KDF_DONE) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	err = acs_kex_build_confirmation_code_response(conn_ctx, ACS_KEY_ID_ECDH,
						       &frame->payload[1], frame->payload_len - 1U,
						       buf);
	if (err == -EINVAL) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}
	if (err == -ENOENT || err == -EALREADY) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}
	if (err) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	return acs_cp_send_payload(proc,
				   BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_CODE_RESPONSE,
				   buf);
}

static int acs_cp_handle_ecdh_confirm_rand(struct acs_procedure *proc, const struct acs_frame *frame)
{
	struct acs_conn_ctx *conn_ctx = acs_runtime_lookup_conn(proc->conn);
	struct net_buf *buf;
	bool send_final = false;
	int err;

	if (!conn_ctx || conn_ctx->kex.state != ACS_KEX_KDF_DONE) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	err = acs_kex_build_confirmation_random_response(conn_ctx, ACS_KEY_ID_ECDH,
							 &frame->payload[1],
							 frame->payload_len - 1U, buf,
							 &send_final);
	if (err == -EINVAL) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}
	if (err == -ENOENT || err == -EALREADY) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}
	if (err == -EACCES) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(
			proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
			ACS_CP_RESPONSE_INVALID_KEY_EXCHANGE_CONFIRMATION_CODE);
	}
	if (err) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	if (send_final && acs_cp_algorithm_parent_key_id() == ACS_KEY_ID_ECDH) {
		err = acs_cp_mark_security_established(conn_ctx, conn_ctx->kex.parent_key,
						       sizeof(conn_ctx->kex.parent_key));
		if (err) {
			acs_channel_buf_free(buf);
			return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND,
							 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
	}

	if (send_final) {
		proc->ops = &acs_cp_kex_final_ops;
		proc->state_kind = ACS_PROC_STATE_KEX_FINAL;
		proc->state.key_id = ACS_KEY_ID_ECDH;
		proc->step = 0U;
	}

	return acs_cp_send_payload(
		proc, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_RANDOM_NUMBER_RESPONSE, buf);
}

static int acs_cp_handle_invalidate_all_established_security(struct acs_procedure *proc,
							     const struct acs_frame *frame)
{
	int err;

	if (frame->payload_len != 1U) {
		return acs_cp_send_response_code(proc,
						 BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	err = acs_runtime_invalidate_all_security();
	if (err) {
		return acs_cp_send_response_code(proc,
						 BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	return acs_cp_send_response_code(proc,
					 BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY,
					 ACS_CP_RESPONSE_SUCCESS);
}

static int acs_cp_handle_invalidate_key(struct acs_procedure *proc, const struct acs_frame *frame)
{
	const struct acs_cp_key_id_req *req;
	uint16_t key_id;
	int err;

	if (frame->payload_len != sizeof(*req)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_INVALIDATE_KEY,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	req = (const void *)frame->payload;
	key_id = acs_cp_wire_get_le16(req->key_id_le);
	err = acs_runtime_invalidate_key(proc->conn, key_id);
	if (err == -ENOENT) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_INVALIDATE_KEY,
						 ACS_CP_RESPONSE_NO_RECORDS_FOUND);
	}
	if (err == -EALREADY) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_INVALIDATE_KEY,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}
	if (err) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_INVALIDATE_KEY,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_INVALIDATE_KEY,
					 ACS_CP_RESPONSE_SUCCESS);
}

static int acs_cp_handle_abort(struct acs_procedure *proc, const struct acs_frame *frame)
{
	if (frame->payload_len != 1U) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ABORT,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ABORT,
					 ACS_CP_RESPONSE_ABORT_UNSUCCESSFUL);
}

static int acs_cp_handle_set_security_controls_switch(struct acs_procedure *proc,
						      const struct acs_frame *frame)
{
	const struct acs_cp_switch_req *req;
	struct acs_conn_ctx *conn_ctx;
	uint8_t switch_state;

	if (!IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH,
						 ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	if (frame->payload_len != sizeof(*req)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	conn_ctx = acs_runtime_lookup_conn(proc->conn);
	if (!conn_ctx) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	req = (const void *)frame->payload;
	if (req->switch_state != 0U && req->switch_state != 1U) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	switch_state = req->switch_state;
	if (switch_state != 0U) {
		conn_ctx->status_flags |= BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	} else {
		conn_ctx->status_flags &= (uint8_t)~BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	}

	(void)acs_persist_save_conn(conn_ctx);
	return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH,
					 ACS_CP_RESPONSE_SUCCESS);
}

static int acs_cp_handle_get_key_uri(struct acs_procedure *proc, const struct acs_frame *frame)
{
#if IS_ENABLED(CONFIG_BT_ACS_KEY_URI)
	const struct acs_cp_key_id_req *req;
	const struct bt_acs_cb *cb = acs_runtime_callbacks();
	struct net_buf *buf;
	uint16_t key_id;
	uint16_t uri_len = 0U;
	uint16_t uri_max;
	int err;

	if (frame->payload_len != sizeof(*req)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_KEY_URI,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	if (!cb || !cb->key_uri_get) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_KEY_URI,
						 ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	}

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_KEY_URI,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	req = (const void *)frame->payload;
	key_id = acs_cp_wire_get_le16(req->key_id_le);
	net_buf_add_le16(buf, key_id);
	uri_max = MIN((uint16_t)net_buf_tailroom(buf), (uint16_t)CONFIG_BT_ACS_KEY_URI_MAX_LEN);
	err = cb->key_uri_get(proc->conn, key_id, net_buf_tail(buf), uri_max, &uri_len);
	if (err || uri_len == 0U) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_KEY_URI,
						 ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	}

	net_buf_add(buf, uri_len);
	return acs_cp_send_payload(proc, BT_ACS_CP_OPCODE_KEY_URI_RESPONSE, buf);
#else
	ARG_UNUSED(frame);
	return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_KEY_URI,
					 ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
#endif
}

static int acs_cp_handle_initiate_pairing(struct acs_procedure *proc, const struct acs_frame *frame)
{
	int err;

	if (!IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_INITIATE_PAIRING,
						 ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	if (frame->payload_len != 1U) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_INITIATE_PAIRING,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

#ifdef BT_SECURITY_FORCE_PAIR
	err = bt_conn_set_security(proc->conn, BT_SECURITY_L2 | BT_SECURITY_FORCE_PAIR);
#else
	err = bt_conn_set_security(proc->conn, BT_SECURITY_L2);
#endif
	if (err) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_INITIATE_PAIRING,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_INITIATE_PAIRING,
					 ACS_CP_RESPONSE_SUCCESS);
}

static int acs_cp_handle_get_service_characteristic_uuids(struct acs_procedure *proc,
							   const struct acs_frame *frame)
{
	const struct acs_cp_handle_req *req;
	struct net_buf *buf;
	uint16_t resource_handle;
	int err;

	if (!IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP)) {
		return acs_cp_send_response_code(
			proc, BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE,
			ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	if (frame->payload_len != sizeof(*req)) {
		return acs_cp_send_response_code(
			proc, BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE,
			ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	req = (const void *)frame->payload;
	resource_handle = acs_cp_wire_get_le16(req->handle_le);
	buf = acs_channel_buf_alloc();
	if (!buf) {
		return acs_cp_send_response_code(
			proc, BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE,
			ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	err = acs_cp_append_service_char_uuids(buf, resource_handle);
	if (err == -ENOENT) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(
			proc, BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE,
			ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	}
	if (err) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(
			proc, BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE,
			ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	return acs_cp_send_payload(
		proc, BT_ACS_CP_OPCODE_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE_RESPONSE,
		buf);
}

static int acs_cp_handle_att_mtu(struct acs_procedure *proc, const struct acs_frame *frame)
{
	struct net_buf *buf;
	uint16_t mtu;

	if (!IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ATT_MTU,
						 ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	if (frame->payload_len != 1U) {
		LOG_WRN("ACS CP ATT_MTU invalid operand len=%u", frame->payload_len);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ATT_MTU,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_ATT_MTU,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	mtu = bt_gatt_get_mtu(proc->conn) - 3U;
	net_buf_add_le16(buf, mtu);

	return acs_cp_send_payload(proc, BT_ACS_CP_OPCODE_ATT_MTU_RESPONSE, buf);
}

static int acs_cp_handle_set_client_nonce_fixed(struct acs_procedure *proc,
						const struct acs_frame *frame)
{
	const struct acs_cp_set_client_nonce_fixed_req *req;
	struct acs_conn_ctx *conn_ctx;
	bool security_established;
	uint16_t key_id;
	const uint8_t *nonce;
	size_t nonce_len;
	int err;

	nonce_len = CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE;
	if (frame->payload_len != sizeof(*req) + nonce_len) {
		LOG_WRN("ACS CP SET_CLIENT_NONCE_FIXED invalid operand len=%u", frame->payload_len);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	conn_ctx = acs_runtime_lookup_conn(proc->conn);
	if (!conn_ctx) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	req = (const void *)frame->payload;
	key_id = acs_cp_wire_get_le16(req->key_id_le);
	if (key_id != acs_cp_algorithm_key_id()) {
		LOG_WRN("ACS CP SET_CLIENT_NONCE_FIXED unsupported key_id=0x%04x", key_id);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
						 ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE);
	}

	if (!acs_cp_algorithm_supports_client_fixed_nonce(key_id)) {
		LOG_WRN("ACS CP SET_CLIENT_NONCE_FIXED unsupported nonce type for key_id=0x%04x",
			key_id);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
						 ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	security_established =
		(conn_ctx->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED) != 0U &&
		conn_ctx->crypto.active_key_id == key_id;
	if (conn_ctx->kex.state != ACS_KEX_IDLE || security_established) {
		LOG_WRN("ACS CP SET_CLIENT_NONCE_FIXED not applicable: kex_state=%d security=%u key_id=0x%04x",
			(int)conn_ctx->kex.state, security_established ? 1U : 0U, key_id);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	nonce = req->nonce;
	if (conn_ctx->crypto.server_nonce_fixed_len == nonce_len &&
	    memcmp(nonce, conn_ctx->crypto.server_nonce_fixed, nonce_len) == 0) {
		LOG_WRN("ACS CP SET_CLIENT_NONCE_FIXED matches server nonce fixed");
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	if (acs_runtime_client_nonce_conflicts(proc->conn, nonce, nonce_len) ||
	    acs_persist_client_nonce_conflicts(proc->conn, nonce, nonce_len)) {
		LOG_WRN("ACS CP SET_CLIENT_NONCE_FIXED conflicts with stored client nonce fixed");
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	err = acs_crypto_set_client_nonce_fixed(&conn_ctx->crypto, nonce, nonce_len);
	if (err) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	(void)acs_persist_save_conn(conn_ctx);

	return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED,
					 ACS_CP_RESPONSE_SUCCESS);
}

static int acs_cp_handle_get_resource_handle_uuid_map(struct acs_procedure *proc,
						      const struct acs_frame *frame)
{
	struct net_buf *buf;
	int err;

	if (!IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP,
						 ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	if (frame->payload_len != 1U) {
		LOG_WRN("ACS CP GET_RESOURCE_HANDLE_TO_UUID_MAP invalid operand len=%u",
			frame->payload_len);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	err = acs_cp_append_resource_handle_uuid_map(buf);
	if (err) {
		acs_channel_buf_free(buf);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP,
						 err == -ENOENT ? ACS_CP_RESPONSE_NO_RECORDS_FOUND :
								  ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	return acs_cp_send_payload(proc, BT_ACS_CP_OPCODE_RESOURCE_HANDLE_UUID_MAP_RESPONSE, buf);
}

static int acs_cp_handle_get_all_active_descriptors(struct acs_procedure *proc,
						    const struct acs_frame *frame)
{
	const struct bt_acs_restriction_map *map;
	int err;

	if (!IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS) ||
	    !IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
						 ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}

	if (frame->payload_len != 1U) {
		LOG_WRN("ACS CP GET_ALL_ACTIVE_DESCRIPTORS invalid operand len=%u",
			frame->payload_len);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
						 ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	map = acs_cp_find_map(acs_cp_active_map_id(proc->conn));
	if (!map) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
						 ACS_CP_RESPONSE_NO_RECORDS_FOUND);
	}

	if (acs_cp_map_requires_secure_transport(map) &&
	    (proc->flags & ACS_PROC_FLAG_SECURE_TRANSPORT) == 0U) {
		LOG_WRN("ACS CP GET_ALL_ACTIVE_DESCRIPTORS requires secure transport for map_id=0x%04x",
			map->map_id);
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	proc->ops = &acs_cp_all_active_ops;
	proc->step = ACS_ALL_ACTIVE_STEP_RMAP;
	err = acs_cp_send_rmap_descriptor_response(proc, map->map_id, ACS_RMAP_FILTER_ALL);
	if (err == -ENOENT) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
						 ACS_CP_RESPONSE_NO_RECORDS_FOUND);
	}
	if (acs_cp_send_failed(err)) {
		return acs_cp_send_response_code(proc, BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS,
						 ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	return err;
}

int acs_cp_domain_handle(struct acs_procedure *proc, const struct acs_frame *frame)
{
	uint8_t opcode;

	if (!proc || !frame || !frame->payload || frame->payload_len == 0U) {
		return -EINVAL;
	}

	opcode = frame->payload[0];

	switch (opcode) {
	case BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE:
		LOG_DBG("ACS CP opcode GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE");
		return acs_cp_handle_get_service_characteristic_uuids(proc, frame);
	case BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS:
		LOG_DBG("ACS CP opcode GET_ALL_ACTIVE_DESCRIPTORS");
		return acs_cp_handle_get_all_active_descriptors(proc, frame);
	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR:
		LOG_DBG("ACS CP opcode GET_RESTRICTION_MAP_DESCRIPTOR");
		return acs_cp_handle_get_restriction_map_descriptor(proc, frame);
	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST:
		LOG_DBG("ACS CP opcode GET_RESTRICTION_MAP_ID_LIST");
		return acs_cp_handle_get_restriction_map_id_list(proc, frame);
	case BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP:
		LOG_DBG("ACS CP opcode ACTIVATE_RESTRICTION_MAP");
		return acs_cp_handle_activate_restriction_map(proc, frame);
	case BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP:
		LOG_DBG("ACS CP opcode GET_RESOURCE_HANDLE_UUID_MAP");
		return acs_cp_handle_get_resource_handle_uuid_map(proc, frame);
	case BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR:
		LOG_DBG("ACS CP opcode GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR");
		return acs_cp_handle_get_information_security_configuration_descriptor(proc, frame);
	case BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR:
		LOG_DBG("ACS CP opcode GET_KEY_DESCRIPTOR");
		return acs_cp_handle_get_key_descriptor(proc, frame);
	case BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST:
		LOG_DBG("ACS CP opcode GET_CURRENT_KEY_LIST");
		return acs_cp_handle_get_current_key_list(proc, frame);
	case BT_ACS_CP_OPCODE_START_KEY_EXCHANGE:
		LOG_DBG("ACS CP opcode START_KEY_EXCHANGE");
		return acs_cp_handle_start_key_exchange(proc, frame);
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH:
		LOG_DBG("ACS CP opcode KEY_EXCHANGE_ECDH");
		return acs_cp_handle_key_exchange_ecdh(proc, frame);
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF:
		LOG_DBG("ACS CP opcode KEY_EXCHANGE_KDF");
		return acs_cp_handle_key_exchange_kdf(proc, frame);
	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE:
		LOG_DBG("ACS CP opcode ECDH_CONFIRM_CODE");
		return acs_cp_handle_ecdh_confirm_code(proc, frame);
	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND:
		LOG_DBG("ACS CP opcode ECDH_CONFIRM_RAND");
		return acs_cp_handle_ecdh_confirm_rand(proc, frame);
	case BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY:
		LOG_DBG("ACS CP opcode INVALIDATE_ALL_ESTABLISHED_SECURITY");
		return acs_cp_handle_invalidate_all_established_security(proc, frame);
	case BT_ACS_CP_OPCODE_INVALIDATE_KEY:
		LOG_DBG("ACS CP opcode INVALIDATE_KEY");
		return acs_cp_handle_invalidate_key(proc, frame);
	case BT_ACS_CP_OPCODE_ABORT:
		LOG_DBG("ACS CP opcode ABORT");
		return acs_cp_handle_abort(proc, frame);
	case BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH:
		LOG_DBG("ACS CP opcode SET_SECURITY_CONTROLS_SWITCH");
		return acs_cp_handle_set_security_controls_switch(proc, frame);
	case BT_ACS_CP_OPCODE_GET_KEY_URI:
		LOG_DBG("ACS CP opcode GET_KEY_URI");
		return acs_cp_handle_get_key_uri(proc, frame);
	case BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED:
		LOG_DBG("ACS CP opcode SET_CLIENT_NONCE_FIXED");
		return acs_cp_handle_set_client_nonce_fixed(proc, frame);
	case BT_ACS_CP_OPCODE_ATT_MTU:
		LOG_DBG("ACS CP opcode ATT_MTU");
		return acs_cp_handle_att_mtu(proc, frame);
	case BT_ACS_CP_OPCODE_INITIATE_PAIRING:
		LOG_DBG("ACS CP opcode INITIATE_PAIRING");
		return acs_cp_handle_initiate_pairing(proc, frame);
	case BT_ACS_CP_OPCODE_GET_FEATURE:
		LOG_DBG("ACS CP opcode GET_FEATURE");
		return acs_cp_handle_get_feature(proc, frame);
	default:
		LOG_WRN("ACS CP unsupported opcode=0x%02x", opcode);
		return acs_cp_send_response_code(proc, opcode,
						 ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
	}
}
