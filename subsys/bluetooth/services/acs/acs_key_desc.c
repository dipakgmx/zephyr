/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include "acs_cp_operands.h"
#include "acs_key_desc.h"
#include "acs_internal.h"
#include "acs_reply.h"
#include "acs_keys.h"

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Security algorithm records take their key from the KDF key (Table 4.45). */
#define ACS_KEY_DESC_ALG_PARENT_KEY_ID ACS_KEY_ID_KDF

/* The sequence number of a nonce is a 64-bit counter (Table 4.47). */
BUILD_ASSERT(ACS_GCM_NONCE_VAR_SIZE == sizeof(uint64_t) &&
	     ACS_CCM_NONCE_VAR_SIZE == sizeof(uint64_t) &&
	     ACS_GMAC_NONCE_VAR_SIZE == sizeof(uint64_t));

/*
 * Key descriptor records (Table 4.36): the ECDH and KDF key exchanges, then one
 * security algorithm record per enabled algorithm. acs_key_desc_alg_record()
 * relies on the algorithm records coming last.
 */
static const struct bt_acs_key_desc_record acs_key_desc_records[] = {
	{
		.type_id = ACS_KEY_REC_ECDH,
		.key_id = ACS_KEY_ID_ECDH,
		/*
		 * Curve P-256 and HKDF SHA-256 128-bit (ACP 1.0 §4.1.1.1, Table 4.44
		 * 0x00): KDF_Info is HKDF's info input, with nothing concatenated to it.
		 */
		.ecdh = {
			.server_pk_fmt = ACS_PK_FMT_UNCOMPRESSED,
			.client_pk_fmt = ACS_PK_FMT_UNCOMPRESSED,
			.curve = ACS_CURVE_P256,
			.kdf = ACS_KDF_SHA256,
		},
	},
	{
		.type_id = ACS_KEY_REC_KDF,
		.key_id = ACS_KEY_ID_KDF,
		.kdf = {
			.parent_key_id = ACS_KEY_ID_ECDH,
			.kdf_algorithm = ACS_KDF_SHA256,
		},
	},
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
	{
		.type_id = ACS_KEY_REC_AES_128_GCM,
		.key_id = ACS_KEY_ID_GCM,
		.aes = {
			.parent_key_id = ACS_KEY_DESC_ALG_PARENT_KEY_ID,
			.msg_type = ACS_MSG_TYPE_PROTECTED,
			.mac_size = ACS_CRYPTO_AUTH_TAG_SIZE,
			.nonce_type = ACS_NONCE_SEQ_DIFF_FIXED,
			.nonce_size = ACS_GCM_NONCE_SIZE,
			.nonce_var_size = ACS_GCM_NONCE_VAR_SIZE,
			.nonce_fixed_size = ACS_GCM_NONCE_FIXED_SIZE,
		},
	},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	{
		.type_id = ACS_KEY_REC_AES_128_CCM,
		.key_id = ACS_KEY_ID_CCM,
		.aes = {
			.parent_key_id = ACS_KEY_DESC_ALG_PARENT_KEY_ID,
			.msg_type = ACS_MSG_TYPE_PROTECTED,
			.mac_size = ACS_CCM_MAC_SIZE,
			.nonce_type = ACS_CCM_NONCE_TYPE,
			.nonce_size = ACS_CCM_NONCE_SIZE,
			.nonce_var_size = ACS_CCM_NONCE_VAR_SIZE,
			.nonce_fixed_size = ACS_CCM_NONCE_FIXED_SIZE,
		},
	},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	{
		.type_id = ACS_KEY_REC_AES_128_GMAC,
		.key_id = ACS_KEY_ID_GMAC,
		.aes = {
			.parent_key_id = ACS_KEY_DESC_ALG_PARENT_KEY_ID,
			.msg_type = ACS_MSG_TYPE_PROTECTED,
			.mac_size = ACS_CRYPTO_AUTH_TAG_SIZE,
			.nonce_type = ACS_NONCE_SEQ_DIFF_FIXED,
			.nonce_size = ACS_GMAC_NONCE_SIZE,
			.nonce_var_size = ACS_GMAC_NONCE_VAR_SIZE,
			.nonce_fixed_size = ACS_GMAC_NONCE_FIXED_SIZE,
		},
	},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	{
		.type_id = ACS_KEY_REC_AES_128_CMAC,
		.key_id = ACS_KEY_ID_CMAC,
		.aes = {
			.parent_key_id = ACS_KEY_DESC_ALG_PARENT_KEY_ID,
			.msg_type = ACS_MSG_TYPE_PROTECTED,
			.mac_size = ACS_CRYPTO_AUTH_TAG_SIZE,
			.nonce_type = ACS_NONCE_PROFILE_DEF,
		},
	},
#endif
};

BUILD_ASSERT(ARRAY_SIZE(acs_key_desc_records) == ACS_KEY_ID_COUNT + ACS_SEC_ALG_COUNT,
	     "one security algorithm record per enabled algorithm");

/* Get Key Descriptor for all records fits one message; an algorithm record is the largest. */
BUILD_ASSERT(ARRAY_SIZE(acs_key_desc_records) *
			     (sizeof(struct acs_desc_rec_hdr) +
			      ACS_KEY_DESC_AES_ALG_MANDATORY_SIZE + ACS_MAX_NONCE_PREFIX_SIZE) <=
		     ACS_MESSAGE_MAX_OPERAND,
	     "Key Descriptor Response exceeds one message");

const struct bt_acs_key_desc_record *acs_key_desc_alg_record(size_t index)
{
	__ASSERT_NO_MSG(index < ACS_SEC_ALG_COUNT);

	return &acs_key_desc_records[ACS_KEY_ID_COUNT + index];
}

const struct bt_acs_key_desc_record *acs_key_desc_lookup(uint16_t key_id)
{
	ARRAY_FOR_EACH_PTR(acs_key_desc_records, rec) {
		if (rec->key_id == key_id) {
			return rec;
		}
	}
	return NULL;
}

bool acs_key_desc_is_algorithm_record(const struct bt_acs_key_desc_record *rec)
{
	return rec && (
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
			      rec->type_id == ACS_KEY_REC_AES_128_CCM ||
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
			      rec->type_id == ACS_KEY_REC_AES_128_GCM ||
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
			      rec->type_id == ACS_KEY_REC_AES_128_CMAC ||
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
			      rec->type_id == ACS_KEY_REC_AES_128_GMAC ||
#endif
			      false);
}

bool acs_key_desc_has_nonce_record(const struct bt_acs_key_desc_record *rec)
{
	return acs_key_desc_is_algorithm_record(rec) && rec->type_id != ACS_KEY_REC_AES_128_CMAC &&
	       rec->aes.nonce_type != ACS_NONCE_PROFILE_DEF && rec->aes.nonce_var_size > 0U;
}

/* ECDH Key Exchange record (Table 4.39). */
static int append_ecdh_record(const struct bt_acs_key_desc_record *rec, struct net_buf *buf)
{
	int err = acs_desc_add_record_header(buf, rec->type_id, rec->key_id,
					     ACS_KEY_DESC_ECDH_DATA_SIZE);

	if (err) {
		return err;
	}

	net_buf_add_u8(buf, rec->ecdh.server_pk_fmt);
	net_buf_add_u8(buf, rec->ecdh.client_pk_fmt);
	net_buf_add_u8(buf, rec->ecdh.curve);
	net_buf_add_u8(buf, rec->ecdh.kdf);

	return 0;
}

/* KDF Key Exchange record (Table 4.43). */
static int append_kdf_record(const struct bt_acs_key_desc_record *rec, struct net_buf *buf)
{
	int err = acs_desc_add_record_header(buf, rec->type_id, rec->key_id,
					     ACS_KEY_DESC_KDF_DATA_SIZE);

	if (err) {
		return err;
	}

	net_buf_add_le16(buf, rec->kdf.parent_key_id);
	net_buf_add_u8(buf, rec->kdf.kdf_algorithm);

	return 0;
}

/* Security algorithm record; CMAC carries no nonce fields (Table 4.45 C.1). */
static int append_alg_record(const struct bt_acs_key_desc_record *rec, struct net_buf *buf,
			     struct bt_acs_conn *acs_conn)
{
	bool has_nonce = rec->type_id != ACS_KEY_REC_AES_128_CMAC;
	uint8_t nonce_fixed[ACS_MAX_NONCE_FIXED_SIZE];
	uint8_t data_size;
	int err;

	if (has_nonce && rec->aes.nonce_fixed_size > 0U) {
		err = acs_keys_server_nonce_fixed(acs_conn, rec->key_id, nonce_fixed);
		if (err) {
			return err;
		}
	}

	data_size = has_nonce ? ACS_KEY_DESC_AES_ALG_MANDATORY_SIZE + rec->aes.nonce_fixed_size
			      : ACS_KEY_DESC_AES_CMAC_DATA_SIZE;
	err = acs_desc_add_record_header(buf, rec->type_id, rec->key_id, data_size);
	if (err) {
		return err;
	}

	net_buf_add_le16(buf, rec->aes.parent_key_id);
	net_buf_add_u8(buf, rec->aes.msg_type);
	net_buf_add_u8(buf, rec->aes.mac_size);

	if (has_nonce) {
		net_buf_add_u8(buf, rec->aes.nonce_type);
		net_buf_add_u8(buf, rec->aes.nonce_var_size);
		net_buf_add_u8(buf, rec->aes.nonce_fixed_size);
		net_buf_add_mem(buf, nonce_fixed, rec->aes.nonce_fixed_size);
	}

	return 0;
}

static int append_record(const struct bt_acs_key_desc_record *rec, struct net_buf *buf,
			 struct bt_acs_conn *acs_conn)
{
	LOG_DBG("Key rec: type=0x%02x key_id=0x%04x", rec->type_id, rec->key_id);

	if (rec->type_id == ACS_KEY_REC_ECDH) {
		return append_ecdh_record(rec, buf);
	}
	if (rec->type_id == ACS_KEY_REC_KDF) {
		return append_kdf_record(rec, buf);
	}
	if (acs_key_desc_is_algorithm_record(rec)) {
		return append_alg_record(rec, buf, acs_conn);
	}

	LOG_WRN("Key rec: unknown type_id 0x%02x for key_id 0x%04x; skipping", rec->type_id,
		rec->key_id);
	return 0;
}

uint8_t acs_key_desc_build_response(uint16_t filter_id, struct net_buf *buf,
				    struct bt_acs_conn *acs_conn)
{
	bool found = false;
	int err;

	ARRAY_FOR_EACH_PTR(acs_key_desc_records, rec) {
		if (filter_id != BT_ACS_GET_KEY_DESC_ALL_RECORDS_FILTER &&
		    filter_id != rec->key_id) {
			continue;
		}

		found = true;
		err = append_record(rec, buf, acs_conn);
		if (err) {
			return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
		}
	}

	if (!found) {
		LOG_WRN("ACS Key Desc: no record found for filter 0x%04X", filter_id);
		return BT_ACS_CP_RESPONSE_NO_RECORDS_FOUND;
	}

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

uint8_t acs_cp_handle_get_key_descriptor(struct acs_reply *reply, struct net_buf_simple *buf)
{
	uint16_t filter_id;

	if (buf->len < sizeof(struct acs_cp_get_key_descriptor_req)) {
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	}

	filter_id = net_buf_simple_pull_le16(buf);

	return acs_key_desc_build_response(filter_id, reply->response, reply->conn);
}
