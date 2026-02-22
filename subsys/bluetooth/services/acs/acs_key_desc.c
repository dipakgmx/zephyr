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

#define ACS_KEY_DESC_PK_FMT ACS_PK_FMT_UNCOMPRESSED

/* ECDH uses Curve P-256 and HKDF-SHA-256 without appended KDF_Info. */
#define ACS_KEY_DESC_CURVE ACS_CURVE_P256
#define ACS_KEY_DESC_KDF   ACS_KDF_SHA256

/* Default key descriptor records selected by Kconfig. */
#define ACS_KEY_DESC_ALG_PARENT_KEY_ID ACS_KEY_ID_KDF

BT_ACS_KEY_DESC_DEFINE(acs_key_desc_ecdh, .type_id = ACS_KEY_REC_ECDH, .key_id = ACS_KEY_ID_ECDH,
		       .ecdh = {
			       .server_pk_fmt = ACS_KEY_DESC_PK_FMT,
			       .client_pk_fmt = ACS_KEY_DESC_PK_FMT,
			       .curve = ACS_KEY_DESC_CURVE,
			       .kdf = ACS_KEY_DESC_KDF,
		       });

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
BT_ACS_KEY_DESC_DEFINE(acs_key_desc_ccm, .type_id = ACS_KEY_REC_AES_128_CCM,
		       .key_id = ACS_KEY_ID_CCM,
		       .aes = {
			       .parent_key_id = ACS_KEY_DESC_ALG_PARENT_KEY_ID,
			       .msg_type = ACS_MSG_TYPE_PROTECTED,
			       .mac_size = ACS_CCM_MAC_SIZE,
			       .nonce_type = ACS_CCM_NONCE_TYPE,
			       .nonce_size = ACS_CCM_NONCE_SIZE,
			       .nonce_var_size = ACS_CCM_NONCE_VAR_SIZE,
			       .nonce_fixed_size = ACS_CCM_NONCE_FIXED_SIZE,
		       });
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
BT_ACS_KEY_DESC_DEFINE(acs_key_desc_gcm, .type_id = ACS_KEY_REC_AES_128_GCM,
		       .key_id = ACS_KEY_ID_GCM,
		       .aes = {
			       .parent_key_id = ACS_KEY_DESC_ALG_PARENT_KEY_ID,
			       .msg_type = ACS_MSG_TYPE_PROTECTED,
			       .mac_size = ACS_CRYPTO_AUTH_TAG_SIZE,
			       .nonce_type = ACS_NONCE_SEQ_DIFF_FIXED,
			       .nonce_size = ACS_GCM_NONCE_SIZE,
			       .nonce_var_size = ACS_GCM_NONCE_VAR_SIZE,
			       .nonce_fixed_size = ACS_GCM_NONCE_FIXED_SIZE,
		       });
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
BT_ACS_KEY_DESC_DEFINE(acs_key_desc_cmac, .type_id = ACS_KEY_REC_AES_128_CMAC,
		       .key_id = ACS_KEY_ID_CMAC,
		       .aes = {
			       .parent_key_id = ACS_KEY_DESC_ALG_PARENT_KEY_ID,
			       .msg_type = ACS_MSG_TYPE_PROTECTED,
			       .mac_size = ACS_CRYPTO_AUTH_TAG_SIZE,
			       .nonce_type = ACS_NONCE_PROFILE_DEF,
			       .nonce_size = 0U,
			       .nonce_var_size = 0U,
			       .nonce_fixed_size = 0U,
		       });
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
BT_ACS_KEY_DESC_DEFINE(acs_key_desc_gmac, .type_id = ACS_KEY_REC_AES_128_GMAC,
		       .key_id = ACS_KEY_ID_GMAC,
		       .aes = {
			       .parent_key_id = ACS_KEY_DESC_ALG_PARENT_KEY_ID,
			       .msg_type = ACS_MSG_TYPE_PROTECTED,
			       .mac_size = ACS_CRYPTO_AUTH_TAG_SIZE,
			       .nonce_type = ACS_NONCE_SEQ_DIFF_FIXED,
			       .nonce_size = ACS_GMAC_NONCE_SIZE,
			       .nonce_var_size = ACS_GMAC_NONCE_VAR_SIZE,
			       .nonce_fixed_size = ACS_GMAC_NONCE_FIXED_SIZE,
		       });
#endif

BT_ACS_KEY_DESC_DEFINE(acs_key_desc_kdf_rec, .type_id = ACS_KEY_REC_KDF, .key_id = ACS_KEY_ID_KDF,
		       .kdf = {
			       /* The KDF child is derived from the ECDH key. */
			       .parent_key_id = ACS_KEY_ID_ECDH,
			       .kdf_algorithm = ACS_KEY_DESC_KDF,
		       });

const struct bt_acs_key_desc_record *acs_key_desc_lookup(uint16_t key_id)
{
	STRUCT_SECTION_FOREACH(bt_acs_key_desc_record, rec) {
		if (rec->key_id == key_id) {
			return rec;
		}
	}
	return NULL;
}

int acs_key_desc_validate_records(void)
{
	size_t algo_records = 0;

	STRUCT_SECTION_FOREACH(bt_acs_key_desc_record, rec) {
		if (rec->key_id == 0U || rec->key_id == BT_ACS_GET_KEY_DESC_ALL_RECORDS_FILTER) {
			LOG_ERR("key descriptor uses reserved Key_ID 0x%04x", rec->key_id);
			return -EINVAL;
		}

		/* Lookup returns the first match, so duplicates resolve by link order. */
		if (acs_key_desc_lookup(rec->key_id) != rec) {
			LOG_ERR("duplicate key descriptor Key_ID 0x%04x", rec->key_id);
			return -EINVAL;
		}

		/* The ECDH key is the root and the KDF key its only child. */
		if (rec->type_id == ACS_KEY_REC_KDF &&
		    (rec->key_id != ACS_KEY_ID_KDF || rec->kdf.parent_key_id != ACS_KEY_ID_ECDH)) {
			LOG_ERR("KDF record 0x%04x must be Key_ID 0x%04x with parent 0x%04x",
				rec->key_id, ACS_KEY_ID_KDF, ACS_KEY_ID_ECDH);
			return -EINVAL;
		}

		if (!acs_key_desc_is_algorithm_record(rec)) {
			continue;
		}

		/* Security algorithm keys are derived by the KDF key exchange. */
		if (rec->aes.parent_key_id != ACS_KEY_ID_KDF) {
			LOG_ERR("algorithm record 0x%04x must have parent Key_ID 0x%04x",
				rec->key_id, ACS_KEY_ID_KDF);
			return -EINVAL;
		}

		/* Sequence numbers are 64-bit counters behind a fixed prefix (Table 4.47). */
		if (acs_key_desc_has_nonce_record(rec) &&
		    (rec->aes.nonce_type != ACS_NONCE_SEQ_DIFF_FIXED ||
		     acs_key_desc_nonce_var_size(rec) != sizeof(uint64_t) ||
		     acs_key_desc_nonce_fixed_size(rec) > ACS_MAX_NONCE_PREFIX_SIZE)) {
			LOG_ERR("algorithm record 0x%04x needs Nonce_Type %u, an 8-octet variable "
				"part and at most %u fixed octets",
				rec->key_id, ACS_NONCE_SEQ_DIFF_FIXED, ACS_MAX_NONCE_PREFIX_SIZE);
			return -EINVAL;
		}

		algo_records++;
	}

	/* Every security algorithm record needs its state on each connection. */
	if (algo_records > ACS_SEC_ALG_COUNT) {
		LOG_ERR("%zu security algorithm records exceed %u per-connection slots",
			algo_records, ACS_SEC_ALG_COUNT);
		return -ENOSPC;
	}

	return 0;
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

int acs_key_desc_build_response(uint16_t filter_id, struct net_buf *buf,
				struct bt_acs_conn *acs_conn)
{
	bool found = false;
	int err;

	STRUCT_SECTION_FOREACH(bt_acs_key_desc_record, rec) {
		if (filter_id != BT_ACS_GET_KEY_DESC_ALL_RECORDS_FILTER &&
		    filter_id != rec->key_id) {
			continue;
		}

		found = true;
		err = append_record(rec, buf, acs_conn);
		if (err) {
			return err;
		}
	}

	if (!found) {
		LOG_WRN("ACS Key Desc: no record found for filter 0x%04X", filter_id);
		return -ENOENT;
	}

	return 0;
}

uint8_t acs_cp_handle_get_key_descriptor(struct acs_reply *reply, struct net_buf_simple *buf)
{
	uint16_t filter_id;
	int err;

	if (buf->len < sizeof(struct acs_cp_get_key_descriptor_req)) {
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	}

	filter_id = net_buf_simple_pull_le16(buf);

	err = acs_key_desc_build_response(filter_id, reply->response, reply->conn);
	if (err) {
		return errno_to_acs_status(err);
	}

	return BT_ACS_CP_RESPONSE_SUCCESS;
}
