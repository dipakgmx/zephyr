/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include "acs_key_desc.h"
#include "acs_internal.h"

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/*
 * Compile-time helpers for key descriptor field values.
 * These expand to integer constant expressions usable in static initialisers.
 */
#define ACS_KEY_DESC_PK_FMT                                                                        \
	(IS_ENABLED(CONFIG_BT_ACS_KEY_FORMAT_X509) ? ACS_PK_FMT_X509_DER : ACS_PK_FMT_UNCOMPRESSED)

#define ACS_KEY_DESC_CURVE                                                                         \
	(IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_P256)   ? ACS_CURVE_P256                              \
	 : IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_P384) ? ACS_CURVE_P384                              \
	 : IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_P521) ? ACS_CURVE_P521                              \
						     : ACS_CURVE_25519)

#define ACS_KEY_DESC_KDF                                                                           \
	(IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA256)             ? ACS_KDF_SHA256                    \
	 : IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA256_WITH_INFO) ? ACS_KDF_SHA256_WITH_INFO          \
	 : IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA384)           ? ACS_KDF_SHA384                    \
	 : IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA384_WITH_INFO) ? ACS_KDF_SHA384_WITH_INFO          \
	 : IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA512)           ? ACS_KDF_SHA512                    \
	 : IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA512_WITH_INFO) ? ACS_KDF_SHA512_WITH_INFO          \
							       : ACS_KDF_RESERVED)

/*
 * Default key descriptor records — compiled in when the corresponding Kconfig is enabled.
 * Applications may register additional records with BT_ACS_KEY_DESC_DEFINE() in their
 * own source files; the library discovers all records via the bt_acs_key_desc_record iterable
 * section.
 *
 * Algorithm records (CCM, GCM, CMAC, GMAC) carry a parent_key_id field that tells the peer
 * which key was used to derive the algorithm's session material.  When KDF is enabled, the
 * spec (Figure 4.4) requires algorithm records to reference the KDF key — not the ECDH key
 * directly — because the KDF step produces the child key that is actually loaded into the AEAD
 * engine.  Without KDF, the ECDH shared secret itself is the AEAD key, so the parent reference
 * stays at ACS_KEY_ID_ECDH.
 */
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
#define ACS_KEY_DESC_ALG_PARENT_KEY_ID ACS_KEY_ID_KDF
#else
#define ACS_KEY_DESC_ALG_PARENT_KEY_ID ACS_KEY_ID_ECDH
#endif

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
BT_ACS_KEY_DESC_DEFINE(acs_key_desc_ecdh, .type_id = ACS_KEY_REC_ECDH, .key_id = ACS_KEY_ID_ECDH,
		       .ecdh = {
			       .server_pk_fmt = ACS_KEY_DESC_PK_FMT,
			       .client_pk_fmt = ACS_KEY_DESC_PK_FMT,
			       .curve = ACS_KEY_DESC_CURVE,
			       .kdf = ACS_KEY_DESC_KDF,
		       });
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) && IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
BT_ACS_KEY_DESC_DEFINE(acs_key_desc_ccm, .type_id = ACS_KEY_REC_AES_128_CCM,
		       .key_id = ACS_KEY_ID_CCM,
		       .aes = {
			       .parent_key_id = ACS_KEY_DESC_ALG_PARENT_KEY_ID,
			       .msg_type = ACS_MSG_TYPE_PROTECTED,
			       .mac_size = ACS_CCM_MAC_SIZE,
			       .nonce_type = ACS_CCM_NONCE_TYPE,
			       .nonce_size = PSA_AEAD_NONCE_LENGTH(PSA_KEY_TYPE_AES, PSA_ALG_CCM),
			       .nonce_var_size = ACS_CCM_NONCE_VAR_SIZE,
			       .nonce_fixed_size = ACS_CCM_NONCE_FIXED_SIZE,
		       });
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) && IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
BT_ACS_KEY_DESC_DEFINE(acs_key_desc_gcm, .type_id = ACS_KEY_REC_AES_128_GCM,
		       .key_id = ACS_KEY_ID_GCM,
		       .aes = {
			       .parent_key_id = ACS_KEY_DESC_ALG_PARENT_KEY_ID,
			       .msg_type = ACS_MSG_TYPE_PROTECTED,
			       .mac_size = PSA_AEAD_TAG_LENGTH(PSA_KEY_TYPE_AES, 128, PSA_ALG_GCM),
			       .nonce_type = ACS_NONCE_SEQ_DIFF_FIXED,
			       .nonce_size = ACS_GCM_NONCE_SIZE,
			       .nonce_var_size = ACS_GCM_NONCE_VAR_SIZE,
			       .nonce_fixed_size = ACS_GCM_NONCE_FIXED_SIZE,
		       });
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC) &&                                          \
	IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
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

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) &&                                          \
	IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
BT_ACS_KEY_DESC_DEFINE(acs_key_desc_gmac, .type_id = ACS_KEY_REC_AES_128_GMAC,
		       .key_id = ACS_KEY_ID_GMAC,
		       .aes = {
			       .parent_key_id = ACS_KEY_DESC_ALG_PARENT_KEY_ID,
			       .msg_type = ACS_MSG_TYPE_PROTECTED,
			       .mac_size = PSA_AEAD_TAG_LENGTH(PSA_KEY_TYPE_AES, 128, PSA_ALG_GCM),
			       .nonce_type = ACS_NONCE_SEQ_DIFF_FIXED,
			       .nonce_size = ACS_GMAC_NONCE_SIZE,
			       .nonce_var_size = ACS_GCM_NONCE_VAR_SIZE,
			       .nonce_fixed_size = ACS_GCM_NONCE_FIXED_SIZE,
		       });
#endif

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
BT_ACS_KEY_DESC_DEFINE(acs_key_desc_oob, .type_id = ACS_KEY_REC_OOB, .key_id = ACS_KEY_ID_OOB,
		       .oob = {
			       .oob_method = ACS_OOB_METHOD_MANUFACTURER,
			       .server_pk_fmt = ACS_KEY_DESC_PK_FMT,
			       .client_pk_fmt = ACS_KEY_DESC_PK_FMT,
			       .curve = ACS_KEY_DESC_CURVE,
			       .kdf = ACS_KEY_DESC_KDF,
		       });
#endif

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
BT_ACS_KEY_DESC_DEFINE(acs_key_desc_kdf_rec, .type_id = ACS_KEY_REC_KDF, .key_id = ACS_KEY_ID_KDF,
		       .kdf = {
			       /* The KDF key record describes the key derivation step itself:
				* parent is always the ECDH key from which the child is derived. */
			       .parent_key_id = ACS_KEY_ID_ECDH,
			       .kdf_algorithm = ACS_KEY_DESC_KDF,
		       });
#endif

const struct bt_acs_key_desc_record *acs_key_desc_lookup(uint16_t key_id)
{
	STRUCT_SECTION_FOREACH(bt_acs_key_desc_record, rec) {
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

uint16_t acs_key_desc_parent_key_id(const struct bt_acs_key_desc_record *rec)
{
	if (!rec) {
		return 0U;
	}

	switch (rec->type_id) {
	case ACS_KEY_REC_KDF:
		return rec->kdf.parent_key_id;
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	case ACS_KEY_REC_AES_128_CCM:
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
	case ACS_KEY_REC_AES_128_GCM:
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	case ACS_KEY_REC_AES_128_CMAC:
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case ACS_KEY_REC_AES_128_GMAC:
#endif
		return rec->aes.parent_key_id;
	default:
		return 0U;
	}
}

/**
 * @brief Serialize an AES algorithm record into the response buffer.
 *
 * Table 4.45 condition C.1: Nonce_Type, Nonce_Variable_Size, Nonce_Fixed_Size,
 * and AC_Server_Nonce_Fixed_Value are Mandatory for CCM, EAX, GCM, and GMAC
 * but Excluded for CMAC.  CMAC's data field is only Key_ID + Message_Type +
 * MAC_Size (4 bytes).
 */
static int append_aes_record(const struct bt_acs_key_desc_record *rec, struct net_buf_simple *buf,
			     struct bt_acs_conn *acs_conn)
{
	if (rec->type_id == ACS_KEY_REC_AES_128_CMAC) {
		/* CMAC: nonce fields excluded (Table 4.45 C.1) */
		const uint8_t total =
			sizeof(struct acs_key_rec_hdr) + ACS_KEY_DESC_AES_CMAC_DATA_SIZE;

		if (net_buf_simple_tailroom(buf) < total) {
			return -ENOMEM;
		}

		struct acs_key_rec_hdr hdr = {
			.type_id = rec->type_id,
			.type_value = sys_cpu_to_le16(rec->key_id),
			.data_size = ACS_KEY_DESC_AES_CMAC_DATA_SIZE,
		};
		uint8_t data[ACS_KEY_DESC_AES_CMAC_DATA_SIZE];

		sys_put_le16(rec->aes.parent_key_id, &data[0]);
		data[2] = rec->aes.msg_type;
		data[3] = rec->aes.mac_size;

		net_buf_simple_add_mem(buf, &hdr, sizeof(hdr));
		net_buf_simple_add_mem(buf, data, sizeof(data));

		LOG_DBG("Key rec: type=0x%02x key_id=0x%04x parent=0x%04x mac=%u (CMAC, no nonce)",
			rec->type_id, rec->key_id, rec->aes.parent_key_id, rec->aes.mac_size);
		return 0;
	}

	/* CCM, GCM, EAX, GMAC: full wire layout with nonce fields */
	const uint8_t total_size =
		sizeof(struct acs_key_rec_aes_alg_hdr) + rec->aes.nonce_fixed_size;

	if (net_buf_simple_tailroom(buf) < total_size) {
		return -ENOMEM;
	}

	struct acs_key_rec_aes_alg_hdr wire = {
		.hdr =
			{
				.type_id = rec->type_id,
				.type_value = sys_cpu_to_le16(rec->key_id),
				.data_size = ACS_KEY_DESC_AES_ALG_MANDATORY_SIZE +
					     rec->aes.nonce_fixed_size,
			},
		.parent_key_id = sys_cpu_to_le16(rec->aes.parent_key_id),
		.msg_type = rec->aes.msg_type,
		.mac_size = rec->aes.mac_size,
		.nonce_type = rec->aes.nonce_type,
		.nonce_var_size = rec->aes.nonce_var_size,
		.nonce_fixed_size = rec->aes.nonce_fixed_size,
	};

	net_buf_simple_add_mem(buf, &wire, sizeof(wire));

	if (rec->aes.nonce_fixed_size > 0) {
		uint8_t server_fixed_nonce[ACS_MAX_NONCE_FIXED_SIZE];
		int err = acs_crypto_get_server_nonce_fixed(
			acs_conn, rec->key_id, server_fixed_nonce, rec->aes.nonce_fixed_size);

		if (err) {
			return err;
		}

		net_buf_simple_add_mem(buf, server_fixed_nonce, rec->aes.nonce_fixed_size);
	}

	LOG_DBG("Key rec: type=0x%02x key_id=0x%04x parent=0x%04x mac=%u nonce_type=%u var=%u "
		"fixed=%u",
		rec->type_id, rec->key_id, rec->aes.parent_key_id, rec->aes.mac_size,
		rec->aes.nonce_type, rec->aes.nonce_var_size, rec->aes.nonce_fixed_size);

	return 0;
}

static bool is_aes_type(uint8_t type_id)
{
	return
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
		type_id == ACS_KEY_REC_AES_128_CCM ||
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
		type_id == ACS_KEY_REC_AES_128_GCM ||
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
		type_id == ACS_KEY_REC_AES_128_CMAC ||
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
		type_id == ACS_KEY_REC_AES_128_GMAC ||
#endif
		false;
}

int acs_key_desc_build_response(struct net_buf_simple *operand, struct net_buf_simple *buf,
				struct bt_acs_conn *acs_conn)
{
	bool record_found;
	uint16_t filter_id;
	int err;

	if (operand->len < 2) {
		return -EINVAL;
	}

	filter_id = net_buf_simple_pull_le16(operand);
	record_found = false;

	LOG_DBG("ACS Key Desc: Querying Filter ID 0x%04X", filter_id);

	STRUCT_SECTION_FOREACH(bt_acs_key_desc_record, rec) {
		if (filter_id != BT_ACS_GET_KEY_DESC_ALL_RECORDS_FILTER &&
		    filter_id != rec->key_id) {
			continue;
		}

		record_found = true;

		if (rec->type_id == ACS_KEY_REC_ECDH) {
			struct acs_key_rec_ecdh wire = {
				.hdr =
					{
						.type_id = ACS_KEY_REC_ECDH,
						.type_value = sys_cpu_to_le16(rec->key_id),
						.data_size = ACS_KEY_DESC_ECDH_DATA_SIZE,
					},
				.server_pk_fmt = rec->ecdh.server_pk_fmt,
				.client_pk_fmt = rec->ecdh.client_pk_fmt,
				.curve = rec->ecdh.curve,
				.kdf = rec->ecdh.kdf,
			};

			if (net_buf_simple_tailroom(buf) < sizeof(wire)) {
				return -ENOMEM;
			}
			net_buf_simple_add_mem(buf, &wire, sizeof(wire));

			LOG_DBG("Key rec: type=ECDH key_id=0x%04x curve=%u kdf=%u srv_fmt=%u "
				"cli_fmt=%u",
				rec->key_id, rec->ecdh.curve, rec->ecdh.kdf,
				rec->ecdh.server_pk_fmt, rec->ecdh.client_pk_fmt);

		} else if (rec->type_id == ACS_KEY_REC_OOB) {
			struct acs_key_rec_oob wire = {
				.hdr =
					{
						.type_id = ACS_KEY_REC_OOB,
						.type_value = sys_cpu_to_le16(rec->key_id),
						.data_size = ACS_KEY_DESC_OOB_DATA_SIZE,
					},
				.oob_method = rec->oob.oob_method,
				.server_pk_fmt = rec->oob.server_pk_fmt,
				.client_pk_fmt = rec->oob.client_pk_fmt,
				.curve = rec->oob.curve,
				.kdf = rec->oob.kdf,
			};

			if (net_buf_simple_tailroom(buf) < sizeof(wire)) {
				return -ENOMEM;
			}
			net_buf_simple_add_mem(buf, &wire, sizeof(wire));

			LOG_DBG("Key rec: type=OOB key_id=0x%04x method=%u curve=%u kdf=%u",
				rec->key_id, rec->oob.oob_method, rec->oob.curve, rec->oob.kdf);

		} else if (rec->type_id == ACS_KEY_REC_KDF) {
			struct acs_key_rec_kdf wire = {
				.hdr =
					{
						.type_id = ACS_KEY_REC_KDF,
						.type_value = sys_cpu_to_le16(rec->key_id),
						.data_size = ACS_KEY_DESC_KDF_DATA_SIZE,
					},
				.parent_key_id = sys_cpu_to_le16(rec->kdf.parent_key_id),
				.kdf_algorithm = rec->kdf.kdf_algorithm,
			};

			if (net_buf_simple_tailroom(buf) < sizeof(wire)) {
				return -ENOMEM;
			}
			net_buf_simple_add_mem(buf, &wire, sizeof(wire));

			LOG_DBG("Key rec: type=KDF key_id=0x%04x parent=0x%04x kdf=%u", rec->key_id,
				rec->kdf.parent_key_id, rec->kdf.kdf_algorithm);

		} else if (is_aes_type(rec->type_id)) {
			err = append_aes_record(rec, buf, acs_conn);
			if (err) {
				return err;
			}
		} else {
			LOG_WRN("Key rec: unknown type_id 0x%02x for key_id 0x%04x; skipping",
				rec->type_id, rec->key_id);
			record_found = false;
		}
	}

	if (!record_found) {
		LOG_WRN("ACS Key Desc: no record found for filter 0x%04X", filter_id);
		return -ENOENT;
	}

	return 0;
}
