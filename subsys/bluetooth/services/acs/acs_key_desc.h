/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_KEY_DESC_H_
#define BT_GATT_ACS_KEY_DESC_H_

#include "acs_wire_constants.h"

#include <zephyr/types.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/iterable_sections.h>

#include "acs_crypto_config.h"

struct bt_acs_conn;

/* Filter value selecting all Key Descriptor records (Table 4.34). */
#define BT_ACS_GET_KEY_DESC_ALL_RECORDS_FILTER 0xFFFF

/* ECDH record Data size (Table 4.39). */
#define ACS_KEY_DESC_ECDH_DATA_SIZE 4

/* AES algorithm record Data size with nonce fields (Table 4.45). */
#define ACS_KEY_DESC_AES_ALG_MANDATORY_SIZE 7

/* AES-CMAC record Data size without nonce fields (Table 4.45 C.1). */
#define ACS_KEY_DESC_AES_CMAC_DATA_SIZE 4

/* KDF record Data size. */
#define ACS_KEY_DESC_KDF_DATA_SIZE 3

/* ECDH Key Exchange record, full wire layout (Table 4.39). */
struct acs_key_rec_ecdh {
	struct acs_desc_rec_hdr hdr;
	uint8_t server_pk_fmt;
	uint8_t client_pk_fmt;
	uint8_t curve;
	uint8_t kdf;
} __packed;

/* Fixed part of an AES algorithm record (Table 4.45). */
struct acs_key_rec_aes_alg_hdr {
	struct acs_desc_rec_hdr hdr;
	uint16_t parent_key_id; /* Parent_Key_ID, little-endian */
	uint8_t msg_type;
	uint8_t mac_size;
	uint8_t nonce_type;
	uint8_t nonce_var_size;
	uint8_t nonce_fixed_size;
	/* AC_Server_Nonce_Fixed bytes follow on the wire. */
} __packed;

/* KDF Key Exchange record wire layout. */
struct acs_key_rec_kdf {
	struct acs_desc_rec_hdr hdr;
	uint16_t parent_key_id; /* Parent_Key_ID, little-endian */
	uint8_t kdf_algorithm;
} __packed;

/* Key_IDs assigned by this server (Table 4.36). */
#define ACS_KEY_ID_ECDH 0x0001 /* ECDH key-exchange record */
#define ACS_KEY_ID_KDF  0x0002 /* KDF key-exchange record */
#define ACS_KEY_ID_GCM  0x0003 /* AES-128-GCM algorithm record */
#define ACS_KEY_ID_CCM  0x0004 /* AES-128-CCM algorithm record */
#define ACS_KEY_ID_CMAC 0x0005 /* AES-128-CMAC algorithm record */
#define ACS_KEY_ID_GMAC 0x0006 /* AES-128-GMAC algorithm record */

/* Maximum number of key-exchange Key_IDs (ECDH, KDF). */
#define ACS_KEY_ID_COUNT 2

/* Key record Type_ID values (Table 4.36); selects the Data field layout. */
enum acs_key_record_type {
	ACS_KEY_REC_OOB = 0x00,          /* Out-of-Band Key Exchange */
	ACS_KEY_REC_ECDH = 0x01,         /* ECDH Key Exchange */
	ACS_KEY_REC_KDF = 0x02,          /* KDF Key Exchange */
	ACS_KEY_REC_AES_128_CMAC = 0x03, /* AES-128-CMAC algorithm */
	ACS_KEY_REC_AES_128_CCM = 0x04,  /* AES-128-CCM algorithm */
	ACS_KEY_REC_AES_128_EAX = 0x05,  /* AES-128-EAX algorithm */
	ACS_KEY_REC_AES_128_GCM = 0x06,  /* AES-128-GCM algorithm */
	ACS_KEY_REC_AES_128_GMAC = 0x07  /* AES-128-GMAC algorithm */
};

/* Public key format values (Tables 4.40 and 4.41). */
enum acs_pub_key_format {
	ACS_PK_FMT_UNCOMPRESSED = 0x00, /* Uncompressed Plain */
	ACS_PK_FMT_X509_DER = 0x01      /* X.509 DER-encoded */
};

/* Elliptic_Curve field values (Table 4.42). */
enum acs_elliptic_curve {
	ACS_CURVE_P256 = 0x00, /* NIST P-256 (secp256r1) */
	ACS_CURVE_P384 = 0x01, /* NIST P-384 (secp384r1) */
	ACS_CURVE_P521 = 0x02, /* NIST P-521 (secp521r1) */
	ACS_CURVE_25519 = 0x03 /* Curve25519 */
};

/* Key Derivation Function types (Table 4.44). */
enum acs_kdf_type {
	ACS_KDF_SHA256 = 0x00,           /* HMAC-SHA-256 */
	ACS_KDF_SHA256_WITH_INFO = 0x01, /* HMAC-SHA-256, KDF_Info concatenated */
	ACS_KDF_SHA384 = 0x02,           /* HMAC-SHA-384 */
	ACS_KDF_SHA384_WITH_INFO = 0x03, /* HMAC-SHA-384, KDF_Info concatenated */
	ACS_KDF_SHA512 = 0x04,           /* HMAC-SHA-512 */
	ACS_KDF_SHA512_WITH_INFO = 0x05, /* HMAC-SHA-512, KDF_Info concatenated */
	ACS_KDF_RESERVED = 0x06          /* RFU, 0x06-0xFF */
};

/* Message_Type field values (Table 4.46). */
enum acs_message_type {
	ACS_MSG_TYPE_PROFILE_DEF = 0x00, /* Profile Defined Parameter */
	ACS_MSG_TYPE_PROTECTED = 0x01    /* Protected Resource Value */
};

/* Nonce_Type field values (Table 4.47). */
enum acs_nonce_type {
	ACS_NONCE_PROFILE_DEF = 0x00, /* Profile Defined Parameter */
	ACS_NONCE_SEQ_EVEN_ODD =
		0x01, /* Sequence Number Even-Odd (variable only, even/odd sequence) */
	ACS_NONCE_SEQ_DIFF_FIXED =
		0x02 /* Sequence Number Different Fixed Parts (fixed + variable) */
};

/* Compile-time key descriptor record. */
struct bt_acs_key_desc_record {
	uint8_t type_id; /* Type_ID (Table 4.36) */
	uint16_t key_id; /* Type_Value / Key_ID (Table 4.36) */
	union {
		struct {                       /* ECDH Key Exchange record data (Table 4.39) */
			uint8_t server_pk_fmt; /* AC_Server_Public_Key_Format (Table 4.40) */
			uint8_t client_pk_fmt; /* AC_Client_Public_Key_Format (Table 4.41) */
			uint8_t curve;         /* Elliptic_Curve (Table 4.42) */
			uint8_t kdf;           /* Key_Derivation_Function (Table 4.43) */
		} ecdh;

		struct {                        /* KDF Key Exchange record data */
			uint16_t parent_key_id; /* Parent Key_ID */
			uint8_t kdf_algorithm;  /* Key_Derivation_Function (Table 4.43) */
		} kdf;

		struct {                          /* AES algorithm record data (Table 4.45) */
			uint16_t parent_key_id;   /* Parent Key_ID */
			uint8_t msg_type;         /* Message_Type (Table 4.46) */
			uint8_t mac_size;         /* MAC_Size */
			uint8_t nonce_type;       /* Nonce_Type (Table 4.47), excluded for CMAC */
			uint8_t nonce_size;       /* Total nonce size used internally */
			uint8_t nonce_var_size;   /* Nonce_Variable_Size, excluded for CMAC */
			uint8_t nonce_fixed_size; /* Nonce_Fixed_Size, excluded for CMAC */
		} aes;
	};
};

/* Static key-descriptor record definition. */
#define BT_ACS_KEY_DESC_DEFINE(_name, ...)                                                         \
	STRUCT_SECTION_ITERABLE(bt_acs_key_desc_record, _name) = {__VA_ARGS__}

/* Build the Key Descriptor Response records selected by operand. */
int acs_key_desc_build_response(struct net_buf_simple *operand, struct net_buf *buf,
				struct bt_acs_conn *acs_conn);

/* Find a registered key descriptor by Key_ID, or return NULL. */
const struct bt_acs_key_desc_record *acs_key_desc_lookup(uint16_t key_id);

/* Check the key descriptor records and their runtime slots. */
int acs_key_desc_validate_records(void);

/* Return true when rec is an algorithm record. */
bool acs_key_desc_is_algorithm_record(const struct bt_acs_key_desc_record *rec);

/* Return true when rec carries nonce parameters. */
bool acs_key_desc_has_nonce_record(const struct bt_acs_key_desc_record *rec);

/* Return the Parent_Key_ID of rec, or 0 when it has no parent. */
uint16_t acs_key_desc_parent_key_id(const struct bt_acs_key_desc_record *rec);

/* Return the total nonce size of an algorithm record. */
static inline uint8_t acs_key_desc_nonce_size(const struct bt_acs_key_desc_record *rec)
{
	return rec->aes.nonce_size != 0U
		       ? rec->aes.nonce_size
		       : (uint8_t)(rec->aes.nonce_var_size + rec->aes.nonce_fixed_size);
}

/*
 * Nonce-prefix length: the advertised Nonce_Fixed_Value length for
 * DIFF_FIXED, 0 for a record without a nonce.
 */
static inline uint8_t acs_key_desc_nonce_prefix_size(const struct bt_acs_key_desc_record *rec)
{
	return rec->aes.nonce_type == ACS_NONCE_SEQ_DIFF_FIXED ? rec->aes.nonce_fixed_size : 0U;
}

/* Return the nonce variable-part size of an algorithm record. */
static inline uint8_t acs_key_desc_nonce_var_size(const struct bt_acs_key_desc_record *rec)
{
	return rec->aes.nonce_var_size;
}

/* Return the nonce fixed-part size of an algorithm record. */
static inline uint8_t acs_key_desc_nonce_fixed_size(const struct bt_acs_key_desc_record *rec)
{
	return rec->aes.nonce_fixed_size;
}

/* Return the authentication-tag size of an algorithm record. */
static inline uint8_t acs_key_desc_auth_tag_size(const struct bt_acs_key_desc_record *rec)
{
	return rec->aes.mac_size;
}

#endif /* BT_GATT_ACS_KEY_DESC_H_ */
