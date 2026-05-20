/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_KEY_DESC_H_
#define BT_GATT_ACS_KEY_DESC_H_

#include <zephyr/types.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/services/acs.h>

struct bt_acs_conn;

/* Special filter value: include all records (Table 4.34) */
#define BT_ACS_GET_KEY_DESC_ALL_RECORDS_FILTER 0xFFFF

/** TLV Header: Type(1) + KeyID(2) + DataSize(1) */
#define ACS_KEY_DESC_RECORD_HEADER_SIZE 4

/** ECDH Data: ServerFmt(1) + ClientFmt(1) + Curve(1) + KDF(1) */
#define ACS_KEY_DESC_ECDH_DATA_SIZE 4

/** AES with nonce: ParentID(2) + MsgType(1) + MAC(1) + NonceType(1) + VarSize(1) + FixSize(1) */
#define ACS_KEY_DESC_AES_ALG_MANDATORY_SIZE 7

/** AES-CMAC: ParentID(2) + MsgType(1) + MAC(1) — nonce fields excluded per Table 4.45 C.1 */
#define ACS_KEY_DESC_AES_CMAC_DATA_SIZE 4

/** OOB Data: OOB_Method(1) + ServerFmt(1) + ClientFmt(1) + Curve(1) + KDF(1) (Table 4.37) */
#define ACS_KEY_DESC_OOB_DATA_SIZE 5

/** KDF Data: Parent_Key_ID(2) + KDF_Algorithm(1) */
#define ACS_KEY_DESC_KDF_DATA_SIZE 3

/**
 * @brief Common TLV header carried by every key descriptor record (Table 4.36).
 *
 * Fields
 *   type_id    — record type (enum acs_key_record_type)
 *   type_value — this record's Key_ID, little-endian
 *   data_size  — byte length of the Data field that follows
 */
struct acs_key_rec_hdr {
	uint8_t type_id;
	uint16_t type_value; /* little-endian */
	uint8_t data_size;
} __packed;

/**
 * @brief ECDH Key Exchange record — full wire layout (Table 4.39).
 *
 * Data fields
 *   server_pk_fmt — AC_Server_Public_Key_Format (Table 4.40)
 *   client_pk_fmt — AC_Client_Public_Key_Format (Table 4.41)
 *   curve         — Elliptic_Curve              (Table 4.42)
 *   kdf           — Key_Derivation_Function     (Table 4.43)
 */
struct acs_key_rec_ecdh {
	struct acs_key_rec_hdr hdr;
	uint8_t server_pk_fmt;
	uint8_t client_pk_fmt;
	uint8_t curve;
	uint8_t kdf;
} __packed;

/**
 * @brief AES algorithm record — fixed header portion (Table 4.45).
 *
 * Applies to AES-128-CCM, AES-128-GCM, and other AES modes.
 * The variable-length AC_Server_Nonce_Fixed bytes (nonce_fixed_size of them)
 * follow immediately on the wire and must be appended separately.
 *
 * Data fields
 *   parent_key_id    — Key_ID of the key-exchange record (LE) (§4.4.4.15.1.4.4.1)
 *   msg_type         — Message_Type      (Table 4.46)
 *   mac_size         — MAC_Size
 *   nonce_type       — Nonce_Type        (Table 4.47)
 *   nonce_var_size   — Nonce_Variable_Size
 *   nonce_fixed_size — Nonce_Fixed_Size
 */
struct acs_key_rec_aes_alg_hdr {
	struct acs_key_rec_hdr hdr;
	uint16_t parent_key_id; /* little-endian */
	uint8_t msg_type;
	uint8_t mac_size;
	uint8_t nonce_type;
	uint8_t nonce_var_size;
	uint8_t nonce_fixed_size;
	/* uint8_t nonce_fixed[nonce_fixed_size] appended on wire */
} __packed;

/**
 * @brief OOB Key Exchange record — full wire layout (Table 4.37).
 *
 * Data fields
 *   oob_method    — OOB_Method                  (Table 4.38)
 *   server_pk_fmt — AC_Server_Public_Key_Format  (Table 4.40)
 *   client_pk_fmt — AC_Client_Public_Key_Format  (Table 4.41)
 *   curve         — Elliptic_Curve               (Table 4.42)
 *   kdf           — Key_Derivation_Function      (Table 4.43)
 */
struct acs_key_rec_oob {
	struct acs_key_rec_hdr hdr;
	uint8_t oob_method;
	uint8_t server_pk_fmt;
	uint8_t client_pk_fmt;
	uint8_t curve;
	uint8_t kdf;
} __packed;

/**
 * @brief KDF Key Exchange record — full wire layout.
 *
 * Data fields
 *   parent_key_id — Key_ID of the parent key-exchange record (LE)
 *   kdf_algorithm — Key_Derivation_Function (Table 4.43)
 */
struct acs_key_rec_kdf {
	struct acs_key_rec_hdr hdr;
	uint16_t parent_key_id; /* little-endian */
	uint8_t kdf_algorithm;
} __packed;

/*
 * Key IDs assigned by this AC Server (Table 4.36 / Section 4.4.4.15.1.2).
 *
 * Per spec, within a single key descriptor record the Data.Key_ID field
 * shall NOT equal the record's own Type_Value.  Algorithm records (CCM …)
 * and key-exchange records (ECDH, OOB …) therefore carry distinct IDs:
 *
 *  ISC.key_id = 0x0001  →  ACS_KEY_ID_CCM  (algorithm record)
 *  CCM.Data.Key_ID      →  ACS_KEY_ID_ECDH (key-exchange record)
 */
#define ACS_KEY_ID_ECDH 0x0001 /**< ECDH key-exchange record      */
#define ACS_KEY_ID_KDF  0x0002 /**< KDF key-exchange record       */
#define ACS_KEY_ID_GCM  0x0003 /**< AES-128-GCM algorithm record  */
#define ACS_KEY_ID_CCM  0x0004 /**< AES-128-CCM algorithm record  */
#define ACS_KEY_ID_OOB  0x0005 /**< OOB key-exchange record       */
#define ACS_KEY_ID_CMAC 0x0006 /**< AES-128-CMAC algorithm record */
#define ACS_KEY_ID_GMAC 0x0007 /**< AES-128-GMAC algorithm record */

/** @brief Maximum number of key-exchange Key_IDs (ECDH, OOB, KDF). */
#define ACS_KEY_ID_COUNT 3

/**
 * @brief Key Record Type_ID values for ACS Key Descriptor (Table 4.36)
 *
 * The Type_ID field defines the type of the record, which in turn defines the requirement of the
 * Data field.
 */
enum acs_key_record_type {
	ACS_KEY_REC_OOB = 0x00,          /**< OOB Key Exchange */
	ACS_KEY_REC_ECDH = 0x01,         /**< ECDH Key Exchange */
	ACS_KEY_REC_KDF = 0x02,          /**< KDF Key Exchange */
	ACS_KEY_REC_AES_128_CMAC = 0x03, /**< AES-128-CMAC algorithm */
	ACS_KEY_REC_AES_128_CCM = 0x04,  /**< AES-128-CCM algorithm */
	ACS_KEY_REC_AES_128_EAX = 0x05,  /**< AES-128-EAX algorithm */
	ACS_KEY_REC_AES_128_GCM = 0x06,  /**< AES-128-GCM algorithm */
	ACS_KEY_REC_AES_128_GMAC = 0x07  /**< AES-128-GMAC algorithm */
};

/**
 * @brief Public Key Format values for ACS Key Descriptor (Table 4.40 & 4.41)
 *
 * The AC_Server_Public_Key_Format field defines the public key format used by the AC Server.
 */
enum acs_pub_key_format {
	ACS_PK_FMT_UNCOMPRESSED = 0x00, /**< Uncompressed Plain */
	ACS_PK_FMT_X509_DER = 0x01      /**< X.509 DER-encoded */
};

/**
 * @brief Elliptic Curve field values for ACS Key Descriptor (Table 4.42)
 *
 * The Elliptic_Curve field defines the NIST-approved curves P-256, P-384, P-521, Curve25519,
 * and the ENISA-recommended key size elliptic curve used to generate the key.                   |
 */
enum acs_elliptic_curve {
	ACS_CURVE_P256 = 0x00, /**< NIST P-256 (secp256r1) */
	ACS_CURVE_P384 = 0x01, /**< NIST P-384 (secp384r1) */
	ACS_CURVE_P521 = 0x02, /**< NIST P-521 (secp521r1) */
	ACS_CURVE_25519 = 0x03 /**< Curve25519 */
};

/*
 * @brief Table 4.43: Key Derivation Function (KDF) Types. The KDF output length is fixed at 16
 * octets for all types. The "_WITH_INFO" variants indicate that the KDF_Info value is concatenated
 * to the shared secret input to the KDF, as specified in Section 4.7.2. The non-"_WITH_INFO"
 * variants indicate that the KDF_Info is not used and the KDF is applied directly to the shared
 * secret. The specific KDF algorithm used is HKDF based on the indicated hash function
 * (HMAC-SHA-256, HMAC-SHA-384, or HMAC-SHA-512).
 *
 */
enum acs_kdf_type {
	/** HKDF based on HMAC-SHA-256, 16 octets */
	ACS_KDF_SHA256 = 0x00,
	/* HKDF SHA-256, 16 octets, with KDF_Info concatenation */
	ACS_KDF_SHA256_WITH_INFO = 0x01,
	/* HKDF based on HMAC-SHA-384, 16 octets */
	ACS_KDF_SHA384 = 0x02,
	/* HKDF SHA-384, 16 octets, with KDF_Info concatenation */
	ACS_KDF_SHA384_WITH_INFO = 0x03,
	/* HKDF based on HMAC-SHA-512, 16 octets */
	ACS_KDF_SHA512 = 0x04,
	/* HKDF SHA-512, 16 octets, with KDF_Info concatenation */
	ACS_KDF_SHA512_WITH_INFO = 0x05,
	/* Reserved for Future Use (0x06–0xFF) */
	ACS_KDF_RESERVED = 0x06
};

/**
 * @brief Message Type field values for ACS Key Descriptor (Table 4.46)
 *
 * The Message_Type field defines the type of the variable length data to be encrypted and
 * authenticated.
 */
enum acs_message_type {
	ACS_MSG_TYPE_PROFILE_DEF = 0x00, /**< Profile Defined Parameter */
	ACS_MSG_TYPE_PROTECTED = 0x01    /**< Protected Resource Value */
};

/** @brief Nonce Type field values for ACS Key Descriptor (Table 4.47)
 *
 * The Nonce_Type field defines the type of the nonce used in the nonce variable and fixed part with
 * the algorithm.
 */
enum acs_nonce_type {
	ACS_NONCE_PROFILE_DEF = 0x00, /**< Profile Defined Parameter */
	ACS_NONCE_SEQ_EVEN_ODD =
		0x01, /**< Sequence Number Even-Odd (variable only, even/odd sequence) */
	ACS_NONCE_SEQ_DIFF_FIXED =
		0x02 /**< Sequence Number Different Fixed Parts (fixed + variable) */
};

/**
 * @brief OOB Method field values for ACS Key Descriptor (Table 4.38)
 *
 * The OOB_Method field defines the method used to get the AC Server key out-of-band.
 */
enum acs_oob_method {
	ACS_OOB_METHOD_MANUFACTURER = 0x00, /**< Manufacturer-specific */
	ACS_OOB_METHOD_URI = 0x01,          /**< URI */
	ACS_OOB_METHOD_2D_CODE = 0x02,      /**< 2D Machine-readable Code */
	ACS_OOB_METHOD_BAR_CODE = 0x03,     /**< Bar Code */
	ACS_OOB_METHOD_NFC = 0x04           /**< NFC */
};

/**
 * @brief Parses the operand and builds the Key Descriptor Response payload.
 * @param operand              The request operand buffer (filter pulled via net_buf_simple).
 * @param buf                  Buffer to store the generated response TLV records.
 * @param server_fixed_nonce   The fixed part of the server nonce to include in the response if
 *                             required by the key descriptor record (otherwise can be NULL).
 * @return 0 on success, negative error code on failure.
 * @return -EINVAL if the operand format is invalid
 * @return -ENOENT if no matching records were found
 * @return -ENOMEM if the provided buffer does not have enough space for the response
 */
int acs_key_desc_build_response(struct net_buf_simple *operand, struct net_buf_simple *buf,
				struct bt_acs_conn *acs_conn);

/**
 * @brief Look up a key descriptor record by Key ID.
 *
 * Scans the bt_acs_key_desc_record iterable section for a record whose
 * @c key_id equals @p key_id.
 *
 * @param key_id  Key ID to search for.
 *
 * @retval pointer to the matching record, or NULL if not found.
 */
const struct bt_acs_key_desc_record *acs_key_desc_lookup(uint16_t key_id);

bool acs_key_desc_is_algorithm_record(const struct bt_acs_key_desc_record *rec);
bool acs_key_desc_has_nonce_record(const struct bt_acs_key_desc_record *rec);
uint16_t acs_key_desc_parent_key_id(const struct bt_acs_key_desc_record *rec);
uint8_t acs_key_desc_nonce_size(const struct bt_acs_key_desc_record *rec);
uint8_t acs_key_desc_nonce_var_size(const struct bt_acs_key_desc_record *rec);
uint8_t acs_key_desc_nonce_fixed_size(const struct bt_acs_key_desc_record *rec);
uint8_t acs_key_desc_auth_tag_size(const struct bt_acs_key_desc_record *rec);

#endif /* BT_GATT_ACS_KEY_DESC_H_ */
