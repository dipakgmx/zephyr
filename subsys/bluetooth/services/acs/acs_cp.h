/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef BT_GATT_ACS_CP_H
#define BT_GATT_ACS_CP_H

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "acs_crypto_config.h"

/**
 * @brief ACS Feature Response wire structure (Table 4.59).
 *
 * Packed wire layout returned in the Feature Response operand.  All multi-byte
 * fields are little-endian on the wire.
 */
struct bt_acs_feature_rsp {
	uint32_t features;                      /**< Feature bits (Table 4.60) */
	uint16_t protection_methods;            /**< Protection methods (Table 4.61) */
	uint16_t oob_key_exchange_capabilities; /**< OOB key exchange caps (Table 4.62) */
	uint16_t confirmation_static_oob_number_capabilities; /**< Static OOB number caps */
	uint32_t confirmation_input_oob_number_max_value;     /**< Input OOB number max value */
	uint16_t confirmation_input_oob_number_capabilities;  /**< Input OOB caps (Table 4.63) */
	uint32_t confirmation_output_oob_number_max_value;    /**< Output OOB number max value */
	uint16_t confirmation_output_oob_number_capabilities; /**< Output OOB caps (Table 4.64) */
} __packed;

/**
 * @defgroup acs_feature_bits ACS Feature Response - Features field bit definitions
 * @brief Bit definitions for the Features field of struct bt_acs_feature_rsp (Table 4.60).
 *
 * Each bit represents a specific optional or conditional feature supported by the AC Server.
 * @{
 */
#define BT_ACS_FEATURE_SET_SECURITY_CONTROLS_SWITCH_SUPPORTED                                      \
	BIT(0) /**< Set Security Controls Switch Supported: AC Server supports turning on/off the  \
		  security controls switch. */
#define BT_ACS_FEATURE_DESCRIPTORS_SUPPORTED                                                       \
	BIT(1) /**< Descriptors Supported: AC Server supports access to descriptors of ACS         \
		  functionality. */
#define BT_ACS_FEATURE_MULTIPLE_RESTRICTION_MAPS_SUPPORTED                                         \
	BIT(2) /**< Multiple Restriction Maps Supported: AC Server supports more than one          \
		  restriction map. */
#define BT_ACS_FEATURE_RESOURCE_HANDLE_TO_UUID_MAP_SUPPORTED                                       \
	BIT(3) /**< Resource Handle To UUID Map Supported: AC Server supports mapping Resource     \
		  Handle to Attribute UUID. */
#define BT_ACS_FEATURE_INITIATION_OF_PAIRING_SUPPORTED                                             \
	BIT(4) /**< Initiation of Pairing Supported: AC Client can initiate pairing on the AC      \
		  Server. */
#define BT_ACS_FEATURE_OOB_KEY_EXCHANGE_SUPPORTED                                                  \
	BIT(5) /**< OOB Key Exchange Supported: AC Server supports exchanging keys using OOB. */
#define BT_ACS_FEATURE_ECDH_KEY_EXCHANGE_SUPPORTED                                                 \
	BIT(6) /**< ECDH Key Exchange Supported: AC Server supports exchanging keys using ECDH. */
#define BT_ACS_FEATURE_KDF_KEY_EXCHANGE_SUPPORTED                                                  \
	BIT(7) /**< KDF Key Exchange Supported: AC Server supports exchanging keys using KDF. */
#define BT_ACS_FEATURE_INVALIDATE_ESTABLISHED_SECURITY_SUPPORTED                                   \
	BIT(9) /**< Invalidate Established Security Supported: AC Server supports invalidating     \
		  established security. */
#define BT_ACS_FEATURE_ATT_MTU_SUPPORTED                                                           \
	BIT(10) /**< ATT_MTU Supported: AC Server supports providing the ATT_MTU. */
#define BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_WRITE_REQUEST                                       \
	BIT(11) /**< Protected Resource Uses Write Request: At least one protected resource        \
		   supports write requests. */
#define BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_READ_REQUEST                                        \
	BIT(12) /**< Protected Resource Uses Read Request: At least one protected resource         \
		   supports read requests. */
#define BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_NOTIFICATION                                        \
	BIT(13) /**< Protected Resource Uses Notification: At least one protected resource         \
		   supports notifications. */
#define BT_ACS_FEATURE_PROTECTED_RESOURCE_USES_INDICATION                                          \
	BIT(14) /**< Protected Resource Uses Indication: At least one protected resource supports  \
		   indications. */
#define BT_ACS_FEATURE_KEY_FORMAT_AC_SERVER_MANUFACTURER_SPECIFIC_SUPPORTED                        \
	BIT(15) /**< Key Format AC Server Manufacturer-specific Supported: Manufacturer-specific   \
		   format for AC Server public key. */
#define BT_ACS_FEATURE_KEY_FORMAT_AC_CLIENT_MANUFACTURER_SPECIFIC_SUPPORTED                        \
	BIT(16) /**< Key Format AC Client Manufacturer-specific Supported: Manufacturer-specific   \
		   format for AC Client public key. */
#define BT_ACS_FEATURE_KEY_FORMAT_AC_SERVER_UNCOMPRESSED_PLAIN_SUPPORTED                           \
	BIT(17) /**< Key Format AC Server Uncompressed Plain Supported: Uncompressed plain format  \
		   for AC Server public key. */
#define BT_ACS_FEATURE_KEY_FORMAT_AC_CLIENT_UNCOMPRESSED_PLAIN_SUPPORTED                           \
	BIT(18) /**< Key Format AC Client Uncompressed Plain Supported: Uncompressed plain format  \
		   for AC Client public key. */
#define BT_ACS_FEATURE_KEY_FORMAT_AC_SERVER_X509_SUPPORTED                                         \
	BIT(19) /**< Key Format AC Server X.509 Supported: X.509 format for AC Server public key.  \
		 */
#define BT_ACS_FEATURE_KEY_FORMAT_AC_CLIENT_X509_SUPPORTED                                         \
	BIT(20) /**< Key Format AC Client X.509 Supported: X.509 format for AC Client public key.  \
		 */
/* Bits 21–31: Reserved for Future Use. */
/** @} */

/**
 * @defgroup acs_protection_methods_bits ACS Feature Response - Protection_Methods field bit
 * definitions
 * @brief Bit definitions for the Protection_Methods field of struct bt_acs_feature_rsp
 * (Table 4.61).
 *
 * Each bit represents a specific information security control supported by the AC Server.
 * @{
 */
#define BT_ACS_PROTECTION_CONFIDENTIALITY_SUPPORTED                                                \
	BIT(0) /**< Confidentiality Supported: AC Server supports confidentiality using encryption \
		  information security controls. */
#define BT_ACS_PROTECTION_INTEGRITY_SUPPORTED                                                      \
	BIT(1) /**< Integrity Supported: AC Server supports integrity information security         \
		  controls. */
#define BT_ACS_PROTECTION_AUTHENTICATION_SUPPORTED                                                 \
	BIT(2) /**< Authentication Supported: AC Server supports authentication information        \
		  security controls. */
#define BT_ACS_PROTECTION_AUTHORIZATION_SUPPORTED                                                  \
	BIT(3) /**< Authorization Supported: AC Server supports authorization information security \
		  controls. */
#define BT_ACS_PROTECTION_NON_REPUDIATION_SUPPORTED                                                \
	BIT(4) /**< Non-repudiation Supported: AC Server supports non-repudiation information      \
		  security controls (audit logging defined by higher-level profile). */
#define BT_ACS_PROTECTION_MANUFACTURER_SPECIFIC_CONTROLS_USED                                      \
	BIT(5) /**< Manufacturer-specific Controls Used: AC Server uses manufacturer-specific      \
		  information security controls. */
/* Bits 6–15: Reserved for Future Use. */
/** @} */

/**
 * @defgroup acs_oob_key_exchange_capabilities_bits ACS Feature Response -
 * OOB_Key_Exchange_Capabilities field bit definitions
 * @brief Bit definitions for the OOB_Key_Exchange_Capabilities field of struct bt_acs_feature_rsp
 * (Table 4.62).
 *
 * Each bit represents a specific OOB key exchange capability supported by the AC Server.
 * @{
 */
#define BT_ACS_OOB_KEY_EXCHANGE_OTHER                                                              \
	BIT(0) /**< Other: Manufacturer-specific key exchange method not listed (e.g., embedded    \
		  key). */
#define BT_ACS_OOB_KEY_EXCHANGE_URI BIT(1) /**< URI: Key may be exchanged using a URI. */
#define BT_ACS_OOB_KEY_EXCHANGE_2D_CODE                                                            \
	BIT(2) /**< 2D Machine-readable Code: Key may be exchanged using 2D code. */
#define BT_ACS_OOB_KEY_EXCHANGE_BAR_CODE                                                           \
	BIT(3) /**< Bar Code: Key may be exchanged using a bar code. */
#define BT_ACS_OOB_KEY_EXCHANGE_NFC                                                                \
	BIT(4) /**< NFC: Key may be exchanged using near-field communication. */
#define BT_ACS_OOB_KEY_EXCHANGE_NUMBER                                                             \
	BIT(5) /**< Number: Key may be exchanged in the format of a number. */
#define BT_ACS_OOB_KEY_EXCHANGE_STRING                                                             \
	BIT(6) /**< String: Key may be exchanged in the format of a string. */
#define BT_ACS_OOB_KEY_EXCHANGE_X509_CERT                                                          \
	BIT(7) /**< X.509 Certificate: Key may be exchanged in the format of an X.509 certificate. \
		*/
/* Bits 8–10: Reserved for Future Use. */
#define BT_ACS_OOB_KEY_EXCHANGE_ON_BOX                                                             \
	BIT(11) /**< On Box: Key may be exchanged using information provided on the box. */
#define BT_ACS_OOB_KEY_EXCHANGE_INSIDE_BOX                                                         \
	BIT(12) /**< Inside Box: Key may be exchanged using information provided inside the box.   \
		 */
#define BT_ACS_OOB_KEY_EXCHANGE_ON_PAPER                                                           \
	BIT(13) /**< On Piece Of Paper: Key may be exchanged using information provided on a piece \
		   of paper. */
#define BT_ACS_OOB_KEY_EXCHANGE_INSIDE_MANUAL                                                      \
	BIT(14) /**< Inside Manual: Key may be exchanged using information provided inside the     \
		   manual. */
#define BT_ACS_OOB_KEY_EXCHANGE_ON_DEVICE                                                          \
	BIT(15) /**< On Device: Key may be exchanged using information provided on the device. */
/** @} */

/**
 * @defgroup acs_confirmation_input_oob_capabilities_bits ACS Feature Response -
 * Confirmation_Input_OOB_Number_Capabilities field bit definitions
 * @brief Bit definitions for the Confirmation_Input_OOB_Number_Capabilities field of struct
 * bt_acs_feature_rsp (Table 4.63).
 *
 * Each bit represents a specific input OOB action capability supported by the AC Server.
 * @{
 */
#define BT_ACS_CONFIRMATION_INPUT_OOB_PUSH                                                         \
	BIT(0) /**< Push: AC Server can receive a push input (user pushes UI control a specific    \
		  number of times). */
/* Bit 1: Reserved for Future Use. */
#define BT_ACS_CONFIRMATION_INPUT_OOB_INPUT_NUMERIC                                                \
	BIT(2) /**< Input Numeric: AC Server can receive a numeric value (typed or spoken sequence \
		  of digits). */
/* Bits 3–15: Reserved for Future Use. */
/** @} */

/**
 * @defgroup acs_confirmation_output_oob_capabilities_bits ACS Feature Response -
 * Confirmation_Output_OOB_Number_Capabilities field bit definitions
 * @brief Bit definitions for the Confirmation_Output_OOB_Number_Capabilities field of struct
 * bt_acs_feature_rsp (Table 4.64).
 *
 * Each bit represents a specific output OOB action capability supported by the AC Server.
 * @{
 */
/* Bit 0: Reserved for Future Use. */
#define BT_ACS_CONFIRMATION_OUTPUT_OOB_BEEP                                                        \
	BIT(1) /**< Beep: AC Server can beep so the user can count the number of beeps. */
/* Bit 2: Reserved for Future Use. */
#define BT_ACS_CONFIRMATION_OUTPUT_OOB_OUTPUT_NUMERIC                                              \
	BIT(3) /**< Output Numeric: AC Server can provide a numeric value (displayed or spoken)    \
		  for the user. */
/* Bits 4–15: Reserved for Future Use. */
/** @} */

/**
 * @brief Wire-format operand of the Key Exchange KDF opcode (Table 4.75, §4.4.4.34).
 *
 * Written by the AC Client to initiate a KDF step.  Key_ID must match the value
 * provided in the preceding Start Key Exchange request (§4.4.4.34.1).
 *
 * On-wire layout (LSO → MSO):
 *   Offset  Size  Field
 *     0       2   Key_ID (uint16, LE)
 *
 * The KDF Response operand (Table 4.76) has variable-length KDF_Salt and KDF_Info
 * fields and is therefore built with net_buf helpers rather than a flat struct.
 * Its layout is documented in acs_key_exchange_kdf() (acs_key_exchange.c).
 */
struct acs_kdf_req {
	uint16_t key_id; /**< Key_ID: references the KDF key descriptor (uint16, LSO…MSO) */
} __packed;

/**
 * @brief Wire-format operand of the ACTIVATE_RESTRICTION_MAP opcode (§4.4.3.4).
 */
struct acs_cp_activate_restriction_map_req {
	uint16_t map_id; /**< Restriction_Map_ID (uint16, LSO…MSO) */
} __packed;

/**
 * @brief Wire-format operand of the GET_SERVICE_CHARACTERISTIC_UUIDS... opcode (§4.4.3.6).
 */
struct acs_cp_get_svc_char_uuids_req {
	uint16_t resource_handle; /**< Characteristic Resource Handle (uint16, LSO…MSO) */
} __packed;

/**
 * @brief Wire-format operand of the GET_KEY_DESCRIPTOR opcode (§4.4.3.8).
 */
struct acs_cp_get_key_descriptor_req {
	uint16_t filter_id; /**< Key_ID filter, or all-records filter (uint16, LSO…MSO) */
} __packed;

/**
 * @brief Wire-format operand of the GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR opcode.
 */
struct acs_cp_get_isc_descriptor_req {
	uint16_t filter_id; /**< ISC_ID filter, or all-records filter (uint16, LSO…MSO) */
} __packed;

/**
 * @brief Wire-format operand of the START_KEY_EXCHANGE opcode (§4.4.3.10, Table 4.49.)
 *
 */
struct acs_cp_start_key_exchange_req {
	uint16_t key_id;             /**< LE */
	uint8_t confirmation_method; /**< Selected_Confirmation_Method (Table 4.50) */
	uint8_t confirmation_action; /**< Selected_Confirmation_Action (§4.4.4.18.3,
					Tables 4.51/4.52) */
} __packed;

/**
 * Size in octets of the AC_Client_Confirmation_Code and
 * AC_Client_Confirmation_Random_Number wire fields (§4.4.4.30.2, §4.4.4.32.2).
 * The spec fixes both at 32 octets, independent of the hash algorithm.
 */
#define ACS_CONFIRM_VALUE_SIZE 32

/**
 * @brief Wire-format operand of the ECDH_CONFIRM_CODE opcode (§4.4.4.30, Table 4.71).
 *
 */
struct acs_cp_ecdh_confirm_code_req {
	uint16_t key_id;                              /**< Key_ID (uint16, LSO…MSO) */
	uint8_t confirm_code[ACS_CONFIRM_VALUE_SIZE]; /**< AC_Client_Confirmation_Code (32 octets,
							 LSO…MSO) */
} __packed;

/**
 * @brief Wire-format operand of the ECDH_CONFIRM_RAND opcode (§4.4.4.32, Table 4.73).
 *
 */
struct acs_cp_ecdh_confirm_rand_req {
	uint16_t key_id;                        /**< Key_ID (uint16, LSO…MSO) */
	uint8_t random[ACS_CONFIRM_VALUE_SIZE]; /**< AC_Client_Confirmation_Random_Number (32
						   octets, LSO…MSO) */
} __packed;

/** SET_CLIENT_NONCE_FIXED operand - Key_ID followed by variable-length nonce bytes.
 *
 * The AC_Client_Nonce_Fixed_Value length is descriptor-defined, not globally fixed:
 * the handler resolves the operand size from the referenced Key Descriptor record.
 */
struct acs_cp_set_client_nonce_fixed_req {
	uint16_t key_id;           /**< Key_ID (uint16, LSO…MSO) */
	uint8_t nonce_fixed_val[]; /**< AC_Client_Nonce_Fixed_Value */
} __packed;

BUILD_ASSERT(sizeof(struct acs_cp_start_key_exchange_req) == 4,
	     "START_KEY_EXCHANGE operand struct size mismatch");
BUILD_ASSERT(sizeof(struct acs_cp_activate_restriction_map_req) == 2,
	     "ACTIVATE_RESTRICTION_MAP operand struct size mismatch");
BUILD_ASSERT(sizeof(struct acs_cp_get_svc_char_uuids_req) == 2,
	     "GET_SERVICE_CHARACTERISTIC_UUIDS operand struct size mismatch");
BUILD_ASSERT(sizeof(struct acs_cp_get_key_descriptor_req) == 2,
	     "GET_KEY_DESCRIPTOR operand struct size mismatch");
BUILD_ASSERT(sizeof(struct acs_cp_get_isc_descriptor_req) == 2,
	     "GET_ISC_DESCRIPTOR operand struct size mismatch");
BUILD_ASSERT(sizeof(struct acs_cp_set_client_nonce_fixed_req) == 2,
	     "SET_CLIENT_NONCE_FIXED operand header size mismatch");
BUILD_ASSERT(sizeof(struct acs_cp_ecdh_confirm_code_req) == 2 + ACS_CONFIRM_VALUE_SIZE,
	     "ECDH_CONFIRM_CODE operand struct size mismatch");
BUILD_ASSERT(sizeof(struct acs_cp_ecdh_confirm_rand_req) == 2 + ACS_CONFIRM_VALUE_SIZE,
	     "ECDH_CONFIRM_RAND operand struct size mismatch");
/**
 * @brief Selected_Confirmation_Method field values (Table 4.50)
 *
 * Identifies how the exchanged key is confirmed during an ECDH key exchange.
 */
enum bt_acs_confirmation_method {
	/** No Confirmation OOB Method Used - AuthValue = 0x00...00, not authenticated */
	BT_ACS_CONFIRM_METHOD_NONE = 0x00,
	/** Confirmation Output OOB Number Action Used - server generates and outputs a number */
	BT_ACS_CONFIRM_METHOD_OUTPUT_OOB = 0x01,
	/** Confirmation Input OOB Number Action Used - client generates a number, user enters into
	 * server */
	BT_ACS_CONFIRM_METHOD_INPUT_OOB = 0x02,
	/** Confirmation Static OOB Number Used - pre-shared static value */
	BT_ACS_CONFIRM_METHOD_STATIC_OOB = 0x03,
};

/**
 * @brief Selected_Confirmation_Action field output action values (Table 4.51)
 *
 * Used when Selected_Confirmation_Method is BT_ACS_CONFIRM_METHOD_OUTPUT_OOB.
 * All other values (0x00, 0x02, 0x04–0xFE) are RFU; 0xFF is Prohibited.
 */
enum bt_acs_confirmation_action_output {
	/** Beep - user confirms by hearing a beep from the server */
	BT_ACS_CONFIRM_ACTION_OUTPUT_BEEP = 0x01,
	/** Output Numeric - server outputs a numeric value for user confirmation */
	BT_ACS_CONFIRM_ACTION_OUTPUT_NUMERIC = 0x03,
};

/**
 * @brief Selected_Confirmation_Action field input action values (Table 4.52)
 *
 * Used when Selected_Confirmation_Method is BT_ACS_CONFIRM_METHOD_INPUT_OOB.
 * All other values (0x01, 0x03–0xFE) are RFU; 0xFF is Prohibited.
 */
enum bt_acs_confirmation_action_input {
	/** Push - user confirms by pushing a control on the server */
	BT_ACS_CONFIRM_ACTION_INPUT_PUSH = 0x00,
	/** Input Numeric - user enters a numeric value into the server */
	BT_ACS_CONFIRM_ACTION_INPUT_NUMERIC = 0x02,
};

/**
 * @brief Selected_Confirmation_Action value for non-OOB methods (§4.4.4.18.3)
 *
 * Required when Selected_Confirmation_Method is BT_ACS_CONFIRM_METHOD_NONE
 * (no user action nor static confirmation) or BT_ACS_CONFIRM_METHOD_STATIC_OOB
 * (static confirmation). Both cases use the same wire value 0xFF.
 */
#define BT_ACS_CONFIRM_ACTION_NOT_APPLICABLE 0xFF

/**
 * @brief ACS Control Point Response Code values (§4.4.4.1, Table 4.4).
 *
 * Sent by the AC Server in the Response_Code_Value field of a Response Code indication.
 * All values 0x0B–0xFF are Reserved for Future Use.
 */
enum bt_acs_cp_response_code {
	BT_ACS_CP_RESPONSE_RFU = 0x00,                  /**< Reserved for Future Use */
	BT_ACS_CP_RESPONSE_SUCCESS = 0x01,              /**< Procedure completed successfully */
	BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED = 0x02, /**< Unsupported opcode received */
	BT_ACS_CP_RESPONSE_INVALID_OPERAND =
		0x03, /**< Operand does not meet service requirements */
	BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED = 0x04, /**< Procedure could not be completed */
	BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE = 0x05,  /**< Operand out of allowed range */
	BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE =
		0x06, /**< Procedure not applicable in current context */
	BT_ACS_CP_RESPONSE_ABORT_UNSUCCESSFUL = 0x07, /**< Abort request could not be completed */
	BT_ACS_CP_RESPONSE_NO_RECORDS_FOUND =
		0x08, /**< No descriptors match the filter criterion */
	BT_ACS_CP_RESPONSE_INVALID_KEY_EXCHANGE_CONFIRMATION_CODE =
		0x09, /**< AC Client confirmation code validation failed */
	BT_ACS_CP_RESPONSE_INVALID_PUBLIC_KEY =
		0x0A,                        /**< AC Client public key is not on the curve */
	BT_ACS_CP_RESPONSE_RFU_RANGE = 0x0B, /**< Reserved for Future Use (0x0B–0xFF) */
};

/**
 * @name CP opcode handler result contract
 *
 * Handlers validate and build but never transmit or stage; for data-response
 * opcodes the dispatcher stages @c reply->response with the response opcode
 * before the handler runs, and the handler appends its operand bytes there.
 * A handler returns @ref ACS_CP_RESULT_STAGED_REPLY once the response is built,
 * a @ref bt_acs_cp_response_code to send a Response Code indication, a negative
 * errno (mapped via errno_to_acs_status()), or @ref ACS_CP_RESULT_NO_REPLY to
 * send nothing (deferred Abort). Multistep opcodes additionally set
 * @c reply->step; follow-up indications are driven by the continuation engine
 * in acs_reply.c.
 *
 * The domains cannot collide: Response Code 0x00 is spec-reserved (RFU) and
 * the wire field is one octet, so neither sentinel is a sendable code.
 * @{
 */
#define ACS_CP_RESULT_STAGED_REPLY 0     /**< Reply staged in the reply's response buffer */
#define ACS_CP_RESULT_NO_REPLY     0x100 /**< Suppress any response for now */
/** @} */

#endif /* BT_GATT_ACS_CP_H */
