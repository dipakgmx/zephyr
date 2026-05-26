/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_ACS_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_ACS_H_

/**
 * @brief Authorization Control Service (ACS)
 * @defgroup bt_acs Authorization Control Service (ACS)
 * @ingroup bluetooth
 * @{
 */

#include <stdint.h>
#include <stdbool.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/iterable_sections.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief ACS Status flags */
enum bt_acs_status_flag {
	/** Security controls are enabled on this server */
	BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED = BIT(0),
	/** Security has been established for this connection */
	BT_ACS_STATUS_SECURITY_ESTABLISHED = BIT(1),
};

/**
 * @brief ACS Control Point opcodes (requests and responses)
 * @anchor bt_acs_cp_opcode
 */
enum bt_acs_cp_opcode {
	/** General response from the ACS CP to state procedure errors or success. */
	BT_ACS_CP_OPCODE_RESPONSE_CODE = 0x00,
	/** Triggers reporting of all active ACS descriptors in a single request. */
	BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS = 0x01,
	/** Gets the restriction map based on restriction map ID and filter criterion. */
	BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR = 0x02,
	/** Used by the AC Server to provide a restriction map. */
	BT_ACS_CP_OPCODE_RESTRICTION_MAP_DESCRIPTOR_RESPONSE = 0x03,
	/** Gets a list of restriction map IDs for all available restriction maps. */
	BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST = 0x04,
	/** Used by the AC Server to provide the restriction map ID list. */
	BT_ACS_CP_OPCODE_RESTRICTION_MAP_ID_LIST_RESPONSE = 0x05,
	/** Activates the restriction map identified by the included restriction map ID. */
	BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP = 0x06,
	/** Gets the map between Resource Handles and UUIDs. */
	BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP = 0x07,
	/** Used by the AC Server to provide the Resource Handle to UUID map. */
	BT_ACS_CP_OPCODE_RESOURCE_HANDLE_UUID_MAP_RESPONSE = 0x08,
	/** Gets the service and characteristic UUIDs for a requested resource handle. */
	BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE = 0x09,
	/** Used by the AC Server to provide the service and characteristic UUIDs for the requested
	 * resource handle. */
	BT_ACS_CP_OPCODE_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE_RESPONSE = 0x0A,
	/** Gets the information security configurations based on filter criterion. */
	BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR = 0x0B,
	/** Used by the AC Server to provide the information security configurations. */
	BT_ACS_CP_OPCODE_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR_RESPONSE = 0x0C,
	/** Requests the supported key exchange methods based on filter criterion. */
	BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR = 0x0D,
	/** Used by the AC Server to provide the supported key exchange methods. */
	BT_ACS_CP_OPCODE_KEY_DESCRIPTOR_RESPONSE = 0x0E,
	/** Requests the list of key IDs that are valid. */
	BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST = 0x0F,
	/** Used by the AC Server to provide a list of the current keys. */
	BT_ACS_CP_OPCODE_CURRENT_KEY_LIST_RESPONSE = 0x10,
	/** Requests the start of key exchange for the included key ID. */
	BT_ACS_CP_OPCODE_START_KEY_EXCHANGE = 0x11,
	/** Used by the AC Server to provide the results of the key exchange. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_RESPONSE = 0x12,
	/** Invalidates all of the established security for all AC Clients. */
	BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY = 0x13,
	/** Invalidates one or more keys for the requesting AC Client. */
	BT_ACS_CP_OPCODE_INVALIDATE_KEY = 0x14,
	/** Stops any ACS CP procedure that is in progress. */
	BT_ACS_CP_OPCODE_ABORT = 0x15,
	/** Sets the state of the security controls switch. */
	BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH = 0x16,
	/** Requests the URI that can be used to get the AC Server key. */
	BT_ACS_CP_OPCODE_GET_KEY_URI = 0x17,
	/** Used by the AC Server to provide the key URI. */
	BT_ACS_CP_OPCODE_KEY_URI_RESPONSE = 0x18,
	/** Requests the features and capabilities supported by the AC Server. */
	BT_ACS_CP_OPCODE_GET_FEATURE = 0x19,
	/** Used by the AC Server to provide the supported features and capabilities. */
	BT_ACS_CP_OPCODE_ACS_FEATURE_RESPONSE = 0x1A,
	/** Requests the exchange of public keys as part of the ECDH key agreement scheme. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH = 0x1B,
	/** Used by the AC Server to provide the AC Server public key as part of the ECDH key
	 * agreement scheme. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_RESPONSE = 0x1C,
	/** Requests the exchange of confirmation codes for ECDH key agreement. */
	BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE = 0x1D,
	/** Used by the AC Server to provide the AC Server confirmation code for ECDH. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_CODE_RESPONSE = 0x1E,
	/** Requests the exchange of random numbers for ECDH confirmation code calculation. */
	BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND = 0x1F,
	/** Used by the AC Server to provide the AC Server random number for ECDH. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_RANDOM_NUMBER_RESPONSE = 0x20,
	/** Requests the exchange of keys using KDF or as part of ECDH. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF = 0x21,
	/** Used by the AC Server to provide the KDF parameters. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF_RESPONSE = 0x22,
	/** Sets the fixed part of the AC Client nonce. */
	BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED = 0x23,
	/** Reserved for future use (0x24 - 0xDC). */
	BT_ACS_CP_OPCODE_RFU = 0x24,
	/** Requests the negotiated ATT_MTU size for communication optimization. */
	BT_ACS_CP_OPCODE_ATT_MTU = 0xDD,
	/** Used by the AC Server to provide the negotiated ATT_MTU. */
	BT_ACS_CP_OPCODE_ATT_MTU_RESPONSE = 0xDE,
	/** Requests the initiation of the pairing procedure by the AC Server. */
	BT_ACS_CP_OPCODE_INITIATE_PAIRING = 0xDF,
	/** Manufacturer-specific opcodes (0xE0 - 0xFF). */
	BT_ACS_CP_OPCODE_MANUFACTURER_SPECIFIC = 0xE0,
};

/** @brief ACS data direction for enforcement checks */
enum bt_acs_direction {
	BT_ACS_DIRECTION_READ,
	BT_ACS_DIRECTION_WRITE,
	BT_ACS_DIRECTION_NOTIFY,
	BT_ACS_DIRECTION_INDICATE,
};

/**
 * @name ATT opcodes for use in restriction map entries (Type_ID 0x02).
 *
 * These are standard ATT opcodes from the Bluetooth Core Specification
 * (Vol 3, Part F, Section 3.4), zero-padded to uint16 as required by the
 * ACS restriction map Data field (spec §4.4.4.4.1.3).
 *
 * Only client-to-server request/command opcodes are meaningful in a
 * restriction map entry; response opcodes are listed for completeness.
 * @{
 */

/* --- Attribute discovery --- */
#define BT_ACS_RMAP_OP_ATT_FIND_INFO_REQ  0x0004u /**< Find Information Request */
#define BT_ACS_RMAP_OP_ATT_FIND_INFO_RSP  0x0005u /**< Find Information Response */
#define BT_ACS_RMAP_OP_ATT_FIND_TYPE_REQ  0x0006u /**< Find By Type Value Request */
#define BT_ACS_RMAP_OP_ATT_FIND_TYPE_RSP  0x0007u /**< Find By Type Value Response */
#define BT_ACS_RMAP_OP_ATT_READ_TYPE_REQ  0x0008u /**< Read By Type Request */
#define BT_ACS_RMAP_OP_ATT_READ_TYPE_RSP  0x0009u /**< Read By Type Response */
#define BT_ACS_RMAP_OP_ATT_READ_GROUP_REQ 0x0010u /**< Read By Group Type Request */
#define BT_ACS_RMAP_OP_ATT_READ_GROUP_RSP 0x0011u /**< Read By Group Type Response */

/* --- Read operations --- */
#define BT_ACS_RMAP_OP_ATT_READ_REQ         0x000Au /**< Read Request */
#define BT_ACS_RMAP_OP_ATT_READ_RSP         0x000Bu /**< Read Response */
#define BT_ACS_RMAP_OP_ATT_READ_BLOB_REQ    0x000Cu /**< Read Blob Request (long reads) */
#define BT_ACS_RMAP_OP_ATT_READ_BLOB_RSP    0x000Du /**< Read Blob Response */
#define BT_ACS_RMAP_OP_ATT_READ_MULT_REQ    0x000Eu /**< Read Multiple Request */
#define BT_ACS_RMAP_OP_ATT_READ_MULT_RSP    0x000Fu /**< Read Multiple Response */
#define BT_ACS_RMAP_OP_ATT_READ_MULT_VL_REQ 0x0020u /**< Read Multiple Variable Length Request */
#define BT_ACS_RMAP_OP_ATT_READ_MULT_VL_RSP 0x0021u /**< Read Multiple Variable Length Response */

/* --- Write operations --- */
#define BT_ACS_RMAP_OP_ATT_WRITE_REQ        0x0012u /**< Write Request */
#define BT_ACS_RMAP_OP_ATT_WRITE_RSP        0x0013u /**< Write Response */
#define BT_ACS_RMAP_OP_ATT_WRITE_CMD        0x0052u /**< Write Command (no response) */
#define BT_ACS_RMAP_OP_ATT_SIGNED_WRITE_CMD 0x00D2u /**< Signed Write Command */
#define BT_ACS_RMAP_OP_ATT_PREPARE_WRITE_REQ                                                       \
	0x0016u /**< Prepare Write Request (reliable/long write) */
#define BT_ACS_RMAP_OP_ATT_PREPARE_WRITE_RSP 0x0017u /**< Prepare Write Response */
#define BT_ACS_RMAP_OP_ATT_EXECUTE_WRITE_REQ 0x0018u /**< Execute Write Request */
#define BT_ACS_RMAP_OP_ATT_EXECUTE_WRITE_RSP 0x0019u /**< Execute Write Response */

/* --- Notification / Indication --- */
#define BT_ACS_RMAP_OP_ATT_NOTIFY      0x001Bu /**< Handle Value Notification */
#define BT_ACS_RMAP_OP_ATT_INDICATE    0x001Du /**< Handle Value Indication */
#define BT_ACS_RMAP_OP_ATT_CONFIRM     0x001Eu /**< Handle Value Confirmation */
#define BT_ACS_RMAP_OP_ATT_NOTIFY_MULT 0x0023u /**< Multiple Handle Value Notification */

/** @} */

/* Convenience aliases matching the most common usage in restriction map entries */
#define BT_ACS_RMAP_OP_ATT_READ BT_ACS_RMAP_OP_ATT_READ_REQ /**< @see BT_ACS_RMAP_OP_ATT_READ_REQ  \
							     */
#define BT_ACS_RMAP_OP_ATT_WRITE                                                                   \
	BT_ACS_RMAP_OP_ATT_WRITE_REQ /**< @see BT_ACS_RMAP_OP_ATT_WRITE_REQ */

/** @brief Opcode-to-ISC_ID mapping entry within a Protected record (Table 4.20). */
struct bt_acs_rmap_op_isc {
	/** ATT opcode (use BT_ACS_RMAP_OP_ATT_* macros) or CP procedure opcode */
	uint16_t opcode;
	/** Information_Security_Configuration_ID */
	uint16_t isc_id;
};

/**
 * @brief Single operation-to-ISC mapping initialiser for use inside
 *        BT_ACS_RMAP_PROTECT_CHAR_IN_MAP().
 *
 * @param _opcode  ATT opcode (use a BT_ACS_RMAP_OP_ATT_* macro)
 * @param _isc_id  Information Security Configuration ID (0x0001–0xFFFE)
 */
#define BT_ACS_RMAP_OP_ENTRY(_opcode, _isc_id) {.opcode = (_opcode), .isc_id = (_isc_id)}

/** @brief Protected Characteristic (Type_ID 0x02) or Control Point (Type_ID 0x03) record. */
struct bt_acs_rmap_protected {
	/** Type_Value: Resource Handle of the protected attribute */
	uint16_t resource_handle;
	/** Data: opcode to ISC_ID mappings */
	const struct bt_acs_rmap_op_isc *ops;
	/** Number of opcode-to-ISC_ID mappings */
	uint8_t num_ops;
};

/**
 * @brief ACS restriction map (spec-aligned, Tables 4.17–4.20).
 *
 * Replaces the old flat bt_acs_restriction_entry layout with the TLV schema
 * required by the wire protocol (Type_ID + Type_Value + Data_Size + Data).
 */
struct bt_acs_restriction_map {
	/** Restriction Map ID (Type_ID 0x00 record Type_Value) */
	uint16_t map_id;
	/** ISC_ID protecting this map (used in Restriction Map ID List response) */
	uint16_t map_isc_id;
	/** Default ISC_ID (Type_ID 0x01); set to 0 to omit the Default ISC record */
	uint16_t default_isc_id;
};

/**
 * @brief Register a restriction map for automatic discovery by the ACS service.
 *
 * Places the bt_acs_restriction_map into the @c bt_acs_restriction_map iterable
 * section. The ACS service iterates this section for all map lookups (Restriction
 * Map Descriptor, ID List, Activate, and permission checks) — no
 * restriction_map_get callback is required.
 *
 * @param _name C identifier for the generated symbol (must be unique)
 * @param ...   Designated initialisers for bt_acs_restriction_map fields
 *
 * Protected characteristics and Control Points are declared separately with
 * BT_ACS_RMAP_PROTECT_CHAR_IN_MAP() and BT_ACS_RMAP_PROTECT_CP_IN_MAP().
 *
 * Example:
 * @code
 *   BT_ACS_RESTRICTION_MAP_DEFINE(default_map,
 *       .map_id         = 0x0001,
 *       .map_isc_id     = 0x0000,
 *       .default_isc_id = 0x0000);
 * @endcode
 */
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define BT_ACS_RESTRICTION_MAP_DEFINE(_name, ...)                                                  \
	STRUCT_SECTION_ITERABLE(bt_acs_restriction_map, _name) = {__VA_ARGS__}
#else
#define BT_ACS_RESTRICTION_MAP_DEFINE(_name, ...)                                                  \
	BUILD_ASSERT(0, "BT_ACS_RESTRICTION_MAP_DEFINE requires "                                  \
			"CONFIG_BT_ACS_FEAT_AUTHORIZATION=y")
#endif

/**
 * @brief Registration record linking a characteristic UUID to its protection entry.
 *
 * Placed in the @c bt_acs_rmap_char_reg iterable section by
 * BT_ACS_RMAP_PROTECT_CHAR_IN_MAP() or BT_ACS_RMAP_PROTECT_CP_IN_MAP().
 * acs_rmap_resolve_handles() iterates this section once after bt_enable()
 * to fill each entry's resource_handle.
 */
struct bt_acs_rmap_char_reg {
	/** UUID of the characteristic to protect */
	const struct bt_uuid *char_uuid;
	/** Pointer to the bt_acs_rmap_protected entry whose handle to populate */
	struct bt_acs_rmap_protected *entry;
	/** Restriction map ID this entry belongs to */
	uint16_t map_id;
};

/**
 * @brief Declare a protected characteristic and auto-register it to a map.
 *
 * Expands to three declarations at file scope:
 *  - @c _name##_ops[]: a static const array of bt_acs_rmap_op_isc entries
 *  - @c _name: a mutable bt_acs_rmap_protected with resource_handle=0
 *  - @c _name##_reg: a bt_acs_rmap_char_reg record placed in the iterable
 *    section so acs_rmap_resolve_handles() can fill the handle at init.
 *
 * The library discovers entries by scanning the iterable section — no extern
 * declarations or explicit pointer arrays are needed.
 *
 * @param _name      C identifier used as the base name for generated symbols
 * @param _map_id    Restriction map ID this characteristic belongs to
 * @param _char_uuid Pointer to the characteristic UUID
 * @param ...        One or more BT_ACS_RMAP_OP_ENTRY() initialisers
 *
 * Example (in service-specific files):
 * @code
 *   // sample_cts.c
 *   BT_ACS_RMAP_PROTECT_CHAR_IN_MAP(cts_current_time, 0x0001, BT_UUID_CTS_CURRENT_TIME,
 *       BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_READ,   BT_ACS_ISC_ID_NONE),
 *       BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_WRITE,  BT_ACS_ISC_ID_HIGH_SEC),
 *       BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_NOTIFY, BT_ACS_ISC_ID_HIGH_SEC));
 *
 *   // main.c — entries auto-discovered via iterable section
 *   BT_ACS_RESTRICTION_MAP_DEFINE(secured_map, .map_id = 0x0001, ...);
 * @endcode
 */
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define BT_ACS_RMAP_PROTECT_CHAR_IN_MAP(_name, _map_id, _char_uuid, ...)                           \
	static const struct bt_acs_rmap_op_isc _name##_ops[] = {__VA_ARGS__};                      \
	struct bt_acs_rmap_protected _name = {                                                     \
		.resource_handle = 0x0000,                                                         \
		.ops = _name##_ops,                                                                \
		.num_ops = ARRAY_SIZE(_name##_ops),                                                \
	};                                                                                         \
	STRUCT_SECTION_ITERABLE(bt_acs_rmap_char_reg, _name##_reg) = {                             \
		.char_uuid = (_char_uuid),                                                         \
		.entry = &_name,                                                                   \
		.map_id = (_map_id),                                                               \
	}
#else
#define BT_ACS_RMAP_PROTECT_CHAR_IN_MAP(_name, _map_id, _char_uuid, ...)                           \
	BUILD_ASSERT(                                                                              \
		0, "BT_ACS_RMAP_PROTECT_CHAR_IN_MAP requires CONFIG_BT_ACS_FEAT_AUTHORIZATION=y")
#endif

/**
 * @brief Declare a protected Control Point and auto-register it to a map.
 *
 * Like BT_ACS_RMAP_PROTECT_CHAR_IN_MAP() but marks the entry as a Control
 * Point (Type_ID 0x03) instead of a characteristic (Type_ID 0x02). CP
 * entries are auto-discovered via the iterable section — no manual
 * restriction map wiring is needed.
 *
 * @param _name      C identifier used as the base name for generated symbols
 * @param _map_id    Restriction map ID this CP belongs to
 * @param _char_uuid Pointer to the CP characteristic UUID (e.g. BT_UUID_GATT_ACS_CP)
 * @param ...        One or more BT_ACS_RMAP_OP_ENTRY() initialisers
 *
 * Example:
 * @code
 *   BT_ACS_RMAP_PROTECT_CP_IN_MAP(acs_cp, 0x0001, BT_UUID_GATT_ACS_CP,
 *       BT_ACS_RMAP_OP_ENTRY(BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR, 0x0001));
 *
 *   // CP entries auto-discovered — no extra map fields needed:
 *   BT_ACS_RESTRICTION_MAP_DEFINE(secured_map, .map_id = 0x0001, ...);
 * @endcode
 */
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define BT_ACS_RMAP_PROTECT_CP_IN_MAP(_name, _map_id, _char_uuid, ...)                             \
	static const struct bt_acs_rmap_op_isc _name##_ops[] = {__VA_ARGS__};                      \
	struct bt_acs_rmap_protected _name = {                                                     \
		.resource_handle = 0x0000,                                                         \
		.ops = _name##_ops,                                                                \
		.num_ops = ARRAY_SIZE(_name##_ops),                                                \
	};                                                                                         \
	STRUCT_SECTION_ITERABLE_ALTERNATE(bt_acs_rmap_cp_reg, bt_acs_rmap_char_reg,                \
					  _name##_reg) = {                                         \
		.char_uuid = (_char_uuid),                                                         \
		.entry = &_name,                                                                   \
		.map_id = (_map_id),                                                               \
	}
#else
#define BT_ACS_RMAP_PROTECT_CP_IN_MAP(_name, _map_id, _char_uuid, ...)                             \
	BUILD_ASSERT(0, "BT_ACS_RMAP_PROTECT_CP_IN_MAP requires "                                  \
			"CONFIG_BT_ACS_FEAT_AUTHORIZATION=y")
#endif

/**
 * @name Well-known Information Security Configuration IDs
 *
 * These ISC IDs correspond to the ISC records registered internally and
 * are the values passed as
 * @c _isc_id to BT_ACS_RMAP_OP_ENTRY() and the convenience macros below.
 *
 * @{
 */
/** No ISC required; resource is unprotected (Default_ISC = 0, §4.4.3.2). */
#define BT_ACS_ISC_ID_NONE      0x0000
/** High security: Nonce + Authenticated Encryption, optionally with explicit MAC for AES-CCM. */
#define BT_ACS_ISC_ID_HIGH_SEC  0x0001
/** Authentication only: Nonce + MAC (requires ECDH key exchange). */
#define BT_ACS_ISC_ID_AUTH      0x0002
/** Unencrypted fallback: no cryptographic protection (always present). */
#define BT_ACS_ISC_ID_UNENC     0x0003
/** Integrity + authentication, no confidentiality (GMAC). */
#define BT_ACS_ISC_ID_INTEGRITY 0x0004
/** MAC-only authentication, no nonce, no confidentiality (CMAC). */
#define BT_ACS_ISC_ID_MAC_ONLY  0x0005

/**
 * @brief Default ISC ID: best-available protection based on compiled algorithms.
 *
 * Resolves at compile time to the strongest available ISC record so applications
 * don't need to duplicate per-algorithm #if chains.  Priority (strongest first):
 *   CONFIDENTIALITY → HIGH_SEC, GMAC → INTEGRITY, AUTH → AUTH, else → UNENC.
 */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_CONFIDENTIALITY)
#define BT_ACS_ISC_ID_DEFAULT BT_ACS_ISC_ID_HIGH_SEC
#elif IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
#define BT_ACS_ISC_ID_DEFAULT BT_ACS_ISC_ID_INTEGRITY
#elif IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
#define BT_ACS_ISC_ID_DEFAULT BT_ACS_ISC_ID_AUTH
#elif IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define BT_ACS_ISC_ID_DEFAULT BT_ACS_ISC_ID_UNENC
#else
#define BT_ACS_ISC_ID_DEFAULT BT_ACS_ISC_ID_UNENC
#endif
/** @} */

/**
 * @name Convenience macros for common protection patterns.
 *
 * These combine BT_ACS_RMAP_PROTECT_CHAR_IN_MAP() with common opcode
 * patterns, eliminating per-entry BT_ACS_RMAP_OP_ENTRY() boilerplate.
 * All entries are automatically discovered by the library via the iterable
 * section — no explicit pointer arrays or extern declarations are needed.
 *
 * @{
 */

/** @brief Read-only characteristic, auto-registered to a map. */
#define BT_ACS_PROTECT_CHAR_R_IN_MAP(_name, _map_id, _char_uuid, _isc_id)                          \
	BT_ACS_RMAP_PROTECT_CHAR_IN_MAP(_name, _map_id, _char_uuid,                                \
					BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_READ, _isc_id))

/** @brief Write-only characteristic, auto-registered to a map. */
#define BT_ACS_PROTECT_CHAR_W_IN_MAP(_name, _map_id, _char_uuid, _isc_id)                          \
	BT_ACS_RMAP_PROTECT_CHAR_IN_MAP(_name, _map_id, _char_uuid,                                \
					BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_WRITE, _isc_id))

/** @brief Notify-only characteristic, auto-registered to a map. */
#define BT_ACS_PROTECT_CHAR_N_IN_MAP(_name, _map_id, _char_uuid, _isc_id)                          \
	BT_ACS_RMAP_PROTECT_CHAR_IN_MAP(_name, _map_id, _char_uuid,                                \
					BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_NOTIFY, _isc_id))

/** @brief Read + write characteristic, auto-registered to a map. */
#define BT_ACS_PROTECT_CHAR_RW_IN_MAP(_name, _map_id, _char_uuid, _isc_id)                         \
	BT_ACS_RMAP_PROTECT_CHAR_IN_MAP(_name, _map_id, _char_uuid,                                \
					BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_READ, _isc_id),    \
					BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_WRITE, _isc_id))

/** @brief Read + write + notify characteristic, auto-registered to a map. */
#define BT_ACS_PROTECT_CHAR_RWN_IN_MAP(_name, _map_id, _char_uuid, _isc_id)                        \
	BT_ACS_RMAP_PROTECT_CHAR_IN_MAP(_name, _map_id, _char_uuid,                                \
					BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_READ, _isc_id),    \
					BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_WRITE, _isc_id),   \
					BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_NOTIFY, _isc_id))
/** @} */

/**
 * @brief GATT permission flags for ACS-protected characteristics.
 *
 * Use these instead of BT_GATT_PERM_READ / BT_GATT_PERM_WRITE on
 * characteristics that are protected by ACS. When CONFIG_BT_ACS_GATT_AUTHORIZATION
 * is enabled, the GATT authorization callback fires for attributes with
 * BT_GATT_PERM_*_AUTHEN permission flags and automatically enforces ACS access.
 * @{
 */
#define BT_ACS_GATT_PERM_READ  BT_GATT_PERM_READ_AUTHEN
#define BT_ACS_GATT_PERM_WRITE BT_GATT_PERM_WRITE_AUTHEN
/** @} */

/** @brief ACS application callbacks */
struct bt_acs_cb {
	/**
	 * @brief Security has been established for a connection.
	 *
	 * Called when ECDH key exchange completes and session key
	 * is derived.
	 *
	 * @param conn Connection object.
	 */
	void (*security_established)(struct bt_conn *conn);

	/**
	 * @brief Security has been invalidated for a connection.
	 *
	 * Called on disconnect or explicit invalidation.
	 *
	 * @param conn Connection object.
	 */
	void (*security_invalidated)(struct bt_conn *conn);

	/**
	 * @brief Application provides OOB key material.
	 *
	 * @param conn Connection object.
	 * @param oob_data Buffer to write OOB data into.
	 * @param oob_len Pointer to store OOB data length.
	 *
	 * @return 0 on success, negative error code on failure.
	 */
	int (*oob_data_get)(struct bt_conn *conn, uint8_t *oob_data, uint16_t *oob_len);

	/**
	 * @brief AC Server shall present an Output OOB Number to the user.
	 *
	 * Called at Start Key Exchange when Selected_Confirmation_Method is
	 * Confirmation Output OOB Number Action Used (0x01, Table 4.50).
	 * The application must display or beep @p oob_number so the user can
	 * enter it into the AC Client before the Confirmation Code step.
	 *
	 * The @p action value comes from Table 4.51 (0x01=Beep, 0x03=Output Numeric).
	 *
	 * Required when @kconfig{CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC} or
	 * @kconfig{CONFIG_BT_ACS_CONFIRMATION_OUTPUT_BEEP} is enabled.
	 *
	 * @param conn       Connection object.
	 * @param action     Selected_Confirmation_Action value (Table 4.51).
	 * @param oob_number The OOB number the server generated (1 … max_value).
	 */
	void (*output_oob_number)(struct bt_conn *conn, uint8_t action, uint32_t oob_number);

	/**
	 * @brief AC Server requires the user to enter an Input OOB Number.
	 *
	 * Called at Start Key Exchange when Selected_Confirmation_Method is
	 * Confirmation Input OOB Number Action Used (0x02, Table 4.50).
	 * The AC Client will generate a number; the user reads it and enters
	 * it into the AC Server via the described @p action (Table 4.52).
	 * The application must call bt_acs_set_oob_number() once the user
	 * has provided the number, before the Confirmation Code opcode arrives.
	 *
	 * Required when @kconfig{CONFIG_BT_ACS_CONFIRMATION_INPUT_NUMERIC} or
	 * @kconfig{CONFIG_BT_ACS_CONFIRMATION_INPUT_PUSH} is enabled.
	 *
	 * @param conn   Connection object.
	 * @param action Selected_Confirmation_Action value (Table 4.52).
	 */
	void (*input_oob_request)(struct bt_conn *conn, uint8_t action);

	/**
	 * @brief Application provides the pre-shared Static OOB Number.
	 *
	 * Called at Start Key Exchange when Selected_Confirmation_Method is
	 * Confirmation Static OOB Number Used (0x03, Table 4.50).
	 * The application must fill @p oob_out with the static value and set
	 * @p oob_len (1–32 bytes, big-endian, right-aligned).
	 *
	 * Required when @kconfig{CONFIG_BT_ACS_OOB_STATIC_NUM_NUMBER} is enabled.
	 *
	 * @param conn    Connection object.
	 * @param oob_out Buffer to write the static OOB number into (32 bytes max).
	 * @param oob_len Number of valid bytes written into oob_out.
	 *
	 * @return 0 on success, negative error code on failure.
	 */
	int (*static_oob_get)(struct bt_conn *conn, uint8_t *oob_out, uint16_t *oob_len);

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	/**
	 * @brief Application provides the OOB pre-shared key.
	 *
	 * Called during OOB key exchange (§4.4.3.17.1.1, ACS_KEY_ID_OOB)
	 * so the server can populate the shared secret used for confirmation
	 * code and random verification.  Write the OOB key bytes into
	 * @p key_out and set @p key_len.
	 *
	 * Required when @kconfig{CONFIG_BT_ACS_KEY_EXCHANGE_OOB} is enabled.
	 *
	 * @param conn    Connection object.
	 * @param key_out Buffer to receive the OOB shared key
	 *                (up to CONFIG_BT_ACS_SHARED_SECRET_MAX_SIZE bytes).
	 * @param key_len Set to actual byte count written.
	 * @return 0 on success, negative error code on failure.
	 */
	int (*oob_key_get)(struct bt_conn *conn, uint8_t *key_out, uint16_t *key_len);
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_OOB */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_URI)
	/**
	 * @brief Application provides the URI for a given key ID.
	 *
	 * Called when an AC Client sends opcode 0x17 (Get Key URI).
	 * Write the URI string (UTF-8, no NUL terminator) into @p uri_buf
	 * and set @p uri_len to the number of bytes written.
	 *
	 * Required when @kconfig{CONFIG_BT_ACS_KEY_URI} is enabled.
	 *
	 * @param conn        Connection object.
	 * @param key_id      Key_ID from the Get Key URI request (host byte order).
	 * @param uri_buf     Buffer to write the URI into.
	 * @param uri_max_len Maximum bytes available in uri_buf
	 *                    (= CONFIG_BT_ACS_KEY_URI_MAX_LEN).
	 * @param uri_len     Set to actual URI byte count on return.
	 * @return 0 on success, -ENOENT if this key_id has no URI
	 *         (server will respond with Parameter Out Of Range).
	 */
	int (*key_uri_get)(struct bt_conn *conn, uint16_t key_id, uint8_t *uri_buf,
			   uint16_t uri_max_len, uint16_t *uri_len);
#endif /* CONFIG_BT_ACS_KEY_URI */
};

/**
 * @brief Provide the Input OOB Number entered by the user.
 *
 * Must be called by the application after the input_oob_request callback fires
 * and before the AC Client sends the Key Exchange ECDH Confirmation Code opcode.
 * The value must be the same number the user read from the AC Client and typed
 * into the AC Server.  It is stored right-aligned, big-endian, zero-padded to
 * 32 bytes to form the 256-bit AuthValue used in the confirmation code HMAC
 * (see §4.4.3.17.1.2).
 *
 * @param conn    Connection object.
 * @param oob     OOB number bytes (big-endian, 1–32 bytes).
 * @param len     Number of bytes in @p oob.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p conn or @p oob is NULL, @p len is 0 or greater than 32.
 * @retval -ENOTCONN No ACS connection found for @p conn.
 * @retval -ESRCH   No key exchange in progress on this connection.
 * @retval -EPERM   Key exchange confirmation method is not Input OOB.
 */
int bt_acs_set_oob_number(struct bt_conn *conn, const uint8_t *oob, uint16_t len);

/**
 * @brief Initialize the Authorization Control Service.
 *
 * @details Must be called after bt_enable() and after all GATT services have been registered, so
 * that ATT handles for protected characteristics can be resolved from the fully-built GATT
 * attribute table.
 *
 * @note If @kconfig{CONFIG_BT_SETTINGS} is enabled, bt_settings_load() must be called before
 * this function so that previously stored ACS session keys are available on reconnect and the
 * bond-deletion callback can clean up stale session data correctly.
 *
 * @note Start advertising only after this function returns successfully. Starting advertising
 * before bt_acs_init() completes leaves protected characteristics accessible without ACS
 * enforcement.
 *
 * @param cb Application callbacks.
 *
 * @retval 0 Success.
 * @retval -EALREADY ACS has already been initialized.
 * @retval -EINVAL A protected characteristic could not be resolved to a valid
 *         GATT handle.
 */
int bt_acs_init(const struct bt_acs_cb *cb);

/**
 * @brief Invalidate the established ACS security session for a connection.
 *
 * Clears the session key and crypto state, resets the security status flag,
 * and erases any persisted session from flash (if @kconfig{CONFIG_BT_SETTINGS}
 * is enabled). The application is notified via the @c security_invalidated
 * callback and the client is informed via an ACS Status indication.
 *
 * After this call the client must perform a new key exchange before accessing
 * protected resources.
 *
 * @param conn Connection whose security session should be invalidated.
 *
 * @return 0 on success.
 * @return -EINVAL if @p conn is NULL or ACS is not initialized.
 * @return -ENOTCONN if no ACS context exists for @p conn.
 */
int bt_acs_invalidate_security(struct bt_conn *conn);

/**
 * @brief Get current ACS status flags for a connection.
 *
 * @param conn Connection to query.
 *
 * @return Status flags bitmask (enum bt_acs_status_flag).
 */
uint8_t bt_acs_status_get(struct bt_conn *conn);

/**
 * @brief Set the active restriction map for a connection.
 *
 * @param conn Connection to configure.
 * @param map_id Restriction map identifier to activate.
 *
 * @retval 0 Success.
 * @retval -EINVAL @p conn is NULL, ACS is not initialized, or @p map_id is
 *         not a defined restriction map.
 * @retval -ENOTCONN No ACS context exists for @p conn.
 */
int bt_acs_set_restriction_map(struct bt_conn *conn, uint16_t map_id);

/**
 * @brief Check if an operation is permitted on a resource.
 *
 * @param conn Connection to check.
 * @param att_handle ATT handle of the resource.
 * @param direction Operation direction to check.
 *
 * @return true if operation is permitted, false otherwise.
 */
bool bt_acs_policy_is_permitted(struct bt_conn *conn, uint16_t att_handle,
				enum bt_acs_direction direction);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_ACS_H_ */
