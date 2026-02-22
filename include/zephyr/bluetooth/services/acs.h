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
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/sys/util_macro.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief ACS Status characteristic Status_Flags field bits (Table 4.7). */
enum bt_acs_status_flag {
	/** Security controls are enabled on this server */
	BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED = BIT(0),
	/** Security has been established for this connection */
	BT_ACS_STATUS_SECURITY_ESTABLISHED = BIT(1),
};

/**
 * @brief ACS Control Point opcodes, requests and responses (Table 4.14).
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

/**
 * @name ATT opcodes for use in restriction map entries (Type_ID 0x02).
 *
 * These are standard ATT opcodes from the Bluetooth Core Specification
 * (Vol 3, Part F, Section 3.4), zero-padded to uint16 as required by the
 * ACS restriction map Data field (spec §4.4.4.4.1.3).
 *
 * A Protected Characteristic record is scoped to direct access or delivery of
 * a single characteristic value, so these are the opcodes the policy engine
 * resolves to a read, write, notification or indication. Any other opcode in a
 * Type_ID 0x02 record fails bt_acs_init() with -EINVAL rather than advertising
 * an ISC that would never be applied.
 * @{
 */

/* --- Read operations --- */
#define BT_ACS_RMAP_OP_ATT_READ_REQ      0x000Au /**< Read Request */
#define BT_ACS_RMAP_OP_ATT_READ_BLOB_REQ 0x000Cu /**< Read Blob Request (long reads) */

/* --- Write operations --- */
#define BT_ACS_RMAP_OP_ATT_WRITE_REQ         0x0012u /**< Write Request */
#define BT_ACS_RMAP_OP_ATT_WRITE_CMD         0x0052u /**< Write Command (no response) */
#define BT_ACS_RMAP_OP_ATT_SIGNED_WRITE_CMD  0x00D2u /**< Signed Write Command */
#define BT_ACS_RMAP_OP_ATT_PREPARE_WRITE_REQ 0x0016u /**< Prepare Write Request (long write) */
#define BT_ACS_RMAP_OP_ATT_EXECUTE_WRITE_REQ 0x0018u /**< Execute Write Request */

/* --- Notification / Indication --- */
#define BT_ACS_RMAP_OP_ATT_NOTIFY   0x001Bu /**< Handle Value Notification */
#define BT_ACS_RMAP_OP_ATT_INDICATE 0x001Du /**< Handle Value Indication */

/** @} */

/** @brief Opcode-to-ISC_ID mapping entry within a Protected record (Table 4.20). */
struct bt_acs_rmap_op_isc {
	/** ATT opcode (use BT_ACS_RMAP_OP_ATT_* macros) or CP procedure opcode */
	uint16_t opcode;
	/** Information_Security_Configuration_ID */
	uint16_t isc_id;
};

/**
 * @brief Single operation-to-ISC mapping initialiser for use inside
 *        BT_ACS_RMAP_DECLARE_CHAR() and BT_ACS_RMAP_DECLARE_CP().
 *
 * One initialiser produces one [opcode | Security Configuration ID] pair in the
 * record's Data field. Pass BT_ACS_ISC_ID_NONE to declare an operation
 * unprotected.
 *
 * @param _opcode  ATT opcode (use a BT_ACS_RMAP_OP_ATT_* macro) or CP procedure opcode
 * @param _isc_id  Information Security Configuration ID (0x0001–0xFFFE), or
 *                 BT_ACS_ISC_ID_NONE for an unprotected operation
 */
#define BT_ACS_RMAP_OP_ENTRY(_opcode, _isc_id) {.opcode = (_opcode), .isc_id = (_isc_id)}

/** @brief Restriction-map protected resource kind. */
enum bt_acs_rmap_resource_kind {
	BT_ACS_RMAP_RESOURCE_CHAR,
	BT_ACS_RMAP_RESOURCE_CP,
};

/**
 * @brief Immutable Protected Characteristic or Protected Control Point declaration.
 *
 * The registration macros keep this policy data in ROM. Its corresponding
 * bt_acs_rmap_entry is placed in a RAM iterable section and populated with
 * resolved handles when ACS is initialized.
 */
struct bt_acs_rmap_protected {
	/** Restriction map containing this record */
	uint16_t map_id;
	/** Protected Characteristic or Protected Control Point */
	enum bt_acs_rmap_resource_kind kind;
	/** UUID of the characteristic backing this resource */
	const struct bt_uuid *char_uuid;
	/** Data: opcode to ISC_ID mappings */
	const struct bt_acs_rmap_op_isc *ops;
	/** Number of opcode-to-ISC_ID mappings */
	uint8_t num_ops;
};

/**
 * @brief Resolved runtime state for one protected resource declaration.
 *
 * Every field but @c record is resolved by bt_acs_init().
 */
struct bt_acs_rmap_entry {
	/** Immutable protected-resource declaration */
	const struct bt_acs_rmap_protected *record;
	/** @cond INTERNAL_HIDDEN */
	/** Resolved GATT characteristic value attribute used to invoke its access callbacks. */
	const struct bt_gatt_attr *value_attr;
	/** ACS Resource Handle assigned to the protected characteristic. */
	uint16_t resource_handle;
	/** Resolved ATT handle of the GATT characteristic value attribute. */
	uint16_t attr_handle;
	/** Bit mask of enum acs_direction values whose operations require ACS protection. */
	uint8_t protected_dir_mask;
	/** GATT characteristic properties copied from its characteristic declaration. */
	uint8_t props;
	/** @endcond */
};

/**
 * @brief Reserved map ID: ACS does not mediate the resources at all (§4.4.3.2).
 *
 * Every resource is reached directly through the service owning it, and the
 * descriptor is only a Restriction Map ID record - no Default record and no
 * resource records. Declaring a protected resource against it fails the build.
 */
#define BT_ACS_RMAP_ID_NONE 0x0000U

struct bt_acs_restriction_map {
	/** Restriction Map ID (Type_ID 0x00 record Type_Value) */
	uint16_t map_id;
	/** ISC_ID protecting this map (used in Restriction Map ID List response) */
	uint16_t map_isc_id;
	/** Default Security Configuration. (Table 4.19. Restriction Map Record Type_ID values) */
	uint16_t default_isc_id;
};

/**
 * @brief Resolved runtime directory for one restriction map.
 *
 * Every field but @c map is resolved by bt_acs_init().
 */
struct bt_acs_rmap_runtime {
	/** Immutable restriction-map declaration */
	const struct bt_acs_restriction_map *map;
	/** @cond INTERNAL_HIDDEN */
	/** Index of this map's first entry in the sorted bt_acs_rmap_entry section. */
	uint16_t first;
	/** Number of consecutive bt_acs_rmap_entry objects belonging to this map. */
	uint16_t count;
	/** Lowest resolved ATT handle in this map, used to reject out-of-range lookups. */
	uint16_t att_min;
	/** Highest resolved ATT handle in this map, used to reject out-of-range lookups. */
	uint16_t att_max;
	/** @endcond */
};

/**
 * @brief Register a restriction map for automatic discovery by the ACS service.
 *
 * Generates @c _name (the immutable declaration) and @c _name##_runtime (a node
 * in the iterable section ACS walks), so @p _name must be unique across the
 * application. bt_acs_init() resolves the map and its protected resources.
 *
 * @param _name            C identifier for the generated symbol (must be unique)
 * @param _map_id          Restriction Map ID (Type_ID 0x00 record Type_Value)
 * @param _map_isc_id      ISC_ID protecting this map, or BT_ACS_ISC_ID_NONE
 * @param _default_isc_id  ISC_ID (Type_ID 0x01) for resources not listed in this
 *                         map, or BT_ACS_ISC_ID_NONE to leave them unprotected
 *
 * Protected resources are declared separately with BT_ACS_RMAP_DECLARE_CHAR()
 * and BT_ACS_RMAP_DECLARE_CP(); resources not listed take @c _default_isc_id.
 *
 * Example:
 * @code
 *   BT_ACS_RESTRICTION_MAP_DEFINE(default_map, 0x0001,
 *                                 BT_ACS_ISC_ID_HIGH_SEC_GCM, BT_ACS_ISC_ID_NONE);
 * @endcode
 */
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define BT_ACS_RESTRICTION_MAP_DEFINE(_name, _map_id, _map_isc_id, _default_isc_id)                \
	BUILD_ASSERT((_map_id) != BT_ACS_RMAP_ID_NONE ||                                           \
			     (_default_isc_id) == BT_ACS_ISC_ID_NONE,                              \
		     "reserved restriction map 0 consists of only a Restriction Map ID record");   \
	static const struct bt_acs_restriction_map _name = {                                       \
		.map_id = (_map_id),                                                               \
		.map_isc_id = (_map_isc_id),                                                       \
		.default_isc_id = (_default_isc_id),                                               \
	};                                                                                         \
	static STRUCT_SECTION_ITERABLE(bt_acs_rmap_runtime, _name##_runtime) = {                   \
		.map = &_name,                                                                 \
	}
#else
#define BT_ACS_RESTRICTION_MAP_DEFINE(_name, _map_id, _map_isc_id, _default_isc_id)                \
	BUILD_ASSERT(0, "BT_ACS_RESTRICTION_MAP_DEFINE requires "                                  \
			"CONFIG_BT_ACS_FEAT_AUTHORIZATION=y")
#endif

/**
 * @brief Declare a Protected Characteristic record and auto-register it to a map.
 *
 * Mirrors a Restriction Map Descriptor Type_ID 0x02 record: the characteristic
 * named by @p _char_uuid, plus one [ATT opcode | Security Configuration ID] pair
 * per BT_ACS_RMAP_OP_ENTRY() argument. Each operation carries its own ISC, so a
 * characteristic can be protected for some operations and unprotected
 * (BT_ACS_ISC_ID_NONE) for others.
 *
 * Generates @c _name##_ops (the opcode/ISC pairs), @c _name (the policy record)
 * and @c _name##_runtime (a node in the iterable section ACS walks), so
 * @p _name must be unique across the application. The record may live in the
 * service's own source file; bt_acs_init() discovers it and binds it to the map
 * named by @p _map_id.
 *
 * @param _name      C identifier used as the base name for generated symbols
 * @param _map_id    Restriction map ID this characteristic belongs to
 * @param _char_uuid Pointer to the characteristic UUID
 * @param ...        One or more BT_ACS_RMAP_OP_ENTRY() initialisers
 *
 * Example (in service-specific files):
 * @code
 *   // read unprotected, write and notify protected
 *   BT_ACS_RMAP_DECLARE_CHAR(cts_current_time, 0x0001, BT_UUID_CTS_CURRENT_TIME,
 *       BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_READ_REQ,  BT_ACS_ISC_ID_NONE),
 *       BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_WRITE_REQ, BT_ACS_ISC_ID_HIGH_SEC_GCM),
 *       BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_NOTIFY,    BT_ACS_ISC_ID_HIGH_SEC_GCM));
 * @endcode
 */
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define BT_ACS_RMAP_DECLARE_CHAR(_name, _map_id, _char_uuid, ...)                                  \
	BUILD_ASSERT((_map_id) != BT_ACS_RMAP_ID_NONE,                                             \
		     "reserved restriction map 0 carries no protected resources");                 \
	static const struct bt_acs_rmap_op_isc _name##_ops[] = {__VA_ARGS__};                      \
	static const struct bt_acs_rmap_protected _name = {                                        \
		.map_id = (_map_id),                                                               \
		.kind = BT_ACS_RMAP_RESOURCE_CHAR,                                                 \
		.char_uuid = (_char_uuid),                                                         \
		.ops = _name##_ops,                                                                \
		.num_ops = ARRAY_SIZE(_name##_ops),                                                \
	};                                                                                         \
	static STRUCT_SECTION_ITERABLE(bt_acs_rmap_entry, _name##_runtime) = {                     \
		.record = &_name,                                                                  \
	}
#else
#define BT_ACS_RMAP_DECLARE_CHAR(_name, _map_id, _char_uuid, ...)                                  \
	BUILD_ASSERT(0, "BT_ACS_RMAP_DECLARE_CHAR requires CONFIG_BT_ACS_FEAT_AUTHORIZATION=y")
#endif

/** @cond INTERNAL_HIDDEN */
/*
 * Type_ID 0x03 record: identical layout to BT_ACS_RMAP_DECLARE_CHAR(), but each
 * pair binds a *procedure* opcode (the first octet of a CP write) rather than an
 * ATT opcode, so the two namespaces stay distinct. Not public: a caller must go
 * through the macro for the control point it is protecting, so that the ACS CP's
 * own procedure opcodes can be checked.
 */
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define Z_BT_ACS_RMAP_DECLARE_CP(_name, _map_id, _char_uuid, ...)                                  \
	BUILD_ASSERT((_map_id) != BT_ACS_RMAP_ID_NONE,                                             \
		     "reserved restriction map 0 carries no protected resources");                 \
	static const struct bt_acs_rmap_op_isc _name##_ops[] = {__VA_ARGS__};                      \
	static const struct bt_acs_rmap_protected _name = {                                        \
		.map_id = (_map_id),                                                               \
		.kind = BT_ACS_RMAP_RESOURCE_CP,                                                   \
		.char_uuid = (_char_uuid),                                                         \
		.ops = _name##_ops,                                                                \
		.num_ops = ARRAY_SIZE(_name##_ops),                                                \
	};                                                                                         \
	static STRUCT_SECTION_ITERABLE(bt_acs_rmap_entry, _name##_runtime) = {                     \
		.record = &_name,                                                                  \
	}
#else
#define Z_BT_ACS_RMAP_DECLARE_CP(_name, _map_id, _char_uuid, ...)                                  \
	BUILD_ASSERT(0, "protected Control Point records require "                                 \
			"CONFIG_BT_ACS_FEAT_AUTHORIZATION=y")
#endif
/** @endcond */

/**
 * @name Implementation-assigned Information Security Configuration IDs
 *
 * Per ACS v1.0 §4.4.3.7, ISC IDs other than 0x0000 and 0xFFFF are
 * implementation-specific and assigned by the AC Server. These are this
 * implementation's assignments, registered with BT_ACS_ISC_DEFINE() in
 * acs_isc.c; applications may extend them.
 *
 * Each record is compiled in only when its algorithm is enabled, and an ID
 * with no compiled-in record leaves the resource unreachable rather than
 * failing the build.
 *
 * @{
 */
/** ISC ID 0x0000: resource not protected by ACS (spec-defined, §3.5.2). */
#define BT_ACS_ISC_ID_NONE           0x0000
/** Nonce, Authenticated And Encrypted Protected Resource Request Or Response (AES-128-GCM). */
#define BT_ACS_ISC_ID_HIGH_SEC_GCM   0x0001
/** @cond INTERNAL_HIDDEN */
/* 0x0002: AES-128-EAX, not supported. */
/** @endcond */
/** Unencrypted Protected Resource Request Or Response. */
#define BT_ACS_ISC_ID_UNENC          0x0003
/** Nonce, Authenticated Protected Resource Request Or Response, MAC (AES-128-GMAC). */
#define BT_ACS_ISC_ID_INTEGRITY_GMAC 0x0004
/** MAC (AES-128-CMAC). */
#define BT_ACS_ISC_ID_MAC_ONLY_CMAC  0x0005
/** Nonce, Authenticated And Encrypted Protected Resource Request Or Response (AES-128-CCM). */
#define BT_ACS_ISC_ID_HIGH_SEC_CCM   0x0006
/** @} */

/**
 * @name Shared-ISC shorthands
 *
 * For the common case where every operation in a record is bound to the *same*
 * ISC, list the opcodes and give the ISC once. Use the general
 * BT_ACS_RMAP_DECLARE_CHAR() / BT_ACS_RMAP_DECLARE_CP() when the ISC differs
 * per operation — that is the one thing these cannot express.
 *
 * @{
 */

/** @cond INTERNAL_HIDDEN */
#define Z_BT_ACS_RMAP_OP_SHARED_ISC(_opcode, _isc_id) BT_ACS_RMAP_OP_ENTRY(_opcode, _isc_id)
/** @endcond */

/**
 * @brief Protected Characteristic record whose operations share one ISC.
 *
 * @param _name      C identifier used as the base name for generated symbols
 * @param _map_id    Restriction map ID this characteristic belongs to
 * @param _char_uuid Pointer to the characteristic UUID
 * @param _isc_id    ISC bound to every listed opcode
 * @param ...        One or more BT_ACS_RMAP_OP_ATT_* opcodes
 *
 * Example:
 * @code
 *   // read and write protected under one ISC; long reads included
 *   BT_ACS_RMAP_DECLARE_CHAR_OPS(cfg_blob, 0x0001, BT_UUID_CFG_BLOB,
 *                                BT_ACS_ISC_ID_HIGH_SEC_GCM,
 *                                BT_ACS_RMAP_OP_ATT_READ_REQ,
 *                                BT_ACS_RMAP_OP_ATT_READ_BLOB_REQ,
 *                                BT_ACS_RMAP_OP_ATT_WRITE_REQ);
 * @endcode
 */
#define BT_ACS_RMAP_DECLARE_CHAR_OPS(_name, _map_id, _char_uuid, _isc_id, ...)                     \
	BT_ACS_RMAP_DECLARE_CHAR(                                                                  \
		_name, _map_id, _char_uuid,                                                        \
		FOR_EACH_FIXED_ARG(Z_BT_ACS_RMAP_OP_SHARED_ISC, (, ), _isc_id, __VA_ARGS__))

/**
 * @brief Protected Control Point record for a *non-ACS* service's Control Point.
 *
 * The opcodes are that service's own procedure opcodes and are opaque to ACS,
 * which enforces only the ISC before forwarding the write. Use
 * BT_ACS_RMAP_DECLARE_ACS_CP_OPS() for ACS's own Control Point, whose opcodes
 * ACS does interpret.
 *
 * Plain GATT writes are authorized using the first payload byte as the Control
 * Point opcode. Opcodes bound to a nonzero ISC must use Data In; unlisted opcodes
 * and opcodes bound to BT_ACS_ISC_ID_NONE remain directly writable. Prepared,
 * executed, fragmented, and empty plain writes are rejected when this Control
 * Point has any protected procedure because their opcode cannot be classified
 * reliably from a single authorization call.
 *
 * @param _name      C identifier used as the base name for generated symbols
 * @param _map_id    Restriction map ID this CP belongs to
 * @param _char_uuid Pointer to the foreign CP characteristic UUID
 * @param _isc_id    ISC bound to every listed procedure opcode
 * @param ...        One or more of that service's procedure opcodes
 */
#define BT_ACS_RMAP_DECLARE_EXTERNAL_CP_OPS(_name, _map_id, _char_uuid, _isc_id, ...)              \
	Z_BT_ACS_RMAP_DECLARE_CP(                                                                  \
		_name, _map_id, _char_uuid,                                                        \
		FOR_EACH_FIXED_ARG(Z_BT_ACS_RMAP_OP_SHARED_ISC, (, ), _isc_id, __VA_ARGS__))

/** @cond INTERNAL_HIDDEN */
/*
 * Protecting any of these strands the AC Client. Get Restriction Map ID List is
 * excluded from the protected path by ACS 1.0 Section 4.4.3.3; Abort is the
 * recovery path (Section 4.4.3.13) and cannot be reached once a failed exchange
 * has left no keys; Get ACS Feature opens the onboarding sequence of ACP 1.0
 * Appendix E; and a Data In write requires established security, which no
 * connection has before its first key exchange.
 *
 * @p _entry is applied to each opcode as _entry(_arg, opcode), so the build-time
 * assert below and the AC Server's init-time check share one list.
 */
#define Z_BT_ACS_CP_UNPROTECTABLE_OPCODES(_arg, _entry)                                            \
	_entry(_arg, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST)                                 \
	_entry(_arg, BT_ACS_CP_OPCODE_ABORT)                                                       \
	_entry(_arg, BT_ACS_CP_OPCODE_GET_FEATURE)                                                 \
	_entry(_arg, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE)                                          \
	_entry(_arg, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH)                                           \
	_entry(_arg, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE)                                           \
	_entry(_arg, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND)                                           \
	_entry(_arg, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF)

#define Z_BT_ACS_OPCODE_DIFFERS(_op, _listed) (_op) != (_listed) &&

/*
 * Only reaches declarations that name their opcodes literally. A record built
 * any other way is caught by the matching check in bt_acs_init().
 */
#define Z_BT_ACS_ASSERT_CP_PROTECTABLE(_op)                                                        \
	BUILD_ASSERT(Z_BT_ACS_CP_UNPROTECTABLE_OPCODES(_op, Z_BT_ACS_OPCODE_DIFFERS) 1,            \
		     "this ACS CP procedure must stay reachable on the plain ACS CP")
/** @endcond */

/**
 * @brief Protected Control Point record for ACS's own Control Point.
 *
 * Takes no UUID, so the opcodes are known to be ACS CP procedure opcodes and the
 * ones that must stay reachable on the plain ACS CP are rejected at build time.
 *
 * @param _name   C identifier used as the base name for generated symbols
 * @param _map_id Restriction map ID this record belongs to
 * @param _isc_id ISC bound to every listed procedure opcode
 * @param ...     One or more BT_ACS_CP_OPCODE_* procedure opcodes
 */
#define BT_ACS_RMAP_DECLARE_ACS_CP_OPS(_name, _map_id, _isc_id, ...)                               \
	FOR_EACH(Z_BT_ACS_ASSERT_CP_PROTECTABLE, (;), __VA_ARGS__);                                \
	BT_ACS_RMAP_DECLARE_EXTERNAL_CP_OPS(_name, _map_id, BT_UUID_GATT_ACS_CP, _isc_id,          \
					    __VA_ARGS__)
/** @} */

/** @brief ACS application callbacks */
struct bt_acs_cb {
	/**
	 * @brief Security has been established for a connection.
	 *
	 * Called when a key exchange completes, by either method: ECDH, or a
	 * standalone KDF exchange deriving fresh session keys from a restored
	 * parent key. Protected resources are reachable from this point.
	 *
	 * @param conn Connection object.
	 */
	void (*security_established)(struct bt_conn *conn);

	/**
	 * @brief Security has been invalidated for a connection.
	 *
	 * Called on disconnect, on an Invalidate procedure from the client, and
	 * on bt_acs_invalidate_security(). A new key exchange is required before
	 * protected resources are reachable again.
	 *
	 * @param conn Connection object.
	 */
	void (*security_invalidated)(struct bt_conn *conn);

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
 * @brief Protected ACS output completion callback.
 *
 * @param conn      Peer the transfer was addressed to.
 * @param err       0 on success, negative errno if the transfer failed.
 * @param user_data The @c user_data given in the notify/indicate parameters.
 */
typedef void (*bt_acs_output_func_t)(struct bt_conn *conn, int err, void *user_data);

/** @brief Parameters for bt_acs_notify_cb(). */
struct bt_acs_notify_params {
	/** Protected resource UUID; optional, selects the resource when @p attr is NULL. */
	const struct bt_uuid *uuid;
	/** Characteristic declaration or value attribute; optional if @p uuid is set. */
	const struct bt_gatt_attr *attr;
	/** Characteristic value payload to protect and send. */
	const void *data;
	/** Length of @p data. */
	uint16_t len;
	/** Optional callback after the notification transfer completes or fails. */
	bt_acs_output_func_t func;
	/** User data passed to @p func. */
	void *user_data;
};

/** @brief Parameters for bt_acs_indicate(). */
struct bt_acs_indicate_params {
	/** Protected resource UUID; optional, selects the resource when @p attr is NULL. */
	const struct bt_uuid *uuid;
	/** Characteristic declaration or value attribute; optional if @p uuid is set. */
	const struct bt_gatt_attr *attr;
	/** Characteristic value payload to protect and send. */
	const void *data;
	/** Length of @p data. */
	uint16_t len;
	/** Optional callback after the indication is confirmed or fails. */
	bt_acs_output_func_t func;
	/** User data passed to @p func. */
	void *user_data;
};

/**
 * @brief Send a protected resource notification through ACS Data Out Notify.
 *
 * The protected resource is selected by @p params->uuid when set, otherwise by
 * @p params->attr, which may point to either the characteristic declaration or
 * the characteristic value attribute. Prefer this over bt_acs_notify_uuid()
 * whenever a completion callback is wanted.
 *
 * ACS copies @p params->data into its own queued reply buffer before returning,
 * so stack/local caller data remains safe while the protected transfer waits in
 * the DON queue.
 *
 * Passing NULL for @p conn sends to every connected peer whose ACS security,
 * restriction map, key, and DON subscription allow the protected notification.
 *
 * @param conn   Target connection, or NULL to send to all eligible peers.
 * @param params Resource, payload and optional completion callback.
 *
 * @retval 0 Success; for broadcast, at least one peer accepted the notification.
 * @retval -EINVAL Invalid parameters.
 * @retval -ENOENT No matching protected resource or ATT Notify mapping.
 * @retval -ENOTCONN No eligible ACS connection.
 * @retval -EACCES ACS security/key/subscription is not ready for the peer.
 * @retval -ENOMEM Buffer or reply pool exhausted.
 */
int bt_acs_notify_cb(struct bt_conn *conn, struct bt_acs_notify_params *params);

/**
 * @brief Send a protected resource indication through ACS Data Out Indicate.
 *
 * Behaves like bt_acs_notify_cb(), but requires an ATT Indicate opcode mapping
 * and keeps the queued payload alive until the DOI indication is confirmed or
 * fails. The protected resource is selected by @p params->uuid when set,
 * otherwise by @p params->attr.
 *
 * @param conn   Target connection, or NULL to send to all eligible peers.
 * @param params Resource, payload and optional completion callback.
 *
 * @retval 0 Success; for broadcast, at least one peer accepted the indication.
 * @retval -EINVAL Invalid parameters.
 * @retval -ENOENT No matching protected resource or ATT Indicate mapping.
 * @retval -ENOTCONN No eligible ACS connection.
 * @retval -EACCES ACS security/key/subscription is not ready for the peer.
 * @retval -ENOMEM Buffer or reply pool exhausted.
 */
int bt_acs_indicate(struct bt_conn *conn, struct bt_acs_indicate_params *params);

/**
 * @brief Send a protected resource notification through ACS Data Out Notify.
 *
 * The characteristic UUID is resolved to the AC Server-assigned Resource Handle,
 * then the active restriction map is checked for an ATT Notify opcode mapping.
 * The value is encrypted/authenticated with that mapping's ISC and sent on DON.
 *
 * Convenience form of bt_acs_notify_cb() with no completion callback; use that
 * function directly to be told when the transfer finishes.
 *
 * Passing NULL for @p conn sends to every connected peer whose ACS security,
 * restriction map, key, and DON subscription allow the protected notification.
 *
 * @param conn      Target connection, or NULL to send to all eligible peers.
 * @param char_uuid UUID of the protected characteristic value.
 * @param data      Characteristic value payload.
 * @param len       Length of @p data.
 *
 * @retval 0 Success; for broadcast, at least one peer accepted the notification.
 * @retval -EINVAL Invalid parameters.
 * @retval -ENOENT No matching protected resource or opcode mapping.
 * @retval -ENOTCONN No eligible ACS connection.
 * @retval -EACCES ACS security/key/subscription is not ready for the peer.
 * @retval -ENOMEM Buffer or reply pool exhausted.
 */
int bt_acs_notify_uuid(struct bt_conn *conn, const struct bt_uuid *char_uuid, const void *data,
		       uint16_t len);

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
 * Never fails: a NULL @p conn, or one with no ACS context, reports the
 * server-wide security controls switch with the per-connection
 * @ref BT_ACS_STATUS_SECURITY_ESTABLISHED bit clear.
 *
 * @param conn Connection to query, or NULL for the server-wide flags.
 *
 * @return Status flags bitmask (@ref bt_acs_status_flag).
 */
uint8_t bt_acs_status_get(struct bt_conn *conn);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_ACS_H_ */
