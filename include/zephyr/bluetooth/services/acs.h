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
	 * resource handle.
	 */
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
	BT_ACS_CP_OPCODE_GET_ACS_FEATURE = 0x19,
	/** Used by the AC Server to provide the supported features and capabilities. */
	BT_ACS_CP_OPCODE_ACS_FEATURE_RESPONSE = 0x1A,
	/** Requests the exchange of public keys as part of the ECDH key agreement scheme. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH = 0x1B,
	/** Used by the AC Server to provide the AC Server public key as part of the ECDH key
	 * agreement scheme.
	 */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_RESPONSE = 0x1C,
	/** Requests the exchange of confirmation codes for ECDH key agreement. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_CODE = 0x1D,
	/** Used by the AC Server to provide the AC Server confirmation code for ECDH. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_CODE_RESPONSE = 0x1E,
	/** Requests the exchange of random numbers for ECDH confirmation code calculation. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_RANDOM_NUMBER = 0x1F,
	/** Used by the AC Server to provide the AC Server random number for ECDH. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_RANDOM_NUMBER_RESPONSE = 0x20,
	/** Requests the exchange of keys using KDF or as part of ECDH. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF = 0x21,
	/** Used by the AC Server to provide the KDF parameters. */
	BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF_RESPONSE = 0x22,
	/** Sets the fixed part of the AC Client nonce. */
	BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED = 0x23,
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
 * Type_ID 0x02 record fails the build, rather than advertising an ISC that would
 * never be applied.
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

/** @cond INTERNAL_HIDDEN */
/* Opcode-to-ISC_ID mapping entry within a Protected record (Table 4.20). */
struct bt_acs_rmap_op_isc {
	uint16_t opcode; /* ATT opcode (BT_ACS_RMAP_OP_ATT_*) or CP procedure opcode */
	uint16_t isc_id; /* Information_Security_Configuration_ID */
};

/*
 * A record's Data field length is carried by the one-octet Data_Size of the
 * descriptor general schema (Table 4.4), and every mapping encodes as four
 * octets (Table 4.20).
 */
#define Z_BT_ACS_RMAP_MAX_OPS (UINT8_MAX / 4U)

/*
 * The ATT opcodes a Type_ID 0x02 record may carry. @p _entry is applied to each
 * as _entry(_arg, opcode), for the build-time assert in
 * BT_ACS_RMAP_CHAR_DEFINE().
 */
#define Z_BT_ACS_RMAP_ATT_OPCODES(_arg, _entry)                                                    \
	_entry(_arg, BT_ACS_RMAP_OP_ATT_READ_REQ)                                                  \
	_entry(_arg, BT_ACS_RMAP_OP_ATT_READ_BLOB_REQ)                                             \
	_entry(_arg, BT_ACS_RMAP_OP_ATT_WRITE_REQ)                                                 \
	_entry(_arg, BT_ACS_RMAP_OP_ATT_WRITE_CMD)                                                 \
	_entry(_arg, BT_ACS_RMAP_OP_ATT_SIGNED_WRITE_CMD)                                          \
	_entry(_arg, BT_ACS_RMAP_OP_ATT_PREPARE_WRITE_REQ)                                         \
	_entry(_arg, BT_ACS_RMAP_OP_ATT_EXECUTE_WRITE_REQ)                                         \
	_entry(_arg, BT_ACS_RMAP_OP_ATT_NOTIFY)                                                    \
	_entry(_arg, BT_ACS_RMAP_OP_ATT_INDICATE)

#define Z_BT_ACS_OPCODE_MATCHES(_op, _listed) (_op) == (_listed) ||

/* clang-format off */
#define Z_BT_ACS_COUNT_ONE(_arg, _item) + 1
/* clang-format on */

/* Only these opcodes are valid, so a longer list is malformed regardless of the
 * Z_BT_ACS_RMAP_MAX_OPS encoding limit.
 */
#define Z_BT_ACS_RMAP_MAX_CHAR_OPS (0 Z_BT_ACS_RMAP_ATT_OPCODES(_, Z_BT_ACS_COUNT_ONE))

/* Protected resource kind selecting the record Type_ID (0x02 char, 0x03 CP). */
enum bt_acs_rmap_resource_kind {
	BT_ACS_RMAP_RESOURCE_CHAR,
	BT_ACS_RMAP_RESOURCE_CP,
};

/* GATT binding of a protected resource, filled in by bt_acs_init(). */
struct bt_acs_rmap_binding {
	uint16_t resource_handle;              /* ACS Resource Handle */
	uint16_t attr_handle;                  /* GATT Attribute Handle of the value */
	const struct bt_gatt_attr *value_attr; /* GATT value attribute */
	uint8_t props;                         /* GATT characteristic properties */
};

struct bt_acs_restriction_map;

/* Resource a restriction map protects: Protected Characteristic or Control Point (Table 4.19). */
struct bt_acs_rmap_resource {
	const struct bt_acs_restriction_map *map; /* restriction map listing this resource */
	enum bt_acs_rmap_resource_kind kind;      /* Protected Characteristic or Control Point */
	const struct bt_uuid *char_uuid;          /* UUID of the characteristic backing it */
	const struct bt_acs_rmap_op_isc *ops;     /* opcode-to-ISC_ID mappings (Data field) */
	uint8_t num_ops;                          /* number of entries in ops */
	struct bt_acs_rmap_binding bound;         /* set by bt_acs_init(), not the application */
};

/** @endcond */

/**
 * @name Information Security Configuration IDs
 *
 * ISC IDs other than 0x0000 and 0xFFFF are assigned by the AC Server (§4.4.3.7).
 * These are this implementation's. Each ISC record is compiled in only when its
 * algorithm is enabled, and naming one that is not fails the build.
 *
 * @{
 */
/** Resource not protected by ACS (§3.5.2). */
#define BT_ACS_ISC_ID_NONE           0x0000
/** Nonce, Authenticated And Encrypted Protected Resource Request Or Response (AES-128-GCM). */
#define BT_ACS_ISC_ID_HIGH_SEC_GCM   0x0001
/** Nonce, Authenticated Protected Resource Request Or Response, MAC (AES-128-GMAC). */
#define BT_ACS_ISC_ID_INTEGRITY_GMAC 0x0004
/** MAC (AES-128-CMAC). */
#define BT_ACS_ISC_ID_MAC_ONLY_CMAC  0x0005
/** Nonce, Authenticated And Encrypted Protected Resource Request Or Response (AES-128-CCM). */
#define BT_ACS_ISC_ID_HIGH_SEC_CCM   0x0006
/** @} */

/** @cond INTERNAL_HIDDEN */
/* True when _isc is BT_ACS_ISC_ID_NONE or an ISC whose record this build compiles in. */
#define Z_BT_ACS_ISC_ENABLED(_isc)                                                                 \
	((_isc) == BT_ACS_ISC_ID_NONE ||                                                           \
	 ((_isc) == BT_ACS_ISC_ID_HIGH_SEC_GCM &&                                                  \
	  IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)) ||                                    \
	 ((_isc) == BT_ACS_ISC_ID_HIGH_SEC_CCM &&                                                  \
	  IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)) ||                                    \
	 ((_isc) == BT_ACS_ISC_ID_INTEGRITY_GMAC &&                                                \
	  IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)) ||                                   \
	 ((_isc) == BT_ACS_ISC_ID_MAC_ONLY_CMAC &&                                                 \
	  IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)))

#define Z_BT_ACS_ASSERT_ISC(_isc)                                                                  \
	BUILD_ASSERT(Z_BT_ACS_ISC_ENABLED(_isc),                                                   \
		     "ISC must be BT_ACS_ISC_ID_NONE or one whose algorithm is enabled")

/* Restriction map (§4.4.3.2): its ID and the ISCs for the map and for resources it omits. */
struct bt_acs_restriction_map {
	uint16_t map_id;         /* Type_ID 0x00 record Type_Value */
	uint16_t map_isc_id;     /* ISC protecting this map */
	uint16_t default_isc_id; /* Default Security Configuration (Table 4.19) */
};
/** @endcond */

/**
 * @brief Reserved map ID: ACS does not mediate the resources at all (§4.4.3.2).
 *
 * Every resource is reached directly through the service owning it, so the map
 * describes nothing an AC Server can hold. BT_ACS_RESTRICTION_MAP_DEFINE() fails
 * the build when given this ID.
 */
#define BT_ACS_RMAP_ID_NONE 0x0000U

/**
 * @brief Define a restriction map.
 *
 * Protected resources name the map by its symbol @p _name, so it must be unique
 * across the application; use BT_ACS_RESTRICTION_MAP_EXTERN() in other files.
 * Resources the map does not list take @p _default_isc_id.
 *
 * @param _name            C identifier of the map
 * @param _map_id          Restriction Map ID (Type_ID 0x00 record Type_Value), not 0
 * @param _map_isc_id      ISC protecting this map's descriptor, or BT_ACS_ISC_ID_NONE
 * @param _default_isc_id  ISC for resources not listed in this map (Type_ID 0x01), or
 *                         BT_ACS_ISC_ID_NONE to leave them unprotected
 *
 * Example:
 * @code
 *   BT_ACS_RESTRICTION_MAP_DEFINE(secret_map, 0x0002,
 *                                 BT_ACS_ISC_ID_HIGH_SEC_GCM,  // map descriptor
 *                                 BT_ACS_ISC_ID_NONE);         // unlisted resources
 * @endcode
 */
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define BT_ACS_RESTRICTION_MAP_DEFINE(_name, _map_id, _map_isc_id, _default_isc_id)                \
	BUILD_ASSERT((_map_id) != BT_ACS_RMAP_ID_NONE,                                             \
		     "reserved restriction map 0 is not registrable");                             \
	Z_BT_ACS_ASSERT_ISC(_map_isc_id);                                                          \
	Z_BT_ACS_ASSERT_ISC(_default_isc_id);                                                      \
	const STRUCT_SECTION_ITERABLE(bt_acs_restriction_map, _name) = {                           \
		.map_id = (_map_id),                                                               \
		.map_isc_id = (_map_isc_id),                                                       \
		.default_isc_id = (_default_isc_id),                                               \
	}
#else
#define BT_ACS_RESTRICTION_MAP_DEFINE(_name, _map_id, _map_isc_id, _default_isc_id)                \
	BUILD_ASSERT(0, "BT_ACS_RESTRICTION_MAP_DEFINE requires "                                  \
			"CONFIG_BT_ACS_FEAT_AUTHORIZATION=y")
#endif

/**
 * @brief Declare a restriction map defined in another file.
 *
 * @param _name Symbol given to BT_ACS_RESTRICTION_MAP_DEFINE()
 */
#define BT_ACS_RESTRICTION_MAP_EXTERN(_name) extern const struct bt_acs_restriction_map _name

/** @cond INTERNAL_HIDDEN */
extern const struct bt_acs_restriction_map *const z_bt_acs_initial_rmap;
/** @endcond */

/**
 * @brief Select the restriction map active when ACS starts.
 *
 * Exactly one per application: leaving it out fails the link, as does a second
 * one. The active map is AC Server state shared by every connection (§4.4.3.4);
 * an AC Client may switch it with Activate Restriction Map. It should be a map
 * whose descriptor is unprotected, so a client without keys can read it.
 *
 * @param _name Symbol given to BT_ACS_RESTRICTION_MAP_DEFINE()
 */
#define BT_ACS_INITIAL_RESTRICTION_MAP(_name)                                                      \
	const struct bt_acs_restriction_map *const z_bt_acs_initial_rmap = &(_name)

/** @cond INTERNAL_HIDDEN */
#define Z_BT_ACS_RMAP_OP_SHARED_ISC(_opcode, _isc_id) {.opcode = (_opcode), .isc_id = (_isc_id)}

#define Z_BT_ACS_ASSERT_ATT_OPCODE(_op)                                                            \
	BUILD_ASSERT(Z_BT_ACS_RMAP_ATT_OPCODES(_op, Z_BT_ACS_OPCODE_MATCHES) 0,                    \
		     "a Protected Characteristic record carries only ATT read, write, "            \
		     "notification or indication opcodes")

/* Emit a Type_ID 0x02 (_kind CHAR) or 0x03 (_kind CP) protected resource. */
#define Z_BT_ACS_RMAP_RECORD_DEFINE(_name, _map, _kind, _char_uuid, _max_ops, _isc_id, ...)        \
	Z_BT_ACS_ASSERT_ISC(_isc_id);                                                              \
	static const struct bt_acs_rmap_op_isc _name##_ops[] = {                                   \
		FOR_EACH_FIXED_ARG(Z_BT_ACS_RMAP_OP_SHARED_ISC, (,), _isc_id, __VA_ARGS__)};       \
	BUILD_ASSERT(ARRAY_SIZE(_name##_ops) >= 1U,                                                \
		     "a protected record must map at least one operation (Data_Size >= 4)");       \
	BUILD_ASSERT(ARRAY_SIZE(_name##_ops) <= (_max_ops),                                        \
		     "more operations than this record type permits");                             \
	static STRUCT_SECTION_ITERABLE(bt_acs_rmap_resource, _name) = {                            \
		.map = &(_map),                                                                    \
		.kind = (_kind),                                                                   \
		.char_uuid = (_char_uuid),                                                         \
		.ops = _name##_ops,                                                                \
		.num_ops = ARRAY_SIZE(_name##_ops),                                                \
	}
/** @endcond */

/**
 * @name Protected resources
 *
 * Each macro adds one record to a restriction map. Every listed operation is
 * bound to the one ISC given; operations left off the list are unprotected, so
 * a resource protected for only some operations lists just those. A map lists
 * a resource at most once.
 *
 * @{
 */

/**
 * @brief Protected Characteristic record (Type_ID 0x02).
 *
 * @param _name      C identifier of the record
 * @param _map       Restriction map symbol (BT_ACS_RESTRICTION_MAP_DEFINE())
 * @param _char_uuid Pointer to the characteristic UUID
 * @param _isc_id    ISC bound to every listed opcode
 * @param ...        One or more BT_ACS_RMAP_OP_ATT_* opcodes
 *
 * Example:
 * @code
 *   // write and notify protected; read is unprotected because it is not listed
 *   BT_ACS_RMAP_CHAR_DEFINE(cts_current_time, secret_map, BT_UUID_CTS_CURRENT_TIME,
 *                           BT_ACS_ISC_ID_HIGH_SEC_GCM,
 *                           BT_ACS_RMAP_OP_ATT_WRITE_REQ, BT_ACS_RMAP_OP_ATT_NOTIFY);
 * @endcode
 */
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define BT_ACS_RMAP_CHAR_DEFINE(_name, _map, _char_uuid, _isc_id, ...)                             \
	FOR_EACH(Z_BT_ACS_ASSERT_ATT_OPCODE, (;), __VA_ARGS__);                                    \
	Z_BT_ACS_RMAP_RECORD_DEFINE(_name, _map, BT_ACS_RMAP_RESOURCE_CHAR, _char_uuid,            \
				    Z_BT_ACS_RMAP_MAX_CHAR_OPS, _isc_id, __VA_ARGS__)
#else
#define BT_ACS_RMAP_CHAR_DEFINE(_name, _map, _char_uuid, _isc_id, ...)                             \
	BUILD_ASSERT(0, "BT_ACS_RMAP_CHAR_DEFINE requires CONFIG_BT_ACS_FEAT_AUTHORIZATION=y")
#endif

/**
 * @brief Protected Control Point record (Type_ID 0x03) for another service's Control Point.
 *
 * The opcodes are that service's own procedure opcodes and are opaque to ACS,
 * which enforces only the ISC before forwarding the write. Use
 * BT_ACS_RMAP_ACS_CP_DEFINE() for ACS's own Control Point.
 *
 * Plain GATT writes are authorized using the first payload byte as the Control
 * Point opcode. Opcodes bound to a nonzero ISC must use Data In; unlisted opcodes
 * and opcodes bound to BT_ACS_ISC_ID_NONE remain directly writable. Prepared,
 * executed, fragmented, and empty plain writes are rejected when this Control
 * Point has any protected procedure because their opcode cannot be classified
 * reliably from a single authorization call.
 *
 * @param _name      C identifier of the record
 * @param _map       Restriction map symbol (BT_ACS_RESTRICTION_MAP_DEFINE())
 * @param _char_uuid Pointer to the Control Point characteristic UUID
 * @param _isc_id    ISC bound to every listed procedure opcode
 * @param ...        One or more of that service's procedure opcodes
 */
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define BT_ACS_RMAP_CP_DEFINE(_name, _map, _char_uuid, _isc_id, ...)                               \
	Z_BT_ACS_RMAP_RECORD_DEFINE(_name, _map, BT_ACS_RMAP_RESOURCE_CP, _char_uuid,              \
				    Z_BT_ACS_RMAP_MAX_OPS, _isc_id, __VA_ARGS__)
#else
#define BT_ACS_RMAP_CP_DEFINE(_name, _map, _char_uuid, _isc_id, ...)                               \
	BUILD_ASSERT(0, "BT_ACS_RMAP_CP_DEFINE requires CONFIG_BT_ACS_FEAT_AUTHORIZATION=y")
#endif

/** @cond INTERNAL_HIDDEN */
/*
 * Protecting any of these strands the AC Client. Get Restriction Map ID List is
 * excluded from the protected path by ACS 1.0 Section 4.4.3.3; Abort is the
 * recovery path (Section 4.4.3.13) and cannot be reached once a failed exchange
 * has left no keys; Get ACS Feature opens the onboarding sequence of ACP 1.0
 * Appendix E; and a Data In write requires established security, which no
 * connection has before its first key exchange.
 *
 * @p _entry is applied to each opcode as _entry(_arg, opcode).
 */
#define Z_BT_ACS_CP_UNPROTECTABLE_OPCODES(_arg, _entry)                                            \
	_entry(_arg, BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST)                                 \
	_entry(_arg, BT_ACS_CP_OPCODE_ABORT)                                                       \
	_entry(_arg, BT_ACS_CP_OPCODE_GET_ACS_FEATURE)                                             \
	_entry(_arg, BT_ACS_CP_OPCODE_START_KEY_EXCHANGE)                                          \
	_entry(_arg, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH)                                           \
	_entry(_arg, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_CODE)                         \
	_entry(_arg, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_RANDOM_NUMBER)                \
	_entry(_arg, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF)

#define Z_BT_ACS_OPCODE_DIFFERS(_op, _listed) (_op) != (_listed) &&

#define Z_BT_ACS_ASSERT_CP_PROTECTABLE(_op)                                                        \
	BUILD_ASSERT(Z_BT_ACS_CP_UNPROTECTABLE_OPCODES(_op, Z_BT_ACS_OPCODE_DIFFERS) 1,            \
		     "this ACS CP procedure must stay reachable on the plain ACS CP")
/** @endcond */

/**
 * @brief Protected Control Point record (Type_ID 0x03) for ACS's own Control Point.
 *
 * Takes no UUID, so the opcodes are known to be ACS CP procedure opcodes and the
 * ones that must stay reachable on the plain ACS CP are rejected at build time.
 *
 * @param _name   C identifier of the record
 * @param _map    Restriction map symbol (BT_ACS_RESTRICTION_MAP_DEFINE())
 * @param _isc_id ISC bound to every listed procedure opcode
 * @param ...     One or more BT_ACS_CP_OPCODE_* procedure opcodes
 */
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define BT_ACS_RMAP_ACS_CP_DEFINE(_name, _map, _isc_id, ...)                                       \
	FOR_EACH(Z_BT_ACS_ASSERT_CP_PROTECTABLE, (;), __VA_ARGS__);                                \
	BT_ACS_RMAP_CP_DEFINE(_name, _map, BT_UUID_GATT_ACS_CP, _isc_id, __VA_ARGS__)
#else
#define BT_ACS_RMAP_ACS_CP_DEFINE(_name, _map, _isc_id, ...)                                       \
	BUILD_ASSERT(0, "BT_ACS_RMAP_ACS_CP_DEFINE requires CONFIG_BT_ACS_FEAT_AUTHORIZATION=y")
#endif
/** @} */

/** @brief How the user enters the Input OOB Number into the AC Server (Table 4.52). */
enum bt_acs_input_oob_action {
	/** The user pushes a control, once per unit of the number. */
	BT_ACS_INPUT_OOB_PUSH = 0x00,
	/** The user types the number. */
	BT_ACS_INPUT_OOB_NUMERIC = 0x02,
};

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
	 * @brief Display the Output OOB Number to the user.
	 *
	 * Called at Start Key Exchange when the AC Client selects Output OOB with
	 * the Output Numeric action (Tables 4.50, 4.51). The user reads the number
	 * and enters it into the AC Client before the Confirmation Code step.
	 *
	 * Required when @kconfig{CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC} is enabled.
	 *
	 * @param conn   Connection object.
	 * @param number The number to display, 1 to
	 *               @kconfig{CONFIG_BT_ACS_CONFIRMATION_OUTPUT_MAX_VALUE}.
	 */
	void (*output_oob_number)(struct bt_conn *conn, uint32_t number);

	/**
	 * @brief Ask the user for the Input OOB Number.
	 *
	 * Called at Start Key Exchange when the AC Client selects Input OOB
	 * (Table 4.50). The AC Client shows a number; the user enters it into the
	 * AC Server by @p action, and the application passes it on with
	 * bt_acs_input_oob_number() before the Confirmation Code step.
	 *
	 * Required when @kconfig{CONFIG_BT_ACS_CONFIRMATION_INPUT_NUMERIC} or
	 * @kconfig{CONFIG_BT_ACS_CONFIRMATION_INPUT_PUSH} is enabled.
	 *
	 * @param conn   Connection object.
	 * @param action How the user enters the number.
	 */
	void (*input_oob_request)(struct bt_conn *conn, enum bt_acs_input_oob_action action);
};

/**
 * @brief Provide the Input OOB Number the user entered.
 *
 * Call after the input_oob_request callback and before the AC Client sends the
 * Confirmation Code. The number becomes the AuthValue of the confirmation code
 * (§4.4.3.17.1.2).
 *
 * @param conn   Connection object.
 * @param number The number the user entered, 1 to
 *               @kconfig{CONFIG_BT_ACS_CONFIRMATION_INPUT_MAX_VALUE}.
 *
 * @retval 0         Success.
 * @retval -EINVAL   @p conn is NULL or @p number is out of range.
 * @retval -ENOTCONN No ACS connection found for @p conn.
 * @retval -ESRCH    No key exchange in progress on this connection.
 * @retval -EPERM    The key exchange does not use Input OOB confirmation.
 */
int bt_acs_input_oob_number(struct bt_conn *conn, uint32_t number);

/**
 * @brief Protected output completion callback.
 *
 * @param conn      Peer the transfer was addressed to.
 * @param err       0 on success, negative errno if the transfer failed.
 * @param user_data The @c user_data given in the output parameters.
 */
typedef void (*bt_acs_output_func_t)(struct bt_conn *conn, int err, void *user_data);

/** @brief Parameters for bt_acs_notify() and bt_acs_indicate(). */
struct bt_acs_output_params {
	/** Protected characteristic UUID; selects the resource when set. */
	const struct bt_uuid *uuid;
	/** Characteristic declaration or value attribute; used when @p uuid is NULL. */
	const struct bt_gatt_attr *attr;
	/** Characteristic value to protect and send. */
	const void *data;
	/** Length of @p data. */
	uint16_t len;
	/** Optional callback once the transfer completes or fails. */
	bt_acs_output_func_t func;
	/** User data passed to @p func. */
	void *user_data;
};

/**
 * @brief Send a protected characteristic value on ACS Data Out Notify.
 *
 * The resource is found in the active restriction map by @p params->uuid, or by
 * @p params->attr when the UUID is NULL; its ATT Notify mapping gives the ISC
 * that protects the value. ACS copies @p params->data before returning, so the
 * caller's buffer may be reused at once.
 *
 * @param conn   Target connection, or NULL for every connected peer.
 * @param params Resource, value and optional completion callback.
 *
 * @retval 0         Success; with @p conn NULL, at least one peer accepted it.
 * @retval -EINVAL   Invalid parameters.
 * @retval -ENOENT   No protected resource or ATT Notify mapping in the active map.
 * @retval -ENOTCONN No eligible ACS connection.
 * @retval -EACCES   Security, key or Data Out subscription not ready for the peer.
 * @retval -ENOMEM   Buffer or reply pool exhausted.
 */
int bt_acs_notify(struct bt_conn *conn, const struct bt_acs_output_params *params);

/**
 * @brief Send a protected characteristic value on ACS Data Out Indicate.
 *
 * As bt_acs_notify(), using the resource's ATT Indicate mapping. The callback
 * runs once the indication is confirmed or fails.
 *
 * @param conn   Target connection, or NULL for every connected peer.
 * @param params Resource, value and optional completion callback.
 *
 * @retval 0         Success; with @p conn NULL, at least one peer accepted it.
 * @retval -EINVAL   Invalid parameters.
 * @retval -ENOENT   No protected resource or ATT Indicate mapping in the active map.
 * @retval -ENOTCONN No eligible ACS connection.
 * @retval -EACCES   Security, key or Data Out subscription not ready for the peer.
 * @retval -ENOMEM   Buffer or reply pool exhausted.
 */
int bt_acs_indicate(struct bt_conn *conn, const struct bt_acs_output_params *params);

/**
 * @brief Initialize the Authorization Control Service.
 *
 * @details Must be called after bt_enable() and after all GATT services have been registered, so
 * that ATT handles for protected characteristics can be resolved from the fully-built GATT
 * attribute table.
 *
 * @note If @kconfig{CONFIG_BT_SETTINGS} is enabled, settings_load() must be called before this
 * function. ACS settings handlers are registered at link time, so loading first restores stored
 * parent keys before ACS starts accepting connections. Initialization then registers the
 * bond-deletion callback that maintains those restored records at runtime.
 *
 * @note Start advertising only after this function returns successfully. Connections established
 * before ACS creates their per-connection state cannot use protected resources and must reconnect.
 *
 * @param cb Application callbacks.
 *
 * @retval 0 Success.
 * @retval -EALREADY ACS has already been initialized.
 * @retval -EINVAL A restriction map or protected resource is malformed, or names
 *         an operation the server cannot enforce.
 * @retval -ENOENT A protected characteristic could not be resolved to a GATT
 *         handle.
 * @retval -EEXIST Two restriction maps share an ID, a protected UUID names more
 *         than one characteristic value, or a map lists a resource twice.
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
