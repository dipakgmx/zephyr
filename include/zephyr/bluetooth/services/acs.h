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

/** @cond INTERNAL_HIDDEN */
/*
 * Most procedures one Protected Control Point record can list: its Data_Size is
 * one octet (Table 4.4) and each procedure takes four (Table 4.20).
 */
#define Z_BT_ACS_RMAP_MAX_PROCEDURES (UINT8_MAX / 4U)

/* Operations a Protected Characteristic record can protect. */
#define Z_BT_ACS_RMAP_CHAR_OPERATIONS                                                              \
	(BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_INDICATE)

/* Protected resource kind selecting the record Type_ID (0x02 char, 0x03 CP). */
enum bt_acs_rmap_resource_kind {
	BT_ACS_RMAP_RESOURCE_CHAR,
	BT_ACS_RMAP_RESOURCE_CP,
};

/* GATT binding of a protected resource, filled in by bt_acs_init(). */
struct bt_acs_rmap_binding {
	uint16_t resource_handle;              /* ACS Resource Handle */
	const struct bt_gatt_attr *value_attr; /* GATT value attribute */
	uint8_t props;                         /* GATT characteristic properties */
};

struct bt_acs_restriction_map;

/* Resource a restriction map protects: Protected Characteristic or Control Point (Table 4.19). */
struct bt_acs_rmap_resource {
	const struct bt_acs_restriction_map *map; /* restriction map listing this resource */
	enum bt_acs_rmap_resource_kind kind;      /* Protected Characteristic or Control Point */
	const struct bt_uuid *char_uuid;          /* UUID of the characteristic backing it */
	uint16_t isc_id;                          /* ISC of every protected operation */
	uint8_t operations;                       /* characteristic: BT_GATT_CHRC_* protected */
	const uint16_t *procedures;               /* Control Point: protected procedure opcodes */
	uint8_t num_procedures;                   /* entries in procedures */
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

/* Decimal digits of a map ID, 0 when it does not fit 16 bits. */
#define Z_BT_ACS_RMAP_ID_DIGITS(_id)                                                               \
	(((_id) < 10)            ? 1U                                                              \
	 : ((_id) < 100)         ? 2U                                                              \
	 : ((_id) < 1000)        ? 3U                                                              \
	 : ((_id) < 10000)       ? 4U                                                              \
	 : ((_id) <= UINT16_MAX) ? 5U                                                              \
				 : 0U)

/*
 * A map's section is keyed by its ID and sorted numerically, which needs the ID
 * spelled in decimal: then its text is exactly as long as its value has digits.
 */
#define Z_BT_ACS_ASSERT_RMAP_ID(_id)                                                               \
	BUILD_ASSERT(sizeof(STRINGIFY(_id)) - 1U == Z_BT_ACS_RMAP_ID_DIGITS(_id),                  \
		     "restriction map ID must be a decimal literal without leading zeroes or "     \
		     "suffix, or a macro expanding to one")

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
 * Resources the map does not list take @p _default_isc_id. The Restriction Map
 * ID List gives the maps in ascending order of @p _map_id.
 *
 * @param _name            C identifier of the map
 * @param _map_id          Restriction Map ID (Type_ID 0x00 record Type_Value), not 0.
 *                         A decimal integer literal without leading zeroes or
 *                         suffix (e.g. `2`), or a macro expanding to one.
 * @param _map_isc_id      ISC protecting this map's descriptor, or BT_ACS_ISC_ID_NONE
 * @param _default_isc_id  ISC for resources not listed in this map (Type_ID 0x01), or
 *                         BT_ACS_ISC_ID_NONE to leave them unprotected. It also
 *                         applies to ACS Control Point procedures the map does not
 *                         list.
 *
 * Example:
 * @code
 *   BT_ACS_RESTRICTION_MAP_DEFINE(secret_map, 2,
 *                                 BT_ACS_ISC_ID_HIGH_SEC_GCM,  // map descriptor
 *                                 BT_ACS_ISC_ID_NONE);         // unlisted resources
 * @endcode
 */
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define BT_ACS_RESTRICTION_MAP_DEFINE(_name, _map_id, _map_isc_id, _default_isc_id)                \
	BUILD_ASSERT((_map_id) != BT_ACS_RMAP_ID_NONE,                                             \
		     "reserved restriction map 0 is not registrable");                             \
	Z_BT_ACS_ASSERT_RMAP_ID(_map_id);                                                          \
	Z_BT_ACS_ASSERT_ISC(_map_isc_id);                                                          \
	Z_BT_ACS_ASSERT_ISC(_default_isc_id);                                                      \
	const STRUCT_SECTION_ITERABLE_NAMED(bt_acs_restriction_map, _CONCAT(_, _map_id),           \
					    _name) = {                                             \
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
 * an AC Client may switch it with Activate Restriction Map. A client without
 * keys starts from Get Restriction Map ID List and Get Information Security
 * Configuration Descriptor, so the initial map should leave the latter
 * unprotected; its own descriptor may be protected.
 *
 * @param _name Symbol given to BT_ACS_RESTRICTION_MAP_DEFINE()
 */
#define BT_ACS_INITIAL_RESTRICTION_MAP(_name)                                                      \
	const struct bt_acs_restriction_map *const z_bt_acs_initial_rmap = &(_name)

/**
 * @name Protected resources
 *
 * Each macro adds one record to a restriction map. Every listed operation is
 * bound to the one ISC given. Characteristic operations left off the list are
 * unprotected, so a characteristic protected for only some operations lists
 * just those. ACS Control Point procedures left off the list take the map's
 * default ISC (§4.4.3.2). A map lists a resource at most once.
 *
 * @{
 */

/**
 * @brief Protected Characteristic record (Type_ID 0x02).
 *
 * @p _operations names what is protected with characteristic property flags:
 * BT_GATT_CHRC_READ, BT_GATT_CHRC_WRITE, BT_GATT_CHRC_NOTIFY and
 * BT_GATT_CHRC_INDICATE, combined with |. BT_GATT_CHRC_WRITE covers every way
 * of writing: with or without response, and long writes. Each operation must be
 * one the characteristic offers, or bt_acs_init() fails.
 *
 * A protected operation is refused over plain ATT whichever ATT opcode carries
 * it, and the restriction map descriptor lists the ATT opcodes the
 * characteristic's properties offer for it (ACS 1.0 Section 3.1).
 *
 * @param _name       C identifier of the record
 * @param _map        Restriction map symbol (BT_ACS_RESTRICTION_MAP_DEFINE())
 * @param _char_uuid  Pointer to the characteristic UUID
 * @param _isc_id     ISC bound to every protected operation
 * @param _operations Protected operations, BT_GATT_CHRC_* flags
 *
 * Example:
 * @code
 *   // write and notify protected; read is unprotected because it is not named
 *   BT_ACS_PROTECTED_CHARACTERISTIC_DEFINE(cts_current_time, secret_map,
 *                                          BT_UUID_CTS_CURRENT_TIME,
 *                                          BT_ACS_ISC_ID_HIGH_SEC_GCM,
 *                                          BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY);
 * @endcode
 */
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define BT_ACS_PROTECTED_CHARACTERISTIC_DEFINE(_name, _map, _char_uuid, _isc_id, _operations)      \
	Z_BT_ACS_ASSERT_ISC(_isc_id);                                                              \
	BUILD_ASSERT((_operations) != 0U &&                                                        \
			     ((_operations) & ~Z_BT_ACS_RMAP_CHAR_OPERATIONS) == 0U,               \
		     "protect one or more of BT_GATT_CHRC_READ, _WRITE, _NOTIFY, _INDICATE");      \
	static STRUCT_SECTION_ITERABLE(bt_acs_rmap_resource, _name) = {                            \
		.map = &(_map),                                                                    \
		.kind = BT_ACS_RMAP_RESOURCE_CHAR,                                                 \
		.char_uuid = (_char_uuid),                                                         \
		.isc_id = (_isc_id),                                                               \
		.operations = (_operations),                                                       \
	}
#else
#define BT_ACS_PROTECTED_CHARACTERISTIC_DEFINE(_name, _map, _char_uuid, _isc_id, _operations)      \
	BUILD_ASSERT(0, "BT_ACS_PROTECTED_CHARACTERISTIC_DEFINE requires "                         \
			"CONFIG_BT_ACS_FEAT_AUTHORIZATION=y")
#endif

/**
 * @brief Protected Control Point record (Type_ID 0x03) for the ACS Control Point.
 *
 * Protects ACS Control Point procedures, named by their BT_ACS_CP_OPCODE_*
 * values. Procedures left off the list take the map's default ISC (§4.4.3.2).
 * Any procedure may be listed except Get Restriction Map ID List (§4.4.3.3),
 * which makes bt_acs_init() fail. A protected procedure is only reachable on
 * ACS Data In, which needs established security. Protecting Get ACS Feature or
 * the key exchange procedures, here or through the map's default ISC, leaves a
 * client without keys unable to discover or establish security while the map
 * is active; keep a map that leaves them unprotected to fall back to.
 *
 * A plain GATT write is authorized per attribute, so another service's Control
 * Point is protected as a whole, with BT_ACS_PROTECTED_CHARACTERISTIC_DEFINE()
 * and BT_GATT_CHRC_WRITE.
 *
 * @param _name   C identifier of the record
 * @param _map    Restriction map symbol (BT_ACS_RESTRICTION_MAP_DEFINE())
 * @param _isc_id ISC bound to every listed procedure
 * @param ...     One or more BT_ACS_CP_OPCODE_* values
 *
 * Example:
 * @code
 *   BT_ACS_PROTECTED_CONTROL_POINT_DEFINE(acs_cp_secret, secret_map, BT_ACS_ISC_ID_HIGH_SEC_GCM,
 *                                         BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH);
 * @endcode
 */
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#define BT_ACS_PROTECTED_CONTROL_POINT_DEFINE(_name, _map, _isc_id, ...)                           \
	Z_BT_ACS_ASSERT_ISC(_isc_id);                                                              \
	static const uint16_t _name##_procedures[] = {__VA_ARGS__};                                \
	BUILD_ASSERT(ARRAY_SIZE(_name##_procedures) >= 1U &&                                       \
			     ARRAY_SIZE(_name##_procedures) <= Z_BT_ACS_RMAP_MAX_PROCEDURES,       \
		     "a Protected Control Point record lists 1 to 63 procedures");                 \
	static STRUCT_SECTION_ITERABLE(bt_acs_rmap_resource, _name) = {                            \
		.map = &(_map),                                                                    \
		.kind = BT_ACS_RMAP_RESOURCE_CP,                                                   \
		.char_uuid = BT_UUID_GATT_ACS_CP,                                                  \
		.isc_id = (_isc_id),                                                               \
		.procedures = _name##_procedures,                                                  \
		.num_procedures = ARRAY_SIZE(_name##_procedures),                                  \
	}
#else
#define BT_ACS_PROTECTED_CONTROL_POINT_DEFINE(_name, _map, _isc_id, ...)                           \
	BUILD_ASSERT(0, "BT_ACS_PROTECTED_CONTROL_POINT_DEFINE requires "                          \
			"CONFIG_BT_ACS_FEAT_AUTHORIZATION=y")
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
	 * parent key.
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
