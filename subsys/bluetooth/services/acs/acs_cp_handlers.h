/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_CP_HANDLERS_H
#define BT_GATT_ACS_CP_HANDLERS_H

#include "acs_internal.h"

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#if IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS)
/** @brief Handle Get All Active Descriptors CP request (§4.4.3.1). */
void acs_cp_all_active_get(acs_procedure *proc);
#endif /* CONFIG_BT_ACS_DESCRIPTORS */

/** @brief Handle Get Restriction Map Descriptor CP request (§4.4.3.2). */
void acs_cp_handle_get_restriction_map_descriptor(acs_procedure *proc,
						  struct net_buf_simple *buf);

/** @brief Handle Get Restriction Map ID List CP request (§4.4.3.3). */
void acs_cp_handle_get_restriction_map_id_list(acs_procedure *proc);

/** @brief Handle Activate Restriction Map CP request (§4.4.3.4). */
void acs_cp_handle_activate_restriction_map(acs_procedure *proc, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

#if IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP)
/** @brief Handle Get Resource Handle to UUID Map CP request (§4.4.3.5). */
void acs_cp_handle_get_resource_handle_uuid_map(acs_procedure *proc);
#endif /* CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP */

/** @brief Handle Get Service and Characteristic UUIDs for Characteristic Resource CP request
 * (§4.4.3.6). */
void acs_cp_handle_get_svc_char_uuids(acs_procedure *proc, struct net_buf_simple *buf);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/** @brief Handle Get ISC Descriptor CP request (§4.4.3.7). */
void acs_cp_handle_get_isc_descriptor(acs_procedure *proc, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
/** @brief Handle Get Key Descriptor CP request (§4.4.3.8).
 *  Parses the Key_ID operand from @p buf and sends a Key_Descriptor indication via @p proc. */
void acs_cp_handle_get_key_descriptor(acs_procedure *proc, struct net_buf_simple *buf);

/** @brief Handle Get Current Key List CP request (§4.4.3.9). */
void acs_cp_kex_get_current_key_list(acs_procedure *proc);

/** @brief Handle Start Key Exchange CP request (§4.4.3.10). */
void acs_cp_kex_start(acs_procedure *proc, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_ANY_KEY_EXCHANGE */

#if IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)
/** @brief Invalidate Key request operand (Table 4.55). */
struct acs_cp_invalidate_key_req {
	uint16_t key_id; /* little-endian */
} __packed;

/** @brief Handle Invalidate All Established Security CP request (§4.4.3.11). */
void acs_sec_mgmt_invalidate_all(acs_procedure *proc);

/** @brief Handle Invalidate Key CP request (§4.4.3.12).
 *  Parses the Key_ID operand from @p buf and invalidates the matching session key. */
void acs_sec_mgmt_invalidate_key(acs_procedure *proc, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY */

#if IS_ENABLED(CONFIG_BT_ACS_ABORT)
/** @brief Handle Abort CP request (§4.4.3.13). */
void acs_sec_mgmt_abort(acs_procedure *proc);
#endif /* CONFIG_BT_ACS_ABORT */

#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
/** @brief Set Security Controls Switch operand (Table 4.56). */
struct acs_cp_sec_switch_req {
	uint8_t switch_state;
} __packed;

/** @brief Handle Set Security Controls Switch CP request (§4.4.3.14). */
void acs_sec_mgmt_set_security_switch(acs_procedure *proc, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_URI)
/** @brief Get Key URI request operand (Table 4.57). */
struct acs_cp_get_key_uri_req {
	uint16_t key_id; /* little-endian */
} __packed;

/** @brief Key URI Response fixed header (Table 4.58). */
struct acs_cp_key_uri_rsp_hdr {
	uint16_t key_id; /* little-endian */
			 /* uint8_t uri[uri_len] follows */
} __packed;

/** @brief Handle Get Key URI CP request (§4.4.3.15). */
void acs_sec_mgmt_get_key_uri(acs_procedure *proc, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_KEY_URI */

/** @brief Handle Get ACS Feature CP request (§4.4.3.16). */
void acs_cp_handle_get_feature(acs_procedure *proc, struct net_buf_simple *buf);

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
/** @brief Handle Key Exchange ECDH Public Key CP request (§4.4.3.17). */
void acs_cp_kex_exchange_ecdh(acs_procedure *proc, struct net_buf_simple *buf);
/** @brief Handle Key Exchange ECDH Confirmation Code CP request (§4.4.3.17.1.2). */
void acs_cp_kex_ecdh_confirm_code(acs_procedure *proc, struct net_buf_simple *buf);
/** @brief Handle Key Exchange ECDH Confirmation Random Number CP request (§4.4.3.17.1.3). */
void acs_cp_kex_ecdh_confirm_rand(acs_procedure *proc, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) || IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
/** @brief Handle Key Exchange KDF Public Key CP request (§4.4.3.17.2). */
void acs_cp_kex_exchange_kdf(acs_procedure *proc, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF || CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
/** @brief Handle Set AC Client Nonce Fixed CP request (§4.4.3.18). */
void acs_cp_handle_set_client_nonce_fixed(acs_procedure *proc, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_HAS_NONCE_FIXED */

#if IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)
/** @brief Handle Get ATT MTU CP request (§4.4.3.19). */
void acs_cp_handle_att_mtu(acs_procedure *proc);
#endif /* CONFIG_BT_ACS_ATT_MTU */

#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)
/** @brief Handle Initiate Pairing CP request (§4.4.3.20). */
void acs_sec_mgmt_initiate_pairing(acs_procedure *proc);
#endif /* CONFIG_BT_ACS_INITIATE_PAIRING */

#endif /* BT_GATT_ACS_CP_HANDLERS_H */
