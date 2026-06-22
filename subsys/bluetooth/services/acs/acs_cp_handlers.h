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
int acs_cp_all_active_get(struct acs_reply *reply);

int acs_all_active_step_isc(struct acs_reply *reply);
int acs_all_active_step_key(struct acs_reply *reply);
int acs_all_active_step_rc(struct acs_reply *reply);
#endif /* CONFIG_BT_ACS_DESCRIPTORS */

/** @brief Handle Get Restriction Map Descriptor CP request (§4.4.3.2). */
int acs_cp_handle_get_restriction_map_descriptor(struct acs_reply *reply,
						 struct net_buf_simple *buf);

/** @brief Handle Get Restriction Map ID List CP request (§4.4.3.3). */
int acs_cp_handle_get_restriction_map_id_list(struct acs_reply *reply);

/** @brief Handle Activate Restriction Map CP request (§4.4.3.4). */
int acs_cp_handle_activate_restriction_map(struct acs_reply *reply, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

#if IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP)
/** @brief Handle Get Resource Handle to UUID Map CP request (§4.4.3.5). */
int acs_cp_handle_get_resource_handle_uuid_map(struct acs_reply *reply);
#endif /* CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP */

/** @brief Handle Get Service and Characteristic UUIDs for Characteristic Resource CP request
 * (§4.4.3.6). */
int acs_cp_handle_get_svc_char_uuids(struct acs_reply *reply, struct net_buf_simple *buf);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/** @brief Handle Get ISC Descriptor CP request (§4.4.3.7). */
int acs_cp_handle_get_isc_descriptor(struct acs_reply *reply, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
/** @brief Handle Get Key Descriptor CP request (§4.4.3.8).
 *  Parses the Key_ID operand from @p buf and sends a Key_Descriptor indication via @p proc. */
int acs_cp_handle_get_key_descriptor(struct acs_reply *reply, struct net_buf_simple *buf);

/** @brief Handle Get Current Key List CP request (§4.4.3.9). */
int acs_cp_kex_get_current_key_list(struct acs_reply *reply);

/** @brief Handle Start Key Exchange CP request (§4.4.3.10). */
int acs_cp_kex_start(struct acs_reply *reply, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_ANY_KEY_EXCHANGE */

#if IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)
/** @brief Invalidate Key request operand (Table 4.55). */
struct acs_cp_invalidate_key_req {
	uint16_t key_id; /* little-endian */
} __packed;

/** @brief Handle Invalidate All Established Security CP request (§4.4.3.11). */
int acs_sec_mgmt_invalidate_all(struct acs_reply *reply);

/** @brief Handle Invalidate Key CP request (§4.4.3.12).
 *  Parses the Key_ID operand from @p buf and invalidates the matching session key. */
int acs_sec_mgmt_invalidate_key(struct acs_reply *reply, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY */

#if IS_ENABLED(CONFIG_BT_ACS_ABORT)
/** @brief Handle Abort CP request (§4.4.3.13). */
int acs_sec_mgmt_abort(struct acs_reply *reply);
#endif /* CONFIG_BT_ACS_ABORT */

#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
/** @brief Set Security Controls Switch operand (Table 4.56). */
struct acs_cp_sec_switch_req {
	uint8_t switch_state;
} __packed;

/** @brief Handle Set Security Controls Switch CP request (§4.4.3.14). */
int acs_sec_mgmt_set_security_switch(struct acs_reply *reply, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH */

/** @brief Handle Get ACS Feature CP request (§4.4.3.16). */
int acs_cp_handle_get_feature(struct acs_reply *reply, struct net_buf_simple *buf);

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
/** @brief Handle Key Exchange ECDH Public Key CP request (§4.4.3.17). */
int acs_cp_kex_exchange_ecdh(struct acs_reply *reply, struct net_buf_simple *buf);
/** @brief Handle Key Exchange ECDH Confirmation Code CP request (§4.4.3.17.1.2). */
int acs_cp_kex_ecdh_confirm_code(struct acs_reply *reply, struct net_buf_simple *buf);
/** @brief Handle Key Exchange ECDH Confirmation Random Number CP request (§4.4.3.17.1.3). */
int acs_cp_kex_ecdh_confirm_rand(struct acs_reply *reply, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) || IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
/** @brief Handle Key Exchange KDF Public Key CP request (§4.4.3.17.2). */
int acs_cp_kex_exchange_kdf(struct acs_reply *reply, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF || CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
/** @brief Handle Set AC Client Nonce Fixed CP request (§4.4.3.18). */
int acs_cp_handle_set_client_nonce_fixed(struct acs_reply *reply, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_HAS_NONCE_FIXED */

#if IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)
/** @brief Handle Get ATT MTU CP request (§4.4.3.19). */
int acs_cp_handle_att_mtu(struct acs_reply *reply);
#endif /* CONFIG_BT_ACS_ATT_MTU */

#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)
/** @brief Handle Initiate Pairing CP request (§4.4.3.20). */
int acs_sec_mgmt_initiate_pairing(struct acs_reply *reply);
#endif /* CONFIG_BT_ACS_INITIATE_PAIRING */

#endif /* BT_GATT_ACS_CP_HANDLERS_H */
