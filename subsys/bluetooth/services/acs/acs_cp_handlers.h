/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_CP_HANDLERS_H
#define BT_GATT_ACS_CP_HANDLERS_H

#include "acs_types.h"
#include "acs_cp_operands.h"

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
/*
 * Get All Active Descriptors composes three procedures and is declared in
 * acs_descs.h, alongside the continuation contract it uses.
 */

/* Handle the Get Restriction Map Descriptor procedure (§4.4.3.2). */
struct acs_cp_result acs_cp_handle_get_restriction_map_descriptor(struct acs_reply *reply,
								  struct net_buf_simple *buf);

/* Handle the Get Restriction Map ID List procedure (§4.4.3.3). */
struct acs_cp_result acs_cp_handle_get_restriction_map_id_list(struct acs_reply *reply,
							       struct net_buf_simple *payload);

/* Handle the Activate Restriction Map procedure (§4.4.3.4). */
struct acs_cp_result acs_cp_handle_activate_restriction_map(struct acs_reply *reply,
							    struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

/* Handle the Get Resource Handle to UUID Map procedure (§4.4.3.5). */
struct acs_cp_result acs_cp_handle_get_resource_handle_uuid_map(struct acs_reply *reply,
								struct net_buf_simple *payload);

/* Handle Get Service and Characteristic UUIDs for Characteristic Resource (§4.4.3.6). */
struct acs_cp_result acs_cp_handle_get_svc_char_uuids(struct acs_reply *reply,
						      struct net_buf_simple *buf);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
/* Handle the Get ISC Descriptor procedure (§4.4.3.7). */
struct acs_cp_result acs_cp_handle_get_isc_descriptor(struct acs_reply *reply,
						      struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/*
 * Handle the Get Key Descriptor procedure (§4.4.3.8). Append its data to the
 * response buffer on reply.
 */
struct acs_cp_result acs_cp_handle_get_key_descriptor(struct acs_reply *reply,
						      struct net_buf_simple *buf);

/* Handle the Get Current Key List procedure (§4.4.3.9). */
struct acs_cp_result acs_cp_kex_get_current_key_list(struct acs_reply *reply,
						     struct net_buf_simple *payload);

/* Handle the Start Key Exchange procedure (§4.4.3.10). */
struct acs_cp_result acs_cp_kex_start(struct acs_reply *reply, struct net_buf_simple *buf);

/* Invalidate Key request operand (Table 4.55). */
struct acs_cp_invalidate_key_req {
	uint16_t key_id; /* Key_ID, little-endian */
} __packed;

/* Handle the Invalidate All Established Security procedure (§4.4.3.11). */
struct acs_cp_result acs_sec_mgmt_invalidate_all(struct acs_reply *reply,
						 struct net_buf_simple *payload);

/* Handle the Invalidate Key procedure (§4.4.3.12). */
struct acs_cp_result acs_sec_mgmt_invalidate_key(struct acs_reply *reply,
						 struct net_buf_simple *buf);

/* Remove a KDF or algorithm key after its protected response has been sent. */
void acs_sec_mgmt_remove_child_key(struct bt_acs_conn *acs_conn, uint16_t key_id);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
/* Set Security Controls Switch operand (Table 4.56). */
struct acs_cp_sec_switch_req {
	uint8_t switch_state;
} __packed;

/* Handle the Set Security Controls Switch procedure (§4.4.3.14). */
struct acs_cp_result acs_sec_mgmt_set_security_switch(struct acs_reply *reply,
						      struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH */

/* Handle the Get ACS Feature procedure (§4.4.3.16). */
struct acs_cp_result acs_cp_handle_get_feature(struct acs_reply *reply, struct net_buf_simple *buf);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/* Handle the Key Exchange ECDH Public Key procedure (§4.4.3.17). */
struct acs_cp_result acs_cp_kex_exchange_ecdh(struct acs_reply *reply, struct net_buf_simple *buf);
/* Handle Key Exchange ECDH Confirmation Code (§4.4.3.17.1.2). */
struct acs_cp_result acs_cp_kex_ecdh_confirm_code(struct acs_reply *reply,
						  struct net_buf_simple *buf);
/* Handle Key Exchange ECDH Confirmation Random Number (§4.4.3.17.1.3). */
struct acs_cp_result acs_cp_kex_ecdh_confirm_rand(struct acs_reply *reply,
						  struct net_buf_simple *buf);
/* Handle the Key Exchange KDF procedure (§4.4.3.17.2). */
struct acs_cp_result acs_cp_kex_exchange_kdf(struct acs_reply *reply, struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
/* Handle the Set AC Client Nonce Fixed procedure (§4.4.3.18). */
struct acs_cp_result acs_cp_handle_set_client_nonce_fixed(struct acs_reply *reply,
							  struct net_buf_simple *buf);
#endif /* CONFIG_BT_ACS_HAS_NONCE_FIXED */

#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)
/* Handle the Initiate Pairing procedure (§4.4.3.20). */
struct acs_cp_result acs_sec_mgmt_initiate_pairing(struct acs_reply *reply,
						   struct net_buf_simple *payload);
#endif /* CONFIG_BT_ACS_INITIATE_PAIRING */

#endif /* BT_GATT_ACS_CP_HANDLERS_H */
