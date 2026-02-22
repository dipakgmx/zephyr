/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_KEYS_H_
#define ACS_KEYS_H_

/*
 * Key table of one connection (bt_acs_conn::keys), and the persistent copy of
 * each peer's ECDH key. Table keys are volatile and last until they are
 * removed or the connection ends. Only the ECDH key is stored: every key below
 * it carries nonce state and is derived again on each connection (§3.6.4.3).
 */

#include <stdbool.h>
#include <stdint.h>

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Destroy a PSA key, if any, and clear the caller's key ID. */
void acs_keys_destroy(psa_key_id_t *key_id);

/* Derive a new key with attrs from secret using the KDF salt and info (HKDF-SHA-256). */
int acs_keys_derive(psa_key_id_t secret, const struct bt_acs_kdf_params *kdf,
		    const psa_key_attributes_t *attrs, psa_key_id_t *key);

/* Remove every key, clear all nonce state and bind each security algorithm to its record. */
void acs_keys_reset(struct bt_acs_conn *acs_conn);

/* Replace the ECDH key with a volatile copy of src; keys derived from the old one are removed. */
int acs_keys_set_ecdh(struct bt_acs_conn *acs_conn, psa_key_id_t src);

/*
 * Standalone KDF key exchange (§4.4.3.17.2.1): derive the key of every security
 * algorithm from the ECDH key. Return -EAGAIN if no ECDH key is installed.
 */
int acs_keys_derive_algs(struct bt_acs_conn *acs_conn, const struct bt_acs_kdf_params *kdf);

/* Remove key_id and every key derived from it. Nonce prefixes are kept. */
void acs_keys_remove(struct bt_acs_conn *acs_conn, uint16_t key_id);

/* Return true when key_id holds key material on acs_conn. */
bool acs_keys_installed(struct bt_acs_conn *acs_conn, uint16_t key_id);

/* Write the Key_IDs holding key material, ECDH first, to key_ids; return the count. */
uint8_t acs_keys_list(struct bt_acs_conn *acs_conn, uint16_t key_ids[ACS_KEY_COUNT]);

/* Security algorithm for key_id, or NULL if key_id is not a security algorithm record. */
struct acs_sec_alg *acs_keys_alg(struct bt_acs_conn *acs_conn, uint16_t key_id);

/* Register the bond-deleted callback that forgets a peer's stored ECDH key. */
void acs_keys_init(void);

/* Store a persistent copy of the ECDH key for the peer. A peer on an RPA is skipped. */
void acs_keys_save(struct bt_acs_conn *acs_conn);

/*
 * Install the peer's stored ECDH key, if any. Security is not established:
 * the peer runs a KDF key exchange for fresh algorithm keys and nonces.
 */
void acs_keys_restore(struct bt_acs_conn *acs_conn);

/* Erase the peer's stored ECDH key. */
void acs_keys_forget(struct bt_acs_conn *acs_conn);

/* Erase the stored ECDH key of every peer except this connection's. */
void acs_keys_forget_all_except(struct bt_acs_conn *acs_conn);

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
/* Write the AC_Server_Nonce_Fixed_Value of key_id to out in wire order (Table 4.45). */
int acs_keys_server_nonce_fixed(struct bt_acs_conn *acs_conn, uint16_t key_id, uint8_t *out);

/*
 * Store an AC_Client_Nonce_Fixed_Value given in wire order (§4.4.3.18).
 * Return -EEXIST if it equals a fixed nonce already stored on the AC Server.
 */
int acs_keys_set_client_nonce_fixed(struct bt_acs_conn *acs_conn, struct acs_sec_alg *alg,
				    const uint8_t *value);
#else
static inline int acs_keys_server_nonce_fixed(struct bt_acs_conn *acs_conn, uint16_t key_id,
					      uint8_t *out)
{
	ARG_UNUSED(acs_conn);
	ARG_UNUSED(key_id);
	ARG_UNUSED(out);
	return -ENOTSUP;
}
#endif /* CONFIG_BT_ACS_HAS_NONCE_FIXED */

#ifdef __cplusplus
}
#endif

#endif /* ACS_KEYS_H_ */
