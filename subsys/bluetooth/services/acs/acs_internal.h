/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_INTERNAL_H_
#define BT_GATT_ACS_INTERNAL_H_

/*
 * acs_internal.h is the aggregator for the service's internal API. It pulls in
 * every layer header so each .c file can keep a single "internal" include.
 *
 * Layering (skim top-to-bottom to follow the data flow):
 *
 *   acs_wire_constants.h  - on-the-wire field sizes / opcodes
 *   acs_types.h           - core structs (acs_frame, acs_reply,
 *                           bt_acs_conn) and enumerations shared across layers
 *   acs_util.h            - small inline helpers
 *
 *   acs_runtime.h         - GATT-write entrypoints + frame dispatch + Data In
 *                           unwrap. This is where every inbound PDU enters.
 *   acs_reply.h           - reply lifecycle (alloc/free/submit/continue/abort),
 *                           reply staging + send, CP opcode dispatch.
 *   acs_crypto.h          - session crypto + key exchange API.
 *
 * The remaining declarations below cover service-level concerns that don't
 * belong to any single layer: buffer pool, GATT attribute accessors, CCC
 * checks, connection lifecycle, session persistence, and global init/policy
 * hooks.
 */

#include "acs_wire_constants.h"
#include "acs_types.h"
#include "acs_util.h"

#include "acs_runtime.h"
#include "acs_reply.h"
#include "acs_crypto.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Destroy one PSA key handle, log failures, and clear the caller's id. */
void acs_psa_destroy_key(psa_key_id_t *key_id);

/**
 * @brief Materialize an operational exchange key from a PSA derivation op.
 *
 * Internal helper used by key exchange to output an AES exchange key directly
 * into the runtime slot without a plaintext import round-trip.
 */
int acs_crypto_output_exchange_key(struct bt_acs_key_desc_runtime *key_runtime,
				   psa_key_derivation_operation_t *op, size_t key_len);

/**
 * @brief Materialize a derivation-capable exchange key from a PSA derivation op.
 *
 * Internal helper used by key exchange to output the paired derive handle for a
 * runtime exchange key.
 */
int acs_crypto_output_exchange_derive_key(struct bt_acs_key_desc_runtime *key_runtime,
					  psa_key_derivation_operation_t *op, size_t key_len);

/**
 * @brief Copy a runtime exchange-key pair into persistent PSA key slots.
 *
 * Session persistence uses this to duplicate both the operational exchange key
 * and its derive-capable twin in-keystore under persistent ids.
 */
int acs_crypto_copy_key_to_persistent(const struct bt_acs_key_desc_runtime *parent,
				      psa_key_id_t dst_id, psa_key_id_t dst_derive_id);

/**
 * @brief Restore a persistent PSA exchange-key pair into a volatile runtime slot.
 *
 * Session restore uses this to duplicate both persistent key handles back into
 * the connection-owned runtime slot.
 */
int acs_crypto_copy_persistent_key_to_runtime(psa_key_id_t src_id, psa_key_id_t src_derive_id,
					      struct bt_acs_key_desc_runtime *parent);

/**
 * @brief Initialise per-record nonce state from @p parent.
 *
 * Resets TX/RX counters to fresh-session state and seeds the fixed nonce part:
 * SEQ_DIFF_FIXED lazily generates random server_nonce_fixed bytes when none
 * are set; SEQ_EVEN_ODD HKDF-derives a shared prefix from parent->derive_key_id
 * (no plaintext IKM).  Called from acs_crypto_bind_algorithm_keys per record;
 * exposed here because the HKDF helpers live in acs_key_exchange.c.
 */
int acs_derive_nonce_state(struct bt_acs_key_desc_runtime *runtime,
			   const struct bt_acs_key_desc_runtime *parent);

/**
 * @brief Allocate a buffer from the shared ACS net_buf pool.
 *
 * Uses reference counting for automatic lifetime management. Call net_buf_unref()
 * to release the buffer when done, or transfer ownership by passing the net_buf
 * pointer without unreferencing.
 *
 * @param timeout  Timeout for allocation (K_NO_WAIT for non-blocking).
 *
 * @return Pointer to net_buf with ACS_BUF_SIZE data capacity, or NULL if pool exhausted.
 */
struct net_buf *acs_buf_alloc(k_timeout_t timeout);

/**
 * @brief Release a buffer back to the shared ACS net_buf pool.
 *
 * Wrapper around net_buf_unref for pool buffers allocated via acs_buf_alloc().
 *
 * @param buf  Buffer to release (NULL-safe).
 */
void acs_buf_free(struct net_buf *buf);

const struct bt_gatt_attr *acs_attr_status(void);
const struct bt_gatt_attr *acs_attr_cp(void);
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
const struct bt_gatt_attr *acs_attr_don(void);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
const struct bt_gatt_attr *acs_attr_doi(void);
#endif

/** @brief Check CP indication CCC is enabled. Returns 0 or negative errno. */
int acs_cp_ccc_check(struct bt_conn *conn);

/** @brief Check DON notification CCC is enabled. Returns 0 or negative errno. */
int acs_don_ccc_check(struct bt_conn *conn);

/** @brief Check DOI indication CCC is enabled. Returns 0 or negative errno. */
int acs_doi_ccc_check(struct bt_conn *conn);

/** @brief Indicate the ACS Status characteristic to @p conn. */
void acs_status_indicate(struct bt_conn *conn);

/** @brief Return true if the ACS service has been initialised. */
bool acs_is_initialized(void);

/** @brief Get the server-wide security controls switch state. */
bool acs_security_switch_get(void);

/** @brief Set the server-wide security controls switch and indicate all connections. */
void acs_security_switch_set(bool enabled);

/** @brief Return the registered application callbacks, or NULL. */
const struct bt_acs_cb *acs_cb_get(void);

/** @brief Return the dedicated ACS workqueue. */
struct k_work_q *acs_get_wq(void);

/**
 * @brief Return per-connection ACS state at pool @p index.
 *
 * @return Pointer to the slot, or NULL if out of range.
 */
struct bt_acs_conn *acs_conn_by_index(uint8_t index);

/** @brief Look up per-connection ACS state. Returns NULL if not found. */
struct bt_acs_conn *acs_conn_lookup(struct bt_conn *conn);

/**
 * @brief Allocate and initialise per-connection ACS state.
 *
 * Indexes into a static array sized by CONFIG_BT_MAX_CONN using
 * bt_conn_index(), so it always returns a valid pointer for any
 * valid @p conn.
 */
struct bt_acs_conn *acs_conn_alloc(struct bt_conn *conn);

/** @brief Release all resources held by @p acs_conn. */
void acs_conn_cleanup(struct bt_acs_conn *acs_conn);

/** @brief Save crypto state to the RAM session cache for @p acs_conn's peer. */
void acs_session_cache_save(const struct bt_acs_conn *acs_conn);

/** @brief Restore crypto state from the RAM session cache for @p acs_conn's peer. */
int acs_session_cache_restore(struct bt_acs_conn *acs_conn);

/** @brief Clear the RAM session cache entry for @p addr. */
void acs_session_cache_clear_peer(const bt_addr_le_t *addr);

/** @brief Clear all RAM session cache entries except the one for @p keep_addr. */
void acs_session_cache_clear_all_except(const bt_addr_le_t *keep_addr);

#if defined(CONFIG_BT_SETTINGS)
/** @brief Persist the parent ACS key for @p acs_conn's peer to PSA persistent storage. */
void acs_session_store(struct bt_acs_conn const *acs_conn);

/** @brief Erase the stored parent key for @p conn's peer. */
void acs_session_clear(struct bt_conn const *conn);

/** @brief Erase stored parent keys for every peer except @p conn's peer. */
void acs_session_clear_all_except(struct bt_conn const *conn);

/** @brief Restore a previously stored parent key for @p acs_conn's peer from PSA storage. */
void acs_session_restore(struct bt_acs_conn *acs_conn);

/** @brief Register ACS's internal bond/auth-info callback. */
void acs_session_register_auth_info_cb(void);
#endif /* CONFIG_BT_SETTINGS */

#if IS_ENABLED(CONFIG_BT_ACS_GATT_AUTHORIZATION)
/** @brief Mark all protected CCCDs as requiring authorisation in the GATT table. */
void acs_policy_resolve_protected_cccds(void);
/** @brief Register ACS's internal GATT authorization callback. */
int acs_policy_register_gatt_auth_cb(void);
#endif /* CONFIG_BT_ACS_GATT_AUTHORIZATION */

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_INTERNAL_H_ */
