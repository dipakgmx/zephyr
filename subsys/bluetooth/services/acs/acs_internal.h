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
 *   acs_wire_constants.h  — on-the-wire field sizes / opcodes
 *   acs_types.h           — core structs (acs_frame, acs_route, acs_procedure,
 *                           bt_acs_conn) and enumerations shared across layers
 *   acs_util.h            — small inline helpers
 *
 *   acs_runtime.h         — GATT-write entrypoints + frame dispatch + Data In
 *                           unwrap. This is where every inbound PDU enters.
 *   acs_procedure.h       — procedure (request) lifecycle, multi-step reply
 *                           sequences, reply staging + send (acs_tx_submit),
 *                           CP opcode dispatch.
 *   acs_crypto.h          — session crypto + key exchange API.
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
#include "acs_procedure.h"
#include "acs_crypto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Buffer pool -------------------------------------------------------- */

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

/* ---- GATT attribute accessors + CCC checks ------------------------------ */

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

/* ---- Service init + callbacks ------------------------------------------ */

/** @brief Return true if the ACS service has been initialised. */
bool acs_is_initialized(void);

/** @brief Return the registered application callbacks, or NULL. */
const struct bt_acs_cb *acs_cb_get(void);

/* ---- Connection state lookup -------------------------------------------- */

/**
 * @brief Return per-connection ACS state at pool @p index.
 *
 * @return Pointer to the slot, or NULL if out of range.
 */
struct bt_acs_conn *acs_conn_by_index(uint8_t index);

/** @brief Look up per-connection ACS state. Returns NULL if not found. */
struct bt_acs_conn *acs_conn_lookup(struct bt_conn *conn);

/** @brief Allocate and initialise per-connection ACS state. Returns NULL on limit. */
struct bt_acs_conn *acs_conn_alloc(struct bt_conn *conn);

/** @brief Release all resources held by @p acs_conn. */
void acs_conn_cleanup(struct bt_acs_conn *acs_conn);

/* ---- Session persistence (BT_SETTINGS) ---------------------------------- */

#if defined(CONFIG_BT_SETTINGS)
/** @brief Persist the crypto session for @p conn to NVS. */
void acs_session_store(struct bt_conn const *conn, struct bt_acs_conn const *acs_conn);

/** @brief Erase all stored sessions for @p conn's peer address. */
void acs_session_clear_all(struct bt_conn const *conn);

/** @brief Return true if the session cache has room for @p addr. */
bool acs_session_cache_has_room(const bt_addr_le_t *addr);

/** @brief Restore a previously stored session for @p conn from NVS. */
void acs_session_restore(struct bt_conn *conn, struct bt_acs_conn *acs_conn);

/** @brief Invalidate any cached session state for @p addr. */
void acs_session_invalidate_cache(const bt_addr_le_t *addr);
extern struct bt_conn_auth_info_cb acs_auth_info_cb;
#endif /* CONFIG_BT_SETTINGS */

/* ---- GATT authorization policy ----------------------------------------- */

#if IS_ENABLED(CONFIG_BT_ACS_GATT_AUTHORIZATION)
/** @brief Mark all protected CCCDs as requiring authorisation in the GATT table. */
void acs_policy_resolve_protected_cccds(void);
extern const struct bt_gatt_authorization_cb acs_gatt_auth_cb;
#endif /* CONFIG_BT_ACS_GATT_AUTHORIZATION */

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_INTERNAL_H_ */
