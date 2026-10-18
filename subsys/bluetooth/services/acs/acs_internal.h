/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_INTERNAL_H_
#define BT_GATT_ACS_INTERNAL_H_

/* Shared ACS state and helpers. */

#include "acs_wire_constants.h"
#include "acs_types.h"
#include "acs_util.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Allocate an ACS buffer from the shared pool, or return NULL when exhausted. */
struct net_buf *acs_buf_alloc(k_timeout_t timeout);

/* ACS Control Point value attribute. */
const struct bt_gatt_attr *acs_attr_cp(void);

/* Cached Resource Handle of the ACS Control Point (resolved at init). */
uint16_t acs_cp_resource_handle(void);

/* Return true when attr is one of the ACS service's own GATT attributes. */
bool acs_attr_is_own(const struct bt_gatt_attr *attr);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/* ACS Data Out Notify value attribute. */
const struct bt_gatt_attr *acs_attr_don(void);

/* ACS Data Out Indicate value attribute. */
const struct bt_gatt_attr *acs_attr_doi(void);

/* Check whether Data Out Notify notifications are enabled. */
int acs_don_ccc_check(struct bt_conn *conn);

/* Check whether Data Out Indicate indications are enabled. */
int acs_doi_ccc_check(struct bt_conn *conn);
#endif

/* Check whether ACS Control Point indications are enabled. */
int acs_cp_ccc_check(struct bt_conn *conn);

/* Mark Status pending and schedule its indication, unless the link is gone. */
void acs_status_schedule(struct bt_acs_conn *acs_conn);

/* Schedule a Status indication for every connected peer. */
void acs_status_schedule_all(void);

/* Initialize the per-connection Status indication work item. */
void acs_status_work_init(struct bt_acs_conn *acs_conn);

/* Return true after the ACS service has initialized successfully. */
bool acs_is_initialized(void);

/* Return the server-wide security-controls switch state. */
bool acs_security_switch_get(void);

/* Set the security-controls switch and indicate the new Status to every peer. */
void acs_security_switch_set(bool enabled);

/* Return the registered application callbacks, or NULL. */
const struct bt_acs_cb *acs_cb_get(void);

/* Return the dedicated ACS workqueue. */
struct k_work_q *acs_get_wq(void);

/* Per-connection state at pool index, which must be below CONFIG_BT_MAX_CONN. */
struct bt_acs_conn *acs_conn_by_index(uint8_t index);

/* Find the ACS state for conn, or return NULL when its slot is inactive. */
struct bt_acs_conn *acs_conn_lookup(struct bt_conn *conn);

/*
 * acs_conn_lookup() for code outside the Bluetooth RX thread: a found slot stays
 * active until acs_conn_release(), so work queued on it meanwhile is cancelled
 * by its teardown. Blocks the disconnect callback while held; do not wait.
 */
struct bt_acs_conn *acs_conn_hold(struct bt_conn *conn);

/* End an acs_conn_hold() that returned acs_conn. */
void acs_conn_release(struct bt_acs_conn *acs_conn);

/*
 * Clear Security Established and, when it was set, notify the application. The
 * keys are left in place for the caller to remove.
 */
void acs_security_end(struct bt_acs_conn *acs_conn);

/*
 * End all security on the connection: Security Established, a running key
 * exchange, every key and the peer's stored entry. Indicate Status unless the
 * link is gone.
 */
void acs_security_invalidate(struct bt_acs_conn *acs_conn);

/* Register the ACS GATT authorization callback. */
int acs_policy_register_gatt_auth_cb(void);

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_INTERNAL_H_ */
