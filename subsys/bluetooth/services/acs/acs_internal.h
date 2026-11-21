/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
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

/* Allocate a buffer from the shared ACS pool, or return NULL when none frees within timeout. */
struct net_buf *acs_buf_alloc(k_timeout_t timeout);

/* Return the ACS Control Point value attribute, cached by bt_acs_init(). */
const struct bt_gatt_attr *acs_attr_cp(void);

/* Return the Resource Handle of the ACS Control Point, cached by bt_acs_init(). */
uint16_t acs_cp_resource_handle(void);

/* Return true when attr belongs to the ACS service itself. */
bool acs_attr_is_own(const struct bt_gatt_attr *attr);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/* Return the ACS Data Out Notify value attribute, cached by bt_acs_init(). */
const struct bt_gatt_attr *acs_attr_don(void);

/* Return the ACS Data Out Indicate value attribute, cached by bt_acs_init(). */
const struct bt_gatt_attr *acs_attr_doi(void);

/* Return 0 when conn has enabled ACS Data Out Notify notifications, else -EINVAL. */
int acs_don_ccc_check(struct bt_conn *conn);

/* Return 0 when conn has enabled ACS Data Out Indicate indications, else -EINVAL. */
int acs_doi_ccc_check(struct bt_conn *conn);
#endif

/* Return 0 when conn has enabled ACS Control Point indications, else -EINVAL. */
int acs_cp_ccc_check(struct bt_conn *conn);

/* Schedule a Status indication to acs_conn, unless its link is gone. */
void acs_status_schedule(struct bt_acs_conn *acs_conn);

/* Schedule a Status indication to every connection with ACS state. */
void acs_status_schedule_all(void);

/* Initialize the Status indication work of acs_conn. */
void acs_status_work_init(struct bt_acs_conn *acs_conn);

/* Return true once bt_acs_init() has succeeded. */
bool acs_is_initialized(void);

/* Return true when the security controls switch is on. */
bool acs_security_switch_get(void);

/* Set the security controls switch; a change schedules a Status indication to every peer. */
void acs_security_switch_set(bool enabled);

/* Return the callbacks given to bt_acs_init(), or NULL. */
const struct bt_acs_cb *acs_cb_get(void);

/* Return the ACS workqueue. */
struct k_work_q *acs_get_wq(void);

/* Return the slot at index, active or not; index must be below CONFIG_BT_MAX_CONN. */
struct bt_acs_conn *acs_conn_by_index(uint8_t index);

/*
 * Return the ACS state of conn, or NULL when its slot is inactive, held by another
 * connection or marked disconnected.
 */
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

/* Register the ACS GATT authorization callbacks for reads and writes. */
int acs_policy_register_gatt_auth_cb(void);

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_INTERNAL_H_ */
