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

/* Release a shared ACS buffer. A NULL buffer is accepted. */
void acs_buf_free(struct net_buf *buf);

/* ACS Control Point value attribute. */
const struct bt_gatt_attr *acs_attr_cp(void);

/* Cached GATT Attribute Handle of the ACS Control Point (resolved at init). */
uint16_t acs_cp_attr_handle(void);

/* Cached Resource Handle of the ACS Control Point (resolved at init). */
uint16_t acs_cp_resource_handle(void);

/* Return true when handle belongs to the ACS service's GATT attribute range. */
bool acs_handle_is_own(uint16_t handle);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/* ACS Data Out Notify value attribute. */
const struct bt_gatt_attr *acs_attr_don(void);

/* ACS Data Out Indicate value attribute. */
const struct bt_gatt_attr *acs_attr_doi(void);
#endif

/* Check whether ACS Control Point indications are enabled. */
int acs_cp_ccc_check(struct bt_conn *conn);

/* Check whether Data Out Notify notifications are enabled. */
int acs_don_ccc_check(struct bt_conn *conn);

/* Check whether Data Out Indicate indications are enabled. */
int acs_doi_ccc_check(struct bt_conn *conn);

/* Mark Status pending and schedule its indication for one connection. */
void acs_status_schedule(struct bt_conn *conn);

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

/* Cache the Get Feature response after registration-time metadata is resolved. */
void acs_cp_feature_init(void);

/* Per-connection state at pool index, which must be below CONFIG_BT_MAX_CONN. */
struct bt_acs_conn *acs_conn_by_index(uint8_t index);

/* Find the ACS state for conn, or return NULL when its slot is inactive. */
struct bt_acs_conn *acs_conn_lookup(struct bt_conn *conn);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
/* Register the ACS GATT authorization callback. */
int acs_policy_register_gatt_auth_cb(void);
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_INTERNAL_H_ */
