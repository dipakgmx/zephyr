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

/* Destroy one PSA key, log any failure, and clear the caller's key ID. */
void acs_psa_destroy_key(psa_key_id_t *key_id);

/* Write a derived key to the selected exchange-key handle. */
int acs_crypto_output_exchange_key(struct bt_acs_key_desc_runtime *key_runtime,
				   psa_key_derivation_operation_t *op, size_t key_len,
				   bool derive_twin);

/* Generate the AC Server fixed nonce when absent, in runtime MSO order. */
int acs_server_nonce_fixed_ensure(struct bt_acs_key_desc_runtime *runtime);

/* Copy both exchange-key handles in the keystore to persistent IDs. */
int acs_crypto_copy_key_to_persistent(const struct bt_acs_key_desc_runtime *parent,
				      psa_key_id_t dst_id, psa_key_id_t dst_derive_id);

/* Copy both persistent handles in the keystore to a volatile runtime slot. */
int acs_crypto_copy_persistent_key_to_runtime(psa_key_id_t src_id, psa_key_id_t src_derive_id,
					      struct bt_acs_key_desc_runtime *parent);

/*
 * Reset a freshly bound record's counters and generate its server nonce
 * prefix. A record without a prefix (CMAC) has none to generate.
 */
int acs_nonce_state_init(struct bt_acs_key_desc_runtime *runtime);

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
/* Select the server-wide map. Return -ENOENT if map_id is not registered. */
int acs_rmap_activate(uint16_t map_id);

/* The AC Server's current restriction map, or NULL before init. */
const struct bt_acs_rmap_runtime *acs_rmap_active(void);

/* Restriction Map ID of the current map, or 0 if none. */
uint16_t acs_rmap_active_id(void);
#else
static inline const struct bt_acs_rmap_runtime *acs_rmap_active(void)
{
	return NULL;
}

static inline uint16_t acs_rmap_active_id(void)
{
	return BT_ACS_RMAP_ID_NONE;
}
#endif

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/* Persist the parent ACS key for acs_conn's peer. */
void acs_key_store(struct bt_acs_conn *acs_conn);

/* Remove the stored parent key for conn's peer. */
void acs_key_store_clear(struct bt_conn const *conn);

/* Remove stored parent keys for every peer except conn's peer. */
void acs_key_store_clear_all_except(struct bt_conn const *conn);

/* Restore the stored parent key for acs_conn's peer. */
void acs_key_restore(struct bt_acs_conn *acs_conn);

/* Register the ACS bond/auth-info callback. */
void acs_key_store_init(void);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
/* Register the ACS GATT authorization callback. */
int acs_policy_register_gatt_auth_cb(void);
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_INTERNAL_H_ */
