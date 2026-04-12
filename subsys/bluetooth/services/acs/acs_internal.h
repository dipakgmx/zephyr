/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_INTERNAL_H_
#define BT_GATT_ACS_INTERNAL_H_

/* Sub-headers: constants → types → utilities (order matters for dependencies) */
#include "acs_wire_constants.h"
#include "acs_types.h"
#include "acs_util.h"

#ifdef __cplusplus
extern "C" {
#endif

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

/**
 * @brief Dispatch a reassembled CP payload to the opcode handler.
 *
 * @param prot_req  NULL for plain CP, non-NULL for protected Data In path.
 * @param acs_conn  Per-connection ACS state.
 * @param payload   Reassembled buffer; payload[0] = opcode.
 */
void acs_cp_dispatch(struct bt_acs_prot_resource_req *prot_req, struct bt_acs_conn *acs_conn,
		     struct net_buf_simple *payload);

/**
 * @brief GATT write handler for the ACS Control Point characteristic.
 *
 * Reassembles segmented writes and dispatches to @ref acs_cp_dispatch.
 */
ssize_t acs_cp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
		     uint16_t len, uint16_t offset, uint8_t flags);

/**
 * @brief Lazy-initialise and copy the server fixed nonce for @p acs_conn.
 *
 * Must be called before START_KEY_EXCHANGE for SEQ_DIFF_FIXED nonce types
 * (spec §4.4.3.13).
 *
 * @param acs_conn  Per-connection ACS state.
 * @param nonce_buf Caller buffer to receive the nonce.
 * @param len       Size of @p nonce_buf.
 * @return 0 on success, -ENOTSUP if fixed nonce is not configured.
 */
int acs_crypto_get_server_nonce_fixed(struct bt_acs_conn *acs_conn, uint8_t *nonce_buf, size_t len);

/**
 * @brief Allocate or reset the CP response buffer for @p ctx.
 *
 * Plain CP: lazy-allocs cp_proc.response.
 * Protected CP: lazy-allocs prot_req->response with crypto headroom.
 *
 * @param ctx  CP dispatch context.
 * @return Response buffer, or NULL on pool exhaustion.
 */
struct net_buf *acs_cp_rsp_alloc(struct acs_cp_ctx *ctx);

/**
 * @brief Send the CP response in the current buffer.
 *
 * Plain CP: arms segmented indication on cp_tx.
 * Protected CP: encrypts and queues via DOI.
 *
 * @param ctx  CP dispatch context.
 * @return 0 on success, negative errno on failure.
 */
int acs_cp_rsp_send(struct acs_cp_ctx *ctx);

/**
 * @brief Send a 3-byte Response Code indication.
 *
 * @param ctx        CP dispatch context.
 * @param req_opcode Opcode being responded to.
 * @param code       ACS CP response code value.
 * @return 0 on success, negative errno on failure.
 */
int acs_cp_rsp_status(struct acs_cp_ctx *ctx, uint8_t req_opcode, uint8_t code);

/** @brief Return true if a multi-step reply sequence is active on @p ctx. */
bool acs_seq_active(const struct acs_cp_ctx *ctx);

/**
 * @brief Start a multi-step reply sequence on @p ctx.
 *
 * @param ctx   CP dispatch context.
 * @param desc  Sequence descriptor (step table + metadata).
 */
void acs_seq_begin(struct acs_cp_ctx *ctx, const struct acs_seq_desc *desc);

/** @brief Mark the active reply sequence as complete and release state. */
void acs_seq_clear(struct acs_cp_ctx *ctx);

/** @brief Abort the active reply sequence, invoking on_abort if set. */
void acs_seq_abort(struct acs_cp_ctx *ctx);

/** @brief Resume a plain CP reply sequence after the peer confirms an indication. */
void acs_seq_on_cp_confirm(struct bt_conn *conn, const struct bt_gatt_attr *attr);

/** @brief Resume a protected CP reply sequence after the peer confirms an indication. */
void acs_seq_on_req_confirm(struct bt_acs_prot_resource_req *req, struct bt_conn *conn,
			    const struct bt_gatt_attr *attr);

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

/** @brief Return the registered application callbacks, or NULL. */
const struct bt_acs_cb *acs_cb_get(void);

/**
 * @brief Return per-connection ACS state at pool @p index.
 *
 * @return Pointer to the slot, or NULL if out of range.
 */
struct bt_acs_conn *acs_conn_by_index(uint8_t index);

/** @brief GATT write handler for the ACS Data In characteristic. */
ssize_t acs_data_in_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags);

/** @brief Increment @p req reference count for caller @p who. */
void acs_prot_resource_req_ref(struct bt_acs_prot_resource_req *req,
			       enum acs_prot_resource_ref_who who);

/** @brief Decrement @p req reference count for caller @p who; frees at zero. */
void acs_prot_resource_req_unref(struct bt_acs_prot_resource_req *req,
				 enum acs_prot_resource_ref_who who);

/** @brief Return the connection for @p req, or NULL after disconnect. */
struct bt_conn *acs_prot_resource_req_conn(const struct bt_acs_prot_resource_req *req);

/** @brief Return the protected resource ATT handle from @p req. */
uint16_t acs_prot_resource_req_handle(const struct bt_acs_prot_resource_req *req);

/**
 * @brief Allocate a protected resource request from the per-connection pool.
 *
 * @param acs_conn        Owning connection state.
 * @param resource_handle Protected resource ATT handle.
 * @param isc_id          ISC identifier for this request.
 * @param data_offset     Offset into the decrypted buffer where payload starts.
 * @param data_length     Plaintext payload length.
 * @return Allocated request, or NULL if no slot is free.
 */
struct bt_acs_prot_resource_req *acs_prot_resource_req_alloc(struct bt_acs_conn *acs_conn,
							     uint16_t resource_handle,
							     uint16_t isc_id, uint16_t data_offset,
							     uint16_t data_length);

/** @brief Drop the owner (allocation) reference on @p req. */
void acs_prot_resource_req_release_owner(struct bt_acs_prot_resource_req *req);

/** @brief Drop the TX-path reference on @p req. */
void acs_prot_resource_req_release_tx(struct bt_acs_prot_resource_req *req);

/** @brief Mark response transmission complete and release the TX reference. */
void acs_prot_resource_req_tx_done(struct bt_acs_prot_resource_req *req);

/** @brief Abort all in-flight protected resource requests on @p acs_conn. */
void acs_prot_resource_req_abort_all(struct bt_acs_conn *acs_conn);

/**
 * @brief Encrypt and send @p req response via Data Out Notify.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_prot_resource_rsp_notify(struct bt_acs_prot_resource_req *req);

/**
 * @brief Encrypt and queue @p req response for Data Out Indicate.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_prot_resource_rsp_indicate(struct bt_acs_prot_resource_req *req);

/**
 * @brief Derive the session key from the completed key exchange.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_derive_session_key(struct bt_acs_conn *acs_conn);

/**
 * @brief Import the session key into the PSA keystore.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_import_session_key(struct bt_acs_conn *acs_conn);

/** @brief Destroy the session key from the PSA keystore. */
void acs_crypto_destroy_session_key(struct bt_acs_conn *acs_conn);

/**
 * @brief Encrypt @p plaintext using the session key.
 *
 * @param acs_conn   Per-connection state (holds key and nonce counters).
 * @param isc_id     ISC identifier for nonce construction.
 * @param plaintext  Input data.
 * @param plain_len  Input length.
 * @param ciphertext Output buffer (must fit ciphertext + auth tag).
 * @param cipher_len [out] Total output length.
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_encrypt(struct bt_acs_conn *acs_conn, uint16_t isc_id, const uint8_t *plaintext,
		       uint16_t plain_len, uint8_t *ciphertext, uint16_t *cipher_len);

/**
 * @brief Decrypt @p ciphertext using the session key.
 *
 * @param acs_conn   Per-connection state.
 * @param isc_id     ISC identifier for nonce construction.
 * @param ciphertext Input (ciphertext + auth tag).
 * @param cipher_len Total input length.
 * @param plaintext  Output buffer.
 * @param plain_len  [out] Plaintext length.
 * @param aad        Additional authenticated data (may be NULL).
 * @param aad_len    Length of @p aad.
 * @return 0 on success, negative errno on failure.
 */
int acs_crypto_decrypt(struct bt_acs_conn *acs_conn, uint16_t isc_id, const uint8_t *ciphertext,
		       uint16_t cipher_len, uint8_t *plaintext, uint16_t *plain_len,
		       const uint8_t *aad, uint16_t aad_len);

/** @brief Allocate a transient key-exchange context. Returns NULL if pool exhausted. */
struct bt_acs_kex_ctx *acs_kex_alloc(void);

/** @brief Return a key-exchange context to the pool. */
void acs_kex_free(struct bt_acs_kex_ctx *kex);

/** @brief Look up per-connection ACS state. Returns NULL if not found. */
struct bt_acs_conn *acs_conn_lookup(struct bt_conn *conn);

/** @brief Allocate and initialise per-connection ACS state. Returns NULL on limit. */
struct bt_acs_conn *acs_conn_alloc(struct bt_conn *conn);

/** @brief Release all resources held by @p acs_conn. */
void acs_conn_cleanup(struct bt_acs_conn *acs_conn);

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

#if IS_ENABLED(CONFIG_BT_ACS_GATT_AUTHORIZATION)
/** @brief Mark all protected CCCDs as requiring authorisation in the GATT table. */
void acs_policy_resolve_protected_cccds(void);
extern const struct bt_gatt_authorization_cb acs_gatt_auth_cb;
#endif /* CONFIG_BT_ACS_GATT_AUTHORIZATION */

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_INTERNAL_H_ */
