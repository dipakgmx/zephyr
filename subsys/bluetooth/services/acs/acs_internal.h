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
 * @param frame     Normalized inbound frame. @c frame->payload[0] is the opcode.
 *                  For the protected CP path, @p prot_req must be non-NULL and
 *                  carries the live request context allocated by the runtime.
 * @param acs_conn  Per-connection ACS state.
 * @param prot_req  NULL for plain CP, non-NULL for protected Data In path.
 */
void acs_cp_dispatch(struct acs_frame *frame, struct bt_acs_conn *acs_conn,
		     struct bt_acs_prot_resource_req *prot_req);

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
 * @brief Send a 3-byte Response Code indication on the active CP channel.
 *
 * Stages a fresh response buffer for @p owner via @ref acs_cp_prepare_reply_buf,
 * fills [BT_ACS_CP_OPCODE_RESPONSE_CODE | req_opcode | code], and submits via
 * @ref acs_tx_submit. Aborts any active reply sequence on prep failure.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_cp_rsp_status(const struct acs_exec_owner *owner, uint8_t req_opcode, uint8_t code);

/** @brief Return true if a multi-step reply sequence is active on @p owner. */
bool acs_seq_active(const struct acs_exec_owner *owner);

/**
 * @brief Start a multi-step reply sequence on @p owner.
 *
 * @param owner Active execution owner.
 * @param desc  Sequence descriptor (step table + metadata).
 */
void acs_seq_begin(const struct acs_exec_owner *owner, const struct acs_seq_desc *desc);

/** @brief Mark the active reply sequence as complete and release state. */
void acs_seq_clear(const struct acs_exec_owner *owner);

/** @brief Abort the active reply sequence, invoking on_abort if set. */
void acs_seq_abort(const struct acs_exec_owner *owner);

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

/**
 * @name Data In pipeline error codes (returned by @ref acs_data_in_unwrap_and_route).
 *
 * Mapped by the runtime entry layer to ATT error codes.
 * @{
 */
#define ACS_DATA_ERR_CCC_IMPROPER_CONF         (-EPIPE)
#define ACS_DATA_ERR_NOT_AUTHORIZED            (-EPERM)
#define ACS_DATA_ERR_INVALID_KEY               (-EACCES)
#define ACS_DATA_ERR_RESOURCE_NOT_PROTECTED    (-ENOENT)
#define ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG (-EPROTO)
/** @} */

/**
 * @brief Unwrap a complete reassembled Data In payload and route it.
 *
 * Validates the ISC_ID/nonce header, decrypts the payload in place, builds a
 * normalized @ref acs_frame, then hands it to @ref acs_runtime_dispatch_frame.
 *
 * @return 0 on success, or one of @ref ACS_DATA_ERR_* on failure.
 */
int acs_data_in_unwrap_and_route(struct bt_conn *conn, struct bt_acs_conn *acs_conn,
				 struct net_buf_simple *buf);

/**
 * @brief Common normalized-frame dispatch entry point.
 *
 * Single seam between the GATT-write entrypoints and the per-route execution
 * helpers. Classifies @p frame against @p acs_conn's active restriction map
 * (or trivially as @ref ACS_ROUTE_ACS_CP for CP-source frames) and forwards
 * to the matching @c acs_runtime_dispatch_*_frame helper.
 *
 * @return 0 on success, negative errno or @ref ACS_DATA_ERR_* on failure.
 */
int acs_runtime_dispatch_frame(struct acs_frame *frame, struct bt_acs_conn *acs_conn);

/**
 * @brief Dispatch a frame whose route kind is @ref ACS_ROUTE_ACS_CP.
 *
 * Forwards directly to @ref acs_cp_dispatch with @p prot_req. For plain CP
 * frames @p prot_req is NULL; for protected CP frames @p prot_req is the
 * already-allocated request context owning the decrypted input buffer.
 */
int acs_runtime_dispatch_cp_frame(struct acs_frame *frame, struct bt_acs_conn *acs_conn,
				  struct bt_acs_prot_resource_req *prot_req);

/**
 * @brief Dispatch a Data-In frame that targets a protected service CP.
 *
 * Performs the DOI CCC check, allocates a request context, transfers ownership
 * of @c acs_conn->data_rx.buf into the context, then forwards to
 * @ref acs_runtime_dispatch_cp_frame. Drops the owner reference here unless a
 * multi-step reply sequence took it over.
 */
int acs_runtime_dispatch_protected_cp_frame(struct acs_frame *frame, struct bt_acs_conn *acs_conn);

/**
 * @brief Dispatch a Data-In frame that targets a protected characteristic.
 *
 * Resolves the required Data Out subscription, allocates a request context,
 * transfers ownership of @c acs_conn->data_rx.buf into the context, then
 * queues the work item that runs the application/auto-respond handler.
 */
int acs_runtime_dispatch_protected_char_frame(struct acs_frame *frame,
					      struct bt_acs_conn *acs_conn);

/**
 * @brief Resolve the Data Out subscription for a protected resource handle.
 *
 * Picks DON or DOI based on the characteristic's properties; for zero-length
 * payloads only DON is checked. Returns 0 if the required CCC is configured,
 * @c -EINVAL when the CCC is missing, or other negative errno on lookup failure.
 */
int acs_require_data_out_subscription(struct bt_conn *conn, uint16_t resource_handle,
				      uint16_t data_length);

/**
 * @brief Single-seam outbound submission for any reply.
 *
 * Decides the transport family from @p reply->channel:
 *   - @ref ACS_REPLY_CP  — plain segmented CP indication on @c acs_conn->cp_tx,
 *     completion via @ref acs_cp_on_indicate_done. @p owner must be the plain-CP
 *     singleton.
 *   - @ref ACS_REPLY_DOI — encrypts in place, queues on @c indicate_fifo with
 *     @ref try_send_next, completion via @ref acs_data_out_on_indicate_done.
 *     @p owner must be a protected request.
 *   - @ref ACS_REPLY_DON — encrypts in place, sends an unconfirmed segmented
 *     notification. @p owner must be a protected request.
 *
 * Domain code should produce a buffer + reply and call this. The send seam
 * picks segmentation, queueing, encryption, and the completion callback.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_tx_submit(const struct acs_exec_owner *owner, const struct acs_reply *reply);

/**
 * @brief Lazy-allocate or reset the response staging buffer for @p owner.
 *
 * Plain CP: returns the singleton @c plain_cp_proc.response, no headroom,
 * no payload prefix.
 *
 * Protected CP / characteristic: returns @c req->response with
 * @ref ACS_CRYPTO_HEADROOM reserved and the protected-resource handle prefix
 * already pushed, so the handler can append its body bytes directly.
 *
 * @param owner     Active owner.
 * @param channel   Reply channel — selects whether the resource_handle prefix
 *                  is seeded (DON / DOI for protected paths).
 * @param encrypted True when the reply will go through AEAD encryption (drives
 *                  the headroom requirement).
 * @return Buffer to append response bytes into, or NULL on pool exhaustion.
 */
struct net_buf *acs_prepare_reply_buf(const struct acs_exec_owner *owner,
				      enum acs_reply_channel channel, bool encrypted);

/**
 * @brief Convenience wrapper that picks the CP-canonical reply mode for @p owner.
 *
 * Plain CP    → @ref ACS_REPLY_CP, encrypted=false.
 * Protected CP → @ref ACS_REPLY_DOI, encrypted=true.
 *
 * Use in CP handlers that always reply on the canonical channel for their
 * dispatch path; for explicit overrides (e.g. DON), call
 * @ref acs_prepare_reply_buf directly.
 */
struct net_buf *acs_cp_prepare_reply_buf(const struct acs_exec_owner *owner);

/**
 * @brief Build a logical CP reply from an owner.
 *
 * Resolves the active procedure via the owner, validates its staged response
 * buffer, and populates @p reply with channel/encrypted/needs_confirm
 * matching the owner kind:
 *   - plain CP   → ACS_REPLY_CP, encrypted=false
 *   - protected  → ACS_REPLY_DOI, encrypted=true
 * (@p needs_confirm is true for both — these are confirmed indications.)
 */
void acs_cp_build_reply(const struct acs_exec_owner *owner, struct acs_reply *reply);

/**
 * @brief Send a CP-channel reply assembled from @p owner.
 *
 * Convenience over @ref acs_cp_build_reply + @ref acs_tx_submit. Aborts any
 * active reply sequence on submit failure and logs the failure.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_cp_send_reply(const struct acs_exec_owner *owner);

/**
 * @brief Owner-centric reply-sequence advance.
 *
 * Common implementation behind @ref acs_seq_on_cp_confirm and
 * @ref acs_seq_on_req_confirm. Builds an @c acs_cp_step_ctx from the owner and
 * drives the next step of any active reply sequence; aborts the sequence on
 * step failure. No-op if no sequence is active.
 */
void acs_seq_on_owner_confirm(const struct acs_exec_owner *owner, struct bt_conn *conn,
			      const struct bt_gatt_attr *attr);

/**
 * @brief Plain-CP indication-confirmation callback (defined in acs_cp.c).
 *
 * Exposed so the data-out channel layer can pass it as the completion handler
 * for plain-CP segmented indications.
 */
void acs_cp_on_indicate_done(struct bt_conn *conn, const struct bt_gatt_attr *attr, int err,
			     void *user_data);

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
 * @brief Encrypt and send @p plaintext via Data Out Notify on @p req's connection.
 *
 * @p plaintext must be the request's currently-staged response buffer
 * (@c req->response). Passing it explicitly keeps the buffer authoritatively
 * sourced from the @ref acs_reply contract rather than via an implicit
 * convention; the helper asserts the consistency in debug builds.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_prot_resource_rsp_notify(struct bt_acs_prot_resource_req *req, struct net_buf *plaintext);

/**
 * @brief Encrypt and queue @p plaintext for Data Out Indicate on @p req's connection.
 *
 * Same contract as @ref acs_prot_resource_rsp_notify regarding @p plaintext.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_prot_resource_rsp_indicate(struct bt_acs_prot_resource_req *req, struct net_buf *plaintext);

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
