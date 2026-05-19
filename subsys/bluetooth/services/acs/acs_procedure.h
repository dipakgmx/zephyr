/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_PROCEDURE_H_
#define ACS_PROCEDURE_H_

/*
 * Procedure (request) lifecycle and reply submission layer.
 *
 * A "procedure" is one in-flight ACS operation:
 *   - the plain-CP singleton  (ACS_PROC_KIND_PLAIN_CP)
 *   - a slab-allocated protected-resource request (ACS_PROC_KIND_PROTECTED_REQ)
 *
 * This header collects:
 *   - allocation / refcounting / lookup helpers
 *   - the multi-step reply sequence API
 *   - reply staging and submission (acs_prepare_reply_buf, acs_tx_submit,
 *     acs_cp_send_reply, acs_cp_rsp_status)
 *   - the CP opcode dispatcher (acs_cp_dispatch) and its plain-CP indication
 *     completion callback (acs_cp_completion_cb)
 *
 * Crypto-side helpers live in acs_crypto.h; transport entry points live in
 * acs_runtime.h.
 */

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/net_buf.h>

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Procedure lifecycle ------------------------------------------------- */

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
struct acs_procedure *acs_procedure_alloc(struct bt_acs_conn *acs_conn, uint16_t resource_handle,
					  uint16_t isc_id, uint16_t data_offset,
					  uint16_t data_length);

/** @brief Increment @p req reference count for caller @p who. */
void acs_procedure_ref(struct acs_procedure *req, enum acs_procedure_ref_who who);

/** @brief Decrement @p req reference count for caller @p who; frees at zero. */
void acs_procedure_unref(struct acs_procedure *req, enum acs_procedure_ref_who who);

/** @brief Drop the proc (allocation) reference on @p req. */
void acs_procedure_release_owner(struct acs_procedure *req);

/** @brief Drop the TX-path reference on @p req. */
void acs_procedure_release_tx(struct acs_procedure *req);

/** @brief Abort all in-flight protected resource requests on @p acs_conn. */
void acs_procedure_abort_all(struct bt_acs_conn *acs_conn);

/** @brief Return the connection for @p req, or NULL after disconnect. */
struct bt_conn *acs_procedure_conn(const struct acs_procedure *req);

/** @brief Return the protected resource ATT handle from @p req. */
uint16_t acs_procedure_resource_handle(const struct acs_procedure *req);

/* ---- Reply sequence (multi-step replies) -------------------------------- */

/** @brief Return true if a multi-step reply sequence is active on @p proc. */
bool acs_seq_active(struct acs_procedure *proc);

/**
 * @brief Start a multi-step reply sequence on @p proc.
 *
 * @param proc Active execution proc.
 * @param desc  Sequence descriptor (step table + metadata).
 */
void acs_seq_begin(struct acs_procedure *proc, const struct acs_seq_desc *desc);

/** @brief Mark the active reply sequence as complete and release state. */
void acs_seq_clear(struct acs_procedure *proc);

/** @brief Abort the active reply sequence, invoking on_abort if set. */
void acs_seq_abort(struct acs_procedure *proc);

/**
 * @brief Owner-centric reply-sequence advance.
 *
 * Called from indication-confirm callbacks (plain CP and DOI). Drives the next
 * step of any active reply sequence; aborts the sequence on step failure.
 * No-op if no sequence is active.
 */
void acs_seq_on_confirm(struct acs_procedure *proc);

/**
 * @brief Lazy-allocate or reset the response staging buffer for @p proc.
 *
 * Plain CP: returns the singleton @c plain_cp_proc.buffers.response_buf, no headroom,
 * no payload prefix.
 *
 * Protected CP / characteristic: returns @c req->buffers.response_buf with
 * @ref ACS_CRYPTO_HEADROOM reserved and the protected-resource handle prefix
 * already pushed, so the handler can append its body bytes directly.
 *
 * @param proc     Active proc.
 * @param encrypted True when the reply will go through AEAD encryption (drives
 *                  the headroom requirement).
 * @return Buffer to append response bytes into, or NULL on pool exhaustion.
 */
struct net_buf *acs_prepare_reply_buf(struct acs_procedure *proc, bool encrypted);

/**
 * @brief Single-seam outbound submission for any reply.
 *
 * Decides the transport family from @p reply->channel:
 *   - @ref ACS_REPLY_CP  — plain segmented CP indication on @c acs_conn->cp_tx,
 *     completion via @ref acs_cp_completion_cb. @p proc must be the plain-CP
 *     singleton.
 *   - @ref ACS_REPLY_DOI — encrypts in place, queues on @c indicate_fifo and
 *     drains it; completion via @c data_tx_completion_cb. @p proc must
 *     be a protected request.
 *   - @ref ACS_REPLY_DON — encrypts in place, sends an unconfirmed segmented
 *     notification. @p proc must be a protected request.
 *
 * Buffer ownership:
 *   - On success, @c reply->plaintext (== @c proc->buffers.response_buf) is borrowed by the
 *     TX layer for the lifetime of the send. The caller must not free it.
 *   - On failure, the caller retains ownership of @c proc->buffers.response_buf. Plain CP
 *     frees it internally on send failure; protected paths leave it on the proc
 *     for a subsequent step or teardown to release.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_tx_submit(struct acs_procedure *proc, const struct acs_reply *reply);

/**
 * @brief Send the staged CP reply assembled from @p proc.
 *
 * Reads the active procedure's staged response buffer, builds an @ref acs_reply
 * using @ref acs_proc_reply_mode for the canonical channel/encrypted/needs_confirm
 * defaults, and submits via @ref acs_tx_submit. Aborts any active reply sequence
 * on submit failure and logs the failure.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_cp_send_reply(struct acs_procedure *proc);

/**
 * @brief Send a 3-byte Response Code indication on the active CP channel.
 *
 * Stages a fresh response buffer for @p proc via @ref acs_prepare_reply_buf
 * with the mode from @ref acs_proc_reply_mode, fills
 * [BT_ACS_CP_OPCODE_RESPONSE_CODE | req_opcode | code], and submits via
 * @ref acs_tx_submit. Mirrors @ref acs_cp_send_reply on submit failure
 * (acs_seq_abort + log); on prep failure additionally releases the plain-CP
 * busy gate.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_cp_rsp_status(struct acs_procedure *proc, uint8_t req_opcode, uint8_t code);

/* ---- CP opcode dispatch ------------------------------------------------- */

/**
 * @brief Dispatch a reassembled CP payload to the opcode handler.
 *
 * @param frame     Normalized inbound frame. @c frame->payload[0] is the opcode.
 *                  For the protected CP path, @p prot_req must be non-NULL and
 *                  carries the live request context allocated by the runtime.
 * @param acs_conn  Per-connection ACS state.
 * @param prot_req  NULL for plain CP, non-NULL for protected Data In path.
 */
int acs_cp_dispatch(struct acs_frame *frame, struct bt_acs_conn *acs_conn,
		    struct acs_procedure *prot_req);

/**
 * @brief Plain-CP indication-confirmation callback (defined in acs_cp.c).
 *
 * Exposed so the data-out channel layer can pass it as the completion handler
 * for plain-CP segmented indications.
 */
void acs_cp_completion_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, int err,
			  void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ACS_PROCEDURE_H_ */
