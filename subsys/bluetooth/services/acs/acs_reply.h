/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_REPLY_H_
#define ACS_REPLY_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/net_buf.h>

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise per-connection reply state (pool flags, continuation work).
 *
 * Called once from acs_conn_alloc after memset.
 */
void acs_reply_init_conn(struct bt_acs_conn *conn);

/**
 * @brief Allocate a reply from the per-connection pool.
 *
 * @return Allocated reply with conn set, or NULL if no slot is free.
 */
struct acs_reply *acs_reply_alloc(struct bt_acs_conn *conn);

/**
 * @brief Free a reply back to the per-connection pool.
 *
 * Frees both request and response buffers if present.
 * Safe to call from any context (completion callbacks, work handlers).
 */
void acs_reply_free(struct acs_reply *reply);

/**
 * @brief Abort all in-flight replies on a connection.
 *
 * Cancels work items, flushes TX FIFOs, and marks any BLE-stack in-flight
 * replies with aborted=true for deferred cleanup by completion callbacks.
 */
void acs_reply_abort_all(struct bt_acs_conn *conn);

/**
 * @brief Execute the deferred abort: cancel work, tear down KEX, abort all
 * replies, and send ABORT SUCCESS on the plain CP channel.
 */
void acs_abort_commit(struct bt_acs_conn *conn);

/**
 * @brief Lazy-allocate or reset the response staging buffer for @p reply.
 *
 * Plain CP: returns the response buffer with no headroom.
 * Protected CP / characteristic: returns the buffer with ACS_CRYPTO_HEADROOM
 * reserved and the resource handle prefix pushed.
 *
 * @return Buffer to append response bytes into, or NULL on pool exhaustion.
 */
struct net_buf *acs_prepare_reply_buf(struct acs_reply *reply);

/**
 * @brief Submit the staged response for transmission.
 *
 * Routes to the correct TX path based on reply->channel:
 *   - ACS_REPLY_CP  — plain segmented CP indication
 *   - ACS_REPLY_DOI — encrypted indication
 *   - ACS_REPLY_DON — encrypted notification
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_reply_submit(struct acs_reply *reply);

/**
 * @brief Send the staged CP reply.
 *
 * Submits via acs_reply_submit using reply->channel. On submit failure,
 * forces reply->step to ACS_REPLY_DONE.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_cp_send_reply(struct acs_reply *reply);

/**
 * @brief Send a 3-byte Response Code indication.
 *
 * Stages a fresh buffer, fills [RESPONSE_CODE | req_opcode | code],
 * and submits. On failure releases cp_locked for plain CP replies.
 *
 * @return 0 on success, negative errno on failure.
 */
int acs_cp_rsp_status(struct acs_reply *reply, uint8_t req_opcode, uint8_t code);

/**
 * @brief Dispatch a reassembled CP payload to the opcode handler.
 *
 * Stages the response buffer, runs the handler, and transmits the reply.
 *
 * @param frame     Normalized inbound frame (payload[0] is the opcode).
 * @param acs_conn  Per-connection ACS state.
 * @param prot_req  NULL for plain CP, non-NULL for protected Data In path.
 */
int acs_cp_dispatch(const struct acs_frame *frame, struct bt_acs_conn *acs_conn,
		    struct acs_reply *prot_req);

/** @brief Initialise the per-connection protected request queue. */
void acs_request_queue_init(struct bt_acs_conn *acs_conn);

/** @brief Enqueue a reply for deferred protected request handling. */
void acs_request_queue_submit(struct bt_acs_conn *acs_conn, struct acs_reply *reply);

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
/** @brief Initialise the per-connection DOI drain queue state. */
void acs_doi_queue_init(struct bt_acs_conn *acs_conn);

/** @brief Schedule DOI drain work. */
void acs_doi_queue_submit(struct bt_acs_conn *acs_conn);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
/** @brief Initialise the per-connection DON drain queue state. */
void acs_don_queue_init(struct bt_acs_conn *acs_conn);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ACS_REPLY_H_ */
