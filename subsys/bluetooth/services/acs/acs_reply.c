/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_internal.h"
#include "acs_reply.h"
#include "acs_cp.h"
#include "acs_key_exchange.h"
#include "acs_cp_handlers.h"
#include "acs_descs.h"
#include "acs_data_out.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static void acs_abort_work_handler(struct k_work *work);
static void acs_reply_abort_active(struct bt_acs_conn *conn);

void acs_abort_request(struct bt_acs_conn *conn)
{
	bool had_work = (atomic_test_bit(&conn->state, ACS_STATE_CP_LOCKED) &&
			 !atomic_test_bit(&conn->state, ACS_STATE_CP_SERVER_TX)) ||
			acs_kex_in_progress(conn);

	/* Block new Control Point procedures while Abort is handled. */
	atomic_set_bit(&conn->state, ACS_STATE_CP_LOCKED);
	atomic_clear_bit(&conn->state, ACS_STATE_CP_SERVER_TX);

	/* Record whether work existed before making the Abort request visible. */
	atomic_set_bit_to(&conn->state, ACS_STATE_ABORT_HAD_WORK, had_work);
	atomic_set_bit(&conn->state, ACS_STATE_ABORT_REQUESTED);

	k_work_submit_to_queue(acs_get_wq(), &conn->abort_work);
}

struct acs_reply *acs_reply_alloc(struct bt_acs_conn *conn)
{
	__ASSERT_NO_MSG(conn != NULL);

	for (int i = 0; i < ACS_REPLY_SLOTS; i++) {
		if (!atomic_test_and_set_bit(conn->reply_in_use, i)) {
			memset(&conn->replies[i], 0, sizeof(conn->replies[i]));
			conn->replies[i].conn = conn;
			return &conn->replies[i];
		}
	}

	LOG_WRN("reply pool exhausted for conn %p", (void *)conn);
	return NULL;
}

/* Claim the reserved Abort slot, which acs_reply_alloc() never touches. */
static struct acs_reply *acs_reply_alloc_abort(struct bt_acs_conn *conn)
{
	if (atomic_test_and_set_bit(conn->reply_in_use, ACS_REPLY_ABORT_SLOT)) {
		return NULL;
	}
	memset(&conn->replies[ACS_REPLY_ABORT_SLOT], 0,
	       sizeof(conn->replies[ACS_REPLY_ABORT_SLOT]));
	conn->replies[ACS_REPLY_ABORT_SLOT].conn = conn;
	return &conn->replies[ACS_REPLY_ABORT_SLOT];
}

void acs_reply_take_request_buf(struct acs_reply *reply, struct acs_seg_rx_ctx *rx)
{
	reply->request = rx->buf;
	rx->buf = NULL;
}

void acs_reply_free(struct acs_reply *reply)
{
	struct bt_acs_conn *conn;
	size_t idx;

	if (reply == NULL) {
		return;
	}

	conn = reply->conn;
	idx = ARRAY_INDEX(conn->replies, reply);

	__ASSERT_NO_MSG(atomic_test_bit(conn->reply_in_use, idx));

	acs_buf_free(reply->request);
	reply->request = NULL;
	acs_buf_free(reply->response);
	reply->response = NULL;
	reply->aborted = false;

	/* Release the lock only when this reply still holds it. */
	acs_cp_unlock(conn, reply);

	atomic_clear_bit(conn->reply_in_use, idx);
}

void acs_reply_response_sent(void *user_data)
{
	struct acs_reply *reply = user_data;

	/* A procedure ends when its response is sent, not confirmed (§4.4.3). */
	if (!reply->holds_cp_lock) {
		return;
	}

	switch (reply->step) {
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	case ACS_REPLY_KEX_COMPLETE:
		/* Make the keys usable as the result is sent. */
		acs_kex_finalize_success(reply->conn);
		break;
#endif
	case ACS_REPLY_DONE:
		break;
	default:
		/* Keep the lock while more responses remain. */
		return;
	}

	acs_cp_unlock(reply->conn, reply);
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
static void acs_invalidate_work_handler(struct k_work *work)
{
	struct bt_acs_conn *conn = CONTAINER_OF(work, struct bt_acs_conn, invalidate_work);

	if (conn->conn == NULL) {
		return;
	}

	if (conn->remove_after_reply_key_id == 0U) {
		bt_acs_invalidate_security(conn->conn);
	} else {
		acs_sec_mgmt_remove_child_key(conn, conn->remove_after_reply_key_id);
	}
}

#endif

void acs_reply_init_conn(struct bt_acs_conn *conn)
{
	k_work_init(&conn->abort_work, acs_abort_work_handler);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	k_work_init(&conn->invalidate_work, acs_invalidate_work_handler);
	acs_data_out_init_conn(conn);
#endif
}

struct net_buf *acs_prepare_reply_buf(struct acs_reply *reply)
{
	struct net_buf *buf;

	__ASSERT_NO_MSG(reply != NULL);
	__ASSERT_NO_MSG(reply->conn != NULL);

	buf = reply->response;
	if (buf == NULL) {
		buf = acs_buf_alloc(K_NO_WAIT);
		if (buf == NULL) {
			LOG_ERR("buffer pool exhausted");
			return NULL;
		}
		reply->response = buf;
	}
	net_buf_reset(buf);

	/* Reserve space for segmentation and, when needed, encryption. */
	if (reply->channel != ACS_REPLY_CP) {
		net_buf_reserve(buf, ACS_CRYPTO_HEADROOM + ACS_SEG_HDR_SIZE);
		net_buf_add_le16(buf, reply->resource_handle);
	} else {
		net_buf_reserve(buf, ACS_SEG_HDR_SIZE);
	}

	return buf;
}

static void cp_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
		       void *user_data);

int acs_reply_submit(struct acs_reply *reply)
{
	__ASSERT_NO_MSG(reply != NULL);
	__ASSERT_NO_MSG(reply->conn != NULL);
	__ASSERT_NO_MSG(reply->response != NULL);

	switch (reply->channel) {
	case ACS_REPLY_CP:
		/* Wait until cp_tx is ready. */
		if (acs_seg_tx_busy(&reply->conn->cp_tx)) {
			if (reply->conn->cp_pending != NULL) {
				return -EBUSY;
			}
			reply->conn->cp_pending = reply;
			return 0;
		}
		return acs_seg_tx_send(&reply->conn->cp_tx, reply->conn->conn, acs_attr_cp(),
				       reply->response, cp_tx_done, reply);
		/* Queue the reply for encryption and transmission. */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	case ACS_REPLY_DON:
	case ACS_REPLY_DOI: {
		struct acs_tx_channel *chan =
			(reply->channel == ACS_REPLY_DON) ? &reply->conn->don : &reply->conn->doi;

		k_fifo_put(&chan->fifo, reply);
		k_work_submit_to_queue(acs_get_wq(), &chan->drain_work);
		return 0;
	}
#endif
	default:
		LOG_ERR("invalid channel %d", (int)reply->channel);
		return -EINVAL;
	}
}

bool acs_reply_continue(struct acs_reply *reply)
{
	int err;

	switch (reply->step) {
	case ACS_REPLY_DONE:
		return false;
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	case ACS_REPLY_KEX_OK:
		reply->step = ACS_REPLY_KEX_COMPLETE;
		err = acs_kex_send_result(reply);
		break;
	case ACS_REPLY_KEX_COMPLETE:
		/* Abort stopped the result before the keys became usable. */
		reply->step = ACS_REPLY_DONE;
		return false;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	case ACS_REPLY_DESCS_ISC:
		reply->step = ACS_REPLY_DESCS_KEY;
		err = acs_all_active_step_isc(reply);
		break;
	case ACS_REPLY_DESCS_KEY:
		reply->step = ACS_REPLY_DESCS_RC;
		err = acs_all_active_step_key(reply);
		break;
	case ACS_REPLY_DESCS_RC:
		reply->step = ACS_REPLY_DONE;
		err = acs_all_active_finish_success(reply);
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	case ACS_REPLY_INVALIDATE:
		reply->conn->remove_after_reply_key_id = reply->invalidate_key_id;
		k_work_submit_to_queue(acs_get_wq(), &reply->conn->invalidate_work);
		reply->step = ACS_REPLY_DONE;
		return false;
#endif
	default:
		CODE_UNREACHABLE;
		return false;
	}

	if (err) {
		reply->step = ACS_REPLY_DONE;
		return false;
	}
	return true;
}

/* Send the response waiting for cp_tx. */
static void cp_pending_submit(struct bt_acs_conn *acs_conn)
{
	struct acs_reply *reply = acs_conn->cp_pending;

	if (reply == NULL) {
		return;
	}

	acs_conn->cp_pending = NULL;
	if (acs_seg_tx_send(&acs_conn->cp_tx, acs_conn->conn, acs_attr_cp(), reply->response,
			    cp_tx_done, reply) != 0) {
		acs_reply_free(reply);
	}
}

static void cp_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
		       void *user_data)
{
	struct acs_reply *reply = user_data;
	struct bt_acs_conn *acs_conn = reply->conn;

	ARG_UNUSED(bt_conn);
	ARG_UNUSED(attr);

	if (err) {
		LOG_WRN("Plain CP indication failed: %d", err);
		acs_reply_free(reply);
		if (atomic_test_bit(&acs_conn->state, ACS_STATE_ABORT_REQUESTED)) {
			k_work_submit_to_queue(acs_get_wq(), &acs_conn->abort_work);
		}
		cp_pending_submit(acs_conn);
		return;
	}

	/* Continue a waiting Abort after the Control Point transfer ends. */
	if (atomic_test_bit(&acs_conn->state, ACS_STATE_ABORT_REQUESTED)) {
		reply->holds_cp_lock = false;
		acs_reply_free(reply);
		k_work_submit_to_queue(acs_get_wq(), &acs_conn->abort_work);
		return;
	}

	/* Build the next response from the confirmation callback. */
	if (acs_reply_continue(reply)) {
		return;
	}

	acs_reply_free(reply);
	cp_pending_submit(acs_conn);
}

/* Process Abort after any active Control Point transfer ends. */
static void acs_abort_process(struct bt_acs_conn *conn)
{
	bool had_work;
	struct acs_reply *reply;
	uint8_t code;

	if (conn->conn == NULL) {
		return;
	}
	if (!atomic_test_bit(&conn->state, ACS_STATE_ABORT_REQUESTED)) {
		return;
	}
	if (conn->cp_tx.buf != NULL) {
		return;
	}

	/* This flag and cp_tx_done() run on the same cooperative thread. */
	had_work = atomic_test_bit(&conn->state, ACS_STATE_ABORT_HAD_WORK);
	atomic_clear_bit(&conn->state, ACS_STATE_ABORT_REQUESTED);
	atomic_clear_bit(&conn->state, ACS_STATE_ABORT_HAD_WORK);

	/* Let an active transfer finish and discard its response. */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	if (acs_kex_in_progress(conn)) {
		acs_key_exchange_abort(conn);
	}
#endif

	acs_reply_abort_active(conn);

	atomic_set_bit(&conn->state, ACS_STATE_CP_LOCKED);

	code = had_work ? BT_ACS_CP_RESPONSE_SUCCESS : BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	LOG_DBG("Abort processed - sending Response Code 0x%02x", code);

	reply = acs_reply_alloc_abort(conn);
	if (reply == NULL) {
		LOG_ERR("Abort response alloc failed");
		atomic_clear_bit(&conn->state, ACS_STATE_CP_LOCKED);
		return;
	}
	reply->channel = ACS_REPLY_CP;
	/* Abort holds the lock until its response is sent. */
	reply->holds_cp_lock = true;

	if (acs_cp_rsp_status(reply, BT_ACS_CP_OPCODE_ABORT, code)) {
		LOG_ERR("Abort response send failed");
		acs_reply_free(reply);
	}
}

static void acs_abort_work_handler(struct k_work *work)
{
	struct bt_acs_conn *conn = CONTAINER_OF(work, struct bt_acs_conn, abort_work);

	acs_abort_process(conn);
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/* Stop a channel reply now or let its active transfer finish first. */
static void abort_active_reply(struct acs_reply **active, bool free_active)
{
	struct acs_reply *reply = *active;

	if (reply == NULL) {
		return;
	}
	if (free_active) {
		*active = NULL;
		acs_reply_free(reply);
	} else {
		/* Abort now holds the lock, so this reply must not release it. */
		reply->aborted = true;
		reply->holds_cp_lock = false;
	}
}
#endif

/* Stop a channel worker and remove its queued replies. */
static void acs_flush_work_fifo(struct k_work *work, struct k_fifo *fifo, bool free_active)
{
	sys_snode_t *snode;

	if (free_active) {
		struct k_work_sync sync;

		k_work_cancel_sync(work, &sync);
	} else {
		k_work_cancel(work);
	}

	while ((snode = k_fifo_get(fifo, K_NO_WAIT)) != NULL) {
		acs_reply_free(CONTAINER_OF(snode, struct acs_reply, node));
	}
}

/* Stop producers before channels, and release active replies last. */
static void acs_reply_flush_queues(struct bt_acs_conn *conn, bool free_active)
{
	if (conn == NULL) {
		return;
	}

	/* Removing a queued procedure also releases its lock. */
	acs_flush_work_fifo(&conn->cp_exec_work, &conn->cp_exec_fifo, free_active);

	if (conn->cp_pending != NULL) {
		struct acs_reply *pending = conn->cp_pending;

		conn->cp_pending = NULL;
		acs_reply_free(pending);
	}

	/* A discovery-only build has no protected request queues. */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	acs_flush_work_fifo(&conn->request_work, &conn->request_fifo, free_active);
	acs_flush_work_fifo(&conn->doi.drain_work, &conn->doi.fifo, free_active);
	acs_flush_work_fifo(&conn->don.drain_work, &conn->don.fifo, free_active);
#endif

	/* Abort lets an active transfer finish; disconnect stops it. */
	if (free_active) {
		/* Recover the plain Control Point reply before resetting its transfer. */
		struct acs_reply *cp_active = acs_seg_tx_user_data(&conn->cp_tx);

		acs_seg_tx_reset(&conn->cp_tx);
		acs_reply_free(cp_active);
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
		acs_seg_tx_reset(&conn->doi.tx);
		acs_seg_tx_reset(&conn->don.tx);
#endif
	}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	abort_active_reply(&conn->doi.active, free_active);
	abort_active_reply(&conn->don.active, free_active);
#endif
}

void acs_reply_cancel_all(struct bt_acs_conn *conn)
{
	acs_reply_flush_queues(conn, true);
}

/* Active replies are freed by their completion callbacks. */
static void acs_reply_abort_active(struct bt_acs_conn *conn)
{
	acs_reply_flush_queues(conn, false);
}
