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
#include "acs_data_out.h"
#include "acs_rmap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static void acs_abort_work_handler(struct k_work *work);
static void acs_reply_abort_active(struct bt_acs_conn *acs_conn);

void acs_abort_request(struct bt_acs_conn *acs_conn)
{
	bool had_work = (atomic_test_bit(&acs_conn->state, ACS_STATE_CP_LOCKED) &&
			 !atomic_test_bit(&acs_conn->state, ACS_STATE_CP_SERVER_TX)) ||
			acs_kex_in_progress(acs_conn);

	/* Block new Control Point procedures while Abort is handled. */
	atomic_set_bit(&acs_conn->state, ACS_STATE_CP_LOCKED);
	atomic_clear_bit(&acs_conn->state, ACS_STATE_CP_SERVER_TX);

	/* Record whether work existed before making the Abort request visible. */
	atomic_set_bit_to(&acs_conn->state, ACS_STATE_ABORT_HAD_WORK, had_work);
	atomic_set_bit(&acs_conn->state, ACS_STATE_ABORT_REQUESTED);

	k_work_submit_to_queue(acs_get_wq(), &acs_conn->abort_work);
}

struct acs_reply *acs_reply_alloc(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);

	ARRAY_FOR_EACH(acs_conn->replies, idx) {
		if (!atomic_test_and_set_bit(acs_conn->reply_in_use, idx)) {
			memset(&acs_conn->replies[idx], 0, sizeof(acs_conn->replies[idx]));
			acs_conn->replies[idx].acs_conn = acs_conn;
			return &acs_conn->replies[idx];
		}
	}

	LOG_WRN("reply pool exhausted for conn %p", (void *)acs_conn);
	return NULL;
}

void acs_reply_free(struct acs_reply *reply)
{
	struct bt_acs_conn *acs_conn;
	size_t idx;

	if (reply == NULL) {
		return;
	}

	acs_conn = reply->acs_conn;
	idx = ARRAY_INDEX(acs_conn->replies, reply);

	__ASSERT_NO_MSG(atomic_test_bit(acs_conn->reply_in_use, idx));

	net_buf_drop(&reply->request);
	net_buf_drop(&reply->response);

	/* Release the lock only when this reply still holds it. */
	acs_cp_unlock(reply);

	atomic_clear_bit(acs_conn->reply_in_use, idx);
}

void acs_reply_finish(struct acs_reply *reply, int err)
{
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	acs_data_out_complete(reply, err);
#endif
	acs_reply_free(reply);
}

void acs_reply_message_sent(void *user_data)
{
	struct acs_reply *reply = user_data;

	if (!reply->holds_cp_lock) {
		return;
	}

	/*
	 * A procedure ends when its last message is sent, not confirmed (§4.4.3).
	 * A key removal that follows the reply keeps it open until delivery.
	 */
	if (reply->tx_msg->frags != NULL || reply->invalidate_key_id != 0U) {
		return;
	}

	/*
	 * Unlock first: storing a new ECDH key writes flash, and the next request
	 * would get Procedure Already In Progress meanwhile. It runs on the ACS
	 * workqueue, so it still waits for the store.
	 */
	acs_cp_unlock(reply);
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	/* A key exchange completes once its Key Exchange Response is sent. */
	if (acs_kex_in_progress(reply->acs_conn)) {
		acs_key_exchange_complete(reply->acs_conn);
	}
#endif
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
static void acs_invalidate_work_handler(struct k_work *work)
{
	struct bt_acs_conn *acs_conn = CONTAINER_OF(work, struct bt_acs_conn, invalidate_work);

	acs_sec_mgmt_remove_key(acs_conn, acs_conn->remove_after_reply_key_id);
}

#endif

static void cp_tx_done(int err, void *user_data);

void acs_reply_init_conn(struct bt_acs_conn *acs_conn)
{
	k_work_init(&acs_conn->abort_work, acs_abort_work_handler);
	acs_seg_tx_init(&acs_conn->cp_tx, acs_conn->conn, acs_attr_cp(), true, false, cp_tx_done,
			acs_reply_message_sent);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	k_work_init(&acs_conn->invalidate_work, acs_invalidate_work_handler);
	acs_data_out_init_conn(acs_conn);
#endif
}

struct net_buf *acs_reply_add_message(struct acs_reply *reply)
{
	struct net_buf *buf = acs_buf_alloc(K_NO_WAIT);

	if (buf == NULL) {
		return NULL;
	}

	/*
	 * Reserve the Segmentation_Header and Data Out header in front and save some space for
	 * the MAC room behind.
	 */
	net_buf_reserve(buf, ACS_BUF_HEADROOM);
	buf->size -= ACS_PROTECTED_TAG_SIZE;
	__ASSERT_NO_MSG(net_buf_tailroom(buf) == ACS_MESSAGE_MAX_SIZE);

	if (reply->response == NULL) {
		reply->response = buf;
	} else {
		net_buf_frag_add(reply->response, buf);
	}

	return buf;
}

struct net_buf *acs_reply_next_message(struct acs_reply *reply)
{
	reply->tx_msg = reply->tx_msg->frags;
	return reply->tx_msg;
}

void acs_reply_delivered(struct acs_reply *reply)
{
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	if (reply->invalidate_key_id != 0U) {
		reply->acs_conn->remove_after_reply_key_id = reply->invalidate_key_id;
		k_work_submit_to_queue(acs_get_wq(), &reply->acs_conn->invalidate_work);
	}
#else
	ARG_UNUSED(reply);
#endif
}

/* Send the reply's current message on the plain Control Point. */
static int cp_tx_send(struct acs_reply *reply)
{
	return acs_seg_tx_send(&reply->acs_conn->cp_tx, reply->tx_msg, reply);
}

int acs_reply_submit(struct acs_reply *reply)
{
	__ASSERT_NO_MSG(reply != NULL);
	__ASSERT_NO_MSG(reply->acs_conn != NULL);
	__ASSERT_NO_MSG(reply->response != NULL);

	reply->tx_msg = reply->response;

	switch (reply->channel) {
	case ACS_REPLY_CP:
		/* Wait until cp_tx is ready. */
		if (acs_seg_tx_busy(&reply->acs_conn->cp_tx)) {
			if (reply->acs_conn->cp_pending != NULL) {
				return -EBUSY;
			}
			reply->acs_conn->cp_pending = reply;
			return 0;
		}
		return cp_tx_send(reply);
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	case ACS_REPLY_DON:
	case ACS_REPLY_DOI: {
		struct bt_acs_conn *acs_conn = reply->acs_conn;
		struct acs_tx_channel *chan =
			(reply->channel == ACS_REPLY_DON) ? &acs_conn->don : &acs_conn->doi;

		/* Queue the reply for encryption and transmission. */
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

/* Send the response waiting for cp_tx. */
static void cp_pending_submit(struct bt_acs_conn *acs_conn)
{
	struct acs_reply *reply = acs_conn->cp_pending;

	if (reply == NULL) {
		return;
	}

	acs_conn->cp_pending = NULL;
	if (cp_tx_send(reply) != 0) {
		acs_reply_free(reply);
	}
}

static void cp_tx_done(int err, void *user_data)
{
	struct acs_reply *reply = user_data;
	struct bt_acs_conn *acs_conn = reply->acs_conn;

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

	if (acs_reply_next_message(reply) == NULL) {
		acs_reply_delivered(reply);
	} else if (cp_tx_send(reply) == 0) {
		return;
	}

	acs_reply_free(reply);
	cp_pending_submit(acs_conn);
}

/* Process Abort after any active Control Point transfer ends. */
static void acs_abort_process(struct bt_acs_conn *acs_conn)
{
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	const struct bt_acs_restriction_map *map;
	uint16_t isc_id;
#endif
	bool had_work;
	struct acs_reply *reply;
	uint8_t code;

	if (!atomic_test_bit(&acs_conn->state, ACS_STATE_ABORT_REQUESTED)) {
		return;
	}
	if (acs_seg_tx_busy(&acs_conn->cp_tx)) {
		return;
	}

	/* This flag and cp_tx_done() run on the same cooperative thread. */
	had_work = atomic_test_bit(&acs_conn->state, ACS_STATE_ABORT_HAD_WORK);
	atomic_clear_bit(&acs_conn->state, ACS_STATE_ABORT_REQUESTED);
	atomic_clear_bit(&acs_conn->state, ACS_STATE_ABORT_HAD_WORK);

	/* Let an active transfer finish and discard its response. */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	if (acs_kex_in_progress(acs_conn)) {
		acs_key_exchange_abort(acs_conn);
	}
#endif

	acs_reply_abort_active(acs_conn);

	atomic_set_bit(&acs_conn->state, ACS_STATE_CP_LOCKED);

	code = had_work ? BT_ACS_CP_RESPONSE_SUCCESS : BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	LOG_DBG("Abort processed - sending Response Code 0x%02x", code);

	/* The flush above freed every queued reply, so a slot is free. */
	reply = acs_reply_alloc(acs_conn);
	if (reply == NULL) {
		LOG_ERR("Abort response alloc failed");
		atomic_clear_bit(&acs_conn->state, ACS_STATE_CP_LOCKED);
		return;
	}
	reply->channel = ACS_REPLY_CP;
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	/*
	 * The active map decides how Abort was accepted: a protected Abort came on
	 * ACS Data In under this ISC and is answered on ACS Data Out Indicate (§4.4.3).
	 */
	map = acs_rmap_get_active_map();
	isc_id = (map != NULL) ? acs_rmap_cp_opcode_isc_id(map, BT_ACS_CP_OPCODE_ABORT)
			       : BT_ACS_ISC_ID_NONE;
	if (isc_id != BT_ACS_ISC_ID_NONE) {
		reply->channel = ACS_REPLY_DOI;
		reply->resource_handle = acs_cp_resource_handle();
		reply->isc_id = isc_id;
	}
#endif
	/* Abort holds the lock until its response is sent. */
	reply->holds_cp_lock = true;

	if (acs_cp_rsp_status(reply, BT_ACS_CP_OPCODE_ABORT, code)) {
		LOG_ERR("Abort response send failed");
		acs_reply_free(reply);
	}
}

static void acs_abort_work_handler(struct k_work *work)
{
	struct bt_acs_conn *acs_conn = CONTAINER_OF(work, struct bt_acs_conn, abort_work);

	acs_abort_process(acs_conn);
}

/* Why the reply queues are being drained. */
enum acs_reply_flush_mode {
	ACS_REPLY_FLUSH_DISCONNECT, /* Teardown: wait for workers, free active replies. */
	ACS_REPLY_FLUSH_ABORT,      /* Abort takes over: async cancel, keep its CP lock. */
};

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/* Stop a channel reply now or let its active transfer finish first. */
static void abort_active_reply(struct acs_reply **active, enum acs_reply_flush_mode mode)
{
	struct acs_reply *reply = *active;

	if (reply == NULL) {
		return;
	}
	if (mode == ACS_REPLY_FLUSH_DISCONNECT) {
		*active = NULL;
		acs_reply_finish(reply, -ECANCELED);
	} else {
		/* Abort now holds the lock, so this reply must not release it. */
		reply->aborted = true;
		reply->holds_cp_lock = false;
	}
}
#endif

/* Stop a channel worker and remove its queued replies. */
static void acs_flush_work_fifo(struct k_work *work, struct k_fifo *fifo,
				enum acs_reply_flush_mode mode, bool preserve_cp_lock)
{
	sys_snode_t *snode;

	if (mode == ACS_REPLY_FLUSH_DISCONNECT) {
		struct k_work_sync sync;

		k_work_cancel_sync(work, &sync);
	} else {
		k_work_cancel(work);
	}

	while ((snode = k_fifo_get(fifo, K_NO_WAIT)) != NULL) {
		struct acs_reply *reply = CONTAINER_OF(snode, struct acs_reply, node);

		if (preserve_cp_lock) {
			reply->holds_cp_lock = false;
		}
		acs_reply_finish(reply, -ECANCELED);
	}
}

/* Stop producers before channels, and release active replies last. */
static void acs_reply_flush_queues(struct bt_acs_conn *acs_conn, enum acs_reply_flush_mode mode)
{
	bool disconnect = (mode == ACS_REPLY_FLUSH_DISCONNECT);

	/* During Abort, queued procedures are discarded without releasing Abort's lock. */
	acs_flush_work_fifo(&acs_conn->cp_exec_work, &acs_conn->cp_exec_fifo, mode, !disconnect);

	if (acs_conn->cp_pending != NULL) {
		struct acs_reply *pending = acs_conn->cp_pending;

		acs_conn->cp_pending = NULL;
		if (!disconnect) {
			pending->holds_cp_lock = false;
		}
		acs_reply_free(pending);
	}

	/* A discovery-only build has no protected request queues. */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	acs_flush_work_fifo(&acs_conn->request_work, &acs_conn->request_fifo, mode, false);
	acs_flush_work_fifo(&acs_conn->doi.drain_work, &acs_conn->doi.fifo, mode, false);
	acs_flush_work_fifo(&acs_conn->don.drain_work, &acs_conn->don.fifo, mode, false);
#endif

	/* Abort lets an active transfer finish; disconnect stops it. */
	if (disconnect) {
		/* Recover the plain Control Point reply before resetting its transfer. */
		struct acs_reply *cp_active = acs_seg_tx_user_data(&acs_conn->cp_tx);

		acs_seg_tx_reset(&acs_conn->cp_tx);
		acs_reply_free(cp_active);
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
		acs_seg_tx_reset(&acs_conn->doi.tx);
		acs_seg_tx_reset(&acs_conn->don.tx);
#endif
	}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	abort_active_reply(&acs_conn->doi.active, mode);
	abort_active_reply(&acs_conn->don.active, mode);
#endif
}

void acs_reply_cancel_all(struct bt_acs_conn *acs_conn)
{
	acs_reply_flush_queues(acs_conn, ACS_REPLY_FLUSH_DISCONNECT);
}

/* Active replies are freed by their completion callbacks. */
static void acs_reply_abort_active(struct bt_acs_conn *acs_conn)
{
	acs_reply_flush_queues(acs_conn, ACS_REPLY_FLUSH_ABORT);
}
