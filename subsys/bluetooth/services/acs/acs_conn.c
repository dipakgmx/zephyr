/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/net_buf.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/logging/log.h>

#include "acs_internal.h"
#include "acs_reply.h"
#include "acs_cp.h"
#include "acs_request.h"
#include "acs_key_exchange.h"
#include "acs_keys.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Shared ACS buffer pool. */
NET_BUF_POOL_FIXED_DEFINE(acs_buf_pool, ACS_BUF_COUNT, ACS_BUF_SIZE, 0, NULL);

/* ACS state indexed by Bluetooth connection slot. */
static struct bt_acs_conn acs_conn_state[CONFIG_BT_MAX_CONN];

/* Orders acs_conn_hold() against the disconnect marking a slot inactive. */
static K_MUTEX_DEFINE(acs_conn_mutex);

struct net_buf *acs_buf_alloc(k_timeout_t timeout)
{
	struct net_buf *buf = net_buf_alloc(&acs_buf_pool, timeout);

	if (buf == NULL) {
		LOG_WRN("buffer pool exhausted");
	}
	return buf;
}

struct bt_acs_conn *acs_conn_by_index(uint8_t index)
{
	__ASSERT_NO_MSG(index < ARRAY_SIZE(acs_conn_state));
	return &acs_conn_state[index];
}

struct bt_acs_conn *acs_conn_lookup(struct bt_conn *conn)
{
	struct bt_acs_conn *acs_conn = &acs_conn_state[bt_conn_index(conn)];

	/* A disconnected slot is invisible to new requests until its teardown has run. */
	if (acs_conn->conn != conn || atomic_test_bit(&acs_conn->state, ACS_STATE_DISCONNECTED)) {
		return NULL;
	}

	return acs_conn;
}

struct bt_acs_conn *acs_conn_hold(struct bt_conn *conn)
{
	struct bt_acs_conn *acs_conn;

	(void)k_mutex_lock(&acs_conn_mutex, K_FOREVER);

	acs_conn = acs_conn_lookup(conn);
	if (acs_conn == NULL) {
		(void)k_mutex_unlock(&acs_conn_mutex);
	}

	return acs_conn;
}

void acs_conn_release(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);

	(void)k_mutex_unlock(&acs_conn_mutex);
}

static void acs_conn_teardown_work(struct k_work *work);

/* Indexes a static array sized by CONFIG_BT_MAX_CONN via bt_conn_index(). */
static struct bt_acs_conn *acs_conn_alloc(struct bt_conn *conn)
{
	struct bt_acs_conn *acs_conn = &acs_conn_state[bt_conn_index(conn)];
	struct k_work_sync sync;

	/*
	 * The previous link of this slot released its bt_conn as the last step of
	 * its teardown work; let that handler return before reusing the slot.
	 */
	(void)k_work_flush(&acs_conn->teardown_work, &sync);

	memset(acs_conn, 0, sizeof(*acs_conn));

	/* Held until the teardown work has stopped every ACS work item of the link. */
	acs_conn->conn = bt_conn_ref(conn);

	k_work_init(&acs_conn->teardown_work, acs_conn_teardown_work);
	acs_reply_init_conn(acs_conn);
	acs_cp_exec_queue_init(acs_conn);
	acs_status_work_init(acs_conn);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	acs_keys_reset(acs_conn);
	acs_request_queue_init(acs_conn);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

	return acs_conn;
}

/*
 * Stop all work and release all state of a disconnected link. Runs on the ACS
 * workqueue, like every ACS work item, so none of them is running and each
 * cancel takes effect at once.
 */
static void acs_conn_cleanup(struct bt_acs_conn *acs_conn)
{
	struct k_work_sync work_sync;

	LOG_DBG("Cleaning up ACS connection state %p", (void *)acs_conn);

	k_work_cancel_sync(&acs_conn->abort_work, &work_sync);
	k_work_cancel_delayable_sync(&acs_conn->status_work, &work_sync);
	acs_reply_cancel_all(acs_conn);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	k_work_cancel_sync(&acs_conn->invalidate_work, &work_sync);
	acs_key_exchange_abort(acs_conn);
	acs_keys_reset(acs_conn);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

	acs_seg_rx_reset(&acs_conn->cp_rx);
	acs_seg_rx_reset(&acs_conn->data_rx);
}

static void acs_bt_connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_acs_conn *acs_conn;

	if (err != 0) {
		return;
	}

	if (!acs_is_initialized()) {
		LOG_WRN("Connection established but ACS not initialized - no ACS state allocated");
		return;
	}

	acs_conn = acs_conn_alloc(conn);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	acs_keys_restore(acs_conn);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

	LOG_DBG("ACS connection allocated");
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
void acs_security_end(struct bt_acs_conn *acs_conn)
{
	const struct bt_acs_cb *cb = acs_cb_get();

	if (atomic_test_and_clear_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED) &&
	    cb != NULL && cb->security_invalidated != NULL) {
		cb->security_invalidated(acs_conn->conn);
	}
}

void acs_security_invalidate(struct bt_acs_conn *acs_conn)
{
	acs_security_end(acs_conn);
	atomic_clear_bit(&acs_conn->state, ACS_STATE_INVALIDATE_PENDING);
	acs_key_exchange_abort(acs_conn);
	acs_keys_reset(acs_conn);
	acs_keys_forget(acs_conn);
	acs_status_schedule(acs_conn);

	LOG_INF("ACS security invalidated for %s", bt_conn_dst_str(acs_conn->conn));
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

/*
 * Disconnect teardown. It runs on the ACS workqueue, where every ACS work item
 * runs, so it never cuts one off halfway, and the reference taken at connect
 * keeps conn valid for the items queued before it (the host likewise holds its
 * own until its disconnect work is done).
 */
static void acs_conn_teardown_work(struct k_work *work)
{
	struct bt_acs_conn *acs_conn = CONTAINER_OF(work, struct bt_acs_conn, teardown_work);
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	/*
	 * Check if invalidate is pending, when yes, invalidate the keys.
	 */
	if (atomic_test_bit(&acs_conn->state, ACS_STATE_INVALIDATE_PENDING)) {
		acs_security_invalidate(acs_conn);
	}
	acs_security_end(acs_conn);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

	acs_conn_cleanup(acs_conn);

	/* Last: once released, the host may reuse conn, and this slot with it. */
	bt_conn_drop(&acs_conn->conn);
}

static void acs_bt_disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_acs_conn *acs_conn;

	if (!acs_is_initialized()) {
		return;
	}

	/* A link established before bt_acs_init() ran has no ACS state. */
	acs_conn = acs_conn_lookup(conn);
	if (acs_conn == NULL) {
		return;
	}

	/*
	 * New requests stop finding the slot; the queued work drains first. The lock
	 * waits out an acs_conn_hold(), so the teardown cancels what its holder queued.
	 */
	(void)k_mutex_lock(&acs_conn_mutex, K_FOREVER);
	atomic_set_bit(&acs_conn->state, ACS_STATE_DISCONNECTED);
	(void)k_mutex_unlock(&acs_conn_mutex);
	k_work_submit_to_queue(acs_get_wq(), &acs_conn->teardown_work);

	LOG_DBG("ACS connection teardown queued (reason 0x%02x %s)", reason,
		bt_hci_err_to_str(reason));
}

BT_CONN_CB_DEFINE(acs_conn_callbacks) = {
	.connected = acs_bt_connected,
	.disconnected = acs_bt_disconnected,
};

int bt_acs_invalidate_security(struct bt_conn *conn)
{
	struct bt_acs_conn *acs_conn;

	if (conn == NULL || !acs_is_initialized()) {
		return -EINVAL;
	}

	acs_conn = acs_conn_lookup(conn);
	if (acs_conn == NULL) {
		return -ENOTCONN;
	}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	acs_security_invalidate(acs_conn);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

	return 0;
}
