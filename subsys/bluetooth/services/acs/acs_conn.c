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
#include "acs_crypto.h"
#include "acs_key_exchange.h"
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#include "acs_rmap.h"
#endif

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Shared ACS buffer pool. */
NET_BUF_POOL_FIXED_DEFINE(acs_buf_pool, ACS_BUF_COUNT, ACS_BUF_SIZE, 0, NULL);

/* ACS state indexed by Bluetooth connection slot. */
static struct bt_acs_conn acs_conn_state[CONFIG_BT_MAX_CONN];

struct net_buf *acs_buf_alloc(k_timeout_t timeout)
{
	struct net_buf *buf = net_buf_alloc(&acs_buf_pool, timeout);

	if (buf == NULL) {
		LOG_WRN("buffer pool exhausted");
	}
	return buf;
}

void acs_buf_free(struct net_buf *buf)
{
	if (buf != NULL) {
		net_buf_unref(buf);
	}
}

struct bt_acs_conn *acs_conn_by_index(uint8_t index)
{
	__ASSERT_NO_MSG(index < ARRAY_SIZE(acs_conn_state));
	return &acs_conn_state[index];
}

struct bt_acs_conn *acs_conn_lookup(struct bt_conn *conn)
{
	struct bt_acs_conn *acs_conn = &acs_conn_state[bt_conn_index(conn)];

	if (acs_conn->conn != conn) {
		return NULL;
	}

	return acs_conn;
}

/* Indexes a static array sized by CONFIG_BT_MAX_CONN via bt_conn_index(). */
static struct bt_acs_conn *acs_conn_alloc(struct bt_conn *conn)
{
	struct bt_acs_conn *acs_conn = &acs_conn_state[bt_conn_index(conn)];
	memset(acs_conn, 0, sizeof(*acs_conn));
	acs_conn->conn = conn;

	/* The current restriction map is server-wide, not connection state. */

	acs_reply_init_conn(acs_conn);
	acs_cp_exec_queue_init(acs_conn);
	acs_status_work_init(acs_conn);
	acs_seg_tx_init(&acs_conn->cp_tx, true, false);
	acs_seg_tx_set_sent_cb(&acs_conn->cp_tx, acs_reply_response_sent);
	acs_seg_rx_init(&acs_conn->cp_rx);
	acs_seg_rx_init(&acs_conn->data_rx);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	acs_crypto_init_slots(acs_conn);
	acs_request_queue_init(acs_conn);
	acs_seg_tx_init(&acs_conn->doi.tx, true, true);
	acs_seg_tx_set_sent_cb(&acs_conn->doi.tx, acs_reply_response_sent);
	acs_seg_tx_init(&acs_conn->don.tx, false, true);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

	return acs_conn;
}

static void acs_conn_cleanup(struct bt_acs_conn *acs_conn)
{
	struct k_work_sync work_sync;

	LOG_DBG("Cleaning up ACS connection state %p", (void *)acs_conn);

	atomic_clear_bit(&acs_conn->state, ACS_STATE_ABORT_REQUESTED);
	atomic_clear_bit(&acs_conn->state, ACS_STATE_ABORT_HAD_WORK);
	atomic_clear_bit(&acs_conn->state, ACS_STATE_INVALIDATE_PENDING);
	k_work_cancel_sync(&acs_conn->abort_work, &work_sync);
	k_work_cancel_delayable_sync(&acs_conn->status_work, &work_sync);
	acs_reply_cancel_all(acs_conn);
	acs_conn->conn = NULL;
	atomic_set_bit_to(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED, false);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	k_work_cancel_sync(&acs_conn->invalidate_work, &work_sync);
	acs_crypto_destroy_exchange_keys(acs_conn);
	acs_crypto_destroy_connection_record_keys(acs_conn);
	acs_key_exchange_abort(acs_conn);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

	atomic_clear_bit(&acs_conn->state, ACS_STATE_CP_LOCKED);
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
	acs_key_restore(acs_conn);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

	LOG_DBG("ACS connection allocated");
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
static void acs_security_invalidate(struct bt_acs_conn *acs_conn, struct bt_conn *conn)
{
	const struct bt_acs_cb *cb = acs_cb_get();
	bool was_established = atomic_test_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED);

	atomic_clear_bit(&acs_conn->state, ACS_STATE_INVALIDATE_PENDING);
	atomic_set_bit_to(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED, false);
	acs_key_exchange_abort(acs_conn);
	acs_crypto_reset(acs_conn);
	acs_key_store_clear(conn);

	if (was_established && cb != NULL && cb->security_invalidated != NULL) {
		cb->security_invalidated(conn);
	}

	acs_status_schedule(conn);
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

static void acs_bt_disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_acs_conn *acs_conn;
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	const struct bt_acs_cb *cb = acs_cb_get();
#endif

	if (!acs_is_initialized()) {
		return;
	}

	/* A link established before bt_acs_init() ran has no ACS state. */
	acs_conn = acs_conn_lookup(conn);
	if (acs_conn == NULL) {
		return;
	}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	/* Remove the key after its encrypted reply was sent. */
	if (atomic_test_bit(&acs_conn->state, ACS_STATE_INVALIDATE_PENDING) &&
	    acs_seg_tx_complete_pending(&acs_conn->doi.tx)) {
		bt_acs_invalidate_security(conn);
	}
	if (atomic_test_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED) && cb != NULL &&
	    cb->security_invalidated != NULL) {
		cb->security_invalidated(conn);
	}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

	acs_conn_cleanup(acs_conn);

	LOG_DBG("ACS connection cleaned up (reason 0x%02x %s)", reason, bt_hci_err_to_str(reason));
}

BT_CONN_CB_DEFINE(acs_conn_callbacks) = {
	.connected = acs_bt_connected,
	.disconnected = acs_bt_disconnected,
};

int bt_acs_invalidate_security(struct bt_conn *conn)
{
	struct bt_acs_conn *acs_conn;
	char addr_str_log[BT_ADDR_LE_STR_LEN];

	if (conn == NULL || !acs_is_initialized()) {
		return -EINVAL;
	}

	acs_conn = acs_conn_lookup(conn);
	if (acs_conn == NULL) {
		return -ENOTCONN;
	}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	acs_security_invalidate(acs_conn, conn);
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str_log, sizeof(addr_str_log));
	LOG_INF("ACS security invalidated for %s", addr_str_log);

	return 0;
}
