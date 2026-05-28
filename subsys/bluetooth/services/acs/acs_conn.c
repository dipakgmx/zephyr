/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/net_buf.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/logging/log.h>

#include "common/bt_str.h"
#include "acs_internal.h"
#include "acs_key_exchange.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/** Pool for ACS working buffers (Data In/Out payloads, CP responses, status indications). */
NET_BUF_POOL_FIXED_DEFINE(acs_buf_pool, ACS_BUF_COUNT, ACS_BUF_SIZE, 0, NULL);
/** Per-connection ACS persistent state, indexed by connection slot. */
static struct bt_acs_conn acs_conn_state[CONFIG_BT_MAX_CONN];

struct net_buf *acs_buf_alloc(k_timeout_t timeout)
{
	struct net_buf *buf = net_buf_alloc(&acs_buf_pool, timeout);

	if (!buf) {
		LOG_WRN("buffer pool exhausted");
	}
	return buf;
}

void acs_buf_free(struct net_buf *buf)
{
	if (buf) {
		net_buf_unref(buf);
	}
}

struct bt_acs_conn *acs_conn_by_index(uint8_t index)
{
	if (index >= CONFIG_BT_MAX_CONN) {
		LOG_ERR("Connection index out of bounds: %u", index);
		return NULL;
	}

	return &acs_conn_state[index];
}

struct bt_acs_conn *acs_conn_lookup(struct bt_conn *conn)
{
	struct bt_acs_conn *acs_conn = &acs_conn_state[bt_conn_index(conn)];

	return (acs_conn->conn == conn) ? acs_conn : NULL;
}

struct bt_acs_conn *acs_conn_alloc(struct bt_conn *conn)
{
	__ASSERT_NO_MSG(conn != NULL);

	struct bt_acs_conn *acs_conn = &acs_conn_state[bt_conn_index(conn)];

	memset(acs_conn, 0, sizeof(*acs_conn));
	acs_crypto_reset_preserve_record_states(acs_conn);
	acs_conn->conn = conn;
	acs_conn->attr_cp = acs_attr_cp();
	acs_conn->attr_status = acs_attr_status();
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	acs_conn->attr_don = acs_attr_don();
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	acs_conn->attr_doi = acs_attr_doi();
#endif
	acs_conn->status_flags = BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	acs_conn->restriction_map_id =
		IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION) ? CONFIG_BT_ACS_ACTIVE_RMAP_ID : 0;
	/* pending_reqs[] is zero-initialised by memset above (NULL = free slot) */
	acs_request_queue_init(acs_conn);
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	acs_don_queue_init(acs_conn);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	acs_doi_queue_init(acs_conn);
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION */

	/* Wire up the embedded plain-CP procedure singleton. The conn's own
	 * memset has already zeroed the memory; mark it as plain CP so the
	 * slab/refcount paths skip it if anything ever drives it through them.
	 */
	acs_conn->plain_cp_proc.kind = ACS_PROC_KIND_PLAIN_CP;
	acs_conn->plain_cp_proc.acs_conn = acs_conn;
	acs_seg_tx_init(&acs_conn->cp_tx);
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	acs_seg_notify_async_init(&acs_conn->notify_tx);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	acs_seg_tx_init(&acs_conn->indicate_tx);
#endif

	/* Initialise RX reassembly contexts */
	acs_seg_rx_init(&acs_conn->cp_rx);
	acs_seg_rx_init(&acs_conn->data_rx);

	return acs_conn;
}

void acs_conn_cleanup(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);

	LOG_DBG("Cleaning up ACS connection state %p", (void *)acs_conn);

	acs_conn->conn = NULL;
	acs_conn->status_flags = BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;

	/* Release transient key-exchange context back to the pool. */
	acs_key_exchange_abort(acs_conn);

	acs_crypto_release_exchange_keys(acs_conn);
	acs_crypto_destroy_connection_record_keys(acs_conn);
	/* Preserve nonce fixed parts across disconnect - they are set once per
	 * device pair and reused on reconnect (§3.6.4: "does not change for
	 * the life of the key").  Session key and counters are wiped.
	 */
	acs_crypto_reset_preserve_record_states(acs_conn);
	/* Abort request contexts before freeing the shared I/O slot so queued/in-flight
	 * ACS Data Out activity cannot outlive the buffers it references.
	 */
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	acs_seg_notify_async_reset(&acs_conn->notify_tx);
#endif
	acs_procedure_abort_all(acs_conn);

	/* Reset the plain-CP procedure singleton. */
	atomic_set(&acs_conn->plain_cp_proc.plain_cp.locked, 0);
	acs_conn->plain_cp_proc.plain_cp.abort_pending = false;
	if (acs_conn->plain_cp_proc.buffers.response_buf) {
		acs_buf_free(acs_conn->plain_cp_proc.buffers.response_buf);
		acs_conn->plain_cp_proc.buffers.response_buf = NULL;
	}
	acs_seg_tx_reset(&acs_conn->cp_tx);
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	acs_seg_tx_reset(&acs_conn->indicate_tx);
#endif

	/* Reset and release buffers from RX reassembly contexts */
	acs_seg_rx_reset(&acs_conn->cp_rx);
	acs_seg_rx_reset(&acs_conn->data_rx);

	LOG_DBG("ACS connection cleanup complete %p", (void *)acs_conn);
}

/* Connected callback */
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

	if (!acs_conn) {
		LOG_ERR("Failed to allocate ACS connection state");
		return;
	}

	/* Try RAM cache first (survives disconnect, cleared on power-off).
	 * Fall back to NVS restore (survives power-off, ECDH parent only).
	 */
	if (acs_session_cache_restore(bt_conn_get_dst(conn), acs_conn) != 0) {
#if defined(CONFIG_BT_SETTINGS)
		acs_session_restore(conn, acs_conn);
#endif
	}

	LOG_DBG("ACS connection allocated");
}

static void acs_bt_disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_acs_conn *acs_conn;
	const struct bt_acs_cb *cb;

	if (!acs_is_initialized()) {
		return;
	}

	acs_conn = acs_conn_lookup(conn);

	if (!acs_conn) {
		return;
	}

	cb = acs_cb_get();

	if (acs_session_established(acs_conn) && cb && cb->security_invalidated) {
		cb->security_invalidated(conn);
	}

	/* Save crypto state to RAM cache before cleanup destroys PSA handles.
	 * This preserves keys, nonces and counters across disconnect/reconnect.
	 */
	acs_session_cache_save(bt_conn_get_dst(conn), acs_conn);

#if defined(CONFIG_BT_SETTINGS)
	if (acs_session_established(acs_conn)) {
		acs_session_store(conn, acs_conn);
	}
#endif

	acs_conn_cleanup(acs_conn);
	LOG_DBG("ACS connection cleaned up (reason 0x%02x)", reason);
}

BT_CONN_CB_DEFINE(acs_conn_callbacks) = {
	.connected = acs_bt_connected,
	.disconnected = acs_bt_disconnected,
};

int bt_acs_invalidate_security(struct bt_conn *conn)
{
	struct bt_acs_conn *acs_conn;
	bool was_established;
	const struct bt_acs_cb *cb;

	if (!conn) {
		return -EINVAL;
	}
	__ASSERT(conn != NULL, "conn must not be NULL");

	if (!acs_is_initialized()) {
		return -EINVAL;
	}

	acs_conn = acs_conn_lookup(conn);
	if (!acs_conn) {
		return -ENOTCONN;
	}

	was_established = acs_session_established(acs_conn);

	acs_conn->status_flags &= ~BT_ACS_STATUS_SECURITY_ESTABLISHED;
	acs_key_exchange_abort(acs_conn);
	acs_crypto_destroy_exchange_keys(acs_conn);
	acs_crypto_destroy_connection_record_keys(acs_conn);
	acs_crypto_reset(acs_conn);

	acs_session_cache_clear_peer(bt_conn_get_dst(conn));

#if defined(CONFIG_BT_SETTINGS)
	acs_session_clear(conn);
#endif

	cb = acs_cb_get();
	if (was_established && cb && cb->security_invalidated) {
		cb->security_invalidated(conn);
	}

	acs_status_indicate(conn);

	char addr_str_log[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str_log, sizeof(addr_str_log));
	LOG_INF("ACS security invalidated for %s", addr_str_log);

	return 0;
}
