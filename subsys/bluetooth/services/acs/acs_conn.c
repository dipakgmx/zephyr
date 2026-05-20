/*
 * Copyright (c) 2026 Dipak Shetty
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

#if defined(CONFIG_BT_SETTINGS)
#include <zephyr/settings/settings.h>
#include "host/settings.h"
#endif

#include "acs_internal.h"
#include "acs_key_exchange.h"
#include "zephyr/sys/check.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/** Pool for ACS working buffers (Data In/Out payloads, CP responses, status indications). */
NET_BUF_POOL_FIXED_DEFINE(acs_buf_pool, ACS_BUF_COUNT, ACS_BUF_SIZE, 0, NULL);
/** Per-connection ACS persistent state, indexed by connection slot. */
static struct bt_acs_conn acs_conn_state[CONFIG_BT_MAX_CONN];
/** Pool of transient key-exchange contexts (released on handshake completion) */
static struct bt_acs_kex_ctx acs_kex_pool[CONFIG_BT_MAX_CONN];

#if defined(CONFIG_BT_SETTINGS) && IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
static bool acs_conn_has_current_key(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	struct bt_acs_runtime_key_state *current_key;
	int err = acs_crypto_current_key_lookup(acs_conn, key_id, &current_key);

	return err == 0 && current_key->psa_key_id != 0U;
}
#endif

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

int acs_kex_alloc(struct bt_acs_conn *acs_conn)
{
	uint8_t idx = bt_conn_index(acs_conn->conn);
	struct bt_acs_kex_ctx *kex = &acs_kex_pool[idx];

	if (kex->in_use) {
		return -EBUSY;
	}
	memset(kex, 0, sizeof(*kex));
	kex->in_use = true;
	acs_conn->crypto.kex = kex;
	return 0;
}

void acs_kex_free(struct bt_acs_kex_ctx *kex)
{
	if (!kex) {
		return;
	}
	memset(kex, 0, sizeof(*kex));
	kex->in_use = false;
}

struct bt_acs_conn *acs_conn_lookup(struct bt_conn *conn)
{
	struct bt_acs_conn *acs_conn = &acs_conn_state[bt_conn_index(conn)];

	return (acs_conn->conn == conn) ? acs_conn : NULL;
}

struct bt_acs_conn *acs_conn_alloc(struct bt_conn *conn)
{
	if (!conn) {
		return NULL;
	}

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
	if (!acs_conn) {
		return;
	}

	LOG_DBG("Cleaning up ACS connection state %p", (void *)acs_conn);

	acs_conn->conn = NULL;
	acs_conn->status_flags = BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;

	/* Release transient key-exchange context back to the pool. */
	acs_key_exchange_abort(acs_conn);

	acs_crypto_destroy_connection_keys(acs_conn);
	acs_crypto_destroy_connection_record_keys(acs_conn);
	/* Preserve nonce fixed parts across disconnect — they are set once per
	 * device pair and reused on reconnect (§3.6.4: "does not change for
	 * the life of the key").  Session key and counters are wiped.
	 */
	acs_crypto_reset_preserve_record_states(acs_conn);
	/* Abort request contexts before freeing the shared I/O slot so queued/in-flight
	 * ACS Data Out activity cannot outlive the buffers it references.
	 */
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
		LOG_WRN("Connection established but ACS not initialized — no ACS state allocated");
		return;
	}

	acs_conn = acs_conn_alloc(conn);

	if (!acs_conn) {
		LOG_ERR("Failed to allocate ACS connection state");
		return;
	}

#if defined(CONFIG_BT_SETTINGS)
	acs_session_restore(conn, acs_conn);
#endif
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

#if defined(CONFIG_BT_SETTINGS)
	if (acs_session_established(acs_conn)
#if IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
	    && !acs_conn_has_current_key(acs_conn, ACS_KEY_ID_KDF)
#endif
	) {
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
#if defined(CONFIG_BT_SETTINGS)
	struct bt_conn_info info;
	const bt_addr_le_t *addr;
	char addr_str[BT_ADDR_LE_STR_LEN];
	int del_err;
#endif

	CHECKIF(!conn) {
		return -EINVAL;
	}

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
	acs_crypto_destroy_connection_keys(acs_conn);
	acs_crypto_destroy_connection_record_keys(acs_conn);
	acs_crypto_reset(acs_conn);

#if defined(CONFIG_BT_SETTINGS)
	if (bt_conn_get_info(conn, &info) == 0) {
		addr = info.le.dst;
		del_err = bt_settings_delete("acs", info.id, addr);
		if (del_err && del_err != -ENOENT) {
			LOG_WRN("Failed to delete ACS session from flash (err %d)", del_err);
		} else {
			acs_session_invalidate_cache(addr);
			bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
			LOG_INF("Session key erased for peer %s", addr_str);
		}
	}
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
