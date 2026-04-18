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
#include "zephyr/sys/check.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/** Pool for ACS working buffers (Data In/Out payloads, CP responses, status indications). */
NET_BUF_POOL_FIXED_DEFINE(acs_buf_pool, ACS_BUF_COUNT, ACS_BUF_SIZE, 0, NULL);
/** Per-connection ACS persistent state, indexed by connection slot. */
static struct bt_acs_conn acs_conn_state[CONFIG_BT_MAX_CONN];
/** Pool of transient key-exchange contexts (released on handshake completion) */
static struct bt_acs_kex_ctx acs_kex_pool[CONFIG_BT_ACS_MAX_CONCURRENT_CONN];

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

struct bt_acs_kex_ctx *acs_kex_alloc(void)
{
	for (int i = 0; i < CONFIG_BT_ACS_MAX_CONCURRENT_CONN; i++) {
		if (!acs_kex_pool[i].in_use) {
			memset(&acs_kex_pool[i], 0, sizeof(acs_kex_pool[i]));
			acs_kex_pool[i].in_use = true;
			return &acs_kex_pool[i];
		}
	}
	return NULL;
}

void acs_kex_free(struct bt_acs_kex_ctx *kex)
{
	if (!kex) {
		return;
	}
	memset(kex, 0, sizeof(*kex));
	kex->in_use = false;
}

static void acs_cp_proc_init(struct bt_acs_conn *acs_conn)
{
	atomic_set(&acs_conn->cp_proc.locked, 0);
	acs_conn->cp_proc.response = NULL;
	acs_conn->cp_proc.abort_pending = false;
	acs_seg_tx_init(&acs_conn->cp_tx);
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	acs_seg_tx_init(&acs_conn->indicate_tx);
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION */
}

static void acs_cp_proc_cleanup(struct bt_acs_conn *acs_conn)
{
	atomic_set(&acs_conn->cp_proc.locked, 0);
	acs_conn->cp_proc.abort_pending = false;
	if (acs_conn->cp_proc.response) {
		acs_buf_free(acs_conn->cp_proc.response);
		acs_conn->cp_proc.response = NULL;
	}
	acs_seg_tx_reset(&acs_conn->cp_tx);

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	acs_seg_tx_reset(&acs_conn->indicate_tx);
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION */
}

struct bt_acs_conn *acs_conn_lookup(struct bt_conn *conn)
{
	uint8_t index = bt_conn_index(conn);
	struct bt_acs_conn *acs_conn = acs_conn_by_index(index);

	if (!acs_conn) {
		LOG_ERR("ACS connect lookup failed: %d", index);
		return NULL;
	}

	if (acs_conn->conn == conn) {
		return acs_conn;
	}

	return NULL;
}

struct bt_acs_conn *acs_conn_alloc(struct bt_conn *conn)
{
	uint8_t index = bt_conn_index(conn);
	struct bt_acs_conn *acs_conn = acs_conn_by_index(index);

	if (!acs_conn) {
		return NULL;
	}

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	{
		uint8_t saved_snf[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE];
		uint8_t saved_cnf[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE];

		memcpy(saved_snf, acs_conn->crypto.server_nonce_fixed,
		       CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
		memcpy(saved_cnf, acs_conn->crypto.client_nonce_fixed,
		       CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
		memset(acs_conn, 0, sizeof(*acs_conn));
		memcpy(acs_conn->crypto.server_nonce_fixed, saved_snf,
		       CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
		memcpy(acs_conn->crypto.client_nonce_fixed, saved_cnf,
		       CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
	}
#else
	memset(acs_conn, 0, sizeof(*acs_conn));
#endif
	acs_conn->conn = conn;
	acs_conn->attr_cp = acs_attr_cp();
	acs_conn->attr_status = acs_attr_status();
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	acs_conn->attr_don = acs_attr_don();
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	acs_conn->attr_doi = acs_attr_doi();
#endif
	acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
	acs_conn->status_flags = BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	acs_conn->restriction_map_id =
		IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION) ? CONFIG_BT_ACS_ACTIVE_RMAP_ID : 0;
	/* pending_reqs[] is zero-initialised by memset above (NULL = free slot) */
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	k_fifo_init(&acs_conn->indicate_fifo);
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION */
	acs_cp_proc_init(acs_conn);

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
	acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	acs_conn->kdf_child_active = false;
	/* Zero the parent key snapshot now that the session is torn down.
	 * crypto.session_key is cleared below via acs_crypto_destroy_session_key
	 * and the crypto memset; ecdh_parent_key lives outside that struct. */
	memset(acs_conn->ecdh_parent_key, 0, sizeof(acs_conn->ecdh_parent_key));
#endif
	acs_conn->status_flags = BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;

	/* Release transient key-exchange context back to the pool. */
	if (acs_conn->kex) {
		if (acs_conn->kex->ecdh_key_id != 0) {
			psa_destroy_key(acs_conn->kex->ecdh_key_id);
			acs_conn->kex->ecdh_key_id = 0;
		}
		acs_kex_free(acs_conn->kex);
		acs_conn->kex = NULL;
	}

	acs_crypto_destroy_session_key(acs_conn);
	/* Preserve nonce fixed parts across disconnect — they are set once per
	 * device pair and reused on reconnect (§3.6.4: "does not change for
	 * the life of the key").  Session key and counters are wiped.
	 */
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	{
		uint8_t saved_snf[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE];
		uint8_t saved_cnf[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE];

		memcpy(saved_snf, acs_conn->crypto.server_nonce_fixed,
		       CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
		memcpy(saved_cnf, acs_conn->crypto.client_nonce_fixed,
		       CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
		memset(&acs_conn->crypto, 0, sizeof(acs_conn->crypto));
		memcpy(acs_conn->crypto.server_nonce_fixed, saved_snf,
		       CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
		memcpy(acs_conn->crypto.client_nonce_fixed, saved_cnf,
		       CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
	}
#else
	memset(&acs_conn->crypto, 0, sizeof(acs_conn->crypto));
#endif
	/* Abort request contexts before freeing the shared I/O slot so queued/in-flight
	 * ACS Data Out activity cannot outlive the buffers it references.
	 */
	acs_prot_resource_req_abort_all(acs_conn);

	acs_cp_proc_cleanup(acs_conn);

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

	if (acs_conn->key_state == BT_ACS_KEY_EXCHANGE_COMPLETE && cb && cb->security_invalidated) {
		cb->security_invalidated(conn);
	}

#if defined(CONFIG_BT_SETTINGS)
	if (acs_conn->key_state == BT_ACS_KEY_EXCHANGE_COMPLETE
#if IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
	    && !acs_conn->kdf_child_active
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

	was_established = (acs_conn->key_state == BT_ACS_KEY_EXCHANGE_COMPLETE);

	acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
	acs_conn->status_flags &= ~BT_ACS_STATUS_SECURITY_ESTABLISHED;
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	acs_conn->kdf_child_active = false;
	memset(acs_conn->ecdh_parent_key, 0, sizeof(acs_conn->ecdh_parent_key));
#endif
	acs_crypto_destroy_session_key(acs_conn);
	memset(&acs_conn->crypto, 0, sizeof(acs_conn->crypto));

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
