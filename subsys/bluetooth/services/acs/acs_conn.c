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
#include "zephyr/sys/check.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/** Pool for ACS working buffers (Data In/Out payloads, CP responses, status indications). */
NET_BUF_POOL_FIXED_DEFINE(acs_buf_pool, ACS_BUF_COUNT, ACS_BUF_SIZE, 0, NULL);
/** Per-connection ACS persistent state, indexed by connection slot. */
static struct bt_acs_conn acs_conn_state[CONFIG_BT_MAX_CONN];
/** Pool of transient key-exchange contexts (released on handshake completion) */
static struct bt_acs_kex_ctx acs_kex_pool[CONFIG_BT_MAX_CONN];

/*
 * RAM session cache — preserves crypto state (keys, nonces, counters) across
 * BLE disconnect/reconnect for the same peer.  Indexed by peer address so it
 * survives conn-index reuse.  Cleared only by explicit invalidation, bond
 * deletion, or power-off.
 */
struct acs_session_cache_entry {
	bt_addr_le_t addr;
	bool valid;
	struct bt_acs_crypto_session crypto;
	uint16_t restriction_map_id;
	uint8_t status_flags;
};

static struct acs_session_cache_entry session_cache[CONFIG_BT_MAX_PAIRED];

static bool key_material_is_set(const uint8_t *key, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (key[i] != 0U) {
			return true;
		}
	}
	return false;
}

static struct acs_session_cache_entry *session_cache_find(const bt_addr_le_t *addr)
{
	for (size_t i = 0; i < ARRAY_SIZE(session_cache); i++) {
		if (session_cache[i].valid && bt_addr_le_eq(&session_cache[i].addr, addr)) {
			return &session_cache[i];
		}
	}
	return NULL;
}

static struct acs_session_cache_entry *session_cache_alloc(const bt_addr_le_t *addr)
{
	struct acs_session_cache_entry *entry = session_cache_find(addr);

	if (entry) {
		return entry;
	}

	for (size_t i = 0; i < ARRAY_SIZE(session_cache); i++) {
		if (!session_cache[i].valid) {
			bt_addr_le_copy(&session_cache[i].addr, addr);
			session_cache[i].valid = true;
			return &session_cache[i];
		}
	}
	return NULL;
}

static bool has_exchanged_key_material(const struct bt_acs_conn *acs_conn)
{
	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.current_keys); i++) {
		if (key_material_is_set(acs_conn->crypto.current_keys[i].key,
					sizeof(acs_conn->crypto.current_keys[i].key))) {
			return true;
		}
	}
	return false;
}

static void session_cache_save(const bt_addr_le_t *addr, const struct bt_acs_conn *acs_conn)
{
	struct acs_session_cache_entry *entry;

	if (!has_exchanged_key_material(acs_conn)) {
		return;
	}

	entry = session_cache_alloc(addr);
	if (!entry) {
		LOG_WRN("No free session cache slot for peer %s", bt_addr_le_str(addr));
		return;
	}

	memcpy(&entry->crypto, &acs_conn->crypto, sizeof(entry->crypto));
	entry->restriction_map_id = acs_conn->restriction_map_id;
	entry->status_flags = acs_conn->status_flags;

	entry->crypto.kex = NULL;
	for (size_t i = 0; i < ARRAY_SIZE(entry->crypto.current_keys); i++) {
		entry->crypto.current_keys[i].psa_key_id = 0U;
	}
	for (size_t i = 0; i < ARRAY_SIZE(entry->crypto.key_desc_runtimes); i++) {
		entry->crypto.key_desc_runtimes[i].psa_key_id = 0U;
	}

	LOG_DBG("Saved ACS session to RAM cache for peer %s", bt_addr_le_str(addr));
}

static int session_cache_reimport_keys(struct bt_acs_conn *acs_conn)
{
	int err;

	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.current_keys); i++) {
		struct bt_acs_runtime_key_state *ck = &acs_conn->crypto.current_keys[i];

		if (!ck->key_desc || !key_material_is_set(ck->key, sizeof(ck->key))) {
			continue;
		}

		err = acs_crypto_import_current_key(ck);
		if (err) {
			LOG_ERR("Re-import current key 0x%04x failed: %d", acs_runtime_key_id(ck),
				err);
			return err;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.key_desc_runtimes); i++) {
		struct bt_acs_key_desc_runtime *rt = &acs_conn->crypto.key_desc_runtimes[i];

		if (!rt->key_desc || !key_material_is_set(rt->key, sizeof(rt->key))) {
			continue;
		}

		err = acs_crypto_import_record_key(rt);
		if (err) {
			LOG_ERR("Re-import record key 0x%04x failed: %d",
				acs_key_desc_runtime_key_id(rt), err);
			return err;
		}
	}

	return 0;
}

static int session_cache_restore(const bt_addr_le_t *addr, struct bt_acs_conn *acs_conn)
{
	struct acs_session_cache_entry *entry = session_cache_find(addr);
	const struct bt_acs_cb *cb;
	int err;

	if (!entry) {
		return -ENOENT;
	}

	memcpy(&acs_conn->crypto, &entry->crypto, sizeof(acs_conn->crypto));
	acs_conn->restriction_map_id = entry->restriction_map_id;
	acs_conn->status_flags = entry->status_flags;

	err = session_cache_reimport_keys(acs_conn);
	if (err) {
		LOG_ERR("Failed to re-import cached keys — resetting crypto");
		acs_crypto_reset(acs_conn);
		acs_conn->status_flags = BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
		return err;
	}

	LOG_INF("ACS session restored from RAM cache for peer %s%s", bt_addr_le_str(addr),
		(acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED)
			? " (security established)"
			: "");

	if ((acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED)) {
		cb = acs_cb_get();
		if (cb && cb->security_established) {
			struct bt_acs_runtime_key_state *parent = NULL;

			for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.current_keys); i++) {
				struct bt_acs_runtime_key_state *ck =
					&acs_conn->crypto.current_keys[i];

				if (ck->psa_key_id != 0U) {
					parent = ck;
				}
			}
			if (parent) {
				cb->security_established(acs_conn->conn, parent->key,
							 CONFIG_BT_ACS_SESSION_KEY_SIZE);
			}
		}
	}

	return 0;
}

void acs_session_cache_clear_peer(const bt_addr_le_t *addr)
{
	struct acs_session_cache_entry *entry = session_cache_find(addr);

	if (entry) {
		memset(entry, 0, sizeof(*entry));
	}
}

void acs_session_cache_clear_all_except(const bt_addr_le_t *keep_addr)
{
	for (size_t i = 0; i < ARRAY_SIZE(session_cache); i++) {
		if (session_cache[i].valid && !bt_addr_le_eq(&session_cache[i].addr, keep_addr)) {
			memset(&session_cache[i], 0, sizeof(session_cache[i]));
		}
	}
}

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

	/* Try RAM cache first (survives disconnect, cleared on power-off).
	 * Fall back to NVS restore (survives power-off, ECDH parent only).
	 */
	if (session_cache_restore(bt_conn_get_dst(conn), acs_conn) != 0) {
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
	session_cache_save(bt_conn_get_dst(conn), acs_conn);

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
