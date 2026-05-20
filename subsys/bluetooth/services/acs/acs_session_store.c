/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_BT_SETTINGS)
#include <zephyr/settings/settings.h>
#include "host/settings.h"
#endif

#include "acs_internal.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if defined(CONFIG_BT_SETTINGS)

/** Cached sessions loaded from settings before any peer connects. */
struct acs_settings_cache_entry {
	struct bt_acs_session_store store;
	bt_addr_le_t addr;
	bool valid;
};

static struct acs_settings_cache_entry acs_settings_cache[CONFIG_BT_MAX_CONN];
K_SEM_DEFINE(acs_settings_cache_lock, 1, 1);

static int acs_session_store_exchange_key(struct bt_acs_conn const *acs_conn,
					  struct bt_acs_runtime_key_state **exchange_key)
{
	struct bt_acs_runtime_key_state *ecdh_key;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(exchange_key != NULL);

	err = acs_crypto_current_key_lookup((struct bt_acs_conn *)acs_conn, ACS_KEY_ID_ECDH,
					    &ecdh_key);
	if (!err && ecdh_key->psa_key_id != 0U) {
		*exchange_key = ecdh_key;
		return 0;
	}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	{
		struct bt_acs_runtime_key_state *oob_key;

		err = acs_crypto_current_key_lookup((struct bt_acs_conn *)acs_conn, ACS_KEY_ID_OOB,
						    &oob_key);
		if (!err && oob_key->psa_key_id != 0U) {
			*exchange_key = oob_key;
			return 0;
		}
	}
#endif

	return -ENOENT;
}

bool acs_session_cache_has_room(const bt_addr_le_t *addr)
{
	k_sem_take(&acs_settings_cache_lock, K_FOREVER);

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (!acs_settings_cache[i].valid) {
			k_sem_give(&acs_settings_cache_lock);
			return true; /* free slot */
		}
		if (bt_addr_le_eq(&acs_settings_cache[i].addr, addr)) {
			k_sem_give(&acs_settings_cache_lock);
			return true; /* existing entry for this peer */
		}
	}
	k_sem_give(&acs_settings_cache_lock);
	return false;
}

void acs_session_store(struct bt_conn const *conn, struct bt_acs_conn const *acs_conn)
{
	const bt_addr_le_t *addr = bt_conn_get_dst(conn);
	struct bt_acs_runtime_key_state *parent_key;
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	struct bt_acs_runtime_key_state *kdf_key;
#endif
	struct bt_conn_info info;
	struct bt_acs_session_store store = {0};
	int cache_found = -1;
	int cache_empty = -1;
	int err;

	err = acs_session_store_exchange_key(acs_conn, &parent_key);
	if (err) {
		LOG_ERR("No exchanged runtime key available to store");
		return;
	}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	err = acs_crypto_current_key_lookup((struct bt_acs_conn *)acs_conn, ACS_KEY_ID_KDF,
					    &kdf_key);
	if (err) {
		LOG_ERR("Missing KDF runtime key state");
		return;
	}
#endif

	if (bt_conn_get_info(conn, &info) < 0) {
		return;
	}

	/* Spec §4.4.4.15.1.4.4.1 reads like per-message counter persistence, but
	 * this implementation snapshots counters at post-KEX/session-store points
	 * instead to avoid NVS write amplification on every TX/RX.
	 */

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) && !IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
	if (kdf_key && kdf_key->psa_key_id != 0U) {
		/* Two-key layout: store the ECDH parent as the base session key with zero
		 * nonce counters (the parent is never directly used for AEAD; only the child
		 * consumes nonces), then write the live child key and its nonce state into the
		 * dedicated kdf_child_* fields.  On restore this lets the peer resume AEAD
		 * with the child key immediately, while the parent remains addressable by
		 * the Get Current Key List and Invalidate Key procedures. */
		memcpy(store.parent_key, parent_key->key, CONFIG_BT_ACS_SESSION_KEY_SIZE);
		store.parent_key_id = acs_runtime_key_id(parent_key);
		store.kdf_child_valid = true;
		memcpy(store.child_key, kdf_key->key, CONFIG_BT_ACS_SESSION_KEY_SIZE);
	} else
#endif
	{
		/* No live KDF child: persist the exchanged key that is actually installed. */
		memcpy(store.parent_key, parent_key->key, CONFIG_BT_ACS_SESSION_KEY_SIZE);
		store.parent_key_id = acs_runtime_key_id(parent_key);
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) && !IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
		store.kdf_child_valid = false;
#endif
	}
	store.restriction_map_id = acs_conn->restriction_map_id;
	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.record_states); i++) {
		const struct bt_acs_record_state *record_state = &acs_conn->crypto.record_states[i];
		struct bt_acs_session_store_record_state *stored_record;

		if (!record_state->key_desc) {
			continue;
		}

		if (store.nonce_record_count >= ARRAY_SIZE(store.nonce_records)) {
			LOG_WRN("Dropping nonce record state for Key_ID 0x%04x due to store "
				"capacity",
				acs_record_key_id(record_state));
			break;
		}

		stored_record = &store.nonce_records[store.nonce_record_count++];
		stored_record->key_id = acs_record_key_id(record_state);
		stored_record->client_nonce_set = record_state->client_nonce_set;
		memcpy(stored_record->server_nonce_fixed, record_state->server_nonce_fixed,
		       sizeof(stored_record->server_nonce_fixed));
		memcpy(stored_record->client_nonce_fixed, record_state->client_nonce_fixed,
		       sizeof(stored_record->client_nonce_fixed));
		stored_record->tx_nonce_counter = record_state->tx_nonce_counter;
		stored_record->rx_nonce_counter = record_state->rx_nonce_counter;
	}

	err = bt_settings_store("acs", info.id, addr, &store, sizeof(store));
	if (err) {
		LOG_ERR("Failed to store ACS session (err %d)", err);
		return;
	}

	/* Update in-memory cache for same-boot reconnects. */
	{
		k_sem_take(&acs_settings_cache_lock, K_FOREVER);

		for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			if (acs_settings_cache[i].valid &&
			    bt_addr_le_eq(&acs_settings_cache[i].addr, addr)) {
				cache_found = i;
				break;
			}
			if (cache_empty < 0 && !acs_settings_cache[i].valid) {
				cache_empty = i;
			}
		}

		if (cache_found >= 0) {
			acs_settings_cache[cache_found].store = store;
			LOG_DBG("ACS session stored for peer");
		} else if (cache_empty >= 0) {
			bt_addr_le_copy(&acs_settings_cache[cache_empty].addr, addr);
			acs_settings_cache[cache_empty].store = store;
			acs_settings_cache[cache_empty].valid = true;
		}

		k_sem_give(&acs_settings_cache_lock);
	}

	if (cache_found < 0 && cache_empty < 0) {
		LOG_WRN("ACS session written to NVS but cache full "
			"(CONFIG_BT_MAX_CONN=%d) — session will not be "
			"restored on reconnect this boot",
			CONFIG_BT_MAX_CONN);
	}
}

static int acs_settings_set(const char *name, size_t len_rd, settings_read_cb read_cb, void *cb_arg)
{
	struct bt_acs_session_store store;
	bt_addr_le_t addr;
	char addr_str[BT_ADDR_LE_STR_LEN];
	ssize_t len;
	int err;

	__ASSERT_NO_MSG(name != NULL);

	err = bt_settings_decode_key(name, &addr);

	if (err) {
		LOG_ERR("Unable to decode ACS settings key");
		return err;
	}

	if (len_rd == 0) {
		/* Key deleted — clear cache entry for this peer */
		k_sem_take(&acs_settings_cache_lock, K_FOREVER);

		for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			if (acs_settings_cache[i].valid &&
			    bt_addr_le_eq(&acs_settings_cache[i].addr, &addr)) {
				acs_settings_cache[i].valid = false;
				break;
			}
		}
		k_sem_give(&acs_settings_cache_lock);
		LOG_DBG("ACS session deleted for peer");
		return 0;
	}

	len = read_cb(cb_arg, &store, sizeof(store));
	if (len < (ssize_t)sizeof(store)) {
		LOG_ERR("ACS settings data too short (%zd)", len);
		return -EINVAL;
	}

	{
		k_sem_take(&acs_settings_cache_lock, K_FOREVER);

		for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			if (!acs_settings_cache[i].valid) {
				bt_addr_le_copy(&acs_settings_cache[i].addr, &addr);
				acs_settings_cache[i].store = store;
				acs_settings_cache[i].valid = true;
				bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
				LOG_INF("Loaded session key for peer %s (slot %d)", addr_str, i);
				break;
			}
		}

		k_sem_give(&acs_settings_cache_lock);
	}

	return 0;
}

static int acs_settings_commit(void)
{
	int count = 0;

	k_sem_take(&acs_settings_cache_lock, K_FOREVER);

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (acs_settings_cache[i].valid) {
			count++;
		}
	}

	k_sem_give(&acs_settings_cache_lock);

	if (count > 0) {
		LOG_INF("%d stored session key(s) ready for reconnect", count);
	} else {
		LOG_INF("no stored session keys found");
	}

	return 0;
}

BT_SETTINGS_DEFINE(acs, "acs", acs_settings_set, acs_settings_commit);

void acs_session_restore(struct bt_conn *conn, struct bt_acs_conn *acs_conn)
{
	const bt_addr_le_t *addr = bt_conn_get_dst(conn);
	struct bt_acs_session_store store_copy;
	bool found = false;
	const struct bt_acs_cb *cb = acs_cb_get();

	{
		k_sem_take(&acs_settings_cache_lock, K_FOREVER);

		for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			if (acs_settings_cache[i].valid &&
			    bt_addr_le_eq(&acs_settings_cache[i].addr, addr)) {
				store_copy = acs_settings_cache[i].store;
				found = true;
				break;
			}
		}

		k_sem_give(&acs_settings_cache_lock);
	}

	if (found) {
		struct bt_acs_runtime_key_state *parent_key;
		int err;
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
		struct bt_acs_runtime_key_state *kdf_key = NULL;
#endif
		struct bt_acs_runtime_key_state *established_key;

		err = acs_crypto_current_key_lookup(acs_conn, store_copy.parent_key_id,
						    &parent_key);

		if (err) {
			LOG_ERR("No runtime key state for restored parent Key_ID 0x%04x",
				store_copy.parent_key_id);
			return;
		}

		acs_conn->restriction_map_id = store_copy.restriction_map_id;

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) && !IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
		if (store_copy.kdf_child_valid) {
			/* Restore both current keys: the exchanged parent key remains
			 * addressable by Key_ID, while the KDF child resumes as the live
			 * AEAD key with its own nonce state. */
			err = acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_KDF, &kdf_key);
			if (err) {
				LOG_ERR("No runtime key state for restored KDF child");
				return;
			}

			memcpy(parent_key->key, store_copy.parent_key,
			       CONFIG_BT_ACS_SESSION_KEY_SIZE);
			memcpy(kdf_key->key, store_copy.child_key, CONFIG_BT_ACS_SESSION_KEY_SIZE);
		} else
#endif
		{
			memcpy(parent_key->key, store_copy.parent_key,
			       CONFIG_BT_ACS_SESSION_KEY_SIZE);
		}

		if (acs_crypto_import_current_key(
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) && !IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
			    store_copy.kdf_child_valid ? kdf_key :
#endif
						       parent_key) != 0) {
			LOG_ERR("Failed to import restored session key into PSA");
			acs_crypto_reset(acs_conn);
			return;
		}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) && !IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
		if (store_copy.kdf_child_valid) {
			if (acs_crypto_import_current_key(parent_key) != 0) {
				LOG_ERR("Failed to import restored parent key into PSA");
				return;
			}
		}
#endif

		if (acs_crypto_rebind_record_states(acs_conn) != 0) {
			LOG_ERR("Failed to import restored record-state keys into PSA");
			acs_crypto_reset(acs_conn);
			return;
		}

		for (size_t rec_idx = 0; rec_idx < store_copy.nonce_record_count; rec_idx++) {
			struct bt_acs_record_state *record_state;
			const struct bt_acs_session_store_record_state *stored_record =
				&store_copy.nonce_records[rec_idx];

			err = acs_crypto_record_state_lookup(acs_conn, stored_record->key_id,
							     &record_state);
			if (err) {
				LOG_WRN("No runtime record state for restored Key_ID "
					"0x%04x",
					stored_record->key_id);
				continue;
			}

			record_state->client_nonce_set = stored_record->client_nonce_set;
			memcpy(record_state->server_nonce_fixed, stored_record->server_nonce_fixed,
			       sizeof(record_state->server_nonce_fixed));
			memcpy(record_state->client_nonce_fixed, stored_record->client_nonce_fixed,
			       sizeof(record_state->client_nonce_fixed));
			record_state->tx_nonce_counter = stored_record->tx_nonce_counter;
			record_state->rx_nonce_counter = stored_record->rx_nonce_counter;
		}

		if (parent_key->psa_key_id != 0U
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
		    || (kdf_key && kdf_key->psa_key_id != 0U)
#endif
		) {
			acs_conn->status_flags |= BT_ACS_STATUS_SECURITY_ESTABLISHED;
		}

		LOG_INF("ACS session restored for bonded peer%s",
			(acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED)
				? " (security established)"
				: " (no valid restored key)");

		established_key = parent_key;
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
		if (acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED) {
			err = acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_KDF,
							    &established_key);
			if (err) {
				LOG_ERR("Missing KDF runtime key state for restored "
					"session");
				return;
			}
		}
#endif

		if ((acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED) && cb &&
		    cb->security_established && established_key) {
			cb->security_established(conn,
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
						 established_key->key,
#else
						 parent_key->key,
#endif
						 CONFIG_BT_ACS_SESSION_KEY_SIZE);
		}
		return;
	}
}

static void acs_bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	err = bt_settings_delete("acs", id, peer);
	if (err && err != -ENOENT) {
		LOG_WRN("Failed to delete session on bond removal (err %d)", err);
		return;
	}

	{
		k_sem_take(&acs_settings_cache_lock, K_FOREVER);

		for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			if (acs_settings_cache[i].valid &&
			    bt_addr_le_eq(&acs_settings_cache[i].addr, peer)) {
				acs_settings_cache[i].valid = false;
				bt_addr_le_to_str(peer, addr_str, sizeof(addr_str));
				LOG_INF("Session key deleted for peer %s (bond removed)", addr_str);
				break;
			}
		}

		k_sem_give(&acs_settings_cache_lock);
	}
}

struct bt_conn_auth_info_cb acs_auth_info_cb = {
	.bond_deleted = acs_bond_deleted,
};

/* Erase flash sessions for all peers except the requesting peer. */
void acs_session_clear_all(struct bt_conn const *conn)
{
	struct bt_conn_info info;
	bt_addr_le_t addrs[CONFIG_BT_MAX_CONN];
	char addr_str[BT_ADDR_LE_STR_LEN];
	size_t addr_count = 0U;
	int ret;

	if (bt_conn_get_info(conn, &info) != 0) {
		return;
	}

	{
		k_sem_take(&acs_settings_cache_lock, K_FOREVER);

		for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			if (!acs_settings_cache[i].valid) {
				continue;
			}

			if (bt_addr_le_eq(&acs_settings_cache[i].addr, bt_conn_get_dst(conn))) {
				continue;
			}

			addrs[addr_count++] = acs_settings_cache[i].addr;
		}

		k_sem_give(&acs_settings_cache_lock);
	}

	for (size_t i = 0; i < addr_count; i++) {
		ret = bt_settings_delete("acs", info.id, &addrs[i]);
		if (ret == 0 || ret == -ENOENT) {
			k_sem_take(&acs_settings_cache_lock, K_FOREVER);

			for (int slot = 0; slot < CONFIG_BT_MAX_CONN; slot++) {
				if (acs_settings_cache[slot].valid &&
				    bt_addr_le_eq(&acs_settings_cache[slot].addr, &addrs[i])) {
					acs_settings_cache[slot].valid = false;
					break;
				}
			}
			k_sem_give(&acs_settings_cache_lock);

			bt_addr_le_to_str(&addrs[i], addr_str, sizeof(addr_str));
			LOG_INF("Session key erased for peer %s (invalidate all)", addr_str);
		} else {
			LOG_WRN("Failed to delete stored session (err %d)", ret);
		}
	}
}

void acs_session_invalidate_cache(const bt_addr_le_t *addr)
{
	k_sem_take(&acs_settings_cache_lock, K_FOREVER);

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (acs_settings_cache[i].valid &&
		    bt_addr_le_eq(&acs_settings_cache[i].addr, addr)) {
			acs_settings_cache[i].valid = false;
			break;
		}
	}

	k_sem_give(&acs_settings_cache_lock);
}

#endif /* CONFIG_BT_SETTINGS */
