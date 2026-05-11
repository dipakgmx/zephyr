/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

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

static void acs_session_restore_key_ids(struct bt_acs_conn *acs_conn)
{
	size_t slot = 0;

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	acs_conn->crypto.current_keys[slot++].key_desc = acs_key_desc_lookup(ACS_KEY_ID_ECDH);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	acs_conn->crypto.current_keys[slot++].key_desc = acs_key_desc_lookup(ACS_KEY_ID_KDF);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	acs_conn->crypto.current_keys[slot++].key_desc = acs_key_desc_lookup(ACS_KEY_ID_OOB);
#endif

	__ASSERT_NO_MSG(slot <= ARRAY_SIZE(acs_conn->crypto.current_keys));
}

bool acs_session_cache_has_room(const bt_addr_le_t *addr)
{
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (!acs_settings_cache[i].valid) {
			return true; /* free slot */
		}
		if (bt_addr_le_eq(&acs_settings_cache[i].addr, addr)) {
			return true; /* existing entry for this peer */
		}
	}
	return false;
}

void acs_session_store(struct bt_conn const *conn, struct bt_acs_conn const *acs_conn)
{
	const bt_addr_le_t *addr = bt_conn_get_dst(conn);
	struct bt_acs_runtime_key_state *parent_key;
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	struct bt_acs_runtime_key_state *oob_key;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	struct bt_acs_runtime_key_state *kdf_key;
#endif
	struct bt_conn_info info;
	struct bt_acs_session_store store = {0};
	int cache_found = -1;
	int cache_empty = -1;
	int err;

	err = acs_crypto_current_key_lookup((struct bt_acs_conn *)acs_conn, ACS_KEY_ID_ECDH,
					    &parent_key);
	if (err) {
		LOG_ERR("Missing ECDH runtime key state");
		return;
	}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	err = acs_crypto_current_key_lookup((struct bt_acs_conn *)acs_conn, ACS_KEY_ID_OOB,
					    &oob_key);
	if (err) {
		LOG_ERR("Missing OOB runtime key state");
		return;
	}
#endif

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
		store.tx_nonce_counter = 0;
		store.rx_nonce_counter = 0;
		store.kdf_child_valid = true;
		memcpy(store.child_key, kdf_key->key, CONFIG_BT_ACS_SESSION_KEY_SIZE);
		store.kdf_tx_nonce_counter = kdf_key->tx_nonce_counter;
		store.kdf_rx_nonce_counter = kdf_key->rx_nonce_counter;
	} else
#endif
	{
		/* ECDH/OOB exchange without active KDF child: session_key IS the AEAD key. */
		if (!parent_key || parent_key->psa_key_id == 0U) {
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
			parent_key = oob_key;
#endif
		}
		memcpy(store.parent_key, parent_key->key, CONFIG_BT_ACS_SESSION_KEY_SIZE);
		store.parent_key_id = acs_runtime_key_id(parent_key);
		store.tx_nonce_counter = parent_key->tx_nonce_counter;
		store.rx_nonce_counter = parent_key->rx_nonce_counter;
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) && !IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
		store.kdf_child_valid = false;
#endif
	}
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	memcpy(store.server_nonce_fixed, acs_conn->crypto.server_nonce_fixed,
	       CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
	memcpy(store.client_nonce_fixed, acs_conn->crypto.client_nonce_fixed,
	       CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
#endif /* BT_ACS_HAS_NONCE_FIXED */
	store.restriction_map_id = acs_conn->restriction_map_id;

	err = bt_settings_store("acs", info.id, addr, &store, sizeof(store));
	if (err) {
		LOG_ERR("Failed to store ACS session (err %d)", err);
		return;
	}

	/* Update in-memory cache for same-boot reconnects. */

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
	} else {
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
		for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			if (acs_settings_cache[i].valid &&
			    bt_addr_le_eq(&acs_settings_cache[i].addr, &addr)) {
				acs_settings_cache[i].valid = false;
				break;
			}
		}
		LOG_DBG("ACS session deleted for peer");
		return 0;
	}

	len = read_cb(cb_arg, &store, sizeof(store));
	if (len < (ssize_t)sizeof(store)) {
		LOG_ERR("ACS settings data too short (%zd)", len);
		return -EINVAL;
	}

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

	return 0;
}

static int acs_settings_commit(void)
{
	int count = 0;

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (acs_settings_cache[i].valid) {
			count++;
		}
	}

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
	struct bt_acs_session_store const *s;
	const struct bt_acs_cb *cb = acs_cb_get();

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (acs_settings_cache[i].valid &&
		    bt_addr_le_eq(&acs_settings_cache[i].addr, addr)) {
			struct bt_acs_runtime_key_state *parent_key;
			int err;
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) && !IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
			struct bt_acs_runtime_key_state *kdf_key;
#endif
			struct bt_acs_runtime_key_state *established_key;

			s = &acs_settings_cache[i].store;
			err = acs_crypto_current_key_lookup(acs_conn, s->parent_key_id,
							    &parent_key);

			if (err) {
				LOG_ERR("No runtime key state for restored parent Key_ID 0x%04x",
					s->parent_key_id);
				return;
			}

			acs_conn->restriction_map_id = s->restriction_map_id;
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
			memcpy(acs_conn->crypto.server_nonce_fixed, s->server_nonce_fixed,
			       CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
			memcpy(acs_conn->crypto.client_nonce_fixed, s->client_nonce_fixed,
			       CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) && !IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
			if (s->kdf_child_valid) {
				/* Restore both current keys: the exchanged parent key remains
				 * addressable by Key_ID, while the KDF child resumes as the live
				 * AEAD key with its own nonce state. */
				err = acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_KDF,
								    &kdf_key);
				if (err) {
					LOG_ERR("No runtime key state for restored KDF child");
					return;
				}

				memcpy(parent_key->key, s->parent_key,
				       CONFIG_BT_ACS_SESSION_KEY_SIZE);
				parent_key->tx_nonce_counter = 0;
				parent_key->rx_nonce_counter = 0;
				memcpy(kdf_key->key, s->child_key, CONFIG_BT_ACS_SESSION_KEY_SIZE);
				kdf_key->tx_nonce_counter = s->kdf_tx_nonce_counter;
				kdf_key->rx_nonce_counter = s->kdf_rx_nonce_counter;
			} else
#endif
			{
				memcpy(parent_key->key, s->parent_key,
				       CONFIG_BT_ACS_SESSION_KEY_SIZE);
				parent_key->tx_nonce_counter = s->tx_nonce_counter;
				parent_key->rx_nonce_counter = s->rx_nonce_counter;
			}

			if (acs_crypto_import_current_key(
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) && !IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
				    s->kdf_child_valid ? kdf_key :
#endif
						       parent_key) != 0) {
				LOG_ERR("Failed to import restored session key into PSA");
				memset(&acs_conn->crypto, 0, sizeof(acs_conn->crypto));
				acs_session_restore_key_ids(acs_conn);
				return;
			}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF) && !IS_ENABLED(CONFIG_BT_ACS_KDF_SESSION_KEY)
			if (s->kdf_child_valid) {
				if (acs_crypto_import_current_key(parent_key) != 0) {
					LOG_ERR("Failed to import restored parent key into PSA");
					return;
				}
			}
#endif

			acs_conn->crypto.key_state = BT_ACS_KEY_EXCHANGE_COMPLETE;

			/* Security is only fully established when the AEAD key is
			 * available.  With KDF configured, that means the child key
			 * must be present; the ECDH parent alone cannot perform
			 * protected operations. */
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
			if (kdf_key && kdf_key->psa_key_id != 0U) {
				acs_conn->status_flags |= BT_ACS_STATUS_SECURITY_ESTABLISHED;
			}
#else
			acs_conn->status_flags |= BT_ACS_STATUS_SECURITY_ESTABLISHED;
#endif

			LOG_INF("ACS session restored for bonded peer%s",
				(acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED)
					? " (security established)"
					: " (parent only, KDF child required)");

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

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (acs_settings_cache[i].valid &&
		    bt_addr_le_eq(&acs_settings_cache[i].addr, peer)) {
			acs_settings_cache[i].valid = false;
			bt_addr_le_to_str(peer, addr_str, sizeof(addr_str));
			LOG_INF("Session key deleted for peer %s (bond removed)", addr_str);
			break;
		}
	}
}

struct bt_conn_auth_info_cb acs_auth_info_cb = {
	.bond_deleted = acs_bond_deleted,
};

/* Erase flash sessions for all peers except the requesting peer. */
void acs_session_clear_all(struct bt_conn const *conn)
{
	struct bt_conn_info info;
	char addr_str[BT_ADDR_LE_STR_LEN];
	int ret;

	if (bt_conn_get_info(conn, &info) != 0) {
		return;
	}

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (!acs_settings_cache[i].valid) {
			continue;
		}

		if (bt_addr_le_eq(&acs_settings_cache[i].addr, bt_conn_get_dst(conn))) {
			continue;
		}

		ret = bt_settings_delete("acs", info.id, &acs_settings_cache[i].addr);
		if (ret == 0 || ret == -ENOENT) {
			bt_addr_le_to_str(&acs_settings_cache[i].addr, addr_str, sizeof(addr_str));
			LOG_INF("Session key erased for peer %s (invalidate all)", addr_str);
			acs_settings_cache[i].valid = false;
		} else {
			LOG_WRN("Failed to delete stored session (err %d)", ret);
		}
	}
}

void acs_session_invalidate_cache(const bt_addr_le_t *addr)
{
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (acs_settings_cache[i].valid &&
		    bt_addr_le_eq(&acs_settings_cache[i].addr, addr)) {
			acs_settings_cache[i].valid = false;
			break;
		}
	}
}

#endif /* CONFIG_BT_SETTINGS */
