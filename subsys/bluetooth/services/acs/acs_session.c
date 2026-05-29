/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <psa/crypto.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/logging/log.h>

#include "common/bt_str.h"
#include "acs_internal.h"
#include "acs_key_exchange.h"

#if defined(CONFIG_BT_SETTINGS)
#include <zephyr/psa/key_ids.h>

#include "host/settings.h"
#include "acs_key_desc.h"
#endif /* CONFIG_BT_SETTINGS */

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/*
 * Preserves crypto state (keys, nonces, counters) across BLE disconnect/
 * reconnect for the same peer.  Indexed by peer address so it survives
 * conn-index reuse.  Cleared only by explicit invalidation, bond deletion,
 * or power-off.
 */
struct acs_session_cache_entry {
	bt_addr_le_t addr;
	struct bt_acs_crypto_session crypto;
	uint16_t restriction_map_id;
	uint8_t status_flags;
};

static struct acs_session_cache_entry session_cache[CONFIG_BT_MAX_PAIRED];

static bool any_current_key_installed(const struct bt_acs_conn *acs_conn)
{
	for (size_t i = 0; i < ACS_KEY_ID_COUNT; i++) {
		if (acs_conn->crypto.key_runtimes[i].psa_key_id != 0U) {
			return true;
		}
	}
	return false;
}

static struct acs_session_cache_entry *session_cache_find(const bt_addr_le_t *addr)
{
	for (size_t i = 0; i < ARRAY_SIZE(session_cache); i++) {
		if (bt_addr_le_eq(&session_cache[i].addr, addr)) {
			return &session_cache[i];
		}
	}
	return NULL;
}

void acs_session_cache_save(const bt_addr_le_t *addr, const struct bt_acs_conn *acs_conn)
{
	struct acs_session_cache_entry *entry;

	if (!any_current_key_installed(acs_conn)) {
		return;
	}

	entry = session_cache_find(addr);
	if (!entry) {
		for (size_t i = 0; i < ARRAY_SIZE(session_cache); i++) {
			if (bt_addr_le_eq(&session_cache[i].addr, BT_ADDR_LE_ANY)) {
				bt_addr_le_copy(&session_cache[i].addr, addr);
				entry = &session_cache[i];
				break;
			}
		}
	}

	if (!entry) {
		LOG_WRN("No free session cache slot for peer %s", bt_addr_le_str(addr));
		return;
	}

	memcpy(&entry->crypto, &acs_conn->crypto, sizeof(entry->crypto));
	entry->restriction_map_id = acs_conn->restriction_map_id;
	entry->status_flags = acs_conn->status_flags;

	for (size_t i = ACS_KEY_ID_COUNT; i < ACS_KEY_RUNTIME_COUNT; i++) {
		entry->crypto.key_runtimes[i].psa_key_id = 0U;
	}

	LOG_DBG("Saved ACS session to RAM cache for peer %s", bt_addr_le_str(addr));
}

int acs_session_cache_restore(const bt_addr_le_t *addr, struct bt_acs_conn *acs_conn)
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

	for (size_t i = 0; i < ACS_KEY_ID_COUNT; i++) {
		struct bt_acs_key_desc_runtime *ck = &acs_conn->crypto.key_runtimes[i];
		psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;

		if (!ck->key_desc || ck->psa_key_id == 0U) {
			continue;
		}

		if (psa_get_key_attributes(ck->psa_key_id, &attrs) != PSA_SUCCESS) {
			LOG_ERR("Cached PSA key 0x%08x no longer valid",
				(unsigned int)ck->psa_key_id);
			psa_reset_key_attributes(&attrs);
			goto reimport_failed;
		}
		psa_reset_key_attributes(&attrs);

		if (ck->derive_key_id == 0U ||
		    psa_get_key_attributes(ck->derive_key_id, &attrs) != PSA_SUCCESS) {
			LOG_ERR("Cached derive key 0x%08x no longer valid",
				(unsigned int)ck->derive_key_id);
			psa_reset_key_attributes(&attrs);
			goto reimport_failed;
		}
		psa_reset_key_attributes(&attrs);

		entry->crypto.key_runtimes[i].psa_key_id = 0U;
		entry->crypto.key_runtimes[i].derive_key_id = 0U;
	}

	/* Find whichever exchange key the cached session left behind as the
	 * active parent for algorithm records; bind its children back.  If the
	 * cache held no established parent (security never reached ESTABLISHED
	 * before disconnect), there's nothing to bind.
	 */
	struct bt_acs_key_desc_runtime *parent = acs_key_exchange_established_key(acs_conn);

	if (parent != NULL) {
		err = acs_crypto_bind_algorithm_keys(acs_conn, parent);
		if (err) {
			LOG_ERR("Re-bind record keys failed: %d", err);
			goto reimport_failed;
		}
	}

	goto reimport_ok;

reimport_failed:
	LOG_ERR("Cached PSA keys invalid - resetting crypto");
	acs_crypto_destroy_exchange_keys(acs_conn);
	acs_crypto_reset(acs_conn);
	acs_conn->status_flags = BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	return -EIO;

reimport_ok:
	LOG_INF("ACS session restored from RAM cache for peer %s%s", bt_addr_le_str(addr),
		(acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED)
			? " (security established)"
			: "");

	if (acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED) {
		cb = acs_cb_get();
		if (cb && cb->security_established) {
			cb->security_established(acs_conn->conn);
		}
	}

	return 0;
}

static void session_cache_destroy_entry(struct acs_session_cache_entry *entry)
{
	for (size_t i = 0; i < ACS_KEY_ID_COUNT; i++) {
		acs_crypto_destroy_key(&entry->crypto.key_runtimes[i]);
	}
	memset(entry, 0, sizeof(*entry));
}

void acs_session_cache_clear_peer(const bt_addr_le_t *addr)
{
	struct acs_session_cache_entry *entry = session_cache_find(addr);

	if (entry) {
		session_cache_destroy_entry(entry);
	}
}

void acs_session_cache_clear_all_except(const bt_addr_le_t *keep_addr)
{
	for (size_t i = 0; i < ARRAY_SIZE(session_cache); i++) {
		if (!bt_addr_le_eq(&session_cache[i].addr, BT_ADDR_LE_ANY) &&
		    !bt_addr_le_eq(&session_cache[i].addr, keep_addr)) {
			session_cache_destroy_entry(&session_cache[i]);
		}
	}
}

#if defined(CONFIG_BT_SETTINGS)

#define ACS_SETTINGS_ADDR_LEN (BT_ADDR_SIZE * 2 + 1)

struct acs_persistent_meta {
	uint16_t parent_key_id;
	uint16_t restriction_map_id;
};

struct acs_session_slot {
	bt_addr_le_t addr;
	uint8_t bt_id;
	struct acs_persistent_meta meta;
};

static struct acs_session_slot acs_slots[CONFIG_BT_MAX_PAIRED];

BUILD_ASSERT((2 * CONFIG_BT_MAX_PAIRED) <= ZEPHYR_PSA_BT_ACS_KEY_ID_RANGE_SIZE,
	     "Bluetooth ACS PSA key ID range is too small for persistent key pairs");

static inline bool acs_slot_is_free(size_t idx)
{
	return bt_addr_le_eq(&acs_slots[idx].addr, BT_ADDR_LE_ANY);
}

static psa_key_id_t acs_psa_key_id(size_t slot)
{
	return ZEPHYR_PSA_BT_ACS_KEY_ID_RANGE_BEGIN + (psa_key_id_t)slot;
}

static psa_key_id_t acs_psa_derive_key_id(size_t slot)
{
	return ZEPHYR_PSA_BT_ACS_KEY_ID_RANGE_BEGIN + (psa_key_id_t)CONFIG_BT_MAX_PAIRED +
	       (psa_key_id_t)slot;
}

static int acs_slot_from_addr(const bt_addr_le_t *addr, size_t *slot)
{
	ARRAY_FOR_EACH(acs_slots, i) {
		if (bt_addr_le_eq(&acs_slots[i].addr, addr)) {
			*slot = i;
			return 0;
		}
	}

	return -ENOENT;
}

static int acs_slot_alloc(const bt_addr_le_t *addr, size_t *slot)
{
	int err;

	err = acs_slot_from_addr(addr, slot);
	if (err == 0) {
		return 0;
	}

	ARRAY_FOR_EACH(acs_slots, i) {
		if (acs_slot_is_free(i)) {
			bt_addr_le_copy(&acs_slots[i].addr, addr);
			*slot = i;
			return 0;
		}
	}

	return -ENOMEM;
}

static void acs_slot_free_idx(size_t idx)
{
	memset(&acs_slots[idx], 0, sizeof(acs_slots[idx]));
}

static void acs_destroy_persistent_slot(size_t slot)
{
	psa_status_t status;

	status = psa_destroy_key(acs_psa_key_id(slot));
	if (status != PSA_SUCCESS && status != PSA_ERROR_INVALID_HANDLE) {
		LOG_WRN("destroy persistent parent key failed for slot %zu: %d", slot, status);
	}

	status = psa_destroy_key(acs_psa_derive_key_id(slot));
	if (status != PSA_SUCCESS && status != PSA_ERROR_INVALID_HANDLE) {
		LOG_WRN("destroy persistent derive key failed for slot %zu: %d", slot, status);
	}
}

static bool acs_persistent_slot_keys_present(size_t slot)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	status = psa_get_key_attributes(acs_psa_key_id(slot), &attrs);
	psa_reset_key_attributes(&attrs);
	if (status != PSA_SUCCESS) {
		return false;
	}

	status = psa_get_key_attributes(acs_psa_derive_key_id(slot), &attrs);
	psa_reset_key_attributes(&attrs);

	return status == PSA_SUCCESS;
}

static void acs_session_erase_slot(size_t slot)
{
	acs_destroy_persistent_slot(slot);
	(void)bt_settings_delete("acs", acs_slots[slot].bt_id, &acs_slots[slot].addr);
	acs_slot_free_idx(slot);
}

/*
 * Find the long-lived parent exchange key (ECDH or OOB) on @p acs_conn.
 *
 * Distinct from acs_key_exchange_established_key() in that the KDF child is
 * deliberately excluded: only the parent is persisted, and the child gets
 * re-derived from it via psa_copy_key/HKDF on each session.
 */
static int acs_session_find_parent_key(const struct bt_acs_conn *acs_conn,
				       struct bt_acs_key_desc_runtime **exchange_key)
{
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(exchange_key != NULL);

	{
		struct bt_acs_key_desc_runtime *ecdh_key;

		if (acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_ECDH, &ecdh_key) == 0 &&
		    ecdh_key->psa_key_id != 0U) {
			*exchange_key = ecdh_key;
			return 0;
		}
	}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	{
		struct bt_acs_key_desc_runtime *oob_key;

		if (acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_OOB, &oob_key) == 0 &&
		    oob_key->psa_key_id != 0U) {
			*exchange_key = oob_key;
			return 0;
		}
	}
#endif

	return -ENOENT;
}

void acs_session_store(const struct bt_conn *conn, const struct bt_acs_conn *acs_conn)
{
	struct bt_acs_key_desc_runtime *parent_key;
	const bt_addr_le_t *addr = bt_conn_get_dst(conn);
	struct bt_conn_info info;
	size_t slot;
	int err;

	err = acs_session_find_parent_key(acs_conn, &parent_key);
	if (err) {
		LOG_ERR("No exchanged runtime key available to persist");
		return;
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info for settings store");
		return;
	}

	err = acs_slot_alloc(addr, &slot);
	if (err) {
		LOG_WRN("No free ACS session slot for peer %s", bt_addr_le_str(addr));
		return;
	}

	if (acs_slots[slot].meta.parent_key_id == acs_key_desc_runtime_key_id(parent_key) &&
	    acs_slots[slot].meta.restriction_map_id == acs_conn->restriction_map_id &&
	    acs_persistent_slot_keys_present(slot)) {
		LOG_DBG("ACS session already persisted, skipping");
		return;
	}

	acs_slots[slot].bt_id = info.id;
	acs_slots[slot].meta.parent_key_id = acs_key_desc_runtime_key_id(parent_key);
	acs_slots[slot].meta.restriction_map_id = acs_conn->restriction_map_id;

	/*
	 * Duplicate the parent key into persistent PSA storage in-keystore via
	 * psa_copy_key(); the raw key material never enters a plaintext buffer.
	 */
	err = acs_crypto_copy_key_to_persistent(parent_key, acs_psa_key_id(slot),
						acs_psa_derive_key_id(slot));
	if (err) {
		LOG_ERR("Failed to copy parent key to persistent store: %d", err);
		goto err_free_slot;
	}

	err = bt_settings_store("acs", info.id, addr, &acs_slots[slot].meta,
				sizeof(acs_slots[slot].meta));
	if (err) {
		LOG_ERR("Failed to store ACS metadata in settings: %d", err);
		acs_destroy_persistent_slot(slot);
		goto err_free_slot;
	}

	LOG_DBG("Stored ACS session for peer %s in slot %zu", bt_addr_le_str(addr), slot);
	return;

err_free_slot:
	acs_slot_free_idx(slot);
}

void acs_session_restore(struct bt_conn *conn, struct bt_acs_conn *acs_conn)
{
	const bt_addr_le_t *addr = bt_conn_get_dst(conn);
	struct bt_acs_key_desc_runtime *established_key;
	struct bt_acs_key_desc_runtime *parent_key;
	const struct bt_acs_cb *cb = acs_cb_get();
	size_t slot;
	int err;

	err = acs_slot_from_addr(addr, &slot);
	if (err) {
		LOG_DBG("No stored ACS session for peer %s", bt_addr_le_str(addr));
		return;
	}

	err = acs_crypto_key_runtime_lookup(acs_conn, acs_slots[slot].meta.parent_key_id,
					    &parent_key);
	if (err) {
		/*
		 * The stored session names a parent key type that this build
		 * can no longer instantiate (e.g. the key-exchange method was
		 * disabled in Kconfig since the session was persisted).  The
		 * NVS record is therefore permanently unrestorable, so erase it
		 * instead of leaving an orphaned entry behind.
		 */
		LOG_ERR("Stored ACS parent Key_ID 0x%04x has no runtime slot",
			acs_slots[slot].meta.parent_key_id);
		acs_session_erase_slot(slot);
		return;
	}

	/*
	 * Duplicate the persistent parent key back into a volatile runtime slot
	 * in-keystore via psa_copy_key(); no plaintext key material is exposed.
	 * A copy failure means the stored key is missing or unusable, so erase
	 * the orphaned slot.
	 */
	err = acs_crypto_copy_persistent_key_to_runtime(acs_psa_key_id(slot),
							acs_psa_derive_key_id(slot), parent_key);
	if (err) {
		LOG_ERR("Failed to restore parent key from persistent store: %d", err);
		acs_session_erase_slot(slot);
		return;
	}

	acs_conn->restriction_map_id = acs_slots[slot].meta.restriction_map_id;

	/* Bind algorithm-record children from the restored parent, again without
	 * export - psa_copy_key clones the parent's bits inside the keystore.
	 */
	err = acs_crypto_bind_algorithm_keys(acs_conn, parent_key);
	if (err) {
		LOG_ERR("Failed to bind restored record keys: %d", err);
		acs_crypto_reset(acs_conn);
		return;
	}

	/* acs_key_exchange_established_key falls through to whichever exchange
	 * key was restored (ECDH or OOB).  The KDF child is never persisted, so
	 * the lookup will skip it.
	 */
	established_key = acs_key_exchange_established_key(acs_conn);
	if (established_key != NULL) {
		acs_conn->status_flags |= BT_ACS_STATUS_SECURITY_ESTABLISHED;
	} else {
		acs_conn->status_flags &= ~BT_ACS_STATUS_SECURITY_ESTABLISHED;
	}

	LOG_INF("ACS session restored for peer %s%s", bt_addr_le_str(addr),
		(acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED)
			? " (security established)"
			: " (restored parent only; secure session not resumed)");

	if ((acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED) && cb &&
	    cb->security_established) {
		cb->security_established(conn);
	}
}

static void acs_bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
	size_t slot;

	acs_session_cache_clear_peer(peer);

	if (acs_slot_from_addr(peer, &slot) != 0) {
		return;
	}

	acs_session_erase_slot(slot);
	LOG_INF("ACS session deleted for peer %s (bond removed)", bt_addr_le_str(peer));
}

static struct bt_conn_auth_info_cb acs_auth_info_cb = {
	.bond_deleted = acs_bond_deleted,
};

void acs_session_register_auth_info_cb(void)
{
	bt_conn_auth_info_cb_register(&acs_auth_info_cb);
}

void acs_session_clear(const struct bt_conn *conn)
{
	const bt_addr_le_t *addr = bt_conn_get_dst(conn);
	size_t slot;

	if (acs_slot_from_addr(addr, &slot) != 0) {
		LOG_DBG("No stored ACS session to clear for peer %s", bt_addr_le_str(addr));
		return;
	}

	acs_session_erase_slot(slot);
	LOG_INF("ACS session erased for peer %s", bt_addr_le_str(addr));
}

void acs_session_clear_all_except(const struct bt_conn *conn)
{
	const bt_addr_le_t *keep_addr = bt_conn_get_dst(conn);

	for (size_t i = 0; i < ARRAY_SIZE(acs_slots); i++) {
		if (acs_slot_is_free(i) || bt_addr_le_eq(&acs_slots[i].addr, keep_addr)) {
			continue;
		}

		LOG_INF("ACS session erased for peer %s (invalidate all)",
			bt_addr_le_str(&acs_slots[i].addr));

		acs_session_erase_slot(i);
	}
}

static int acs_settings_set(const char *name, size_t len_rd, settings_read_cb read_cb, void *cb_arg)
{
	bt_addr_le_t addr;
	struct acs_persistent_meta meta;
	uint8_t bt_id = BT_ID_DEFAULT;
	ssize_t len;
	size_t slot;
	int err;

	if (!name) {
		return -EINVAL;
	}

	err = bt_settings_decode_key(name, &addr);
	if (err) {
		LOG_ERR("ACS settings: unable to decode address from '%s'", name);
		return -EINVAL;
	}

	if (!len_rd) {
		if (acs_slot_from_addr(&addr, &slot) == 0) {
			acs_slot_free_idx(slot);
		}
		return 0;
	}

	if (name[ACS_SETTINGS_ADDR_LEN] == '/') {
		char c = name[ACS_SETTINGS_ADDR_LEN + 1];

		if (c >= '0' && c <= '9') {
			bt_id = (uint8_t)(c - '0');
		}
	}

	len = read_cb(cb_arg, &meta, sizeof(meta));
	if (len != sizeof(meta)) {
		LOG_ERR("ACS settings: unexpected meta size (%zd)", len);
		return -EINVAL;
	}

	err = acs_slot_alloc(&addr, &slot);
	if (err) {
		LOG_WRN("ACS settings: no free slot for peer %s", bt_addr_le_str(&addr));
		return -ENOMEM;
	}

	acs_slots[slot].bt_id = bt_id;
	acs_slots[slot].meta = meta;

	LOG_DBG("ACS settings: restored meta for peer %s in slot %zu (Key_ID 0x%04x)",
		bt_addr_le_str(&addr), slot, meta.parent_key_id);

	return 0;
}

BT_SETTINGS_DEFINE(acs, "acs", acs_settings_set, NULL);

#endif /* CONFIG_BT_SETTINGS */
