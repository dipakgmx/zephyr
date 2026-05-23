/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/logging/log.h>

#include "common/bt_str.h"
#include "acs_internal.h"

#if defined(CONFIG_BT_SETTINGS)
#include <zephyr/psa/key_ids.h>
#include <psa/crypto.h>

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
		if (bt_addr_le_eq(&session_cache[i].addr, addr)) {
			return &session_cache[i];
		}
	}
	return NULL;
}

void acs_session_cache_save(const bt_addr_le_t *addr, const struct bt_acs_conn *acs_conn)
{
	struct acs_session_cache_entry *entry;
	bool has_key_material = false;

	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.current_keys); i++) {
		if (key_material_is_set(acs_conn->crypto.current_keys[i].key,
					sizeof(acs_conn->crypto.current_keys[i].key))) {
			has_key_material = true;
			break;
		}
	}

	if (!has_key_material) {
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

	entry->crypto.kex = NULL;
	for (size_t i = 0; i < ARRAY_SIZE(entry->crypto.current_keys); i++) {
		entry->crypto.current_keys[i].psa_key_id = 0U;
	}
	for (size_t i = 0; i < ARRAY_SIZE(entry->crypto.key_desc_runtimes); i++) {
		entry->crypto.key_desc_runtimes[i].psa_key_id = 0U;
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

	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.current_keys); i++) {
		struct bt_acs_runtime_key_state *ck = &acs_conn->crypto.current_keys[i];

		if (!ck->key_desc || !key_material_is_set(ck->key, sizeof(ck->key))) {
			continue;
		}

		err = acs_crypto_import_current_key(ck);
		if (err) {
			LOG_ERR("Re-import current key 0x%04x failed: %d", acs_runtime_key_id(ck),
				err);
			goto reimport_failed;
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
			goto reimport_failed;
		}
	}

	goto reimport_ok;

reimport_failed:
	LOG_ERR("Failed to re-import cached keys — resetting crypto");
	acs_crypto_reset(acs_conn);
	acs_conn->status_flags = BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	return err;

reimport_ok:
	LOG_INF("ACS session restored from RAM cache for peer %s%s", bt_addr_le_str(addr),
		(acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED)
			? " (security established)"
			: "");

	if (acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED) {
		cb = acs_cb_get();
		if (cb && cb->security_established) {
			struct bt_acs_runtime_key_state const *parent = NULL;

			for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.current_keys); i++) {
				struct bt_acs_runtime_key_state const *ck =
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
		if (!bt_addr_le_eq(&session_cache[i].addr, BT_ADDR_LE_ANY) &&
		    !bt_addr_le_eq(&session_cache[i].addr, keep_addr)) {
			memset(&session_cache[i], 0, sizeof(session_cache[i]));
		}
	}
}

#if defined(CONFIG_BT_SETTINGS)

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

BUILD_ASSERT(CONFIG_BT_MAX_PAIRED <= ZEPHYR_PSA_BT_ACS_KEY_ID_RANGE_SIZE,
	     "Bluetooth ACS PSA key ID range is too small for CONFIG_BT_MAX_PAIRED");

static inline bool acs_slot_is_free(size_t idx)
{
	return bt_addr_le_eq(&acs_slots[idx].addr, BT_ADDR_LE_ANY);
}

static psa_key_id_t acs_psa_key_id(size_t slot)
{
	return ZEPHYR_PSA_BT_ACS_KEY_ID_RANGE_BEGIN + (psa_key_id_t)slot;
}

static int acs_slot_from_addr(const bt_addr_le_t *addr, size_t *slot)
{
	for (size_t i = 0; i < ARRAY_SIZE(acs_slots); i++) {
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

	for (size_t i = 0; i < ARRAY_SIZE(acs_slots); i++) {
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

static int acs_persistent_import_key(psa_key_attributes_t *attrs, const void *data, size_t len,
				     const char *ctx)
{
	psa_key_id_t key_id = psa_get_key_id(attrs);
	psa_key_id_t imported_key_id;
	psa_status_t status;

	status = psa_import_key(attrs, data, len, &imported_key_id);
	if (status == PSA_ERROR_ALREADY_EXISTS) {
		status = psa_destroy_key(key_id);
		if (status != PSA_SUCCESS) {
			LOG_ERR("%s: destroy existing key 0x%08x failed: %d", ctx,
				(unsigned int)key_id, status);
			psa_reset_key_attributes(attrs);
			return -EIO;
		}

		status = psa_import_key(attrs, data, len, &imported_key_id);
	}

	psa_reset_key_attributes(attrs);

	if (status != PSA_SUCCESS) {
		LOG_ERR("%s: psa_import_key(0x%08x) failed: %d", ctx, (unsigned int)key_id, status);
		return -EIO;
	}

	return 0;
}

static int acs_persistent_export_key(psa_key_id_t key_id, void *data, size_t len, const char *ctx)
{
	size_t out_len;
	psa_status_t status;

	status = psa_export_key(key_id, data, len, &out_len);
	if (status != PSA_SUCCESS) {
		if (status != PSA_ERROR_INVALID_HANDLE) {
			LOG_ERR("%s: psa_export_key(0x%08x) failed: %d", ctx, (unsigned int)key_id,
				status);
		}
		return (status == PSA_ERROR_INVALID_HANDLE) ? -ENOENT : -EIO;
	}

	if (out_len != len) {
		LOG_ERR("%s: exported size mismatch for key 0x%08x (%zu != %zu)", ctx,
			(unsigned int)key_id, out_len, len);
		return -EIO;
	}

	return 0;
}

static void acs_destroy_persistent_slot(size_t slot)
{
	psa_status_t status;

	status = psa_destroy_key(acs_psa_key_id(slot));
	if (status != PSA_SUCCESS && status != PSA_ERROR_INVALID_HANDLE) {
		LOG_WRN("destroy persistent parent key failed for slot %zu: %d", slot, status);
	}
}

static void acs_session_erase_slot(size_t slot)
{
	acs_destroy_persistent_slot(slot);
	(void)bt_settings_delete("acs", acs_slots[slot].bt_id, &acs_slots[slot].addr);
	acs_slot_free_idx(slot);
}

static struct bt_acs_runtime_key_state *acs_restored_established_key(struct bt_acs_conn *acs_conn)
{
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	struct bt_acs_runtime_key_state *kdf_key;

	if (acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_KDF, &kdf_key) == 0 &&
	    acs_current_key_installed(kdf_key)) {
		return kdf_key;
	}

	return NULL;
#elif IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	struct bt_acs_runtime_key_state *ecdh_key;

	if (acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_ECDH, &ecdh_key) == 0 &&
	    acs_current_key_installed(ecdh_key)) {
		return ecdh_key;
	}

	return NULL;
#elif IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	struct bt_acs_runtime_key_state *oob_key;

	if (acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_OOB, &oob_key) == 0 &&
	    acs_current_key_installed(oob_key)) {
		return oob_key;
	}

	return NULL;
#else
	ARG_UNUSED(acs_conn);
	return NULL;
#endif
}

static int acs_session_store_exchange_key(const struct bt_acs_conn *acs_conn,
					  struct bt_acs_runtime_key_state **exchange_key)
{
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(exchange_key != NULL);

	{
		struct bt_acs_runtime_key_state *ecdh_key;

		err = acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_ECDH, &ecdh_key);
		if (!err && ecdh_key->psa_key_id != 0U) {
			*exchange_key = ecdh_key;
			return 0;
		}
	}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	{
		struct bt_acs_runtime_key_state *oob_key;

		err = acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_OOB, &oob_key);
		if (!err && oob_key->psa_key_id != 0U) {
			*exchange_key = oob_key;
			return 0;
		}
	}
#endif

	return -ENOENT;
}

void acs_session_store(const struct bt_conn *conn, const struct bt_acs_conn *acs_conn)
{
	struct bt_acs_runtime_key_state *parent_key;
	const bt_addr_le_t *addr = bt_conn_get_dst(conn);
	struct bt_conn_info info;
	psa_key_attributes_t parent_attrs = PSA_KEY_ATTRIBUTES_INIT;
	size_t slot;
	int err;

	err = acs_session_store_exchange_key(acs_conn, &parent_key);
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

	if (acs_slots[slot].meta.parent_key_id == acs_runtime_key_id(parent_key) &&
	    acs_slots[slot].meta.restriction_map_id == acs_conn->restriction_map_id) {
		psa_key_attributes_t chk = PSA_KEY_ATTRIBUTES_INIT;
		psa_status_t chk_st = psa_get_key_attributes(acs_psa_key_id(slot), &chk);

		psa_reset_key_attributes(&chk);
		if (chk_st == PSA_SUCCESS) {
			LOG_DBG("ACS session already persisted, skipping");
			return;
		}
	}

	acs_slots[slot].bt_id = info.id;
	acs_slots[slot].meta.parent_key_id = acs_runtime_key_id(parent_key);
	acs_slots[slot].meta.restriction_map_id = acs_conn->restriction_map_id;

	psa_set_key_id(&parent_attrs, acs_psa_key_id(slot));
	psa_set_key_lifetime(&parent_attrs, PSA_KEY_LIFETIME_PERSISTENT);
	psa_set_key_type(&parent_attrs, PSA_KEY_TYPE_RAW_DATA);
	psa_set_key_bits(&parent_attrs, CONFIG_BT_ACS_SESSION_KEY_SIZE * BITS_PER_BYTE);
	psa_set_key_usage_flags(&parent_attrs, PSA_KEY_USAGE_EXPORT);
	psa_set_key_algorithm(&parent_attrs, PSA_ALG_NONE);

	err = acs_persistent_import_key(&parent_attrs, parent_key->key, sizeof(parent_key->key),
					"store parent key");
	if (err) {
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
	struct bt_acs_runtime_key_state *established_key;
	struct bt_acs_runtime_key_state *parent_key;
	const struct bt_acs_cb *cb = acs_cb_get();
	size_t slot;
	int err;

	err = acs_slot_from_addr(addr, &slot);
	if (err) {
		LOG_DBG("No stored ACS session for peer %s", bt_addr_le_str(addr));
		return;
	}

	err = acs_crypto_current_key_lookup(acs_conn, acs_slots[slot].meta.parent_key_id,
					    &parent_key);
	if (err) {
		LOG_ERR("Stored ACS parent Key_ID 0x%04x has no runtime slot",
			acs_slots[slot].meta.parent_key_id);
		acs_session_erase_slot(slot);
		return;
	}

	err = acs_persistent_export_key(acs_psa_key_id(slot), parent_key->key,
					sizeof(parent_key->key), "restore ACS parent key");
	if (err) {
		acs_session_erase_slot(slot);
		return;
	}

	parent_key->psa_key_id = 0U;
	if (acs_crypto_import_current_key(parent_key) != 0) {
		LOG_ERR("Failed to import restored parent key into PSA");
		acs_crypto_reset(acs_conn);
		return;
	}

	acs_conn->restriction_map_id = acs_slots[slot].meta.restriction_map_id;

	if (acs_crypto_rebind_key_desc_runtimes(acs_conn) != 0) {
		LOG_ERR("Failed to import restored key descriptor runtimes into PSA");
		acs_crypto_reset(acs_conn);
		return;
	}

	established_key = acs_restored_established_key(acs_conn);
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
		cb->security_established(conn, established_key->key,
					 CONFIG_BT_ACS_SESSION_KEY_SIZE);
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

void acs_session_clear_all(const struct bt_conn *conn)
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

#define ACS_SETTINGS_ADDR_LEN 13

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
		bt_id = (uint8_t)(name[ACS_SETTINGS_ADDR_LEN + 1] - '0');
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
