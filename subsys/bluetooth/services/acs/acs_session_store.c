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
#include <zephyr/psa/key_ids.h>

#include <psa/crypto.h>

#include "common/bt_str.h"
#include "host/settings.h"
#include "acs_internal.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

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

static void acs_persistent_parent_attrs(psa_key_attributes_t *attrs, psa_key_id_t key_id)
{
	psa_set_key_id(attrs, key_id);
	psa_set_key_lifetime(attrs, PSA_KEY_LIFETIME_PERSISTENT);
	psa_set_key_type(attrs, PSA_KEY_TYPE_RAW_DATA);
	psa_set_key_bits(attrs, CONFIG_BT_ACS_SESSION_KEY_SIZE * BITS_PER_BYTE);
	psa_set_key_usage_flags(attrs, PSA_KEY_USAGE_EXPORT);
	psa_set_key_algorithm(attrs, PSA_ALG_NONE);
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

static bool acs_session_already_stored(size_t slot, uint16_t parent_key_id, uint16_t rmap_id)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	if (acs_slots[slot].meta.parent_key_id != parent_key_id ||
	    acs_slots[slot].meta.restriction_map_id != rmap_id) {
		return false;
	}

	status = psa_get_key_attributes(acs_psa_key_id(slot), &attrs);
	psa_reset_key_attributes(&attrs);

	return status == PSA_SUCCESS;
}

static int acs_session_store_exchange_key(const struct bt_acs_conn *acs_conn,
					  struct bt_acs_runtime_key_state **exchange_key)
{
	struct bt_acs_runtime_key_state *ecdh_key;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(exchange_key != NULL);

	err = acs_crypto_current_key_lookup(acs_conn, ACS_KEY_ID_ECDH, &ecdh_key);
	if (!err && ecdh_key->psa_key_id != 0U) {
		*exchange_key = ecdh_key;
		return 0;
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

	if (acs_session_already_stored(slot, acs_runtime_key_id(parent_key),
				       acs_conn->restriction_map_id)) {
		LOG_DBG("ACS session already persisted, skipping");
		return;
	}

	acs_slots[slot].bt_id = info.id;
	acs_slots[slot].meta.parent_key_id = acs_runtime_key_id(parent_key);
	acs_slots[slot].meta.restriction_map_id = acs_conn->restriction_map_id;

	acs_persistent_parent_attrs(&parent_attrs, acs_psa_key_id(slot));
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

	if (parent_key->psa_key_id != 0U) {
		acs_conn->status_flags |= BT_ACS_STATUS_SECURITY_ESTABLISHED;
	}

	LOG_INF("ACS session restored for peer %s%s", bt_addr_le_str(addr),
		(acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED)
			? " (security established)"
			: " (no valid restored key)");

	if ((acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED) && cb &&
	    cb->security_established) {
		cb->security_established(conn, parent_key->key, CONFIG_BT_ACS_SESSION_KEY_SIZE);
	}
}

static void acs_bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
	size_t slot;

	if (acs_slot_from_addr(peer, &slot) != 0) {
		return;
	}

	acs_session_erase_slot(slot);
	LOG_INF("ACS session deleted for peer %s (bond removed)", bt_addr_le_str(peer));
}

struct bt_conn_auth_info_cb acs_auth_info_cb = {
	.bond_deleted = acs_bond_deleted,
};

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

static int acs_settings_set(const char *name, size_t len_rd, settings_read_cb read_cb,
			    void *cb_arg)
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
