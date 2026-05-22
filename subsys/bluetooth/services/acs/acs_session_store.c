/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/logging/log.h>
#include <zephyr/psa/key_ids.h>

#include <psa/crypto.h>

#include "host/keys.h"
#include "acs_internal.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if defined(CONFIG_BT_SETTINGS)

enum acs_psa_slot {
	ACS_PSA_SLOT_PARENT = 0,
	ACS_PSA_SLOT_META = 1,
	ACS_PSA_SLOT_COUNT,
};

struct acs_persistent_meta {
	uint16_t parent_key_id;
	uint16_t restriction_map_id;
};

struct acs_bond_slot_lookup_ctx {
	const struct bt_keys *target;
	size_t current;
	size_t slot;
	bool found;
};

struct acs_clear_all_ctx {
	size_t keep_slot;
};

BUILD_ASSERT((CONFIG_BT_MAX_PAIRED * ACS_PSA_SLOT_COUNT) <= ZEPHYR_PSA_BT_ACS_KEY_ID_RANGE_SIZE,
	     "Bluetooth ACS PSA key ID range is too small for CONFIG_BT_MAX_PAIRED");

static psa_key_id_t acs_psa_key_id(size_t bond_slot, enum acs_psa_slot slot)
{
	return ZEPHYR_PSA_BT_ACS_KEY_ID_RANGE_BEGIN + (bond_slot * ACS_PSA_SLOT_COUNT) + slot;
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

static void acs_persistent_meta_attrs(psa_key_attributes_t *attrs, psa_key_id_t key_id)
{
	psa_set_key_id(attrs, key_id);
	psa_set_key_lifetime(attrs, PSA_KEY_LIFETIME_PERSISTENT);
	psa_set_key_type(attrs, PSA_KEY_TYPE_RAW_DATA);
	psa_set_key_bits(attrs, sizeof(struct acs_persistent_meta) * BITS_PER_BYTE);
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

static void acs_destroy_persistent_slot(size_t bond_slot)
{
	psa_status_t parent_status;
	psa_status_t meta_status;

	parent_status = psa_destroy_key(acs_psa_key_id(bond_slot, ACS_PSA_SLOT_PARENT));
	meta_status = psa_destroy_key(acs_psa_key_id(bond_slot, ACS_PSA_SLOT_META));

	if (parent_status != PSA_SUCCESS && parent_status != PSA_ERROR_INVALID_HANDLE) {
		LOG_WRN("destroy persistent parent key failed for slot %zu: %d", bond_slot,
			parent_status);
	}

	if (meta_status != PSA_SUCCESS && meta_status != PSA_ERROR_INVALID_HANDLE) {
		LOG_WRN("destroy persistent meta key failed for slot %zu: %d", bond_slot,
			meta_status);
	}
}

static void acs_find_bond_slot_cb(struct bt_keys *keys, void *user_data)
{
	struct acs_bond_slot_lookup_ctx *ctx = user_data;

	if (ctx->found) {
		return;
	}

	if (keys == ctx->target) {
		ctx->slot = ctx->current;
		ctx->found = true;
		return;
	}

	ctx->current++;
}

static int acs_bond_slot_from_keys(const struct bt_keys *keys, size_t *bond_slot)
{
	struct acs_bond_slot_lookup_ctx ctx = {
		.target = keys,
	};

	if (!keys || !bond_slot) {
		return -EINVAL;
	}

	bt_keys_foreach_type(BT_KEYS_ALL, acs_find_bond_slot_cb, &ctx);

	if (!ctx.found) {
		return -ENOENT;
	}

	*bond_slot = ctx.slot;
	return 0;
}

static int acs_bond_slot_from_conn(const struct bt_conn *conn, size_t *bond_slot)
{
	struct bt_conn_info info;
	struct bt_keys *keys;
	int err;

	if (!conn || !bond_slot) {
		return -EINVAL;
	}

	err = bt_conn_get_info(conn, &info);
	if (err != 0) {
		return -ENOTCONN;
	}

	keys = bt_keys_find_addr(info.id, info.le.dst);
	if (!keys) {
		keys = bt_keys_find_irk(info.id, info.le.dst);
	}
	if (!keys) {
		return -ENOENT;
	}

	return acs_bond_slot_from_keys(keys, bond_slot);
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
	struct acs_persistent_meta meta;
	psa_key_attributes_t parent_attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_attributes_t meta_attrs = PSA_KEY_ATTRIBUTES_INIT;
	size_t bond_slot;
	int err;

	err = acs_session_store_exchange_key(acs_conn, &parent_key);
	if (err) {
		LOG_ERR("No exchanged runtime key available to persist");
		return;
	}

	err = acs_bond_slot_from_conn(conn, &bond_slot);
	if (err) {
		LOG_DBG("Peer has no bonded slot; skipping ACS persistent store");
		return;
	}

	meta.parent_key_id = acs_runtime_key_id(parent_key);
	meta.restriction_map_id = acs_conn->restriction_map_id;

	acs_persistent_parent_attrs(&parent_attrs, acs_psa_key_id(bond_slot, ACS_PSA_SLOT_PARENT));
	err = acs_persistent_import_key(&parent_attrs, parent_key->key, sizeof(parent_key->key),
					"store parent key");
	if (err) {
		return;
	}

	acs_persistent_meta_attrs(&meta_attrs, acs_psa_key_id(bond_slot, ACS_PSA_SLOT_META));
	err = acs_persistent_import_key(&meta_attrs, &meta, sizeof(meta), "store ACS metadata");
	if (err) {
		acs_destroy_persistent_slot(bond_slot);
		return;
	}

	LOG_DBG("Stored ACS parent key in PSA slot %zu", bond_slot);
}

void acs_session_restore(struct bt_conn *conn, struct bt_acs_conn *acs_conn)
{
	struct acs_persistent_meta meta;
	struct bt_acs_runtime_key_state *parent_key;
	const struct bt_acs_cb *cb = acs_cb_get();
	size_t bond_slot;
	int err;

	err = acs_bond_slot_from_conn(conn, &bond_slot);
	if (err) {
		LOG_DBG("Peer has no bonded slot; skipping ACS restore");
		return;
	}

	err = acs_persistent_export_key(acs_psa_key_id(bond_slot, ACS_PSA_SLOT_META), &meta,
					sizeof(meta), "restore ACS metadata");
	if (err) {
		return;
	}

	err = acs_crypto_current_key_lookup(acs_conn, meta.parent_key_id, &parent_key);
	if (err) {
		LOG_ERR("Stored ACS parent Key_ID 0x%04x has no runtime slot", meta.parent_key_id);
		acs_destroy_persistent_slot(bond_slot);
		return;
	}

	err = acs_persistent_export_key(acs_psa_key_id(bond_slot, ACS_PSA_SLOT_PARENT),
					parent_key->key, sizeof(parent_key->key),
					"restore ACS parent key");
	if (err) {
		acs_destroy_persistent_slot(bond_slot);
		return;
	}

	parent_key->psa_key_id = 0U;
	if (acs_crypto_import_current_key(parent_key) != 0) {
		LOG_ERR("Failed to import restored parent key into PSA");
		acs_crypto_reset(acs_conn);
		return;
	}

	acs_conn->restriction_map_id = meta.restriction_map_id;

	if (acs_crypto_rebind_key_desc_runtimes(acs_conn) != 0) {
		LOG_ERR("Failed to import restored key descriptor runtimes into PSA");
		acs_crypto_reset(acs_conn);
		return;
	}

	if (parent_key->psa_key_id != 0U) {
		acs_conn->status_flags |= BT_ACS_STATUS_SECURITY_ESTABLISHED;
	}

	LOG_INF("ACS parent key restored from PSA for bonded peer%s",
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
	struct bt_keys *keys;
	size_t bond_slot;
	char addr_str[BT_ADDR_LE_STR_LEN];

	keys = bt_keys_find_addr(id, peer);
	if (!keys) {
		keys = bt_keys_find_irk(id, peer);
	}
	if (!keys || acs_bond_slot_from_keys(keys, &bond_slot) != 0) {
		return;
	}

	acs_destroy_persistent_slot(bond_slot);
	bt_addr_le_to_str(peer, addr_str, sizeof(addr_str));
	LOG_INF("Stored ACS parent key deleted for peer %s (bond removed)", addr_str);
}

struct bt_conn_auth_info_cb acs_auth_info_cb = {
	.bond_deleted = acs_bond_deleted,
};

void acs_session_clear(const struct bt_conn *conn)
{
	size_t bond_slot;
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	err = acs_bond_slot_from_conn(conn, &bond_slot);
	if (err) {
		LOG_DBG("Peer has no bonded slot; no ACS persistent session to clear");
		return;
	}

	acs_destroy_persistent_slot(bond_slot);
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
	LOG_INF("ACS parent key erased for peer %s", addr_str);
}

static void acs_clear_all_cb(struct bt_keys *keys, void *user_data)
{
	struct acs_clear_all_ctx *ctx = user_data;
	size_t bond_slot;
	char addr_str[BT_ADDR_LE_STR_LEN];

	if (acs_bond_slot_from_keys(keys, &bond_slot) != 0 || bond_slot == ctx->keep_slot) {
		return;
	}

	acs_destroy_persistent_slot(bond_slot);
	bt_addr_le_to_str(&keys->addr, addr_str, sizeof(addr_str));
	LOG_INF("Stored ACS parent key erased for peer %s (invalidate all)", addr_str);
}

void acs_session_clear_all(const struct bt_conn *conn)
{
	struct acs_clear_all_ctx ctx = {
		.keep_slot = SIZE_MAX,
	};

	(void)acs_bond_slot_from_conn(conn, &ctx.keep_slot);
	bt_keys_foreach_type(BT_KEYS_ALL, acs_clear_all_cb, &ctx);
}

#endif /* CONFIG_BT_SETTINGS */
