/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include "acs_internal.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#define ACS_SETTINGS_ROOT "bt/acs"
#define ACS_PERSIST_VERSION 1U
#define ACS_SETTINGS_KEY_LEN 20U

struct acs_persist_record {
	uint8_t version;
	bt_addr_le_t peer;
	uint16_t active_map_id;
	uint16_t active_isc_id;
	uint16_t active_key_id;
	uint8_t status_flags;
	uint8_t mode;
	uint8_t client_nonce_fixed_len;
	uint8_t server_nonce_fixed_len;
	uint64_t tx_nonce_counter;
	uint64_t rx_nonce_counter;
	uint8_t parent_key_valid;
	uint8_t session_key_valid;
	uint8_t parent_key[CONFIG_BT_ACS_SESSION_KEY_SIZE];
	uint8_t session_key[CONFIG_BT_ACS_SESSION_KEY_SIZE];
	uint8_t client_nonce_fixed[ACS_CRYPTO_NONCE_FIXED_MAX_SIZE];
	uint8_t server_nonce_fixed[ACS_CRYPTO_NONCE_FIXED_MAX_SIZE];
};

struct acs_saved_record {
	bool used;
	struct acs_persist_record record;
};

static struct acs_saved_record acs_saved_records[CONFIG_BT_MAX_PAIRED];

static bool acs_persist_client_nonce_matches_wire(const struct acs_persist_record *rec,
						  const uint8_t *nonce, size_t nonce_len)
{
	if (!rec || !nonce || rec->client_nonce_fixed_len != nonce_len) {
		return false;
	}

	for (size_t i = 0U; i < nonce_len; i++) {
		if (rec->client_nonce_fixed[nonce_len - 1U - i] != nonce[i]) {
			return false;
		}
	}

	return true;
}

static void acs_persist_key_from_addr(const bt_addr_le_t *addr, char *key, size_t len)
{
	snprintk(key, len, "%02X%02X%02X%02X%02X%02X_%02X", addr->a.val[5], addr->a.val[4],
		 addr->a.val[3], addr->a.val[2], addr->a.val[1], addr->a.val[0], addr->type);
}

static struct acs_saved_record *acs_persist_find_slot(const bt_addr_le_t *addr)
{
	for (size_t i = 0U; i < ARRAY_SIZE(acs_saved_records); i++) {
		if (acs_saved_records[i].used &&
		    bt_addr_le_eq(&acs_saved_records[i].record.peer, addr)) {
			return &acs_saved_records[i];
		}
	}

	return NULL;
}

static struct acs_saved_record *acs_persist_get_or_alloc_slot(const bt_addr_le_t *addr)
{
	struct acs_saved_record *empty = NULL;

	for (size_t i = 0U; i < ARRAY_SIZE(acs_saved_records); i++) {
		if (acs_saved_records[i].used &&
		    bt_addr_le_eq(&acs_saved_records[i].record.peer, addr)) {
			return &acs_saved_records[i];
		}

		if (!acs_saved_records[i].used && empty == NULL) {
			empty = &acs_saved_records[i];
		}
	}

	if (empty) {
		memset(empty, 0, sizeof(*empty));
		empty->used = true;
		bt_addr_le_copy(&empty->record.peer, addr);
	}

	return empty;
}

static int acs_persist_build_record(const struct acs_conn_ctx *conn_ctx, struct acs_persist_record *rec)
{
	const bt_addr_le_t *peer;

	if (!conn_ctx || !conn_ctx->conn || !rec) {
		return -EINVAL;
	}

	peer = bt_conn_get_dst(conn_ctx->conn);
	if (!peer) {
		return -ENOTCONN;
	}

	memset(rec, 0, sizeof(*rec));
	rec->version = ACS_PERSIST_VERSION;
	bt_addr_le_copy(&rec->peer, peer);
	rec->active_map_id = conn_ctx->active_map_id;
	rec->active_isc_id = conn_ctx->crypto.active_isc_id;
	rec->active_key_id = conn_ctx->crypto.active_key_id;
	rec->status_flags = conn_ctx->status_flags;
	rec->mode = (uint8_t)conn_ctx->crypto.mode;
	rec->client_nonce_fixed_len = (uint8_t)conn_ctx->crypto.client_nonce_fixed_len;
	rec->server_nonce_fixed_len = (uint8_t)conn_ctx->crypto.server_nonce_fixed_len;
	rec->tx_nonce_counter = conn_ctx->crypto.tx_nonce_counter;
	rec->rx_nonce_counter = conn_ctx->crypto.rx_nonce_counter;
	rec->parent_key_valid = conn_ctx->kex.parent_key_valid ? 1U : 0U;
	rec->session_key_valid = conn_ctx->crypto.session_key_valid ? 1U : 0U;
	memcpy(rec->parent_key, conn_ctx->kex.parent_key, sizeof(rec->parent_key));
	memcpy(rec->session_key, conn_ctx->crypto.session_key, sizeof(rec->session_key));
	memcpy(rec->client_nonce_fixed, conn_ctx->crypto.client_nonce_fixed,
	       sizeof(rec->client_nonce_fixed));
	memcpy(rec->server_nonce_fixed, conn_ctx->crypto.server_nonce_fixed,
	       sizeof(rec->server_nonce_fixed));

	return 0;
}

static int acs_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	struct acs_persist_record rec;
	struct acs_saved_record *slot;
	const char *next;
	int nlen;
	ssize_t rd_len;

	nlen = settings_name_next(name, &next);
	if (nlen <= 0 || next != NULL) {
		return -ENOENT;
	}

	if (len != sizeof(rec)) {
		return -EINVAL;
	}

	rd_len = read_cb(cb_arg, &rec, sizeof(rec));
	if (rd_len < 0) {
		return (int)rd_len;
	}

	if ((size_t)rd_len != sizeof(rec) || rec.version != ACS_PERSIST_VERSION) {
		return -EINVAL;
	}

	slot = acs_persist_get_or_alloc_slot(&rec.peer);
	if (!slot) {
		return -ENOMEM;
	}

	slot->used = true;
	slot->record = rec;
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(bt_acs, ACS_SETTINGS_ROOT, NULL, acs_settings_set, NULL, NULL);

int acs_persist_save_conn(struct acs_conn_ctx *conn_ctx)
{
	struct acs_persist_record rec;
	struct acs_saved_record *slot;
	char key[sizeof(ACS_SETTINGS_ROOT) + 1U + ACS_SETTINGS_KEY_LEN];
	int err;
	char addr_str[BT_ADDR_LE_STR_LEN];

	if (!IS_ENABLED(CONFIG_BT_SETTINGS)) {
		return 0;
	}

	if (!conn_ctx || !conn_ctx->conn) {
		return -EINVAL;
	}

	if (!conn_ctx->kex.parent_key_valid && !conn_ctx->crypto.session_key_valid &&
	    conn_ctx->crypto.client_nonce_fixed_len == 0U &&
	    conn_ctx->crypto.server_nonce_fixed_len == 0U) {
		return 0;
	}

	err = acs_persist_build_record(conn_ctx, &rec);
	if (err) {
		return err;
	}

	slot = acs_persist_get_or_alloc_slot(&rec.peer);
	if (!slot) {
		return -ENOMEM;
	}

	snprintk(key, sizeof(key), "%s/", ACS_SETTINGS_ROOT);
	acs_persist_key_from_addr(&rec.peer, key + strlen(key), sizeof(key) - strlen(key));

	err = settings_save_one(key, &rec, sizeof(rec));
	if (err == 0) {
		slot->used = true;
		slot->record = rec;
		bt_addr_le_to_str(&rec.peer, addr_str, sizeof(addr_str));
		LOG_DBG("ACS persisted keys for %s", addr_str);
	}

	return err;
}

void acs_persist_restore_conn(struct acs_conn_ctx *conn_ctx)
{
	struct acs_saved_record *slot;
	const bt_addr_le_t *peer;
	char addr_str[BT_ADDR_LE_STR_LEN];

	if (!IS_ENABLED(CONFIG_BT_SETTINGS) || !conn_ctx || !conn_ctx->conn) {
		return;
	}

	peer = bt_conn_get_dst(conn_ctx->conn);
	if (!peer) {
		return;
	}

	slot = acs_persist_find_slot(peer);
	if (!slot) {
		return;
	}

	conn_ctx->active_map_id = slot->record.active_map_id;
	conn_ctx->status_flags = slot->record.status_flags;
	conn_ctx->crypto.mode = (enum acs_crypto_mode)slot->record.mode;
	conn_ctx->crypto.active_isc_id = slot->record.active_isc_id;
	conn_ctx->crypto.active_key_id = slot->record.active_key_id;
	conn_ctx->crypto.client_nonce_fixed_len = slot->record.client_nonce_fixed_len;
	conn_ctx->crypto.server_nonce_fixed_len = slot->record.server_nonce_fixed_len;
	conn_ctx->crypto.tx_nonce_counter = slot->record.tx_nonce_counter;
	conn_ctx->crypto.rx_nonce_counter = slot->record.rx_nonce_counter;
	conn_ctx->crypto.session_key_valid = slot->record.session_key_valid != 0U;
	memcpy(conn_ctx->crypto.session_key, slot->record.session_key,
	       sizeof(conn_ctx->crypto.session_key));
	memcpy(conn_ctx->crypto.client_nonce_fixed, slot->record.client_nonce_fixed,
	       sizeof(conn_ctx->crypto.client_nonce_fixed));
	memcpy(conn_ctx->crypto.server_nonce_fixed, slot->record.server_nonce_fixed,
	       sizeof(conn_ctx->crypto.server_nonce_fixed));

	conn_ctx->kex.parent_key_valid = slot->record.parent_key_valid != 0U;
	memcpy(conn_ctx->kex.parent_key, slot->record.parent_key, sizeof(conn_ctx->kex.parent_key));
	conn_ctx->kex.session_key_valid = slot->record.session_key_valid != 0U;
	memcpy(conn_ctx->kex.session_key, slot->record.session_key, sizeof(conn_ctx->kex.session_key));

	bt_addr_le_to_str(peer, addr_str, sizeof(addr_str));
	LOG_DBG("ACS restored persisted keys for %s", addr_str);
}

int acs_persist_delete_conn(struct bt_conn *conn)
{
	struct acs_saved_record *slot;
	const bt_addr_le_t *peer;
	char key[sizeof(ACS_SETTINGS_ROOT) + 1U + ACS_SETTINGS_KEY_LEN];
	int err = 0;
	char addr_str[BT_ADDR_LE_STR_LEN];

	if (!IS_ENABLED(CONFIG_BT_SETTINGS)) {
		return 0;
	}

	if (!conn) {
		return -EINVAL;
	}

	peer = bt_conn_get_dst(conn);
	if (!peer) {
		return -ENOTCONN;
	}

	slot = acs_persist_find_slot(peer);
	if (slot) {
		memset(slot, 0, sizeof(*slot));
	}

	snprintk(key, sizeof(key), "%s/", ACS_SETTINGS_ROOT);
	acs_persist_key_from_addr(peer, key + strlen(key), sizeof(key) - strlen(key));
	err = settings_delete(key);
	if (err == 0) {
		bt_addr_le_to_str(peer, addr_str, sizeof(addr_str));
		LOG_DBG("ACS deleted persisted keys for %s", addr_str);
	}

	return err;
}

int acs_persist_delete_all(void)
{
	int err = 0;

	if (!IS_ENABLED(CONFIG_BT_SETTINGS)) {
		return 0;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(acs_saved_records); i++) {
		struct acs_saved_record *slot = &acs_saved_records[i];
		char key[sizeof(ACS_SETTINGS_ROOT) + 1U + ACS_SETTINGS_KEY_LEN];

		if (!slot->used) {
			continue;
		}

		snprintk(key, sizeof(key), "%s/", ACS_SETTINGS_ROOT);
		acs_persist_key_from_addr(&slot->record.peer, key + strlen(key),
					  sizeof(key) - strlen(key));
		err = settings_delete(key);
		if (err != 0) {
			return err;
		}

		memset(slot, 0, sizeof(*slot));
	}

	return 0;
}

bool acs_persist_client_nonce_conflicts(struct bt_conn *exclude_conn, const uint8_t *nonce,
					size_t nonce_len)
{
	const bt_addr_le_t *exclude_peer = exclude_conn ? bt_conn_get_dst(exclude_conn) : NULL;

	for (size_t i = 0U; i < ARRAY_SIZE(acs_saved_records); i++) {
		const struct acs_saved_record *slot = &acs_saved_records[i];

		if (!slot->used) {
			continue;
		}

		if (exclude_peer && bt_addr_le_eq(&slot->record.peer, exclude_peer)) {
			continue;
		}

		if (acs_persist_client_nonce_matches_wire(&slot->record, nonce, nonce_len)) {
			return true;
		}
	}

	return false;
}
