/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <psa/crypto.h>
#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/logging/log.h>
#include <zephyr/psa/key_ids.h>

#include "common/bt_str.h"
#include "acs_internal.h"
#include "acs_crypto.h"
#include "acs_key_exchange.h"
#include "host/settings.h"
#include "acs_key_desc.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Stored ACS keys for one peer. PSA IDs remain stable across reboots. */
struct acs_pst_record {
	psa_key_id_t psa_id;
	psa_key_id_t derive_id;
} __packed;

/* In-memory copy of a peer's stored key record. */
struct acs_key_store_slot {
	bt_addr_le_t addr;
	uint8_t bt_id;
	struct acs_pst_record record;
};

static struct acs_key_store_slot acs_slots[CONFIG_BT_MAX_PAIRED];

/* Persisting one ECDH parent costs 2 PSA handles (operational + derive twin). */
#define ACS_PST_HANDLES_PER_PEER 2U
#define ACS_PST_KEY_COUNT        (CONFIG_BT_MAX_PAIRED * ACS_PST_HANDLES_PER_PEER)

BUILD_ASSERT(ACS_PST_KEY_COUNT <= ZEPHYR_PSA_BT_ACS_KEY_ID_RANGE_SIZE,
	     "Bluetooth ACS PSA key ID range is too small for the persistent key registry");

#define ACS_PST_KEY_ID_BEGIN ZEPHYR_PSA_BT_ACS_KEY_ID_RANGE_BEGIN
#define ACS_PST_KEY_ID_END   (ACS_PST_KEY_ID_BEGIN + ACS_PST_KEY_COUNT - 1)

/* Allocator for the PSA key IDs reserved for ACS. */
static ATOMIC_DEFINE(acs_pst_keys, ACS_PST_KEY_COUNT);

static inline bool acs_keyid_in_range(psa_key_id_t id)
{
	return IN_RANGE(id, ACS_PST_KEY_ID_BEGIN, ACS_PST_KEY_ID_END);
}

static psa_key_id_t acs_keyid_alloc(void)
{
	for (unsigned int i = 0; i < ACS_PST_KEY_COUNT; i++) {
		if (!atomic_test_and_set_bit(acs_pst_keys, i)) {
			return ACS_PST_KEY_ID_BEGIN + (psa_key_id_t)i;
		}
	}
	return PSA_KEY_ID_NULL;
}

static void acs_keyid_assign(psa_key_id_t id)
{
	if (acs_keyid_in_range(id)) {
		atomic_set_bit(acs_pst_keys, id - ACS_PST_KEY_ID_BEGIN);
	}
}

static void acs_keyid_free(psa_key_id_t id)
{
	if (acs_keyid_in_range(id)) {
		atomic_clear_bit(acs_pst_keys, id - ACS_PST_KEY_ID_BEGIN);
	}
}

static inline bool acs_slot_is_free(size_t idx)
{
	return bt_addr_le_eq(&acs_slots[idx].addr, BT_ADDR_LE_ANY);
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
	size_t free_slot = ARRAY_SIZE(acs_slots);

	ARRAY_FOR_EACH(acs_slots, i) {
		if (bt_addr_le_eq(&acs_slots[i].addr, addr)) {
			*slot = i;
			return 0;
		}
		if (free_slot == ARRAY_SIZE(acs_slots) && acs_slot_is_free(i)) {
			free_slot = i;
		}
	}

	if (free_slot < ARRAY_SIZE(acs_slots)) {
		bt_addr_le_copy(&acs_slots[free_slot].addr, addr);
		*slot = free_slot;
		return 0;
	}

	return -ENOMEM;
}

static void acs_slot_free_idx(size_t idx)
{
	memset(&acs_slots[idx], 0, sizeof(acs_slots[idx]));
}

/* Discard an invalid record without using its untrusted PSA IDs. */
static void acs_pst_erase_metadata(uint8_t bt_id, const bt_addr_le_t *addr)
{
	(void)bt_settings_delete("acs", bt_id, addr);
}

/* Remove a checked record and destroy its PSA keys. */
static void acs_key_store_destroy_slot(size_t slot)
{
	struct acs_key_store_slot *s = &acs_slots[slot];
	psa_key_id_t ids[] = {s->record.psa_id, s->record.derive_id};

	for (size_t k = 0; k < ARRAY_SIZE(ids); k++) {
		psa_status_t status;

		if (ids[k] == PSA_KEY_ID_NULL) {
			continue;
		}
		status = psa_destroy_key(ids[k]);
		if (status != PSA_SUCCESS && status != PSA_ERROR_INVALID_HANDLE) {
			LOG_WRN("destroy persistent key 0x%08x failed: %d", (unsigned int)ids[k],
				status);
		}
		acs_keyid_free(ids[k]);
	}

	(void)bt_settings_delete("acs", s->bt_id, &s->addr);
	acs_slot_free_idx(slot);
}

/* Check stored PSA IDs before assigning them to this record. */
static bool acs_pst_key_id_valid(psa_key_id_t id, psa_key_id_t *seen, size_t *n)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	if (!acs_keyid_in_range(id)) {
		return false;
	}
	if (atomic_test_bit(acs_pst_keys, id - ACS_PST_KEY_ID_BEGIN)) {
		return false;
	}
	for (size_t s = 0; s < *n; s++) {
		if (seen[s] == id) {
			return false;
		}
	}

	status = psa_get_key_attributes(id, &attrs);
	psa_reset_key_attributes(&attrs);
	if (status == PSA_ERROR_INVALID_HANDLE) {
		/* The NVS record survived but the ITS key did not (desync). */
		return false;
	}
	/* If PSA is not ready, key existence is checked when the record is restored. */

	seen[(*n)++] = id;
	return true;
}

static bool acs_pst_record_valid(const struct acs_pst_record *rec)
{
	psa_key_id_t seen[ACS_PST_HANDLES_PER_PEER];
	size_t n = 0;

	if (rec->psa_id == PSA_KEY_ID_NULL || rec->derive_id == PSA_KEY_ID_NULL) {
		return false;
	}
	if (!acs_pst_key_id_valid(rec->psa_id, seen, &n)) {
		return false;
	}
	return acs_pst_key_id_valid(rec->derive_id, seen, &n);
}

static void acs_pst_record_assign_ids(const struct acs_pst_record *rec)
{
	acs_keyid_assign(rec->psa_id);
	acs_keyid_assign(rec->derive_id);
}

/* Find the ECDH parent key stored across connections. */
static int acs_key_store_find_parent(struct bt_acs_conn *acs_conn,
				     struct bt_acs_key_desc_runtime **exchange_key)
{
	struct bt_acs_key_desc_runtime *ecdh_key;

	if (acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_ECDH, &ecdh_key) != 0 ||
	    ecdh_key->psa_key_id == 0U) {
		return -ENOENT;
	}

	*exchange_key = ecdh_key;
	return 0;
}

void acs_key_store(struct bt_acs_conn *acs_conn)
{
	struct bt_acs_key_desc_runtime *parent_key;
	struct acs_key_store_slot *s;
	struct bt_conn_info info;
	const bt_addr_le_t *addr;
	struct bt_conn *conn;
	size_t slot;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL && acs_conn->conn != NULL);

	conn = acs_conn->conn;
	addr = bt_conn_get_dst(conn);

	if (bt_addr_le_is_rpa(addr)) {
		return;
	}

	err = acs_key_store_find_parent(acs_conn, &parent_key);
	if (err) {
		LOG_ERR("No exchanged runtime key available to persist");
		return;
	}

	/* Store only keys that do not carry nonce state (§3.6.4.3). */
	__ASSERT(!acs_key_desc_has_nonce_record(parent_key->key_desc),
		 "persisting a nonce-bearing key requires persisting its nonce/counter state");

	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info for settings store");
		return;
	}

	err = acs_slot_alloc(addr, &slot);
	if (err) {
		LOG_WRN("No free ACS key-store slot for peer %s", bt_addr_le_str(addr));
		return;
	}
	s = &acs_slots[slot];

	/* Reuse the peer's PSA IDs when replacing its key material. */
	if (s->record.psa_id == PSA_KEY_ID_NULL) {
		s->record.psa_id = acs_keyid_alloc();
		s->record.derive_id = acs_keyid_alloc();
	}

	s->bt_id = info.id;

	if (s->record.psa_id == PSA_KEY_ID_NULL || s->record.derive_id == PSA_KEY_ID_NULL) {
		LOG_ERR("No free ACS persistent key id for peer %s", bt_addr_le_str(addr));
		goto err;
	}

	/*
	 * Copied in-keystore, so the raw key material never enters a plaintext
	 * buffer.
	 */
	err = acs_crypto_copy_key_to_persistent(parent_key, s->record.psa_id, s->record.derive_id);
	if (err) {
		LOG_ERR("Failed to copy parent key to persistent store: %d", err);
		goto err;
	}

	err = bt_settings_store("acs", info.id, addr, &s->record, sizeof(s->record));
	if (err) {
		LOG_ERR("Failed to store ACS record in settings: %d", err);
		goto err;
	}

	LOG_DBG("Stored ACS ECDH parent for peer %s in slot %zu", bt_addr_le_str(addr), slot);
	return;

err:
	acs_key_store_destroy_slot(slot);
}

void acs_key_restore(struct bt_acs_conn *acs_conn)
{
	struct bt_acs_key_desc_runtime *runtime;
	struct acs_key_store_slot *s;
	const bt_addr_le_t *addr;
	struct bt_conn *conn;
	size_t slot;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL && acs_conn->conn != NULL);

	conn = acs_conn->conn;
	addr = bt_conn_get_dst(conn);

	if (acs_slot_from_addr(addr, &slot) != 0) {
		LOG_DBG("No stored ACS parent key for peer %s", bt_addr_le_str(addr));
		return;
	}
	s = &acs_slots[slot];

	err = acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_ECDH, &runtime);
	if (err) {
		LOG_ERR("ECDH key descriptor has no runtime slot");
		acs_key_store_destroy_slot(slot);
		return;
	}

	err = acs_crypto_copy_persistent_key_to_runtime(s->record.psa_id, s->record.derive_id,
							runtime);
	if (err) {
		LOG_ERR("Failed to restore ECDH parent from persistent store: %d", err);
		acs_key_store_destroy_slot(slot);
		return;
	}

	/* Restore only the parent; key exchange creates fresh record keys and nonces. */
	atomic_set_bit_to(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED, false);

	LOG_INF("ACS parent key restored for peer %s (security not established)",
		bt_addr_le_str(addr));
}

static void acs_bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
	size_t slot;

	if (acs_slot_from_addr(peer, &slot) != 0) {
		return;
	}

	acs_key_store_destroy_slot(slot);
	LOG_INF("ACS parent key deleted for peer %s (bond removed)", bt_addr_le_str(peer));
}

static struct bt_conn_auth_info_cb acs_auth_info_cb = {
	.bond_deleted = acs_bond_deleted,
};

void acs_key_store_init(void)
{
	bt_conn_auth_info_cb_register(&acs_auth_info_cb);
}

void acs_key_store_clear(const struct bt_conn *conn)
{
	const bt_addr_le_t *addr = bt_conn_get_dst(conn);
	size_t slot;

	if (acs_slot_from_addr(addr, &slot) != 0) {
		LOG_DBG("No stored ACS parent key to clear for peer %s", bt_addr_le_str(addr));
		return;
	}

	acs_key_store_destroy_slot(slot);
	LOG_INF("ACS parent key erased for peer %s", bt_addr_le_str(addr));
}

void acs_key_store_clear_all_except(const struct bt_conn *conn)
{
	const bt_addr_le_t *keep_addr = bt_conn_get_dst(conn);

	for (size_t i = 0; i < ARRAY_SIZE(acs_slots); i++) {
		if (acs_slot_is_free(i) || bt_addr_le_eq(&acs_slots[i].addr, keep_addr)) {
			continue;
		}

		LOG_INF("ACS parent key erased for peer %s (invalidate all)",
			bt_addr_le_str(&acs_slots[i].addr));

		acs_key_store_destroy_slot(i);
	}
}

static int acs_settings_set(const char *name, size_t len_rd, settings_read_cb read_cb, void *cb_arg)
{
	bt_addr_le_t addr;
	struct acs_pst_record record;
	uint8_t bt_id = BT_ID_DEFAULT;
	ssize_t len;
	size_t slot;
	int err;

	if (name == NULL) {
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

	{
		const char *next;

		if (settings_name_next(name, &next) > 0 && next != NULL) {
			bt_id = (uint8_t)strtoul(next, NULL, 10);
		}
	}

	len = read_cb(cb_arg, &record, sizeof(record));

	/* Invalid records are removed without trusting their PSA IDs. */
	if (len != sizeof(record)) {
		LOG_WRN("ACS settings: stale record for peer %s (len %zd), erasing",
			bt_addr_le_str(&addr), len);
		acs_pst_erase_metadata(bt_id, &addr);
		return 0;
	}

	if (!acs_pst_record_valid(&record)) {
		LOG_WRN("ACS settings: invalid key ids for peer %s, erasing",
			bt_addr_le_str(&addr));
		acs_pst_erase_metadata(bt_id, &addr);
		return 0;
	}

	err = acs_slot_alloc(&addr, &slot);
	if (err) {
		LOG_WRN("ACS settings: no free slot for peer %s", bt_addr_le_str(&addr));
		return -ENOMEM;
	}

	acs_slots[slot].bt_id = bt_id;
	acs_slots[slot].record = record;
	acs_pst_record_assign_ids(&record);

	LOG_DBG("ACS settings: restored ECDH parent for peer %s in slot %zu", bt_addr_le_str(&addr),
		slot);

	return 0;
}

BT_SETTINGS_DEFINE(acs, "acs", acs_settings_set, NULL);
