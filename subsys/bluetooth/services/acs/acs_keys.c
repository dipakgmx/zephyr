/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/psa/key_ids.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <mbedtls/platform_util.h>
#include <psa/crypto.h>

#include "common/bt_str.h"
#include "host/settings.h"
#include "acs_internal.h"
#include "acs_key_desc.h"
#include "acs_keys.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Attempts at a server nonce prefix that no other security algorithm on the connection uses. */
#define ACS_SERVER_NONCE_ATTEMPTS 8U

void acs_keys_destroy(psa_key_id_t *key_id)
{
	psa_status_t status;

	if (key_id == NULL || *key_id == 0U) {
		return;
	}

	status = psa_destroy_key(*key_id);
	if (status != PSA_SUCCESS) {
		LOG_WRN("psa_destroy_key(%u) failed: %d", (unsigned int)*key_id, status);
	}

	*key_id = 0U;
}

int acs_keys_derive(psa_key_id_t secret, const struct bt_acs_kdf_params *kdf,
		    const psa_key_attributes_t *attrs, psa_key_id_t *key)
{
	psa_key_derivation_operation_t op = PSA_KEY_DERIVATION_OPERATION_INIT;
	uint8_t salt_be[ACS_KDF_SALT_SIZE];
	uint8_t info_be[ACS_KDF_INFO_SIZE];
	psa_status_t abort_status;
	psa_status_t status;

	/* KDF_Salt and KDF_Info are carried LSO first; HKDF takes them MSO first. */
	sys_memcpy_swap(salt_be, kdf->salt, kdf->salt_size);
	sys_memcpy_swap(info_be, kdf->info, kdf->info_size);

	status = psa_key_derivation_setup(&op, ACS_PSA_HKDF_ALG);
	if (status == PSA_SUCCESS) {
		status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_SALT, salt_be,
							kdf->salt_size);
	}
	if (status == PSA_SUCCESS) {
		status = psa_key_derivation_input_key(&op, PSA_KEY_DERIVATION_INPUT_SECRET, secret);
	}
	if (status == PSA_SUCCESS) {
		status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_INFO, info_be,
							kdf->info_size);
	}
	if (status == PSA_SUCCESS) {
		status = psa_key_derivation_output_key(attrs, &op, key);
	}

	abort_status = psa_key_derivation_abort(&op);
	if (status == PSA_SUCCESS && abort_status != PSA_SUCCESS) {
		acs_keys_destroy(key);
		status = abort_status;
	}

	if (status != PSA_SUCCESS) {
		LOG_ERR("HKDF key derivation failed: %d", status);
		return -EIO;
	}

	return 0;
}

/* Set the PSA policy of a security algorithm's key and cache its PSA algorithm. */
static int alg_key_attributes(struct acs_sec_alg *alg, psa_key_attributes_t *attrs)
{
	const struct bt_acs_key_desc_record *desc = alg->key_desc;
	psa_key_usage_t usage = PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT;

	switch (desc->type_id) {
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	case ACS_KEY_REC_AES_128_CCM:
		alg->psa_alg = PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, desc->aes.mac_size);
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                           \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case ACS_KEY_REC_AES_128_GCM:
	case ACS_KEY_REC_AES_128_GMAC:
		alg->psa_alg = PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_GCM, desc->aes.mac_size);
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	case ACS_KEY_REC_AES_128_CMAC:
		usage = PSA_KEY_USAGE_SIGN_MESSAGE | PSA_KEY_USAGE_VERIFY_MESSAGE;
		alg->psa_alg = PSA_ALG_CMAC;
		break;
#endif
	default:
		return -ENOTSUP;
	}

	psa_set_key_type(attrs, PSA_KEY_TYPE_AES);
	psa_set_key_bits(attrs, ACS_AES_KEY_BITS);
	psa_set_key_usage_flags(attrs, usage);
	psa_set_key_algorithm(attrs, alg->psa_alg);
	return 0;
}

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
/* Return true when another security algorithm on the connection uses alg's server prefix. */
static bool server_nonce_fixed_in_use(const struct acs_keys *keys,
				      const struct acs_sec_alg *alg)
{
	uint8_t size = acs_key_desc_nonce_fixed_size(alg->key_desc);

	ARRAY_FOR_EACH_PTR(keys->algs, other) {
		if (other != alg && other->server_nonce_set &&
		    acs_key_desc_nonce_fixed_size(other->key_desc) == size &&
		    memcmp(other->server_nonce_fixed, alg->server_nonce_fixed, size) == 0) {
			return true;
		}
	}

	return false;
}

/*
 * Generate the AC Server fixed nonce on first use. Records derived from the
 * same key material must use different prefixes.
 */
static int server_nonce_fixed_ensure(struct acs_keys *keys, struct acs_sec_alg *alg)
{
	uint8_t size = acs_key_desc_nonce_fixed_size(alg->key_desc);

	if (alg->server_nonce_set || size == 0U) {
		return 0;
	}

	for (unsigned int attempt = 0U; attempt < ACS_SERVER_NONCE_ATTEMPTS; attempt++) {
		psa_status_t status = psa_generate_random(alg->server_nonce_fixed, size);

		if (status != PSA_SUCCESS) {
			LOG_ERR("server nonce prefix generation failed: %d", status);
			return -EIO;
		}

		if (!server_nonce_fixed_in_use(keys, alg)) {
			alg->server_nonce_set = true;
			return 0;
		}
	}

	LOG_ERR("could not generate a unique server nonce prefix");
	return -EAGAIN;
}

/*
 * Return true when candidate equals a fixed nonce stored on any connection
 * (§4.4.3.18). The target's own client value is not compared, so it may be set
 * again.
 */
static bool client_nonce_fixed_in_use(const struct acs_sec_alg *target, const uint8_t *candidate,
				      uint8_t size)
{
	for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		struct bt_acs_conn *acs_conn = acs_conn_by_index(i);

		if (acs_conn->conn == NULL) {
			continue;
		}

		ARRAY_FOR_EACH_PTR(acs_conn->keys.algs, other) {
			if (other->key_desc == NULL ||
			    acs_key_desc_nonce_fixed_size(other->key_desc) != size) {
				continue;
			}
			if (other->server_nonce_set &&
			    memcmp(candidate, other->server_nonce_fixed, size) == 0) {
				return true;
			}
			if (other != target && other->client_nonce_set &&
			    memcmp(candidate, other->client_nonce_fixed, size) == 0) {
				return true;
			}
		}
	}

	return false;
}

int acs_keys_server_nonce_fixed(struct bt_acs_conn *acs_conn, uint16_t key_id, uint8_t *out)
{
	struct acs_sec_alg *alg = acs_keys_alg(acs_conn, key_id);
	int err;

	if (alg == NULL) {
		return -ENOENT;
	}

	err = server_nonce_fixed_ensure(&acs_conn->keys, alg);
	if (err) {
		return err;
	}

	sys_memcpy_swap(out, alg->server_nonce_fixed, acs_key_desc_nonce_fixed_size(alg->key_desc));
	return 0;
}

int acs_keys_set_client_nonce_fixed(struct bt_acs_conn *acs_conn, struct acs_sec_alg *alg,
				    const uint8_t *value)
{
	uint8_t size = acs_key_desc_nonce_fixed_size(alg->key_desc);
	uint8_t candidate[ACS_MAX_NONCE_PREFIX_SIZE];
	int err;

	/* The server prefix takes part in the uniqueness check, so fix it first. */
	err = server_nonce_fixed_ensure(&acs_conn->keys, alg);
	if (err) {
		return err;
	}

	sys_memcpy_swap(candidate, value, size);
	if (client_nonce_fixed_in_use(alg, candidate, size)) {
		return -EEXIST;
	}

	memcpy(alg->client_nonce_fixed, candidate, size);
	alg->client_nonce_set = true;
	return 0;
}
#endif /* CONFIG_BT_ACS_HAS_NONCE_FIXED */

/* Derive the key of one security algorithm from the ECDH key and restart its sequence numbers. */
static int alg_derive_key(struct acs_keys *keys, struct acs_sec_alg *alg,
			     const struct bt_acs_kdf_params *kdf)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	int err;

	err = alg_key_attributes(alg, &attrs);
	if (err) {
		return err;
	}

	err = acs_keys_derive(keys->ecdh, kdf, &attrs, &alg->psa_key_id);
	if (err) {
		return err;
	}

	alg->tx_nonce_counter = 0U;
	alg->rx_nonce_counter = 0U;

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	return server_nonce_fixed_ensure(keys, alg);
#else
	return 0;
#endif
}

/*
 * Copy an ECDH key inside the keystore, so its material never leaves it: to
 * persistent_id, or to a new volatile key when persistent_id is PSA_KEY_ID_NULL.
 * Type and size come from src; the copy keeps only the HKDF use and can never
 * be exported.
 */
static int ecdh_key_copy(psa_key_id_t src, psa_key_id_t persistent_id, psa_key_id_t *dst)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_DERIVE | PSA_KEY_USAGE_COPY);
	psa_set_key_algorithm(&attrs, ACS_PSA_HKDF_ALG);
	if (persistent_id != PSA_KEY_ID_NULL) {
		psa_set_key_id(&attrs, persistent_id);
	}

	status = psa_copy_key(src, &attrs, dst);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_copy_key 0x%08x failed: %d", (unsigned int)src, status);
		return -EIO;
	}

	return 0;
}

void acs_keys_reset(struct bt_acs_conn *acs_conn)
{
	struct acs_keys *keys = &acs_conn->keys;
	size_t i = 0;

	acs_keys_remove(acs_conn, ACS_KEY_ID_ECDH);
	mbedtls_platform_zeroize(keys, sizeof(*keys));

	STRUCT_SECTION_FOREACH(bt_acs_key_desc_record, rec) {
		if (acs_key_desc_is_algorithm_record(rec)) {
			__ASSERT_NO_MSG(i < ARRAY_SIZE(keys->algs));
			keys->algs[i++].key_desc = rec;
		}
	}
}

int acs_keys_set_ecdh(struct bt_acs_conn *acs_conn, psa_key_id_t src)
{
	acs_keys_remove(acs_conn, ACS_KEY_ID_ECDH);

	return ecdh_key_copy(src, PSA_KEY_ID_NULL, &acs_conn->keys.ecdh);
}

int acs_keys_derive_algs(struct bt_acs_conn *acs_conn, const struct bt_acs_kdf_params *kdf)
{
	struct acs_keys *keys = &acs_conn->keys;

	if (keys->ecdh == PSA_KEY_ID_NULL) {
		LOG_WRN("KDF key exchange needs an established ECDH key");
		return -EAGAIN;
	}

	acs_keys_remove(acs_conn, ACS_KEY_ID_KDF);

	/*
	 * Every record's Parent_Key_ID is the KDF key (Table 4.45), so each one
	 * takes the same HKDF output as its key material.
	 */
	ARRAY_FOR_EACH_PTR(keys->algs, alg) {
		int err;

		if (alg->key_desc == NULL) {
			continue;
		}

		err = alg_derive_key(keys, alg, kdf);
		if (err) {
			acs_keys_remove(acs_conn, ACS_KEY_ID_KDF);
			return err;
		}
	}

	keys->kdf = true;
	return 0;
}

void acs_keys_remove(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	struct acs_keys *keys = &acs_conn->keys;

	switch (key_id) {
	case ACS_KEY_ID_ECDH:
		acs_keys_destroy(&keys->ecdh);
		__fallthrough;
	case ACS_KEY_ID_KDF:
		keys->kdf = false;
		ARRAY_FOR_EACH_PTR(keys->algs, alg) {
			acs_keys_destroy(&alg->psa_key_id);
		}
		break;
	default: {
		struct acs_sec_alg *alg = acs_keys_alg(acs_conn, key_id);

		if (alg != NULL) {
			acs_keys_destroy(&alg->psa_key_id);
		}
		break;
	}
	}
}

bool acs_keys_installed(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	switch (key_id) {
	case ACS_KEY_ID_ECDH:
		return acs_conn->keys.ecdh != PSA_KEY_ID_NULL;
	case ACS_KEY_ID_KDF:
		return acs_conn->keys.kdf;
	default:
		return acs_sec_alg_ready(acs_keys_alg(acs_conn, key_id));
	}
}

uint8_t acs_keys_list(struct bt_acs_conn *acs_conn, uint16_t key_ids[ACS_KEY_COUNT])
{
	uint8_t count = 0;

	if (acs_conn->keys.ecdh != PSA_KEY_ID_NULL) {
		key_ids[count++] = ACS_KEY_ID_ECDH;
	}
	if (acs_conn->keys.kdf) {
		key_ids[count++] = ACS_KEY_ID_KDF;
	}
	ARRAY_FOR_EACH_PTR(acs_conn->keys.algs, alg) {
		if (acs_sec_alg_ready(alg)) {
			key_ids[count++] = acs_sec_alg_id(alg);
		}
	}

	return count;
}

struct acs_sec_alg *acs_keys_alg(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	ARRAY_FOR_EACH_PTR(acs_conn->keys.algs, alg) {
		if (alg->key_desc != NULL && alg->key_desc->key_id == key_id) {
			return alg;
		}
	}

	return NULL;
}

/*
 * Stored ECDH key of one peer. The PSA key ID is stored as allocated, so the
 * binding to the address survives any settings load order.
 */
struct acs_stored_key {
	bt_addr_le_t addr;
	uint8_t bt_id;
	psa_key_id_t key_id;
};

static struct acs_stored_key acs_stored[CONFIG_BT_MAX_PAIRED];

BUILD_ASSERT(CONFIG_BT_MAX_PAIRED <= ZEPHYR_PSA_BT_ACS_KEY_ID_RANGE_SIZE,
	     "Bluetooth ACS PSA key ID range is too small for the stored keys");

#define ACS_PST_KEY_ID_BEGIN ZEPHYR_PSA_BT_ACS_KEY_ID_RANGE_BEGIN
#define ACS_PST_KEY_ID_END   (ACS_PST_KEY_ID_BEGIN + CONFIG_BT_MAX_PAIRED - 1)

/* Allocator for the PSA key IDs reserved for ACS. */
static ATOMIC_DEFINE(acs_pst_keys, CONFIG_BT_MAX_PAIRED);

static inline bool pst_key_id_in_range(psa_key_id_t id)
{
	return IN_RANGE(id, ACS_PST_KEY_ID_BEGIN, ACS_PST_KEY_ID_END);
}

static psa_key_id_t pst_key_id_alloc(void)
{
	for (unsigned int i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
		if (!atomic_test_and_set_bit(acs_pst_keys, i)) {
			return ACS_PST_KEY_ID_BEGIN + (psa_key_id_t)i;
		}
	}
	return PSA_KEY_ID_NULL;
}

static void pst_key_id_free(psa_key_id_t id)
{
	if (pst_key_id_in_range(id)) {
		atomic_clear_bit(acs_pst_keys, id - ACS_PST_KEY_ID_BEGIN);
	}
}

static struct acs_stored_key *stored_key_find(const bt_addr_le_t *addr)
{
	ARRAY_FOR_EACH_PTR(acs_stored, s) {
		if (bt_addr_le_eq(&s->addr, addr)) {
			return s;
		}
	}

	return NULL;
}

static struct acs_stored_key *stored_key_alloc(const bt_addr_le_t *addr)
{
	struct acs_stored_key *s = stored_key_find(addr);

	if (s == NULL) {
		s = stored_key_find(BT_ADDR_LE_ANY);
		if (s != NULL) {
			bt_addr_le_copy(&s->addr, addr);
		}
	}

	return s;
}

/* Destroy the stored key and delete its settings entry. */
static void stored_key_erase(struct acs_stored_key *s)
{
	LOG_INF("ACS ECDH key erased for peer %s", bt_addr_le_str(&s->addr));

	if (s->key_id != PSA_KEY_ID_NULL) {
		psa_status_t status = psa_destroy_key(s->key_id);

		if (status != PSA_SUCCESS && status != PSA_ERROR_INVALID_HANDLE) {
			LOG_WRN("destroy persistent key 0x%08x failed: %d",
				(unsigned int)s->key_id, status);
		}
		pst_key_id_free(s->key_id);
	}

	(void)bt_settings_delete("acs", s->bt_id, &s->addr);
	memset(s, 0, sizeof(*s));
}

void acs_keys_save(struct bt_acs_conn *acs_conn)
{
	const bt_addr_le_t *addr = bt_conn_get_dst(acs_conn->conn);
	struct acs_stored_key *s;
	struct bt_conn_info info;
	psa_key_id_t stored;

	if (bt_addr_le_is_rpa(addr)) {
		return;
	}

	if (bt_conn_get_info(acs_conn->conn, &info) != 0) {
		LOG_ERR("Failed to get connection info for settings store");
		return;
	}

	s = stored_key_alloc(addr);
	if (s == NULL) {
		LOG_WRN("No free ACS stored-key slot for peer %s", bt_addr_le_str(addr));
		return;
	}

	s->bt_id = info.id;

	/* A peer keeps its PSA key ID when its key is replaced. */
	if (s->key_id == PSA_KEY_ID_NULL) {
		s->key_id = pst_key_id_alloc();
		if (s->key_id == PSA_KEY_ID_NULL) {
			LOG_ERR("No free ACS persistent key id for peer %s", bt_addr_le_str(addr));
			goto err;
		}
	}

	(void)psa_destroy_key(s->key_id);

	if (ecdh_key_copy(acs_conn->keys.ecdh, s->key_id, &stored) != 0 ||
	    bt_settings_store("acs", info.id, addr, &s->key_id, sizeof(s->key_id)) != 0) {
		LOG_ERR("Failed to store the ACS ECDH key for peer %s", bt_addr_le_str(addr));
		goto err;
	}

	LOG_DBG("Stored ACS ECDH key for peer %s", bt_addr_le_str(addr));
	return;

err:
	stored_key_erase(s);
}

void acs_keys_restore(struct bt_acs_conn *acs_conn)
{
	struct acs_stored_key *s = stored_key_find(bt_conn_get_dst(acs_conn->conn));

	if (s == NULL) {
		return;
	}

	if (acs_keys_set_ecdh(acs_conn, s->key_id) != 0) {
		stored_key_erase(s);
		return;
	}

	LOG_INF("ACS ECDH key restored for peer %s", bt_addr_le_str(&s->addr));
}

void acs_keys_forget(struct bt_acs_conn *acs_conn)
{
	struct acs_stored_key *s = stored_key_find(bt_conn_get_dst(acs_conn->conn));

	if (s != NULL) {
		stored_key_erase(s);
	}
}

void acs_keys_forget_all_except(struct bt_acs_conn *acs_conn)
{
	const bt_addr_le_t *keep = bt_conn_get_dst(acs_conn->conn);

	ARRAY_FOR_EACH_PTR(acs_stored, s) {
		if (!bt_addr_le_eq(&s->addr, BT_ADDR_LE_ANY) && !bt_addr_le_eq(&s->addr, keep)) {
			stored_key_erase(s);
		}
	}
}

static void acs_bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
	struct acs_stored_key *s = stored_key_find(peer);

	ARG_UNUSED(id);

	if (s != NULL) {
		stored_key_erase(s);
	}
}

static struct bt_conn_auth_info_cb acs_auth_info_cb = {
	.bond_deleted = acs_bond_deleted,
};

void acs_keys_init(void)
{
	bt_conn_auth_info_cb_register(&acs_auth_info_cb);
}

/* Check a stored PSA key ID before claiming it. */
static bool stored_key_id_valid(psa_key_id_t id)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	if (!pst_key_id_in_range(id) || atomic_test_bit(acs_pst_keys, id - ACS_PST_KEY_ID_BEGIN)) {
		return false;
	}

	/* A settings entry whose ITS key is gone is stale; if PSA is not ready yet, trust it. */
	status = psa_get_key_attributes(id, &attrs);
	psa_reset_key_attributes(&attrs);
	return status != PSA_ERROR_INVALID_HANDLE;
}

static int acs_settings_set(const char *name, size_t len_rd, settings_read_cb read_cb, void *cb_arg)
{
	struct acs_stored_key *s;
	uint8_t bt_id = BT_ID_DEFAULT;
	psa_key_id_t key_id;
	bt_addr_le_t addr;
	const char *next;

	if (name == NULL) {
		return -EINVAL;
	}

	if (bt_settings_decode_key(name, &addr) != 0) {
		LOG_ERR("ACS settings: unable to decode address from '%s'", name);
		return -EINVAL;
	}

	if (len_rd == 0U) {
		s = stored_key_find(&addr);
		if (s != NULL) {
			memset(s, 0, sizeof(*s));
		}
		return 0;
	}

	if (settings_name_next(name, &next) > 0 && next != NULL) {
		bt_id = (uint8_t)strtoul(next, NULL, 10);
	}

	/* An invalid entry is deleted without touching the PSA key it names. */
	if (read_cb(cb_arg, &key_id, sizeof(key_id)) != sizeof(key_id) ||
	    !stored_key_id_valid(key_id)) {
		LOG_WRN("ACS settings: invalid entry for peer %s, erasing", bt_addr_le_str(&addr));
		(void)bt_settings_delete("acs", bt_id, &addr);
		return 0;
	}

	s = stored_key_alloc(&addr);
	if (s == NULL) {
		LOG_WRN("ACS settings: no free slot for peer %s", bt_addr_le_str(&addr));
		return -ENOMEM;
	}

	s->bt_id = bt_id;
	s->key_id = key_id;
	atomic_set_bit(acs_pst_keys, key_id - ACS_PST_KEY_ID_BEGIN);

	return 0;
}

BT_SETTINGS_DEFINE(acs, "acs", acs_settings_set, NULL);
