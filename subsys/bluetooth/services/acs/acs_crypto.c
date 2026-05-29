/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/random/random.h>
#include <zephyr/bluetooth/services/acs.h>

#include <mbedtls/platform_util.h>
#include <psa/crypto.h>

#include "acs_internal.h"
#include "acs_isc.h"
#include "acs_key_desc.h"
#include "acs_crypto.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA384) || IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA384_WITH_INFO)
#define ACS_PSA_HKDF_ALG PSA_ALG_HKDF(PSA_ALG_SHA_384)
#elif IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA512) ||                                                 \
	IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA512_WITH_INFO)
#define ACS_PSA_HKDF_ALG PSA_ALG_HKDF(PSA_ALG_SHA_512)
#else
#define ACS_PSA_HKDF_ALG PSA_ALG_HKDF(PSA_ALG_SHA_256)
#endif

BUILD_ASSERT(!(IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC) &&
	       (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||
		IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) ||
		IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM))),
	     "AES-CMAC cannot coexist with AEAD algorithms");

int acs_crypto_current_key_id_from_key_desc(const struct bt_acs_key_desc_record *rec,
					    uint16_t *current_key_id)
{
	const struct bt_acs_key_desc_record *parent;

	if (!rec || !current_key_id) {
		LOG_DBG("current key descriptor resolution called with invalid arguments");
		__ASSERT_NO_MSG(rec != NULL);
		__ASSERT_NO_MSG(current_key_id != NULL);
		return -EINVAL;
	}

	if (!acs_key_desc_is_algorithm_record(rec)) {
		*current_key_id = rec->key_id;
		return 0;
	}

	/* Algorithm records always point directly at an exchange key (1 hop). */
	parent = acs_key_desc_lookup(acs_key_desc_parent_key_id(rec));
	if (parent && !acs_key_desc_is_algorithm_record(parent)) {
		*current_key_id = parent->key_id;
		return 0;
	}

	LOG_ERR("Unable to resolve current key from key descriptor relation");
	return -ENOENT;
}

void acs_crypto_init_slots(struct bt_acs_conn *acs_conn)
{
	size_t slot = 0;

	__ASSERT_NO_MSG(acs_conn != NULL);

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	acs_conn->crypto.key_runtimes[slot++].key_desc = acs_key_desc_lookup(ACS_KEY_ID_ECDH);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	acs_conn->crypto.key_runtimes[slot++].key_desc = acs_key_desc_lookup(ACS_KEY_ID_KDF);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	acs_conn->crypto.key_runtimes[slot++].key_desc = acs_key_desc_lookup(ACS_KEY_ID_OOB);
#endif

	__ASSERT_NO_MSG(slot <= ACS_KEY_ID_COUNT);

	slot = ACS_KEY_ID_COUNT;

	STRUCT_SECTION_FOREACH(bt_acs_key_desc_record, rec) {
		struct bt_acs_key_desc_runtime *runtime;
		int err;

		if (!acs_key_desc_has_nonce_record(rec)) {
			continue;
		}

		__ASSERT_NO_MSG(slot < ACS_KEY_RUNTIME_COUNT);
		runtime = &acs_conn->crypto.key_runtimes[slot++];
		runtime->key_desc = rec;
		err = acs_crypto_current_key_id_from_key_desc(rec, &runtime->current_key_id);
		if (err != 0) {
			LOG_WRN("Unable to resolve current key for descriptor Key_ID 0x%04x",
				rec->key_id);
			runtime->current_key_id = 0U;
		}
	}
}

void acs_crypto_reset(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);

	acs_crypto_destroy_connection_record_keys(acs_conn);
	acs_crypto_destroy_exchange_keys(acs_conn);
	memset(&acs_conn->crypto, 0, sizeof(acs_conn->crypto));
	acs_crypto_init_slots(acs_conn);
}

int acs_crypto_key_runtime_lookup(const struct bt_acs_conn *acs_conn, uint16_t key_id,
				  struct bt_acs_key_desc_runtime **key_runtime)
{
	if (!acs_conn || !key_runtime) {
		LOG_DBG("key runtime lookup called with invalid arguments");
		__ASSERT_NO_MSG(acs_conn != NULL);
		__ASSERT_NO_MSG(key_runtime != NULL);
		return -EINVAL;
	}

	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.key_runtimes); i++) {
		if (acs_key_desc_runtime_key_id(&acs_conn->crypto.key_runtimes[i]) == key_id) {
			*key_runtime =
				(struct bt_acs_key_desc_runtime *)&acs_conn->crypto.key_runtimes[i];
			return 0;
		}
	}

	*key_runtime = NULL;
	LOG_ERR("No runtime slot reserved for Key_ID 0x%04x", key_id);
	return -ENOENT;
}

int acs_crypto_current_key_from_isc(struct bt_acs_conn *acs_conn, uint16_t isc_id,
				    struct bt_acs_key_desc_runtime **current_key)
{
	const struct bt_acs_isc_record *isc = acs_isc_lookup(isc_id);
	const struct bt_acs_key_desc_record *desc;
	uint16_t current_key_id;
	int err;

	if (!acs_conn || !current_key) {
		LOG_DBG("current key resolution from ISC called with invalid arguments");
		__ASSERT_NO_MSG(acs_conn != NULL);
		__ASSERT_NO_MSG(current_key != NULL);
		return -EINVAL;
	}

	if (!isc) {
		*current_key = NULL;
		return -ENOENT;
	}

	desc = acs_key_desc_lookup(isc->key_id);
	if (!desc) {
		LOG_ERR("ISC 0x%04x references unknown key descriptor 0x%04x", isc_id, isc->key_id);
		*current_key = NULL;
		return -ENOENT;
	}

	err = acs_crypto_current_key_id_from_key_desc(desc, &current_key_id);
	if (err) {
		*current_key = NULL;
		return err;
	}

	return acs_crypto_key_runtime_lookup(acs_conn, current_key_id, current_key);
}

int acs_crypto_get_server_nonce_fixed(struct bt_acs_conn *acs_conn, uint16_t key_id,
				      uint8_t *nonce_buf, size_t len)
{
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	struct bt_acs_key_desc_runtime *key_desc_runtime;
	uint8_t fixed_size;
	int err;

	if (!acs_conn || !nonce_buf) {
		LOG_ERR("server nonce fixed lookup called with invalid arguments");
		__ASSERT_NO_MSG(acs_conn != NULL);
		__ASSERT_NO_MSG(nonce_buf != NULL);
		return -EINVAL;
	}

	err = acs_crypto_key_runtime_lookup(acs_conn, key_id, &key_desc_runtime);
	if (err) {
		return err;
	}

	fixed_size = acs_key_desc_nonce_fixed_size(key_desc_runtime->key_desc);
	if (fixed_size == 0U) {
		return -ENOTSUP;
	}

	if (fixed_size > sizeof(key_desc_runtime->server_nonce_fixed)) {
		return -EOVERFLOW;
	}

	{
		bool nonce_unset = true;

		for (uint8_t i = 0U; i < fixed_size; i++) {
			if (key_desc_runtime->server_nonce_fixed[i] != 0U) {
				nonce_unset = false;
				break;
			}
		}
		if (nonce_unset) {
			sys_rand_get(key_desc_runtime->server_nonce_fixed, fixed_size);
			sys_mem_swap(key_desc_runtime->server_nonce_fixed, fixed_size);
		}
	}

	memcpy(nonce_buf, key_desc_runtime->server_nonce_fixed, MIN(len, (size_t)fixed_size));
	sys_mem_swap(nonce_buf, MIN(len, (size_t)fixed_size));
	return 0;
#else
	ARG_UNUSED(acs_conn);
	ARG_UNUSED(key_id);
	ARG_UNUSED(nonce_buf);
	ARG_UNUSED(len);
	return -ENOTSUP;
#endif
}

void acs_psa_destroy_key(psa_key_id_t *key_id)
{
	psa_status_t status;

	if (!key_id || *key_id == 0U) {
		return;
	}

	status = psa_destroy_key(*key_id);
	if (status != PSA_SUCCESS) {
		LOG_WRN("psa_destroy_key(%u) failed: %d", (unsigned int)*key_id, status);
	}

	*key_id = 0U;
}

/*
 * Apply the exchange-key PSA policy (type, algorithm, usage) for the build's
 * configured data-protection algorithm.  Shared by the import and the
 * persistence copy paths so the copy target can never drift from the imported
 * key's policy - psa_copy_key requires a common permitted algorithm.
 */
static void acs_crypto_set_exchange_key_policy(psa_key_attributes_t *attrs, psa_key_usage_t usage)
{
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	psa_set_key_usage_flags(attrs,
				PSA_KEY_USAGE_SIGN_MESSAGE | PSA_KEY_USAGE_VERIFY_MESSAGE | usage);
	psa_set_key_algorithm(attrs, PSA_ALG_CMAC);
#elif IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	psa_set_key_usage_flags(attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT | usage);
	psa_set_key_algorithm(
		attrs, PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, CONFIG_BT_ACS_CCM_MAC_SIZE));
#elif IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	psa_set_key_usage_flags(attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT | usage);
	psa_set_key_algorithm(attrs, PSA_ALG_AEAD_WITH_SHORTENED_TAG(
					     PSA_ALG_GCM, PSA_AEAD_TAG_LENGTH(PSA_KEY_TYPE_AES, 128,
									      PSA_ALG_GCM)));
#else
	psa_set_key_usage_flags(attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT | usage);
	psa_set_key_algorithm(attrs, PSA_ALG_AEAD_WITH_SHORTENED_TAG(
					     PSA_ALG_GCM, PSA_AEAD_TAG_LENGTH(PSA_KEY_TYPE_AES, 128,
									      PSA_ALG_GCM)));
#endif
	psa_set_key_type(attrs, PSA_KEY_TYPE_AES);
}

static void acs_crypto_set_exchange_derive_policy(psa_key_attributes_t *attrs,
						  psa_key_usage_t usage)
{
	psa_set_key_usage_flags(attrs, PSA_KEY_USAGE_DERIVE | PSA_KEY_USAGE_EXPORT |
					       PSA_KEY_USAGE_COPY | usage);
	psa_set_key_algorithm(attrs, ACS_PSA_HKDF_ALG);
	psa_set_key_type(attrs, PSA_KEY_TYPE_DERIVE);
}

static int acs_crypto_import_exchange_derive_key(struct bt_acs_key_desc_runtime *key_runtime,
						 const uint8_t *key_material, size_t key_len)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	acs_crypto_set_exchange_derive_policy(&attrs, 0U);
	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);
	psa_set_key_bits(&attrs, key_len * BITS_PER_BYTE);

	status = psa_import_key(&attrs, key_material, key_len, &key_runtime->derive_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Derive-key import failed: %d", status);
		return -EIO;
	}

	return 0;
}

int acs_crypto_import_exchange_key(struct bt_acs_key_desc_runtime *key_runtime,
				   const uint8_t *key_material, size_t key_len)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;
	int err;

	acs_crypto_set_exchange_key_policy(&attrs, PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_COPY);
	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);
	psa_set_key_bits(&attrs, key_len * BITS_PER_BYTE);

	status = psa_import_key(&attrs, key_material, key_len, &key_runtime->psa_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Key import failed: %d", status);
		return -EIO;
	}

	err = acs_crypto_import_exchange_derive_key(key_runtime, key_material, key_len);
	if (err != 0) {
		acs_crypto_destroy_key(key_runtime);
		return err;
	}

	return 0;
}

int acs_crypto_output_exchange_key(struct bt_acs_key_desc_runtime *key_runtime,
				   psa_key_derivation_operation_t *op, size_t key_len)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	if (!key_runtime || !op) {
		return -EINVAL;
	}

	if (key_runtime->psa_key_id != 0U) {
		acs_psa_destroy_key(&key_runtime->psa_key_id);
	}

	acs_crypto_set_exchange_key_policy(&attrs, PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_COPY);
	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);
	psa_set_key_bits(&attrs, key_len * BITS_PER_BYTE);

	status = psa_key_derivation_output_key(&attrs, op, &key_runtime->psa_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Derived exchange key output failed: %d", status);
		return -EIO;
	}

	return 0;
}

int acs_crypto_output_exchange_derive_key(struct bt_acs_key_desc_runtime *key_runtime,
					  psa_key_derivation_operation_t *op, size_t key_len)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	if (!key_runtime || !op) {
		return -EINVAL;
	}

	if (key_runtime->derive_key_id != 0U) {
		acs_psa_destroy_key(&key_runtime->derive_key_id);
	}

	acs_crypto_set_exchange_derive_policy(&attrs, 0U);
	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);
	psa_set_key_bits(&attrs, key_len * BITS_PER_BYTE);

	status = psa_key_derivation_output_key(&attrs, op, &key_runtime->derive_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Derived exchange key output failed: %d", status);
		return -EIO;
	}

	return 0;
}

int acs_crypto_copy_key_to_persistent(const struct bt_acs_key_desc_runtime *parent,
				      psa_key_id_t dst_id, psa_key_id_t dst_derive_id)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t out_id;
	psa_status_t status;

	if (!parent || parent->psa_key_id == 0U || parent->derive_key_id == 0U) {
		return -EINVAL;
	}

	acs_crypto_set_exchange_key_policy(&attrs, PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_COPY);
	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_PERSISTENT);
	psa_set_key_id(&attrs, dst_id);
	psa_set_key_bits(&attrs, CONFIG_BT_ACS_SESSION_KEY_SIZE * BITS_PER_BYTE);

	status = psa_copy_key(parent->psa_key_id, &attrs, &out_id);
	if (status == PSA_ERROR_ALREADY_EXISTS) {
		status = psa_destroy_key(dst_id);
		if (status != PSA_SUCCESS) {
			LOG_ERR("destroy existing persistent exchange key 0x%08x failed: %d",
				(unsigned int)dst_id, status);
			return -EIO;
		}
		status = psa_copy_key(parent->psa_key_id, &attrs, &out_id);
	}
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_copy_key for persistent exchange key 0x%08x failed: %d",
			(unsigned int)dst_id, status);
		return -EIO;
	}

	psa_reset_key_attributes(&attrs);
	acs_crypto_set_exchange_derive_policy(&attrs, PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_COPY);
	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_PERSISTENT);
	psa_set_key_id(&attrs, dst_derive_id);
	psa_set_key_bits(&attrs, CONFIG_BT_ACS_SESSION_KEY_SIZE * BITS_PER_BYTE);

	status = psa_copy_key(parent->derive_key_id, &attrs, &out_id);
	if (status == PSA_ERROR_ALREADY_EXISTS) {
		status = psa_destroy_key(dst_derive_id);
		if (status != PSA_SUCCESS) {
			LOG_ERR("destroy existing persistent derive key 0x%08x failed: %d",
				(unsigned int)dst_derive_id, status);
			return -EIO;
		}
		status = psa_copy_key(parent->derive_key_id, &attrs, &out_id);
	}
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_copy_key for persistent derive key 0x%08x failed: %d",
			(unsigned int)dst_derive_id, status);
		return -EIO;
	}

	return 0;
}

int acs_crypto_copy_persistent_key_to_runtime(psa_key_id_t src_id, psa_key_id_t src_derive_id,
					      struct bt_acs_key_desc_runtime *parent)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	if (!parent) {
		return -EINVAL;
	}

	acs_crypto_set_exchange_key_policy(&attrs, PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_COPY);
	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);
	psa_set_key_bits(&attrs, CONFIG_BT_ACS_SESSION_KEY_SIZE * BITS_PER_BYTE);

	acs_crypto_destroy_key(parent);

	status = psa_copy_key(src_id, &attrs, &parent->psa_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_copy_key from persistent 0x%08x failed: %d", (unsigned int)src_id,
			status);
		parent->psa_key_id = 0U;
		return -EIO;
	}

	psa_reset_key_attributes(&attrs);
	acs_crypto_set_exchange_derive_policy(&attrs, PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_COPY);
	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);
	psa_set_key_bits(&attrs, CONFIG_BT_ACS_SESSION_KEY_SIZE * BITS_PER_BYTE);

	status = psa_copy_key(src_derive_id, &attrs, &parent->derive_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_copy_key from persistent derive 0x%08x failed: %d",
			(unsigned int)src_derive_id, status);
		parent->derive_key_id = 0U;
		acs_crypto_destroy_key(parent);
		return -EIO;
	}

	return 0;
}

void acs_crypto_destroy_key(struct bt_acs_key_desc_runtime *key_runtime)
{
	if (key_runtime) {
		acs_psa_destroy_key(&key_runtime->psa_key_id);
		acs_psa_destroy_key(&key_runtime->derive_key_id);
	}
}

void acs_crypto_destroy_exchange_keys(struct bt_acs_conn *acs_conn)
{
	for (size_t i = 0; i < ACS_KEY_ID_COUNT; i++) {
		acs_crypto_destroy_key(&acs_conn->crypto.key_runtimes[i]);
	}
}

void acs_crypto_release_exchange_keys(struct bt_acs_conn *acs_conn)
{
	for (size_t i = 0; i < ACS_KEY_ID_COUNT; i++) {
		acs_conn->crypto.key_runtimes[i].psa_key_id = 0U;
		acs_conn->crypto.key_runtimes[i].derive_key_id = 0U;
	}
}

/*
 * Apply the record-key PSA policy (type, algorithm, usage) for a key
 * descriptor's configured algorithm.  Shared by the import and the copy-based
 * rebind paths so the copy target matches the imported policy exactly.
 */
static int acs_crypto_set_record_key_policy(psa_key_attributes_t *attrs,
					    const struct bt_acs_key_desc_runtime *key_desc_runtime,
					    psa_key_usage_t usage)
{
	switch (key_desc_runtime->key_desc->type_id) {
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	case ACS_KEY_REC_AES_128_CCM:
		psa_set_key_usage_flags(attrs,
					PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT | usage);
		psa_set_key_algorithm(
			attrs, PSA_ALG_AEAD_WITH_SHORTENED_TAG(
				       PSA_ALG_CCM, key_desc_runtime->key_desc->aes.mac_size));
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                           \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case ACS_KEY_REC_AES_128_GCM:
	case ACS_KEY_REC_AES_128_GMAC:
		psa_set_key_usage_flags(attrs,
					PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT | usage);
		psa_set_key_algorithm(
			attrs, PSA_ALG_AEAD_WITH_SHORTENED_TAG(
				       PSA_ALG_GCM, key_desc_runtime->key_desc->aes.mac_size));
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	case ACS_KEY_REC_AES_128_CMAC:
		psa_set_key_usage_flags(attrs, PSA_KEY_USAGE_SIGN_MESSAGE |
						       PSA_KEY_USAGE_VERIFY_MESSAGE | usage);
		psa_set_key_algorithm(attrs, PSA_ALG_CMAC);
		break;
#endif
	default:
		return -ENOTSUP;
	}

	psa_set_key_type(attrs, PSA_KEY_TYPE_AES);
	psa_set_key_bits(attrs, CONFIG_BT_ACS_SESSION_KEY_SIZE * BITS_PER_BYTE);
	return 0;
}

int acs_crypto_import_record_key(struct bt_acs_key_desc_runtime *key_desc_runtime,
				 const uint8_t *key_material, size_t key_len)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;
	int err;

	__ASSERT_NO_MSG(key_desc_runtime != NULL);
	__ASSERT_NO_MSG(key_desc_runtime->key_desc != NULL);
	__ASSERT_NO_MSG(key_material != NULL);

	err = acs_crypto_set_record_key_policy(&attrs, key_desc_runtime, 0U);
	if (err) {
		return err;
	}
	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);

	status = psa_import_key(&attrs, key_material, key_len, &key_desc_runtime->psa_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("key descriptor runtime import failed for Key_ID 0x%04x: %d",
			acs_key_desc_runtime_key_id(key_desc_runtime), status);
		return -EIO;
	}

	return 0;
}

void acs_crypto_destroy_connection_record_keys(struct bt_acs_conn *acs_conn)
{
	for (size_t i = ACS_KEY_ID_COUNT; i < ACS_KEY_RUNTIME_COUNT; i++) {
		acs_crypto_destroy_key(&acs_conn->crypto.key_runtimes[i]);
	}
}

/*
 * Invalidate per-session nonce state on a record whose key has just become
 * unusable.  The record is not expected to encrypt/decrypt again until a
 * successful bind re-runs acs_derive_nonce_state() - so plain zero is fine
 * here, no need for the EVEN_ODD-aware initial state.
 */
static void acs_clear_nonce_state(struct bt_acs_key_desc_runtime *runtime)
{
	mbedtls_platform_zeroize(runtime->server_nonce_fixed, sizeof(runtime->server_nonce_fixed));
	mbedtls_platform_zeroize(runtime->client_nonce_fixed, sizeof(runtime->client_nonce_fixed));
	runtime->client_nonce_set = false;
	runtime->tx_nonce_counter = 0U;
	runtime->rx_nonce_counter = 0U;
}

int acs_crypto_bind_algorithm_keys(struct bt_acs_conn *acs_conn,
				   struct bt_acs_key_desc_runtime *parent)
{
	uint16_t parent_key_id;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(parent != NULL);
	__ASSERT_NO_MSG(parent->psa_key_id != 0U);

	parent_key_id = acs_key_desc_runtime_key_id(parent);

	for (size_t i = ACS_KEY_ID_COUNT; i < ACS_KEY_RUNTIME_COUNT; i++) {
		struct bt_acs_key_desc_runtime *runtime = &acs_conn->crypto.key_runtimes[i];
		psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
		psa_status_t status;
		int err;

		if (!runtime->key_desc || runtime->current_key_id != parent_key_id) {
			continue;
		}

		/* psa_copy_key allocates a fresh PSA id; destroy any stale child
		 * key on this slot first or it leaks in the PSA keystore.
		 */
		acs_crypto_destroy_key(runtime);

		err = acs_crypto_set_record_key_policy(&attrs, runtime, 0U);
		if (err) {
			return err;
		}
		psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);

		status = psa_copy_key(parent->psa_key_id, &attrs, &runtime->psa_key_id);
		if (status != PSA_SUCCESS) {
			LOG_ERR("record key copy for Key_ID 0x%04x failed: %d",
				acs_key_desc_runtime_key_id(runtime), status);
			runtime->psa_key_id = 0U;
			return -EIO;
		}

		err = acs_derive_nonce_state(runtime, parent);
		if (err) {
			return err;
		}
	}

	return 0;
}

void acs_crypto_invalidate_algorithm_keys(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);

	for (size_t i = ACS_KEY_ID_COUNT; i < ACS_KEY_RUNTIME_COUNT; i++) {
		struct bt_acs_key_desc_runtime *runtime = &acs_conn->crypto.key_runtimes[i];

		if (!runtime->key_desc) {
			continue;
		}

		acs_crypto_destroy_key(runtime);
		acs_clear_nonce_state(runtime);
	}
}
