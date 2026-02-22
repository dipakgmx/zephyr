/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/services/acs.h>

#include <mbedtls/platform_util.h>
#include <psa/crypto.h>

#include "acs_internal.h"
#include "acs_isc.h"
#include "acs_key_desc.h"
#include "acs_crypto.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* PSA key size for Curve P-256. */
#define ACS_PSA_KEY_BITS_P256 256U

/* Uncompressed NIST-curve point prefix: [0x04][X_BE][Y_BE]. */
#define ACS_ECDH_UNCOMPRESSED_POINT 0x04U

/* ECDH is fixed to Curve P-256 (secp256r1) per ACP 1.0 §4.1.1.1. */
#define ACS_PSA_ECC_FAMILY PSA_ECC_FAMILY_SECP_R1
#define ACS_PSA_KEY_BITS   ACS_PSA_KEY_BITS_P256

/* Return the Key_ID that supplies this record's key material. */
static int acs_crypto_current_key_id_from_key_desc(const struct bt_acs_key_desc_record *rec,
						   uint16_t *current_key_id)
{
	const struct bt_acs_key_desc_record *parent;

	if (!acs_key_desc_is_algorithm_record(rec)) {
		*current_key_id = rec->key_id;
		return 0;
	}

	/* Algorithm records always point directly at an exchange key (1 hop). */
	parent = acs_key_desc_lookup(acs_key_desc_parent_key_id(rec));
	if (parent != NULL && !acs_key_desc_is_algorithm_record(parent)) {
		*current_key_id = parent->key_id;
		return 0;
	}

	LOG_ERR("Unable to resolve current key from key descriptor relation");
	return -ENOENT;
}

void acs_crypto_init_slots(struct bt_acs_conn *acs_conn)
{
	size_t slot = 0;

	acs_conn->crypto.key_runtimes[slot++].key_desc = acs_key_desc_lookup(ACS_KEY_ID_ECDH);
	acs_conn->crypto.key_runtimes[slot++].key_desc = acs_key_desc_lookup(ACS_KEY_ID_KDF);

	__ASSERT_NO_MSG(slot <= ACS_KEY_ID_COUNT);

	slot = ACS_KEY_ID_COUNT;

	STRUCT_SECTION_FOREACH(bt_acs_key_desc_record, rec) {
		struct bt_acs_key_desc_runtime *runtime;
		int err;

		if (!acs_key_desc_is_algorithm_record(rec)) {
			continue;
		}

		if (slot >= ACS_KEY_RUNTIME_COUNT) {
			LOG_ERR("No runtime slot for algorithm record Key_ID 0x%04x", rec->key_id);
			break;
		}
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

int acs_crypto_key_runtime_lookup(struct bt_acs_conn *acs_conn, uint16_t key_id,
				  struct bt_acs_key_desc_runtime **key_runtime)
{
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(key_runtime != NULL);

	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.key_runtimes); i++) {
		if (acs_key_desc_runtime_key_id(&acs_conn->crypto.key_runtimes[i]) == key_id) {
			*key_runtime = &acs_conn->crypto.key_runtimes[i];
			return 0;
		}
	}

	*key_runtime = NULL;
	LOG_ERR("No runtime slot reserved for Key_ID 0x%04x", key_id);
	return -ENOENT;
}

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
int acs_server_nonce_fixed_ensure(struct bt_acs_key_desc_runtime *runtime)
{
	uint8_t size;
	psa_status_t rand_status;

	__ASSERT_NO_MSG(runtime != NULL);
	__ASSERT_NO_MSG(runtime->key_desc != NULL);

	size = acs_key_desc_nonce_prefix_size(runtime->key_desc);
	if (size == 0U) {
		runtime->server_nonce_set = false;
		return 0;
	}

	if (size > sizeof(runtime->server_nonce_fixed)) {
		return -EOVERFLOW;
	}

	if (runtime->server_nonce_set) {
		return 0;
	}

	rand_status = psa_generate_random(runtime->server_nonce_fixed, size);
	if (rand_status != PSA_SUCCESS) {
		LOG_ERR("nonce fixed random generation failed: %d", rand_status);
		return -EIO;
	}

	/* Store in runtime (MSO) order; wire emission reverses to LSO. */
	sys_mem_swap(runtime->server_nonce_fixed, size);
	runtime->server_nonce_set = true;
	return 0;
}
#endif /* CONFIG_BT_ACS_HAS_NONCE_FIXED */

int acs_crypto_get_server_nonce_fixed(struct bt_acs_conn *acs_conn, uint16_t key_id,
				      uint8_t *nonce_buf, size_t len)
{
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	struct bt_acs_key_desc_runtime *key_desc_runtime;
	uint8_t fixed_size;
	size_t copy_len;
	int err;

	__ASSERT_NO_MSG(nonce_buf != NULL);

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

	err = acs_server_nonce_fixed_ensure(key_desc_runtime);
	if (err) {
		return err;
	}

	copy_len = MIN(len, (size_t)fixed_size);
	memcpy(nonce_buf, key_desc_runtime->server_nonce_fixed, copy_len);
	sys_mem_swap(nonce_buf, copy_len);
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

	if (key_id == NULL || *key_id == 0U) {
		return;
	}

	status = psa_destroy_key(*key_id);
	if (status != PSA_SUCCESS) {
		LOG_WRN("psa_destroy_key(%u) failed: %d", (unsigned int)*key_id, status);
	}

	*key_id = 0U;
}

int acs_crypto_generate_keypair(struct bt_acs_conn *acs_conn)
{
	struct acs_ecdh_pubkey *pk = &acs_conn->kex->server_pubkey;
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;
	uint8_t pub[1U + 2U * ACS_ECDH_COORD_SIZE];
	size_t pub_len;
	const uint8_t *x_be = &pub[1];
	const uint8_t *y_be = &pub[1U + ACS_ECDH_COORD_SIZE];

	psa_set_key_type(&attrs, PSA_KEY_TYPE_ECC_KEY_PAIR(ACS_PSA_ECC_FAMILY));
	psa_set_key_bits(&attrs, ACS_PSA_KEY_BITS);
	psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_DERIVE);
	psa_set_key_algorithm(&attrs, PSA_ALG_ECDH);

	status = psa_generate_key(&attrs, &acs_conn->kex->ecdh_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_generate_key failed: %d", status);
		return -EIO;
	}

	/* Export the public key to populate the server coordinates. */
	status = psa_export_public_key(acs_conn->kex->ecdh_key_id, pub, sizeof(pub), &pub_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_export_public_key failed: %d", status);
		acs_psa_destroy_key(&acs_conn->kex->ecdh_key_id);
		return -EIO;
	}

	pk->key_id = sys_cpu_to_le16(ACS_KEY_ID_ECDH);
	pk->x_size = ACS_ECDH_COORD_SIZE;
	pk->y_size = ACS_ECDH_COORD_SIZE;

	/* PSA exports P-256 as [0x04][X_BE][Y_BE]; ACS carries each coordinate LE. */
	if (pub_len != sizeof(pub)) {
		LOG_ERR("Unexpected NIST public key size: %zu (expected %zu)", pub_len,
			sizeof(pub));
		acs_psa_destroy_key(&acs_conn->kex->ecdh_key_id);
		return -EIO;
	}

	sys_memcpy_swap(pk->x, x_be, ACS_ECDH_COORD_SIZE); /* X: BE->LE */
	sys_memcpy_swap(pk->y, y_be, ACS_ECDH_COORD_SIZE); /* Y: BE->LE */

	return 0;
}

int acs_crypto_compute_shared_secret(struct bt_acs_conn *acs_conn)
{
	const struct acs_ecdh_pubkey *cpk = &acs_conn->kex->client_pubkey;
	uint8_t psa_pubkey[1U + 2U * ACS_ECDH_COORD_SIZE];
	size_t psa_pubkey_len;
	size_t olen;
	psa_status_t status;

	/* The client and server public keys must differ (§4.4.3.17.1.1). */
	if (memcmp(cpk->x, acs_conn->kex->server_pubkey.x, ACS_ECDH_COORD_SIZE) == 0 &&
	    memcmp(cpk->y, acs_conn->kex->server_pubkey.y, ACS_ECDH_COORD_SIZE) == 0) {
		LOG_WRN("client public key matches server public key");
		acs_psa_destroy_key(&acs_conn->kex->ecdh_key_id);
		return -EINVAL;
	}

	/*
	 * P-256: PSA expects [0x04][X_BE][Y_BE]. The parser zero-extends both
	 * coordinates to the curve size.
	 */
	psa_pubkey[0] = ACS_ECDH_UNCOMPRESSED_POINT;
	sys_memcpy_swap(&psa_pubkey[1], cpk->x, ACS_ECDH_COORD_SIZE);
	sys_memcpy_swap(&psa_pubkey[1U + ACS_ECDH_COORD_SIZE], cpk->y, ACS_ECDH_COORD_SIZE);
	psa_pubkey_len = sizeof(psa_pubkey);

	{
		uint8_t raw_secret[ACS_SHARED_SECRET_SIZE];
		psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;

		status = psa_raw_key_agreement(PSA_ALG_ECDH, acs_conn->kex->ecdh_key_id, psa_pubkey,
					       psa_pubkey_len, raw_secret, sizeof(raw_secret),
					       &olen);

		acs_psa_destroy_key(&acs_conn->kex->ecdh_key_id);

		if (status != PSA_SUCCESS) {
			LOG_ERR("psa_raw_key_agreement failed: %d", status);
			return (status == PSA_ERROR_INVALID_ARGUMENT) ? -EBADMSG : -EIO;
		}

		psa_set_key_type(&attrs, PSA_KEY_TYPE_DERIVE);
		psa_set_key_bits(&attrs, olen * BITS_PER_BYTE);
		psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_DERIVE | PSA_KEY_USAGE_EXPORT);
		psa_set_key_algorithm(&attrs, ACS_PSA_HKDF_ALG);

		status = psa_import_key(&attrs, raw_secret, olen, &acs_conn->kex->derived_key_id);
		mbedtls_platform_zeroize(raw_secret, sizeof(raw_secret));

		if (status != PSA_SUCCESS) {
			LOG_ERR("Failed to import shared secret into PSA: %d", status);
			return -EIO;
		}
	}

	return 0;
}

/* Exchange keys have no algorithm limit; record keys add it when imported. */
static void acs_crypto_set_exchange_key_policy(psa_key_attributes_t *attrs, psa_key_usage_t usage)
{
	psa_set_key_usage_flags(attrs, usage);
	psa_set_key_algorithm(attrs, PSA_ALG_NONE);
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
				   psa_key_derivation_operation_t *op, size_t key_len,
				   bool derive_twin)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t *target;
	psa_status_t status;

	if (key_runtime == NULL || op == NULL) {
		return -EINVAL;
	}

	if (derive_twin) {
		target = &key_runtime->derive_key_id;
		acs_crypto_set_exchange_derive_policy(&attrs, 0U);
	} else {
		target = &key_runtime->psa_key_id;
		acs_crypto_set_exchange_key_policy(&attrs,
						   PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_COPY);
	}

	acs_psa_destroy_key(target);
	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);
	psa_set_key_bits(&attrs, key_len * BITS_PER_BYTE);

	status = psa_key_derivation_output_key(&attrs, op, target);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Derived exchange key output failed: %d", status);
		return -EIO;
	}

	return 0;
}

/* attrs carries the persistent lifetime and key ID. */
static int acs_psa_copy_key_replace(psa_key_id_t src, const psa_key_attributes_t *attrs,
				    psa_key_id_t dst_id)
{
	psa_key_id_t out_id;
	psa_status_t status;

	status = psa_copy_key(src, attrs, &out_id);
	if (status == PSA_ERROR_ALREADY_EXISTS) {
		status = psa_destroy_key(dst_id);
		if (status != PSA_SUCCESS) {
			LOG_ERR("destroy existing persistent key 0x%08x failed: %d",
				(unsigned int)dst_id, status);
			return -EIO;
		}
		status = psa_copy_key(src, attrs, &out_id);
	}
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_copy_key to persistent key 0x%08x failed: %d", (unsigned int)dst_id,
			status);
		return -EIO;
	}

	return 0;
}

int acs_crypto_copy_key_to_persistent(const struct bt_acs_key_desc_runtime *parent,
				      psa_key_id_t dst_id, psa_key_id_t dst_derive_id)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	int err;

	if (!parent || parent->psa_key_id == 0U || parent->derive_key_id == 0U) {
		return -EINVAL;
	}

	acs_crypto_set_exchange_key_policy(&attrs, PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_COPY);
	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_PERSISTENT);
	psa_set_key_id(&attrs, dst_id);
	psa_set_key_bits(&attrs, ACS_AES_KEY_BITS);

	err = acs_psa_copy_key_replace(parent->psa_key_id, &attrs, dst_id);
	if (err) {
		return err;
	}

	psa_reset_key_attributes(&attrs);
	acs_crypto_set_exchange_derive_policy(&attrs, PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_COPY);
	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_PERSISTENT);
	psa_set_key_id(&attrs, dst_derive_id);
	psa_set_key_bits(&attrs, ACS_AES_KEY_BITS);

	return acs_psa_copy_key_replace(parent->derive_key_id, &attrs, dst_derive_id);
}

int acs_crypto_copy_persistent_key_to_runtime(psa_key_id_t src_id, psa_key_id_t src_derive_id,
					      struct bt_acs_key_desc_runtime *parent)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	if (parent == NULL) {
		return -EINVAL;
	}

	acs_crypto_set_exchange_key_policy(&attrs, PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_COPY);
	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);
	psa_set_key_bits(&attrs, ACS_AES_KEY_BITS);

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
	psa_set_key_bits(&attrs, ACS_AES_KEY_BITS);

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
	if (key_runtime != NULL) {
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

/* Use the same PSA policy for imported and restored keys. */
static int acs_crypto_set_record_key_policy(psa_key_attributes_t *attrs,
					    struct bt_acs_key_desc_runtime *key_desc_runtime,
					    psa_key_usage_t usage)
{
	switch (key_desc_runtime->key_desc->type_id) {
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	case ACS_KEY_REC_AES_128_CCM:
		psa_set_key_usage_flags(attrs,
					PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT | usage);
		key_desc_runtime->psa_alg = PSA_ALG_AEAD_WITH_SHORTENED_TAG(
			PSA_ALG_CCM, key_desc_runtime->key_desc->aes.mac_size);
		psa_set_key_algorithm(attrs, key_desc_runtime->psa_alg);
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                           \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case ACS_KEY_REC_AES_128_GCM:
	case ACS_KEY_REC_AES_128_GMAC:
		psa_set_key_usage_flags(attrs,
					PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT | usage);
		key_desc_runtime->psa_alg = PSA_ALG_AEAD_WITH_SHORTENED_TAG(
			PSA_ALG_GCM, key_desc_runtime->key_desc->aes.mac_size);
		psa_set_key_algorithm(attrs, key_desc_runtime->psa_alg);
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	case ACS_KEY_REC_AES_128_CMAC:
		psa_set_key_usage_flags(attrs, PSA_KEY_USAGE_SIGN_MESSAGE |
						       PSA_KEY_USAGE_VERIFY_MESSAGE | usage);
		key_desc_runtime->psa_alg = PSA_ALG_CMAC;
		psa_set_key_algorithm(attrs, key_desc_runtime->psa_alg);
		break;
#endif
	default:
		return -ENOTSUP;
	}

	psa_set_key_type(attrs, PSA_KEY_TYPE_AES);
	psa_set_key_bits(attrs, ACS_AES_KEY_BITS);
	return 0;
}

void acs_crypto_destroy_connection_record_keys(struct bt_acs_conn *acs_conn)
{
	for (size_t i = ACS_KEY_ID_COUNT; i < ACS_KEY_RUNTIME_COUNT; i++) {
		acs_crypto_destroy_key(&acs_conn->crypto.key_runtimes[i]);
	}
}

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
/* Records sharing a PSA key and algorithm must use different nonce prefixes. */
static int acs_crypto_ensure_unique_server_nonce(struct bt_acs_conn *acs_conn,
						 struct bt_acs_key_desc_runtime *runtime,
						 uint8_t size)
{
	if (size == 0U) {
		return 0;
	}

	for (unsigned int attempt = 0U; attempt < 8U; attempt++) {
		bool collision = false;
		int err;

		for (size_t i = ACS_KEY_ID_COUNT; i < ACS_KEY_RUNTIME_COUNT; i++) {
			struct bt_acs_key_desc_runtime *other = &acs_conn->crypto.key_runtimes[i];

			if (other == runtime || other->key_desc == NULL) {
				continue;
			}
			if (acs_key_desc_nonce_fixed_size(other->key_desc) != size ||
			    !other->server_nonce_set) {
				continue;
			}
			if (memcmp(runtime->server_nonce_fixed, other->server_nonce_fixed, size) ==
			    0) {
				collision = true;
				break;
			}
		}

		if (!collision) {
			return 0;
		}

		memset(runtime->server_nonce_fixed, 0, size);
		runtime->server_nonce_set = false;

		err = acs_server_nonce_fixed_ensure(runtime);
		if (err) {
			return err;
		}
	}

	LOG_ERR("could not derive a unique server nonce prefix");
	return -EAGAIN;
}
#endif /* CONFIG_BT_ACS_HAS_NONCE_FIXED */

int acs_crypto_bind_algorithm_keys(struct bt_acs_conn *acs_conn,
				   struct bt_acs_key_desc_runtime *parent, bool reset_nonce_state)
{
	uint16_t parent_key_id;
	uint8_t key_material[ACS_AES_KEY_SIZE];
	size_t key_len;
	psa_status_t status;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(parent != NULL);
	__ASSERT_NO_MSG(parent->psa_key_id != 0U);

	parent_key_id = acs_key_desc_runtime_key_id(parent);

	status = psa_export_key(parent->psa_key_id, key_material, sizeof(key_material), &key_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("parent key export for bind failed: %d", status);
		return -EIO;
	}

	for (size_t i = ACS_KEY_ID_COUNT; i < ACS_KEY_RUNTIME_COUNT; i++) {
		struct bt_acs_key_desc_runtime *runtime = &acs_conn->crypto.key_runtimes[i];
		psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
		int err;

		if (runtime->key_desc == NULL || runtime->current_key_id != parent_key_id) {
			continue;
		}

		acs_crypto_destroy_key(runtime);

		err = acs_crypto_set_record_key_policy(&attrs, runtime, 0U);
		if (err) {
			mbedtls_platform_zeroize(key_material, sizeof(key_material));
			return err;
		}
		psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);

		status = psa_import_key(&attrs, key_material, key_len, &runtime->psa_key_id);
		if (status != PSA_SUCCESS) {
			LOG_ERR("record key import for Key_ID 0x%04x failed: %d",
				acs_key_desc_runtime_key_id(runtime), status);
			runtime->psa_key_id = 0U;
			mbedtls_platform_zeroize(key_material, sizeof(key_material));
			return -EIO;
		}

		if (reset_nonce_state) {
			err = acs_nonce_state_init(runtime);
			if (err) {
				mbedtls_platform_zeroize(key_material, sizeof(key_material));
				return err;
			}
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
			err = acs_crypto_ensure_unique_server_nonce(
				acs_conn, runtime,
				acs_key_desc_nonce_prefix_size(runtime->key_desc));
			if (err) {
				mbedtls_platform_zeroize(key_material, sizeof(key_material));
				return err;
			}
#endif
		}
	}

	mbedtls_platform_zeroize(key_material, sizeof(key_material));
	return 0;
}

void acs_crypto_invalidate_algorithm_keys(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);

	for (size_t i = ACS_KEY_ID_COUNT; i < ACS_KEY_RUNTIME_COUNT; i++) {
		struct bt_acs_key_desc_runtime *runtime = &acs_conn->crypto.key_runtimes[i];

		if (runtime->key_desc == NULL) {
			continue;
		}

		acs_crypto_destroy_key(runtime);
		mbedtls_platform_zeroize(runtime->server_nonce_fixed,
					 sizeof(runtime->server_nonce_fixed));
		runtime->server_nonce_set = false;
		mbedtls_platform_zeroize(runtime->client_nonce_fixed,
					 sizeof(runtime->client_nonce_fixed));
		runtime->client_nonce_set = false;
		runtime->tx_nonce_counter = 0U;
		runtime->rx_nonce_counter = 0U;
	}
}

void acs_crypto_destroy_kdf_keys(struct bt_acs_conn *acs_conn)
{
	struct bt_acs_key_desc_runtime *kdf_key;

	if (acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_KDF, &kdf_key) == 0) {
		acs_crypto_destroy_key(kdf_key);
	}

	acs_crypto_invalidate_algorithm_keys(acs_conn);
}

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
static int acs_client_nonce_fixed_check_runtime(struct bt_acs_conn const *owner,
						const struct bt_acs_key_desc_runtime *runtime,
						const uint8_t *candidate, uint8_t fixed_size,
						struct bt_acs_conn *other_conn)
{
	for (size_t i = ACS_KEY_ID_COUNT; i < ACS_KEY_RUNTIME_COUNT; i++) {
		struct bt_acs_key_desc_runtime *other = &other_conn->crypto.key_runtimes[i];

		if (other->key_desc == NULL) {
			continue;
		}
		if (acs_key_desc_nonce_fixed_size(other->key_desc) != fixed_size) {
			continue;
		}

		if (other->server_nonce_set &&
		    memcmp(candidate, other->server_nonce_fixed, fixed_size) == 0) {
			return -EEXIST;
		}

		if (other_conn == owner && other == runtime) {
			continue;
		}
		if (other->client_nonce_set &&
		    memcmp(candidate, other->client_nonce_fixed, fixed_size) == 0) {
			return -EEXIST;
		}
	}

	return 0;
}

int acs_client_nonce_fixed_check_unique(struct bt_acs_conn *acs_conn,
					const struct bt_acs_key_desc_runtime *runtime,
					const uint8_t *candidate, uint8_t fixed_size)
{
	for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		struct bt_acs_conn *other_conn = acs_conn_by_index(i);
		int err;

		if (other_conn->conn == NULL) {
			continue;
		}

		err = acs_client_nonce_fixed_check_runtime(acs_conn, runtime, candidate, fixed_size,
							   other_conn);
		if (err) {
			return err;
		}
	}

	return 0;
}
#endif /* BT_ACS_HAS_NONCE_FIXED */
