/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/services/acs.h>

#include <mbedtls/platform_util.h>
#include <psa/crypto.h>

#include "acs_crypto.h"
#include "acs_key_desc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

void acs_crypto_warn_destroy_key_failure(psa_status_t status, psa_key_id_t key_id, const char *ctx)
{
	if (status != PSA_SUCCESS) {
		LOG_WRN("%s: psa_destroy_key(%u) failed: %d", ctx, (unsigned int)key_id, status);
	}
}

int acs_crypto_import_current_key(struct bt_acs_runtime_key_state *current_key)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_SIGN_MESSAGE | PSA_KEY_USAGE_VERIFY_MESSAGE);
	psa_set_key_algorithm(&attrs, PSA_ALG_CMAC);
#elif IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
	psa_set_key_algorithm(
		&attrs, PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, CONFIG_BT_ACS_CCM_MAC_SIZE));
#elif IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
	psa_set_key_algorithm(&attrs,
			      PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_GCM, ACS_GCM_MAC_SIZE));
#else
	psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
	psa_set_key_algorithm(&attrs,
			      PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_GCM, ACS_GCM_MAC_SIZE));
#endif

	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);
	psa_set_key_type(&attrs, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attrs, CONFIG_BT_ACS_SESSION_KEY_SIZE * BITS_PER_BYTE);

	status = psa_import_key(&attrs, current_key->key, CONFIG_BT_ACS_SESSION_KEY_SIZE,
				&current_key->psa_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Key import failed: %d", status);
		return -EIO;
	}

	return 0;
}

void acs_crypto_destroy_current_key(struct bt_acs_runtime_key_state *current_key)
{
	if (current_key && current_key->psa_key_id != 0U) {
		psa_status_t status = psa_destroy_key(current_key->psa_key_id);

		acs_crypto_warn_destroy_key_failure(status, current_key->psa_key_id,
						    "destroy current key");
		current_key->psa_key_id = 0U;
	}
}

void acs_crypto_destroy_connection_keys(struct bt_acs_conn *acs_conn)
{
	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.current_keys); i++) {
		acs_crypto_destroy_current_key(&acs_conn->crypto.current_keys[i]);
	}
}

int acs_crypto_import_record_key(struct bt_acs_key_desc_runtime *key_desc_runtime)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	__ASSERT_NO_MSG(key_desc_runtime != NULL);
	__ASSERT_NO_MSG(key_desc_runtime->key_desc != NULL);

	switch (key_desc_runtime->key_desc->type_id) {
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	case ACS_KEY_REC_AES_128_CCM:
		psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
		psa_set_key_algorithm(
			&attrs, PSA_ALG_AEAD_WITH_SHORTENED_TAG(
					PSA_ALG_CCM, key_desc_runtime->key_desc->aes.mac_size));
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                           \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case ACS_KEY_REC_AES_128_GCM:
	case ACS_KEY_REC_AES_128_GMAC:
		psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
		psa_set_key_algorithm(
			&attrs, PSA_ALG_AEAD_WITH_SHORTENED_TAG(
					PSA_ALG_GCM, key_desc_runtime->key_desc->aes.mac_size));
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	case ACS_KEY_REC_AES_128_CMAC:
		psa_set_key_usage_flags(&attrs,
					PSA_KEY_USAGE_SIGN_MESSAGE | PSA_KEY_USAGE_VERIFY_MESSAGE);
		psa_set_key_algorithm(&attrs, PSA_ALG_CMAC);
		break;
#endif
	default:
		return -ENOTSUP;
	}

	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);
	psa_set_key_type(&attrs, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attrs, CONFIG_BT_ACS_SESSION_KEY_SIZE * BITS_PER_BYTE);

	status = psa_import_key(&attrs, key_desc_runtime->key, CONFIG_BT_ACS_SESSION_KEY_SIZE,
				&key_desc_runtime->psa_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("key descriptor runtime import failed for Key_ID 0x%04x: %d",
			acs_key_desc_runtime_key_id(key_desc_runtime), status);
		return -EIO;
	}

	return 0;
}

void acs_crypto_destroy_record_key(struct bt_acs_key_desc_runtime *key_desc_runtime)
{
	if (key_desc_runtime && key_desc_runtime->psa_key_id != 0U) {
		psa_status_t status = psa_destroy_key(key_desc_runtime->psa_key_id);

		acs_crypto_warn_destroy_key_failure(status, key_desc_runtime->psa_key_id,
						    "destroy record key");
		key_desc_runtime->psa_key_id = 0U;
	}
}

void acs_crypto_destroy_connection_record_keys(struct bt_acs_conn *acs_conn)
{
	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.key_desc_runtimes); i++) {
		acs_crypto_destroy_record_key(&acs_conn->crypto.key_desc_runtimes[i]);
	}
}

static void acs_clear_key_desc_runtime_nonce_state(struct bt_acs_key_desc_runtime *key_desc_runtime)
{
	mbedtls_platform_zeroize(key_desc_runtime->server_nonce_fixed,
				 sizeof(key_desc_runtime->server_nonce_fixed));
	mbedtls_platform_zeroize(key_desc_runtime->client_nonce_fixed,
				 sizeof(key_desc_runtime->client_nonce_fixed));
	key_desc_runtime->client_nonce_set = false;
	key_desc_runtime->tx_nonce_counter = 0U;
	key_desc_runtime->rx_nonce_counter =
		key_desc_runtime->key_desc &&
				key_desc_runtime->key_desc->aes.nonce_type == ACS_NONCE_SEQ_EVEN_ODD
			? 1U
			: 0U;
}

int acs_crypto_rebind_key_desc_runtimes(struct bt_acs_conn *acs_conn)
{
	int first_err = 0;

	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.key_desc_runtimes); i++) {
		struct bt_acs_key_desc_runtime *key_desc_runtime =
			&acs_conn->crypto.key_desc_runtimes[i];
		struct bt_acs_runtime_key_state *current_key = NULL;
		int err;

		if (!key_desc_runtime->key_desc) {
			continue;
		}

		acs_crypto_destroy_record_key(key_desc_runtime);
		mbedtls_platform_zeroize(key_desc_runtime->key, sizeof(key_desc_runtime->key));

		if (key_desc_runtime->current_key_id == 0U) {
			if (first_err == 0) {
				first_err = -ENOENT;
			}
			acs_clear_key_desc_runtime_nonce_state(key_desc_runtime);
			continue;
		}

		err = acs_crypto_current_key_lookup(acs_conn, key_desc_runtime->current_key_id,
						    &current_key);
		if (err || !current_key || current_key->psa_key_id == 0U) {
			acs_clear_key_desc_runtime_nonce_state(key_desc_runtime);
			continue;
		}

		memcpy(key_desc_runtime->key, current_key->key, sizeof(key_desc_runtime->key));
		err = acs_crypto_import_record_key(key_desc_runtime);
		if (err && first_err == 0) {
			first_err = err;
		}
	}

	return first_err;
}

void acs_crypto_reset_key_desc_runtime_counters(struct bt_acs_conn *acs_conn,
						uint16_t current_key_id)
{
	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.key_desc_runtimes); i++) {
		struct bt_acs_key_desc_runtime *key_desc_runtime =
			&acs_conn->crypto.key_desc_runtimes[i];

		if (!key_desc_runtime->key_desc) {
			continue;
		}

		if (key_desc_runtime->current_key_id != current_key_id) {
			continue;
		}

		acs_clear_key_desc_runtime_nonce_state(key_desc_runtime);
	}
}
