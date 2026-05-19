/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>

#include <psa/crypto.h>

#include "acs_crypto.h"
#include "acs_key_desc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

BUILD_ASSERT(CONFIG_BT_ACS_SESSION_KEY_SIZE == 16, "AES-128 session key fixed at 16 bytes");
BUILD_ASSERT(ACS_MAX_NONCE_VAR_SIZE <= ACS_NONCE_VAR_COUNTER_SIZE,
	     "ACS runtime currently supports at most an 8-octet nonce variable part");
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD),
	     "EVEN_ODD not supported by the current runtime");

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
#define ACS_PSA_CMAC_TAG_LEN ACS_CRYPTO_AUTH_TAG_SIZE
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
#define ACS_PSA_GMAC_ALG     PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_GCM, ACS_GCM_MAC_SIZE)
#define ACS_PSA_GMAC_TAG_LEN ACS_GCM_MAC_SIZE
#endif

static void acs_build_nonce(const uint8_t *fixed, uint64_t counter, uint8_t nonce_size,
			    uint8_t fixed_size, uint8_t *nonce)
{
	uint8_t var_size = nonce_size - fixed_size;
	uint8_t counter_be[sizeof(counter)];

	memset(nonce, 0, nonce_size);
	if (fixed && fixed_size > 0U) {
		for (uint8_t i = 0; i < fixed_size; i++) {
			nonce[i] = fixed[fixed_size - 1U - i];
		}
	}

	sys_put_be64(counter, counter_be);
	if (var_size <= sizeof(counter_be)) {
		memcpy(&nonce[nonce_size - var_size], &counter_be[sizeof(counter_be) - var_size],
		       var_size);
	}
}

static void acs_advance_tx_counter(struct bt_acs_record_state *record_state)
{
	record_state->tx_nonce_counter++;
}

static void acs_advance_rx_counter(struct bt_acs_record_state *record_state)
{
	record_state->rx_nonce_counter++;
}

static void acs_build_record_tx_nonce(const struct bt_acs_record_state *record_state,
				      uint8_t *nonce)
{
	acs_build_nonce(record_state->server_nonce_fixed, record_state->tx_nonce_counter,
			acs_key_desc_nonce_size(record_state->key_desc),
			acs_key_desc_nonce_fixed_size(record_state->key_desc), nonce);
}

static void acs_build_record_rx_nonce(const struct bt_acs_record_state *record_state,
				      uint8_t *nonce)
{
	acs_build_nonce(record_state->client_nonce_fixed, record_state->rx_nonce_counter,
			acs_key_desc_nonce_size(record_state->key_desc),
			acs_key_desc_nonce_fixed_size(record_state->key_desc), nonce);
}

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) ||                                           \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
static int acs_crypto_aead_encrypt(struct bt_acs_conn *acs_conn,
				   struct bt_acs_record_state *record_state,
				   const uint8_t *plaintext, uint16_t plain_len,
				   uint8_t *ciphertext, uint16_t *cipher_len)
{
	uint8_t nonce[ACS_MAX_NONCE_SIZE];
	psa_status_t status;
	size_t out_len;
	uint8_t nonce_size = acs_key_desc_nonce_size(record_state->key_desc);
	uint8_t tag_len = acs_key_desc_auth_tag_size(record_state->key_desc);

	ARG_UNUSED(acs_conn);

	if (record_state->tx_nonce_counter > ACS_COUNTER_TX_MAX) {
		LOG_ERR("TX nonce space exhausted — rekey required");
		return -ENOSPC;
	}

	acs_build_record_tx_nonce(record_state, nonce);

	status = psa_aead_encrypt(record_state->psa_key_id,
				  record_state->key_desc->type_id == ACS_KEY_REC_AES_128_CCM
					  ? PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, tag_len)
					  : PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_GCM, tag_len),
				  nonce, nonce_size, NULL, 0, plaintext, plain_len, ciphertext,
				  plain_len + tag_len, &out_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("encrypt failed: status=%d, len=%u, tx_nonce_counter=0x%016llx", status,
			plain_len, (unsigned long long)record_state->tx_nonce_counter);
		return -EIO;
	}

	*cipher_len = (uint16_t)out_len;
	acs_advance_tx_counter(record_state);

	return 0;
}

static int acs_crypto_aead_decrypt(struct bt_acs_conn *acs_conn,
				   struct bt_acs_record_state *record_state,
				   const uint8_t *ciphertext, uint16_t cipher_len,
				   uint8_t *plaintext, uint16_t *plain_len, const uint8_t *aad,
				   uint16_t aad_len)
{
	uint8_t nonce[ACS_MAX_NONCE_SIZE];
	psa_status_t status;
	size_t out_len;
	uint8_t nonce_size = acs_key_desc_nonce_size(record_state->key_desc);
	uint8_t tag_len = acs_key_desc_auth_tag_size(record_state->key_desc);

	ARG_UNUSED(acs_conn);

	if (cipher_len < tag_len) {
		return -EINVAL;
	}

	if (record_state->rx_nonce_counter > ACS_COUNTER_RX_MAX) {
		LOG_ERR("RX nonce space exhausted — rekey required");
		return -ENOSPC;
	}

	acs_build_record_rx_nonce(record_state, nonce);

	status = psa_aead_decrypt(record_state->psa_key_id,
				  record_state->key_desc->type_id == ACS_KEY_REC_AES_128_CCM
					  ? PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, tag_len)
					  : PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_GCM, tag_len),
				  nonce, nonce_size, aad, aad_len, ciphertext, cipher_len,
				  plaintext, cipher_len - tag_len, &out_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("decrypt failed: %d (cipher_len=%u tag_len=%u nonce_len=%u aad_len=%u "
			"rx_nonce_counter=%llu)",
			status, cipher_len, tag_len, nonce_size, aad_len,
			(unsigned long long)record_state->rx_nonce_counter);
		return -EACCES;
	}

	*plain_len = (uint16_t)out_len;
	acs_advance_rx_counter(record_state);

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
static int acs_crypto_cmac_encrypt(struct bt_acs_conn *acs_conn,
				   struct bt_acs_record_state *record_state,
				   const uint8_t *plaintext, uint16_t plain_len,
				   uint8_t *ciphertext, uint16_t *cipher_len)
{
	uint8_t nonce[ACS_MAX_NONCE_SIZE];
	psa_status_t status;
	psa_mac_operation_t op = PSA_MAC_OPERATION_INIT;
	size_t mac_len;

	ARG_UNUSED(acs_conn);

	if (record_state->tx_nonce_counter > ACS_COUNTER_TX_MAX) {
		LOG_ERR("TX nonce space exhausted — rekey required");
		return -ENOSPC;
	}

	acs_build_record_tx_nonce(record_state, nonce);

	status = psa_mac_sign_setup(&op, record_state->psa_key_id, PSA_ALG_CMAC);
	if (status != PSA_SUCCESS) {
		LOG_ERR("ACS CMAC sign setup failed: %d", status);
		return -EIO;
	}

	status = psa_mac_update(&op, nonce, acs_key_desc_nonce_size(record_state->key_desc));
	if (status == PSA_SUCCESS) {
		status = psa_mac_update(&op, plaintext, plain_len);
	}

	if (plaintext != ciphertext) {
		memcpy(ciphertext, plaintext, plain_len);
	}

	if (status == PSA_SUCCESS) {
		status = psa_mac_sign_finish(&op, &ciphertext[plain_len], ACS_PSA_CMAC_TAG_LEN,
					     &mac_len);
	} else {
		psa_mac_abort(&op);
	}

	if (status != PSA_SUCCESS) {
		LOG_ERR("ACS CMAC sign failed: %d", status);
		return -EIO;
	}

	*cipher_len = plain_len + (uint16_t)mac_len;
	acs_advance_tx_counter(record_state);

	return 0;
}

static int acs_crypto_cmac_decrypt(struct bt_acs_conn *acs_conn,
				   struct bt_acs_record_state *record_state,
				   const uint8_t *ciphertext, uint16_t cipher_len,
				   uint8_t *plaintext, uint16_t *plain_len, const uint8_t *aad,
				   uint16_t aad_len)
{
	uint16_t data_len;
	const uint8_t *tag;
	uint8_t nonce[ACS_MAX_NONCE_SIZE];
	psa_status_t status;
	psa_mac_operation_t op = PSA_MAC_OPERATION_INIT;

	ARG_UNUSED(acs_conn);
	ARG_UNUSED(aad);
	ARG_UNUSED(aad_len);

	if (cipher_len < ACS_PSA_CMAC_TAG_LEN) {
		return -EINVAL;
	}

	if (record_state->rx_nonce_counter > ACS_COUNTER_RX_MAX) {
		LOG_ERR("RX nonce space exhausted — rekey required");
		return -ENOSPC;
	}

	data_len = cipher_len - ACS_PSA_CMAC_TAG_LEN;
	tag = &ciphertext[data_len];
	acs_build_record_rx_nonce(record_state, nonce);

	status = psa_mac_verify_setup(&op, record_state->psa_key_id, PSA_ALG_CMAC);
	if (status != PSA_SUCCESS) {
		LOG_ERR("ACS CMAC verify setup failed: %d", status);
		return -EIO;
	}

	status = psa_mac_update(&op, nonce, acs_key_desc_nonce_size(record_state->key_desc));
	if (status == PSA_SUCCESS) {
		status = psa_mac_update(&op, ciphertext, data_len);
	}
	if (status == PSA_SUCCESS) {
		status = psa_mac_verify_finish(&op, tag, ACS_PSA_CMAC_TAG_LEN);
	} else {
		psa_mac_abort(&op);
	}

	if (status != PSA_SUCCESS) {
		LOG_ERR("ACS CMAC verify failed: %d (cipher_len=%u rx_counter=%llu)", status,
			cipher_len, (unsigned long long)record_state->rx_nonce_counter);
		return -EACCES;
	}

	if (plaintext != ciphertext) {
		memcpy(plaintext, ciphertext, data_len);
	}
	*plain_len = data_len;
	acs_advance_rx_counter(record_state);

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
static int acs_crypto_gmac_encrypt(struct bt_acs_conn *acs_conn,
				   struct bt_acs_record_state *record_state,
				   const uint8_t *plaintext, uint16_t plain_len,
				   uint8_t *ciphertext, uint16_t *cipher_len)
{
	uint8_t nonce[ACS_MAX_NONCE_SIZE];
	psa_status_t status;
	uint8_t tag[ACS_PSA_GMAC_TAG_LEN];
	size_t tag_len;

	ARG_UNUSED(acs_conn);

	if (record_state->tx_nonce_counter > ACS_COUNTER_TX_MAX) {
		LOG_ERR("TX nonce space exhausted — rekey required");
		return -ENOSPC;
	}

	acs_build_record_tx_nonce(record_state, nonce);

	status = psa_aead_encrypt(record_state->psa_key_id, ACS_PSA_GMAC_ALG, nonce,
				  acs_key_desc_nonce_size(record_state->key_desc), plaintext,
				  plain_len, NULL, 0, tag, sizeof(tag), &tag_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("ACS GMAC encrypt failed: %d", status);
		return -EIO;
	}

	if (plaintext != ciphertext) {
		memcpy(ciphertext, plaintext, plain_len);
	}
	memcpy(&ciphertext[plain_len], tag, tag_len);

	*cipher_len = plain_len + (uint16_t)tag_len;
	acs_advance_tx_counter(record_state);

	return 0;
}

static int acs_crypto_gmac_decrypt(struct bt_acs_conn *acs_conn,
				   struct bt_acs_record_state *record_state,
				   const uint8_t *ciphertext, uint16_t cipher_len,
				   uint8_t *plaintext, uint16_t *plain_len, const uint8_t *aad,
				   uint16_t aad_len)
{
	uint16_t data_len;
	uint8_t nonce[ACS_MAX_NONCE_SIZE];
	psa_status_t status;
	uint8_t empty;
	size_t out_len;

	ARG_UNUSED(acs_conn);
	ARG_UNUSED(aad);
	ARG_UNUSED(aad_len);

	if (cipher_len < ACS_PSA_GMAC_TAG_LEN) {
		return -EINVAL;
	}

	if (record_state->rx_nonce_counter > ACS_COUNTER_RX_MAX) {
		LOG_ERR("RX nonce space exhausted — rekey required");
		return -ENOSPC;
	}

	data_len = cipher_len - ACS_PSA_GMAC_TAG_LEN;
	acs_build_record_rx_nonce(record_state, nonce);

	status = psa_aead_decrypt(record_state->psa_key_id, ACS_PSA_GMAC_ALG, nonce,
				  acs_key_desc_nonce_size(record_state->key_desc), ciphertext,
				  data_len, &ciphertext[data_len], ACS_PSA_GMAC_TAG_LEN, &empty, 0,
				  &out_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("ACS GMAC verify failed: %d (cipher_len=%u rx_counter=%llu)", status,
			cipher_len, (unsigned long long)record_state->rx_nonce_counter);
		return -EACCES;
	}

	if (plaintext != ciphertext) {
		memcpy(plaintext, ciphertext, data_len);
	}
	*plain_len = data_len;
	acs_advance_rx_counter(record_state);

	return 0;
}
#endif

int acs_crypto_encrypt(struct bt_acs_conn *acs_conn, struct bt_acs_record_state *record_state,
		       const uint8_t *plaintext, uint16_t plain_len, uint8_t *ciphertext,
		       uint16_t *cipher_len)
{
	const struct bt_acs_key_desc_record *key;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(record_state != NULL);
	__ASSERT_NO_MSG(record_state->key_desc != NULL);

	key = record_state->key_desc;
	if (record_state->psa_key_id == 0U) {
		LOG_ERR("Encrypt: no record key installed for Key_ID 0x%04x",
			acs_record_key_id(record_state));
		return -EACCES;
	}

	switch (key->type_id) {
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	case ACS_KEY_REC_AES_128_CCM:
		return acs_crypto_aead_encrypt(acs_conn, record_state, plaintext, plain_len,
					       ciphertext, cipher_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
	case ACS_KEY_REC_AES_128_GCM:
		return acs_crypto_aead_encrypt(acs_conn, record_state, plaintext, plain_len,
					       ciphertext, cipher_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	case ACS_KEY_REC_AES_128_CMAC:
		return acs_crypto_cmac_encrypt(acs_conn, record_state, plaintext, plain_len,
					       ciphertext, cipher_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case ACS_KEY_REC_AES_128_GMAC:
		return acs_crypto_gmac_encrypt(acs_conn, record_state, plaintext, plain_len,
					       ciphertext, cipher_len);
#endif
	default:
		LOG_ERR("unsupported algorithm type 0x%02x for Key_ID 0x%04x", key->type_id,
			acs_record_key_id(record_state));
		return -ENOTSUP;
	}
}

int acs_crypto_decrypt(struct bt_acs_conn *acs_conn, struct bt_acs_record_state *record_state,
		       const uint8_t *ciphertext, uint16_t cipher_len, uint8_t *plaintext,
		       uint16_t *plain_len, const uint8_t *aad, uint16_t aad_len)
{
	const struct bt_acs_key_desc_record *key;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(record_state != NULL);
	__ASSERT_NO_MSG(record_state->key_desc != NULL);

	key = record_state->key_desc;
	if (record_state->psa_key_id == 0U) {
		LOG_ERR("Decrypt: no record key installed for Key_ID 0x%04x",
			acs_record_key_id(record_state));
		return -EACCES;
	}

	switch (key->type_id) {
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	case ACS_KEY_REC_AES_128_CCM:
		return acs_crypto_aead_decrypt(acs_conn, record_state, ciphertext, cipher_len,
					       plaintext, plain_len, aad, aad_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
	case ACS_KEY_REC_AES_128_GCM:
		return acs_crypto_aead_decrypt(acs_conn, record_state, ciphertext, cipher_len,
					       plaintext, plain_len, aad, aad_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	case ACS_KEY_REC_AES_128_CMAC:
		return acs_crypto_cmac_decrypt(acs_conn, record_state, ciphertext, cipher_len,
					       plaintext, plain_len, aad, aad_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case ACS_KEY_REC_AES_128_GMAC:
		return acs_crypto_gmac_decrypt(acs_conn, record_state, ciphertext, cipher_len,
					       plaintext, plain_len, aad, aad_len);
#endif
	default:
		LOG_ERR("unsupported algorithm type 0x%02x for Key_ID 0x%04x", key->type_id,
			acs_record_key_id(record_state));
		return -ENOTSUP;
	}
}
