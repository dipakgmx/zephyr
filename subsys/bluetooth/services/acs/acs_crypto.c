/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Common wrappers manage nonces; the helpers below only call PSA. */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>

#include <psa/crypto.h>

#include "acs_crypto.h"
#include "acs_key_desc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
/* CCM only supports even tag lengths; an odd MAC size fails at PSA key import. */
BUILD_ASSERT((CONFIG_BT_ACS_CCM_MAC_SIZE % 2) == 0,
	     "CONFIG_BT_ACS_CCM_MAC_SIZE must be an even number of bytes (4..16)");
#endif

/* Build an MSO-first nonce from its prefix and 64-bit sequence number. */
static void acs_build_nonce(const uint8_t *prefix, uint64_t counter, uint8_t nonce_size,
			    uint8_t prefix_size, uint8_t *nonce)
{
	uint8_t var_size = nonce_size - prefix_size;
	uint8_t word_be[sizeof(uint64_t)];

	if (prefix && prefix_size > 0U) {
		memcpy(nonce, prefix, prefix_size);
	}

	sys_put_be64(counter, word_be);
	memcpy(&nonce[prefix_size], &word_be[sizeof(word_be) - var_size], var_size);
}

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) ||                                           \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
static int acs_crypto_aead_encrypt(const struct acs_sec_alg *alg, const uint8_t *nonce,
				   uint8_t nonce_size, uint8_t *buf, uint16_t plain_len,
				   uint16_t *cipher_len)
{
	uint8_t tag_len = acs_key_desc_auth_tag_size(alg->key_desc);
	psa_status_t status;
	size_t out_len;

	status = psa_aead_encrypt(alg->psa_key_id, alg->psa_alg, nonce, nonce_size, NULL, 0, buf,
				  plain_len, buf, plain_len + tag_len, &out_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("encrypt failed: status=%d, len=%u", status, plain_len);
		return -EIO;
	}

	__ASSERT_NO_MSG(out_len <= UINT16_MAX);
	*cipher_len = (uint16_t)out_len;

	return 0;
}

static int acs_crypto_aead_decrypt(const struct acs_sec_alg *alg, const uint8_t *nonce,
				   uint8_t nonce_size, const uint8_t *ciphertext,
				   uint16_t cipher_len, uint8_t *plaintext, uint16_t *plain_len)
{
	uint8_t tag_len = acs_key_desc_auth_tag_size(alg->key_desc);
	psa_status_t status;
	size_t out_len;

	status =
		psa_aead_decrypt(alg->psa_key_id, alg->psa_alg, nonce, nonce_size, NULL, 0,
				 ciphertext, cipher_len, plaintext, cipher_len - tag_len, &out_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("decrypt failed: %d (cipher_len=%u tag_len=%u nonce_len=%u)", status,
			cipher_len, tag_len, nonce_size);
		return -EACCES;
	}

	__ASSERT_NO_MSG(out_len <= UINT16_MAX);
	*plain_len = (uint16_t)out_len;

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
static int acs_crypto_cmac_encrypt(const struct acs_sec_alg *alg, const uint8_t *nonce,
				   uint8_t nonce_size, uint8_t *buf, uint16_t plain_len,
				   uint16_t *cipher_len)
{
	uint8_t tag_len = acs_key_desc_auth_tag_size(alg->key_desc);
	psa_status_t status;
	size_t mac_len;

	/* A CMAC record has no nonce (Table 4.45 C.1). */
	ARG_UNUSED(nonce);
	__ASSERT_NO_MSG(nonce_size == 0U);

	status = psa_mac_compute(alg->psa_key_id, PSA_ALG_CMAC, buf, plain_len, &buf[plain_len],
				 tag_len, &mac_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CMAC sign failed: %d", status);
		return -EIO;
	}

	__ASSERT_NO_MSG(mac_len <= UINT16_MAX);
	*cipher_len = plain_len + (uint16_t)mac_len;

	return 0;
}

static int acs_crypto_cmac_decrypt(const struct acs_sec_alg *alg, const uint8_t *nonce,
				   uint8_t nonce_size, const uint8_t *ciphertext,
				   uint16_t cipher_len, uint16_t *plain_len)
{
	uint8_t tag_len = acs_key_desc_auth_tag_size(alg->key_desc);
	uint16_t data_len = cipher_len - tag_len;
	const uint8_t *tag = &ciphertext[data_len];
	psa_status_t status;

	/* A CMAC record has no nonce (Table 4.45 C.1). */
	ARG_UNUSED(nonce);
	__ASSERT_NO_MSG(nonce_size == 0U);

	status = psa_mac_verify(alg->psa_key_id, PSA_ALG_CMAC, ciphertext, data_len, tag, tag_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CMAC verify failed: %d (cipher_len=%u)", status, cipher_len);
		return -EACCES;
	}

	*plain_len = data_len;

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
static int acs_crypto_gmac_encrypt(const struct acs_sec_alg *alg, const uint8_t *nonce,
				   uint8_t nonce_size, uint8_t *buf, uint16_t plain_len,
				   uint16_t *cipher_len)
{
	uint8_t tag[ACS_CRYPTO_AUTH_TAG_SIZE];
	psa_status_t status;
	size_t tag_len;

	/* GMAC = GCM with the payload passed as AAD and zero-length plaintext. */
	status = psa_aead_encrypt(alg->psa_key_id, alg->psa_alg, nonce, nonce_size, buf, plain_len,
				  NULL, 0, tag, sizeof(tag), &tag_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("GMAC encrypt failed: %d", status);
		return -EIO;
	}

	memcpy(&buf[plain_len], tag, tag_len);

	__ASSERT_NO_MSG(tag_len <= UINT16_MAX);
	*cipher_len = plain_len + (uint16_t)tag_len;

	return 0;
}

static int acs_crypto_gmac_decrypt(const struct acs_sec_alg *alg, const uint8_t *nonce,
				   uint8_t nonce_size, const uint8_t *ciphertext,
				   uint16_t cipher_len, uint16_t *plain_len)
{
	uint8_t tag_len = acs_key_desc_auth_tag_size(alg->key_desc);
	uint16_t data_len = cipher_len - tag_len;
	psa_status_t status;
	size_t out_len;

	status = psa_aead_decrypt(alg->psa_key_id, alg->psa_alg, nonce, nonce_size, ciphertext,
				  data_len, &ciphertext[data_len], tag_len, NULL, 0, &out_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("GMAC verify failed: %d (cipher_len=%u)", status, cipher_len);
		return -EACCES;
	}

	*plain_len = data_len;

	return 0;
}
#endif

static int acs_crypto_run_encrypt(const struct acs_sec_alg *alg, const uint8_t *nonce,
				  uint8_t nonce_size, uint8_t *buf, uint16_t plain_len,
				  uint16_t *cipher_len)
{
	switch (alg->key_desc->type_id) {
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	case ACS_KEY_REC_AES_128_CCM:
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
	case ACS_KEY_REC_AES_128_GCM:
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) ||                                           \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
		return acs_crypto_aead_encrypt(alg, nonce, nonce_size, buf, plain_len, cipher_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	case ACS_KEY_REC_AES_128_CMAC:
		return acs_crypto_cmac_encrypt(alg, nonce, nonce_size, buf, plain_len, cipher_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case ACS_KEY_REC_AES_128_GMAC:
		return acs_crypto_gmac_encrypt(alg, nonce, nonce_size, buf, plain_len, cipher_len);
#endif
	default:
		LOG_ERR("unsupported algorithm type 0x%02x for Key_ID 0x%04x",
			alg->key_desc->type_id, acs_sec_alg_id(alg));
		return -ENOTSUP;
	}
}

/* Decrypt buf in place. */
static int acs_crypto_run_decrypt(const struct acs_sec_alg *alg, const uint8_t *nonce,
				  uint8_t nonce_size, uint8_t *buf, uint16_t cipher_len,
				  uint16_t *plain_len)
{
	switch (alg->key_desc->type_id) {
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	case ACS_KEY_REC_AES_128_CCM:
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
	case ACS_KEY_REC_AES_128_GCM:
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) ||                                           \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
		return acs_crypto_aead_decrypt(alg, nonce, nonce_size, buf, cipher_len, buf,
					       plain_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	case ACS_KEY_REC_AES_128_CMAC:
		return acs_crypto_cmac_decrypt(alg, nonce, nonce_size, buf, cipher_len, plain_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case ACS_KEY_REC_AES_128_GMAC:
		return acs_crypto_gmac_decrypt(alg, nonce, nonce_size, buf, cipher_len, plain_len);
#endif
	default:
		LOG_ERR("unsupported algorithm type 0x%02x for Key_ID 0x%04x",
			alg->key_desc->type_id, acs_sec_alg_id(alg));
		return -ENOTSUP;
	}
}

static int acs_crypto_check_key(const struct acs_sec_alg *alg)
{
	if (!acs_sec_alg_ready(alg)) {
		LOG_ERR("no key installed for security algorithm 0x%04x", acs_sec_alg_id(alg));
		return -EACCES;
	}

	return 0;
}

int acs_crypto_encrypt(struct acs_sec_alg *alg, uint8_t *buf, uint16_t plain_len,
		       uint16_t *cipher_len)
{
	const struct bt_acs_key_desc_record *desc;
	uint8_t nonce[MAX(1, ACS_MAX_NONCE_SIZE)];
	uint8_t nonce_size;
	int err;

	err = acs_crypto_check_key(alg);
	if (err) {
		return err;
	}
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(cipher_len != NULL);

	desc = alg->key_desc;
	nonce_size = acs_key_desc_nonce_size(desc);

	if (nonce_size > 0U) {
		uint64_t tx_counter = alg->tx_nonce_counter;

		if (tx_counter == UINT64_MAX) {
			LOG_ERR("TX nonce space exhausted, rekey required");
			return -ENOSPC;
		}

		acs_build_nonce(alg->server_nonce_fixed, tx_counter, nonce_size,
				acs_key_desc_nonce_prefix_size(desc), nonce);
	}

	/* Convert between ACS wire order and PSA byte order. */
	sys_mem_swap(buf, plain_len);

	err = acs_crypto_run_encrypt(alg, nonce, nonce_size, buf, plain_len, cipher_len);
	if (err) {
		LOG_ERR("encrypt failed for Key_ID 0x%04x: %d (tx_nonce_counter=0x%016llx)",
			acs_sec_alg_id(alg), err, (unsigned long long)alg->tx_nonce_counter);
		return err;
	}

	sys_mem_swap(buf, *cipher_len);

	if (nonce_size > 0U) {
		/* DIFF_FIXED advances the sequence number by one (Table 4.47). */
		__ASSERT(desc->aes.nonce_type == ACS_NONCE_SEQ_DIFF_FIXED,
			 "unsupported Nonce_Type %u for counter increment", desc->aes.nonce_type);
		alg->tx_nonce_counter++;
	}

	return 0;
}

int acs_crypto_decrypt(struct acs_sec_alg *alg, uint64_t received_counter, uint8_t *buf,
		       uint16_t buf_len, uint16_t *plain_len)
{
	const struct bt_acs_key_desc_record *desc;
	uint8_t nonce[MAX(1, ACS_MAX_NONCE_SIZE)];
	uint8_t nonce_size;
	int err;

	err = acs_crypto_check_key(alg);
	if (err) {
		return err;
	}
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(plain_len != NULL);

	desc = alg->key_desc;
	nonce_size = acs_key_desc_nonce_size(desc);

	__ASSERT_NO_MSG(buf_len >= acs_key_desc_auth_tag_size(desc));

	if (nonce_size > 0U) {
		if (received_counter == UINT64_MAX) {
			LOG_ERR("RX nonce space exhausted, rekey required");
			return -ENOSPC;
		}

		acs_build_nonce(alg->client_nonce_fixed, received_counter, nonce_size,
				acs_key_desc_nonce_prefix_size(desc), nonce);
	}

	/* Convert between ACS wire order and PSA byte order. */
	sys_mem_swap(buf, buf_len);
	LOG_HEXDUMP_DBG(nonce, nonce_size, "Nonce received");

	err = acs_crypto_run_decrypt(alg, nonce, nonce_size, buf, buf_len, plain_len);
	if (err) {
		LOG_ERR("decrypt failed for Key_ID 0x%04x: %d (received_counter=0x%016llx)",
			acs_sec_alg_id(alg), err, (unsigned long long)received_counter);
		return err;
	}

	sys_mem_swap(buf, *plain_len);

	/* Advance the receive counter only after authentication succeeds. */
	if (nonce_size > 0U) {
		alg->rx_nonce_counter = received_counter + 1U;
	}

	return 0;
}
