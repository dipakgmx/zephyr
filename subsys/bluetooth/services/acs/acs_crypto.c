/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/random/random.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_internal.h"
#include "acs_isc.h"
#include "acs_key_desc.h"

#include <psa/crypto.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/*
 * GCM and GMAC share the same 12-byte IV layout (8 fixed + 4 variable) and can
 * coexist. CCM uses a 13-byte nonce and CMAC uses a different auth model;
 * neither can share the compile-time ACS_ACTIVE_* constants with GCM/GMAC.
 */
BUILD_ASSERT(!(IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) &&
	       (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||
		IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC))),
	     "AES-CCM cannot coexist with GCM/GMAC (incompatible nonce sizes)");

BUILD_ASSERT(!(IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC) &&
	       (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||
		IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) ||
		IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM))),
	     "AES-CMAC cannot coexist with AEAD algorithms");
BUILD_ASSERT(ACS_ACTIVE_NONCE_VAR_SIZE <= sizeof(uint32_t),
	     "ACS runtime currently supports at most a 32-bit nonce variable part");

/*
 * Per-algorithm PSA constants.  Each block is compiled when its Kconfig is
 * enabled; multiple can be active simultaneously (GCM + GMAC).
 */
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
#define ACS_PSA_CCM_ALG     PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, CONFIG_BT_ACS_CCM_MAC_SIZE)
#define ACS_PSA_CCM_TAG_LEN CONFIG_BT_ACS_CCM_MAC_SIZE
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
#define ACS_PSA_GCM_ALG     PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_GCM, ACS_GCM_MAC_SIZE)
#define ACS_PSA_GCM_TAG_LEN ACS_GCM_MAC_SIZE
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
#define ACS_PSA_GMAC_ALG     PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_GCM, ACS_GCM_MAC_SIZE)
#define ACS_PSA_GMAC_TAG_LEN ACS_GCM_MAC_SIZE
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
#define ACS_PSA_CMAC_TAG_LEN ACS_CRYPTO_AUTH_TAG_SIZE
#endif

/* Legacy single-algorithm aliases — used by data path for buffer sizing. */
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
#define ACS_PSA_AEAD_ALG ACS_PSA_CCM_ALG
#define ACS_PSA_TAG_LEN  ACS_PSA_CCM_TAG_LEN
#elif IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
#define ACS_PSA_AEAD_ALG ACS_PSA_GCM_ALG
#define ACS_PSA_TAG_LEN  ACS_PSA_GCM_TAG_LEN
#elif IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
#define ACS_PSA_TAG_LEN ACS_PSA_CMAC_TAG_LEN
#elif IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
#define ACS_PSA_AEAD_ALG ACS_PSA_GMAC_ALG
#define ACS_PSA_TAG_LEN  ACS_PSA_GMAC_TAG_LEN
#endif

static bool acs_key_desc_is_algorithm_record(const struct bt_acs_key_desc_record *rec)
{
	return rec->type_id == ACS_KEY_REC_AES_128_CCM || rec->type_id == ACS_KEY_REC_AES_128_GCM ||
	       rec->type_id == ACS_KEY_REC_AES_128_EAX ||
	       rec->type_id == ACS_KEY_REC_AES_128_CMAC || rec->type_id == ACS_KEY_REC_AES_128_GMAC;
}

static uint16_t acs_key_desc_parent_key_id(const struct bt_acs_key_desc_record *rec)
{
	switch (rec->type_id) {
	case ACS_KEY_REC_KDF:
		return rec->kdf.parent_key_id;
	case ACS_KEY_REC_AES_128_CCM:
	case ACS_KEY_REC_AES_128_GCM:
	case ACS_KEY_REC_AES_128_EAX:
	case ACS_KEY_REC_AES_128_CMAC:
	case ACS_KEY_REC_AES_128_GMAC:
		return rec->aes.parent_key_id;
	default:
		return 0U;
	}
}

static int acs_current_key_id_from_key_desc(const struct bt_acs_key_desc_record *rec,
					    uint16_t *current_key_id)
{
	const struct bt_acs_key_desc_record *current = rec;
	uint8_t depth = 0U;

	if (!rec || !current_key_id) {
		LOG_ERR("current key descriptor resolution called with invalid arguments");
		__ASSERT_NO_MSG(rec != NULL);
		__ASSERT_NO_MSG(current_key_id != NULL);
		return -EINVAL;
	}

	while (current && depth++ < ACS_KEY_ID_COUNT) {
		if (!acs_key_desc_is_algorithm_record(current)) {
			*current_key_id = current->key_id;
			return 0;
		}
		current = acs_key_desc_lookup(acs_key_desc_parent_key_id(current));
	}

	LOG_ERR("Unable to resolve current key from key descriptor relation");
	return -ENOENT;
}

int acs_crypto_current_key_lookup(struct bt_acs_conn *acs_conn, uint16_t key_id,
				  struct bt_acs_runtime_key_state **current_key)
{
	if (!acs_conn || !current_key) {
		LOG_ERR("current key lookup called with invalid arguments");
		__ASSERT_NO_MSG(acs_conn != NULL);
		__ASSERT_NO_MSG(current_key != NULL);
		return -EINVAL;
	}

	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.current_keys); i++) {
		if (acs_runtime_key_id(&acs_conn->crypto.current_keys[i]) == key_id) {
			*current_key = &acs_conn->crypto.current_keys[i];
			return 0;
		}
	}

	*current_key = NULL;
	LOG_ERR("No runtime runtime key state reserved for Key_ID 0x%04x", key_id);
	return -ENOENT;
}

int acs_crypto_current_key_from_isc(struct bt_acs_conn *acs_conn, uint16_t isc_id,
				    struct bt_acs_runtime_key_state **current_key)
{
	const struct bt_acs_isc_record *isc = acs_isc_lookup(isc_id);
	const struct bt_acs_key_desc_record *desc;
	uint16_t current_key_id;
	int err;

	if (!acs_conn || !current_key) {
		LOG_ERR("current key resolution from ISC called with invalid arguments");
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

	err = acs_current_key_id_from_key_desc(desc, &current_key_id);
	if (err) {
		*current_key = NULL;
		return err;
	}

	return acs_crypto_current_key_lookup(acs_conn, current_key_id, current_key);
}

int acs_crypto_get_server_nonce_fixed(struct bt_acs_conn *acs_conn, uint8_t *nonce_buf, size_t len)
{
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	bool snf_zero = true;

	for (int i = 0; i < CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE; i++) {
		if (acs_conn->crypto.server_nonce_fixed[i] != 0) {
			snf_zero = false;
			break;
		}
	}
	if (snf_zero) {
		sys_rand_get(acs_conn->crypto.server_nonce_fixed,
			     sizeof(acs_conn->crypto.server_nonce_fixed));
	}
	memcpy(nonce_buf, acs_conn->crypto.server_nonce_fixed,
	       MIN(len, sizeof(acs_conn->crypto.server_nonce_fixed)));
	return 0;
#else
	ARG_UNUSED(acs_conn);
	ARG_UNUSED(nonce_buf);
	ARG_UNUSED(len);
	return -ENOTSUP;
#endif /* CONFIG_BT_ACS_HAS_NONCE_FIXED */
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
	psa_set_key_algorithm(&attrs, ACS_PSA_CCM_ALG);
#else /* GCM and/or GMAC — identical PSA algorithm */
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
	if (current_key && current_key->psa_key_id != 0) {
		psa_destroy_key(current_key->psa_key_id);
		current_key->psa_key_id = 0;
	}
}

void acs_crypto_destroy_connection_keys(struct bt_acs_conn *acs_conn)
{
	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.current_keys); i++) {
		acs_crypto_destroy_current_key(&acs_conn->crypto.current_keys[i]);
	}
}

/**
 * @brief Build an ACS nonce from a fixed part and packet counter.
 *
 * For `CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD`, the nonce is formed only from the
 * counter encoded in big-endian form and right-aligned in the active nonce
 * buffer. Otherwise, the fixed part is placed in the most-significant portion
 * of the nonce and the counter is encoded big-endian in the variable portion.
 *
 * @param fixed Pointer to the fixed nonce bytes, or `NULL` when no fixed part is used.
 * @param counter Nonce sequence counter to encode.
 * @param[out] nonce Output buffer that receives the constructed nonce. Must be
 * `ACS_ACTIVE_NONCE_SIZE` bytes long.
 */
static void acs_build_nonce(const uint8_t *fixed, uint32_t counter, uint8_t *nonce)
{
#if defined(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD)
	/* Counter encoded BE, right-aligned in the nonce buffer. */
	memset(nonce, 0, ACS_ACTIVE_NONCE_SIZE);
	sys_put_be32(counter, &nonce[ACS_ACTIVE_NONCE_SIZE - sizeof(counter)]);
#else
	/* §4.4.4.15.1.4.4.6/7: SEQ_DIFF_FIXED: fixed part (MSO) followed by counter (BE,
	 * right-aligned). */
	const uint8_t var_size = ACS_ACTIVE_NONCE_SIZE - CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE;

	/* Reverse fixed part: wire LSO to nonce MSO. */
	for (uint8_t i = 0; i < CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE; i++) {
		nonce[i] = fixed[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE - 1U - i];
	}

	memset(&nonce[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE], 0, var_size);
	if (var_size >= sizeof(counter)) {
		sys_put_be32(counter, &nonce[ACS_ACTIVE_NONCE_SIZE - sizeof(counter)]);
	} else {
		for (uint8_t i = 0; i < var_size; i++) {
			nonce[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE + i] =
				(counter >> (BITS_PER_BYTE * (var_size - 1U - i))) & UINT8_MAX;
		}
	}
#endif
}

static void acs_advance_tx_counter(struct bt_acs_runtime_key_state *current_key)
{
#if defined(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD)
	/* §3.6.2: EVEN_ODD: step by 2; sentinel 0xFFFFFFFF on wrap. */
	current_key->tx_nonce_counter += 2U;
	if (current_key->tx_nonce_counter == 0U) {
		current_key->tx_nonce_counter = UINT32_MAX;
	}
#else
	current_key->tx_nonce_counter++;
#endif
}

static void acs_advance_rx_counter(struct bt_acs_runtime_key_state *current_key)
{
#if defined(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD)
	/* §3.6.2: EVEN_ODD: step by 2; sentinel 0xFFFFFFFE on wrap. */
	current_key->rx_nonce_counter += 2U;
	if ((current_key->rx_nonce_counter & 1U) == 0U) {
		current_key->rx_nonce_counter = UINT32_MAX - 1UL;
	}
#else
	current_key->rx_nonce_counter++;
#endif
}

/* Build the TX nonce from the server's fixed part (if present) and the counter. */
static void acs_build_tx_nonce(struct bt_acs_conn const *acs_conn,
			       const struct bt_acs_runtime_key_state *current_key, uint8_t *nonce)
{
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	acs_build_nonce(acs_conn->crypto.server_nonce_fixed, current_key->tx_nonce_counter, nonce);
#else
	acs_build_nonce(NULL, current_key->tx_nonce_counter, nonce);
#endif
}

/* Build the RX nonce from the client's fixed part (if present) and the counter. */
static void acs_build_rx_nonce(struct bt_acs_conn const *acs_conn,
			       const struct bt_acs_runtime_key_state *current_key, uint8_t *nonce)
{
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	acs_build_nonce(acs_conn->crypto.client_nonce_fixed, current_key->rx_nonce_counter, nonce);
#else
	acs_build_nonce(NULL, current_key->rx_nonce_counter, nonce);
#endif
}

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) ||                                           \
	IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)

static int acs_crypto_aead_encrypt(struct bt_acs_conn *acs_conn,
				   struct bt_acs_runtime_key_state *current_key,
				   const uint8_t *plaintext, uint16_t plain_len,
				   uint8_t *ciphertext, uint16_t *cipher_len)
{
	uint8_t nonce[ACS_ACTIVE_NONCE_SIZE];
	psa_status_t status;
	size_t out_len;

	if (current_key->tx_nonce_counter > ACS_COUNTER_TX_MAX) {
		LOG_ERR("TX nonce space exhausted — rekey required");
		return -ENOSPC;
	}

	acs_build_tx_nonce(acs_conn, current_key, nonce);

	status = psa_aead_encrypt(current_key->psa_key_id, ACS_PSA_AEAD_ALG, nonce,
				  ACS_ACTIVE_NONCE_SIZE, NULL, 0, plaintext, plain_len, ciphertext,
				  plain_len + ACS_PSA_TAG_LEN, &out_len);

	if (status != PSA_SUCCESS) {
		LOG_ERR("encrypt failed: status=%d, len=%u, tx_nonce_counter=0x%08x", status,
			plain_len, current_key->tx_nonce_counter);
		return -EIO;
	}

	*cipher_len = (uint16_t)out_len;

	acs_advance_tx_counter(current_key);

	return 0;
}

static int acs_crypto_aead_decrypt(struct bt_acs_conn *acs_conn,
				   struct bt_acs_runtime_key_state *current_key,
				   const uint8_t *ciphertext, uint16_t cipher_len,
				   uint8_t *plaintext, uint16_t *plain_len, const uint8_t *aad,
				   uint16_t aad_len)
{
	uint8_t nonce[ACS_ACTIVE_NONCE_SIZE];
	psa_status_t status;
	size_t out_len;

	if (cipher_len < ACS_PSA_TAG_LEN) {
		return -EINVAL;
	}

#if defined(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD)
	/* §4.3.1: client nonces must be odd; even rx_counter indicates corruption */
	if ((current_key->rx_nonce_counter & 1U) == 0U) {
		LOG_ERR("Even-Odd parity violated (rx_counter=0x%08x)",
			current_key->rx_nonce_counter);
		return -EINVAL;
	}
#endif

	/* §3.6.2: refuse to decrypt once the nonce space is exhausted */
	if (current_key->rx_nonce_counter > ACS_COUNTER_RX_MAX) {
		LOG_ERR("RX nonce space exhausted — rekey required");
		return -ENOSPC;
	}

	acs_build_rx_nonce(acs_conn, current_key, nonce);

	status = psa_aead_decrypt(current_key->psa_key_id, ACS_PSA_AEAD_ALG, nonce,
				  ACS_ACTIVE_NONCE_SIZE, aad, aad_len, ciphertext, cipher_len,
				  plaintext, cipher_len - ACS_PSA_TAG_LEN, &out_len);

	if (status != PSA_SUCCESS) {
		LOG_ERR("decrypt failed: %d (cipher_len=%u tag_len=%u "
			"nonce_len=%u aad_len=%u rx_nonce_counter=%u)",
			status, cipher_len, ACS_PSA_TAG_LEN, ACS_ACTIVE_NONCE_SIZE, aad_len,
			current_key->rx_nonce_counter);
		return -EACCES;
	}

	*plain_len = (uint16_t)out_len;

	acs_advance_rx_counter(current_key);

	return 0;
}

#endif /* CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM || CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM */

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)

static int acs_crypto_cmac_encrypt(struct bt_acs_conn *acs_conn,
				   struct bt_acs_runtime_key_state *current_key,
				   const uint8_t *plaintext, uint16_t plain_len,
				   uint8_t *ciphertext, uint16_t *cipher_len)
{
	uint8_t nonce[ACS_ACTIVE_NONCE_SIZE];
	psa_status_t status;
	psa_mac_operation_t op = PSA_MAC_OPERATION_INIT;
	size_t mac_len;

	if (current_key->tx_nonce_counter > ACS_COUNTER_TX_MAX) {
		LOG_ERR("TX nonce space exhausted — rekey required");
		return -ENOSPC;
	}

	acs_build_tx_nonce(acs_conn, current_key, nonce);

	status = psa_mac_sign_setup(&op, current_key->psa_key_id, PSA_ALG_CMAC);
	if (status != PSA_SUCCESS) {
		LOG_ERR("ACS CMAC sign setup failed: %d", status);
		return -EIO;
	}

	status = psa_mac_update(&op, nonce, ACS_ACTIVE_NONCE_SIZE);
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

	acs_advance_tx_counter(current_key);

	return 0;
}

static int acs_crypto_cmac_decrypt(struct bt_acs_conn *acs_conn,
				   struct bt_acs_runtime_key_state *current_key,
				   const uint8_t *ciphertext, uint16_t cipher_len,
				   uint8_t *plaintext, uint16_t *plain_len, const uint8_t *aad,
				   uint16_t aad_len)
{
	uint16_t data_len;
	const uint8_t *tag;
	uint8_t nonce[ACS_ACTIVE_NONCE_SIZE];
	psa_status_t status;
	psa_mac_operation_t op = PSA_MAC_OPERATION_INIT;

	ARG_UNUSED(aad);
	ARG_UNUSED(aad_len);

	if (cipher_len < ACS_PSA_CMAC_TAG_LEN) {
		return -EINVAL;
	}

	if (current_key->rx_nonce_counter > ACS_COUNTER_RX_MAX) {
		LOG_ERR("RX nonce space exhausted — rekey required");
		return -ENOSPC;
	}

	data_len = cipher_len - ACS_PSA_CMAC_TAG_LEN;
	tag = &ciphertext[data_len];

	acs_build_rx_nonce(acs_conn, current_key, nonce);

	status = psa_mac_verify_setup(&op, current_key->psa_key_id, PSA_ALG_CMAC);
	if (status != PSA_SUCCESS) {
		LOG_ERR("ACS CMAC verify setup failed: %d", status);
		return -EIO;
	}

	status = psa_mac_update(&op, nonce, ACS_ACTIVE_NONCE_SIZE);
	if (status == PSA_SUCCESS) {
		status = psa_mac_update(&op, ciphertext, data_len);
	}
	if (status == PSA_SUCCESS) {
		status = psa_mac_verify_finish(&op, tag, ACS_PSA_CMAC_TAG_LEN);
	} else {
		psa_mac_abort(&op);
	}

	if (status != PSA_SUCCESS) {
		LOG_ERR("ACS CMAC verify failed: %d (cipher_len=%u rx_counter=%u)", status,
			cipher_len, current_key->rx_nonce_counter);
		return -EACCES;
	}

	if (plaintext != ciphertext) {
		memcpy(plaintext, ciphertext, data_len);
	}
	*plain_len = data_len;

	acs_advance_rx_counter(current_key);

	return 0;
}

#endif /* CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC */

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)

static int acs_crypto_gmac_encrypt(struct bt_acs_conn *acs_conn,
				   struct bt_acs_runtime_key_state *current_key,
				   const uint8_t *plaintext, uint16_t plain_len,
				   uint8_t *ciphertext, uint16_t *cipher_len)
{
	uint8_t nonce[ACS_ACTIVE_NONCE_SIZE];
	psa_status_t status;
	uint8_t tag[ACS_PSA_GMAC_TAG_LEN];
	size_t tag_len;

	if (current_key->tx_nonce_counter > ACS_COUNTER_TX_MAX) {
		LOG_ERR("TX nonce space exhausted — rekey required");
		return -ENOSPC;
	}

	acs_build_tx_nonce(acs_conn, current_key, nonce);

	status = psa_aead_encrypt(current_key->psa_key_id, ACS_PSA_GMAC_ALG, nonce,
				  ACS_ACTIVE_NONCE_SIZE, plaintext, plain_len, NULL, 0, tag,
				  sizeof(tag), &tag_len);

	if (status != PSA_SUCCESS) {
		LOG_ERR("ACS GMAC encrypt failed: %d", status);
		return -EIO;
	}

	if (plaintext != ciphertext) {
		memcpy(ciphertext, plaintext, plain_len);
	}
	memcpy(&ciphertext[plain_len], tag, tag_len);

	*cipher_len = plain_len + (uint16_t)tag_len;

	acs_advance_tx_counter(current_key);

	return 0;
}

static int acs_crypto_gmac_decrypt(struct bt_acs_conn *acs_conn,
				   struct bt_acs_runtime_key_state *current_key,
				   const uint8_t *ciphertext, uint16_t cipher_len,
				   uint8_t *plaintext, uint16_t *plain_len, const uint8_t *aad,
				   uint16_t aad_len)
{
	uint16_t data_len;
	uint8_t nonce[ACS_ACTIVE_NONCE_SIZE];
	psa_status_t status;
	uint8_t empty;
	size_t out_len;

	ARG_UNUSED(aad);
	ARG_UNUSED(aad_len);

	if (cipher_len < ACS_PSA_GMAC_TAG_LEN) {
		return -EINVAL;
	}

	if (current_key->rx_nonce_counter > ACS_COUNTER_RX_MAX) {
		LOG_ERR("RX nonce space exhausted — rekey required");
		return -ENOSPC;
	}

	data_len = cipher_len - ACS_PSA_GMAC_TAG_LEN;

	acs_build_rx_nonce(acs_conn, current_key, nonce);

	status = psa_aead_decrypt(current_key->psa_key_id, ACS_PSA_GMAC_ALG, nonce,
				  ACS_ACTIVE_NONCE_SIZE, ciphertext, data_len,
				  &ciphertext[data_len], ACS_PSA_GMAC_TAG_LEN, &empty, 0, &out_len);

	if (status != PSA_SUCCESS) {
		LOG_ERR("ACS GMAC verify failed: %d (cipher_len=%u rx_counter=%u)", status,
			cipher_len, current_key->rx_nonce_counter);
		return -EACCES;
	}

	if (plaintext != ciphertext) {
		memcpy(plaintext, ciphertext, data_len);
	}
	*plain_len = data_len;

	acs_advance_rx_counter(current_key);

	return 0;
}

#endif /* CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC */

int acs_crypto_encrypt(struct bt_acs_conn *acs_conn, uint16_t isc_id, const uint8_t *plaintext,
		       uint16_t plain_len, uint8_t *ciphertext, uint16_t *cipher_len)
{
	const struct bt_acs_isc_record *isc;
	const struct bt_acs_key_desc_record *key;
	struct bt_acs_runtime_key_state *current_key;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);

	isc = acs_isc_lookup(isc_id);
	if (!isc) {
		LOG_ERR("Encrypt: unknown ISC 0x%04x", isc_id);
		return -EINVAL;
	}

	key = acs_key_desc_lookup(isc->key_id);
	if (!key) {
		LOG_ERR("Encrypt: ISC 0x%04x references unknown key 0x%04x", isc_id, isc->key_id);
		return -EINVAL;
	}

	err = acs_crypto_current_key_from_isc(acs_conn, isc_id, &current_key);
	if (err || current_key->psa_key_id == 0U) {
		LOG_ERR("Encrypt: no current key installed for ISC 0x%04x", isc_id);
		return -EACCES;
	}

	switch (key->type_id) {
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	case ACS_KEY_REC_AES_128_CCM:
		return acs_crypto_aead_encrypt(acs_conn, current_key, plaintext, plain_len,
					       ciphertext, cipher_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
	case ACS_KEY_REC_AES_128_GCM:
		return acs_crypto_aead_encrypt(acs_conn, current_key, plaintext, plain_len,
					       ciphertext, cipher_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	case ACS_KEY_REC_AES_128_CMAC:
		return acs_crypto_cmac_encrypt(acs_conn, current_key, plaintext, plain_len,
					       ciphertext, cipher_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case ACS_KEY_REC_AES_128_GMAC:
		return acs_crypto_gmac_encrypt(acs_conn, current_key, plaintext, plain_len,
					       ciphertext, cipher_len);
#endif
	default:
		LOG_ERR("unsupported algorithm type 0x%02x for ISC 0x%04x", key->type_id, isc_id);
		return -ENOTSUP;
	}
}

int acs_crypto_decrypt(struct bt_acs_conn *acs_conn, uint16_t isc_id, const uint8_t *ciphertext,
		       uint16_t cipher_len, uint8_t *plaintext, uint16_t *plain_len,
		       const uint8_t *aad, uint16_t aad_len)
{
	const struct bt_acs_isc_record *isc;
	const struct bt_acs_key_desc_record *key;
	struct bt_acs_runtime_key_state *current_key;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);

	isc = acs_isc_lookup(isc_id);
	if (!isc) {
		LOG_ERR("Decrypt: unknown ISC 0x%04x", isc_id);
		return -EINVAL;
	}

	key = acs_key_desc_lookup(isc->key_id);
	if (!key) {
		LOG_ERR("Decrypt: ISC 0x%04x references unknown key 0x%04x", isc_id, isc->key_id);
		return -EINVAL;
	}

	err = acs_crypto_current_key_from_isc(acs_conn, isc_id, &current_key);
	if (err || current_key->psa_key_id == 0U) {
		LOG_ERR("Decrypt: no current key installed for ISC 0x%04x", isc_id);
		return -EACCES;
	}

	switch (key->type_id) {
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	case ACS_KEY_REC_AES_128_CCM:
		return acs_crypto_aead_decrypt(acs_conn, current_key, ciphertext, cipher_len,
					       plaintext, plain_len, aad, aad_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
	case ACS_KEY_REC_AES_128_GCM:
		return acs_crypto_aead_decrypt(acs_conn, current_key, ciphertext, cipher_len,
					       plaintext, plain_len, aad, aad_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	case ACS_KEY_REC_AES_128_CMAC:
		return acs_crypto_cmac_decrypt(acs_conn, current_key, ciphertext, cipher_len,
					       plaintext, plain_len, aad, aad_len);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	case ACS_KEY_REC_AES_128_GMAC:
		return acs_crypto_gmac_decrypt(acs_conn, current_key, ciphertext, cipher_len,
					       plaintext, plain_len, aad, aad_len);
#endif
	default:
		LOG_ERR("unsupported algorithm type 0x%02x for ISC 0x%04x", key->type_id, isc_id);
		return -ENOTSUP;
	}
}
