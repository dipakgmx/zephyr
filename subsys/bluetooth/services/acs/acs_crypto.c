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

BUILD_ASSERT(!(IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC) &&
	       (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||
		IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) ||
		IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM))),
	     "AES-CMAC cannot coexist with AEAD algorithms");
BUILD_ASSERT(ACS_MAX_NONCE_VAR_SIZE <= ACS_NONCE_VAR_COUNTER_SIZE,
	     "ACS runtime currently supports at most an 8-octet nonce variable part");

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

static uint64_t acs_record_initial_rx_counter(const struct bt_acs_record_state *record_state)
{
	return record_state && record_state->key_desc &&
			       record_state->key_desc->aes.nonce_type == ACS_NONCE_SEQ_EVEN_ODD
		       ? UINT64_C(1)
		       : UINT64_C(0);
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

int acs_crypto_record_state_lookup(struct bt_acs_conn *acs_conn, uint16_t key_id,
				   struct bt_acs_record_state **record_state)
{
	if (!acs_conn || !record_state) {
		LOG_ERR("record state lookup called with invalid arguments");
		__ASSERT_NO_MSG(acs_conn != NULL);
		__ASSERT_NO_MSG(record_state != NULL);
		return -EINVAL;
	}

	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.record_states); i++) {
		if (acs_record_key_id(&acs_conn->crypto.record_states[i]) == key_id) {
			*record_state = &acs_conn->crypto.record_states[i];
			return 0;
		}
	}

	*record_state = NULL;
	LOG_ERR("No runtime record state reserved for Key_ID 0x%04x", key_id);
	return -ENOENT;
}

int acs_crypto_record_state_from_isc(struct bt_acs_conn *acs_conn, uint16_t isc_id,
				     struct bt_acs_record_state **record_state)
{
	const struct bt_acs_isc_record *isc = acs_isc_lookup(isc_id);

	if (!acs_conn || !record_state) {
		LOG_ERR("record state resolution from ISC called with invalid arguments");
		__ASSERT_NO_MSG(acs_conn != NULL);
		__ASSERT_NO_MSG(record_state != NULL);
		return -EINVAL;
	}

	if (!isc) {
		*record_state = NULL;
		return -ENOENT;
	}

	return acs_crypto_record_state_lookup(acs_conn, isc->key_id, record_state);
}

int acs_crypto_get_server_nonce_fixed(struct bt_acs_conn *acs_conn, uint16_t key_id,
				      uint8_t *nonce_buf, size_t len)
{
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	struct bt_acs_record_state *record_state;
	uint8_t fixed_size;
	int err;

	if (!acs_conn || !nonce_buf) {
		LOG_ERR("server nonce fixed lookup called with invalid arguments");
		__ASSERT_NO_MSG(acs_conn != NULL);
		__ASSERT_NO_MSG(nonce_buf != NULL);
		return -EINVAL;
	}

	err = acs_crypto_record_state_lookup(acs_conn, key_id, &record_state);
	if (err) {
		return err;
	}

	fixed_size = acs_key_desc_nonce_fixed_size(record_state->key_desc);
	if (fixed_size == 0U) {
		return -ENOTSUP;
	}

	if (fixed_size > sizeof(record_state->server_nonce_fixed)) {
		return -EOVERFLOW;
	}

	bool nonce_unset = true;

	for (uint8_t i = 0U; i < fixed_size; i++) {
		if (record_state->server_nonce_fixed[i] != 0U) {
			nonce_unset = false;
			break;
		}
	}

	if (nonce_unset) {
		sys_rand_get(record_state->server_nonce_fixed, fixed_size);
	}

	memcpy(nonce_buf, record_state->server_nonce_fixed, MIN(len, (size_t)fixed_size));
	return 0;
#else
	ARG_UNUSED(acs_conn);
	ARG_UNUSED(key_id);
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

int acs_crypto_import_record_key(struct bt_acs_record_state *record_state)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	__ASSERT_NO_MSG(record_state != NULL);
	__ASSERT_NO_MSG(record_state->key_desc != NULL);

	switch (record_state->key_desc->type_id) {
	case ACS_KEY_REC_AES_128_CCM:
		psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
		psa_set_key_algorithm(&attrs,
				      PSA_ALG_AEAD_WITH_SHORTENED_TAG(
					      PSA_ALG_CCM, record_state->key_desc->aes.mac_size));
		break;
	case ACS_KEY_REC_AES_128_GCM:
		psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
		psa_set_key_algorithm(&attrs,
				      PSA_ALG_AEAD_WITH_SHORTENED_TAG(
					      PSA_ALG_GCM, record_state->key_desc->aes.mac_size));
		break;
	case ACS_KEY_REC_AES_128_GMAC:
		psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
		psa_set_key_algorithm(&attrs,
				      PSA_ALG_AEAD_WITH_SHORTENED_TAG(
					      PSA_ALG_GCM, record_state->key_desc->aes.mac_size));
		break;
	default:
		return -ENOTSUP;
	}

	psa_set_key_lifetime(&attrs, PSA_KEY_LIFETIME_VOLATILE);
	psa_set_key_type(&attrs, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attrs, CONFIG_BT_ACS_SESSION_KEY_SIZE * BITS_PER_BYTE);

	status = psa_import_key(&attrs, record_state->key, CONFIG_BT_ACS_SESSION_KEY_SIZE,
				&record_state->psa_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Record key import failed for Key_ID 0x%04x: %d",
			acs_record_key_id(record_state), status);
		return -EIO;
	}

	return 0;
}

void acs_crypto_destroy_record_key(struct bt_acs_record_state *record_state)
{
	if (record_state && record_state->psa_key_id != 0) {
		psa_destroy_key(record_state->psa_key_id);
		record_state->psa_key_id = 0;
	}
}

void acs_crypto_destroy_connection_record_keys(struct bt_acs_conn *acs_conn)
{
	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.record_states); i++) {
		acs_crypto_destroy_record_key(&acs_conn->crypto.record_states[i]);
	}
}

int acs_crypto_rebind_record_states(struct bt_acs_conn *acs_conn)
{
	int first_err = 0;

	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.record_states); i++) {
		struct bt_acs_record_state *record_state = &acs_conn->crypto.record_states[i];
		struct bt_acs_runtime_key_state *current_key = NULL;
		uint16_t current_key_id = 0U;
		int err;

		if (!record_state->key_desc) {
			continue;
		}

		acs_crypto_destroy_record_key(record_state);
		memset(record_state->key, 0, sizeof(record_state->key));

		err = acs_current_key_id_from_key_desc(record_state->key_desc, &current_key_id);
		if (err) {
			if (first_err == 0) {
				first_err = err;
			}
			record_state->tx_nonce_counter = 0U;
			record_state->rx_nonce_counter =
				acs_record_initial_rx_counter(record_state);
			continue;
		}

		err = acs_crypto_current_key_lookup(acs_conn, current_key_id, &current_key);
		if (err || !current_key || current_key->psa_key_id == 0U) {
			record_state->tx_nonce_counter = 0U;
			record_state->rx_nonce_counter =
				acs_record_initial_rx_counter(record_state);
			continue;
		}

		memcpy(record_state->key, current_key->key, sizeof(record_state->key));
		err = acs_crypto_import_record_key(record_state);
		if (err && first_err == 0) {
			first_err = err;
		}
	}

	return first_err;
}

void acs_crypto_reset_record_counters(struct bt_acs_conn *acs_conn, uint16_t current_key_id)
{
	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.record_states); i++) {
		struct bt_acs_record_state *record_state = &acs_conn->crypto.record_states[i];
		uint16_t record_current_key_id;

		if (!record_state->key_desc) {
			continue;
		}

		if (acs_current_key_id_from_key_desc(record_state->key_desc,
						     &record_current_key_id) != 0 ||
		    record_current_key_id != current_key_id) {
			continue;
		}

		record_state->tx_nonce_counter = 0U;
		record_state->rx_nonce_counter = acs_record_initial_rx_counter(record_state);
	}
}

/**
 * @brief Build an ACS nonce from a fixed part and packet counter.
 *
 * The fixed part is copied into the most-significant portion of the nonce in
 * reverse order (wire LSO -> nonce MSO). The variable part is then encoded in
 * big-endian form and right-aligned in the remaining bytes.
 *
 * @param fixed Pointer to the fixed nonce bytes, or `NULL` when no fixed part is used.
 * @param counter Nonce sequence counter to encode.
 * @param nonce_size Total nonce size for the selected algorithm.
 * @param fixed_size Fixed nonce part size for the selected algorithm.
 * @param[out] nonce Output buffer that receives the constructed nonce.
 */
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
	if (record_state->key_desc->aes.nonce_type == ACS_NONCE_SEQ_EVEN_ODD) {
		record_state->tx_nonce_counter += 2U;
		if (record_state->tx_nonce_counter == 0U) {
			record_state->tx_nonce_counter = UINT64_MAX - 1U;
		}
	} else {
		record_state->tx_nonce_counter++;
	}
}

static void acs_advance_rx_counter(struct bt_acs_record_state *record_state)
{
	if (record_state->key_desc->aes.nonce_type == ACS_NONCE_SEQ_EVEN_ODD) {
		record_state->rx_nonce_counter += 2U;
		if ((record_state->rx_nonce_counter & 1U) == 0U) {
			record_state->rx_nonce_counter = UINT64_MAX - 2U;
		}
	} else {
		record_state->rx_nonce_counter++;
	}
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

	if (cipher_len < tag_len) {
		return -EINVAL;
	}

	if (record_state->key_desc->aes.nonce_type == ACS_NONCE_SEQ_EVEN_ODD &&
	    (record_state->rx_nonce_counter & 1U) == 0U) {
		LOG_ERR("Even-Odd parity violated (rx_counter=0x%016llx)",
			(unsigned long long)record_state->rx_nonce_counter);
		return -EINVAL;
	}

	/* §3.6.2: refuse to decrypt once the nonce space is exhausted */
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
		LOG_ERR("decrypt failed: %d (cipher_len=%u tag_len=%u "
			"nonce_len=%u aad_len=%u rx_nonce_counter=%llu)",
			status, cipher_len, tag_len, nonce_size, aad_len,
			(unsigned long long)record_state->rx_nonce_counter);
		return -EACCES;
	}

	*plain_len = (uint16_t)out_len;

	acs_advance_rx_counter(record_state);

	return 0;
}

#endif /* CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM || CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM */

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

#endif /* CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC */

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

#endif /* CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC */

int acs_crypto_encrypt(struct bt_acs_conn *acs_conn, uint16_t isc_id, const uint8_t *plaintext,
		       uint16_t plain_len, uint8_t *ciphertext, uint16_t *cipher_len)
{
	const struct bt_acs_isc_record *isc;
	const struct bt_acs_key_desc_record *key;
	struct bt_acs_record_state *record_state;
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

	err = acs_crypto_record_state_from_isc(acs_conn, isc_id, &record_state);
	if (err || record_state->psa_key_id == 0U) {
		LOG_ERR("Encrypt: no record key installed for ISC 0x%04x", isc_id);
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
	struct bt_acs_record_state *record_state;
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

	err = acs_crypto_record_state_from_isc(acs_conn, isc_id, &record_state);
	if (err || record_state->psa_key_id == 0U) {
		LOG_ERR("Decrypt: no record key installed for ISC 0x%04x", isc_id);
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
		LOG_ERR("unsupported algorithm type 0x%02x for ISC 0x%04x", key->type_id, isc_id);
		return -ENOTSUP;
	}
}
