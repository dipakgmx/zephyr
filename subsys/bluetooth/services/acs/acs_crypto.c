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
#include "acs_crypto.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

BUILD_ASSERT(!(IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC) &&
	       (IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||
		IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) ||
		IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM))),
	     "AES-CMAC cannot coexist with AEAD algorithms");

int acs_crypto_current_key_id_from_key_desc(const struct bt_acs_key_desc_record *rec,
					    uint16_t *current_key_id)
{
	const struct bt_acs_key_desc_record *current = rec;
	uint8_t depth = 0U;

	if (!rec || !current_key_id) {
		LOG_DBG("current key descriptor resolution called with invalid arguments");
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

void acs_crypto_init_slots(struct bt_acs_conn *acs_conn)
{
	size_t key_slot = 0;
	size_t record_slot = 0;

	__ASSERT_NO_MSG(acs_conn != NULL);

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	acs_conn->crypto.current_keys[key_slot++].key_desc = acs_key_desc_lookup(ACS_KEY_ID_ECDH);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	acs_conn->crypto.current_keys[key_slot++].key_desc = acs_key_desc_lookup(ACS_KEY_ID_KDF);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	acs_conn->crypto.current_keys[key_slot++].key_desc = acs_key_desc_lookup(ACS_KEY_ID_OOB);
#endif

	__ASSERT_NO_MSG(key_slot <= ARRAY_SIZE(acs_conn->crypto.current_keys));

	STRUCT_SECTION_FOREACH(bt_acs_key_desc_record, rec) {
		struct bt_acs_key_desc_runtime *record_state;
		int err;

		if (!acs_key_desc_has_nonce_record(rec)) {
			continue;
		}

		__ASSERT_NO_MSG(record_slot < ARRAY_SIZE(acs_conn->crypto.key_desc_runtimes));
		record_state = &acs_conn->crypto.key_desc_runtimes[record_slot++];
		record_state->key_desc = rec;
		err = acs_crypto_current_key_id_from_key_desc(rec, &record_state->current_key_id);
		if (err != 0) {
			LOG_WRN("Unable to resolve current key for descriptor Key_ID 0x%04x",
				rec->key_id);
			record_state->current_key_id = 0U;
		}
	}
}

void acs_crypto_reset(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);

	acs_crypto_destroy_connection_record_keys(acs_conn);
	acs_crypto_destroy_connection_keys(acs_conn);
	memset(&acs_conn->crypto, 0, sizeof(acs_conn->crypto));
	acs_crypto_init_slots(acs_conn);
}

void acs_crypto_reset_preserve_record_states(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	{
		struct bt_acs_key_desc_runtime saved_record_states[CONFIG_BT_ACS_MAX_NONCE_RECORDS];

		memcpy(saved_record_states, acs_conn->crypto.key_desc_runtimes,
		       sizeof(saved_record_states));
		memset(&acs_conn->crypto, 0, sizeof(acs_conn->crypto));
		memcpy(acs_conn->crypto.key_desc_runtimes, saved_record_states,
		       sizeof(saved_record_states));
		acs_crypto_init_slots(acs_conn);
	}
#else
	acs_crypto_reset(acs_conn);
#endif
}

int acs_crypto_current_key_lookup(const struct bt_acs_conn *acs_conn, uint16_t key_id,
				  struct bt_acs_runtime_key_state **current_key)
{
	if (!acs_conn || !current_key) {
		LOG_DBG("current key lookup called with invalid arguments");
		__ASSERT_NO_MSG(acs_conn != NULL);
		__ASSERT_NO_MSG(current_key != NULL);
		return -EINVAL;
	}

	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.current_keys); i++) {
		if (acs_runtime_key_id(&acs_conn->crypto.current_keys[i]) == key_id) {
			*current_key = (struct bt_acs_runtime_key_state *)&acs_conn->crypto
					       .current_keys[i];
			return 0;
		}
	}

	*current_key = NULL;
	LOG_ERR("No runtime key state reserved for Key_ID 0x%04x", key_id);
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

	return acs_crypto_current_key_lookup(acs_conn, current_key_id, current_key);
}

int acs_crypto_key_desc_runtime_lookup(struct bt_acs_conn *acs_conn, uint16_t key_id,
				       struct bt_acs_key_desc_runtime **key_desc_runtime)
{
	if (!acs_conn || !key_desc_runtime) {
		LOG_DBG("key descriptor runtime lookup called with invalid arguments");
		__ASSERT_NO_MSG(acs_conn != NULL);
		__ASSERT_NO_MSG(key_desc_runtime != NULL);
		return -EINVAL;
	}

	for (size_t i = 0; i < ARRAY_SIZE(acs_conn->crypto.key_desc_runtimes); i++) {
		if (acs_key_desc_runtime_key_id(&acs_conn->crypto.key_desc_runtimes[i]) == key_id) {
			*key_desc_runtime = &acs_conn->crypto.key_desc_runtimes[i];
			return 0;
		}
	}

	*key_desc_runtime = NULL;
	LOG_ERR("No key descriptor runtime reserved for Key_ID 0x%04x", key_id);
	return -ENOENT;
}

int acs_crypto_get_server_nonce_fixed(struct bt_acs_conn *acs_conn, uint16_t key_id,
				      uint8_t *nonce_buf, size_t len)
{
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	struct bt_acs_key_desc_runtime *key_desc_runtime;
	uint8_t nonce_fixed_wire[ACS_MAX_NONCE_FIXED_SIZE];
	uint8_t fixed_size;
	bool nonce_unset = true;
	int err;

	if (!acs_conn || !nonce_buf) {
		LOG_ERR("server nonce fixed lookup called with invalid arguments");
		__ASSERT_NO_MSG(acs_conn != NULL);
		__ASSERT_NO_MSG(nonce_buf != NULL);
		return -EINVAL;
	}

	err = acs_crypto_key_desc_runtime_lookup(acs_conn, key_id, &key_desc_runtime);
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

	for (uint8_t i = 0U; i < fixed_size; i++) {
		if (key_desc_runtime->server_nonce_fixed[i] != 0U) {
			nonce_unset = false;
			break;
		}
	}

	if (nonce_unset) {
		sys_rand_get(nonce_fixed_wire, fixed_size);
		memcpy(key_desc_runtime->server_nonce_fixed, nonce_fixed_wire, fixed_size);
		sys_mem_swap(key_desc_runtime->server_nonce_fixed, fixed_size);
	} else {
		memcpy(nonce_fixed_wire, key_desc_runtime->server_nonce_fixed, fixed_size);
		sys_mem_swap(nonce_fixed_wire, fixed_size);
	}

	memcpy(nonce_buf, nonce_fixed_wire, MIN(len, (size_t)fixed_size));
	return 0;
#else
	ARG_UNUSED(acs_conn);
	ARG_UNUSED(key_id);
	ARG_UNUSED(nonce_buf);
	ARG_UNUSED(len);
	return -ENOTSUP;
#endif
}
