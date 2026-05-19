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
		if (!acs_key_desc_has_nonce_record(rec)) {
			continue;
		}

		__ASSERT_NO_MSG(record_slot < ARRAY_SIZE(acs_conn->crypto.record_states));
		acs_conn->crypto.record_states[record_slot++].key_desc = rec;
	}
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

	err = acs_crypto_current_key_id_from_key_desc(desc, &current_key_id);
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
	bool nonce_unset = true;
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
#endif
}
