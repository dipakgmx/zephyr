/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_internal.h"
#include "acs_runtime.h"
#include "acs_crypto.h"
#include "acs_isc.h"
#include "acs_rhandle.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Protected_Resource_Handle alone; Request_Or_Response is C.1 (Table 4.10). */
#define ACS_SECURE_DATA_PLAIN_MIN_SIZE 2

/* Resolve the ISC and distinguish a missing key from an invalid setup (§4.3.2). */
static int acs_data_in_resolve_key(struct bt_acs_conn *acs_conn, uint16_t isc_id,
				   struct bt_acs_key_desc_runtime **key_runtime)
{
	const struct bt_acs_key_desc_record *key_desc;

	if (acs_resolve_isc_slot(acs_conn, isc_id, key_runtime) != 0) {
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	key_desc = (*key_runtime)->key_desc;

	if (!acs_key_desc_is_algorithm_record(key_desc)) {
		LOG_WRN("ISC_ID 0x%04x Key_ID 0x%04x does not reference an algorithm record",
			isc_id, acs_key_desc_runtime_key_id(*key_runtime));
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	if (!acs_current_key_installed(*key_runtime)) {
		LOG_WRN("key for ISC_ID 0x%04x not exchanged yet", isc_id);
		return ACS_DATA_ERR_INVALID_KEY;
	}

	if (acs_key_desc_nonce_prefix_size(key_desc) > 0U && !(*key_runtime)->client_nonce_set) {
		LOG_WRN("client nonce fixed not set for ISC_ID 0x%04x", isc_id);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	return 0;
}

/* Validate the Data In security fields and leave buf at MAC || ciphertext. */
static int acs_data_in_validate(struct bt_acs_conn *acs_conn, struct net_buf *buf, uint16_t *isc_id,
				struct bt_acs_key_desc_runtime **key_runtime,
				uint64_t *received_counter)
{
	struct bt_acs_key_desc_runtime *runtime;
	const uint8_t *nonce_var;
	uint64_t counter = 0U;
	uint16_t controls_size;
	uint8_t nonce_var_size;
	uint16_t isc;
	int err;

	if (buf->len < ACS_DATA_IN_HDR_SIZE) {
		LOG_ERR("data-in payload too short for ISC_ID (%u)", buf->len);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	isc = net_buf_pull_le16(buf);

	err = acs_data_in_resolve_key(acs_conn, isc, &runtime);
	if (err) {
		return err;
	}

	/* After ISC_ID: Nonce_Var, MAC, then protected data, all LSO first. */
	nonce_var_size = acs_key_desc_nonce_var_size(runtime->key_desc);
	controls_size = (uint16_t)nonce_var_size + acs_key_desc_auth_tag_size(runtime->key_desc);

	if (buf->len <= controls_size) {
		LOG_ERR("secure data (%u) too short for %u octets of security controls", buf->len,
			controls_size);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	if (nonce_var_size > 0U) {
		nonce_var = net_buf_pull_mem(buf, nonce_var_size);
		sys_get_le(&counter, nonce_var, nonce_var_size);

		if (counter < runtime->rx_nonce_counter) {
			LOG_WRN("stale or replayed nonce (received=0x%016llx vs "
				"min_expected=0x%016llx)",
				(unsigned long long)counter,
				(unsigned long long)runtime->rx_nonce_counter);
			return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
		}
	}

	*isc_id = isc;
	*key_runtime = runtime;
	*received_counter = counter;

	return 0;
}

/*
 * Decrypt in place - MAC || ciphertext in, plaintext out - and pull the
 * resource handle off the front.
 */
static int acs_data_in_decrypt(struct net_buf *buf, uint16_t isc_id,
			       struct bt_acs_key_desc_runtime *key_desc_runtime,
			       uint64_t received_counter, uint16_t *resource_handle)
{
	uint16_t plain_len = 0;
	int err;

	/* The crypto layer updates the receive counter only after authentication. */
	err = acs_crypto_decrypt(key_desc_runtime, received_counter, buf->data, buf->len,
				 &plain_len);
	if (err) {
		if (err == -ENOSPC) {
			/* Do not end the session based on an unauthenticated counter. */
			LOG_WRN("RX nonce space exhausted, rejecting data-in");
			return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
		}

		if (err == -EINVAL) {
			/* The payload does not match the ISC's MAC size (§4.3.2). */
			LOG_ERR("decryption failed: payload shorter than the auth tag");
			return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
		}

		if (err == -EACCES) {
			LOG_ERR("decryption failed: authentication tag mismatch (invalid "
				"key/tampered data)");
		} else {
			LOG_ERR("decryption failed: err=%d (isc_id=0x%04x, key_id=0x%04x)", err,
				isc_id, acs_key_desc_runtime_key_id(key_desc_runtime));
		}

		return ACS_DATA_ERR_INVALID_KEY;
	}

	buf->len = plain_len;

	/* Plaintext must contain a Protected_Resource_Handle (§4.3.2). */
	if (plain_len < ACS_SECURE_DATA_PLAIN_MIN_SIZE) {
		LOG_ERR("decrypted payload too short (%u)", plain_len);
		return ACS_DATA_ERR_RESOURCE_NOT_PROTECTED;
	}

	*resource_handle = net_buf_pull_le16(buf);
	if (*resource_handle == 0U) {
		LOG_ERR("invalid zero resource handle in decrypted payload");
		return ACS_DATA_ERR_RESOURCE_NOT_PROTECTED;
	}

	return 0;
}

int acs_data_in_unwrap_and_route(struct bt_acs_conn *acs_conn, struct net_buf *buf)
{
	struct acs_frame frame;
	struct bt_acs_key_desc_runtime *key_desc_runtime;
	uint64_t received_counter;
	uint16_t isc_id;
	uint16_t resource_handle;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(buf != NULL);

	if (!atomic_test_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED)) {
		return ACS_DATA_ERR_INVALID_KEY;
	}

	err = acs_data_in_validate(acs_conn, buf, &isc_id, &key_desc_runtime, &received_counter);
	if (err) {
		return err;
	}

	err = acs_data_in_decrypt(buf, isc_id, key_desc_runtime, received_counter,
				  &resource_handle);
	if (err != 0) {
		return err;
	}

	frame = (struct acs_frame){
		.resource_handle = resource_handle,
		.isc_id = isc_id,
		.key_runtime = key_desc_runtime,
		.payload = buf->data,
		.payload_len = buf->len,
		.source_channel = ACS_SRC_DATA_IN,
	};

	return acs_runtime_dispatch_frame(&frame, acs_conn);
}
