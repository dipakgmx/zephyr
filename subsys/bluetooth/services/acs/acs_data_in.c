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
#include "acs_isc.h"
#include "acs_rhandle.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Minimum plaintext size after decryption: Protected_Resource_Handle (2 bytes) */
#define ACS_SECURE_DATA_PLAIN_MIN_SIZE 2

static bool
acs_nonce_var_matches_runtime_prefix(const uint8_t *nonce_var,
				     const struct bt_acs_key_desc_runtime *key_desc_runtime)
{
	const struct bt_acs_key_desc_record *key_desc = key_desc_runtime->key_desc;
	uint8_t counter_size = acs_key_desc_nonce_counter_size(key_desc);
	uint8_t prefix_size = acs_key_desc_nonce_prefix_size(key_desc);
	uint8_t expected_prefix[ACS_MAX_NONCE_FIXED_SIZE];

	if (key_desc->aes.nonce_type != ACS_NONCE_SEQ_EVEN_ODD || prefix_size == 0U) {
		return true;
	}

	memcpy(expected_prefix, key_desc_runtime->client_nonce_fixed, prefix_size);
	sys_mem_swap(expected_prefix, prefix_size);

	return memcmp(&nonce_var[counter_size], expected_prefix, prefix_size) == 0;
}

int acs_require_data_out_subscription(struct bt_conn *conn, uint16_t resource_handle,
				      uint16_t data_length)
{
	struct acs_char_attr_ctx ctx = {
		.value_handle = resource_handle,
		.decl = NULL,
		.value = NULL,
	};
	uint8_t props = 0U;

	if (data_length == 0U) {
		return acs_don_ccc_check(conn);
	}

	bt_gatt_foreach_attr(resource_handle - 1U, resource_handle, acs_find_char_attrs_cb, &ctx);
	if (!ctx.value) {
		return -ENOENT;
	}

	if (ctx.decl && ctx.decl->user_data) {
		props = ((const struct bt_gatt_chrc *)ctx.decl->user_data)->properties;
	}

	if (props & BT_GATT_CHRC_INDICATE) {
		return acs_doi_ccc_check(conn);
	}

	return acs_don_ccc_check(conn);
}

static int acs_data_in_validate(struct bt_acs_conn *acs_conn, struct net_buf_simple *buf,
				uint16_t *isc_id, struct bt_acs_key_desc_runtime **record_state,
				uint64_t *received_counter)
{
	const struct bt_acs_isc_record *isc;
	const struct bt_acs_key_desc_record *key_desc;
	const uint8_t *nonce_var;
	uint8_t nonce_var_size;
	uint8_t auth_tag_size;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(isc_id != NULL);
	__ASSERT_NO_MSG(record_state != NULL);
	__ASSERT_NO_MSG(received_counter != NULL);

	*received_counter = 0U;

	if (buf->len < ACS_DATA_IN_HDR_SIZE) {
		LOG_ERR("data-in payload too short for ISC_ID");
		return -EINVAL;
	}

	*isc_id = net_buf_simple_pull_le16(buf);

	isc = acs_isc_lookup(*isc_id);
	if (!isc) {
		LOG_WRN("unknown ISC_ID 0x%04x", *isc_id);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	key_desc = acs_key_desc_lookup(isc->key_id);
	if (!key_desc) {
		LOG_WRN("unknown key descriptor 0x%04x for ISC_ID 0x%04x", isc->key_id, *isc_id);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	if (acs_crypto_key_desc_runtime_lookup(acs_conn, isc->key_id, record_state) != 0) {
		LOG_WRN("no record state installed for ISC_ID 0x%04x", *isc_id);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	if ((*record_state)->psa_key_id == 0U) {
		LOG_WRN("key for ISC_ID 0x%04x not provisioned (psa_key_id == 0)", *isc_id);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	nonce_var_size = acs_key_desc_nonce_var_size(key_desc);
	auth_tag_size = acs_key_desc_auth_tag_size(key_desc);

	/* Wire format after ISC_ID: Nonce_Var(LSO) || MAC(LSO) || Cipher(LSO) */
	if (buf->len <= (uint16_t)(nonce_var_size + auth_tag_size)) {
		LOG_ERR("secure data too short (%u)", buf->len);
		return -EINVAL;
	}

	nonce_var = net_buf_simple_pull_mem(buf, nonce_var_size);
	sys_get_le(received_counter, nonce_var, acs_key_desc_nonce_counter_size(key_desc));

	if (!acs_nonce_var_matches_runtime_prefix(nonce_var, *record_state)) {
		LOG_WRN("received nonce variable prefix does not match runtime state");
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	/* Reject stale/replayed messages. */
	if (*received_counter < (*record_state)->rx_nonce_counter) {
		LOG_WRN("stale or replayed nonce (received=0x%016llx vs "
			"min_expected=0x%016llx)",
			(unsigned long long)*received_counter,
			(unsigned long long)(*record_state)->rx_nonce_counter);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	return 0;
}

/*
 * Byte-order dance — done in three steps because PSA AEAD demands MSO input/output
 * but the wire format is LSO. We avoid an intermediate buffer by swapping in place:
 *
 *   1. swap(buf, buf->len)          : wire LSO    → PSA MSO   (input to PSA)
 *   2. acs_crypto_decrypt           : PSA produces MSO plaintext over the same bytes
 *   3. swap(buf, plain_len)         : PSA MSO     → host LSO  (output to caller)
 *
 * @brief Decrypt a validated Data In payload and extract its resource handle.
 *
 * @param acs_conn          Per-connection ACS state.
 * @param buf               Buffer positioned at MAC || ciphertext on entry.
 *                          Replaced in place with plaintext payload on success.
 * @param isc_id            Information Security Configuration ID for decryption.
 * @param record_state      Runtime record state selected for @p isc_id.
 * @param received_counter  Received nonce-variable counter from the message.
 * @param resource_handle   [out] Decrypted protected resource handle.
 *
 * @return 0 on success, or an ACS Data error / negative errno on failure.
 */
static int acs_data_in_decrypt(struct bt_acs_conn *acs_conn, struct net_buf_simple *buf,
			       uint16_t isc_id, struct bt_acs_key_desc_runtime *record_state,
			       uint64_t received_counter, uint16_t *resource_handle)
{
	uint16_t plain_len = 0;
	uint64_t previous_rx_nonce_counter;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(record_state != NULL);
	__ASSERT_NO_MSG(resource_handle != NULL);

	/* Step 1: wire LSO → PSA MSO. */
	sys_mem_swap(buf->data, buf->len);

	/* Decrypt using the received counter as the nonce variable part. Commit the
	 * advanced receive state only if authentication succeeds. */
	previous_rx_nonce_counter = record_state->rx_nonce_counter;
	record_state->rx_nonce_counter = received_counter;
	err = acs_crypto_decrypt(record_state, buf->data, buf->len, buf->data, &plain_len, NULL, 0);
	if (err) {
		record_state->rx_nonce_counter = previous_rx_nonce_counter;
		if (err == -ENOSPC) {
			LOG_WRN("nonce exhausted on decrypt, invalidating security");
			bt_acs_invalidate_security(acs_conn->conn);
			return ACS_DATA_ERR_INVALID_KEY;
		} else if (err == -EACCES) {
			LOG_ERR("decryption failed: authentication tag mismatch (invalid "
				"key/tampered data)");
			return ACS_DATA_ERR_INVALID_KEY;
		} else {
			LOG_ERR("decryption failed: err=%d (isc_id=0x%04x, key_id=0x%04x)", err,
				isc_id, acs_key_desc_runtime_key_id(record_state));
			return ACS_DATA_ERR_INVALID_KEY;
		}
	}

	/* Step 3: PSA MSO → host LSO. */
	sys_mem_swap(buf->data, plain_len);
	buf->len = plain_len;

	if (plain_len < ACS_SECURE_DATA_PLAIN_MIN_SIZE) {
		LOG_ERR("decrypted payload too short (%u)", plain_len);
		return -EINVAL;
	}

	*resource_handle = net_buf_simple_pull_le16(buf);

	return 0;
}

int acs_data_in_unwrap_and_route(struct bt_acs_conn *acs_conn, struct net_buf_simple *buf)
{
	struct acs_frame frame;
	struct bt_acs_key_desc_runtime *record_state;
	uint64_t received_counter;
	uint16_t isc_id;
	uint16_t resource_handle;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->conn != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(buf->len > 0);

	if (!acs_session_established(acs_conn)) {
		return ACS_DATA_ERR_NOT_AUTHORIZED;
	}

	err = acs_data_in_validate(acs_conn, buf, &isc_id, &record_state, &received_counter);
	if (err) {
		return err;
	}

	err = acs_data_in_decrypt(acs_conn, buf, isc_id, record_state, received_counter,
				  &resource_handle);
	if (err) {
		return err;
	}

	frame = (struct acs_frame){
		.conn = acs_conn->conn,
		.resource_handle = resource_handle,
		.isc_id = isc_id,
		.payload = buf->data,
		.payload_len = buf->len,
		.source_channel = ACS_SRC_DATA_IN,
		.encrypted = false,
		.backing_buf = NULL,
	};

	return acs_runtime_dispatch_frame(&frame, acs_conn);
}
