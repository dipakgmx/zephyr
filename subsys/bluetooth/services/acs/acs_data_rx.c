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

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Minimum plaintext size after decryption: Protected_Resource_Handle (2 bytes) */
#define ACS_SECURE_DATA_PLAIN_MIN_SIZE 2

/**
 * Context for locating a characteristic declaration and value attribute by
 * their ATT handles via bt_gatt_foreach_attr().
 */
struct acs_data_rx_attr_ctx {
	uint16_t value_handle;
	const struct bt_gatt_attr *decl;  /**< declaration attribute (value_handle - 1) */
	const struct bt_gatt_attr *value; /**< value attribute (value_handle) */
};

static uint8_t acs_data_rx_find_char_attrs_cb(const struct bt_gatt_attr *attr, uint16_t handle,
					      void *user_data)
{
	struct acs_data_rx_attr_ctx *ctx = user_data;

	if (handle == ctx->value_handle - 1U) {
		ctx->decl = attr;
	} else if (handle == ctx->value_handle) {
		ctx->value = attr;
	}
	return BT_GATT_ITER_CONTINUE;
}

int acs_require_data_out_subscription(struct bt_conn *conn, uint16_t resource_handle,
				      uint16_t data_length)
{
	struct acs_data_rx_attr_ctx ctx = {
		.value_handle = resource_handle,
		.decl = NULL,
		.value = NULL,
	};
	uint8_t props = 0U;

	if (data_length == 0U) {
		return acs_don_ccc_check(conn);
	}

	bt_gatt_foreach_attr(resource_handle - 1U, resource_handle, acs_data_rx_find_char_attrs_cb,
			     &ctx);
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
				uint16_t *isc_id, struct bt_acs_runtime_key_state **current_key,
				uint32_t *received_counter)
{
	const struct bt_acs_isc_record *isc;
	const uint8_t *nonce_var;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(isc_id != NULL);
	__ASSERT_NO_MSG(current_key != NULL);
	__ASSERT_NO_MSG(received_counter != NULL);

	*received_counter = 0U;

	if (buf->len < ACS_DATA_IN_HDR_SIZE) {
		LOG_ERR("Data In payload too short for ISC_ID");
		return -EINVAL;
	}

	*isc_id = net_buf_simple_pull_le16(buf);

	isc = acs_isc_lookup(*isc_id);
	if (!isc) {
		LOG_WRN("unknown ISC_ID 0x%04x", *isc_id);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	if (acs_crypto_current_key_from_isc(acs_conn, *isc_id, current_key) != 0) {
		LOG_WRN("no current key installed for ISC_ID 0x%04x", *isc_id);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	if ((*current_key)->psa_key_id == 0U) {
		LOG_WRN("key for ISC_ID 0x%04x not provisioned (psa_key_id == 0)", *isc_id);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	/* Wire format after ISC_ID: Nonce_Var(LSO) || MAC(LSO) || Cipher(LSO) */
	if (buf->len <= (uint16_t)(ACS_ACTIVE_NONCE_VAR_SIZE + ACS_ACTIVE_AUTH_TAG_SIZE)) {
		LOG_ERR("Data In: Secure_Data too short (%u)", buf->len);
		return -EINVAL;
	}

	nonce_var = net_buf_simple_pull_mem(buf, ACS_ACTIVE_NONCE_VAR_SIZE);

	sys_get_le(received_counter, nonce_var, sizeof(*received_counter));

	/* Reject stale/replayed messages. */
	if (*received_counter < (*current_key)->rx_nonce_counter) {
		LOG_WRN("Data In: stale/replayed nonce (received=0x%08x vs min_expected=0x%08x)",
			*received_counter, (*current_key)->rx_nonce_counter);
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
 * @param current_key       Runtime key state selected for @p isc_id.
 * @param received_counter  Received nonce-variable counter from the message.
 * @param resource_handle   [out] Decrypted protected resource handle.
 *
 * @return 0 on success, or an ACS Data error / negative errno on failure.
 */
static int acs_data_in_decrypt(struct bt_acs_conn *acs_conn, struct net_buf_simple *buf,
			       uint16_t isc_id, struct bt_acs_runtime_key_state *current_key,
			       uint32_t received_counter, uint16_t *resource_handle)
{
	uint16_t plain_len = 0;
	uint32_t previous_rx_nonce_counter;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(current_key != NULL);
	__ASSERT_NO_MSG(resource_handle != NULL);

	/* Step 1: wire LSO → PSA MSO. */
	sys_mem_swap(buf->data, buf->len);

	/* Decrypt using the received counter as the nonce variable part. Commit the
	 * advanced receive state only if authentication succeeds. */
	previous_rx_nonce_counter = current_key->rx_nonce_counter;
	current_key->rx_nonce_counter = received_counter;
	err = acs_crypto_decrypt(acs_conn, isc_id, buf->data, buf->len, buf->data, &plain_len, NULL,
				 0);
	if (err) {
		current_key->rx_nonce_counter = previous_rx_nonce_counter;
		if (err == -ENOSPC) {
			LOG_WRN("Nonce exhausted on decrypt — invalidating security");
			bt_acs_invalidate_security(acs_conn->conn);
			return ACS_DATA_ERR_INVALID_KEY;
		} else if (err == -EACCES) {
			LOG_ERR("Decryption failed: authentication tag mismatch (invalid "
				"key/tampered data)");
			return ACS_DATA_ERR_INVALID_KEY;
		} else {
			LOG_ERR("Decryption failed: err=%d (isc_id=0x%04x, current_key=0x%04x)",
				err, isc_id, acs_runtime_key_id(current_key));
			return ACS_DATA_ERR_INVALID_KEY;
		}
	}

	/* Step 3: PSA MSO → host LSO. */
	sys_mem_swap(buf->data, plain_len);
	buf->len = plain_len;

	if (plain_len < ACS_SECURE_DATA_PLAIN_MIN_SIZE) {
		LOG_ERR("Data In decrypted payload too short (%u)", plain_len);
		return -EINVAL;
	}

	*resource_handle = net_buf_simple_pull_le16(buf);

	return 0;
}

static void acs_data_in_build_frame(struct bt_acs_conn *acs_conn, struct net_buf_simple *buf,
				    uint16_t isc_id, uint16_t resource_handle,
				    struct acs_frame *frame)
{
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->conn != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(frame != NULL);

	*frame = (struct acs_frame){
		.conn = acs_conn->conn,
		.resource_handle = resource_handle,
		.isc_id = isc_id,
		.payload = buf->data,
		.payload_len = buf->len,
		.source_channel = ACS_SRC_DATA_IN,
		.encrypted = false,
		/* Storage transfer to req_ctx happens inside the runtime
		 * dispatcher; the frame itself does not own the backing buffer.
		 */
		.backing_buf = NULL,
	};
}

int acs_data_in_unwrap_and_route(struct bt_acs_conn *acs_conn, struct net_buf_simple *buf)
{
	struct acs_frame frame;
	struct bt_acs_runtime_key_state *current_key;
	uint32_t received_counter;
	uint16_t isc_id;
	uint16_t resource_handle;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->conn != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(buf->len > 0);

	err = acs_data_in_validate(acs_conn, buf, &isc_id, &current_key, &received_counter);
	if (err) {
		return err;
	}

	err = acs_data_in_decrypt(acs_conn, buf, isc_id, current_key, received_counter,
				  &resource_handle);
	if (err) {
		return err;
	}

	acs_data_in_build_frame(acs_conn, buf, isc_id, resource_handle, &frame);
	return acs_runtime_dispatch_frame(&frame, acs_conn);
}
