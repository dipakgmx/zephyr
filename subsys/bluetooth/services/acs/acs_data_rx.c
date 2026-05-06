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

/** Processing context threaded through the Data In pipeline stages. */
struct acs_data_in_pipeline {
	struct bt_conn *conn;
	struct bt_acs_conn *acs_conn;
	struct net_buf_simple *buf;

	/* Populated by validate */
	uint16_t isc_id;
	const struct bt_acs_restriction_map *map;
	uint32_t expected_min_counter;
	uint32_t received_counter;

	/* Populated by decrypt */
	uint16_t resource_handle;
	uint16_t data_length;
};

static int acs_data_in_validate(struct acs_data_in_pipeline *pipe)
{
	struct bt_acs_conn *acs_conn = pipe->acs_conn;
	struct net_buf_simple *buf = pipe->buf;
	const uint8_t *nonce_var;

	if (acs_conn->key_state != BT_ACS_KEY_EXCHANGE_COMPLETE) {
		LOG_WRN("Data In: no security established — rejecting with Insufficient "
			"Authorization");
		return ACS_DATA_ERR_NOT_AUTHORIZED;
	}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	/* The ISC for protected resources references the KDF child key (spec Figure 4.4:
	 * algorithm records point to the KDF key, not the ECDH parent).  The child key only
	 * exists after the peer has completed the KDF exchange for this connection.
	 * Spec §4.4.4.15.1.4.4.1: "the key has not been exchanged yet" → Invalid Key. */
	if (!acs_conn->kdf_child_active) {
		LOG_WRN("Data In: KDF child key not yet established — Invalid Key");
		return ACS_DATA_ERR_INVALID_KEY;
	}
#endif

	if (buf->len < ACS_DATA_IN_HDR_SIZE) {
		LOG_ERR("Data In payload too short for ISC_ID");
		return -EINVAL;
	}

	pipe->isc_id = net_buf_simple_pull_le16(buf);

	/* Find the restriction map whose map_isc_id matches the ISC_ID in the payload. */
	pipe->map = NULL;
	STRUCT_SECTION_FOREACH(bt_acs_restriction_map, m) {
		if (m->map_isc_id == pipe->isc_id) {
			pipe->map = m;
			break;
		}
	}

	if (!pipe->map) {
		LOG_WRN("Data In: ISC_ID 0x%04x not found in any restriction map", pipe->isc_id);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	/* Wire format after ISC_ID: Nonce_Var(LSO) || MAC(LSO) || Cipher(LSO) */
	if (buf->len <= (uint16_t)(ACS_ACTIVE_NONCE_VAR_SIZE + ACS_ACTIVE_AUTH_TAG_SIZE)) {
		LOG_ERR("Data In: Secure_Data too short (%u)", buf->len);
		return -EINVAL;
	}

	nonce_var = net_buf_simple_pull_mem(buf, ACS_ACTIVE_NONCE_VAR_SIZE);

	/* Extract the received counter from Nonce_Var */
	for (size_t i = 0; i < MIN(sizeof(pipe->received_counter), ACS_ACTIVE_NONCE_VAR_SIZE);
	     i++) {
		pipe->received_counter |= (uint32_t)nonce_var[i] << (8U * i);
	}
	pipe->expected_min_counter = acs_conn->crypto.rx_nonce_counter;

	/* Reject stale/replayed messages. */
	if (pipe->received_counter < pipe->expected_min_counter) {
		LOG_WRN("Data In: stale/replayed nonce "
			"(received=0x%08x vs min_expected=0x%08x)",
			pipe->received_counter, pipe->expected_min_counter);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	/* Ensure Nonce_Var fits within the representation of the RX counter. */
	for (size_t i = sizeof(acs_conn->crypto.rx_nonce_counter); i < ACS_ACTIVE_NONCE_VAR_SIZE;
	     i++) {
		if (nonce_var[i] != 0U) {
			LOG_ERR("Data In: Nonce_Variable exceeds 32-bit counter representation");
			return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
		}
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
 * Each swap is the inverse of the previous step over the relevant prefix; the
 * three together convert wire bytes to host plaintext without any extra copy.
 */
static int acs_data_in_decrypt(struct acs_data_in_pipeline *pipe)
{
	struct bt_acs_conn *acs_conn = pipe->acs_conn;
	struct net_buf_simple *buf = pipe->buf;
	uint16_t plain_len = 0;
	int err;

	/* Step 1: wire LSO → PSA MSO. */
	sys_mem_swap(buf->data, buf->len);

	/* Decrypt using the received counter as the nonce variable part. Commit the
	 * advanced receive state only if authentication succeeds. */
	acs_conn->crypto.rx_nonce_counter = pipe->received_counter;
	err = acs_crypto_decrypt(acs_conn, pipe->isc_id, buf->data, buf->len, buf->data, &plain_len,
				 NULL, 0);
	if (err) {
		acs_conn->crypto.rx_nonce_counter = pipe->expected_min_counter;
		if (err == -ENOSPC) {
			LOG_WRN("Nonce exhausted on decrypt — invalidating security");
			bt_acs_invalidate_security(acs_conn->conn);
			return ACS_DATA_ERR_INVALID_KEY;
		} else if (err == -EACCES) {
			LOG_ERR("Decryption failed: authentication tag mismatch (invalid "
				"key/tampered data)");
			return ACS_DATA_ERR_INVALID_KEY;
		} else {
			LOG_ERR("Decryption failed: err=%d (isc_id=0x%04x, map_id=0x%04x)", err,
				pipe->isc_id, pipe->map->map_id);
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

	pipe->resource_handle = net_buf_simple_pull_le16(buf);
	pipe->data_length = buf->len;

	return 0;
}

/* Populate @p frame from a validated+decrypted Data In pipeline. */
static void acs_data_in_build_frame(const struct acs_data_in_pipeline *pipe, struct acs_frame *frame)
{
	*frame = (struct acs_frame){
		.conn = pipe->conn,
		.resource_handle = pipe->resource_handle,
		.isc_id = pipe->isc_id,
		.payload = pipe->buf->data,
		.payload_len = pipe->buf->len,
		.source_channel = ACS_SRC_DATA_IN,
		.encrypted = false,
		/* Storage transfer to req_ctx happens inside the runtime
		 * dispatcher; the frame itself does not own the backing buffer.
		 */
		.backing_buf = NULL,
	};
}

int acs_data_in_unwrap_and_route(struct bt_conn *conn, struct bt_acs_conn *acs_conn,
				 struct net_buf_simple *buf)
{
	struct acs_data_in_pipeline pipe = {
		.conn = conn,
		.acs_conn = acs_conn,
		.buf = buf,
	};
	struct acs_frame frame;
	int err;

	__ASSERT_NO_MSG(conn != NULL);
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(buf->len > 0);

	err = acs_data_in_validate(&pipe);
	if (err) {
		return err;
	}

	err = acs_data_in_decrypt(&pipe);
	if (err) {
		return err;
	}

	acs_data_in_build_frame(&pipe, &frame);
	return acs_runtime_dispatch_frame(&frame, acs_conn);
}
