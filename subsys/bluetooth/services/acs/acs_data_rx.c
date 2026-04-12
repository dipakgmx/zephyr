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
#include "acs_rhandle.h"
#include "acs_rmap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Return codes for acs_process_complete_payload() error mapping */
#define ACS_DATA_ERR_CCC_IMPROPER_CONF         (-EPIPE)
#define ACS_DATA_ERR_NOT_AUTHORIZED            (-EPERM)
#define ACS_DATA_ERR_INVALID_KEY               (-EACCES)
#define ACS_DATA_ERR_RESOURCE_NOT_PROTECTED    (-ENOENT)
#define ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG (-EPROTO)

/* Minimum plaintext size after decryption: Protected_Resource_Handle (2 bytes) */
#define ACS_SECURE_DATA_PLAIN_MIN_SIZE 2

/**
 * @brief Check if a resource handle is listed in the active restriction map.
 *
 * @return true if the handle appears in the map's protected characteristics.
 */
static bool find_handle_cb(const struct bt_acs_rmap_protected *prot, void *user_data)
{
	uint16_t *target = user_data;

	if (prot->resource_handle == *target) {
		*target = 0; /* sentinel: found */
		return false;
	}
	return true;
}

static bool acs_resource_handle_in_map(uint16_t map_id, uint16_t resource_handle)
{
	struct bt_acs_restriction_map map;
	uint16_t target = resource_handle;

	if (acs_rmap_lookup(map_id, &map) != 0) {
		return false;
	}

	acs_rmap_foreach_char(&map, find_handle_cb, &target);
	return target == 0;
}

struct cp_handle_match_ctx {
	uint16_t resource_handle;
	bool found;
};

static bool cp_handle_match_cb(const struct bt_acs_rmap_protected *entry, void *user_data)
{
	struct cp_handle_match_ctx *ctx = user_data;

	if (entry->resource_handle == ctx->resource_handle) {
		ctx->found = true;
		return false; /* stop iteration */
	}
	return true;
}

/* Check if a resource handle is listed in the protected CPs of the active map. */
static bool acs_resource_handle_in_cps(uint16_t map_id, uint16_t resource_handle)
{
	struct bt_acs_restriction_map map;
	struct cp_handle_match_ctx ctx;

	/* ACS_RMAP_FILTER_ALL is a wildcard that always matches. */
	if (resource_handle == ACS_RMAP_FILTER_ALL) {
		return true;
	}

	if (acs_rmap_lookup(map_id, &map) != 0) {
		return false;
	}

	ctx.resource_handle = resource_handle;
	ctx.found = false;
	acs_rmap_foreach_cp(&map, cp_handle_match_cb, &ctx);

	return ctx.found;
}

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

static int acs_require_data_out_subscription(struct bt_conn *conn, uint16_t resource_handle,
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

static int acs_data_in_decrypt(struct acs_data_in_pipeline *pipe)
{
	struct bt_acs_conn *acs_conn = pipe->acs_conn;
	struct net_buf_simple *buf = pipe->buf;
	uint16_t plain_len = 0;
	int err;

	/* buf->data points at MAC(LSO) || Cipher(LSO).
	 * Reversing the whole block in-place gives [Cipher(MSO) || Tag(MSO)]:
	 * exactly the PSA AEAD input format, with no intermediate buffer. */
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

	/* Reverse LSO-first plaintext to recover host byte order. */
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

static int acs_data_in_route(struct acs_data_in_pipeline *pipe)
{
	struct bt_conn *conn = pipe->conn;
	struct bt_acs_conn *acs_conn = pipe->acs_conn;
	struct net_buf_simple *buf = pipe->buf;
	uint16_t resource_handle = pipe->resource_handle;
	uint16_t data_length = pipe->data_length;
	struct bt_acs_prot_resource_req *req_ctx;
	int sub_err;

	/* If handle matches a protected CP entry, route to the CP dispatcher. */
	if (acs_resource_handle_in_cps(pipe->map->map_id, resource_handle)) {
		/* Spec-mandated: reject Data In write if DOI CCC not configured. */
		sub_err = acs_doi_ccc_check(conn);
		if (sub_err == -EINVAL) {
			LOG_WRN("Data In: DOI indications not enabled for protected CP handle "
				"0x%04x",
				resource_handle);
			return ACS_DATA_ERR_CCC_IMPROPER_CONF;
		}
		if (sub_err) {
			LOG_WRN("Data In: DOI unavailable for protected CP handle 0x%04x (%d)",
				resource_handle, sub_err);
			return sub_err;
		}

		LOG_DBG("Data In: routing handle 0x%04x to CP dispatcher (respond via DOI)",
			resource_handle);
		req_ctx = acs_prot_resource_req_alloc(
			acs_conn, resource_handle, pipe->isc_id,
			(uint16_t)(buf->data - acs_conn->data_rx.buf->data), data_length);
		if (!req_ctx) {
			LOG_WRN("Data In: no free CP request context for handle 0x%04x",
				resource_handle);
			return -ENOMEM;
		}

		req_ctx->decrypted_request = acs_conn->data_rx.buf;
		req_ctx->input_owned = true;
		acs_conn->data_rx.buf = NULL;

		acs_cp_dispatch(req_ctx, acs_conn, buf);
		/* If a multi-step reply sequence is active, the sequence now
		 * owns the ALLOC ref and will release it in acs_seq_clear().
		 * Otherwise, drop it here — the single response is already
		 * queued or completed.
		 */
		if (!req_ctx->reply_seq.desc) {
			acs_prot_resource_req_release_owner(req_ctx);
		}
		return 0;
	}

	if (!acs_resource_handle_in_map(pipe->map->map_id, resource_handle)) {
		LOG_WRN("Data In: resource handle 0x%04x not in restriction map", resource_handle);
		return ACS_DATA_ERR_RESOURCE_NOT_PROTECTED;
	}

	sub_err = acs_require_data_out_subscription(conn, resource_handle, data_length);

	if (sub_err == -EINVAL) {
		LOG_WRN("Data In: required Data Out CCC not enabled for handle 0x%04x",
			resource_handle);
		return ACS_DATA_ERR_CCC_IMPROPER_CONF;
	}
	if (sub_err) {
		LOG_WRN("Data In: unable to resolve Data Out path for handle 0x%04x (%d)",
			resource_handle, sub_err);
		return sub_err;
	}

	req_ctx = acs_prot_resource_req_alloc(acs_conn, resource_handle, pipe->isc_id,
					      (uint16_t)(buf->data - acs_conn->data_rx.buf->data),
					      data_length);

	if (!req_ctx) {
		LOG_WRN("Data In: no free request context for handle 0x%04x", resource_handle);
		return -ENOMEM;
	}

	req_ctx->decrypted_request = acs_conn->data_rx.buf;
	req_ctx->input_owned = true;
	acs_conn->data_rx.buf = NULL;

	k_work_submit(&req_ctx->work);

	return 0;
}

static int acs_process_complete_payload(struct bt_conn *conn, struct bt_acs_conn *acs_conn,
					struct net_buf_simple *buf)
{
	struct acs_data_in_pipeline pipe = {
		.conn = conn,
		.acs_conn = acs_conn,
		.buf = buf,
	};
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

	return acs_data_in_route(&pipe);
}

/**
 * @brief Check if this is the first segment of a multi-segment write.
 */
static inline bool is_first_segment(const uint8_t *data)
{
	return (data[0] & ACS_SEG_FIRST_MASK) != 0;
}

/* ATT Write Long not handled; ACS segmentation covers large payloads. */
ssize_t acs_data_in_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags)
{
	struct bt_acs_conn *acs_conn;
	enum acs_seg_rx_result res;
	int proc_err;
	uint16_t attr_handle;

	ARG_UNUSED(flags);

	if (offset != 0) {
		LOG_ERR("Data In write with non-zero offset: %u", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len < 1) {
		LOG_ERR("Data In write with invalid length: %u", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (!acs_is_initialized()) {
		LOG_ERR("Data In write received but ACS not initialized");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	acs_conn = acs_conn_lookup(conn);

	if (!acs_conn) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* Allocate buffer from pool on first segment if not already allocated */
	__ASSERT_NO_MSG(acs_conn->data_rx.buf == NULL || !is_first_segment(buf));
	if (is_first_segment(buf) && !acs_conn->data_rx.buf) {
		struct net_buf *rx_buf = acs_buf_alloc(K_NO_WAIT);

		if (!rx_buf) {
			LOG_ERR("Data In: Failed to allocate buffer from pool");
			return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
		}
		acs_seg_rx_begin(&acs_conn->data_rx, rx_buf);
	}

	if (!acs_conn->data_rx.buf) {
		LOG_ERR("Data In: No buffer for continuation segment");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	res = acs_seg_rx_process(&acs_conn->data_rx, buf, len);

	switch (res) {
	case ACS_SEG_RX_COMPLETE: {
		struct net_buf_simple simple;

		net_buf_simple_init_with_data(&simple, acs_conn->data_rx.buf->data,
					      acs_conn->data_rx.buf->len);

		proc_err = acs_process_complete_payload(conn, acs_conn, &simple);
		attr_handle = bt_gatt_attr_get_handle(attr);

		/*
		 * On success the buffer is either owned by the request context
		 * (regular protected resource) or already freed (protected CP).
		 * On error, release the buffer back to the pool.
		 */
		if (proc_err && acs_conn->data_rx.buf) {
			acs_seg_rx_reset(&acs_conn->data_rx);
		}

		if (proc_err == ACS_DATA_ERR_NOT_AUTHORIZED) {
			LOG_WRN("Data In ATT write to handle 0x%04x: no security established",
				attr_handle);
			return BT_GATT_ERR(BT_ATT_ERR_AUTHORIZATION);
		}
		if (proc_err == ACS_DATA_ERR_INVALID_KEY) {
			LOG_ERR("Data In ATT write to handle 0x%04x failed with Invalid Key; inner "
				"protected resource handle is unavailable unless decryption "
				"succeeds",
				attr_handle);
			return BT_GATT_ERR(BT_ACS_ATT_ERR_INVALID_KEY);
		}
		if (proc_err == ACS_DATA_ERR_CCC_IMPROPER_CONF) {
			LOG_WRN("Data In ATT write to handle 0x%04x: required CCC not configured",
				attr_handle);
			return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
		}
		if (proc_err == ACS_DATA_ERR_RESOURCE_NOT_PROTECTED) {
			LOG_WRN("Data In ATT write to handle 0x%04x: resource handle not protected "
				"by active map",
				attr_handle);
			return BT_GATT_ERR(BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED);
		}
		if (proc_err == ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG) {
			LOG_WRN("Data In ATT write to handle 0x%04x: ISC_ID not found or security "
				"config mismatch",
				attr_handle);
			return BT_GATT_ERR(BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG);
		}
		if (proc_err == -ENOMEM) {
			LOG_WRN("Data In ATT write to handle 0x%04x: no free request context for "
				"protected resource",
				attr_handle);
			return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
		}
		if (proc_err) {
			LOG_ERR("Data In processing failed: %d", proc_err);
			return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
		}
		break;
	}
	case ACS_SEG_RX_FRAGMENT:
		/* More segments expected – nothing to do yet */
		break;
	case ACS_SEG_RX_ERR_COUNTER:
		acs_seg_rx_reset(&acs_conn->data_rx);
		return BT_GATT_ERR(BT_ACS_ATT_ERR_INVALID_SEG_COUNTER);
	case ACS_SEG_RX_ERR_OVERFLOW:
		acs_seg_rx_reset(&acs_conn->data_rx);
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	case ACS_SEG_RX_ERR_TIMEOUT:
		acs_seg_rx_reset(&acs_conn->data_rx);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	case ACS_SEG_RX_ERR_ORPHAN:
	case ACS_SEG_RX_ERR_LEN:
	default:
		acs_seg_rx_reset(&acs_conn->data_rx);
		LOG_ERR("Data In processing failed");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return len;
}
