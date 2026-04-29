/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <psa/crypto.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "acs_internal.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

enum acs_confirmation_method {
	ACS_CONFIRM_METHOD_NONE = 0x00,
	ACS_CONFIRM_METHOD_OUTPUT_OOB = 0x01,
	ACS_CONFIRM_METHOD_INPUT_OOB = 0x02,
	ACS_CONFIRM_METHOD_STATIC_OOB = 0x03,
};

enum acs_confirmation_action {
	ACS_CONFIRM_ACTION_INPUT_PUSH = 0x00,
	ACS_CONFIRM_ACTION_OUTPUT_BEEP = 0x01,
	ACS_CONFIRM_ACTION_INPUT_NUMERIC = 0x02,
	ACS_CONFIRM_ACTION_OUTPUT_NUMERIC = 0x03,
	ACS_CONFIRM_ACTION_NOT_APPLICABLE = 0xFF,
};

enum {
	ACS_KEY_ID_ECDH = 0x0001,
	ACS_KEY_ID_KDF = 0x0002,
};

static int acs_kex_psa_status_to_errno(psa_status_t status)
{
	switch (status) {
	case PSA_SUCCESS:
		return 0;
	case PSA_ERROR_NOT_SUPPORTED:
		return -ENOTSUP;
	case PSA_ERROR_INVALID_ARGUMENT:
	case PSA_ERROR_INVALID_HANDLE:
	case PSA_ERROR_INVALID_SIGNATURE:
		return -EINVAL;
	case PSA_ERROR_BUFFER_TOO_SMALL:
		return -ENOBUFS;
	case PSA_ERROR_NOT_PERMITTED:
		return -EACCES;
	case PSA_ERROR_DOES_NOT_EXIST:
		return -ENOENT;
	case PSA_ERROR_BAD_STATE:
		return -EALREADY;
	case PSA_ERROR_INSUFFICIENT_MEMORY:
	case PSA_ERROR_INSUFFICIENT_STORAGE:
	case PSA_ERROR_STORAGE_FAILURE:
		return -ENOMEM;
	default:
		return -EIO;
	}
}

static int acs_kex_psa_init(void)
{
	return acs_kex_psa_status_to_errno(psa_crypto_init());
}

static psa_ecc_family_t acs_kex_curve_family(void)
{
#if defined(CONFIG_BT_ACS_ECDH_CURVE_CURVE25519) && CONFIG_BT_ACS_ECDH_CURVE_CURVE25519
	return PSA_ECC_FAMILY_MONTGOMERY;
#else
	return PSA_ECC_FAMILY_SECP_R1;
#endif
}

static size_t acs_kex_curve_bits(void)
{
#if defined(CONFIG_BT_ACS_ECDH_CURVE_P521) && CONFIG_BT_ACS_ECDH_CURVE_P521
	return 521U;
#elif defined(CONFIG_BT_ACS_ECDH_CURVE_P384) && CONFIG_BT_ACS_ECDH_CURVE_P384
	return 384U;
#elif defined(CONFIG_BT_ACS_ECDH_CURVE_CURVE25519) && CONFIG_BT_ACS_ECDH_CURVE_CURVE25519
	return 255U;
#else
	return 256U;
#endif
}

static bool acs_kex_curve_has_y(void)
{
	return CONFIG_BT_ACS_ECDH_HAS_Y != 0;
}

static size_t acs_kex_hash_len(void)
{
#if defined(CONFIG_BT_ACS_KDF_HKDF_SHA512) && CONFIG_BT_ACS_KDF_HKDF_SHA512
	return 64U;
#elif defined(CONFIG_BT_ACS_KDF_HKDF_SHA512_WITH_INFO) && CONFIG_BT_ACS_KDF_HKDF_SHA512_WITH_INFO
	return 64U;
#elif defined(CONFIG_BT_ACS_KDF_HKDF_SHA384) && CONFIG_BT_ACS_KDF_HKDF_SHA384
	return 48U;
#elif defined(CONFIG_BT_ACS_KDF_HKDF_SHA384_WITH_INFO) && CONFIG_BT_ACS_KDF_HKDF_SHA384_WITH_INFO
	return 48U;
#else
	return 32U;
#endif
}

static psa_algorithm_t acs_kex_hkdf_algorithm(void)
{
#if defined(CONFIG_BT_ACS_KDF_HKDF_SHA512) && CONFIG_BT_ACS_KDF_HKDF_SHA512
	return PSA_ALG_HKDF(PSA_ALG_SHA_512);
#elif defined(CONFIG_BT_ACS_KDF_HKDF_SHA512_WITH_INFO) && CONFIG_BT_ACS_KDF_HKDF_SHA512_WITH_INFO
	return PSA_ALG_HKDF(PSA_ALG_SHA_512);
#elif defined(CONFIG_BT_ACS_KDF_HKDF_SHA384) && CONFIG_BT_ACS_KDF_HKDF_SHA384
	return PSA_ALG_HKDF(PSA_ALG_SHA_384);
#elif defined(CONFIG_BT_ACS_KDF_HKDF_SHA384_WITH_INFO) && CONFIG_BT_ACS_KDF_HKDF_SHA384_WITH_INFO
	return PSA_ALG_HKDF(PSA_ALG_SHA_384);
#else
	return PSA_ALG_HKDF(PSA_ALG_SHA_256);
#endif
}

static bool acs_kex_key_supported(uint16_t key_id)
{
	if (key_id == ACS_KEY_ID_ECDH) {
		return IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH);
	}

	if (key_id == ACS_KEY_ID_KDF) {
		return IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF);
	}

	return false;
}

static bool acs_kex_method_action_supported(uint8_t method, uint8_t action)
{
	switch (method) {
	case ACS_CONFIRM_METHOD_NONE:
		return action == ACS_CONFIRM_ACTION_NOT_APPLICABLE;
	case ACS_CONFIRM_METHOD_OUTPUT_OOB:
		if (action == ACS_CONFIRM_ACTION_OUTPUT_BEEP) {
			return IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_BEEP);
		}
		if (action == ACS_CONFIRM_ACTION_OUTPUT_NUMERIC) {
			return IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC);
		}
		return false;
	case ACS_CONFIRM_METHOD_INPUT_OOB:
		if (action == ACS_CONFIRM_ACTION_INPUT_PUSH) {
			return IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_PUSH);
		}
		if (action == ACS_CONFIRM_ACTION_INPUT_NUMERIC) {
			return IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_NUMERIC);
		}
		return false;
	case ACS_CONFIRM_METHOD_STATIC_OOB:
		return action == ACS_CONFIRM_ACTION_NOT_APPLICABLE &&
		       (IS_ENABLED(CONFIG_BT_ACS_OOB_STATIC_NUM_NUMBER) ||
			IS_ENABLED(CONFIG_BT_ACS_OOB_STATIC_NUM_ON_DEVICE));
	default:
		return false;
	}
}

static bool acs_kex_kdf_confirmation_supported(uint16_t key_id, uint8_t method, uint8_t action)
{
	if (key_id != ACS_KEY_ID_KDF) {
		return true;
	}

	return method == ACS_CONFIRM_METHOD_NONE && action == ACS_CONFIRM_ACTION_NOT_APPLICABLE;
}

static int acs_kex_prepare_auth_value(struct acs_conn_ctx *conn_ctx, uint8_t method, uint8_t action)
{
	const struct bt_acs_cb *cb = acs_runtime_callbacks();
	uint32_t oob_num;
	uint16_t oob_len = 0U;
	uint8_t oob_buf[sizeof(conn_ctx->kex.auth_value)];
	int err;

	memset(conn_ctx->kex.auth_value, 0, sizeof(conn_ctx->kex.auth_value));

	switch (method) {
	case ACS_CONFIRM_METHOD_NONE:
		return 0;
	case ACS_CONFIRM_METHOD_OUTPUT_OOB:
		if (!cb || !cb->output_oob_number) {
			return -EIO;
		}
		sys_rand_get(&oob_num, sizeof(oob_num));
#if IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC)
		oob_num = (oob_num % CONFIG_BT_ACS_CONFIRMATION_OUTPUT_MAX_VALUE) + 1U;
#else
		oob_num = (oob_num % 9U) + 1U;
#endif
		sys_put_be32(oob_num,
			     &conn_ctx->kex.auth_value[sizeof(conn_ctx->kex.auth_value) -
							sizeof(oob_num)]);
		cb->output_oob_number(conn_ctx->conn, action, oob_num);
		return 0;
	case ACS_CONFIRM_METHOD_INPUT_OOB:
		if (!cb || !cb->input_oob_request) {
			return -EIO;
		}
		cb->input_oob_request(conn_ctx->conn, action);
		return 0;
	case ACS_CONFIRM_METHOD_STATIC_OOB:
		if (!cb || !cb->static_oob_get) {
			return -EIO;
		}
		err = cb->static_oob_get(conn_ctx->conn, oob_buf, &oob_len);
		if (err || oob_len == 0U || oob_len > sizeof(conn_ctx->kex.auth_value)) {
			return err ? err : -EINVAL;
		}
		memcpy(&conn_ctx->kex.auth_value[sizeof(conn_ctx->kex.auth_value) - oob_len], oob_buf,
		       oob_len);
		return 0;
	default:
		return -EINVAL;
	}
}

static int acs_kex_generate_key_pair(struct acs_kex_ctx *ctx)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	uint8_t exported[1U + (2U * CONFIG_BT_ACS_ECDH_COORD_SIZE)];
	size_t exported_len = 0U;
	psa_status_t status;
	int err;

	err = acs_kex_psa_init();
	if (err) {
		return err;
	}

	psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_DERIVE | PSA_KEY_USAGE_EXPORT);
	psa_set_key_algorithm(&attrs, PSA_ALG_ECDH);
	psa_set_key_type(&attrs, PSA_KEY_TYPE_ECC_KEY_PAIR(acs_kex_curve_family()));
	psa_set_key_bits(&attrs, acs_kex_curve_bits());

	status = psa_generate_key(&attrs, &ctx->private_key_handle);
	psa_reset_key_attributes(&attrs);
	if (status != PSA_SUCCESS) {
		ctx->private_key_handle = 0;
		return acs_kex_psa_status_to_errno(status);
	}

	status = psa_export_public_key(ctx->private_key_handle, exported, sizeof(exported),
				       &exported_len);
	if (status != PSA_SUCCESS) {
		psa_destroy_key(ctx->private_key_handle);
		ctx->private_key_handle = 0;
		return acs_kex_psa_status_to_errno(status);
	}

	ctx->public_coord_len = CONFIG_BT_ACS_ECDH_COORD_SIZE;
	if (acs_kex_curve_has_y()) {
		if (exported_len != (1U + (2U * ctx->public_coord_len)) || exported[0] != 0x04U) {
			psa_destroy_key(ctx->private_key_handle);
			ctx->private_key_handle = 0;
			return -EINVAL;
		}

		memcpy(ctx->local_public_x, &exported[1], ctx->public_coord_len);
		memcpy(ctx->local_public_y, &exported[1 + ctx->public_coord_len],
		       ctx->public_coord_len);
	} else {
		if (exported_len != ctx->public_coord_len) {
			psa_destroy_key(ctx->private_key_handle);
			ctx->private_key_handle = 0;
			return -EINVAL;
		}

		memcpy(ctx->local_public_x, exported, ctx->public_coord_len);
		memset(ctx->local_public_y, 0, sizeof(ctx->local_public_y));
	}

	return 0;
}

static void acs_kex_reverse_copy(uint8_t *dst, const uint8_t *src, size_t len)
{
	for (size_t i = 0U; i < len; i++) {
		dst[i] = src[len - 1U - i];
	}
}

static int acs_kex_parse_wire_public_key(struct acs_kex_ctx *ctx, const uint8_t *operand,
					 uint16_t operand_len, uint16_t key_id)
{
	size_t coord_len = CONFIG_BT_ACS_ECDH_COORD_SIZE;
	size_t needed = 2U + 1U + coord_len + 1U + (acs_kex_curve_has_y() ? coord_len : 0U);
	uint8_t x_len;
	uint8_t y_len;
	size_t index = 0U;

	if (!ctx || !operand || operand_len != needed) {
		return -EINVAL;
	}

	if (sys_get_le16(&operand[index]) != key_id) {
		return -ENOENT;
	}
	index += 2U;

	x_len = operand[index++];
	if (x_len != coord_len) {
		return -EINVAL;
	}

	acs_kex_reverse_copy(ctx->peer_public_x, &operand[index], coord_len);
	index += coord_len;

	y_len = operand[index++];
	if (acs_kex_curve_has_y()) {
		if (y_len != coord_len) {
			return -EINVAL;
		}

		acs_kex_reverse_copy(ctx->peer_public_y, &operand[index], coord_len);
		index += coord_len;
	} else if (y_len != 0U) {
		return -EINVAL;
	}

	ctx->peer_public_coord_len = coord_len;

	if (ctx->peer_public_coord_len == ctx->public_coord_len &&
	    memcmp(ctx->peer_public_x, ctx->local_public_x, ctx->public_coord_len) == 0 &&
	    memcmp(ctx->peer_public_y, ctx->local_public_y, ctx->public_coord_len) == 0) {
		return ACS_KEX_ERR_INVALID_PUBLIC_KEY;
	}

	return index == operand_len ? 0 : -EINVAL;
}

static int acs_kex_compute_shared_secret(struct acs_kex_ctx *ctx)
{
	uint8_t peer_key[1U + (2U * CONFIG_BT_ACS_ECDH_COORD_SIZE)];
	size_t peer_key_len;
	psa_status_t status;

	if (!ctx || ctx->private_key_handle == 0 || ctx->peer_public_coord_len == 0U) {
		return -EINVAL;
	}

	if (acs_kex_curve_has_y()) {
		peer_key[0] = 0x04U;
		memcpy(&peer_key[1], ctx->peer_public_x, ctx->peer_public_coord_len);
		memcpy(&peer_key[1 + ctx->peer_public_coord_len], ctx->peer_public_y,
		       ctx->peer_public_coord_len);
		peer_key_len = 1U + (2U * ctx->peer_public_coord_len);
	} else {
		memcpy(peer_key, ctx->peer_public_x, ctx->peer_public_coord_len);
		peer_key_len = ctx->peer_public_coord_len;
	}

	status = psa_raw_key_agreement(PSA_ALG_ECDH, ctx->private_key_handle, peer_key, peer_key_len,
					 ctx->shared_secret, sizeof(ctx->shared_secret),
					 &ctx->shared_secret_len);
	if (status == PSA_ERROR_INVALID_ARGUMENT || status == PSA_ERROR_INVALID_HANDLE ||
	    status == PSA_ERROR_INVALID_SIGNATURE) {
		return ACS_KEX_ERR_INVALID_PUBLIC_KEY;
	}

	return acs_kex_psa_status_to_errno(status);
}

static int acs_kex_hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *msg,
			       size_t msg_len, uint8_t out[32])
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t handle = 0;
	size_t out_len = 0U;
	psa_status_t status;
	int err;

	if (!key || !msg || !out) {
		return -EINVAL;
	}

	err = acs_kex_psa_init();
	if (err) {
		return err;
	}

	psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_SIGN_MESSAGE);
	psa_set_key_algorithm(&attrs, PSA_ALG_HMAC(PSA_ALG_SHA_256));
	psa_set_key_type(&attrs, PSA_KEY_TYPE_HMAC);
	psa_set_key_bits(&attrs, key_len * 8U);

	status = psa_import_key(&attrs, key, key_len, &handle);
	psa_reset_key_attributes(&attrs);
	if (status != PSA_SUCCESS) {
		return acs_kex_psa_status_to_errno(status);
	}

	status = psa_mac_compute(handle, PSA_ALG_HMAC(PSA_ALG_SHA_256), msg, msg_len, out, 32U,
				 &out_len);
	psa_destroy_key(handle);
	if (status != PSA_SUCCESS || out_len != 32U) {
		return status == PSA_SUCCESS ? -EIO : acs_kex_psa_status_to_errno(status);
	}

	return 0;
}

static int acs_kex_build_confirmation_key(struct acs_kex_ctx *ctx, uint8_t out_key[32])
{
	uint8_t zero32[32] = { 0 };
	uint8_t salt[32];
	uint8_t pub_concat[4U * CONFIG_BT_ACS_ECDH_COORD_SIZE];
	uint8_t conf_input[CONFIG_BT_ACS_SESSION_KEY_SIZE + sizeof(ctx->auth_value)];
	int err;

	if (!ctx || !out_key || !ctx->parent_key_valid || ctx->peer_public_coord_len == 0U ||
	    ctx->public_coord_len == 0U) {
		return -EINVAL;
	}

	memcpy(pub_concat, ctx->local_public_x, ctx->public_coord_len);
	memcpy(&pub_concat[ctx->public_coord_len], ctx->local_public_y, ctx->public_coord_len);
	memcpy(&pub_concat[2U * ctx->public_coord_len], ctx->peer_public_x, ctx->peer_public_coord_len);
	memcpy(&pub_concat[(2U * ctx->public_coord_len) + ctx->peer_public_coord_len], ctx->peer_public_y,
	       ctx->peer_public_coord_len);

	err = acs_kex_hmac_sha256(zero32, sizeof(zero32), pub_concat,
				  2U * (ctx->public_coord_len + ctx->peer_public_coord_len), salt);
	if (err) {
		return err;
	}

	memcpy(conf_input, ctx->parent_key, sizeof(ctx->parent_key));
	memcpy(&conf_input[sizeof(ctx->parent_key)], ctx->auth_value, sizeof(ctx->auth_value));

	return acs_kex_hmac_sha256(salt, sizeof(salt), conf_input, sizeof(conf_input), out_key);
}

static int acs_kex_calculate_confirm_wire(struct acs_kex_ctx *ctx, const uint8_t random_be[32],
					  uint8_t confirm_wire[32])
{
	uint8_t confirmation_key[32];
	uint8_t confirm_be[32];
	int err;

	err = acs_kex_build_confirmation_key(ctx, confirmation_key);
	if (err) {
		return err;
	}

	err = acs_kex_hmac_sha256(confirmation_key, sizeof(confirmation_key), random_be, 32U,
				  confirm_be);
	if (err) {
		return err;
	}

	acs_kex_reverse_copy(confirm_wire, confirm_be, 32U);
	return 0;
}

static int acs_kex_generate_kdf_material(struct acs_kex_ctx *ctx)
{
	static const uint8_t default_info[] = "tidepool";

	if (!ctx) {
		return -EINVAL;
	}

	ctx->kdf_salt_len = MIN(sizeof(ctx->kdf_salt), acs_kex_hash_len());
	sys_rand_get(ctx->kdf_salt, ctx->kdf_salt_len);

	if (IS_ENABLED(CONFIG_BT_ACS_HAS_INFO_FOR_HKDF)) {
		ctx->kdf_info_len = MIN(sizeof(default_info) - 1U, sizeof(ctx->kdf_info));
		memcpy(ctx->kdf_info, default_info, ctx->kdf_info_len);
	} else {
		ctx->kdf_info_len = 0U;
	}

	return 0;
}

static int acs_kex_derive_hkdf(const uint8_t *secret, size_t secret_len,
			       const struct acs_kex_ctx *ctx, uint8_t *out_key, size_t out_len)
{
	psa_key_derivation_operation_t op = PSA_KEY_DERIVATION_OPERATION_INIT;
	psa_status_t status;
	int err;

	if (!secret || !ctx || !out_key) {
		return -EINVAL;
	}

	err = acs_kex_psa_init();
	if (err) {
		return err;
	}

	status = psa_key_derivation_setup(&op, acs_kex_hkdf_algorithm());
	if (status == PSA_SUCCESS) {
		status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_SALT,
							ctx->kdf_salt, ctx->kdf_salt_len);
	}
	if (status == PSA_SUCCESS) {
		status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_INFO,
							ctx->kdf_info, ctx->kdf_info_len);
	}
	if (status == PSA_SUCCESS) {
		status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_SECRET,
							secret, secret_len);
	}
	if (status == PSA_SUCCESS) {
		status = psa_key_derivation_output_bytes(&op, out_key, out_len);
	}

	psa_key_derivation_abort(&op);
	return acs_kex_psa_status_to_errno(status);
}

void acs_kex_reset(struct acs_kex_ctx *ctx)
{
	if (!ctx) {
		return;
	}

	if (ctx->private_key_handle != 0) {
		psa_destroy_key(ctx->private_key_handle);
	}

	memset(ctx, 0, sizeof(*ctx));
	ctx->state = ACS_KEX_IDLE;
}

int acs_kex_start(struct acs_conn_ctx *conn_ctx, uint16_t key_id, uint8_t confirmation_method,
		  uint8_t confirmation_action)
{
	struct acs_kex_ctx saved = { 0 };
	bool preserve_parent;
	int err;

	if (!conn_ctx) {
		return -EINVAL;
	}

	if (!acs_kex_key_supported(key_id)) {
		return -ENOENT;
	}

	if (!acs_kex_method_action_supported(confirmation_method, confirmation_action)) {
		return -EINVAL;
	}

	if (!acs_kex_kdf_confirmation_supported(key_id, confirmation_method,
						 confirmation_action)) {
		return -EAGAIN;
	}

	if (conn_ctx->kex.state != ACS_KEX_IDLE && conn_ctx->kex.state != ACS_KEX_COMPLETE) {
		return -EALREADY;
	}

	if (key_id == ACS_KEY_ID_KDF && !conn_ctx->kex.parent_key_valid) {
		return -EAGAIN;
	}

	preserve_parent = (key_id == ACS_KEY_ID_KDF && conn_ctx->kex.parent_key_valid);
	if (preserve_parent) {
		memcpy(saved.parent_key, conn_ctx->kex.parent_key, sizeof(saved.parent_key));
		saved.parent_key_valid = conn_ctx->kex.parent_key_valid;
	}

	acs_kex_reset(&conn_ctx->kex);
	if (preserve_parent) {
		memcpy(conn_ctx->kex.parent_key, saved.parent_key, sizeof(saved.parent_key));
		conn_ctx->kex.parent_key_valid = saved.parent_key_valid;
	}

	conn_ctx->kex.key_id = key_id;
	conn_ctx->kex.confirmation_method = confirmation_method;
	conn_ctx->kex.confirmation_action = confirmation_action;

	err = acs_kex_prepare_auth_value(conn_ctx, confirmation_method, confirmation_action);
	if (err) {
		acs_kex_reset(&conn_ctx->kex);
		return err;
	}

	if (key_id == ACS_KEY_ID_ECDH) {
		err = acs_kex_generate_key_pair(&conn_ctx->kex);
		if (err) {
			acs_kex_reset(&conn_ctx->kex);
			return err;
		}
	}

	conn_ctx->kex.state = ACS_KEX_STARTED;
	LOG_DBG("kex start armed: key_id=0x%04x method=0x%02x action=0x%02x", key_id,
		confirmation_method, confirmation_action);
	return 0;
}

int acs_kex_build_ecdh_response(struct acs_conn_ctx *conn_ctx, uint16_t key_id,
				const uint8_t *operand, uint16_t operand_len,
				struct net_buf *response_buf)
{
	struct acs_kex_ctx *ctx;
	int err;

	if (!conn_ctx || !response_buf) {
		return -EINVAL;
	}

	ctx = &conn_ctx->kex;
	if (ctx->state != ACS_KEX_STARTED || ctx->key_id != ACS_KEY_ID_ECDH ||
	    key_id != ACS_KEY_ID_ECDH) {
		return -EALREADY;
	}

	err = acs_kex_parse_wire_public_key(ctx, operand, operand_len, key_id);
	if (err) {
		return err;
	}

	err = acs_kex_compute_shared_secret(ctx);
	if (err) {
		return err;
	}

	if (net_buf_tailroom(response_buf) <
	    (size_t)(2U + 1U + ctx->public_coord_len + 1U +
		     (acs_kex_curve_has_y() ? ctx->public_coord_len : 0U))) {
		return -ENOMEM;
	}

	net_buf_add_le16(response_buf, key_id);
	net_buf_add_u8(response_buf, ctx->public_coord_len);
	for (size_t i = 0U; i < ctx->public_coord_len; i++) {
		net_buf_add_u8(response_buf, ctx->local_public_x[ctx->public_coord_len - 1U - i]);
	}
	net_buf_add_u8(response_buf, acs_kex_curve_has_y() ? ctx->public_coord_len : 0U);
	if (acs_kex_curve_has_y()) {
		for (size_t i = 0U; i < ctx->public_coord_len; i++) {
			net_buf_add_u8(response_buf,
				       ctx->local_public_y[ctx->public_coord_len - 1U - i]);
		}
	}

	ctx->state = ACS_KEX_PUBKEY_EXCHANGED;
	LOG_DBG("kex ecdh: shared secret ready (%u bytes)", (unsigned int)ctx->shared_secret_len);
	return 0;
}

int acs_kex_build_kdf_response(struct acs_conn_ctx *conn_ctx, uint16_t key_id,
			       struct net_buf *response_buf, bool *send_final_response)
{
	struct acs_kex_ctx *ctx;
	const uint8_t *secret;
	size_t secret_len;
	int err;

	if (!conn_ctx || !response_buf || !send_final_response) {
		return -EINVAL;
	}

	ctx = &conn_ctx->kex;
	if (ctx->state != ACS_KEX_STARTED && ctx->state != ACS_KEX_PUBKEY_EXCHANGED &&
	    ctx->state != ACS_KEX_COMPLETE && ctx->state != ACS_KEX_KDF_DONE) {
		return -EALREADY;
	}
	if (ctx->key_id != key_id) {
		return -ENOENT;
	}

	err = acs_kex_generate_kdf_material(ctx);
	if (err) {
		return err;
	}

	if (key_id == ACS_KEY_ID_ECDH) {
		if (ctx->shared_secret_len == 0U) {
			return -EAGAIN;
		}
		secret = ctx->shared_secret;
		secret_len = ctx->shared_secret_len;
		err = acs_kex_derive_hkdf(secret, secret_len, ctx, ctx->parent_key,
					  sizeof(ctx->parent_key));
		if (err) {
			return err;
		}
		ctx->parent_key_valid = true;
		ctx->state = ACS_KEX_KDF_DONE;
		*send_final_response = false;
		LOG_DBG("kex kdf: parent key derived for key_id=0x%04x", key_id);
	} else if (key_id == ACS_KEY_ID_KDF) {
		if (!ctx->parent_key_valid) {
			return -EAGAIN;
		}
		secret = ctx->parent_key;
		secret_len = sizeof(ctx->parent_key);
		err = acs_kex_derive_hkdf(secret, secret_len, ctx, ctx->session_key,
					  sizeof(ctx->session_key));
		if (err) {
			return err;
		}
		ctx->session_key_valid = true;
		ctx->state = ACS_KEX_COMPLETE;
		*send_final_response = true;
		LOG_DBG("kex kdf: session key derived for key_id=0x%04x", key_id);
	} else {
		return -ENOENT;
	}

	if (net_buf_tailroom(response_buf) < (size_t)(2U + 1U + ctx->kdf_salt_len + 1U +
						      ctx->kdf_info_len)) {
		return -ENOMEM;
	}

	net_buf_add_le16(response_buf, key_id);
	net_buf_add_u8(response_buf, ctx->kdf_salt_len);
	for (size_t i = 0U; i < ctx->kdf_salt_len; i++) {
		net_buf_add_u8(response_buf, ctx->kdf_salt[ctx->kdf_salt_len - 1U - i]);
	}
	net_buf_add_u8(response_buf, ctx->kdf_info_len);
	for (size_t i = 0U; i < ctx->kdf_info_len; i++) {
		net_buf_add_u8(response_buf, ctx->kdf_info[ctx->kdf_info_len - 1U - i]);
	}

	return 0;
}

int acs_kex_build_confirmation_code_response(struct acs_conn_ctx *conn_ctx, uint16_t key_id,
					     const uint8_t *operand, uint16_t operand_len,
					     struct net_buf *response_buf)
{
	struct acs_kex_ctx *ctx;
	uint8_t confirm_wire[32];
	int err;

	if (!conn_ctx || !operand || !response_buf) {
		return -EINVAL;
	}

	ctx = &conn_ctx->kex;
	if (ctx->state != ACS_KEX_KDF_DONE || ctx->key_id != ACS_KEY_ID_ECDH ||
	    key_id != ACS_KEY_ID_ECDH) {
		return -EALREADY;
	}

	if (operand_len != (2U + 32U) || sys_get_le16(operand) != key_id) {
		return -EINVAL;
	}

	memcpy(ctx->received_confirm_code, &operand[2], 32U);
	ctx->received_confirm_code_len = 32U;
	ctx->server_random_len = 32U;
	sys_rand_get(ctx->server_random, ctx->server_random_len);

	err = acs_kex_calculate_confirm_wire(ctx, ctx->server_random, confirm_wire);
	if (err) {
		return err;
	}

	if (net_buf_tailroom(response_buf) < (size_t)(2U + sizeof(confirm_wire))) {
		return -ENOMEM;
	}

	net_buf_add_le16(response_buf, key_id);
	net_buf_add_mem(response_buf, confirm_wire, sizeof(confirm_wire));
	LOG_DBG("kex confirm: server confirmation code prepared");
	return 0;
}

int acs_kex_build_confirmation_random_response(struct acs_conn_ctx *conn_ctx, uint16_t key_id,
					       const uint8_t *operand, uint16_t operand_len,
					       struct net_buf *response_buf,
					       bool *send_final_response)
{
	struct acs_kex_ctx *ctx;
	uint8_t client_random_be[32];
	uint8_t expected_confirm_wire[32];
	uint8_t server_random_wire[32];
	int err;

	if (!conn_ctx || !operand || !response_buf || !send_final_response) {
		return -EINVAL;
	}

	ctx = &conn_ctx->kex;
	if (ctx->state != ACS_KEX_KDF_DONE || ctx->key_id != ACS_KEY_ID_ECDH ||
	    key_id != ACS_KEY_ID_ECDH) {
		return -EALREADY;
	}

	if (operand_len != (2U + 32U) || sys_get_le16(operand) != key_id ||
	    ctx->received_confirm_code_len != 32U || ctx->server_random_len != 32U) {
		return -EINVAL;
	}

	acs_kex_reverse_copy(client_random_be, &operand[2], sizeof(client_random_be));
	if (memcmp(client_random_be, ctx->server_random, sizeof(client_random_be)) == 0) {
		return -EINVAL;
	}

	err = acs_kex_calculate_confirm_wire(ctx, client_random_be, expected_confirm_wire);
	if (err) {
		return err;
	}

	if (memcmp(expected_confirm_wire, ctx->received_confirm_code, sizeof(expected_confirm_wire)) !=
	    0) {
		return -EACCES;
	}

	acs_kex_reverse_copy(server_random_wire, ctx->server_random, sizeof(server_random_wire));
	if (net_buf_tailroom(response_buf) < (size_t)(2U + sizeof(server_random_wire))) {
		return -ENOMEM;
	}

	net_buf_add_le16(response_buf, key_id);
	net_buf_add_mem(response_buf, server_random_wire, sizeof(server_random_wire));
	ctx->state = ACS_KEX_COMPLETE;
	*send_final_response = true;
	LOG_DBG("kex confirm: client confirmation validated");
	return 0;
}
