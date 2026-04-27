/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <psa/crypto.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "acs_crypto.h"
#include "acs_wire_constants.h"

static int acs_crypto_psa_status_to_errno(psa_status_t status)
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

static int acs_crypto_psa_init(void)
{
	return acs_crypto_psa_status_to_errno(psa_crypto_init());
}

static void acs_crypto_reverse_copy(uint8_t *dst, const uint8_t *src, size_t len)
{
	for (size_t i = 0U; i < len; i++) {
		dst[i] = src[len - 1U - i];
	}
}

static size_t acs_crypto_nonce_fixed_len(const struct acs_crypto_ctx *ctx, bool outbound)
{
	return outbound ? ctx->server_nonce_fixed_len : ctx->client_nonce_fixed_len;
}

static const uint8_t *acs_crypto_nonce_fixed_ptr(const struct acs_crypto_ctx *ctx, bool outbound)
{
	return outbound ? ctx->server_nonce_fixed : ctx->client_nonce_fixed;
}

size_t acs_crypto_nonce_variable_size(const struct acs_crypto_ctx *ctx)
{
	size_t fixed_len;

	if (!ctx) {
		return 0U;
	}

#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) && CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM
	if (ctx->active_key_id == 0x0004U) {
		return 13U - acs_crypto_nonce_fixed_len(ctx, true);
	}
#endif

	return 8U;
}

size_t acs_crypto_auth_tag_size(const struct acs_crypto_ctx *ctx)
{
	if (!ctx) {
		return 0U;
	}

#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) && CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM
	if (ctx->active_key_id == 0x0004U) {
		return CONFIG_BT_ACS_CCM_MAC_SIZE;
	}
#endif

	return 16U;
}

static psa_algorithm_t acs_crypto_aead_algorithm(const struct acs_crypto_ctx *ctx)
{
	size_t tag_len = acs_crypto_auth_tag_size(ctx);

#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) && CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM
	if (ctx->active_key_id == 0x0006U) {
		return PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_GCM, tag_len);
	}
#endif
#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) && CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC
	if (ctx->active_key_id == 0x0007U) {
		return PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_GCM, tag_len);
	}
#endif
#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) && CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM
	if (ctx->active_key_id == 0x0004U) {
		return PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, tag_len);
	}
#endif

	return 0U;
}

static bool acs_crypto_is_gmac_key(const struct acs_crypto_ctx *ctx)
{
#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) && CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC
	return ctx && ctx->active_key_id == 0x0007U;
#else
	ARG_UNUSED(ctx);
	return false;
#endif
}

static int acs_crypto_import_session_key(const struct acs_crypto_ctx *ctx, psa_key_id_t *handle,
					 psa_algorithm_t alg)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;
	int err;

	err = acs_crypto_psa_init();
	if (err) {
		return err;
	}

	psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
	psa_set_key_algorithm(&attrs, alg);
	psa_set_key_type(&attrs, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attrs, sizeof(ctx->session_key) * 8U);

	status = psa_import_key(&attrs, ctx->session_key, sizeof(ctx->session_key), handle);
	psa_reset_key_attributes(&attrs);
	return acs_crypto_psa_status_to_errno(status);
}

void acs_crypto_init(struct acs_crypto_ctx *ctx)
{
	if (!ctx) {
		return;
	}

	memset(ctx, 0, sizeof(*ctx));
}

void acs_crypto_reset(struct acs_crypto_ctx *ctx)
{
	if (!ctx) {
		return;
	}

	memset(ctx, 0, sizeof(*ctx));
}

bool acs_crypto_has_session(const struct acs_crypto_ctx *ctx)
{
	return ctx && ctx->session_key_valid;
}

int acs_crypto_session_install(struct acs_crypto_ctx *ctx, enum acs_crypto_mode mode,
			       uint16_t isc_id, uint16_t key_id, const uint8_t *key,
			       size_t key_len)
{
	if (!ctx || !key || key_len == 0U ||
	    key_len > sizeof(ctx->session_key) || mode == ACS_CRYPTO_MODE_NONE) {
		return -EINVAL;
	}

	memset(ctx->session_key, 0, sizeof(ctx->session_key));
	memcpy(ctx->session_key, key, key_len);
	ctx->tx_nonce_counter = 0U;
	ctx->rx_nonce_counter = 0U;
	ctx->active_isc_id = isc_id;
	ctx->active_key_id = key_id;
	ctx->mode = mode;
	ctx->session_key_valid = true;

	return 0;
}

void acs_crypto_session_clear(struct acs_crypto_ctx *ctx)
{
	if (!ctx) {
		return;
	}

	memset(ctx->session_key, 0, sizeof(ctx->session_key));
	ctx->tx_nonce_counter = 0U;
	ctx->rx_nonce_counter = 0U;
	ctx->active_isc_id = 0U;
	ctx->active_key_id = 0U;
	ctx->mode = ACS_CRYPTO_MODE_NONE;
	ctx->session_key_valid = false;
}

int acs_crypto_set_client_nonce_fixed(struct acs_crypto_ctx *ctx, const uint8_t *nonce,
				      size_t len)
{
	if (!ctx || (!nonce && len > 0U) || len > sizeof(ctx->client_nonce_fixed)) {
		return -EINVAL;
	}

	memset(ctx->client_nonce_fixed, 0, sizeof(ctx->client_nonce_fixed));
	if (len > 0U) {
		acs_crypto_reverse_copy(ctx->client_nonce_fixed, nonce, len);
	}
	ctx->client_nonce_fixed_len = len;

	return 0;
}

int acs_crypto_set_server_nonce_fixed(struct acs_crypto_ctx *ctx, const uint8_t *nonce,
				      size_t len)
{
	if (!ctx || (!nonce && len > 0U) || len > sizeof(ctx->server_nonce_fixed)) {
		return -EINVAL;
	}

	memset(ctx->server_nonce_fixed, 0, sizeof(ctx->server_nonce_fixed));
	if (len > 0U) {
		memcpy(ctx->server_nonce_fixed, nonce, len);
	}
	ctx->server_nonce_fixed_len = len;

	return 0;
}

int acs_crypto_build_nonce(const struct acs_crypto_ctx *ctx, bool outbound, uint8_t *out_nonce,
			   size_t *out_len)
{
	const uint8_t *fixed;
	size_t fixed_len;
	uint64_t counter;
	size_t var_len;
	uint8_t nonce_var_le[sizeof(counter)];
	uint8_t nonce_var_be[sizeof(counter)];

	if (!ctx || !out_nonce || !out_len) {
		return -EINVAL;
	}

	if (!ctx->session_key_valid) {
		return -EACCES;
	}

	if (outbound) {
		fixed = ctx->server_nonce_fixed;
		fixed_len = ctx->server_nonce_fixed_len;
		counter = ctx->tx_nonce_counter;
	} else {
		fixed = ctx->client_nonce_fixed;
		fixed_len = ctx->client_nonce_fixed_len;
		counter = ctx->rx_nonce_counter;
	}

	var_len = acs_crypto_nonce_variable_size(ctx);
	if (*out_len < fixed_len + var_len) {
		return -ENOBUFS;
	}

	if (fixed_len > 0U) {
		memcpy(out_nonce, fixed, fixed_len);
	}
	sys_put_le64(counter, nonce_var_le);
	acs_crypto_reverse_copy(nonce_var_be, nonce_var_le, var_len);
	memcpy(out_nonce + fixed_len, nonce_var_be, var_len);
	*out_len = fixed_len + var_len;

	return 0;
}

int acs_crypto_next_tx_counter(struct acs_crypto_ctx *ctx, uint64_t *counter)
{
	if (!ctx || !counter) {
		return -EINVAL;
	}

	if (!ctx->session_key_valid) {
		return -EACCES;
	}

	*counter = ctx->tx_nonce_counter++;
	return 0;
}

int acs_crypto_accept_rx_counter(struct acs_crypto_ctx *ctx, uint64_t counter)
{
	if (!ctx) {
		return -EINVAL;
	}

	if (!ctx->session_key_valid) {
		return -EACCES;
	}

	if (counter < ctx->rx_nonce_counter) {
		return -EALREADY;
	}

	ctx->rx_nonce_counter = counter + 1U;
	return 0;
}

int acs_crypto_encrypt(struct acs_crypto_ctx *ctx, const uint8_t *plaintext, size_t plain_len,
		       uint8_t *nonce_var, size_t *nonce_var_len, uint8_t *tag,
		       size_t *tag_len, uint8_t *ciphertext, size_t *cipher_len)
{
	psa_key_id_t handle = 0;
	psa_algorithm_t alg;
	uint8_t nonce[ACS_CRYPTO_NONCE_FIXED_MAX_SIZE + sizeof(uint64_t)];
	uint8_t plain_be[ACS_BUF_SIZE];
	uint8_t cipher_and_tag[ACS_BUF_SIZE + ACS_CRYPTO_AUTH_TAG_MAX_SIZE];
	size_t nonce_len = sizeof(nonce);
	size_t var_len;
	size_t local_tag_len;
	size_t output_len = 0U;
	uint64_t counter = 0U;
	psa_status_t status;
	int err;

	if (!ctx || !plaintext || !nonce_var || !nonce_var_len || !tag || !tag_len || !ciphertext ||
	    !cipher_len) {
		return -EINVAL;
	}

	if (!ctx->session_key_valid) {
		return -EACCES;
	}

	var_len = acs_crypto_nonce_variable_size(ctx);
	local_tag_len = acs_crypto_auth_tag_size(ctx);
	alg = acs_crypto_aead_algorithm(ctx);
	if (alg == 0U) {
		return -ENOTSUP;
	}

	if (*nonce_var_len < var_len || *tag_len < local_tag_len || *cipher_len < plain_len ||
	    plain_len > sizeof(plain_be) || plain_len + local_tag_len > sizeof(cipher_and_tag)) {
		return -ENOBUFS;
	}

	err = acs_crypto_next_tx_counter(ctx, &counter);
	if (err) {
		return err;
	}

	sys_put_le64(counter, nonce_var);
	acs_crypto_reverse_copy(nonce + acs_crypto_nonce_fixed_len(ctx, true), nonce_var, var_len);
	memcpy(nonce, acs_crypto_nonce_fixed_ptr(ctx, true), acs_crypto_nonce_fixed_len(ctx, true));
	nonce_len = acs_crypto_nonce_fixed_len(ctx, true) + var_len;
	acs_crypto_reverse_copy(plain_be, plaintext, plain_len);

	err = acs_crypto_import_session_key(ctx, &handle, alg);
	if (err) {
		return err;
	}

	if (acs_crypto_is_gmac_key(ctx)) {
		status = psa_aead_encrypt(handle, alg, nonce, nonce_len, plain_be, plain_len, NULL, 0U,
					  cipher_and_tag, sizeof(cipher_and_tag), &output_len);
		if (status == PSA_SUCCESS && output_len < local_tag_len) {
			status = PSA_ERROR_DATA_INVALID;
		}
		if (status == PSA_SUCCESS) {
			memcpy(ciphertext, plaintext, plain_len);
			acs_crypto_reverse_copy(tag, cipher_and_tag, local_tag_len);
			*cipher_len = plain_len;
			*tag_len = local_tag_len;
			*nonce_var_len = var_len;
		}
	} else {
		status = psa_aead_encrypt(handle, alg, nonce, nonce_len, NULL, 0U, plain_be, plain_len,
					  cipher_and_tag, sizeof(cipher_and_tag), &output_len);
		if (status == PSA_SUCCESS && output_len < local_tag_len) {
			status = PSA_ERROR_DATA_INVALID;
		}
		if (status == PSA_SUCCESS) {
			acs_crypto_reverse_copy(ciphertext, cipher_and_tag, plain_len);
			acs_crypto_reverse_copy(tag, cipher_and_tag + plain_len, local_tag_len);
			*cipher_len = plain_len;
			*tag_len = local_tag_len;
			*nonce_var_len = var_len;
		}
	}

	psa_destroy_key(handle);
	return acs_crypto_psa_status_to_errno(status);
}

int acs_crypto_decrypt(struct acs_crypto_ctx *ctx, const uint8_t *nonce_var, size_t nonce_var_len,
		       const uint8_t *tag, size_t tag_len, const uint8_t *ciphertext,
		       size_t cipher_len, uint8_t *plaintext, size_t *plain_len)
{
	psa_key_id_t handle = 0;
	psa_algorithm_t alg;
	uint8_t nonce[ACS_CRYPTO_NONCE_FIXED_MAX_SIZE + sizeof(uint64_t)];
	uint8_t cipher_and_tag[ACS_BUF_SIZE + ACS_CRYPTO_AUTH_TAG_MAX_SIZE];
	uint8_t plain_be[ACS_BUF_SIZE];
	size_t fixed_len;
	size_t expected_var_len;
	size_t expected_tag_len;
	size_t nonce_len;
	size_t output_len = 0U;
	uint64_t counter = 0U;
	psa_status_t status;
	int err;

	if (!ctx || !nonce_var || !tag || !ciphertext || !plaintext || !plain_len) {
		return -EINVAL;
	}

	if (!ctx->session_key_valid) {
		return -EACCES;
	}

	expected_var_len = acs_crypto_nonce_variable_size(ctx);
	expected_tag_len = acs_crypto_auth_tag_size(ctx);
	alg = acs_crypto_aead_algorithm(ctx);
	if (alg == 0U) {
		return -ENOTSUP;
	}

	if (nonce_var_len != expected_var_len || tag_len != expected_tag_len ||
	    cipher_len > sizeof(plain_be) || *plain_len < cipher_len ||
	    cipher_len + tag_len > sizeof(cipher_and_tag)) {
		return -EINVAL;
	}

	for (size_t i = 0U; i < nonce_var_len; i++) {
		counter |= ((uint64_t)nonce_var[i]) << (8U * i);
	}

	err = acs_crypto_accept_rx_counter(ctx, counter);
	if (err) {
		return err;
	}

	fixed_len = acs_crypto_nonce_fixed_len(ctx, false);
	memcpy(nonce, acs_crypto_nonce_fixed_ptr(ctx, false), fixed_len);
	acs_crypto_reverse_copy(nonce + fixed_len, nonce_var, nonce_var_len);
	nonce_len = fixed_len + nonce_var_len;

	err = acs_crypto_import_session_key(ctx, &handle, alg);
	if (err) {
		return err;
	}

	if (acs_crypto_is_gmac_key(ctx)) {
		acs_crypto_reverse_copy(cipher_and_tag, tag, tag_len);
		acs_crypto_reverse_copy(plain_be, ciphertext, cipher_len);
		status = psa_aead_decrypt(handle, alg, nonce, nonce_len, plain_be, cipher_len,
					  cipher_and_tag, tag_len, NULL, 0U, &output_len);
		if (status == PSA_SUCCESS) {
			memcpy(plaintext, ciphertext, cipher_len);
			*plain_len = cipher_len;
		}
	} else {
		acs_crypto_reverse_copy(cipher_and_tag, ciphertext, cipher_len);
		acs_crypto_reverse_copy(cipher_and_tag + cipher_len, tag, tag_len);
		status = psa_aead_decrypt(handle, alg, nonce, nonce_len, NULL, 0U, cipher_and_tag,
					  cipher_len + tag_len, plain_be, sizeof(plain_be),
					  &output_len);
		if (status == PSA_SUCCESS) {
			acs_crypto_reverse_copy(plaintext, plain_be, output_len);
			*plain_len = output_len;
		}
	}

	psa_destroy_key(handle);
	return acs_crypto_psa_status_to_errno(status);
}
