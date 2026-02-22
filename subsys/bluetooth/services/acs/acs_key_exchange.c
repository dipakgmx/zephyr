/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <mbedtls/platform_util.h>
#include "acs_key_exchange.h"
#include "acs_internal.h"
#include "acs_reply.h"
#include "acs_crypto.h"
#include "acs_key_desc.h"

#include <psa/crypto.h>

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

K_MEM_SLAB_DEFINE_STATIC(acs_kex_slab, sizeof(struct bt_acs_kex_ctx), CONFIG_BT_ACS_KEX_CTX_COUNT,
			 __alignof__(struct bt_acs_kex_ctx));

struct bt_acs_key_desc_runtime *acs_key_exchange_installed_key(struct bt_acs_conn *acs_conn)
{
	struct bt_acs_key_desc_runtime *kdf_key;

	if (acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_KDF, &kdf_key) == 0 &&
	    kdf_key->psa_key_id != 0U) {
		return kdf_key;
	}

	{
		struct bt_acs_key_desc_runtime *ecdh_key;

		if (acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_ECDH, &ecdh_key) == 0 &&
		    ecdh_key->psa_key_id != 0U) {
			return ecdh_key;
		}
	}

	return NULL;
}

struct bt_acs_kex_ctx *acs_kex_alloc(struct bt_acs_conn *acs_conn)
{
	struct bt_acs_kex_ctx *kex;

	__ASSERT_NO_MSG(acs_conn->kex == NULL);

	if (k_mem_slab_alloc(&acs_kex_slab, (void **)&kex, K_NO_WAIT) != 0) {
		LOG_WRN("no free key-exchange context");
		return NULL;
	}

	memset(kex, 0, sizeof(*kex));
	acs_conn->kex = kex;
	return kex;
}

/* Clear the exchange secrets before releasing the context. */
static void acs_kex_free(struct bt_acs_conn *acs_conn)
{
	struct bt_acs_kex_ctx *kex = acs_conn->kex;

	if (kex == NULL) {
		return;
	}

	acs_psa_destroy_key(&kex->ecdh_key_id);
	acs_psa_destroy_key(&kex->derived_key_id);

	acs_conn->kex = NULL;
	mbedtls_platform_zeroize(kex, sizeof(*kex));
	k_mem_slab_free(&acs_kex_slab, kex);
}

/* Send the successful Key Exchange Response (§4.4.3.10). */
int acs_kex_send_result(struct acs_reply *reply)
{
	struct bt_acs_conn const *acs_conn = reply->conn;
	struct net_buf *buf;

	buf = acs_prepare_reply_buf(reply);
	if (buf == NULL) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_RESPONSE);
	net_buf_add_le16(buf, sys_le16_to_cpu(acs_conn->kex->start_kex.key_id));
	net_buf_add_u8(buf, 0x00);

	/* Security starts when this response is sent, not when it is confirmed. */
	atomic_set_bit_to(&reply->conn->state, ACS_STATE_SECURITY_ESTABLISHED, true);

	return acs_reply_submit(reply);
}

void acs_kex_finalize_success(struct bt_acs_conn *acs_conn)
{
	const struct bt_acs_cb *cb = acs_cb_get();
	uint16_t key_id = sys_le16_to_cpu(acs_conn->kex->start_kex.key_id);

	acs_kex_free(acs_conn);

	/* The response path has already marked security as established. */
	if (cb != NULL && cb->security_established != NULL) {
		cb->security_established(acs_conn->conn);
	}

	/* Only ECDH replaces the stored parent key. */
	if (key_id == ACS_KEY_ID_ECDH) {
		acs_key_store(acs_conn);
	}

	acs_status_schedule(acs_conn->conn);
}

void acs_kex_conclude(struct acs_reply *reply)
{
	__ASSERT_NO_MSG(reply->conn->kex != NULL);

	reply->step = ACS_REPLY_KEX_OK;
}

void acs_key_exchange_abort(struct bt_acs_conn *acs_conn)
{
	uint16_t key_id;

	if (!acs_kex_in_progress(acs_conn)) {
		return;
	}

	key_id = sys_le16_to_cpu(acs_conn->kex->start_kex.key_id);

	if (key_id == ACS_KEY_ID_KDF) {
		/* Keep the ECDH parent so the client can retry KDF. */
		acs_crypto_destroy_kdf_keys(acs_conn);

		/* A failed KDF exchange leaves no usable session key. */
		if (atomic_test_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED)) {
			atomic_set_bit_to(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED, false);
			acs_status_schedule(acs_conn->conn);
		}

		acs_kex_free(acs_conn);
		return;
	}

	acs_crypto_destroy_exchange_keys(acs_conn);
	acs_crypto_invalidate_algorithm_keys(acs_conn);
	acs_kex_free(acs_conn);
}

static int acs_hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *msg, size_t msg_len,
			   uint8_t out[PSA_HASH_LENGTH(PSA_ALG_SHA_256)])
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;
	psa_status_t destroy_status;
	psa_key_id_t hmac_key;
	size_t out_len;

	psa_set_key_type(&attrs, PSA_KEY_TYPE_HMAC);
	psa_set_key_bits(&attrs, key_len * BITS_PER_BYTE);
	psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_SIGN_MESSAGE);
	psa_set_key_algorithm(&attrs, PSA_ALG_HMAC(PSA_ALG_SHA_256));

	status = psa_import_key(&attrs, key, key_len, &hmac_key);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Failed to import HMAC key: status=%d, key_len=%zu", status, key_len);
		return -EIO;
	}

	status = psa_mac_compute(hmac_key, PSA_ALG_HMAC(PSA_ALG_SHA_256), msg, msg_len, out,
				 PSA_HASH_LENGTH(PSA_ALG_SHA_256), &out_len);
	destroy_status = psa_destroy_key(hmac_key);
	if ((status != PSA_SUCCESS) || (destroy_status != PSA_SUCCESS)) {
		LOG_ERR("Failed to compute HMAC-SHA-256: status=%d, destroy=%d, msg_len=%zu",
			status, destroy_status, msg_len);
		return -EIO;
	}
	return 0;
}

/* Derive a key from the input secret held by ikm_key_id. */
static int acs_hkdf_op_setup(psa_key_derivation_operation_t *op, psa_algorithm_t alg,
			     const uint8_t *salt, size_t salt_len, psa_key_id_t ikm_key_id,
			     const uint8_t *info, size_t info_len)
{
	psa_status_t status;

	status = psa_key_derivation_setup(op, alg);
	if (status != PSA_SUCCESS) {
		LOG_ERR("HKDF setup failed: status=%d, alg=0x%08x", status, alg);
		return -EIO;
	}

	if (salt_len > 0U) {
		status = psa_key_derivation_input_bytes(op, PSA_KEY_DERIVATION_INPUT_SALT, salt,
							salt_len);
		if (status != PSA_SUCCESS) {
			LOG_ERR("HKDF salt input failed: status=%d, salt_len=%zu", status,
				salt_len);
			return -EIO;
		}
	}

	status = psa_key_derivation_input_key(op, PSA_KEY_DERIVATION_INPUT_SECRET, ikm_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("HKDF secret input failed: status=%d", status);
		return -EIO;
	}

	status = psa_key_derivation_input_bytes(op, PSA_KEY_DERIVATION_INPUT_INFO, info, info_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("HKDF info input failed: status=%d, info_len=%zu", status, info_len);
		return -EIO;
	}

	return 0;
}

static int acs_hkdf_op_abort(psa_key_derivation_operation_t *op, int prior_ret)
{
	psa_status_t status = psa_key_derivation_abort(op);

	if (status != PSA_SUCCESS && prior_ret == 0) {
		LOG_ERR("Failed to abort key derivation operation: %d", status);
		return -EIO;
	}
	return prior_ret;
}

/* Create operational and derivation keys with the same key material. */
static int acs_kex_derive_exchange_key_pair(struct bt_acs_key_desc_runtime *key_runtime,
					    psa_algorithm_t alg, const uint8_t *salt,
					    size_t salt_len, psa_key_id_t ikm_key_id,
					    const uint8_t *info, size_t info_len, size_t key_len)
{
	psa_key_derivation_operation_t op_key = PSA_KEY_DERIVATION_OPERATION_INIT;
	psa_key_derivation_operation_t op_derive = PSA_KEY_DERIVATION_OPERATION_INIT;
	int ret;

	ret = acs_hkdf_op_setup(&op_key, alg, salt, salt_len, ikm_key_id, info, info_len);
	if (ret) {
		goto cleanup;
	}

	ret = acs_hkdf_op_setup(&op_derive, alg, salt, salt_len, ikm_key_id, info, info_len);
	if (ret) {
		goto cleanup;
	}

	acs_crypto_destroy_key(key_runtime);

	ret = acs_crypto_output_exchange_key(key_runtime, &op_key, key_len, false);
	if (ret) {
		goto cleanup;
	}

	ret = acs_crypto_output_exchange_key(key_runtime, &op_derive, key_len, true);

cleanup:
	ret = acs_hkdf_op_abort(&op_key, ret);
	ret = acs_hkdf_op_abort(&op_derive, ret);
	return ret;
}

int acs_nonce_state_init(struct bt_acs_key_desc_runtime *runtime)
{
	const struct bt_acs_key_desc_record *key_desc = runtime->key_desc;
	uint8_t prefix_size = acs_key_desc_nonce_prefix_size(key_desc);

	/* Sequence-number nonces use the full 64-bit counter. */
	__ASSERT(!acs_key_desc_has_nonce_record(key_desc) ||
			 acs_key_desc_nonce_var_size(key_desc) == sizeof(uint64_t),
		 "nonce-bearing records require an 8-octet variable part");

	/* Only Sequence-Number-Different-Fixed-Parts is supported. */
	if (acs_key_desc_has_nonce_record(key_desc) &&
	    key_desc->aes.nonce_type != ACS_NONCE_SEQ_DIFF_FIXED) {
		LOG_ERR("Key_ID 0x%04x nonce type %u unsupported (only DIFF_FIXED)",
			acs_key_desc_runtime_key_id(runtime), key_desc->aes.nonce_type);
		return -ENOTSUP;
	}

	runtime->tx_nonce_counter = 0U;
	runtime->rx_nonce_counter = 0U;

	if (prefix_size == 0U) {
		return 0;
	}

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	return acs_server_nonce_fixed_ensure(runtime);
#else
	return 0;
#endif
}

/* Replace the exchange key and install it for its algorithm records. */
static int acs_kex_install_keys(struct bt_acs_conn *acs_conn,
				struct bt_acs_key_desc_runtime *exchange_key,
				const uint8_t *key_material, size_t key_len)
{
	int err;

	acs_crypto_destroy_key(exchange_key);

	err = acs_crypto_import_exchange_key(exchange_key, key_material, key_len);
	if (err) {
		return err;
	}

	/* Set up each record's nonce state with its child key. */
	return acs_crypto_bind_algorithm_keys(acs_conn, exchange_key, true);
}

/* Compute the ECDH confirmation code (§4.4.3.17.1.2). */
static int acs_ecdh_confirm_code_compute(const uint8_t *server_x_le, const uint8_t *server_y_le,
					 const uint8_t *client_x_le, const uint8_t *client_y_le,
					 size_t coord_size, const uint8_t *ecdh_key,
					 size_t ecdh_key_len,
					 const uint8_t auth_value[ACS_CONFIRM_VALUE_SIZE],
					 const uint8_t random[ACS_CONFIRM_VALUE_SIZE],
					 uint8_t confirm_out[ACS_CONFIRM_VALUE_SIZE])
{
	int ret;
	const uint8_t zero_key[PSA_HASH_LENGTH(PSA_ALG_SHA_256)] = {0};
	size_t pubkey_concat_len = 0;
	/* These values are used at different steps and can share storage. */
	union {
		uint8_t pubkey_concat[ACS_ECDH_COORD_SIZE * 4];
		uint8_t ecdh_auth[ACS_ECDH_COORD_SIZE + ACS_CONFIRM_VALUE_SIZE];
	} scratch;
	uint8_t confirm_salt[ACS_CONFIRM_VALUE_SIZE];
	uint8_t confirm_key[ACS_CONFIRM_VALUE_SIZE];

	/* Step 1: Salt = HMAC(Zero, PKsx_BE||PKsy_BE||PKcx_BE||PKcy_BE) */
	sys_memcpy_swap(&scratch.pubkey_concat[pubkey_concat_len], server_x_le, coord_size);
	pubkey_concat_len += coord_size;

	if (server_y_le != NULL) {
		sys_memcpy_swap(&scratch.pubkey_concat[pubkey_concat_len], server_y_le, coord_size);
		pubkey_concat_len += coord_size;
	}

	sys_memcpy_swap(&scratch.pubkey_concat[pubkey_concat_len], client_x_le, coord_size);
	pubkey_concat_len += coord_size;

	if (client_y_le != NULL) {
		sys_memcpy_swap(&scratch.pubkey_concat[pubkey_concat_len], client_y_le, coord_size);
		pubkey_concat_len += coord_size;
	}

	ret = acs_hmac_sha256(zero_key, sizeof(zero_key), scratch.pubkey_concat, pubkey_concat_len,
			      confirm_salt);
	if (ret != 0) {
		LOG_ERR("HMAC-SHA-256 failed in deriving confirmation salt: %d", ret);
		goto cleanup;
	}

	/* Step 2: ConfirmationKey = HMAC(Salt, ECDHKey || AuthValue) */
	memcpy(&scratch.ecdh_auth[0], ecdh_key, ecdh_key_len);
	memcpy(&scratch.ecdh_auth[ecdh_key_len], auth_value, ACS_CONFIRM_VALUE_SIZE);

	ret = acs_hmac_sha256(confirm_salt, sizeof(confirm_salt), scratch.ecdh_auth,
			      ecdh_key_len + ACS_CONFIRM_VALUE_SIZE, confirm_key);
	if (ret != 0) {
		LOG_ERR("HMAC-SHA-256 failed in deriving confirmation key: %d", ret);
		goto cleanup;
	}

	/* Step 3: ConfirmationCode = HMAC(ConfirmationKey, RandomNumber) */
	ret = acs_hmac_sha256(confirm_key, sizeof(confirm_key), random, ACS_CONFIRM_VALUE_SIZE,
			      confirm_out);
	if (ret != 0) {
		LOG_ERR("HMAC-SHA-256 failed in deriving confirmation code: %d", ret);
	}

cleanup:
	mbedtls_platform_zeroize(confirm_salt, sizeof(confirm_salt));
	mbedtls_platform_zeroize(confirm_key, sizeof(confirm_key));
	mbedtls_platform_zeroize(&scratch, sizeof(scratch));
	return ret;
}

/* Derive both KDF child handles without exporting key material (§4.4.3.17.2.1). */
static int acs_kex_derive_kdf_child_key(struct bt_acs_conn *acs_conn)
{
	struct bt_acs_key_desc_runtime *parent_key;
	struct bt_acs_key_desc_runtime *kdf_key;
	const struct bt_acs_key_desc_record *kdf_desc;
	uint16_t parent_key_id;
	uint8_t salt_be[ACS_KDF_SALT_SIZE];
	uint8_t info_be[ACS_KDF_INFO_SIZE];
	int ret;

	/* Derive the KDF child from the ECDH parent named by its descriptor. */
	kdf_desc = acs_key_desc_lookup(ACS_KEY_ID_KDF);
	parent_key_id = acs_key_desc_parent_key_id(kdf_desc);

	if (acs_crypto_key_runtime_lookup(acs_conn, parent_key_id, &parent_key) != 0 ||
	    parent_key->psa_key_id == 0U) {
		LOG_WRN("No established parent Key_ID 0x%04x for KDF child", parent_key_id);
		return -EAGAIN;
	}

	ret = acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_KDF, &kdf_key);
	if (ret) {
		LOG_ERR("No runtime key state for KDF child");
		return ret;
	}

	if (parent_key->derive_key_id == 0U) {
		LOG_WRN("ECDH parent derive key missing");
		return -EAGAIN;
	}

	/* Salt and info are stored in wire order (LSO) but HKDF requires BE input. */
	sys_memcpy_swap(salt_be, acs_conn->kex->kdf.salt, acs_conn->kex->kdf.salt_size);
	sys_memcpy_swap(info_be, acs_conn->kex->kdf.info, acs_conn->kex->kdf.info_size);

	ret = acs_kex_derive_exchange_key_pair(
		kdf_key, ACS_PSA_HKDF_ALG, salt_be, acs_conn->kex->kdf.salt_size,
		parent_key->derive_key_id, info_be, acs_conn->kex->kdf.info_size,
		ACS_AES_KEY_SIZE);
	if (ret) {
		return ret;
	}

	return acs_crypto_bind_algorithm_keys(acs_conn, kdf_key, true);
}

/* Fill the KDF parameters sent to the peer: random salt, fixed "ACS" info. */
static int acs_kdf_params_generate(struct bt_acs_kdf_params *kdf)
{
	psa_status_t psa_ret;

	psa_ret = psa_generate_random(kdf->salt, ACS_KDF_SALT_SIZE);
	if (psa_ret != PSA_SUCCESS) {
		LOG_ERR("psa_generate_random failed for KDF salt: %d", psa_ret);
		return -EIO;
	}
	kdf->salt_size = ACS_KDF_SALT_SIZE;

	memcpy(kdf->info, "ACS", 3);
	kdf->info_size = 3;

	return 0;
}

static int acs_kdf_serialize_response(struct bt_acs_kex_ctx const *kex, struct net_buf *rsp_buf)
{
	uint8_t needed = ACS_KDF_RSP_FIXED_SIZE + kex->kdf.salt_size + kex->kdf.info_size;

	if (net_buf_tailroom(rsp_buf) < needed) {
		LOG_WRN("KDF response buffer too small: need %u, have %u", needed,
			net_buf_tailroom(rsp_buf));
		return -ENOMEM;
	}

	net_buf_add_le16(rsp_buf, sys_le16_to_cpu(kex->start_kex.key_id));
	net_buf_add_u8(rsp_buf, kex->kdf.salt_size);
	net_buf_add_mem(rsp_buf, kex->kdf.salt, kex->kdf.salt_size);
	net_buf_add_u8(rsp_buf, kex->kdf.info_size);
	net_buf_add_mem(rsp_buf, kex->kdf.info, kex->kdf.info_size);

	return 0;
}

int acs_key_exchange_ecdh_start(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	if (key_id != ACS_KEY_ID_ECDH) {
		LOG_ERR("key_id 0x%04x does not match supported exchange key IDs", key_id);
		return -EALREADY;
	}

	/* Keep nonce prefixes that were already published in a Key Descriptor. */
	acs_crypto_destroy_exchange_keys(acs_conn);
	acs_crypto_destroy_connection_record_keys(acs_conn);

	{
		int err = acs_crypto_generate_keypair(acs_conn);

		if (err) {
			return -EIO;
		}
	}

	LOG_INF("ECDH Procedure Started");
	return 0;
}

int acs_key_exchange_ecdh_pubkey(struct bt_acs_conn *acs_conn, struct net_buf *rsp_buf)
{
	int err = acs_crypto_compute_shared_secret(acs_conn);

	if (err) {
		return err;
	}

	if (net_buf_tailroom(rsp_buf) < sizeof(acs_conn->kex->server_pubkey)) {
		LOG_ERR("Not enough tailroom for server_pubkey in response buffer: need %zu, have "
			"%zu",
			sizeof(acs_conn->kex->server_pubkey), net_buf_tailroom(rsp_buf));
		return -ENOMEM;
	}

	net_buf_add_mem(rsp_buf, &acs_conn->kex->server_pubkey,
			sizeof(acs_conn->kex->server_pubkey));

	/* §4.4.3.17.1: ECDH key exchange requires KDF next. */
	acs_conn->kex->state = ACS_KEX_AWAIT_KDF;
	LOG_INF("ECDH Public Keys Exchanged");
	return 0;
}

/* Replace the raw ECDH secret with the derived exchange key (§4.4.3.17.1). */
static int acs_kex_derive_intermediate_key(struct bt_acs_kex_ctx *kex, const uint8_t *salt_be,
					   const uint8_t *info_be)
{
	psa_key_derivation_operation_t op = PSA_KEY_DERIVATION_OPERATION_INIT;
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t new_key_id = 0;
	int ret;

	ret = acs_hkdf_op_setup(&op, ACS_PSA_HKDF_ALG, salt_be, kex->kdf.salt_size,
				kex->derived_key_id, info_be, kex->kdf.info_size);
	if (ret == 0) {
		psa_status_t status;

		psa_set_key_type(&attrs, PSA_KEY_TYPE_AES);
		psa_set_key_bits(&attrs, ACS_AES_KEY_BITS);
		psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_EXPORT);
		psa_set_key_algorithm(&attrs, PSA_ALG_NONE);

		status = psa_key_derivation_output_key(&attrs, &op, &new_key_id);
		if (status != PSA_SUCCESS) {
			LOG_ERR("HKDF output_key failed: %d", status);
			ret = -EIO;
		}
	}

	ret = acs_hkdf_op_abort(&op, ret);
	if (ret) {
		/* Destroy the key because it has not been saved on kex. */
		acs_psa_destroy_key(&new_key_id);
		return ret;
	}

	acs_psa_destroy_key(&kex->derived_key_id);
	kex->derived_key_id = new_key_id;
	return 0;
}

int acs_key_exchange_ecdh_kdf(struct bt_acs_conn *acs_conn, struct net_buf *rsp_buf)
{
	uint8_t salt_be[ACS_KDF_SALT_SIZE];
	uint8_t info_be[ACS_KDF_INFO_SIZE];
	int err;

	err = acs_kdf_params_generate(&acs_conn->kex->kdf);
	if (err) {
		return err;
	}

	sys_memcpy_swap(salt_be, acs_conn->kex->kdf.salt, acs_conn->kex->kdf.salt_size);
	sys_memcpy_swap(info_be, acs_conn->kex->kdf.info, acs_conn->kex->kdf.info_size);

	err = acs_kex_derive_intermediate_key(acs_conn->kex, salt_be, info_be);
	if (err) {
		return err;
	}

	err = acs_kdf_serialize_response(acs_conn->kex, rsp_buf);
	if (err) {
		return err;
	}

	acs_conn->kex->state = ACS_KEX_AWAIT_CONFIRM_CODE;
	LOG_INF("KDF Complete, ECDHKey Derived");
	return 0;
}

/* Compute a confirmation code over the exchange transcript and random_be. */
static int acs_kex_compute_confirm(struct bt_acs_kex_ctx *kex, const uint8_t *ecdh_key,
				   size_t ecdh_key_len,
				   const uint8_t random_be[ACS_CONFIRM_VALUE_SIZE],
				   uint8_t confirm_out[ACS_CONFIRM_VALUE_SIZE])
{
	return acs_ecdh_confirm_code_compute(kex->server_pubkey.x, kex->server_pubkey.y,
					     kex->client_pubkey.x, kex->client_pubkey.y,
					     ACS_ECDH_COORD_SIZE, ecdh_key, ecdh_key_len,
					     kex->auth_value, random_be, confirm_out);
}

int acs_key_exchange_ecdh_confirm_code(struct bt_acs_conn *acs_conn, struct net_buf *rsp_buf)
{
	uint8_t key_mat[ACS_AES_KEY_SIZE];
	size_t key_mat_len;
	int err;
	uint8_t server_confirm[ACS_CONFIRM_VALUE_SIZE];
	uint8_t server_confirm_le[ACS_CONFIRM_VALUE_SIZE];

	{
		psa_status_t rand_status;

		rand_status = psa_generate_random(acs_conn->kex->server_random,
						  sizeof(acs_conn->kex->server_random));
		if (rand_status != PSA_SUCCESS) {
			LOG_ERR("server random generation failed: %d", rand_status);
			return -EIO;
		}
	}

	{
		psa_status_t status;

		status = psa_export_key(acs_conn->kex->derived_key_id, key_mat, sizeof(key_mat),
					&key_mat_len);
		if (status != PSA_SUCCESS) {
			LOG_ERR("Failed to export derived key for confirm: %d", status);
			return -EIO;
		}
	}

	err = acs_kex_compute_confirm(acs_conn->kex, key_mat, key_mat_len,
				      acs_conn->kex->server_random, server_confirm);

	mbedtls_platform_zeroize(key_mat, sizeof(key_mat));

	if (err) {
		LOG_ERR("confirm code compute failed: %d", err);
		return -EIO;
	}

	/* Encode Key_ID followed by ACServerConfirmationCode in wire order. */
	if (net_buf_tailroom(rsp_buf) < sizeof(uint16_t) + ACS_CONFIRM_VALUE_SIZE) {
		return -ENOMEM;
	}

	net_buf_add_le16(rsp_buf, sys_le16_to_cpu(acs_conn->kex->start_kex.key_id));

	/* The calculated confirmation code is big-endian; ACS transmits it LSO first. */
	sys_memcpy_swap(server_confirm_le, server_confirm, ACS_CONFIRM_VALUE_SIZE);
	net_buf_add_mem(rsp_buf, server_confirm_le, ACS_CONFIRM_VALUE_SIZE);
	acs_conn->kex->state = ACS_KEX_AWAIT_CONFIRM_RAND;
	LOG_INF("Server Confirmation Code Sent");
	return 0;
}

int acs_key_exchange_ecdh_confirm_rand(struct bt_acs_conn *acs_conn,
				       const uint8_t client_random[ACS_CONFIRM_VALUE_SIZE],
				       struct net_buf *rsp_buf)
{
	uint8_t key_mat[ACS_AES_KEY_SIZE];
	size_t key_mat_len;
	uint8_t computed[ACS_CONFIRM_VALUE_SIZE];
	uint8_t client_random_be[ACS_CONFIRM_VALUE_SIZE];
	uint8_t client_confirm_be[ACS_CONFIRM_VALUE_SIZE];
	uint8_t server_random_le[sizeof(acs_conn->kex->server_random)];
	struct bt_acs_key_desc_runtime *exchange_key;
	uint8_t diff = 0;
	int err;

	/* §4.4.3.17.1.1: the client random must differ from the server random. */
	sys_memcpy_swap(client_random_be, client_random, ACS_CONFIRM_VALUE_SIZE);
	if (memcmp(client_random_be, acs_conn->kex->server_random, ACS_CONFIRM_VALUE_SIZE) == 0) {
		LOG_WRN("Client random equals server random");
		return -EINVAL;
	}

	{
		psa_status_t status;

		status = psa_export_key(acs_conn->kex->derived_key_id, key_mat, sizeof(key_mat),
					&key_mat_len);
		if (status != PSA_SUCCESS) {
			LOG_ERR("Failed to export derived key for verify: %d", status);
			return -EIO;
		}
	}

	err = acs_kex_compute_confirm(acs_conn->kex, key_mat, key_mat_len, client_random_be,
				      computed);

	mbedtls_platform_zeroize(key_mat, sizeof(key_mat));

	if (err) {
		LOG_ERR("confirm code compute failed during verify: %d", err);
		return -EIO;
	}

	/* client_confirm is wire LE; reverse it before the constant-time BE comparison. */
	sys_memcpy_swap(client_confirm_be, acs_conn->kex->client_confirm, ACS_CONFIRM_VALUE_SIZE);

	for (int i = 0; i < ACS_CONFIRM_VALUE_SIZE; i++) {
		diff |= client_confirm_be[i] ^ computed[i];
	}

	if (diff) {
		LOG_ERR("Client confirmation code mismatch - authentication failed");
		return -EACCES;
	}

	/* Encode Key_ID followed by ACServerConfirmationRandomNumber in wire order. */
	if (net_buf_tailroom(rsp_buf) < sizeof(uint16_t) + sizeof(acs_conn->kex->server_random)) {
		return -ENOMEM;
	}

	net_buf_add_le16(rsp_buf, sys_le16_to_cpu(acs_conn->kex->start_kex.key_id));

	/* server_random is BE; reverse it to LE for the wire. */
	sys_memcpy_swap(server_random_le, acs_conn->kex->server_random,
			sizeof(acs_conn->kex->server_random));
	net_buf_add_mem(rsp_buf, server_random_le, sizeof(acs_conn->kex->server_random));

	/* Activate the session key after client confirmation succeeds. */
	if (acs_crypto_key_runtime_lookup(acs_conn,
					  sys_le16_to_cpu(acs_conn->kex->start_kex.key_id),
					  &exchange_key) != 0) {
		LOG_ERR("Missing exchange runtime key state");
		return -EIO;
	}

	{
		uint8_t key_buf[ACS_AES_KEY_SIZE];
		size_t key_buf_len;
		psa_status_t status;

		status = psa_export_key(acs_conn->kex->derived_key_id, key_buf, sizeof(key_buf),
					&key_buf_len);
		if (status != PSA_SUCCESS) {
			LOG_ERR("Failed to export derived key: %d", status);
			return -EIO;
		}

		acs_psa_destroy_key(&acs_conn->kex->derived_key_id);
		err = acs_kex_install_keys(acs_conn, exchange_key, key_buf, key_buf_len);
		mbedtls_platform_zeroize(key_buf, sizeof(key_buf));

		if (err) {
			LOG_ERR("Failed to establish session key: %d", err);
			return -EIO;
		}
	}

	LOG_INF("Client Confirmed - Server Random Sent");
	return 0;
}

int acs_key_exchange_kdf(struct bt_acs_conn *acs_conn, struct net_buf *rsp_buf)
{
	int err = acs_kdf_params_generate(&acs_conn->kex->kdf);

	if (err) {
		return err;
	}

	err = acs_kex_derive_kdf_child_key(acs_conn);
	if (err) {
		if (err == -EAGAIN) {
			LOG_WRN("KDF child key derivation not applicable: %d", err);
			return err;
		}

		LOG_ERR("KDF child key derivation failed: %d", err);
		return -EIO;
	}

	err = acs_kdf_serialize_response(acs_conn->kex, rsp_buf);
	if (err) {
		return err;
	}

	LOG_INF("KDF child key derived success");
	return 0;
}

/* A stored parent key may exist before a session is established. */
static bool acs_exchange_key_is_live(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	struct bt_acs_key_desc_runtime *key;

	return acs_crypto_key_runtime_lookup(acs_conn, key_id, &key) == 0 && key->psa_key_id != 0U;
}

static bool acs_kex_opcode_is_step(uint8_t opcode)
{
	switch (opcode) {
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH:
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF:
	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE:
	case BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND:
		return true;
	default:
		return false;
	}
}

static uint8_t acs_kex_expected_opcode(enum acs_kex_state state)
{
	switch (state) {
	case ACS_KEX_AWAIT_PUBKEY:
		return BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH;
	case ACS_KEX_AWAIT_KDF:
		return BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF;
	case ACS_KEX_AWAIT_CONFIRM_CODE:
		return BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE;
	case ACS_KEX_AWAIT_CONFIRM_RAND:
		return BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND;
	}

	return 0U;
}

/* Check the exchange flow and Key_ID before dispatch. */
bool acs_kex_step_allowed(struct bt_acs_conn *acs_conn, uint8_t opcode,
			  const struct net_buf_simple *operand)
{
	/* Start Key Exchange cannot replace a key that is still in use. */
	if (opcode == BT_ACS_CP_OPCODE_START_KEY_EXCHANGE &&
	    acs_exchange_key_is_live(acs_conn, sys_get_le16(operand->data))) {
		return false;
	}

	if (!acs_kex_opcode_is_step(opcode)) {
		return true;
	}
	if (!acs_kex_in_progress(acs_conn) ||
	    acs_kex_expected_opcode(acs_conn->kex->state) != opcode) {
		return false;
	}
	return sys_get_le16(operand->data) == sys_le16_to_cpu(acs_conn->kex->start_kex.key_id);
}

void acs_kex_abort_failed_procedure(struct bt_acs_conn *acs_conn, uint8_t opcode)
{
	if ((opcode == BT_ACS_CP_OPCODE_START_KEY_EXCHANGE || acs_kex_opcode_is_step(opcode)) &&
	    acs_kex_in_progress(acs_conn)) {
		acs_key_exchange_abort(acs_conn);
	}
}
