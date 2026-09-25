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
#include "acs_key_desc.h"
#include "acs_keys.h"

#include <psa/crypto.h>

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* PSA key size for Curve P-256. */
#define ACS_PSA_KEY_BITS_P256 256U

/* Uncompressed NIST-curve point prefix: [0x04][X_BE][Y_BE]. */
#define ACS_ECDH_UNCOMPRESSED_POINT 0x04U

/* ECDH is fixed to Curve P-256 (secp256r1) per ACP 1.0 §4.1.1.1. */
#define ACS_PSA_ECC_FAMILY PSA_ECC_FAMILY_SECP_R1
#define ACS_PSA_KEY_BITS   ACS_PSA_KEY_BITS_P256

K_MEM_SLAB_DEFINE_STATIC(acs_kex_slab, sizeof(struct bt_acs_kex_ctx), CONFIG_BT_ACS_KEX_CTX_COUNT,
			 __alignof__(struct bt_acs_kex_ctx));

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

void acs_kex_set_auth_value(struct bt_acs_kex_ctx *kex, uint32_t number)
{
	memset(kex->auth_value, 0, sizeof(kex->auth_value));
	sys_put_be32(number, &kex->auth_value[sizeof(kex->auth_value) - sizeof(number)]);
}

/*
 * Generate the ephemeral ECDH key pair and export its public coordinates to
 * kex->server_pubkey in little-endian wire order.
 */
static int acs_kex_generate_keypair(struct bt_acs_kex_ctx *kex)
{
	struct acs_ecdh_pubkey *pk = &kex->server_pubkey;
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;
	uint8_t pub[1U + 2U * ACS_ECDH_COORD_SIZE];
	size_t pub_len;
	const uint8_t *x_be = &pub[1];
	const uint8_t *y_be = &pub[1U + ACS_ECDH_COORD_SIZE];

	psa_set_key_type(&attrs, PSA_KEY_TYPE_ECC_KEY_PAIR(ACS_PSA_ECC_FAMILY));
	psa_set_key_bits(&attrs, ACS_PSA_KEY_BITS);
	psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_DERIVE);
	psa_set_key_algorithm(&attrs, PSA_ALG_ECDH);

	status = psa_generate_key(&attrs, &kex->ecdh_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_generate_key failed: %d", status);
		return -EIO;
	}

	/* Export the public key to populate the server coordinates. */
	status = psa_export_public_key(kex->ecdh_key_id, pub, sizeof(pub), &pub_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_export_public_key failed: %d", status);
		acs_keys_destroy(&kex->ecdh_key_id);
		return -EIO;
	}

	pk->key_id = sys_cpu_to_le16(ACS_KEY_ID_ECDH);
	pk->x_size = ACS_ECDH_COORD_SIZE;
	pk->y_size = ACS_ECDH_COORD_SIZE;

	/* PSA exports P-256 as [0x04][X_BE][Y_BE]; ACS carries each coordinate LE. */
	if (pub_len != sizeof(pub)) {
		LOG_ERR("Unexpected NIST public key size: %zu (expected %zu)", pub_len,
			sizeof(pub));
		acs_keys_destroy(&kex->ecdh_key_id);
		return -EIO;
	}

	sys_memcpy_swap(pk->x, x_be, ACS_ECDH_COORD_SIZE); /* X: BE->LE */
	sys_memcpy_swap(pk->y, y_be, ACS_ECDH_COORD_SIZE); /* Y: BE->LE */

	return 0;
}

/*
 * Compute the ECDH shared secret using kex->client_pubkey, import it as
 * kex->derived_key_id, and destroy the ephemeral private key.
 */
static int acs_kex_compute_shared_secret(struct bt_acs_kex_ctx *kex)
{
	const struct acs_ecdh_pubkey *cpk = &kex->client_pubkey;
	uint8_t psa_pubkey[1U + 2U * ACS_ECDH_COORD_SIZE];
	size_t psa_pubkey_len;
	size_t olen;
	psa_status_t status;

	/* The client and server public keys must differ (§4.4.3.17.1.1). */
	if (memcmp(cpk->x, kex->server_pubkey.x, ACS_ECDH_COORD_SIZE) == 0 &&
	    memcmp(cpk->y, kex->server_pubkey.y, ACS_ECDH_COORD_SIZE) == 0) {
		LOG_WRN("client public key matches server public key");
		acs_keys_destroy(&kex->ecdh_key_id);
		return -EINVAL;
	}

	/*
	 * P-256: PSA expects [0x04][X_BE][Y_BE]. The parser zero-extends both
	 * coordinates to the curve size.
	 */
	psa_pubkey[0] = ACS_ECDH_UNCOMPRESSED_POINT;
	sys_memcpy_swap(&psa_pubkey[1], cpk->x, ACS_ECDH_COORD_SIZE);
	sys_memcpy_swap(&psa_pubkey[1U + ACS_ECDH_COORD_SIZE], cpk->y, ACS_ECDH_COORD_SIZE);
	psa_pubkey_len = sizeof(psa_pubkey);

	{
		uint8_t raw_secret[ACS_SHARED_SECRET_SIZE];
		psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;

		status = psa_raw_key_agreement(PSA_ALG_ECDH, kex->ecdh_key_id, psa_pubkey,
					       psa_pubkey_len, raw_secret, sizeof(raw_secret),
					       &olen);

		acs_keys_destroy(&kex->ecdh_key_id);

		if (status != PSA_SUCCESS) {
			LOG_ERR("psa_raw_key_agreement failed: %d", status);
			return (status == PSA_ERROR_INVALID_ARGUMENT) ? -EBADMSG : -EIO;
		}

		psa_set_key_type(&attrs, PSA_KEY_TYPE_DERIVE);
		psa_set_key_bits(&attrs, olen * BITS_PER_BYTE);
		psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_DERIVE);
		psa_set_key_algorithm(&attrs, ACS_PSA_HKDF_ALG);

		status = psa_import_key(&attrs, raw_secret, olen, &kex->derived_key_id);
		mbedtls_platform_zeroize(raw_secret, sizeof(raw_secret));

		if (status != PSA_SUCCESS) {
			LOG_ERR("Failed to import shared secret into PSA: %d", status);
			return -EIO;
		}
	}

	return 0;
}

/* Clear the exchange secrets before releasing the context. */
static void acs_kex_free(struct bt_acs_conn *acs_conn)
{
	struct bt_acs_kex_ctx *kex = acs_conn->kex;

	if (kex == NULL) {
		return;
	}

	acs_keys_destroy(&kex->ecdh_key_id);
	acs_keys_destroy(&kex->derived_key_id);

	acs_conn->kex = NULL;
	mbedtls_platform_zeroize(kex, sizeof(*kex));
	k_mem_slab_free(&acs_kex_slab, kex);
}

int acs_kex_add_result(struct acs_reply *reply)
{
	struct net_buf *buf = acs_reply_add_message(reply);

	if (buf == NULL) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_RESPONSE);
	net_buf_add_le16(buf, sys_le16_to_cpu(reply->conn->kex->start_kex.key_id));
	net_buf_add_u8(buf, ACS_KEX_RSP_SUCCESSFUL);
	return 0;
}

void acs_kex_procedure_sent(struct bt_acs_conn *acs_conn)
{
	const struct bt_acs_cb *cb = acs_cb_get();
	uint16_t key_id;

	if (!acs_kex_in_progress(acs_conn) || acs_conn->kex->next_opcode != ACS_KEX_COMPLETE) {
		return;
	}

	key_id = sys_le16_to_cpu(acs_conn->kex->start_kex.key_id);
	acs_kex_free(acs_conn);

	/* Security starts when the Key Exchange Response is sent, not when it is confirmed. */
	atomic_set_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED);
	if (cb != NULL && cb->security_established != NULL) {
		cb->security_established(acs_conn->conn);
	}

	/* Only ECDH replaces the stored key. */
	if (key_id == ACS_KEY_ID_ECDH) {
		acs_keys_save(acs_conn);
	}

	acs_status_schedule(acs_conn->conn);
}

void acs_key_exchange_abort(struct bt_acs_conn *acs_conn)
{
	if (!acs_kex_in_progress(acs_conn)) {
		return;
	}

	/* Remove what the exchange installed; a failed KDF exchange keeps the ECDH key. */
	acs_keys_remove(acs_conn, sys_le16_to_cpu(acs_conn->kex->start_kex.key_id));

	if (atomic_test_and_clear_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED)) {
		acs_status_schedule(acs_conn->conn);
	}

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

/* Compute the ECDH confirmation code over the exchange transcript (§4.4.3.17.1.2). */
static int acs_kex_compute_confirm(const struct bt_acs_kex_ctx *kex,
				   const uint8_t random[ACS_CONFIRM_VALUE_SIZE],
				   uint8_t confirm_out[ACS_CONFIRM_VALUE_SIZE])
{
	psa_status_t status;
	size_t ecdh_key_len;
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
	sys_memcpy_swap(&scratch.pubkey_concat[pubkey_concat_len], kex->server_pubkey.x,
			ACS_ECDH_COORD_SIZE);
	pubkey_concat_len += ACS_ECDH_COORD_SIZE;
	sys_memcpy_swap(&scratch.pubkey_concat[pubkey_concat_len], kex->server_pubkey.y,
			ACS_ECDH_COORD_SIZE);
	pubkey_concat_len += ACS_ECDH_COORD_SIZE;

	sys_memcpy_swap(&scratch.pubkey_concat[pubkey_concat_len], kex->client_pubkey.x,
			ACS_ECDH_COORD_SIZE);
	pubkey_concat_len += ACS_ECDH_COORD_SIZE;
	sys_memcpy_swap(&scratch.pubkey_concat[pubkey_concat_len], kex->client_pubkey.y,
			ACS_ECDH_COORD_SIZE);
	pubkey_concat_len += ACS_ECDH_COORD_SIZE;

	ret = acs_hmac_sha256(zero_key, sizeof(zero_key), scratch.pubkey_concat, pubkey_concat_len,
			      confirm_salt);
	if (ret != 0) {
		LOG_ERR("HMAC-SHA-256 failed in deriving confirmation salt: %d", ret);
		goto cleanup;
	}

	/* Step 2: ConfirmationKey = HMAC(Salt, ECDHKey || AuthValue) */
	status = psa_export_key(kex->derived_key_id, scratch.ecdh_auth, ACS_ECDH_COORD_SIZE,
				&ecdh_key_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("ECDHKey export for confirmation failed: %d", status);
		ret = -EIO;
		goto cleanup;
	}
	memcpy(&scratch.ecdh_auth[ecdh_key_len], kex->auth_value, ACS_CONFIRM_VALUE_SIZE);

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

static int acs_kdf_serialize_response(const struct bt_acs_kex_ctx *kex,
				      const struct bt_acs_kdf_params *kdf, struct net_buf *rsp_buf)
{
	uint8_t needed = ACS_KDF_RSP_FIXED_SIZE + kdf->salt_size + kdf->info_size;

	if (net_buf_tailroom(rsp_buf) < needed) {
		LOG_WRN("KDF response buffer too small: need %u, have %u", needed,
			net_buf_tailroom(rsp_buf));
		return -ENOMEM;
	}

	net_buf_add_le16(rsp_buf, sys_le16_to_cpu(kex->start_kex.key_id));
	net_buf_add_u8(rsp_buf, kdf->salt_size);
	net_buf_add_mem(rsp_buf, kdf->salt, kdf->salt_size);
	net_buf_add_u8(rsp_buf, kdf->info_size);
	net_buf_add_mem(rsp_buf, kdf->info, kdf->info_size);

	return 0;
}

int acs_key_exchange_ecdh_start(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	if (key_id != ACS_KEY_ID_ECDH) {
		LOG_ERR("key_id 0x%04x does not match supported exchange key IDs", key_id);
		return -EALREADY;
	}

	if (acs_kex_generate_keypair(acs_conn->kex) != 0) {
		return -EIO;
	}

	LOG_INF("ECDH Procedure Started");
	return 0;
}

int acs_key_exchange_ecdh_pubkey(struct bt_acs_conn *acs_conn, struct net_buf *rsp_buf)
{
	int err = acs_kex_compute_shared_secret(acs_conn->kex);

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

	LOG_INF("ECDH Public Keys Exchanged");
	return 0;
}

/*
 * Replace the raw ECDH shared secret with ECDHKey (§4.4.3.17.1): an HKDF-SHA-256
 * secret that stays exportable until the confirmation values are computed.
 */
static int acs_kex_derive_ecdh_key(struct bt_acs_kex_ctx *kex, const struct bt_acs_kdf_params *kdf)
{
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t ecdh_key;
	int err;

	psa_set_key_type(&attrs, PSA_KEY_TYPE_DERIVE);
	psa_set_key_bits(&attrs, ACS_AES_KEY_BITS);
	psa_set_key_usage_flags(&attrs,
				PSA_KEY_USAGE_DERIVE | PSA_KEY_USAGE_COPY | PSA_KEY_USAGE_EXPORT);
	psa_set_key_algorithm(&attrs, ACS_PSA_HKDF_ALG);

	err = acs_keys_derive(kex->derived_key_id, kdf, &attrs, &ecdh_key);
	if (err) {
		return err;
	}

	acs_keys_destroy(&kex->derived_key_id);
	kex->derived_key_id = ecdh_key;
	return 0;
}

int acs_key_exchange_ecdh_kdf(struct bt_acs_conn *acs_conn, struct net_buf *rsp_buf)
{
	struct bt_acs_kdf_params kdf = {0};
	int err;

	err = acs_kdf_params_generate(&kdf);
	if (err) {
		return err;
	}

	err = acs_kex_derive_ecdh_key(acs_conn->kex, &kdf);
	if (err) {
		return err;
	}

	err = acs_kdf_serialize_response(acs_conn->kex, &kdf, rsp_buf);
	if (err) {
		return err;
	}

	LOG_INF("KDF Complete, ECDHKey Derived");
	return 0;
}

int acs_key_exchange_ecdh_confirm_code(struct bt_acs_conn *acs_conn, struct net_buf *rsp_buf)
{
	uint8_t server_confirm[ACS_CONFIRM_VALUE_SIZE];
	uint8_t server_confirm_le[ACS_CONFIRM_VALUE_SIZE];
	psa_status_t rand_status;
	int err;

	rand_status = psa_generate_random(acs_conn->kex->server_random,
					  sizeof(acs_conn->kex->server_random));
	if (rand_status != PSA_SUCCESS) {
		LOG_ERR("server random generation failed: %d", rand_status);
		return -EIO;
	}

	err = acs_kex_compute_confirm(acs_conn->kex, acs_conn->kex->server_random, server_confirm);
	if (err) {
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
	LOG_INF("Server Confirmation Code Sent");
	return 0;
}

int acs_key_exchange_ecdh_confirm_rand(struct bt_acs_conn *acs_conn,
				       const uint8_t client_random[ACS_CONFIRM_VALUE_SIZE],
				       struct net_buf *rsp_buf)
{
	uint8_t computed[ACS_CONFIRM_VALUE_SIZE];
	uint8_t client_random_be[ACS_CONFIRM_VALUE_SIZE];
	uint8_t client_confirm_be[ACS_CONFIRM_VALUE_SIZE];
	uint8_t server_random_le[sizeof(acs_conn->kex->server_random)];
	uint8_t diff = 0;
	int err;

	/* §4.4.3.17.1.1: the client random must differ from the server random. */
	sys_memcpy_swap(client_random_be, client_random, ACS_CONFIRM_VALUE_SIZE);
	if (memcmp(client_random_be, acs_conn->kex->server_random, ACS_CONFIRM_VALUE_SIZE) == 0) {
		LOG_WRN("Client random equals server random");
		return -EINVAL;
	}

	err = acs_kex_compute_confirm(acs_conn->kex, client_random_be, computed);
	if (err) {
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

	/* The client is confirmed: ECDHKey becomes the connection's ECDH key. */
	err = acs_keys_set_ecdh(acs_conn, acs_conn->kex->derived_key_id);
	acs_keys_destroy(&acs_conn->kex->derived_key_id);
	if (err) {
		LOG_ERR("Failed to install the ECDH key: %d", err);
		return -EIO;
	}

	LOG_INF("Client Confirmed - Server Random Sent");
	return 0;
}

int acs_key_exchange_kdf(struct bt_acs_conn *acs_conn, struct net_buf *rsp_buf)
{
	struct bt_acs_kdf_params kdf = {0};
	int err = acs_kdf_params_generate(&kdf);

	if (err) {
		return err;
	}

	err = acs_keys_derive_algs(acs_conn, &kdf);
	if (err) {
		return (err == -EAGAIN) ? err : -EIO;
	}

	err = acs_kdf_serialize_response(acs_conn->kex, &kdf, rsp_buf);
	if (err) {
		return err;
	}

	LOG_INF("KDF key exchange complete");
	return 0;
}

static bool acs_kex_opcode_is_step(uint8_t opcode)
{
	switch (opcode) {
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH:
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF:
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_CODE:
	case BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_RANDOM_NUMBER:
		return true;
	default:
		return false;
	}
}

/* Check the exchange flow and Key_ID before dispatch. */
bool acs_kex_step_allowed(struct bt_acs_conn *acs_conn, uint8_t opcode,
			  const struct net_buf_simple *operand)
{
	/* Start Key Exchange cannot replace a key that is still in use. */
	if (opcode == BT_ACS_CP_OPCODE_START_KEY_EXCHANGE &&
	    acs_keys_installed(acs_conn, sys_get_le16(operand->data))) {
		return false;
	}

	if (!acs_kex_opcode_is_step(opcode)) {
		return true;
	}

	return acs_kex_in_progress(acs_conn) && opcode == acs_conn->kex->next_opcode &&
	       sys_get_le16(operand->data) == sys_le16_to_cpu(acs_conn->kex->start_kex.key_id);
}

void acs_kex_abort_failed_procedure(struct bt_acs_conn *acs_conn, uint8_t opcode)
{
	if ((opcode == BT_ACS_CP_OPCODE_START_KEY_EXCHANGE || acs_kex_opcode_is_step(opcode)) &&
	    acs_kex_in_progress(acs_conn)) {
		acs_key_exchange_abort(acs_conn);
	}
}
