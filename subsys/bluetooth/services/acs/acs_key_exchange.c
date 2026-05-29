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
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <mbedtls/platform_util.h>
#include "acs_key_exchange.h"
#include "acs_internal.h"
#include "acs_key_desc.h"

#include <psa/crypto.h>

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* PSA key bit sizes for supported ECDH curves */
#define ACS_PSA_KEY_BITS_P521   521U
#define ACS_PSA_KEY_BITS_P384   384U
#define ACS_PSA_KEY_BITS_C25519 255U
#define ACS_PSA_KEY_BITS_P256   256U

/* Uncompressed point prefix for NIST curves ([0x04][X_BE][Y_BE]) */
#define ACS_ECDH_UNCOMPRESSED_POINT 0x04U

/* Compile-time curve selection - all Kconfig-determined, no runtime branch needed */
#if IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_CURVE25519)
#define ACS_PSA_ECC_FAMILY PSA_ECC_FAMILY_MONTGOMERY
#define ACS_PSA_KEY_BITS   ACS_PSA_KEY_BITS_C25519
#elif IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_P521)
#define ACS_PSA_ECC_FAMILY PSA_ECC_FAMILY_SECP_R1
#define ACS_PSA_KEY_BITS   ACS_PSA_KEY_BITS_P521
#elif IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_P384)
#define ACS_PSA_ECC_FAMILY PSA_ECC_FAMILY_SECP_R1
#define ACS_PSA_KEY_BITS   ACS_PSA_KEY_BITS_P384
#else
#define ACS_PSA_ECC_FAMILY PSA_ECC_FAMILY_SECP_R1
#define ACS_PSA_KEY_BITS   ACS_PSA_KEY_BITS_P256
#endif

#if IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA384) || IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA384_WITH_INFO)
#define ACS_PSA_HKDF_ALG PSA_ALG_HKDF(PSA_ALG_SHA_384)
#elif IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA512) ||                                                 \
	IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA512_WITH_INFO)
#define ACS_PSA_HKDF_ALG PSA_ALG_HKDF(PSA_ALG_SHA_512)
#else
#define ACS_PSA_HKDF_ALG PSA_ALG_HKDF(PSA_ALG_SHA_256)
#endif

struct bt_acs_key_desc_runtime *acs_key_exchange_established_key(struct bt_acs_conn *acs_conn)
{
#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	struct bt_acs_key_desc_runtime *kdf_key;

	if (acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_KDF, &kdf_key) == 0 &&
	    kdf_key->psa_key_id != 0U) {
		return kdf_key;
	}
#endif

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
	{
		struct bt_acs_key_desc_runtime *ecdh_key;

		if (acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_ECDH, &ecdh_key) == 0 &&
		    ecdh_key->psa_key_id != 0U) {
			return ecdh_key;
		}
	}
#endif

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_OOB)
	{
		struct bt_acs_key_desc_runtime *oob_key;

		if (acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_OOB, &oob_key) == 0 &&
		    oob_key->psa_key_id != 0U) {
			return oob_key;
		}
	}
#endif

	return NULL;
}

int acs_key_exchange_step_response(struct acs_procedure *proc, uint8_t status)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;
	struct acs_reply_mode reply_mode = acs_proc_reply_mode(proc);
	uint8_t payload[3];
	uint16_t key_id;
	struct net_buf *buf;

	if (!acs_conn || !acs_kex_in_progress(acs_conn)) {
		return -EINVAL;
	}

	key_id = sys_le16_to_cpu(acs_conn->kex.start_kex.key_id);
	sys_put_le16(key_id, &payload[0]);
	payload[2] = status;

	buf = acs_prepare_reply_buf(proc, reply_mode.encrypted);
	if (!buf) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_RESPONSE);
	net_buf_add_mem(buf, payload, sizeof(payload));
	return acs_cp_send_reply(proc);
}

int acs_key_exchange_step_success_status(struct acs_procedure *proc)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;

	memset(&acs_conn->kex, 0, sizeof(acs_conn->kex));

	acs_conn->status_flags |= BT_ACS_STATUS_SECURITY_ESTABLISHED;
	{
		const struct bt_acs_cb *cb = acs_cb_get();

		if (cb && cb->security_established) {
			cb->security_established(proc->acs_conn->conn);
		}
	}

#if defined(CONFIG_BT_SETTINGS)
	acs_session_store(proc->acs_conn->conn, acs_conn);
#endif

	acs_seq_clear(proc);
	acs_status_indicate(acs_conn->conn);
	return 0;
}

static int check_expected_opcode(struct bt_acs_conn const *acs_conn, uint8_t expected_opcode)
{
	if (!acs_kex_accepts_opcode(acs_conn, expected_opcode)) {
		LOG_WRN("Invalid KEX transition (requested opcode 0x%02x, current next 0x%02x)",
			expected_opcode,
			acs_kex_in_progress(acs_conn) ? acs_conn->kex.next_expected_opcode : 0U);
		return -EAGAIN;
	}
	return 0;
}

static void acs_key_exchange_free_ctx(struct bt_acs_conn *acs_conn)
{
	if (!acs_kex_in_progress(acs_conn)) {
		return;
	}

	if (acs_conn->kex.ecdh_key_id != 0U) {
		acs_psa_destroy_key(&acs_conn->kex.ecdh_key_id);
	}

	memset(&acs_conn->kex, 0, sizeof(acs_conn->kex));
}

void acs_key_exchange_abort(struct bt_acs_conn *acs_conn)
{
	uint16_t key_id;

	if (!acs_kex_in_progress(acs_conn)) {
		return;
	}

	key_id = sys_le16_to_cpu(acs_conn->kex.start_kex.key_id);

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
	if (key_id == ACS_KEY_ID_KDF) {
		/* Spec §4.4.3.17.2: error during KDF kex restarts at Start Key
		 * Exchange (KDF), which §4.4.3.17.2.1 requires a live parent for.
		 * Tear down only the in-progress KDF child + its algorithm
		 * children; leave the ECDH parent so the peer can retry KDF.
		 */
		struct bt_acs_key_desc_runtime *kdf_key;

		if (acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_KDF, &kdf_key) == 0) {
			acs_crypto_destroy_key(kdf_key);
		}

		acs_crypto_invalidate_algorithm_keys(acs_conn);
		acs_key_exchange_free_ctx(acs_conn);
		return;
	}
#endif

	acs_crypto_destroy_exchange_keys(acs_conn);
	acs_crypto_invalidate_algorithm_keys(acs_conn);
	acs_key_exchange_free_ctx(acs_conn);
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

/*
 * IKM source for an HKDF op.  Setting key_id != 0 selects the PSA derive-key
 * path (input_key); otherwise the bytes/bytes_len path (input_bytes) is used.
 * The two paths are otherwise identical, which is why every HKDF helper below
 * threads this struct through instead of duplicating its logic.
 */
struct acs_hkdf_ikm {
	psa_key_id_t key_id;
	const uint8_t *bytes;
	size_t bytes_len;
};

static int acs_hkdf_op_setup(psa_key_derivation_operation_t *op, psa_algorithm_t alg,
			     const uint8_t *salt, size_t salt_len, const struct acs_hkdf_ikm *ikm,
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

	if (ikm->key_id != 0U) {
		status = psa_key_derivation_input_key(op, PSA_KEY_DERIVATION_INPUT_SECRET,
						      ikm->key_id);
	} else {
		status = psa_key_derivation_input_bytes(op, PSA_KEY_DERIVATION_INPUT_SECRET,
							ikm->bytes, ikm->bytes_len);
	}
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

static int acs_hkdf_output_bytes(psa_algorithm_t alg, const uint8_t *salt, size_t salt_len,
				 const struct acs_hkdf_ikm *ikm, const uint8_t *info,
				 size_t info_len, uint8_t *out, size_t out_len)
{
	psa_key_derivation_operation_t op = PSA_KEY_DERIVATION_OPERATION_INIT;
	int ret = acs_hkdf_op_setup(&op, alg, salt, salt_len, ikm, info, info_len);

	if (ret == 0) {
		psa_status_t status = psa_key_derivation_output_bytes(&op, out, out_len);

		if (status != PSA_SUCCESS) {
			LOG_ERR("HKDF output failed: status=%d, out_len=%zu", status, out_len);
			ret = -EIO;
		}
	}

	return acs_hkdf_op_abort(&op, ret);
}

/*
 * Materialise an exchange-key pair (operational AES + derive twin) into
 * @p key_runtime.  Two parallel derivation ops are required because
 * psa_key_derivation_output_key advances the stream - a single op outputting
 * two keys would produce two distinct secrets, but the AEAD key and its derive
 * twin must hold the same underlying material so the peer's KDF / nonce
 * derivations stay in lock-step.  Both ops are aborted before return.
 *
 * Any prior PSA keys on @p key_runtime are destroyed first.
 */
static int acs_crypto_derive_exchange_key_pair(struct bt_acs_key_desc_runtime *key_runtime,
					       psa_algorithm_t alg, const uint8_t *salt,
					       size_t salt_len, const struct acs_hkdf_ikm *ikm,
					       const uint8_t *info, size_t info_len, size_t key_len)
{
	psa_key_derivation_operation_t op_key = PSA_KEY_DERIVATION_OPERATION_INIT;
	psa_key_derivation_operation_t op_derive = PSA_KEY_DERIVATION_OPERATION_INIT;
	int ret;

	ret = acs_hkdf_op_setup(&op_key, alg, salt, salt_len, ikm, info, info_len);
	if (ret) {
		goto cleanup;
	}

	ret = acs_hkdf_op_setup(&op_derive, alg, salt, salt_len, ikm, info, info_len);
	if (ret) {
		goto cleanup;
	}

	acs_crypto_destroy_key(key_runtime);

	ret = acs_crypto_output_exchange_key(key_runtime, &op_key, key_len);
	if (ret) {
		goto cleanup;
	}

	ret = acs_crypto_output_exchange_derive_key(key_runtime, &op_derive, key_len);

cleanup:
	ret = acs_hkdf_op_abort(&op_key, ret);
	ret = acs_hkdf_op_abort(&op_derive, ret);
	return ret;
}

static size_t acs_nonce_label_build(uint16_t key_id, uint8_t kind, uint8_t *label, size_t len)
{
	static const uint8_t prefix[] = "ACS:nonce:";

	__ASSERT_NO_MSG(len >= sizeof(prefix) + sizeof(kind) + sizeof(key_id));

	memcpy(label, prefix, sizeof(prefix) - 1U);
	label[sizeof(prefix) - 1U] = kind;
	sys_put_le16(key_id, &label[sizeof(prefix)]);

	return sizeof(prefix) + sizeof(key_id);
}

int acs_derive_nonce_state(struct bt_acs_key_desc_runtime *runtime,
			   const struct bt_acs_key_desc_runtime *parent)
{
	const struct bt_acs_key_desc_record *key_desc = runtime->key_desc;
	uint8_t prefix_size = acs_key_desc_nonce_prefix_size(key_desc);
	struct acs_hkdf_ikm ikm;
	uint8_t label[sizeof("ACS:nonce:") + sizeof(uint8_t) + sizeof(uint16_t)];
	size_t label_len;
	int ret;

	runtime->tx_nonce_counter = 0U;
	runtime->rx_nonce_counter = key_desc->aes.nonce_type == ACS_NONCE_SEQ_EVEN_ODD ? 1U : 0U;

	if (prefix_size == 0U) {
		return 0;
	}

#if IS_ENABLED(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD)
	if (key_desc->aes.nonce_type == ACS_NONCE_SEQ_EVEN_ODD) {
		if (!parent || parent->derive_key_id == 0U) {
			return -EINVAL;
		}

		memset(runtime->server_nonce_fixed, 0, sizeof(runtime->server_nonce_fixed));
		memset(runtime->client_nonce_fixed, 0, sizeof(runtime->client_nonce_fixed));
		runtime->client_nonce_set = false;

		label_len = acs_nonce_label_build(acs_key_desc_runtime_key_id(runtime), 'e', label,
						  sizeof(label));
		ikm = (struct acs_hkdf_ikm){.key_id = parent->derive_key_id};
		ret = acs_hkdf_output_bytes(ACS_PSA_HKDF_ALG, NULL, 0U, &ikm, label, label_len,
					    runtime->server_nonce_fixed, prefix_size);
		if (ret != 0) {
			return ret;
		}
		memcpy(runtime->client_nonce_fixed, runtime->server_nonce_fixed, prefix_size);
		runtime->client_nonce_set = true;
		return 0;
	}
#endif

	{
		bool nonce_unset = true;

		for (uint8_t i = 0U; i < prefix_size; i++) {
			if (runtime->server_nonce_fixed[i] != 0U) {
				nonce_unset = false;
				break;
			}
		}
		if (nonce_unset) {
			sys_rand_get(runtime->server_nonce_fixed, prefix_size);
			sys_mem_swap(runtime->server_nonce_fixed, prefix_size);
		}
	}

	return 0;
}

int acs_crypto_activate_key(struct bt_acs_conn *acs_conn,
			    struct bt_acs_key_desc_runtime *exchange_key,
			    const uint8_t *key_material, size_t key_len)
{
	int err;

	acs_crypto_destroy_key(exchange_key);

	err = acs_crypto_import_exchange_key(exchange_key, key_material, key_len);
	if (err) {
		return err;
	}

	/* bind_algorithm_keys handles per-record nonce state setup (counters +
	 * fixed prefix) as part of the same loop that mints the child keys.
	 */
	return acs_crypto_bind_algorithm_keys(acs_conn, exchange_key);
}

/* §4.4.3.17.1.2: Salt=HMAC(Zero,PKs||PKc), ConfKey=HMAC(Salt,ECDHKey||Auth),
 * Code=HMAC(ConfKey,Random). Coords supplied in LE (wire format); reversed to BE internally. Pass
 * NULL for Y on Curve25519.
 */
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
	uint8_t salt[PSA_HASH_LENGTH(PSA_ALG_SHA_256)];
	uint8_t pubkey_concat[CONFIG_BT_ACS_ECDH_COORD_SIZE * 4];
	size_t pubkey_concat_len = 0;
	uint8_t confirmation_key[PSA_HASH_LENGTH(PSA_ALG_SHA_256)];
	uint8_t ecdh_auth[CONFIG_BT_ACS_ECDH_COORD_SIZE + ACS_CONFIRM_VALUE_SIZE];

	/* Step 1: Salt = HMAC(Zero, PKsx_BE||PKsy_BE||PKcx_BE||PKcy_BE) */
	sys_memcpy_swap(&pubkey_concat[pubkey_concat_len], server_x_le, coord_size);
	pubkey_concat_len += coord_size;

	if (server_y_le != NULL) {
		sys_memcpy_swap(&pubkey_concat[pubkey_concat_len], server_y_le, coord_size);
		pubkey_concat_len += coord_size;
	}

	sys_memcpy_swap(&pubkey_concat[pubkey_concat_len], client_x_le, coord_size);
	pubkey_concat_len += coord_size;

	if (client_y_le != NULL) {
		sys_memcpy_swap(&pubkey_concat[pubkey_concat_len], client_y_le, coord_size);
		pubkey_concat_len += coord_size;
	}

	ret = acs_hmac_sha256(zero_key, sizeof(zero_key), pubkey_concat, pubkey_concat_len, salt);
	if (ret != 0) {
		LOG_ERR("HMAC-SHA-256 failed in deriving confirmation salt: %d", ret);
		goto cleanup;
	}

	/* Step 2: ConfirmationKey = HMAC(Salt, ECDHKey || AuthValue) */
	memcpy(&ecdh_auth[0], ecdh_key, ecdh_key_len);
	memcpy(&ecdh_auth[ecdh_key_len], auth_value, ACS_CONFIRM_VALUE_SIZE);

	ret = acs_hmac_sha256(salt, sizeof(salt), ecdh_auth, ecdh_key_len + ACS_CONFIRM_VALUE_SIZE,
			      confirmation_key);
	if (ret != 0) {
		LOG_ERR("HMAC-SHA-256 failed in deriving confirmation key: %d", ret);
		goto cleanup;
	}

	/* Step 3: ConfirmationCode = HMAC(ConfirmationKey, RandomNumber) */
	ret = acs_hmac_sha256(confirmation_key, sizeof(confirmation_key), random,
			      ACS_CONFIRM_VALUE_SIZE, confirm_out);
	if (ret != 0) {
		LOG_ERR("HMAC-SHA-256 failed in deriving confirmation code: %d", ret);
	}

cleanup:
	mbedtls_platform_zeroize(salt, sizeof(salt));
	mbedtls_platform_zeroize(confirmation_key, sizeof(confirmation_key));
	mbedtls_platform_zeroize(ecdh_auth, sizeof(ecdh_auth));
	return ret;
}

static int acs_crypto_generate_keypair(struct bt_acs_conn *acs_conn)
{
	struct acs_ecdh_pubkey *pk = &acs_conn->kex.server_pubkey;
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;
	uint8_t pub[1U + 2U * CONFIG_BT_ACS_ECDH_COORD_SIZE];
	size_t pub_len;

	psa_set_key_type(&attrs, PSA_KEY_TYPE_ECC_KEY_PAIR(ACS_PSA_ECC_FAMILY));
	psa_set_key_bits(&attrs, ACS_PSA_KEY_BITS);
	psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_DERIVE);
	psa_set_key_algorithm(&attrs, PSA_ALG_ECDH);

	status = psa_generate_key(&attrs, &acs_conn->kex.ecdh_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_generate_key failed: %d", status);
		return -EIO;
	}

	/* Export public key to populate server_pubkey coords */
	status = psa_export_public_key(acs_conn->kex.ecdh_key_id, pub, sizeof(pub), &pub_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_export_public_key failed: %d", status);
		acs_psa_destroy_key(&acs_conn->kex.ecdh_key_id);
		return -EIO;
	}

	pk->key_id = sys_cpu_to_le16(ACS_KEY_ID_ECDH);
	pk->x_size = CONFIG_BT_ACS_ECDH_COORD_SIZE;
	pk->y_size = CONFIG_BT_ACS_ECDH_HAS_Y ? CONFIG_BT_ACS_ECDH_COORD_SIZE : 0;

#if IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_CURVE25519)
	/* Curve25519: PSA exports raw X_LE, no prefix */
	if (pub_len != (size_t)CONFIG_BT_ACS_ECDH_COORD_SIZE) {
		LOG_ERR("Unexpected Curve25519 public key size: %zu (expected %u)", pub_len,
			CONFIG_BT_ACS_ECDH_COORD_SIZE);
		acs_psa_destroy_key(&acs_conn->kex.ecdh_key_id);
		return -EIO;
	}
	memcpy(pk->x, pub, CONFIG_BT_ACS_ECDH_COORD_SIZE);
#else
	/* NIST: PSA exports [0x04][X_BE][Y_BE]; swap to LE for ACS wire format */
	size_t expected_len = 1U + 2U * CONFIG_BT_ACS_ECDH_COORD_SIZE;
	const uint8_t *x_be = &pub[1];

	if (pub_len != expected_len) {
		LOG_ERR("Unexpected NIST public key size: %zu (expected %zu)", pub_len,
			expected_len);
		acs_psa_destroy_key(&acs_conn->kex.ecdh_key_id);
		return -EIO;
	}

	sys_memcpy_swap(pk->x, x_be, CONFIG_BT_ACS_ECDH_COORD_SIZE); /* X: BE→LE */
#if CONFIG_BT_ACS_ECDH_HAS_Y
	const uint8_t *y_be = &pub[1U + CONFIG_BT_ACS_ECDH_COORD_SIZE];

	sys_memcpy_swap(pk->y, y_be, CONFIG_BT_ACS_ECDH_COORD_SIZE); /* Y: BE→LE */
#endif
#endif /* IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_CURVE25519) */

	return 0;
}

static int acs_crypto_compute_shared_secret(struct bt_acs_conn *acs_conn)
{
	const struct acs_ecdh_pubkey *cpk = &acs_conn->kex.client_pubkey;
	uint8_t x_size = cpk->x_size;
	const uint8_t *x_le = cpk->x;
	uint8_t psa_pubkey[1U + 2U * CONFIG_BT_ACS_ECDH_COORD_SIZE];
	size_t psa_pubkey_len;
	size_t olen;
	psa_status_t status;
#if CONFIG_BT_ACS_ECDH_HAS_Y
	uint8_t y_size = cpk->y_size;
	const uint8_t *y_le = cpk->y;
#endif

	/* Reject if coord sizes don't match the compiled-in curve */
	if (x_size != CONFIG_BT_ACS_ECDH_COORD_SIZE
#if CONFIG_BT_ACS_ECDH_HAS_Y
	    || y_size != CONFIG_BT_ACS_ECDH_COORD_SIZE
#endif
	) {
		LOG_ERR("Client pubkey coord size mismatch: x=%u (expected %u)", x_size,
			CONFIG_BT_ACS_ECDH_COORD_SIZE);
		acs_psa_destroy_key(&acs_conn->kex.ecdh_key_id);
		return -EINVAL;
	}

	/* Reject if client public key equals server public key (spec 4.4.3.17.1.1) */
	if (memcmp(cpk->x, acs_conn->kex.server_pubkey.x, CONFIG_BT_ACS_ECDH_COORD_SIZE) == 0
#if CONFIG_BT_ACS_ECDH_HAS_Y
	    && memcmp(cpk->y, acs_conn->kex.server_pubkey.y, CONFIG_BT_ACS_ECDH_COORD_SIZE) == 0
#endif
	) {
		LOG_WRN("client public key matches server public key");
		acs_psa_destroy_key(&acs_conn->kex.ecdh_key_id);
		return -EINVAL;
	}

#if IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_CURVE25519)
	memcpy(psa_pubkey, x_le, x_size);
	psa_pubkey_len = x_size;
#else
	/* NIST: PSA expects [0x04][X_BE][Y_BE] */
	psa_pubkey[0] = ACS_ECDH_UNCOMPRESSED_POINT;
	sys_memcpy_swap(&psa_pubkey[1], x_le, x_size);
	if (y_le != NULL && y_size > 0U) {
		sys_memcpy_swap(&psa_pubkey[1U + x_size], y_le, y_size);
	}
	psa_pubkey_len = 1U + x_size + y_size;
#endif

	status = psa_raw_key_agreement(PSA_ALG_ECDH, acs_conn->kex.ecdh_key_id, psa_pubkey,
				       psa_pubkey_len, acs_conn->kex.key_material,
				       sizeof(acs_conn->kex.key_material), &olen);

	acs_psa_destroy_key(&acs_conn->kex.ecdh_key_id);

	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_raw_key_agreement failed: %d", status);
		if (status == PSA_ERROR_INVALID_ARGUMENT) {
			return -EBADMSG;
		}
		return -EIO;
	}

	acs_conn->kex.key_mat_len = (uint16_t)olen;

	return 0;
}

int acs_crypto_derive_session_key(struct bt_acs_conn *acs_conn,
				  const uint8_t client_random[ACS_CONFIRM_VALUE_SIZE])
{
	static const uint8_t info[] = "ACS";
	uint16_t key_id = sys_le16_to_cpu(acs_conn->kex.start_kex.key_id);
	struct bt_acs_key_desc_runtime *exchange_key;
	struct acs_hkdf_ikm ikm = {
		.bytes = acs_conn->kex.key_material,
		.bytes_len = acs_conn->kex.key_mat_len,
	};
	uint8_t salt[2 * ACS_CONFIRM_VALUE_SIZE];
	uint8_t client_random_be[ACS_CONFIRM_VALUE_SIZE];
	int ret;

	ret = acs_crypto_key_runtime_lookup(acs_conn, key_id, &exchange_key);
	if (ret) {
		LOG_ERR("No runtime key state for exchange Key_ID 0x%04x", key_id);
		return ret;
	}

	/* salt = ServerRandom_BE || ClientRandom_BE */
	sys_memcpy_swap(client_random_be, client_random, ACS_CONFIRM_VALUE_SIZE);
	memcpy(salt, acs_conn->kex.server_random, ACS_CONFIRM_VALUE_SIZE);
	memcpy(salt + ACS_CONFIRM_VALUE_SIZE, client_random_be, ACS_CONFIRM_VALUE_SIZE);

	ret = acs_crypto_derive_exchange_key_pair(exchange_key, ACS_PSA_HKDF_ALG, salt,
						  sizeof(salt), &ikm, info, sizeof(info) - 1,
						  CONFIG_BT_ACS_SESSION_KEY_SIZE);
	if (ret) {
		LOG_ERR("HKDF session key derivation failed: %d", ret);
		return ret;
	}

	return acs_crypto_bind_algorithm_keys(acs_conn, exchange_key);
}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
/* §4.4.3.17.2.1: Derive the KDF child from the parent's derive-key via HKDF.
 * Both the operational AES child and its derive twin are materialised directly
 * in the PSA keystore via output_key - no plaintext key material at any step.
 */
static int acs_crypto_derive_kdf_child_key(struct bt_acs_conn *acs_conn)
{
	struct bt_acs_key_desc_runtime *parent_key;
	struct bt_acs_key_desc_runtime *kdf_key;
	uint8_t salt_be[CONFIG_BT_ACS_KDF_SALT_MAX_SIZE];
	uint8_t info_be[CONFIG_BT_ACS_KDF_INFO_MAX_SIZE];
	struct acs_hkdf_ikm ikm;
	int ret;

	ret = acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_ECDH, &parent_key);
	if (ret) {
		LOG_ERR("No runtime key state for ECDH parent");
		return ret;
	}

	ret = acs_crypto_key_runtime_lookup(acs_conn, ACS_KEY_ID_KDF, &kdf_key);
	if (ret) {
		LOG_ERR("No runtime key state for KDF child");
		return ret;
	}

	if (parent_key->derive_key_id == 0U) {
		LOG_ERR("ECDH parent derive key missing");
		return -ENOENT;
	}

	/* Salt and info are stored in wire order (LSO) but HKDF requires BE input. */
	sys_memcpy_swap(salt_be, acs_conn->kex.kdf.salt, acs_conn->kex.kdf.salt_size);
	sys_memcpy_swap(info_be, acs_conn->kex.kdf.info, acs_conn->kex.kdf.info_size);

	ikm = (struct acs_hkdf_ikm){.key_id = parent_key->derive_key_id};

	ret = acs_crypto_derive_exchange_key_pair(
		kdf_key, ACS_PSA_HKDF_ALG, salt_be, acs_conn->kex.kdf.salt_size, &ikm, info_be,
		acs_conn->kex.kdf.info_size, CONFIG_BT_ACS_SESSION_KEY_SIZE);
	if (ret) {
		return ret;
	}

	return acs_crypto_bind_algorithm_keys(acs_conn, kdf_key);
}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF */

static int acs_kdf_generate_params(struct bt_acs_kex_ctx *kex)
{
	psa_status_t psa_ret;

	psa_ret = psa_generate_random(kex->kdf.salt, CONFIG_BT_ACS_KDF_SALT_MAX_SIZE);
	if (psa_ret != PSA_SUCCESS) {
		LOG_ERR("psa_generate_random failed for KDF salt: %d", psa_ret);
		return -EIO;
	}
	kex->kdf.salt_size = CONFIG_BT_ACS_KDF_SALT_MAX_SIZE;

#if CONFIG_BT_ACS_KDF_INFO_MAX_SIZE > 0
	{
		const char *acs_info = "ACS";

		memset(kex->kdf.info, 0, CONFIG_BT_ACS_KDF_INFO_MAX_SIZE);
		memcpy(kex->kdf.info, acs_info, strlen(acs_info));
		kex->kdf.info_size = strlen(acs_info);
	}
#else
	kex->kdf.info_size = 0;
#endif

	return 0;
}

static int acs_kdf_serialize_response(struct bt_acs_kex_ctx *kex, struct net_buf_simple *rsp_buf)
{
	uint8_t needed = ACS_KDF_RSP_FIXED_SIZE + kex->kdf.salt_size + kex->kdf.info_size;

	if (net_buf_simple_tailroom(rsp_buf) < needed) {
		LOG_WRN("KDF response buffer too small: need %u, have %u", needed,
			net_buf_simple_tailroom(rsp_buf));
		return -ENOMEM;
	}

	net_buf_simple_add_le16(rsp_buf, sys_le16_to_cpu(kex->start_kex.key_id));
	net_buf_simple_add_u8(rsp_buf, kex->kdf.salt_size);
	net_buf_simple_add_mem(rsp_buf, kex->kdf.salt, kex->kdf.salt_size);
	net_buf_simple_add_u8(rsp_buf, kex->kdf.info_size);
	net_buf_simple_add_mem(rsp_buf, kex->kdf.info, kex->kdf.info_size);

	return 0;
}

int acs_key_exchange_ecdh_start(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	if (key_id != ACS_KEY_ID_ECDH && key_id != ACS_KEY_ID_OOB) {
		LOG_ERR("key_id 0x%04x does not match supported exchange key IDs", key_id);
		return -EALREADY;
	}

	/* Destroy any ECDH private key left from a prior aborted exchange. */
	if (acs_kex_in_progress(acs_conn) && acs_conn->kex.ecdh_key_id != 0U) {
		acs_psa_destroy_key(&acs_conn->kex.ecdh_key_id);
	}

	/* Tear down any prior exchange keys and their algorithm-record children;
	 * the fresh key exchange about to start will repopulate everything.
	 * Algorithm-record nonce_fixed values carry over - either lazily
	 * regenerated by acs_derive_nonce_state on first use (DIFF_FIXED with
	 * all-zero state) or re-derived from the new parent (EVEN_ODD).
	 */
	acs_crypto_destroy_exchange_keys(acs_conn);
	acs_crypto_destroy_connection_record_keys(acs_conn);
	memset(&acs_conn->kex, 0, sizeof(acs_conn->kex));
	acs_conn->kex.active = true;

	if (key_id == ACS_KEY_ID_ECDH) {
		int err = acs_crypto_generate_keypair(acs_conn);

		if (err) {
			return -EIO;
		}
	}

	LOG_INF("ECDH Procedure Started");
	return 0;
}

int acs_key_exchange_ecdh_pubkey(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf)
{
	int err = check_expected_opcode(acs_conn, BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH);

	if (err) {
		return err;
	}

	err = acs_crypto_compute_shared_secret(acs_conn);
	if (err) {
		return err;
	}

	if (net_buf_simple_tailroom(rsp_buf) < sizeof(acs_conn->kex.server_pubkey)) {
		LOG_ERR("Not enough tailroom for server_pubkey in response buffer: need %zu, have "
			"%zu",
			sizeof(acs_conn->kex.server_pubkey), net_buf_simple_tailroom(rsp_buf));
		return -ENOMEM;
	}

	net_buf_simple_add_mem(rsp_buf, &acs_conn->kex.server_pubkey,
			       sizeof(acs_conn->kex.server_pubkey));

	if (sys_le16_to_cpu(acs_conn->kex.start_kex.key_id) == ACS_KEY_ID_ECDH) {
		acs_conn->kex.next_expected_opcode = BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF;
	} else {
		acs_conn->kex.next_expected_opcode = BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE;
	}
	LOG_INF("ECDH Public Keys Exchanged");
	return 0;
}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
int acs_key_exchange_ecdh_kdf(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf)
{
	uint8_t salt_be[CONFIG_BT_ACS_KDF_SALT_MAX_SIZE];
	uint8_t info_be[CONFIG_BT_ACS_KDF_INFO_MAX_SIZE];
	int err = check_expected_opcode(acs_conn, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF);

	if (err) {
		return err;
	}

	err = acs_kdf_generate_params(&acs_conn->kex);
	if (err) {
		return err;
	}

	/* HKDF overwrites key_material in-place (safe: PSA reads IKM fully
	 * during extract before writing during expand).
	 */
	sys_memcpy_swap(salt_be, acs_conn->kex.kdf.salt, acs_conn->kex.kdf.salt_size);
	sys_memcpy_swap(info_be, acs_conn->kex.kdf.info, acs_conn->kex.kdf.info_size);

	struct acs_hkdf_ikm ikm = {
		.bytes = acs_conn->kex.key_material,
		.bytes_len = acs_conn->kex.key_mat_len,
	};

	err = acs_hkdf_output_bytes(ACS_PSA_HKDF_ALG, salt_be, acs_conn->kex.kdf.salt_size, &ikm,
				    info_be, acs_conn->kex.kdf.info_size,
				    acs_conn->kex.key_material, CONFIG_BT_ACS_SESSION_KEY_SIZE);
	if (err != 0) {
		LOG_ERR("HKDF ECDHKey derivation failed: %d", err);
		return err;
	}

	acs_conn->kex.key_mat_len = CONFIG_BT_ACS_SESSION_KEY_SIZE;
	acs_conn->kex.kdf_applied = true;

	err = acs_kdf_serialize_response(&acs_conn->kex, rsp_buf);
	if (err) {
		return err;
	}

	acs_conn->kex.next_expected_opcode = BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE;
	LOG_INF("KDF Complete, ECDHKey Derived");
	return 0;
}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

int acs_key_exchange_ecdh_confirm_code(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf)
{
	const uint8_t *key_mat;
	uint16_t key_mat_len;
	int err;
	uint8_t server_confirm[ACS_CONFIRM_VALUE_SIZE];
	uint8_t server_confirm_le[ACS_CONFIRM_VALUE_SIZE];

	err = check_expected_opcode(acs_conn, BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE);

	if (err) {
		return err;
	}

	/* Generate fresh server random; client_confirm verification deferred to confirm_rand */
	sys_rand_get(acs_conn->kex.server_random, sizeof(acs_conn->kex.server_random));

	key_mat = acs_conn->kex.key_material;
	key_mat_len = acs_conn->kex.key_mat_len;

	err = acs_ecdh_confirm_code_compute(acs_conn->kex.server_pubkey.x,
#if CONFIG_BT_ACS_ECDH_HAS_Y
					    acs_conn->kex.server_pubkey.y,
#else
					    NULL,
#endif
					    acs_conn->kex.client_pubkey.x,
#if CONFIG_BT_ACS_ECDH_HAS_Y
					    acs_conn->kex.client_pubkey.y,
#else
					    NULL,
#endif
					    CONFIG_BT_ACS_ECDH_COORD_SIZE, key_mat, key_mat_len,
					    acs_conn->kex.auth_value, acs_conn->kex.server_random,
					    server_confirm);

	if (err) {
		LOG_ERR("confirm code compute failed: %d", err);
		return -EIO;
	}

	/* Response: Key_ID(2 LE) + ACServerConfirmationCode(32, LSO first) */
	if (net_buf_simple_tailroom(rsp_buf) < sizeof(uint16_t) + ACS_CONFIRM_VALUE_SIZE) {
		return -ENOMEM;
	}

	net_buf_simple_add_le16(rsp_buf, sys_le16_to_cpu(acs_conn->kex.start_kex.key_id));

	/* Confirmation code is BE; reverse to LE for wire */
	sys_memcpy_swap(server_confirm_le, server_confirm, ACS_CONFIRM_VALUE_SIZE);
	net_buf_simple_add_mem(rsp_buf, server_confirm_le, ACS_CONFIRM_VALUE_SIZE);
	acs_conn->kex.next_expected_opcode = BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND;
	LOG_INF("Server Confirmation Code Sent");
	return 0;
}

int acs_key_exchange_ecdh_confirm_rand(struct bt_acs_conn *acs_conn,
				       const uint8_t client_random[ACS_CONFIRM_VALUE_SIZE],
				       struct net_buf_simple *rsp_buf)
{
	const uint8_t *key_mat;
	uint16_t key_mat_len;
	uint8_t computed[ACS_CONFIRM_VALUE_SIZE];
	uint8_t client_random_be[ACS_CONFIRM_VALUE_SIZE];
	uint8_t client_confirm_be[ACS_CONFIRM_VALUE_SIZE];
	uint8_t server_random_le[sizeof(acs_conn->kex.server_random)];
	uint8_t diff = 0;
	int err = check_expected_opcode(acs_conn, BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND);

	if (err) {
		return err;
	}

	/* Reject if client random equals server random (spec 4.4.3.17.1.3) */
	if (memcmp(client_random, acs_conn->kex.server_random, ACS_CONFIRM_VALUE_SIZE) == 0) {
		LOG_WRN("client random matches server random");
		return -EINVAL;
	}

	key_mat = acs_conn->kex.key_material;
	key_mat_len = acs_conn->kex.key_mat_len;

	/* client_random is wire LE; reverse to BE for HMAC */
	sys_memcpy_swap(client_random_be, client_random, ACS_CONFIRM_VALUE_SIZE);

	err = acs_ecdh_confirm_code_compute(acs_conn->kex.server_pubkey.x,
#if CONFIG_BT_ACS_ECDH_HAS_Y
					    acs_conn->kex.server_pubkey.y,
#else
					    NULL,
#endif
					    acs_conn->kex.client_pubkey.x,
#if CONFIG_BT_ACS_ECDH_HAS_Y
					    acs_conn->kex.client_pubkey.y,
#else
					    NULL,
#endif
					    CONFIG_BT_ACS_ECDH_COORD_SIZE, key_mat, key_mat_len,
					    acs_conn->kex.auth_value, client_random_be, computed);

	if (err) {
		LOG_ERR("confirm code compute failed during verify: %d", err);
		return -EIO;
	}

	/* client_confirm is wire LE; reverse to BE before constant-time compare */
	sys_memcpy_swap(client_confirm_be, acs_conn->kex.client_confirm, ACS_CONFIRM_VALUE_SIZE);

	for (int i = 0; i < ACS_CONFIRM_VALUE_SIZE; i++) {
		diff |= client_confirm_be[i] ^ computed[i];
	}

	if (diff) {
		LOG_ERR("Client confirmation code mismatch - authentication failed");
		return -EACCES;
	}

	/* Response: Key_ID(2 LE) + ACServerConfirmationRandomNumber(32, LSO first) */
	if (net_buf_simple_tailroom(rsp_buf) <
	    sizeof(uint16_t) + sizeof(acs_conn->kex.server_random)) {
		return -ENOMEM;
	}

	net_buf_simple_add_le16(rsp_buf, sys_le16_to_cpu(acs_conn->kex.start_kex.key_id));

	/* server_random is BE; reverse to LE for wire */
	sys_memcpy_swap(server_random_le, acs_conn->kex.server_random,
			sizeof(acs_conn->kex.server_random));
	net_buf_simple_add_mem(rsp_buf, server_random_le, sizeof(acs_conn->kex.server_random));

	acs_conn->kex.next_expected_opcode = 0U;
	LOG_INF("Client Confirmed - Server Random Sent");
	return 0;
}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
int acs_key_exchange_kdf(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf)
{
	int err = check_expected_opcode(acs_conn, BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF);

	if (err) {
		return err;
	}

	err = acs_kdf_generate_params(&acs_conn->kex);
	if (err) {
		return err;
	}

	/* Derive child key: HKDF(salt, ikm=parent_key, info) → child replaces parent_key */
	err = acs_crypto_derive_kdf_child_key(acs_conn);
	if (err) {
		LOG_ERR("KDF child key derivation failed: %d", err);
		return -EIO;
	}

	err = acs_kdf_serialize_response(&acs_conn->kex, rsp_buf);
	if (err) {
		return err;
	}

	acs_conn->kex.next_expected_opcode = 0U;
	LOG_INF("KDF child key derived success");
	return 0;
}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF */
