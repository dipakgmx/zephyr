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

K_MEM_SLAB_DEFINE_STATIC(acs_kex_slab, sizeof(struct bt_acs_kex_ctx), CONFIG_BT_ACS_KEX_CTX_COUNT,
			 __alignof__(struct bt_acs_kex_ctx));

struct bt_acs_key_desc_runtime *acs_key_exchange_established_key(struct bt_acs_conn const *acs_conn)
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

/* Zeroize the transcript (it held the shared secret and auth value) and
 * release the context; the connection is idle for key exchange afterwards.
 */
static void acs_kex_free(struct bt_acs_conn *acs_conn)
{
	struct bt_acs_kex_ctx *kex = acs_conn->kex;

	if (!kex) {
		return;
	}

	acs_psa_destroy_key(&kex->ecdh_key_id);
	acs_psa_destroy_key(&kex->derived_key_id);

	acs_conn->kex = NULL;
	mbedtls_platform_zeroize(kex, sizeof(*kex));
	k_mem_slab_free(&acs_kex_slab, kex);
}

/* Send Key Exchange Response carrying the exchange verdict (Table 4.77). */
static int acs_kex_send_result(struct acs_procedure *proc)
{
	struct bt_acs_conn const *acs_conn = proc->acs_conn;
	uint8_t payload[3];
	struct net_buf *buf;

	sys_put_le16(sys_le16_to_cpu(acs_conn->kex->start_kex.key_id), &payload[0]);
	payload[2] = acs_conn->kex->failed ? 0x01 : 0x00;

	buf = acs_prepare_reply_buf(proc);
	if (!buf) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_KEY_EXCHANGE_RESPONSE);
	net_buf_add_mem(buf, payload, sizeof(payload));
	return acs_cp_send_reply(proc);
}

/* Commit the established exchange: release the transcript, raise
 * SECURITY_ESTABLISHED, persist, and indicate the new status.
 */
static int acs_kex_finalize(struct acs_procedure *proc)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;
	const struct bt_acs_cb *cb = acs_cb_get();

	acs_kex_free(acs_conn);

	acs_conn->status_flags |= BT_ACS_STATUS_SECURITY_ESTABLISHED;
	if (cb && cb->security_established) {
		cb->security_established(acs_conn->conn);
	}

#if defined(CONFIG_BT_SETTINGS)
	acs_session_store(acs_conn->conn, acs_conn);
#endif

	acs_seq_clear(proc);
	acs_status_indicate(acs_conn->conn);
	return 0;
}

void acs_kex_conclude(struct acs_procedure *proc, bool failed)
{
	struct bt_acs_kex_ctx *kex = proc->acs_conn->kex;

	__ASSERT_NO_MSG(kex != NULL);

	kex->failed = failed;
	kex->state = ACS_KEX_SEND_RESULT;
	proc->seq_state = ACS_CP_SEQ_KEX_CONTINUE;
}

int acs_kex_continue(struct acs_procedure *proc)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;
	struct bt_acs_kex_ctx *kex = acs_conn->kex;

	if (!kex) {
		acs_seq_clear(proc);
		return 0;
	}

	switch (kex->state) {
	case ACS_KEX_SEND_RESULT:
		kex->state = ACS_KEX_FINALIZE;
		return acs_kex_send_result(proc);
	case ACS_KEX_FINALIZE:
		if (kex->failed) {
			acs_key_exchange_abort(acs_conn);
			acs_seq_clear(proc);
			return 0;
		}
		return acs_kex_finalize(proc);
	default:
		LOG_WRN("KEX continue in unexpected state %d", kex->state);
		acs_seq_clear(proc);
		return 0;
	}
}

static int check_state(struct bt_acs_conn const *acs_conn, enum acs_kex_state state)
{
	if (!acs_kex_expects(acs_conn, state)) {
		LOG_WRN("Invalid KEX transition (step %d requested, current %d)", state,
			acs_kex_in_progress(acs_conn) ? (int)acs_conn->kex->state : -1);
		return -EAGAIN;
	}
	return 0;
}

void acs_key_exchange_abort(struct bt_acs_conn *acs_conn)
{
	uint16_t key_id;

	if (!acs_kex_in_progress(acs_conn)) {
		return;
	}

	key_id = sys_le16_to_cpu(acs_conn->kex->start_kex.key_id);

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
		acs_kex_free(acs_conn);
		return;
	}
#endif

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

static int __maybe_unused acs_hkdf_output_bytes(psa_algorithm_t alg, const uint8_t *salt,
						size_t salt_len, const struct acs_hkdf_ikm *ikm,
						const uint8_t *info, size_t info_len, uint8_t *out,
						size_t out_len)
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

__maybe_unused size_t acs_nonce_label_build(uint16_t key_id, uint8_t kind, uint8_t *label,
					    size_t len)
{
	static const uint8_t prefix[] = "ACS:nonce:";

	__ASSERT_NO_MSG(len >= sizeof(prefix) + sizeof(kind) + sizeof(key_id));

	memcpy(label, prefix, sizeof(prefix) - 1U);
	label[sizeof(prefix) - 1U] = kind;
	sys_put_le16(key_id, &label[sizeof(prefix)]);

	return sizeof(prefix) + sizeof(key_id);
}

int acs_derive_nonce_state(struct bt_acs_key_desc_runtime *runtime,
			   __maybe_unused const struct bt_acs_key_desc_runtime *parent)
{
	const struct bt_acs_key_desc_record *key_desc = runtime->key_desc;
	uint8_t prefix_size = acs_key_desc_nonce_prefix_size(key_desc);

	runtime->tx_nonce_counter = 0U;
	runtime->rx_nonce_counter = key_desc->aes.nonce_type == ACS_NONCE_SEQ_EVEN_ODD ? 1U : 0U;

	if (prefix_size == 0U) {
		return 0;
	}

#if IS_ENABLED(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD)
	if (key_desc->aes.nonce_type == ACS_NONCE_SEQ_EVEN_ODD) {
		struct acs_hkdf_ikm ikm;
		static const char prefix[] = "ACS:nonce:";
		uint8_t label[sizeof(prefix) + sizeof(uint8_t) + sizeof(uint16_t)];
		size_t label_len;
		int ret;

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

	/** Step 3: ConfirmationCode = HMAC(ConfirmationKey, RandomNumber) **/
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

int acs_crypto_derive_session_key(struct bt_acs_conn *acs_conn,
				  const uint8_t client_random[ACS_CONFIRM_VALUE_SIZE])
{
	static const uint8_t info[] = "ACS";
	uint16_t key_id = sys_le16_to_cpu(acs_conn->kex->start_kex.key_id);
	struct bt_acs_key_desc_runtime *exchange_key;
	struct acs_hkdf_ikm ikm = {
		.bytes = acs_conn->kex->key_material,
		.bytes_len = acs_conn->kex->key_mat_len,
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
	memcpy(salt, acs_conn->kex->server_random, ACS_CONFIRM_VALUE_SIZE);
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
	sys_memcpy_swap(salt_be, acs_conn->kex->kdf.salt, acs_conn->kex->kdf.salt_size);
	sys_memcpy_swap(info_be, acs_conn->kex->kdf.info, acs_conn->kex->kdf.info_size);

	ikm = (struct acs_hkdf_ikm){.key_id = parent_key->derive_key_id};

	ret = acs_crypto_derive_exchange_key_pair(
		kdf_key, ACS_PSA_HKDF_ALG, salt_be, acs_conn->kex->kdf.salt_size, &ikm, info_be,
		acs_conn->kex->kdf.info_size, CONFIG_BT_ACS_SESSION_KEY_SIZE);
	if (ret) {
		return ret;
	}

	return acs_crypto_bind_algorithm_keys(acs_conn, kdf_key);
}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF */

static int acs_kdf_serialize_response(struct bt_acs_kex_ctx const *kex,
				      struct net_buf_simple *rsp_buf)
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

	/* Tear down any prior exchange keys and their algorithm-record children;
	 * the fresh key exchange about to start will repopulate everything.
	 * Algorithm-record nonce_fixed values carry over - either lazily
	 * regenerated by acs_derive_nonce_state on first use (DIFF_FIXED with
	 * all-zero state) or re-derived from the new parent (EVEN_ODD).
	 */
	acs_crypto_destroy_exchange_keys(acs_conn);
	acs_crypto_destroy_connection_record_keys(acs_conn);

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
	int err = check_state(acs_conn, ACS_KEX_AWAIT_PUBKEY);

	if (err) {
		return err;
	}

	err = acs_crypto_compute_shared_secret(acs_conn);
	if (err) {
		return err;
	}

	if (net_buf_simple_tailroom(rsp_buf) < sizeof(acs_conn->kex->server_pubkey)) {
		LOG_ERR("Not enough tailroom for server_pubkey in response buffer: need %zu, have "
			"%zu",
			sizeof(acs_conn->kex->server_pubkey), net_buf_simple_tailroom(rsp_buf));
		return -ENOMEM;
	}

	net_buf_simple_add_mem(rsp_buf, &acs_conn->kex->server_pubkey,
			       sizeof(acs_conn->kex->server_pubkey));

	if (sys_le16_to_cpu(acs_conn->kex->start_kex.key_id) == ACS_KEY_ID_ECDH) {
		acs_conn->kex->state = ACS_KEX_AWAIT_KDF;
	} else {
		acs_conn->kex->state = ACS_KEX_AWAIT_CONFIRM_CODE;
	}
	LOG_INF("ECDH Public Keys Exchanged");
	return 0;
}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
int acs_key_exchange_ecdh_kdf(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf)
{
	uint8_t salt_be[CONFIG_BT_ACS_KDF_SALT_MAX_SIZE];
	uint8_t info_be[CONFIG_BT_ACS_KDF_INFO_MAX_SIZE];
	int err = check_state(acs_conn, ACS_KEX_AWAIT_KDF);

	if (err) {
		return err;
	}

	{
		psa_status_t psa_ret;

		psa_ret = psa_generate_random(acs_conn->kex->kdf.salt,
					      CONFIG_BT_ACS_KDF_SALT_MAX_SIZE);
		if (psa_ret != PSA_SUCCESS) {
			LOG_ERR("psa_generate_random failed for KDF salt: %d", psa_ret);
			return -EIO;
		}
		acs_conn->kex->kdf.salt_size = CONFIG_BT_ACS_KDF_SALT_MAX_SIZE;

#if CONFIG_BT_ACS_KDF_INFO_MAX_SIZE > 0
		memset(acs_conn->kex->kdf.info, 0, CONFIG_BT_ACS_KDF_INFO_MAX_SIZE);
		memcpy(acs_conn->kex->kdf.info, "ACS", 3);
		acs_conn->kex->kdf.info_size = 3;
#else
		acs_conn->kex->kdf.info_size = 0;
#endif
	}

	sys_memcpy_swap(salt_be, acs_conn->kex->kdf.salt, acs_conn->kex->kdf.salt_size);
	sys_memcpy_swap(info_be, acs_conn->kex->kdf.info, acs_conn->kex->kdf.info_size);

	{
		psa_key_derivation_operation_t op = PSA_KEY_DERIVATION_OPERATION_INIT;
		psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
		psa_key_id_t ecdhkey_id = 0;
		psa_status_t status;

		status = psa_key_derivation_setup(&op, ACS_PSA_HKDF_ALG);
		if (status != PSA_SUCCESS) {
			LOG_ERR("HKDF setup failed: %d", status);
			return -EIO;
		}

		if (acs_conn->kex->kdf.salt_size > 0U) {
			status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_SALT,
								salt_be,
								acs_conn->kex->kdf.salt_size);
			if (status != PSA_SUCCESS) {
				LOG_ERR("HKDF salt input failed: %d", status);
				psa_key_derivation_abort(&op);
				return -EIO;
			}
		}

		status = psa_key_derivation_input_key(&op, PSA_KEY_DERIVATION_INPUT_SECRET,
						      acs_conn->kex->derived_key_id);
		if (status != PSA_SUCCESS) {
			LOG_ERR("HKDF secret input_key failed: %d", status);
			psa_key_derivation_abort(&op);
			return -EIO;
		}

		status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_INFO, info_be,
							acs_conn->kex->kdf.info_size);
		if (status != PSA_SUCCESS) {
			LOG_ERR("HKDF info input failed: %d", status);
			psa_key_derivation_abort(&op);
			return -EIO;
		}

		psa_set_key_type(&attrs, PSA_KEY_TYPE_AES);
		psa_set_key_bits(&attrs, CONFIG_BT_ACS_SESSION_KEY_SIZE * BITS_PER_BYTE);
		psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT |
							PSA_KEY_USAGE_EXPORT);
		psa_set_key_algorithm(&attrs, PSA_ALG_GCM);

		status = psa_key_derivation_output_key(&attrs, &op, &ecdhkey_id);
		psa_key_derivation_abort(&op);

		if (status != PSA_SUCCESS) {
			LOG_ERR("HKDF output_key failed: %d", status);
			return -EIO;
		}

		acs_psa_destroy_key(&acs_conn->kex->derived_key_id);
		acs_conn->kex->derived_key_id = ecdhkey_id;
	}

	acs_conn->kex->key_mat_len = CONFIG_BT_ACS_SESSION_KEY_SIZE;
	acs_conn->kex->kdf_applied = true;

	err = acs_kdf_serialize_response(acs_conn->kex, rsp_buf);
	if (err) {
		return err;
	}

	acs_conn->kex->state = ACS_KEX_AWAIT_CONFIRM_CODE;
	LOG_INF("KDF Complete, ECDHKey Derived");
	return 0;
}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

int acs_key_exchange_ecdh_confirm_code(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf)
{
	uint8_t key_mat[CONFIG_BT_ACS_SESSION_KEY_SIZE];
	size_t key_mat_len;
	int err;
	uint8_t server_confirm[ACS_CONFIRM_VALUE_SIZE];
	uint8_t server_confirm_le[ACS_CONFIRM_VALUE_SIZE];

	err = check_state(acs_conn, ACS_KEX_AWAIT_CONFIRM_CODE);

	if (err) {
		return err;
	}

	sys_rand_get(acs_conn->kex->server_random, sizeof(acs_conn->kex->server_random));

	{
		psa_status_t status;

		status = psa_export_key(acs_conn->kex->derived_key_id, key_mat, sizeof(key_mat),
					&key_mat_len);
		if (status != PSA_SUCCESS) {
			LOG_ERR("Failed to export derived key for confirm: %d", status);
			return -EIO;
		}
	}

	err = acs_ecdh_confirm_code_compute(acs_conn->kex->server_pubkey.x,
#if CONFIG_BT_ACS_ECDH_HAS_Y
					    acs_conn->kex->server_pubkey.y,
#else
					    NULL,
#endif
					    acs_conn->kex->client_pubkey.x,
#if CONFIG_BT_ACS_ECDH_HAS_Y
					    acs_conn->kex->client_pubkey.y,
#else
					    NULL,
#endif
					    CONFIG_BT_ACS_ECDH_COORD_SIZE, key_mat, key_mat_len,
					    acs_conn->kex->auth_value, acs_conn->kex->server_random,
					    server_confirm);

	mbedtls_platform_zeroize(key_mat, sizeof(key_mat));

	if (err) {
		LOG_ERR("confirm code compute failed: %d", err);
		return -EIO;
	}

	/* Response: Key_ID(2 LE) + ACServerConfirmationCode(32, LSO first) */
	if (net_buf_simple_tailroom(rsp_buf) < sizeof(uint16_t) + ACS_CONFIRM_VALUE_SIZE) {
		return -ENOMEM;
	}

	net_buf_simple_add_le16(rsp_buf, sys_le16_to_cpu(acs_conn->kex->start_kex.key_id));

	/* Confirmation code is BE; reverse to LE for wire */
	sys_memcpy_swap(server_confirm_le, server_confirm, ACS_CONFIRM_VALUE_SIZE);
	net_buf_simple_add_mem(rsp_buf, server_confirm_le, ACS_CONFIRM_VALUE_SIZE);
	acs_conn->kex->state = ACS_KEX_AWAIT_CONFIRM_RAND;
	LOG_INF("Server Confirmation Code Sent");
	return 0;
}

int acs_key_exchange_ecdh_confirm_rand(struct bt_acs_conn *acs_conn,
				       const uint8_t client_random[ACS_CONFIRM_VALUE_SIZE],
				       struct net_buf_simple *rsp_buf)
{
	uint8_t key_mat[CONFIG_BT_ACS_SESSION_KEY_SIZE];
	size_t key_mat_len;
	uint8_t computed[ACS_CONFIRM_VALUE_SIZE];
	uint8_t client_random_be[ACS_CONFIRM_VALUE_SIZE];
	uint8_t client_confirm_be[ACS_CONFIRM_VALUE_SIZE];
	uint8_t server_random_le[sizeof(acs_conn->kex->server_random)];
	uint8_t diff = 0;
	int err = check_state(acs_conn, ACS_KEX_AWAIT_CONFIRM_RAND);

	if (err) {
		return err;
	}

	if (memcmp(client_random, acs_conn->kex->server_random, ACS_CONFIRM_VALUE_SIZE) == 0) {
		LOG_WRN("client random matches server random");
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

	sys_memcpy_swap(client_random_be, client_random, ACS_CONFIRM_VALUE_SIZE);

	err = acs_ecdh_confirm_code_compute(acs_conn->kex->server_pubkey.x,
#if CONFIG_BT_ACS_ECDH_HAS_Y
					    acs_conn->kex->server_pubkey.y,
#else
					    NULL,
#endif
					    acs_conn->kex->client_pubkey.x,
#if CONFIG_BT_ACS_ECDH_HAS_Y
					    acs_conn->kex->client_pubkey.y,
#else
					    NULL,
#endif
					    CONFIG_BT_ACS_ECDH_COORD_SIZE, key_mat, key_mat_len,
					    acs_conn->kex->auth_value, client_random_be, computed);

	mbedtls_platform_zeroize(key_mat, sizeof(key_mat));

	if (err) {
		LOG_ERR("confirm code compute failed during verify: %d", err);
		return -EIO;
	}

	/* client_confirm is wire LE; reverse to BE before constant-time compare */
	sys_memcpy_swap(client_confirm_be, acs_conn->kex->client_confirm, ACS_CONFIRM_VALUE_SIZE);

	for (int i = 0; i < ACS_CONFIRM_VALUE_SIZE; i++) {
		diff |= client_confirm_be[i] ^ computed[i];
	}

	if (diff) {
		LOG_ERR("Client confirmation code mismatch - authentication failed");
		return -EACCES;
	}

	/* Response: Key_ID(2 LE) + ACServerConfirmationRandomNumber(32, LSO first) */
	if (net_buf_simple_tailroom(rsp_buf) <
	    sizeof(uint16_t) + sizeof(acs_conn->kex->server_random)) {
		return -ENOMEM;
	}

	net_buf_simple_add_le16(rsp_buf, sys_le16_to_cpu(acs_conn->kex->start_kex.key_id));

	/* server_random is BE; reverse to LE for wire */
	sys_memcpy_swap(server_random_le, acs_conn->kex->server_random,
			sizeof(acs_conn->kex->server_random));
	net_buf_simple_add_mem(rsp_buf, server_random_le, sizeof(acs_conn->kex->server_random));

	LOG_INF("Client Confirmed - Server Random Sent");
	return 0;
}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
int acs_key_exchange_kdf(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf)
{
	int err = check_state(acs_conn, ACS_KEX_AWAIT_KDF);

	if (err) {
		return err;
	}

	{
		psa_status_t psa_ret;

		psa_ret = psa_generate_random(acs_conn->kex->kdf.salt,
					      CONFIG_BT_ACS_KDF_SALT_MAX_SIZE);
		if (psa_ret != PSA_SUCCESS) {
			LOG_ERR("psa_generate_random failed for KDF salt: %d", psa_ret);
			return -EIO;
		}
		acs_conn->kex->kdf.salt_size = CONFIG_BT_ACS_KDF_SALT_MAX_SIZE;

#if CONFIG_BT_ACS_KDF_INFO_MAX_SIZE > 0
		memset(acs_conn->kex->kdf.info, 0, CONFIG_BT_ACS_KDF_INFO_MAX_SIZE);
		memcpy(acs_conn->kex->kdf.info, "ACS", 3);
		acs_conn->kex->kdf.info_size = 3;
#else
		acs_conn->kex->kdf.info_size = 0;
#endif
	}

	/* Derive child key: HKDF(salt, ikm=parent_key, info) → child replaces parent_key */
	err = acs_crypto_derive_kdf_child_key(acs_conn);
	if (err) {
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
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF */
