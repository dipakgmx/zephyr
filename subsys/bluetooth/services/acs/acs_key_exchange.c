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

#include "acs_key_exchange.h"
#include "acs_internal.h"
#include "acs_key_desc.h"

#include <psa/crypto.h>

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static int check_state(struct bt_acs_conn const *acs_conn, enum bt_acs_key_exchange_state expected)
{
	if (acs_conn->key_state != expected) {
		LOG_WRN("Invalid state transition (current: %d, expected: %d)", acs_conn->key_state,
			expected);
		return -EAGAIN;
	}
	return 0;
}

/* PSA key bit sizes for supported ECDH curves */
#define ACS_PSA_KEY_BITS_P521   521U
#define ACS_PSA_KEY_BITS_P384   384U
#define ACS_PSA_KEY_BITS_C25519 255U
#define ACS_PSA_KEY_BITS_P256   256U

/* Uncompressed point prefix for NIST curves ([0x04][X_BE][Y_BE]) */
#define ACS_ECDH_UNCOMPRESSED_POINT 0x04U

/* Compile-time curve selection helpers */
static psa_ecc_family_t acs_get_psa_ecc_family(void)
{
	if (IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_CURVE25519)) {
		return PSA_ECC_FAMILY_MONTGOMERY;
	}
	return PSA_ECC_FAMILY_SECP_R1;
}

static size_t acs_get_psa_key_bits(void)
{
	if (IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_P521)) {
		return ACS_PSA_KEY_BITS_P521;
	} else if (IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_P384)) {
		return ACS_PSA_KEY_BITS_P384;
	} else if (IS_ENABLED(CONFIG_BT_ACS_ECDH_CURVE_CURVE25519)) {
		return ACS_PSA_KEY_BITS_C25519;
	}
	return ACS_PSA_KEY_BITS_P256;
}

static psa_algorithm_t acs_get_psa_hkdf_alg(void)
{
	if (IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA384) ||
	    IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA384_WITH_INFO)) {
		return PSA_ALG_HKDF(PSA_ALG_SHA_384);
	} else if (IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA512) ||
		   IS_ENABLED(CONFIG_BT_ACS_KDF_HKDF_SHA512_WITH_INFO)) {
		return PSA_ALG_HKDF(PSA_ALG_SHA_512);
	}
	return PSA_ALG_HKDF(PSA_ALG_SHA_256);
}

static int acs_hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *msg, size_t msg_len,
			   uint8_t out[ACS_HMAC_SHA256_SIZE])
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
				 ACS_HMAC_SHA256_SIZE, &out_len);
	destroy_status = psa_destroy_key(hmac_key);
	if ((status != PSA_SUCCESS) || (destroy_status != PSA_SUCCESS)) {
		LOG_ERR("Failed to compute HMAC-SHA-256: status=%d, destroy=%d, msg_len=%zu",
			status, destroy_status, msg_len);
		return -EIO;
	}
	return 0;
}

static int acs_hkdf(psa_algorithm_t hkdf_alg, const uint8_t *salt, size_t salt_len,
		    const uint8_t *ikm, size_t ikm_len, const uint8_t *info, size_t info_len,
		    uint8_t *out, size_t out_len)
{
	psa_key_derivation_operation_t op = PSA_KEY_DERIVATION_OPERATION_INIT;
	psa_status_t status;

	status = psa_key_derivation_setup(&op, hkdf_alg);
	if (status != PSA_SUCCESS) {
		LOG_ERR("HKDF setup failed: status=%d, alg=0x%08x", status, hkdf_alg);
		goto cleanup;
	}

	if (salt_len > 0U) {
		status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_SALT, salt,
							salt_len);
		if (status != PSA_SUCCESS) {
			LOG_ERR("HKDF salt input failed: status=%d, salt_len=%zu", status,
				salt_len);
			goto cleanup;
		}
	}

	status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_SECRET, ikm, ikm_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("HKDF IKM input failed: status=%d, ikm_len=%zu", status, ikm_len);
		goto cleanup;
	}

	status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_INFO, info, info_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("HKDF info input failed: status=%d, info_len=%zu", status, info_len);
		goto cleanup;
	}

	status = psa_key_derivation_output_bytes(&op, out, out_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("HKDF output failed: status=%d, out_len=%zu", status, out_len);
	}

cleanup:
	status = psa_key_derivation_abort(&op);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Failed to abort key derivation operation: %d", status);
		return -EIO;
	}
	return 0;
}

/* §4.4.3.17.1.2: Salt=HMAC(Zero,PKs||PKc), ConfKey=HMAC(Salt,ECDHKey||Auth),
 * Code=HMAC(ConfKey,Random). Coords supplied in LE (wire format); reversed to BE internally. Pass
 * NULL for Y on Curve25519.
 */
static int acs_ecdh_confirm_code_compute(const uint8_t *server_x_le, const uint8_t *server_y_le,
					 const uint8_t *client_x_le, const uint8_t *client_y_le,
					 size_t coord_size, const uint8_t *ecdh_key,
					 size_t ecdh_key_len,
					 const uint8_t auth_value[ACS_HMAC_SHA256_SIZE],
					 const uint8_t random[ACS_HMAC_SHA256_SIZE],
					 uint8_t confirm_out[ACS_HMAC_SHA256_SIZE])
{
	int ret;
	static const uint8_t zero_key[ACS_HMAC_SHA256_SIZE];
	uint8_t salt[ACS_HMAC_SHA256_SIZE];
	uint8_t pubkey_concat[CONFIG_BT_ACS_ECDH_COORD_SIZE * 4];
	size_t pubkey_concat_len = 0;
	uint8_t confirmation_key[ACS_HMAC_SHA256_SIZE];
	uint8_t ecdh_auth[CONFIG_BT_ACS_ECDH_COORD_SIZE + ACS_HMAC_SHA256_SIZE];

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
		return ret;
	}

	/* Step 2: ConfirmationKey = HMAC(Salt, ECDHKey || AuthValue) */
	memcpy(&ecdh_auth[0], ecdh_key, ecdh_key_len);
	memcpy(&ecdh_auth[ecdh_key_len], auth_value, ACS_HMAC_SHA256_SIZE);

	ret = acs_hmac_sha256(salt, sizeof(salt), ecdh_auth, ecdh_key_len + ACS_HMAC_SHA256_SIZE,
			      confirmation_key);
	if (ret != 0) {
		LOG_ERR("HMAC-SHA-256 failed in deriving confirmation key: %d", ret);
		return ret;
	}

	/* Step 3: ConfirmationCode = HMAC(ConfirmationKey, RandomNumber) */
	ret = acs_hmac_sha256(confirmation_key, sizeof(confirmation_key), random,
			      ACS_HMAC_SHA256_SIZE, confirm_out);
	if (ret != 0) {
		LOG_ERR("HMAC-SHA-256 failed in deriving confirmation code: %d", ret);
		return ret;
	}

	return 0;
}

static int bt_acs_crypto_generate_keypair(struct bt_acs_conn *acs_conn)
{
	struct acs_ecdh_pubkey *pk = &acs_conn->kex->server_pubkey;
	psa_key_attributes_t attrs = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;
	uint8_t pub[1U + 2U * CONFIG_BT_ACS_ECDH_COORD_SIZE];
	size_t pub_len;

	psa_set_key_type(&attrs, PSA_KEY_TYPE_ECC_KEY_PAIR(acs_get_psa_ecc_family()));
	psa_set_key_bits(&attrs, acs_get_psa_key_bits());
	psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_DERIVE);
	psa_set_key_algorithm(&attrs, PSA_ALG_ECDH);

	status = psa_generate_key(&attrs, &acs_conn->kex->ecdh_key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_generate_key failed: %d", status);
		return -EIO;
	}

	/* Export public key to populate server_pubkey coords */
	status = psa_export_public_key(acs_conn->kex->ecdh_key_id, pub, sizeof(pub), &pub_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_export_public_key failed: %d", status);
		psa_destroy_key(acs_conn->kex->ecdh_key_id);
		acs_conn->kex->ecdh_key_id = 0;
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
		psa_destroy_key(acs_conn->kex->ecdh_key_id);
		acs_conn->kex->ecdh_key_id = 0;
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
		psa_destroy_key(acs_conn->kex->ecdh_key_id);
		acs_conn->kex->ecdh_key_id = 0;
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

static int bt_acs_crypto_compute_shared_secret(struct bt_acs_conn *acs_conn)
{
	const struct acs_ecdh_pubkey *cpk = &acs_conn->kex->client_pubkey;
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
		psa_destroy_key(acs_conn->kex->ecdh_key_id);
		acs_conn->kex->ecdh_key_id = 0;
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

	status = psa_raw_key_agreement(PSA_ALG_ECDH, acs_conn->kex->ecdh_key_id, psa_pubkey,
				       psa_pubkey_len, acs_conn->kex->shared_secret,
				       sizeof(acs_conn->kex->shared_secret), &olen);

	psa_destroy_key(acs_conn->kex->ecdh_key_id);
	acs_conn->kex->ecdh_key_id = 0;

	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_raw_key_agreement failed: %d", status);
		if (status == PSA_ERROR_INVALID_ARGUMENT) {
			return -EBADMSG;
		}
		return -EIO;
	}

	acs_conn->kex->key_mat_len = (uint16_t)olen;

	return 0;
}

int acs_crypto_derive_session_key(struct bt_acs_conn *acs_conn)
{
	static const uint8_t info[] = "ACS";
	uint8_t salt[2 * ACS_HMAC_SHA256_SIZE];
	uint8_t client_random_be[ACS_HMAC_SHA256_SIZE];
	int ret;

	/* salt = ServerRandom_BE || ClientRandom_BE */
	sys_memcpy_swap(client_random_be, acs_conn->kex->client_random, ACS_HMAC_SHA256_SIZE);
	memcpy(salt, acs_conn->kex->server_random, ACS_HMAC_SHA256_SIZE);
	memcpy(salt + ACS_HMAC_SHA256_SIZE, client_random_be, ACS_HMAC_SHA256_SIZE);

	ret = acs_hkdf(acs_get_psa_hkdf_alg(), salt, sizeof(salt), acs_conn->kex->shared_secret,
		       acs_conn->kex->key_mat_len, info, sizeof(info) - 1,
		       acs_conn->crypto.session_key, CONFIG_BT_ACS_SESSION_KEY_SIZE);

	if (ret != 0) {
		LOG_ERR("HKDF session key derivation failed: %d", ret);
		return ret;
	}

	acs_conn->crypto.tx_nonce_counter = 0;
#if defined(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD)
	acs_conn->crypto.rx_nonce_counter = 1; /* client uses odd counters: 1, 3, 5 ... */
#else
	acs_conn->crypto.rx_nonce_counter = 0;
#endif

	return acs_crypto_import_session_key(acs_conn);
}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
/* §4.4.3.17.2.1: Derive child key from session key via HKDF; replaces session_key. */
static int bt_acs_crypto_derive_kdf_child_key(struct bt_acs_conn *acs_conn)
{
	uint8_t child_key[CONFIG_BT_ACS_SESSION_KEY_SIZE];
	uint8_t salt_be[CONFIG_BT_ACS_KDF_SALT_MAX_SIZE];
	uint8_t info_be[CONFIG_BT_ACS_KDF_INFO_MAX_SIZE];
	int ret;

	/* Salt and info are stored in wire order (LSO) but HKDF requires big-endian input. */
	sys_memcpy_swap(salt_be, acs_conn->kex->kdf.salt, acs_conn->kex->kdf.salt_size);
	sys_memcpy_swap(info_be, acs_conn->kex->kdf.info, acs_conn->kex->kdf.info_size);

	ret = acs_hkdf(acs_get_psa_hkdf_alg(), salt_be, acs_conn->kex->kdf.salt_size,
		       acs_conn->crypto.session_key, CONFIG_BT_ACS_SESSION_KEY_SIZE, info_be,
		       acs_conn->kex->kdf.info_size, child_key, CONFIG_BT_ACS_SESSION_KEY_SIZE);

	if (ret != 0) {
		LOG_ERR("KDF child key derivation failed: %d", ret);
		return ret;
	}

	memcpy(acs_conn->crypto.session_key, child_key, CONFIG_BT_ACS_SESSION_KEY_SIZE);
	acs_conn->crypto.tx_nonce_counter = 0;
#if defined(CONFIG_BT_ACS_CCM_NONCE_SEQ_EVEN_ODD)
	acs_conn->crypto.rx_nonce_counter = 1; /* client uses odd counters: 1, 3, 5 ... */
#else
	acs_conn->crypto.rx_nonce_counter = 0;
#endif

	/* Destroy the parent key handle before importing the child key,
	 * otherwise the old PSA key slot is leaked.
	 */
	acs_crypto_destroy_session_key(acs_conn);
	return acs_crypto_import_session_key(acs_conn);
}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF */

int acs_key_exchange_ecdh_start(struct bt_acs_conn *acs_conn, uint16_t key_id)
{
	if (key_id != ACS_KEY_ID_ECDH) {
		LOG_ERR("key_id 0x%04x does not match expected ECDH key ID 0x%04x", key_id,
			ACS_KEY_ID_ECDH);
		return -EALREADY;
	}

#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	/* Spec §4.4.3.13: SET_CLIENT_NONCE_FIXED must precede START_KEY_EXCHANGE
	 * for SEQ_DIFF_FIXED nonce types. A non-zero value is present when:
	 *   - The client called SET_CLIENT_NONCE_FIXED on this connection, or
	 *   - A previous session was restored from NVS (nonces persisted).
	 */
	{
		bool cnf_zero = true;

		for (int i = 0; i < CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE; i++) {
			if (acs_conn->crypto.client_nonce_fixed[i] != 0) {
				cnf_zero = false;
				break;
			}
		}
		if (cnf_zero) {
			LOG_WRN("client nonce fixed not set");
			return -EALREADY;
		}
	}
#endif /* CONFIG_BT_ACS_HAS_NONCE_FIXED */

	/* Destroy any ECDH private key left from a prior aborted exchange. */
	if (acs_conn->kex && acs_conn->kex->ecdh_key_id != 0) {
		psa_destroy_key(acs_conn->kex->ecdh_key_id);
		acs_conn->kex->ecdh_key_id = 0;
	}

	/* Reset crypto session; preserve nonce fixed parts across exchanges (§4.4.3.18) */
	acs_crypto_destroy_session_key(acs_conn);
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	uint8_t saved_snf[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE];
	uint8_t saved_cnf[CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE];

	memcpy(saved_snf, acs_conn->crypto.server_nonce_fixed, CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
	memcpy(saved_cnf, acs_conn->crypto.client_nonce_fixed, CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
	memset(&acs_conn->crypto, 0, sizeof(acs_conn->crypto));
	memcpy(acs_conn->crypto.server_nonce_fixed, saved_snf, CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
	memcpy(acs_conn->crypto.client_nonce_fixed, saved_cnf, CONFIG_BT_ACS_NONCE_FIXED_BUF_SIZE);
#else
	memset(&acs_conn->crypto, 0, sizeof(acs_conn->crypto));
#endif /* BT_ACS_HAS_NONCE_FIXED */

	if (acs_conn->kex) {
		memset(&acs_conn->kex->server_pubkey, 0, sizeof(acs_conn->kex->server_pubkey));
		memset(&acs_conn->kex->client_pubkey, 0, sizeof(acs_conn->kex->client_pubkey));
		memset(acs_conn->kex->server_random, 0, sizeof(acs_conn->kex->server_random));
		memset(acs_conn->kex->client_random, 0, sizeof(acs_conn->kex->client_random));
		memset(acs_conn->kex->server_confirm, 0, sizeof(acs_conn->kex->server_confirm));
		memset(acs_conn->kex->client_confirm, 0, sizeof(acs_conn->kex->client_confirm));
	}

	acs_conn->key_state = BT_ACS_KEY_EXCHANGE_STARTED;

	LOG_INF("ECDH Procedure Started");
	return 0;
}

int acs_key_exchange_ecdh_pubkey(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf)
{
	int err = check_state(acs_conn, BT_ACS_KEY_EXCHANGE_STARTED);

	if (err) {
		LOG_WRN("Pubkey exchange in wrong state: %d", acs_conn->key_state);
		return err;
	}

	err = bt_acs_crypto_generate_keypair(acs_conn);
	if (err) {
		return -EIO;
	}

	err = bt_acs_crypto_compute_shared_secret(acs_conn);
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

	acs_conn->key_state = BT_ACS_KEY_EXCHANGE_PUBKEY_EXCHANGED;
	LOG_INF("ECDH Public Keys Exchanged");
	return 0;
}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
int acs_key_exchange_ecdh_kdf(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf)
{
	psa_status_t psa_ret;
	uint8_t salt_be[CONFIG_BT_ACS_KDF_SALT_MAX_SIZE];
	uint8_t info_be[CONFIG_BT_ACS_KDF_INFO_MAX_SIZE];
	uint8_t needed;
	int err = check_state(acs_conn, BT_ACS_KEY_EXCHANGE_PUBKEY_EXCHANGED);

	if (err) {
		return err;
	}

	psa_ret = psa_generate_random(acs_conn->kex->kdf.salt, CONFIG_BT_ACS_KDF_SALT_MAX_SIZE);
	if (psa_ret != PSA_SUCCESS) {
		LOG_ERR("psa_generate_random failed for KDF salt: %d", psa_ret);
		return -EIO;
	}
	acs_conn->kex->kdf.salt_size = CONFIG_BT_ACS_KDF_SALT_MAX_SIZE;

#if CONFIG_BT_ACS_KDF_INFO_MAX_SIZE > 0
	const char *acs_info = "ACS";

	memset(acs_conn->kex->kdf.info, 0, CONFIG_BT_ACS_KDF_INFO_MAX_SIZE);
	memcpy(acs_conn->kex->kdf.info, acs_info, strlen(acs_info));
	acs_conn->kex->kdf.info_size = strlen(acs_info);
#else
	acs_conn->kex->kdf.info_size = 0;
#endif

	/* HKDF: IKM=shared_secret → ecdh_key (same union buffer, safe: PSA
	 * reads IKM fully during extract before writing during expand).
	 * Salt and info reversed to BE per spec. */
	sys_memcpy_swap(salt_be, acs_conn->kex->kdf.salt, acs_conn->kex->kdf.salt_size);
	sys_memcpy_swap(info_be, acs_conn->kex->kdf.info, acs_conn->kex->kdf.info_size);

	err = acs_hkdf(acs_get_psa_hkdf_alg(), salt_be, acs_conn->kex->kdf.salt_size,
		       acs_conn->kex->shared_secret, acs_conn->kex->key_mat_len, info_be,
		       acs_conn->kex->kdf.info_size, acs_conn->kex->ecdh_key,
		       CONFIG_BT_ACS_SESSION_KEY_SIZE);
	if (err != 0) {
		LOG_ERR("HKDF ECDHKey derivation failed: %d", err);
		return err;
	}

	acs_conn->kex->key_mat_len = CONFIG_BT_ACS_SESSION_KEY_SIZE;
	acs_conn->kex->kdf_applied = true;

	/* Response: Key_ID(2) + KDF_Salt_Size(1) + KDF_Salt(N) + KDF_Info_Size(1) + KDF_Info(M) */
	needed = ACS_KDF_RSP_FIXED_SIZE + acs_conn->kex->kdf.salt_size +
		 acs_conn->kex->kdf.info_size;

	if (net_buf_simple_tailroom(rsp_buf) < needed) {
		return -ENOMEM;
	}

	net_buf_simple_add_le16(rsp_buf, sys_le16_to_cpu(acs_conn->kex->start_kex.key_id));
	net_buf_simple_add_u8(rsp_buf, acs_conn->kex->kdf.salt_size);
	net_buf_simple_add_mem(rsp_buf, acs_conn->kex->kdf.salt, acs_conn->kex->kdf.salt_size);
	net_buf_simple_add_u8(rsp_buf, acs_conn->kex->kdf.info_size);
	net_buf_simple_add_mem(rsp_buf, acs_conn->kex->kdf.info, acs_conn->kex->kdf.info_size);

	acs_conn->key_state = BT_ACS_KEY_EXCHANGE_KDF_DONE;
	LOG_INF("KDF Complete, ECDHKey Derived");
	return 0;
}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_ECDH */

int acs_key_exchange_ecdh_confirm_code(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf)
{
	const uint8_t *key_mat;
	uint16_t key_mat_len;
	int err;
	uint8_t server_confirm_le[ACS_HMAC_SHA256_SIZE];

	/* Valid after pubkey exchange or KDF step */
	if (acs_conn->key_state != BT_ACS_KEY_EXCHANGE_PUBKEY_EXCHANGED &&
	    acs_conn->key_state != BT_ACS_KEY_EXCHANGE_KDF_DONE) {
		LOG_WRN("Confirm Code in wrong state: %d", acs_conn->key_state);
		return -EAGAIN;
	}

	/* Generate fresh server random; client_confirm verification deferred to confirm_rand */
	sys_rand_get(acs_conn->kex->server_random, sizeof(acs_conn->kex->server_random));

	/* Key material: KDF overwrites the union buffer in-place, so
	 * shared_secret/ecdh_key are the same storage; key_mat_len tracks
	 * whichever was written last. */
	key_mat = acs_conn->kex->shared_secret;
	key_mat_len = acs_conn->kex->key_mat_len;

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
					    acs_conn->kex->server_confirm);

	if (err) {
		LOG_ERR("confirm code compute failed: %d", err);
		return -EIO;
	}

	/* Response: Key_ID(2 LE) + ACServerConfirmationCode(32, LSO first) */
	if (net_buf_simple_tailroom(rsp_buf) < sizeof(uint16_t) + ACS_HMAC_SHA256_SIZE) {
		return -ENOMEM;
	}

	net_buf_simple_add_le16(rsp_buf, sys_le16_to_cpu(acs_conn->kex->start_kex.key_id));

	/* Confirmation code is BE; reverse to LE for wire */
	sys_memcpy_swap(server_confirm_le, acs_conn->kex->server_confirm, ACS_HMAC_SHA256_SIZE);
	net_buf_simple_add_mem(rsp_buf, server_confirm_le, ACS_HMAC_SHA256_SIZE);
	acs_conn->key_state = BT_ACS_KEY_EXCHANGE_CONFIRM_CODE;
	LOG_INF("Server Confirmation Code Sent");
	return 0;
}

int acs_key_exchange_ecdh_confirm_rand(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf)
{
	const uint8_t *key_mat;
	uint16_t key_mat_len;
	uint8_t computed[ACS_HMAC_SHA256_SIZE];
	uint8_t client_random_be[ACS_HMAC_SHA256_SIZE];
	uint8_t client_confirm_be[ACS_HMAC_SHA256_SIZE];
	uint8_t server_random_le[sizeof(acs_conn->kex->server_random)];
	uint8_t diff = 0;
	int err = check_state(acs_conn, BT_ACS_KEY_EXCHANGE_CONFIRM_CODE);

	if (err) {
		return err;
	}

	key_mat = acs_conn->kex->shared_secret;
	key_mat_len = acs_conn->kex->key_mat_len;

	/* client_random is wire LE; reverse to BE for HMAC */
	sys_memcpy_swap(client_random_be, acs_conn->kex->client_random, ACS_HMAC_SHA256_SIZE);

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

	if (err) {
		LOG_ERR("confirm code compute failed during verify: %d", err);
		acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
		return -EIO;
	}

	/* client_confirm is wire LE; reverse to BE before constant-time compare */
	sys_memcpy_swap(client_confirm_be, acs_conn->kex->client_confirm, ACS_HMAC_SHA256_SIZE);

	for (int i = 0; i < ACS_HMAC_SHA256_SIZE; i++) {
		diff |= client_confirm_be[i] ^ computed[i];
	}

	if (diff) {
		LOG_ERR("Client confirmation code mismatch - authentication failed");
		acs_conn->key_state = BT_ACS_KEY_EXCHANGE_IDLE;
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

	LOG_INF("Client Confirmed — Server Random Sent");
	return 0;
}

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_KDF)
int acs_key_exchange_kdf(struct bt_acs_conn *acs_conn, struct net_buf_simple *rsp_buf)
{
	uint8_t needed;
	int err = check_state(acs_conn, BT_ACS_KEY_EXCHANGE_STARTED);

	if (err) {
		LOG_WRN("KDF standalone exchange in wrong state: %d", acs_conn->key_state);
		return err;
	}

	sys_rand_get(acs_conn->kex->kdf.salt, CONFIG_BT_ACS_KDF_SALT_MAX_SIZE);
	acs_conn->kex->kdf.salt_size = CONFIG_BT_ACS_KDF_SALT_MAX_SIZE;

#if CONFIG_BT_ACS_KDF_INFO_MAX_SIZE > 0
	const char *acs_info = "ACS";

	memset(acs_conn->kex->kdf.info, 0, CONFIG_BT_ACS_KDF_INFO_MAX_SIZE);
	memcpy(acs_conn->kex->kdf.info, acs_info, strlen(acs_info));
	acs_conn->kex->kdf.info_size = strlen(acs_info);
#else
	acs_conn->kex->kdf.info_size = 0;
#endif

	/* Derive child session key: HKDF(salt, ikm=session_key, info) → new session_key */
	err = bt_acs_crypto_derive_kdf_child_key(acs_conn);
	if (err) {
		LOG_ERR("KDF child key derivation failed: %d", err);
		return -EIO;
	}

	/* Response: Key_ID(2) | KDF_Salt_Size(1) | KDF_Salt(N) | KDF_Info_Size(1) | KDF_Info(M) */
	needed = sizeof(uint16_t) + sizeof(uint8_t) + acs_conn->kex->kdf.salt_size +
		 sizeof(uint8_t) + acs_conn->kex->kdf.info_size;

	if (net_buf_simple_tailroom(rsp_buf) < needed) {
		LOG_WRN("KDF response buffer too small: need %u, have %u", needed,
			net_buf_simple_tailroom(rsp_buf));
		return -ENOMEM;
	}

	net_buf_simple_add_le16(rsp_buf, sys_le16_to_cpu(acs_conn->kex->start_kex.key_id));
	net_buf_simple_add_u8(rsp_buf, acs_conn->kex->kdf.salt_size);
	net_buf_simple_add_mem(rsp_buf, acs_conn->kex->kdf.salt, acs_conn->kex->kdf.salt_size);
	net_buf_simple_add_u8(rsp_buf, acs_conn->kex->kdf.info_size);
	net_buf_simple_add_mem(rsp_buf, acs_conn->kex->kdf.info, acs_conn->kex->kdf.info_size);

	LOG_INF("KDF child key derived success");
	return 0;
}
#endif /* CONFIG_BT_ACS_KEY_EXCHANGE_KDF */
