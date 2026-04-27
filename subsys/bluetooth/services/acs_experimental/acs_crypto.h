/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_CRYPTO_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_CRYPTO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum fixed nonce contribution kept in connection state.
 *
 * The exact nonce layout depends on the negotiated data protection algorithm.
 * The scaffold keeps both client and server fixed parts in a shared-size
 * buffer so later key exchange and Data In/Out code can rely on one stable
 * state shape.
 */
#define ACS_CRYPTO_NONCE_FIXED_MAX_SIZE 8U

/**
 * @brief Maximum AEAD authentication tag size the scaffold reserves for.
 */
#define ACS_CRYPTO_AUTH_TAG_MAX_SIZE 16U

/**
 * @brief Supported high-level crypto modes exposed to the rest of ACS.
 *
 * This enum is intentionally coarse-grained. It gives the runtime and routing
 * layers a stable way to ask "what kind of protection is currently active?"
 * without baking PSA-specific details into unrelated modules.
 */
enum acs_crypto_mode {
	ACS_CRYPTO_MODE_NONE = 0,
	ACS_CRYPTO_MODE_AUTH_ONLY,
	ACS_CRYPTO_MODE_INTEGRITY_ONLY,
	ACS_CRYPTO_MODE_AEAD,
};

/**
 * @brief Per-connection crypto and session state.
 *
 * The crypto module owns this state. The rest of ACS treats it as a compact
 * summary of whether security exists, which key material is active, and which
 * nonce counters should be used for the next protected transfer.
 */
struct acs_crypto_ctx {
	uint8_t session_key[CONFIG_BT_ACS_SESSION_KEY_SIZE];
	uint8_t client_nonce_fixed[ACS_CRYPTO_NONCE_FIXED_MAX_SIZE];
	uint8_t server_nonce_fixed[ACS_CRYPTO_NONCE_FIXED_MAX_SIZE];
	size_t client_nonce_fixed_len;
	size_t server_nonce_fixed_len;
	uint64_t tx_nonce_counter;
	uint64_t rx_nonce_counter;
	uint16_t active_isc_id;
	uint16_t active_key_id;
	enum acs_crypto_mode mode;
	bool session_key_valid;
};

/**
 * @brief Initialize a crypto context to the disconnected state.
 */
void acs_crypto_init(struct acs_crypto_ctx *ctx);

/**
 * @brief Clear all active crypto material and counters.
 */
void acs_crypto_reset(struct acs_crypto_ctx *ctx);

/**
 * @brief Return true when a usable session key is installed.
 */
bool acs_crypto_has_session(const struct acs_crypto_ctx *ctx);

/**
 * @brief Install session key material and reset per-session counters.
 *
 * @param ctx     Crypto context to update.
 * @param mode    Active protection mode for the installed key.
 * @param isc_id  ISC identifier this session was derived for.
 * @param key_id  Key identifier selected by key exchange or policy.
 * @param key     Raw session key bytes.
 * @param key_len Number of valid bytes in @p key.
 *
 * @retval 0 Session installed.
 * @retval -EINVAL Invalid parameters.
 */
int acs_crypto_session_install(struct acs_crypto_ctx *ctx, enum acs_crypto_mode mode,
			       uint16_t isc_id, uint16_t key_id, const uint8_t *key,
			       size_t key_len);

/**
 * @brief Remove the active session key and reset counters.
 */
void acs_crypto_session_clear(struct acs_crypto_ctx *ctx);

/**
 * @brief Set the client-provided fixed nonce contribution.
 */
int acs_crypto_set_client_nonce_fixed(struct acs_crypto_ctx *ctx, const uint8_t *nonce,
				      size_t len);

/**
 * @brief Set the server-provided fixed nonce contribution.
 */
int acs_crypto_set_server_nonce_fixed(struct acs_crypto_ctx *ctx, const uint8_t *nonce,
				      size_t len);

/**
 * @brief Build a nonce for the next protected transfer.
 *
 * The nonce format is intentionally abstract at this layer. For now the
 * scaffold concatenates a fixed part with the counter in little-endian order.
 * Later algorithm-specific layouts can remain behind this API.
 *
 * @param ctx        Crypto context holding nonce state.
 * @param outbound   True for TX nonce generation, false for RX.
 * @param out_nonce  Destination buffer.
 * @param out_len    In: buffer size, Out: nonce length written.
 *
 * @retval 0 Nonce written.
 * @retval -EINVAL Invalid parameters.
 * @retval -EACCES No active session.
 * @retval -ENOBUFS Output buffer too small.
 */
int acs_crypto_build_nonce(const struct acs_crypto_ctx *ctx, bool outbound, uint8_t *out_nonce,
			   size_t *out_len);

/**
 * @brief Advance and return the next outbound nonce counter.
 *
 * @retval 0 Counter written.
 * @retval -EINVAL Invalid parameters.
 * @retval -EACCES No active session.
 */
int acs_crypto_next_tx_counter(struct acs_crypto_ctx *ctx, uint64_t *counter);

/**
 * @brief Accept a received nonce counter and update replay state.
 *
 * The current rule is monotonic: counters must be greater than or equal to the
 * next expected RX counter. This keeps replay protection semantics explicit
 * even before the full decrypt pipeline is implemented.
 *
 * @retval 0 Counter accepted.
 * @retval -EINVAL Invalid parameters.
 * @retval -EACCES No active session.
 * @retval -EALREADY Counter is stale or replayed.
 */
int acs_crypto_accept_rx_counter(struct acs_crypto_ctx *ctx, uint64_t counter);

/**
 * @brief Encrypt plaintext for protected ACS transport.
 *
 * This scaffold-level API intentionally exists before the full crypto backend
 * is wired. It gives Data Out and future procedure code a stable call shape.
 *
 * @retval -ENOTSUP The crypto backend is not implemented yet.
 */
size_t acs_crypto_nonce_variable_size(const struct acs_crypto_ctx *ctx);

size_t acs_crypto_auth_tag_size(const struct acs_crypto_ctx *ctx);

int acs_crypto_encrypt(struct acs_crypto_ctx *ctx, const uint8_t *plaintext, size_t plain_len,
		       uint8_t *nonce_var, size_t *nonce_var_len, uint8_t *tag,
		       size_t *tag_len, uint8_t *ciphertext, size_t *cipher_len);

/**
 * @brief Decrypt ciphertext for protected ACS transport.
 *
 * @retval -ENOTSUP The crypto backend is not implemented yet.
 */
int acs_crypto_decrypt(struct acs_crypto_ctx *ctx, const uint8_t *nonce_var,
		       size_t nonce_var_len, const uint8_t *tag, size_t tag_len,
		       const uint8_t *ciphertext, size_t cipher_len, uint8_t *plaintext,
		       size_t *plain_len);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_CRYPTO_H_ */
