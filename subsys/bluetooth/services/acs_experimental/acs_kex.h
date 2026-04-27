/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_KEX_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_KEX_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <psa/crypto.h>

struct net_buf;

struct acs_conn_ctx;

enum acs_kex_state {
	ACS_KEX_IDLE = 0,
	ACS_KEX_STARTED,
	ACS_KEX_PUBKEY_EXCHANGED,
	ACS_KEX_KDF_DONE,
	ACS_KEX_COMPLETE,
};

#if CONFIG_BT_ACS_KDF_INFO_MAX_SIZE > 0
#define ACS_KEX_INFO_BUF_SIZE CONFIG_BT_ACS_KDF_INFO_MAX_SIZE
#else
#define ACS_KEX_INFO_BUF_SIZE 1
#endif

struct acs_kex_ctx {
	enum acs_kex_state state;
	uint16_t key_id;
	uint8_t confirmation_method;
	uint8_t confirmation_action;
	uint8_t auth_value[32];
	psa_key_id_t private_key_handle;
	uint8_t local_public_x[CONFIG_BT_ACS_ECDH_COORD_SIZE];
	uint8_t local_public_y[CONFIG_BT_ACS_ECDH_COORD_SIZE];
	uint8_t peer_public_x[CONFIG_BT_ACS_ECDH_COORD_SIZE];
	uint8_t peer_public_y[CONFIG_BT_ACS_ECDH_COORD_SIZE];
	size_t public_coord_len;
	size_t peer_public_coord_len;
	uint8_t shared_secret[CONFIG_BT_ACS_SHARED_SECRET_MAX_SIZE];
	size_t shared_secret_len;
	uint8_t parent_key[CONFIG_BT_ACS_SESSION_KEY_SIZE];
	bool parent_key_valid;
	uint8_t session_key[CONFIG_BT_ACS_SESSION_KEY_SIZE];
	bool session_key_valid;
	uint8_t kdf_salt[CONFIG_BT_ACS_KDF_SALT_MAX_SIZE];
	size_t kdf_salt_len;
	uint8_t kdf_info[ACS_KEX_INFO_BUF_SIZE];
	size_t kdf_info_len;
	uint8_t received_confirm_code[32];
	size_t received_confirm_code_len;
	uint8_t server_random[32];
	size_t server_random_len;
};

void acs_kex_reset(struct acs_kex_ctx *ctx);

int acs_kex_start(struct acs_conn_ctx *conn_ctx, uint16_t key_id, uint8_t confirmation_method,
		  uint8_t confirmation_action);

int acs_kex_build_ecdh_response(struct acs_conn_ctx *conn_ctx, uint16_t key_id,
				const uint8_t *operand, uint16_t operand_len,
				struct net_buf *response_buf);

int acs_kex_build_kdf_response(struct acs_conn_ctx *conn_ctx, uint16_t key_id,
			       struct net_buf *response_buf, bool *send_final_response);

int acs_kex_build_confirmation_code_response(struct acs_conn_ctx *conn_ctx, uint16_t key_id,
					     const uint8_t *operand, uint16_t operand_len,
					     struct net_buf *response_buf);

int acs_kex_build_confirmation_random_response(struct acs_conn_ctx *conn_ctx, uint16_t key_id,
					       const uint8_t *operand, uint16_t operand_len,
					       struct net_buf *response_buf,
					       bool *send_final_response);

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_KEX_H_ */
