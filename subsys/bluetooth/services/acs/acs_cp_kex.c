/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_cp_operands.h"
#include "acs_internal.h"
#include "acs_reply.h"
#include "acs_key_desc.h"
#include "acs_key_exchange.h"
#include "acs_keys.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

uint8_t acs_cp_kex_get_current_key_list(struct acs_reply *reply, struct net_buf_simple *payload)
{
	ARG_UNUSED(payload);

	uint16_t key_ids[ACS_KEY_COUNT];
	uint8_t count = acs_keys_list(reply->conn, key_ids);

	if (net_buf_tailroom(reply->response) < sizeof(count) + count * sizeof(uint16_t)) {
		LOG_ERR("no room for the current key list (%u keys)", count);
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	net_buf_add_u8(reply->response, count);
	for (uint8_t i = 0; i < count; i++) {
		net_buf_add_le16(reply->response, key_ids[i]);
	}

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

uint8_t acs_cp_kex_exchange_kdf(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	bool standalone;
	int err;

	standalone = (net_buf_simple_pull_le16(buf) == ACS_KEY_ID_KDF);

	/* Standalone KDF requires an established parent key. */
	err = standalone ? acs_key_exchange_kdf(acs_conn, reply->response)
			 : acs_key_exchange_ecdh_kdf(acs_conn, reply->response);
	if (err) {
		if (err != -EAGAIN) {
			LOG_ERR("%s KDF key exchange: internal error (err %d)",
				standalone ? "standalone" : "ECDH", err);
		}
		return errno_to_acs_status(err);
	}

	if (!standalone) {
		acs_conn->kex->next_opcode = BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE;
		return BT_ACS_CP_RESPONSE_SUCCESS;
	}

	acs_conn->kex->next_opcode = ACS_KEX_COMPLETE;
	return (acs_kex_add_result(reply) == 0) ? BT_ACS_CP_RESPONSE_SUCCESS
						: BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
}

/* Whether the AC Server advertises this confirmation method and action (§4.4.4.18.2). */
static bool confirmation_supported(uint8_t method, uint8_t action)
{
	switch (method) {
	case BT_ACS_CONFIRM_METHOD_NONE:
		/* §4.4.4.18.3 requires action 0xFF. */
		return action == BT_ACS_CONFIRM_ACTION_NOT_APPLICABLE;
	case BT_ACS_CONFIRM_METHOD_OUTPUT_OOB:
		/* Table 4.51: Output Numeric is the only output action offered. */
		return action == BT_ACS_CONFIRM_ACTION_OUTPUT_NUMERIC &&
		       IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC);
	case BT_ACS_CONFIRM_METHOD_INPUT_OOB:
		/* Table 4.52 */
		return (action == BT_ACS_CONFIRM_ACTION_INPUT_PUSH &&
			IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_PUSH)) ||
		       (action == BT_ACS_CONFIRM_ACTION_INPUT_NUMERIC &&
			IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_NUMERIC));
	default:
		/* Static OOB (0x03) is not implemented; 0x04-0xFF are RFU. */
		return false;
	}
}

#if IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC)
/*
 * Pick the Output OOB number between 1 and the advertised maximum (§4.4.4.27.8),
 * store it as the AuthValue and hand it to the application.
 */
static uint8_t output_oob_start(struct bt_acs_conn *acs_conn, uint8_t action)
{
	const struct bt_acs_cb *cb = acs_cb_get();
	uint32_t number;
	psa_status_t status;

	status = psa_generate_random((uint8_t *)&number, sizeof(number));
	if (status != PSA_SUCCESS) {
		LOG_ERR("Failed to generate OOB number: %d", status);
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	number = (number % CONFIG_BT_ACS_CONFIRMATION_OUTPUT_MAX_VALUE) + 1U;
	sys_put_be32(number, &acs_conn->kex->auth_value[ACS_CONFIRM_VALUE_SIZE - sizeof(number)]);

	if (cb != NULL && cb->output_oob_number != NULL) {
		cb->output_oob_number(acs_conn->conn, action, number);
	}

	return BT_ACS_CP_RESPONSE_SUCCESS;
}
#endif

uint8_t acs_cp_kex_start(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	const struct bt_acs_cb *cb = acs_cb_get();
	struct acs_cp_start_key_exchange_req req;
	uint16_t key_id;
	int err;

	/* Copy the packed operand out of the request buffer for aligned access. */
	memcpy(&req, net_buf_simple_pull_mem(buf, sizeof(req)), sizeof(req));
	key_id = sys_le16_to_cpu(req.key_id);

	if (!confirmation_supported(req.confirmation_method, req.confirmation_action)) {
		LOG_WRN("Start Key Exchange: unsupported method 0x%02x action 0x%02x",
			req.confirmation_method, req.confirmation_action);
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	}

	if (acs_kex_in_progress(acs_conn)) {
		LOG_WRN("Start Key Exchange rejected - another key exchange is already active");
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	}

	if (key_id == ACS_KEY_ID_KDF && req.confirmation_method != BT_ACS_CONFIRM_METHOD_NONE) {
		LOG_WRN("standalone KDF key exchange takes no confirmation method");
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	}

	if (acs_kex_alloc(acs_conn) == NULL) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	/* Store the method before application callbacks can provide OOB data. */
	acs_conn->kex->start_kex = req;

	if (key_id != ACS_KEY_ID_KDF) {
		err = acs_key_exchange_ecdh_start(acs_conn, key_id);
		if (err != 0) {
			return errno_to_acs_status(err);
		}
	}

	memset(acs_conn->kex->auth_value, 0, sizeof(acs_conn->kex->auth_value));

#if IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC)
	if (req.confirmation_method == BT_ACS_CONFIRM_METHOD_OUTPUT_OOB) {
		uint8_t rc = output_oob_start(acs_conn, req.confirmation_action);

		if (rc != BT_ACS_CP_RESPONSE_SUCCESS) {
			return rc;
		}
	}
#endif

	if (req.confirmation_method == BT_ACS_CONFIRM_METHOD_INPUT_OOB && cb != NULL &&
	    cb->input_oob_request != NULL) {
		cb->input_oob_request(acs_conn->conn, req.confirmation_action);
	}

	acs_conn->kex->next_opcode = (key_id == ACS_KEY_ID_KDF) ? BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF
								: BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH;

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

/* Parse and zero-extend the variable-length ECDH coordinates (Table 4.66). */
static int acs_kex_parse_client_pubkey(struct net_buf_simple *buf, struct acs_ecdh_pubkey *pk)
{
	uint16_t key_id;
	uint8_t x_size;
	uint8_t y_size;

	if (buf->len < sizeof(uint16_t) + 1U) {
		return -EINVAL;
	}
	key_id = net_buf_simple_pull_le16(buf);
	x_size = net_buf_simple_pull_u8(buf);
	if (x_size == 0U || x_size > ACS_ECDH_COORD_SIZE || buf->len < (uint16_t)x_size + 1U) {
		return -EINVAL;
	}

	memset(pk, 0, sizeof(*pk));
	pk->key_id = sys_cpu_to_le16(key_id);
	memcpy(pk->x, net_buf_simple_pull_mem(buf, x_size), x_size);
	pk->x_size = ACS_ECDH_COORD_SIZE;

	y_size = net_buf_simple_pull_u8(buf);
	if (y_size == 0U || y_size > ACS_ECDH_COORD_SIZE || buf->len < y_size) {
		return -EINVAL;
	}
	memcpy(pk->y, net_buf_simple_pull_mem(buf, y_size), y_size);
	pk->y_size = ACS_ECDH_COORD_SIZE;

	return 0;
}

uint8_t acs_cp_kex_exchange_ecdh(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	int err;

	if (acs_kex_parse_client_pubkey(buf, &acs_conn->kex->client_pubkey) != 0) {
		LOG_ERR("ECDH pubkey: malformed public-key operand");
		return BT_ACS_CP_RESPONSE_INVALID_PUBLIC_KEY;
	}

	err = acs_key_exchange_ecdh_pubkey(acs_conn, reply->response);
	if (err == -EBADMSG || err == -EINVAL) {
		LOG_ERR("ECDH pubkey: invalid client public key (err %d)", err);
		return BT_ACS_CP_RESPONSE_INVALID_PUBLIC_KEY;
	} else if (err) {
		if (err != -EAGAIN) {
			LOG_ERR("ECDH pubkey: internal error (err %d)", err);
		}
		return errno_to_acs_status(err);
	}

	acs_conn->kex->next_opcode = BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF;
	return BT_ACS_CP_RESPONSE_SUCCESS;
}

uint8_t acs_cp_kex_ecdh_confirm_code(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	struct acs_cp_ecdh_confirm_code_req req_data;
	int err;

	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));
	memcpy(acs_conn->kex->client_confirm, req_data.confirm_code, ACS_CONFIRM_VALUE_SIZE);

	err = acs_key_exchange_ecdh_confirm_code(acs_conn, reply->response);
	if (err) {
		return errno_to_acs_status(err);
	}

	acs_conn->kex->next_opcode = BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND;
	return BT_ACS_CP_RESPONSE_SUCCESS;
}

uint8_t acs_cp_kex_ecdh_confirm_rand(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	struct acs_cp_ecdh_confirm_rand_req req_data;
	int err;

	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));

	err = acs_key_exchange_ecdh_confirm_rand(acs_conn, req_data.random, reply->response);
	if (err == -EINVAL) {
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	} else if (err == -EACCES) {
		return BT_ACS_CP_RESPONSE_INVALID_KEY_EXCHANGE_CONFIRMATION_CODE;
	} else if (err) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	acs_conn->kex->next_opcode = ACS_KEX_COMPLETE;
	return (acs_kex_add_result(reply) == 0) ? BT_ACS_CP_RESPONSE_SUCCESS
						: BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
}
