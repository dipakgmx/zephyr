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

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#define ACS_OUTPUT_NUMERIC_BUCKET_COUNT 9U

struct acs_cp_result acs_cp_kex_get_current_key_list(struct acs_reply *reply,
						     struct net_buf_simple *payload)
{
	ARG_UNUSED(payload);

	/* Include every Key_ID accepted by Invalidate Key (§4.4.3.9). */
	uint8_t buf[sizeof(uint8_t) + ACS_KEY_RUNTIME_COUNT * sizeof(uint16_t)];
	uint8_t count = 0;
	uint16_t pos = sizeof(uint8_t); /* Byte 0 reserved for count */

	for (size_t i = 0; i < ACS_KEY_RUNTIME_COUNT; i++) {
		const struct bt_acs_key_desc_runtime *current_key =
			&reply->conn->crypto.key_runtimes[i];

		if (!acs_current_key_installed(current_key)) {
			continue;
		}

		sys_put_le16(acs_key_desc_runtime_key_id(current_key), &buf[pos]);
		pos += sizeof(uint16_t);
		count++;
	}

	buf[0] = count;

	if (net_buf_tailroom(reply->response) < pos) {
		LOG_ERR("no room for the current key list (%u octets)", pos);
		return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	net_buf_add_mem(reply->response, buf, pos);
	return acs_cp_reply();
}

struct acs_cp_result acs_cp_kex_exchange_kdf(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	struct acs_kdf_req req_data;
	bool standalone;
	int err;

	/* Copy the packed operand before aligned access. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));
	standalone = (sys_le16_to_cpu(req_data.key_id) == ACS_KEY_ID_KDF);

	/* Standalone KDF requires an established parent key. */
	err = standalone ? acs_key_exchange_kdf(acs_conn, reply->response)
			 : acs_key_exchange_ecdh_kdf(acs_conn, reply->response);
	if (err) {
		if (err != -EAGAIN) {
			LOG_ERR("%s KDF key exchange: internal error (err %d)",
				standalone ? "standalone" : "ECDH", err);
		}
		return acs_cp_status(errno_to_acs_status(err));
	}

	if (standalone) {
		acs_kex_conclude(reply);
	}
	return acs_cp_reply();
}

struct acs_cp_result acs_cp_kex_start(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	struct acs_cp_start_key_exchange_req req_data;
	uint16_t key_id;
	uint8_t method;
	uint8_t action;
	bool method_action_valid;
	const struct bt_acs_cb *cb;
	int err;
	uint32_t oob_num;
	int oob_err;

	/* Copy the packed operand out of the request buffer for aligned access. */
	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));
	key_id = sys_le16_to_cpu(req_data.key_id);
	method = req_data.confirmation_method;
	action = req_data.confirmation_action;
	method_action_valid = true;
	cb = acs_cb_get();

	/* The selected confirmation method must be advertised (§4.4.4.18.2). */
	switch (method) {
	case BT_ACS_CONFIRM_METHOD_NONE:
		/* Always supported; §4.4.4.18.3 requires action 0xFF. */
		if (action != BT_ACS_CONFIRM_ACTION_NOT_APPLICABLE) {
			method_action_valid = false;
		}
		break;

	case BT_ACS_CONFIRM_METHOD_OUTPUT_OOB:
		/* Output methods are advertised individually (Table 4.51). */
		if (action == BT_ACS_CONFIRM_ACTION_OUTPUT_BEEP) {
			method_action_valid = IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_BEEP);
		} else if (action == BT_ACS_CONFIRM_ACTION_OUTPUT_NUMERIC) {
			method_action_valid = IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC);
		} else {
			method_action_valid = false;
		}
		break;

	case BT_ACS_CONFIRM_METHOD_INPUT_OOB:
		/* Input methods are advertised individually (Table 4.52). */
		if (action == BT_ACS_CONFIRM_ACTION_INPUT_PUSH) {
			method_action_valid = IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_PUSH);
		} else if (action == BT_ACS_CONFIRM_ACTION_INPUT_NUMERIC) {
			method_action_valid = IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_NUMERIC);
		} else {
			method_action_valid = false;
		}
		break;

	default:
		/* Static OOB (0x03) is not implemented; 0x04-0xFF are RFU. */
		method_action_valid = false;
		break;
	}

	if (!method_action_valid) {
		LOG_WRN("Start Key Exchange: unsupported method/action (method=0x%02x "
			"action=0x%02x)",
			method, action);
		return acs_cp_status(BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	}

	if (acs_kex_in_progress(acs_conn)) {
		LOG_WRN("Start Key Exchange rejected - another key exchange is already active");
		return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	}

	if (key_id == ACS_KEY_ID_KDF) {
		/* Standalone KDF requires confirmation method None. */
		if (method != BT_ACS_CONFIRM_METHOD_NONE ||
		    action != BT_ACS_CONFIRM_ACTION_NOT_APPLICABLE) {
			LOG_WRN("invalid confirmation method/action (method=0x%02x action=0x%02x)",
				method, action);
			return acs_cp_status(BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		}
	}

	if (acs_kex_alloc(acs_conn) == NULL) {
		return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	/* Store the method before application callbacks can provide OOB data. */
	acs_conn->kex->start_kex = req_data;

	if (key_id != ACS_KEY_ID_KDF) {
		err = acs_key_exchange_ecdh_start(acs_conn, key_id);
		if (err != 0) {
			return acs_cp_status(errno_to_acs_status(err));
		}
	}

	memset(acs_conn->kex->auth_value, 0, sizeof(acs_conn->kex->auth_value));

	switch (method) {
	case BT_ACS_CONFIRM_METHOD_OUTPUT_OOB:
		/* Generate the AuthValue with a CSPRNG and store it big-endian. */
		oob_err = psa_generate_random((uint8_t *)&oob_num, sizeof(oob_num));
		if (oob_err != PSA_SUCCESS) {
			LOG_ERR("Failed to generate OOB number: %d", oob_err);
			return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
		}
		/* Beep uses the nine audible values defined by its action. */
		if (action == BT_ACS_CONFIRM_ACTION_OUTPUT_BEEP) {
			oob_num = (oob_num % ACS_OUTPUT_NUMERIC_BUCKET_COUNT) + 1;
		} else {
#if IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_OUTPUT_NUMERIC)
			oob_num = (oob_num % CONFIG_BT_ACS_CONFIRMATION_OUTPUT_MAX_VALUE) + 1;
#else
			oob_num = (oob_num % ACS_OUTPUT_NUMERIC_BUCKET_COUNT) + 1;
#endif
		}
		sys_put_be32(oob_num,
			     &acs_conn->kex->auth_value[ACS_CONFIRM_VALUE_SIZE - sizeof(oob_num)]);
		if (cb != NULL && cb->output_oob_number != NULL) {
			cb->output_oob_number(reply->conn->conn, action, oob_num);
		}
		break;
	case BT_ACS_CONFIRM_METHOD_INPUT_OOB:
		if (cb != NULL && cb->input_oob_request != NULL) {
			cb->input_oob_request(reply->conn->conn, action);
		}
		break;
	default:
		break;
	}

	acs_conn->kex->state =
		(key_id == ACS_KEY_ID_KDF) ? ACS_KEX_AWAIT_KDF : ACS_KEX_AWAIT_PUBKEY;

	return acs_cp_status(BT_ACS_CP_RESPONSE_SUCCESS);
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

struct acs_cp_result acs_cp_kex_exchange_ecdh(struct acs_reply *reply, struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	int err;

	if (acs_kex_parse_client_pubkey(buf, &acs_conn->kex->client_pubkey) != 0) {
		LOG_ERR("ECDH pubkey: malformed public-key operand");
		return acs_cp_status(BT_ACS_CP_RESPONSE_INVALID_PUBLIC_KEY);
	}

	err = acs_key_exchange_ecdh_pubkey(acs_conn, reply->response);
	if (err == -EBADMSG || err == -EINVAL) {
		LOG_ERR("ECDH pubkey: invalid client public key (err %d)", err);
		return acs_cp_status(BT_ACS_CP_RESPONSE_INVALID_PUBLIC_KEY);
	} else if (err) {
		if (err != -EAGAIN) {
			LOG_ERR("ECDH pubkey: internal error (err %d)", err);
		}
		return acs_cp_status(errno_to_acs_status(err));
	}

	return acs_cp_reply();
}

struct acs_cp_result acs_cp_kex_ecdh_confirm_code(struct acs_reply *reply,
						  struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	struct acs_cp_ecdh_confirm_code_req req_data;
	int err;

	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));
	memcpy(acs_conn->kex->client_confirm, req_data.confirm_code, ACS_CONFIRM_VALUE_SIZE);

	err = acs_key_exchange_ecdh_confirm_code(acs_conn, reply->response);
	if (err) {
		return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	return acs_cp_reply();
}

struct acs_cp_result acs_cp_kex_ecdh_confirm_rand(struct acs_reply *reply,
						  struct net_buf_simple *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	struct acs_cp_ecdh_confirm_rand_req req_data;
	int err;

	memcpy(&req_data, net_buf_simple_pull_mem(buf, sizeof(req_data)), sizeof(req_data));

	err = acs_key_exchange_ecdh_confirm_rand(acs_conn, req_data.random, reply->response);
	if (err == -EINVAL) {
		return acs_cp_status(BT_ACS_CP_RESPONSE_INVALID_OPERAND);
	} else if (err == -EACCES) {
		return acs_cp_status(BT_ACS_CP_RESPONSE_INVALID_KEY_EXCHANGE_CONFIRMATION_CODE);
	} else if (err) {
		return acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED);
	}

	acs_kex_conclude(reply);
	return acs_cp_reply();
}
