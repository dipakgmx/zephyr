/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_internal.h"
#include "acs_data_in.h"
#include "acs_runtime.h"
#include "acs_crypto.h"
#include "acs_isc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Protected_Resource_Handle alone; Request_Or_Response is C.1 (Table 4.10). */
#define ACS_SECURE_DATA_PLAIN_MIN_SIZE 2

/* Check that alg can unprotect Data In; an unusable setup outranks a missing key (§4.3.2). */
static uint8_t acs_data_in_check_alg(const struct acs_sec_alg *alg)
{
	if (alg == NULL) {
		return BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG;
	}

	if (!acs_sec_alg_ready(alg)) {
		LOG_WRN("no key exchanged for Key_ID 0x%04x", acs_sec_alg_id(alg));
		return BT_ACS_ATT_ERR_INVALID_KEY;
	}

	if (acs_key_desc_nonce_prefix_size(alg->key_desc) > 0U && !alg->client_nonce_set) {
		LOG_WRN("client nonce fixed not set for Key_ID 0x%04x", acs_sec_alg_id(alg));
		return BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG;
	}

	return BT_ATT_ERR_SUCCESS;
}

/*
 * Pull Nonce_Var and reject a replayed sequence number; buf advances to
 * MAC || ciphertext.
 */
static uint8_t acs_data_in_pull_counter(struct net_buf *buf, const struct acs_sec_alg *alg,
					uint64_t *received_counter)
{
	uint8_t nonce_var_size = acs_key_desc_nonce_var_size(alg->key_desc);
	uint16_t controls_size;
	uint64_t counter = 0U;

	/* After ISC_ID: Nonce_Var, MAC, then protected data, all LSO first. */
	controls_size = (uint16_t)nonce_var_size + acs_key_desc_auth_tag_size(alg->key_desc);
	if (buf->len <= controls_size) {
		LOG_ERR("secure data (%u) too short for %u octets of security controls", buf->len,
			controls_size);
		return BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG;
	}

	if (nonce_var_size > 0U) {
		sys_get_le(&counter, net_buf_pull_mem(buf, nonce_var_size), nonce_var_size);

		if (counter < alg->rx_nonce_counter) {
			LOG_WRN("stale or replayed nonce (received=0x%016llx vs "
				"min_expected=0x%016llx)",
				(unsigned long long)counter,
				(unsigned long long)alg->rx_nonce_counter);
			return BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG;
		}
	}

	*received_counter = counter;
	return BT_ATT_ERR_SUCCESS;
}

/*
 * Decrypt in place - MAC || ciphertext in, plaintext out - and pull the
 * resource handle off the front.
 */
static uint8_t acs_data_in_decrypt(struct net_buf *buf, struct acs_sec_alg *alg,
				   uint64_t received_counter, uint16_t *resource_handle)
{
	uint16_t plain_len = 0;
	int err;

	/* The crypto layer updates the receive counter only after authentication. */
	err = acs_crypto_decrypt(alg, received_counter, buf->data, buf->len, &plain_len);
	if (err) {
		if (err == -ENOSPC) {
			/* Do not end the session based on an unauthenticated counter. */
			LOG_WRN("RX nonce space exhausted, rejecting data-in");
			return BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG;
		}

		if (err == -EACCES) {
			LOG_ERR("decryption failed: authentication tag mismatch (invalid "
				"key/tampered data)");
		} else {
			LOG_ERR("decryption failed: err=%d (Key_ID 0x%04x)", err, acs_sec_alg_id(alg));
		}

		return BT_ACS_ATT_ERR_INVALID_KEY;
	}

	buf->len = plain_len;

	/* Plaintext must contain a Protected_Resource_Handle (§4.3.2). */
	if (plain_len < ACS_SECURE_DATA_PLAIN_MIN_SIZE) {
		LOG_ERR("decrypted payload too short (%u)", plain_len);
		return BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED;
	}

	*resource_handle = net_buf_pull_le16(buf);
	if (*resource_handle == 0U) {
		LOG_ERR("invalid zero resource handle in decrypted payload");
		return BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED;
	}

	return BT_ATT_ERR_SUCCESS;
}

uint8_t acs_data_in_unwrap_and_route(struct bt_acs_conn *acs_conn, struct net_buf *buf)
{
	struct acs_frame frame = {.source_channel = ACS_SRC_DATA_IN};
	uint64_t received_counter;
	uint8_t att_err;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(buf != NULL);

	if (!atomic_test_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED)) {
		return BT_ACS_ATT_ERR_INVALID_KEY;
	}

	if (buf->len < ACS_DATA_IN_HDR_SIZE) {
		LOG_ERR("data-in payload too short for ISC_ID (%u)", buf->len);
		return BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG;
	}

	frame.isc_id = net_buf_pull_le16(buf);
	frame.alg = acs_isc_alg(acs_conn, frame.isc_id);

	att_err = acs_data_in_check_alg(frame.alg);
	if (att_err != BT_ATT_ERR_SUCCESS) {
		return att_err;
	}

	att_err = acs_data_in_pull_counter(buf, frame.alg, &received_counter);
	if (att_err != BT_ATT_ERR_SUCCESS) {
		return att_err;
	}

	att_err = acs_data_in_decrypt(buf, frame.alg, received_counter, &frame.resource_handle);
	if (att_err != BT_ATT_ERR_SUCCESS) {
		return att_err;
	}

	frame.payload = buf->data;
	frame.payload_len = buf->len;

	return acs_runtime_dispatch_frame(&frame, acs_conn);
}
