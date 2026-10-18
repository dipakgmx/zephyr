/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_cp_operands.h"
#include "acs_internal.h"
#include "acs_runtime.h"
#include "acs_reply.h"
#include "acs_cp.h"
#include "acs_cp_handlers.h"
#include "acs_key_exchange.h"
#include "acs_rmap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)
/* ATT_MTU excludes the ATT opcode and handle (Table 4.7). */
static uint16_t acs_cp_att_mtu_value(struct bt_conn *conn)
{
	return bt_gatt_get_mtu(conn) - ACS_SEG_ATT_HDR_SIZE;
}

/* Handle the Get ATT MTU procedure (§4.4.3.19). */
static uint8_t acs_cp_handle_att_mtu(struct acs_reply *reply, struct net_buf_simple *payload)
{
	ARG_UNUSED(payload);

	net_buf_add_le16(reply->response, acs_cp_att_mtu_value(reply->acs_conn->conn));
	return BT_ACS_CP_RESPONSE_SUCCESS;
}

void acs_cp_indicate_att_mtu(struct bt_acs_conn *acs_conn)
{
	struct acs_reply *reply;
	struct net_buf *buf;
	uint16_t mtu;

	if (acs_cp_ccc_check(acs_conn->conn) != 0) {
		return;
	}

	reply = acs_reply_alloc(acs_conn);
	if (reply == NULL) {
		return;
	}

	reply->channel = ACS_REPLY_CP;

	/* Do not send this indication during another Control Point procedure. */
	if (!acs_cp_lock(reply)) {
		acs_reply_free(reply);
		return;
	}

	atomic_set_bit(&acs_conn->state, ACS_STATE_CP_SERVER_TX);

	buf = acs_reply_add_message(reply);
	if (buf == NULL) {
		acs_reply_free(reply);
		return;
	}

	mtu = acs_cp_att_mtu_value(acs_conn->conn);
	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_ATT_MTU_RESPONSE);
	net_buf_add_le16(buf, mtu);

	if (acs_reply_submit(reply) != 0) {
		acs_reply_free(reply);
	}
}
#endif /* CONFIG_BT_ACS_ATT_MTU */

/*
 * ACS Control Point opcode dispatch entry. A missing entry means Opcode Not Supported.
 *
 * A handler returns a Response Code (§4.4.5). On BT_ACS_CP_RESPONSE_SUCCESS the
 * procedure answers with the messages in reply->response, in order: the first
 * starts with rsp_opcode, and the handler may append more. When rsp_opcode is
 * the Response Code opcode, a Success Response Code follows the handler's
 * messages instead. Any other value discards them and is sent as a Response Code.
 */
struct acs_cp_opcode_info {
	uint8_t opcode;            /* Control Point opcode */
	uint16_t min_operand_size; /* Shortest valid operand */
	uint8_t rsp_opcode;        /* Response opcode sent on success */
	uint8_t (*handler)(struct acs_reply *reply, struct net_buf_simple *payload);
};

static const struct acs_cp_opcode_info acs_cp_opcodes[] = {
	{BT_ACS_CP_OPCODE_GET_ACS_FEATURE, 0U, BT_ACS_CP_OPCODE_ACS_FEATURE_RESPONSE,
	 acs_cp_handle_get_feature},
#if IS_ENABLED(CONFIG_BT_ACS_ATT_MTU)
	{BT_ACS_CP_OPCODE_ATT_MTU, 0U, BT_ACS_CP_OPCODE_ATT_MTU_RESPONSE, acs_cp_handle_att_mtu},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	{BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED, sizeof(struct acs_cp_set_client_nonce_fixed_req),
	 BT_ACS_CP_OPCODE_RESPONSE_CODE, acs_cp_handle_set_client_nonce_fixed},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	{BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP,
	 sizeof(struct acs_cp_activate_restriction_map_req), BT_ACS_CP_OPCODE_RESPONSE_CODE,
	 acs_cp_handle_activate_restriction_map},
	{BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR,
	 sizeof(struct acs_rmap_get_descriptor_req),
	 BT_ACS_CP_OPCODE_RESTRICTION_MAP_DESCRIPTOR_RESPONSE,
	 acs_cp_handle_get_restriction_map_descriptor},
	{BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_ID_LIST, 0U,
	 BT_ACS_CP_OPCODE_RESTRICTION_MAP_ID_LIST_RESPONSE,
	 acs_cp_handle_get_restriction_map_id_list},
#endif
	{BT_ACS_CP_OPCODE_GET_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE,
	 sizeof(struct acs_cp_get_svc_char_uuids_req),
	 BT_ACS_CP_OPCODE_SERVICE_CHARACTERISTIC_UUIDS_CHAR_RESOURCE_HANDLE_RESPONSE,
	 acs_cp_handle_get_svc_char_uuids},
	{BT_ACS_CP_OPCODE_GET_RESOURCE_HANDLE_UUID_MAP, 0U,
	 BT_ACS_CP_OPCODE_RESOURCE_HANDLE_UUID_MAP_RESPONSE,
	 acs_cp_handle_get_resource_handle_uuid_map},
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	{BT_ACS_CP_OPCODE_START_KEY_EXCHANGE, sizeof(struct acs_cp_start_key_exchange_req),
	 BT_ACS_CP_OPCODE_RESPONSE_CODE, acs_cp_kex_start},
	{BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR, sizeof(struct acs_cp_get_key_descriptor_req),
	 BT_ACS_CP_OPCODE_KEY_DESCRIPTOR_RESPONSE, acs_cp_handle_get_key_descriptor},
	{BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST, 0U, BT_ACS_CP_OPCODE_CURRENT_KEY_LIST_RESPONSE,
	 acs_cp_kex_get_current_key_list},
	{BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF, sizeof(struct acs_kdf_req),
	 BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF_RESPONSE, acs_cp_kex_exchange_kdf},
	{BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH, ACS_ECDH_PUBKEY_MIN_OPERAND,
	 BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_RESPONSE, acs_cp_kex_exchange_ecdh},
	{BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_CODE,
	 sizeof(struct acs_cp_ecdh_confirm_code_req),
	 BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_CODE_RESPONSE,
	 acs_cp_kex_ecdh_confirm_code},
	{BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_RANDOM_NUMBER,
	 sizeof(struct acs_cp_ecdh_confirm_rand_req),
	 BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_RANDOM_NUMBER_RESPONSE,
	 acs_cp_kex_ecdh_confirm_rand},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	{BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR,
	 sizeof(struct acs_cp_get_isc_descriptor_req),
	 BT_ACS_CP_OPCODE_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR_RESPONSE,
	 acs_cp_handle_get_isc_descriptor},
	{BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS, 0U, BT_ACS_CP_OPCODE_RESPONSE_CODE,
	 acs_cp_all_active_get},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	{BT_ACS_CP_OPCODE_INVALIDATE_KEY, sizeof(struct acs_cp_invalidate_key_req),
	 BT_ACS_CP_OPCODE_RESPONSE_CODE, acs_sec_mgmt_invalidate_key},
	{BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY, 0U, BT_ACS_CP_OPCODE_RESPONSE_CODE,
	 acs_sec_mgmt_invalidate_all},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
	{BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH, sizeof(struct acs_cp_sec_switch_req),
	 BT_ACS_CP_OPCODE_RESPONSE_CODE, acs_sec_mgmt_set_security_switch},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)
	{BT_ACS_CP_OPCODE_INITIATE_PAIRING, 0U, BT_ACS_CP_OPCODE_RESPONSE_CODE,
	 acs_sec_mgmt_initiate_pairing},
#endif
};

static const struct acs_cp_opcode_info *acs_cp_opcode_lookup(uint8_t opcode)
{
	ARRAY_FOR_EACH_PTR(acs_cp_opcodes, info) {
		if (info->opcode == opcode) {
			return info;
		}
	}
	return NULL;
}

/* Append a Response Code message for req_opcode (§4.4.4.1). */
static int acs_cp_add_response_code(struct acs_reply *reply, uint8_t req_opcode, uint8_t code)
{
	struct net_buf *buf = acs_reply_add_message(reply);

	if (buf == NULL) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_RESPONSE_CODE);
	net_buf_add_u8(buf, req_opcode);
	net_buf_add_u8(buf, code);
	return 0;
}

int acs_cp_rsp_status(struct acs_reply *reply, uint8_t req_opcode, uint8_t code)
{
	int err;

	net_buf_drop(&reply->response);

	err = acs_cp_add_response_code(reply, req_opcode, code);
	if (err) {
		return err;
	}

	return acs_reply_submit(reply);
}

/* Start the procedure response with its response opcode, if it has one. */
static uint8_t acs_cp_prepare_response(struct acs_reply *reply,
				       const struct acs_cp_opcode_info *info)
{
	struct net_buf *rsp_buf;

	/*
	 * Nothing to add yet. A Response Code holds the handler's result, so
	 * acs_cp_execute() adds it once the handler has returned.
	 */
	if (info->rsp_opcode == BT_ACS_CP_OPCODE_RESPONSE_CODE) {
		return BT_ACS_CP_RESPONSE_SUCCESS;
	}

	rsp_buf = acs_reply_add_message(reply);
	if (rsp_buf == NULL) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	net_buf_add_u8(rsp_buf, info->rsp_opcode);
	return BT_ACS_CP_RESPONSE_SUCCESS;
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
/* Whether an ACS Control Point procedure requires protection. */
enum acs_cp_protection {
	ACS_CP_UNPROTECTED,
	ACS_CP_PROTECTED,
	ACS_CP_UNDECIDABLE, /* No resolvable map; Section 4.4.5 defines the result */
};

/* Required protection and ISC_ID for a Control Point procedure. */
struct acs_cp_protection_state {
	enum acs_cp_protection protection;
	uint16_t isc_id;
};

/* Operand begins with Restriction_Map_ID; channel follows that map (Section 4.4.3.3). */
static struct acs_cp_protection_state
acs_cp_get_target_rmap_protection_state(const struct net_buf_simple *operand)
{
	struct acs_cp_protection_state state = {
		.protection = ACS_CP_UNDECIDABLE,
		.isc_id = BT_ACS_ISC_ID_NONE,
	};
	const struct bt_acs_restriction_map *map;
	uint16_t map_id;

	if (operand->len < sizeof(uint16_t)) {
		return state;
	}

	map_id = sys_get_le16(operand->data);
	map = acs_rmap_lookup(map_id);
	if (map == NULL) {
		return state;
	}

	state.isc_id = map->map_isc_id;
	if (state.isc_id != BT_ACS_ISC_ID_NONE) {
		state.protection = ACS_CP_PROTECTED;
	} else {
		state.protection = ACS_CP_UNPROTECTED;
	}

	return state;
}

/* Checks if a control point request is protected by the active restriction map, and if so, which
 * ISC_ID it requires.
 */
static struct acs_cp_protection_state
acs_cp_get_opcode_protection_state(const struct bt_acs_restriction_map *active_rmap, uint8_t opcode,
				   const struct net_buf_simple *operand)
{
	struct acs_cp_protection_state state = {
		.protection = ACS_CP_UNPROTECTED,
		.isc_id = BT_ACS_ISC_ID_NONE,
	};

	switch (opcode) {
	case BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS: {
		uint16_t descriptor_isc;
		/* If any of the active descriptors are protected (eg: the active rmap, ISC
		 * descriptor, or any of the active key descriptors), the procedure is
		 * protected. See (§4.4.3.1).
		 */
		if (acs_rmap_is_active_descriptor_protected(active_rmap, &descriptor_isc)) {
			state.protection = ACS_CP_PROTECTED;
			state.isc_id = descriptor_isc;
		}
		return state;
	}
	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR:
	case BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP:
		return acs_cp_get_target_rmap_protection_state(operand);
	default:
		state.isc_id = acs_rmap_cp_opcode_isc_id(active_rmap, opcode);
		if (state.isc_id != BT_ACS_ISC_ID_NONE) {
			state.protection = ACS_CP_PROTECTED;
		}
		return state;
	}
}

/* Verifies if the request on the plain ACS Control Point requires protection, according to the
 * active restriction map (exception for Activate Restriction Map, see §4.4.3.4)
 */
static bool acs_cp_plain_requires_data_in(const struct bt_acs_restriction_map *active_rmap,
					  const uint8_t *payload, uint16_t payload_len)
{
	struct acs_cp_protection_state state;
	struct net_buf_simple operand;
	uint8_t opcode;

	if (active_rmap == NULL || payload_len == 0U) {
		return false;
	}

	opcode = payload[0];

	/*
	 * A protected map requested on the plain CP gets a Procedure Not Applicable
	 * Response Code, not an ATT error (§4.4.3.4); the Activate handler checks it.
	 */
	if (opcode == BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP) {
		return false;
	}

	net_buf_simple_init_with_data(&operand, (void *)&payload[1], payload_len - 1U);
	state = acs_cp_get_opcode_protection_state(active_rmap, opcode, &operand);

	return state.protection == ACS_CP_PROTECTED;
}

/*
 * ATT error for a procedure requested over Data In under isc_id (§4.3.2): the
 * procedure must be protected, and by that ISC.
 */
static uint8_t acs_cp_data_in_source_att_err(uint16_t isc_id,
					     const struct bt_acs_restriction_map *active_rmap,
					     uint8_t opcode, const struct net_buf_simple *operand)
{
	struct acs_cp_protection_state state;

	if (active_rmap == NULL) {
		LOG_ERR("CP opcode 0x%02x has no active restriction map", opcode);
		return BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG;
	}

	state = acs_cp_get_opcode_protection_state(active_rmap, opcode, operand);
	switch (state.protection) {
	case ACS_CP_UNDECIDABLE:
		return BT_ATT_ERR_SUCCESS;
	case ACS_CP_UNPROTECTED:
		/* A Data In request for a procedure ACS does not protect (§4.3.2). */
		return BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED;
	case ACS_CP_PROTECTED:
	default:
		break;
	}

	if (state.isc_id != BT_ACS_ISC_ID_NONE && isc_id != state.isc_id) {
		LOG_WRN("CP opcode 0x%02x wrong ISC 0x%04x (expected 0x%04x)", opcode, isc_id,
			state.isc_id);
		return BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIG;
	}

	return BT_ATT_ERR_SUCCESS;
}
#else
static bool acs_cp_plain_requires_data_in(const struct bt_acs_restriction_map *active_rmap,
					  const uint8_t *payload, uint16_t payload_len)
{
	ARG_UNUSED(active_rmap);
	ARG_UNUSED(payload);
	ARG_UNUSED(payload_len);
	return false;
}

static uint8_t acs_cp_data_in_source_att_err(uint16_t isc_id,
					     const struct bt_acs_restriction_map *active_rmap,
					     uint8_t opcode, const struct net_buf_simple *operand)
{
	ARG_UNUSED(isc_id);
	ARG_UNUSED(active_rmap);
	ARG_UNUSED(opcode);
	ARG_UNUSED(operand);
	return BT_ATT_ERR_SUCCESS;
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

/* Execute a queued Control Point procedure and release its request buffer. */
static void acs_cp_execute(struct acs_reply *reply)
{
	struct bt_acs_conn *acs_conn = reply->acs_conn;
	struct net_buf_simple operand;
	const struct acs_cp_opcode_info *info;
	uint8_t opcode;
	uint8_t rc;
	int err = 0;

	/* Both queue paths reject an empty request. */
	__ASSERT_NO_MSG(reply->request != NULL && reply->request->len > 0U);

	net_buf_simple_init_with_data(&operand, reply->request->data, reply->request->len);

	opcode = net_buf_simple_pull_u8(&operand);
	LOG_DBG("CP execute: opcode 0x%02x, operand len %u", opcode, operand.len);

	info = acs_cp_opcode_lookup(opcode);
	if (info == NULL) {
		LOG_WRN("CP execute: unsupported opcode 0x%02x", opcode);
		rc = BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED;
		goto handle_result;
	}

	if (operand.len < info->min_operand_size) {
		LOG_WRN("CP execute: opcode 0x%02x operand too short (%u)", opcode, operand.len);
		rc = BT_ACS_CP_RESPONSE_INVALID_OPERAND;
		goto handle_result;
	}

	if (!acs_key_exchange_is_step_allowed(acs_conn, opcode, &operand)) {
		rc = BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
		goto handle_result;
	}

	rc = acs_cp_prepare_response(reply, info);
	if (rc != BT_ACS_CP_RESPONSE_SUCCESS) {
		goto handle_result;
	}

	rc = info->handler(reply, &operand);

handle_result:
	/* Releasing the incoming request buffer, since reading the data is done */
	net_buf_drop(&reply->request);

	if (rc != BT_ACS_CP_RESPONSE_SUCCESS) {
		/* A failed key exchange procedure ends the running exchange (§4.4.3.17.1). */
		acs_key_exchange_abort_if_active(acs_conn, opcode);
		err = acs_cp_rsp_status(reply, opcode, rc);
	} else {
		if (info->rsp_opcode == BT_ACS_CP_OPCODE_RESPONSE_CODE) {
			err = acs_cp_add_response_code(reply, opcode, rc);
		}

		if (err == 0) {
			err = acs_reply_submit(reply);
		}
	}

	if (err == 0) {
		return;
	}

	/* The client never sees this response, so the key exchange cannot continue. */
	acs_key_exchange_abort_if_active(acs_conn, opcode);

	/* The ATT write is complete; report this failure with a Response Code. */
	if (acs_cp_rsp_status(reply, opcode, BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED) != 0) {
		acs_reply_free(reply);
	}
}

/* Execute accepted procedures outside the cooperative BT RX thread. */
static void acs_cp_exec_work_handler(struct k_work *work)
{
	struct bt_acs_conn *acs_conn = CONTAINER_OF(work, struct bt_acs_conn, cp_exec_work);
	sys_snode_t *snode;

	while ((snode = k_fifo_get(&acs_conn->cp_exec_fifo, K_NO_WAIT)) != NULL) {
		struct acs_reply *reply = CONTAINER_OF(snode, struct acs_reply, node);

		/* Abort holds the lock and suppresses the queued procedure's response. */
		if (atomic_test_bit(&acs_conn->state, ACS_STATE_ABORT_REQUESTED)) {
			reply->holds_cp_lock = false;
			acs_reply_free(reply);
			continue;
		}

		acs_cp_execute(reply);
	}
}

void acs_cp_exec_queue_init(struct bt_acs_conn *acs_conn)
{
	k_fifo_init(&acs_conn->cp_exec_fifo);
	k_work_init(&acs_conn->cp_exec_work, acs_cp_exec_work_handler);
}

/* Update the reply flag and connection flag together. */
bool acs_cp_lock(struct acs_reply *reply)
{
	if (atomic_test_and_set_bit(&reply->acs_conn->state, ACS_STATE_CP_LOCKED)) {
		return false;
	}

	reply->holds_cp_lock = true;
	return true;
}

void acs_cp_unlock(struct acs_reply *reply)
{
	if (!reply->holds_cp_lock) {
		return;
	}

	reply->holds_cp_lock = false;
	atomic_clear_bit(&reply->acs_conn->state, ACS_STATE_CP_SERVER_TX);
	atomic_clear_bit(&reply->acs_conn->state, ACS_STATE_CP_LOCKED);
}

/* Enforce one Control Point procedure per connection (§4.4.3). */
static uint8_t acs_cp_submit(struct acs_reply *reply)
{
	struct bt_acs_conn *acs_conn = reply->acs_conn;

	if (!acs_cp_lock(reply)) {
		LOG_WRN("ACS CP procedure already in progress");
		acs_reply_free(reply);
		return BT_ATT_ERR_PROCEDURE_IN_PROGRESS;
	}

	k_fifo_put(&acs_conn->cp_exec_fifo, reply);
	k_work_submit_to_queue(acs_get_wq(), &acs_conn->cp_exec_work);
	return BT_ATT_ERR_SUCCESS;
}

uint8_t acs_cp_queue_plain(struct bt_acs_conn *acs_conn)
{
	const struct net_buf *request = acs_conn->cp_rx.buf;
	const struct bt_acs_restriction_map *rmap = acs_rmap_get_active_map();
	struct acs_reply *reply;
	uint8_t att_err;

	/* The response is an ACS Control Point indication (ACS 1.0 §4.4.5). */
	if (acs_cp_ccc_check(acs_conn->conn) != 0) {
		LOG_WRN("CP indications not enabled by client");
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return BT_ATT_ERR_CCC_IMPROPER_CONF;
	}

	__ASSERT_NO_MSG(request->len > 0U);

	if (request->len > ACS_MESSAGE_MAX_SIZE) {
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return BT_ATT_ERR_INVALID_ATTRIBUTE_LEN;
	}

	/*
	 * Check if the control point request requires protection according to the active
	 * restriction map (§4.4.3). (If the active restriction map marks that opcode as
	 * protected, do not execute it on ACS Control Point; require ACS Data In instead.)
	 */
	if (acs_cp_plain_requires_data_in(rmap, request->data, request->len)) {
		LOG_WRN("plain CP opcode 0x%02x requires Data In", request->data[0]);
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return BT_ATT_ERR_AUTHORIZATION;
	}

	/*
	 * Abort bypasses the Control Point lock (§4.4.5) for eg: to abort a currently running
	 * procedure.
	 */
	if (request->data[0] == BT_ACS_CP_OPCODE_ABORT) {
		acs_abort_request(acs_conn);
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return BT_ATT_ERR_SUCCESS;
	}

	reply = acs_reply_alloc(acs_conn);
	if (reply == NULL) {
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return BT_ATT_ERR_INSUFFICIENT_RESOURCES;
	}
	reply->channel = ACS_REPLY_CP;
	reply->request = net_buf_take(&acs_conn->cp_rx.buf);

	att_err = acs_cp_submit(reply);
	if (att_err != BT_ATT_ERR_SUCCESS) {
		acs_seg_rx_reset(&acs_conn->cp_rx);
	}

	return att_err;
}

uint8_t acs_cp_queue_protected(struct acs_reply *reply)
{
	struct net_buf_simple operand;
	uint8_t att_err;
	uint8_t opcode;

	__ASSERT_NO_MSG(reply->request != NULL);

	net_buf_simple_init_with_data(&operand, reply->request->data, reply->request->len);

	if (operand.len == 0U) {
		LOG_WRN("empty payload, no opcode");
		acs_reply_free(reply);
		return BT_ATT_ERR_INVALID_ATTRIBUTE_LEN;
	}

	opcode = net_buf_simple_pull_u8(&operand);

	/* Checked here, not on the workqueue: §4.3.2 wants an ATT error. */
	att_err = acs_cp_data_in_source_att_err(reply->isc_id, acs_rmap_get_active_map(), opcode,
						&operand);
	if (att_err != BT_ATT_ERR_SUCCESS) {
		acs_reply_free(reply);
		return att_err;
	}

	/* Abort bypasses the Control Point lock (§4.4.5). */
	if (opcode == BT_ACS_CP_OPCODE_ABORT) {
		acs_abort_request(reply->acs_conn);
		acs_reply_free(reply);
		return BT_ATT_ERR_SUCCESS;
	}

	return acs_cp_submit(reply);
}
