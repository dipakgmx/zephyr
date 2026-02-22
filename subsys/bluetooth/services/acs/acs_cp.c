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
#include "acs_descs.h"
#include "acs_crypto.h"
#include "acs_key_exchange.h"
#include "acs_rmap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static struct acs_cp_result acs_cp_handle_att_mtu(struct acs_reply *reply,
						  struct net_buf_simple *payload);

/* ACS Control Point opcode dispatch entry. A missing entry means Opcode Not Supported. */
struct acs_cp_opcode_info {
	uint8_t opcode;            /* Control Point opcode */
	uint16_t min_operand_size; /* Shortest valid operand */
	uint8_t rsp_opcode;        /* Response opcode, 0 for generic Response Code */
	struct acs_cp_result (*handler)(struct acs_reply *reply, struct net_buf_simple *payload);
};

static const struct acs_cp_opcode_info acs_cp_opcodes[] = {
	{BT_ACS_CP_OPCODE_GET_FEATURE, 0U, BT_ACS_CP_OPCODE_ACS_FEATURE_RESPONSE,
	 acs_cp_handle_get_feature},
	{BT_ACS_CP_OPCODE_ATT_MTU, 0U, BT_ACS_CP_OPCODE_ATT_MTU_RESPONSE, acs_cp_handle_att_mtu},
#if IS_ENABLED(CONFIG_BT_ACS_HAS_NONCE_FIXED)
	{BT_ACS_CP_OPCODE_SET_CLIENT_NONCE_FIXED, sizeof(struct acs_cp_set_client_nonce_fixed_req),
	 0U, acs_cp_handle_set_client_nonce_fixed},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	{BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP,
	 sizeof(struct acs_cp_activate_restriction_map_req), 0U,
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
	{BT_ACS_CP_OPCODE_START_KEY_EXCHANGE, sizeof(struct acs_cp_start_key_exchange_req), 0U,
	 acs_cp_kex_start},
	{BT_ACS_CP_OPCODE_GET_KEY_DESCRIPTOR, sizeof(struct acs_cp_get_key_descriptor_req),
	 BT_ACS_CP_OPCODE_KEY_DESCRIPTOR_RESPONSE, acs_cp_handle_get_key_descriptor},
	{BT_ACS_CP_OPCODE_GET_CURRENT_KEY_LIST, 0U, BT_ACS_CP_OPCODE_CURRENT_KEY_LIST_RESPONSE,
	 acs_cp_kex_get_current_key_list},
	{BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF, sizeof(struct acs_kdf_req),
	 BT_ACS_CP_OPCODE_KEY_EXCHANGE_KDF_RESPONSE, acs_cp_kex_exchange_kdf},
	{BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH, ACS_ECDH_PUBKEY_MIN_OPERAND,
	 BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_RESPONSE, acs_cp_kex_exchange_ecdh},
	{BT_ACS_CP_OPCODE_ECDH_CONFIRM_CODE, sizeof(struct acs_cp_ecdh_confirm_code_req),
	 BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_CODE_RESPONSE,
	 acs_cp_kex_ecdh_confirm_code},
	{BT_ACS_CP_OPCODE_ECDH_CONFIRM_RAND, sizeof(struct acs_cp_ecdh_confirm_rand_req),
	 BT_ACS_CP_OPCODE_KEY_EXCHANGE_ECDH_CONFIRMATION_RANDOM_NUMBER_RESPONSE,
	 acs_cp_kex_ecdh_confirm_rand},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	{BT_ACS_CP_OPCODE_GET_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR,
	 sizeof(struct acs_cp_get_isc_descriptor_req),
	 BT_ACS_CP_OPCODE_INFORMATION_SECURITY_CONFIGURATION_DESCRIPTOR_RESPONSE,
	 acs_cp_handle_get_isc_descriptor},
	{BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS, 0U,
	 BT_ACS_CP_OPCODE_RESTRICTION_MAP_DESCRIPTOR_RESPONSE, acs_cp_all_active_get},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	{BT_ACS_CP_OPCODE_INVALIDATE_KEY, sizeof(struct acs_cp_invalidate_key_req), 0U,
	 acs_sec_mgmt_invalidate_key},
	{BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY, 0U, 0U, acs_sec_mgmt_invalidate_all},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_SET_SECURITY_CONTROLS_SWITCH)
	{BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH, sizeof(struct acs_cp_sec_switch_req), 0U,
	 acs_sec_mgmt_set_security_switch},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_INITIATE_PAIRING)
	{BT_ACS_CP_OPCODE_INITIATE_PAIRING, 0U, 0U, acs_sec_mgmt_initiate_pairing},
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

int acs_cp_rsp_status(struct acs_reply *reply, uint8_t req_opcode, uint8_t code)
{
	struct net_buf *buf = acs_prepare_reply_buf(reply);

	if (buf == NULL) {
		reply->step = ACS_REPLY_DONE;
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_RESPONSE_CODE);
	net_buf_add_u8(buf, req_opcode);
	net_buf_add_u8(buf, code);

	return acs_reply_submit(reply);
}

/* ATT_MTU excludes the ATT opcode and handle (Table 4.7). */
static uint16_t acs_cp_att_mtu_value(struct bt_conn *conn)
{
	return bt_gatt_get_mtu(conn) - ACS_SEG_ATT_HDR_SIZE;
}

/* Handle the Get ATT MTU procedure (§4.4.3.19). */
static struct acs_cp_result acs_cp_handle_att_mtu(struct acs_reply *reply,
						  struct net_buf_simple *payload)
{
	ARG_UNUSED(payload);

	net_buf_add_le16(reply->response, acs_cp_att_mtu_value(reply->conn->conn));
	return acs_cp_reply();
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
	if (!acs_cp_lock(acs_conn, reply)) {
		acs_reply_free(reply);
		return;
	}

	atomic_set_bit(&acs_conn->state, ACS_STATE_CP_SERVER_TX);

	buf = acs_prepare_reply_buf(reply);
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

/* Prepare the procedure response, returning a Response Code if allocation fails. */
static int acs_cp_prepare_response(struct acs_reply *reply, const struct acs_cp_opcode_info *info)
{
	struct net_buf *rsp_buf;

	if (!info || info->rsp_opcode == 0U) {
		return 0;
	}

	rsp_buf = acs_prepare_reply_buf(reply);
	if (rsp_buf == NULL) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	net_buf_add_u8(rsp_buf, info->rsp_opcode);
	return 0;
}

/* Return true when the handler result reports a failure. */
static bool acs_cp_result_is_failure(struct acs_cp_result res)
{
	switch (res.action) {
	case ACS_CP_SEND_REPLY:
		return false;
	case ACS_CP_SEND_STATUS:
		return res.status != BT_ACS_CP_RESPONSE_SUCCESS;
	}

	CODE_UNREACHABLE;
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
static struct acs_cp_protection_state acs_cp_rmap_target_state(const struct net_buf_simple *operand)
{
	struct acs_cp_protection_state state = {
		.protection = ACS_CP_UNDECIDABLE,
		.isc_id = BT_ACS_ISC_ID_NONE,
	};
	const struct bt_acs_rmap_runtime *runtime;
	uint16_t map_id;

	if (operand->len < sizeof(uint16_t)) {
		return state;
	}

	map_id = sys_get_le16(operand->data);
	runtime = acs_rmap_lookup(map_id);
	if (runtime == NULL) {
		return state;
	}

	state.isc_id = runtime->map->map_isc_id;
	if (state.isc_id != BT_ACS_ISC_ID_NONE) {
		state.protection = ACS_CP_PROTECTED;
	} else {
		state.protection = ACS_CP_UNPROTECTED;
	}

	return state;
}

static struct acs_cp_protection_state
acs_cp_opcode_protection(const struct bt_acs_rmap_runtime *active_rmap, uint8_t opcode,
			 const struct net_buf_simple *operand)
{
	struct acs_cp_protection_state state = {
		.protection = ACS_CP_UNPROTECTED,
		.isc_id = BT_ACS_ISC_ID_NONE,
	};

	switch (opcode) {
	case BT_ACS_CP_OPCODE_GET_ALL_ACTIVE_DESCRIPTORS: {
		uint16_t descriptor_isc;

		if (acs_rmap_descriptor_protection(active_rmap, &descriptor_isc)) {
			state.protection = ACS_CP_PROTECTED;
			state.isc_id = descriptor_isc;
		}
		return state;
	}
	case BT_ACS_CP_OPCODE_GET_RESTRICTION_MAP_DESCRIPTOR:
	case BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP:
		return acs_cp_rmap_target_state(operand);
	default:
		state.isc_id = acs_rmap_cp_opcode_isc(active_rmap, acs_cp_attr_handle(), opcode);
		if (state.isc_id != BT_ACS_ISC_ID_NONE) {
			state.protection = ACS_CP_PROTECTED;
		}
		return state;
	}
}

bool acs_cp_plain_requires_data_in(const struct bt_acs_rmap_runtime *active_rmap,
				   const uint8_t *payload, uint16_t payload_len)
{
	struct acs_cp_protection_state state;
	struct net_buf_simple operand;
	uint8_t opcode;

	if (active_rmap == NULL || payload_len == 0U) {
		return false;
	}

	opcode = payload[0];

	/* Source validation selects the required Response Code (§4.4.3.4). */
	if (opcode == BT_ACS_CP_OPCODE_ACTIVATE_RESTRICTION_MAP) {
		return false;
	}

	net_buf_simple_init_with_data(&operand, (void *)&payload[1], payload_len - 1U);
	state = acs_cp_opcode_protection(active_rmap, opcode, &operand);

	return state.protection == ACS_CP_PROTECTED;
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

/* Validate the source channel and ISC; the result sign selects the response path. */
static int acs_cp_validate_source(const struct acs_frame *frame,
				  const struct bt_acs_rmap_runtime *active_rmap, uint8_t opcode,
				  const struct net_buf_simple *operand)
{
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	bool from_data_in = (frame->source_channel == ACS_SRC_DATA_IN);
	struct acs_cp_protection_state state;

	if (active_rmap == NULL) {
		LOG_ERR("CP opcode 0x%02x has no active restriction map", opcode);
		return from_data_in ? ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG
				    : BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}

	state = acs_cp_opcode_protection(active_rmap, opcode, operand);
	if (state.protection == ACS_CP_UNDECIDABLE) {
		return 0;
	}

	if (from_data_in && state.protection == ACS_CP_PROTECTED &&
	    state.isc_id != BT_ACS_ISC_ID_NONE && frame->isc_id != state.isc_id) {
		LOG_WRN("CP opcode 0x%02x wrong ISC 0x%04x (expected 0x%04x)", opcode,
			frame->isc_id, state.isc_id);
		return ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG;
	}

	/* Both codes generalized: Section 4.3.2 and Section 4.4.3.4 state them narrowly. */
	if (from_data_in && state.protection != ACS_CP_PROTECTED) {
		return ACS_DATA_ERR_RESOURCE_NOT_PROTECTED;
	}
	if (!from_data_in && state.protection == ACS_CP_PROTECTED) {
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	}
	return 0;
#else
	ARG_UNUSED(frame);
	ARG_UNUSED(active_rmap);
	ARG_UNUSED(opcode);
	ARG_UNUSED(operand);
	return 0;
#endif
}

int acs_cp_execute(const struct acs_frame *frame, struct bt_acs_conn *acs_conn,
		   struct acs_reply *reply)
{
	struct net_buf_simple payload_simple;
	struct net_buf_simple *payload = &payload_simple;
	const struct acs_cp_opcode_info *info;
	uint8_t opcode;
	struct acs_cp_result res;
	int err;

	__ASSERT_NO_MSG(frame->payload != NULL);
	__ASSERT_NO_MSG(frame->payload_len > 0U);

	net_buf_simple_init_with_data(&payload_simple, (void *)frame->payload, frame->payload_len);

	opcode = net_buf_simple_pull_u8(payload);
	LOG_DBG("CP execute: opcode 0x%02x, operand len %u", opcode, payload->len);

	info = acs_cp_opcode_lookup(opcode);
	if (info == NULL) {
		LOG_WRN("CP execute: unsupported opcode 0x%02x", opcode);
		res = acs_cp_status(BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED);
		goto handle_result;
	}

	if (payload->len < info->min_operand_size) {
		LOG_WRN("CP execute: opcode 0x%02x operand too short (%u)", opcode, payload->len);
		res = acs_cp_status(BT_ACS_CP_RESPONSE_INVALID_OPERAND);
		goto handle_result;
	}

	/* Data In was validated synchronously before it entered the workqueue. */
	if (frame->source_channel == ACS_SRC_CP) {
		err = acs_cp_validate_source(frame, reply->rmap, opcode, payload);
		if (err < 0) {
			acs_reply_free(reply);
			return err;
		}
		if (err > 0) {
			res = acs_cp_status((uint8_t)err);
			goto handle_result;
		}
	}

	if (!acs_kex_step_allowed(acs_conn, opcode, payload)) {
		LOG_WRN("CP execute: KEX opcode 0x%02x out of order or wrong Key_ID", opcode);
		res = acs_cp_status(BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE);
	} else {
		err = acs_cp_prepare_response(reply, info);
		if (err == 0) {
			res = info->handler(reply, payload);
		} else {
			res = acs_cp_status((uint8_t)err);
		}
	}

handle_result:
	/*
	 * The dispatcher owns KEX abort-on-failure, so no individual KEX handler
	 * repeats it.
	 */
	if (acs_cp_result_is_failure(res)) {
		acs_kex_abort_failed_procedure(acs_conn, opcode);
	}

	/* The handler has consumed the request buffer. */
	acs_buf_free(reply->request);
	reply->request = NULL;

	switch (res.action) {
	case ACS_CP_SEND_REPLY:
		err = acs_reply_submit(reply);
		break;
	case ACS_CP_SEND_STATUS:
	default:
		err = acs_cp_rsp_status(reply, opcode, res.status);
		break;
	}

	if (err) {
		acs_kex_abort_failed_procedure(acs_conn, opcode);

		/* The ATT write is complete; report this failure with a Response Code. */
		if (acs_cp_rsp_status(reply, opcode, BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED) ==
		    0) {
			return 0;
		}

		acs_reply_free(reply);
	}
	return err;
}

/* Execute accepted procedures outside the cooperative BT RX thread. */
static void acs_cp_exec_work_handler(struct k_work *work)
{
	struct bt_acs_conn *acs_conn = CONTAINER_OF(work, struct bt_acs_conn, cp_exec_work);
	sys_snode_t *snode;

	while ((snode = k_fifo_get(&acs_conn->cp_exec_fifo, K_NO_WAIT)) != NULL) {
		struct acs_reply *reply = CONTAINER_OF(snode, struct acs_reply, node);
		struct acs_frame frame;

		if (reply->conn->conn == NULL) {
			/* Disconnected while queued. */
			acs_reply_free(reply);
			continue;
		}

		/* Abort holds the lock and suppresses the queued procedure's response. */
		if (atomic_test_bit(&acs_conn->state, ACS_STATE_ABORT_REQUESTED)) {
			reply->holds_cp_lock = false;
			acs_reply_free(reply);
			continue;
		}

		frame = acs_frame_from_reply(reply);
		acs_cp_execute(&frame, reply->conn, reply);
	}
}

void acs_cp_exec_queue_init(struct bt_acs_conn *acs_conn)
{
	k_fifo_init(&acs_conn->cp_exec_fifo);
	k_work_init(&acs_conn->cp_exec_work, acs_cp_exec_work_handler);
}

/* Update the reply flag and connection flag together. */
bool acs_cp_lock(struct bt_acs_conn *conn, struct acs_reply *reply)
{
	if (atomic_test_and_set_bit(&conn->state, ACS_STATE_CP_LOCKED)) {
		return false;
	}

	reply->holds_cp_lock = true;
	return true;
}

void acs_cp_unlock(struct bt_acs_conn *conn, struct acs_reply *reply)
{
	if (!reply->holds_cp_lock) {
		return;
	}

	reply->holds_cp_lock = false;
	atomic_clear_bit(&conn->state, ACS_STATE_CP_SERVER_TX);
	atomic_clear_bit(&conn->state, ACS_STATE_CP_LOCKED);
}

/* Enforce one Control Point procedure per connection (§4.4.3). */
static int acs_cp_submit(struct bt_acs_conn *acs_conn, struct acs_reply *reply)
{
	if (!acs_cp_lock(acs_conn, reply)) {
		LOG_WRN("ACS CP procedure already in progress");
		acs_reply_free(reply);
		return ACS_CP_RESULT_PROCEDURE_IN_PROGRESS;
	}

	k_fifo_put(&acs_conn->cp_exec_fifo, reply);
	k_work_submit_to_queue(acs_get_wq(), &acs_conn->cp_exec_work);
	return 0;
}

int acs_cp_queue_plain(struct bt_acs_conn *acs_conn)
{
	struct acs_frame frame = acs_frame_from_cp_rx(acs_conn->cp_rx.buf);
	const struct bt_acs_rmap_runtime *rmap = acs_rmap_active();
	struct acs_reply *reply;
	int err;

	if (frame.payload_len == 0U) {
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return ACS_CP_RESULT_INVALID_LENGTH;
	}

	/* Abort bypasses the Control Point lock (§4.4.5). */
	if (frame.payload[0] == BT_ACS_CP_OPCODE_ABORT) {
		acs_abort_request(acs_conn);
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return 0;
	}

	if (acs_cp_plain_requires_data_in(rmap, frame.payload, frame.payload_len)) {
		LOG_WRN("plain CP opcode 0x%02x requires Data In", frame.payload[0]);
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return ACS_CP_RESULT_INSUFFICIENT_AUTH;
	}

	reply = acs_reply_alloc(acs_conn);
	if (reply == NULL) {
		acs_seg_rx_reset(&acs_conn->cp_rx);
		return ACS_DATA_ERR_NO_RESOURCES;
	}
	reply->channel = ACS_REPLY_CP;
	reply->rmap = rmap;
	acs_reply_take_request_buf(reply, &acs_conn->cp_rx);

	err = acs_cp_submit(acs_conn, reply);
	if (err) {
		acs_seg_rx_reset(&acs_conn->cp_rx);
	}

	return err;
}

int acs_cp_queue_protected(const struct acs_frame *frame, struct bt_acs_conn *acs_conn,
			   struct acs_reply *reply)
{
	struct net_buf_simple payload_simple;
	struct net_buf_simple *payload = &payload_simple;
	uint8_t opcode;
	int err;

	__ASSERT_NO_MSG(frame->payload != NULL);

	net_buf_simple_init_with_data(&payload_simple, (void *)frame->payload, frame->payload_len);

	if (payload->len == 0U) {
		LOG_WRN("empty payload, no opcode");
		acs_reply_free(reply);
		return ACS_CP_RESULT_INVALID_LENGTH;
	}

	opcode = net_buf_simple_pull_u8(payload);

	reply->rmap = acs_rmap_active();

	/* Checked here, not on the workqueue: §4.3.2 wants an ATT error. */
	err = acs_cp_validate_source(frame, reply->rmap, opcode, payload);
	if (err < 0) {
		acs_reply_free(reply);
		return err;
	}

	return acs_cp_submit(acs_conn, reply);
}
