/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Protected output over Data Out Notify and Data Out Indicate (§4.3). */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_internal.h"
#include "acs_reply.h"
#include "acs_crypto.h"
#include "acs_isc.h"
#include "acs_data_out.h"
#include "acs_rmap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
static void data_tx_drain_work(struct k_work *work);
static void data_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
			 void *user_data);
#endif

/* Service-initiated protected output. */
struct acs_output_spec {
	const void *data;
	uint16_t len;
	enum acs_reply_channel channel;
	bt_acs_output_func_t func;
	void *user_data;
};

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
static void acs_reply_output_complete(struct acs_reply *reply, int err)
{
	if (reply->output_func != NULL) {
		reply->output_func(reply->conn->conn, err, reply->output_user_data);
	}
}

/*
 * Encrypt one message in place and prepend ISC_ID || Nonce_Var (§4.3.2). The
 * reply's key may have been invalidated while it was queued; the crypto layer
 * reports that as -EACCES.
 */
static int data_tx_encrypt_in_place(struct acs_reply *reply, struct net_buf *buf)
{
	struct acs_sec_alg *alg = reply->alg;
	uint8_t nonce_var_size = acs_key_desc_nonce_var_size(alg->key_desc);
	uint8_t auth_tag_size = acs_key_desc_auth_tag_size(alg->key_desc);
	uint64_t tx_counter = alg->tx_nonce_counter;
	uint16_t cipher_len;
	uint8_t *hdr;
	int err;

	/* acs_reply_add_message() reserves ACS_CRYPTO_HEADROOM on every Data Out message. */
	__ASSERT_NO_MSG(net_buf_headroom(buf) >= sizeof(uint16_t) + nonce_var_size);

	/* A Control Point response can fill the buffer and leave no room for the tag. */
	if (net_buf_tailroom(buf) < auth_tag_size) {
		LOG_ERR("insufficient tailroom for auth tag (%zu < %u)", net_buf_tailroom(buf),
			auth_tag_size);
		return -ENOMEM;
	}

	err = acs_crypto_encrypt(alg, buf->data, buf->len, &cipher_len);
	if (err == -ENOSPC) {
		LOG_WRN("nonce exhausted on encrypt, invalidating security");
		(void)bt_acs_invalidate_security(reply->conn->conn);
	}
	if (err) {
		return err;
	}

	net_buf_add(buf, cipher_len - buf->len);

	hdr = net_buf_push(buf, sizeof(uint16_t) + nonce_var_size);
	sys_put_le16(reply->isc_id, hdr);
	sys_put_le(&hdr[sizeof(uint16_t)], &tx_counter, nonce_var_size);

	return 0;
}

/* Encrypt the reply's current message and send it on chan. */
static int data_tx_send_message(struct acs_tx_channel *chan, struct acs_reply *reply)
{
	int err = data_tx_encrypt_in_place(reply, reply->tx_msg);

	if (err == 0) {
		err = acs_seg_tx_send(&chan->tx, chan->conn->conn, chan->attr, reply->tx_msg,
				      data_tx_done, reply);
	}

	return err;
}

/* Finish a reply on chan and let the channel take the next one. */
static void data_tx_finish(struct acs_tx_channel *chan, struct acs_reply *reply, int err)
{
	if (err) {
		LOG_WRN("%s reply for handle 0x%04x failed: %d", chan->name,
			reply->resource_handle, err);
	}

	chan->active = NULL;
	acs_reply_output_complete(reply, err);
	acs_reply_free(reply);
	k_work_submit_to_queue(acs_get_wq(), &chan->drain_work);
}

/* Send queued replies one at a time on a Data Out channel. */
static void data_tx_drain(struct acs_tx_channel *chan)
{
	struct acs_reply *reply;
	sys_snode_t *snode;
	int err;

	if (chan->conn->conn == NULL || chan->active != NULL) {
		return;
	}

	snode = k_fifo_get(&chan->fifo, K_NO_WAIT);
	if (snode == NULL) {
		return;
	}

	reply = CONTAINER_OF(snode, struct acs_reply, node);
	chan->active = reply;

	err = data_tx_send_message(chan, reply);
	if (err) {
		data_tx_finish(chan, reply, err);
	}
}

static void data_tx_drain_work(struct k_work *work)
{
	struct acs_tx_channel *chan = CONTAINER_OF(work, struct acs_tx_channel, drain_work);

	data_tx_drain(chan);
}

/* A message went out on Data Out: send the reply's next one, or finish the reply. */
static void data_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
			 void *user_data)
{
	struct acs_reply *reply = user_data;
	struct acs_tx_channel *chan =
		(reply->channel == ACS_REPLY_DON) ? &reply->conn->don : &reply->conn->doi;

	ARG_UNUSED(bt_conn);
	ARG_UNUSED(attr);

	if (err == 0 && !reply->aborted) {
		if (acs_reply_next_message(reply) == NULL) {
			acs_reply_delivered(reply);
		} else {
			err = data_tx_send_message(chan, reply);
			if (err == 0) {
				return;
			}
		}
	}

	data_tx_finish(chan, reply, err);
}

static void acs_tx_channel_init(struct acs_tx_channel *chan, struct bt_acs_conn *conn,
				const struct bt_gatt_attr *attr, const char *name)
{
	chan->conn = conn;
	chan->attr = attr;
	chan->name = name;
	chan->active = NULL;

	k_fifo_init(&chan->fifo);
	k_work_init(&chan->drain_work, data_tx_drain_work);
}

void acs_data_out_init_conn(struct bt_acs_conn *conn)
{
	acs_tx_channel_init(&conn->don, conn, acs_attr_don(), "DON");
	acs_tx_channel_init(&conn->doi, conn, acs_attr_doi(), "DOI");
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION) && IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
/* ISC governing att_opcode on a protected characteristic, or BT_ACS_ISC_ID_NONE. */
static uint16_t output_isc_id(const struct bt_acs_rmap_resource *resource, uint16_t att_opcode)
{
	const struct bt_acs_rmap_op_isc *op;

	if (resource->kind != BT_ACS_RMAP_RESOURCE_CHAR) {
		return BT_ACS_ISC_ID_NONE;
	}

	op = acs_rmap_find_op(resource, att_opcode);

	return op != NULL ? op->isc_id : BT_ACS_ISC_ID_NONE;
}

static int acs_send_protected_output_to_conn(struct bt_acs_conn *acs_conn,
					     const struct bt_acs_rmap_resource *resource, uint16_t isc_id,
					     const struct acs_output_spec *spec)
{
	struct acs_sec_alg *alg;
	struct acs_reply *reply;
	struct net_buf *buf;
	int err;

	if (acs_conn == NULL) {
		return -ENOTCONN;
	}

	if (!acs_security_switch_get() ||
	    !atomic_test_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED)) {
		return -EACCES;
	}

	alg = acs_isc_alg(acs_conn, isc_id);
	if (alg == NULL) {
		return -ENOENT;
	}
	if (!acs_sec_alg_ready(alg)) {
		return -EACCES;
	}

	err = (spec->channel == ACS_REPLY_DON) ? acs_don_ccc_check(acs_conn->conn)
					       : acs_doi_ccc_check(acs_conn->conn);
	if (err) {
		return -EACCES;
	}

	reply = acs_reply_alloc(acs_conn);
	if (reply == NULL) {
		return -ENOMEM;
	}

	reply->channel = spec->channel;
	reply->resource_handle = resource->resource_handle;
	reply->isc_id = isc_id;
	reply->alg = alg;
	reply->output_func = spec->func;
	reply->output_user_data = spec->user_data;

	buf = acs_reply_add_message(reply);
	if (buf == NULL) {
		acs_reply_free(reply);
		return -ENOMEM;
	}

	/* Check the payload before adding it to the channel queue. */
	if (net_buf_tailroom(buf) <
	    (size_t)spec->len + acs_key_desc_auth_tag_size(alg->key_desc)) {
		acs_reply_free(reply);
		return -EMSGSIZE;
	}

	if (spec->len > 0U) {
		net_buf_add_mem(buf, spec->data, spec->len);
	}

	err = acs_reply_submit(reply);
	if (err) {
		acs_reply_free(reply);
	}

	return err;
}

/* Result of sending protected output to every connected peer. */
struct output_fanout_ctx {
	const struct bt_acs_rmap_resource *resource;
	const struct acs_output_spec *spec;
	uint16_t isc_id;
	int err;
};

/* Every peer is attempted; the result is success if any accepted, else the first failure. */
static void send_output_to_conn(struct bt_conn *conn, void *data)
{
	struct output_fanout_ctx *ctx = data;
	struct bt_acs_conn *acs_conn = acs_conn_lookup(conn);
	int ret;

	if (acs_conn == NULL) {
		return;
	}

	ret = acs_send_protected_output_to_conn(acs_conn, ctx->resource, ctx->isc_id, ctx->spec);
	if (ret == 0) {
		ctx->err = 0;
	} else if (ctx->err == -ENOTCONN) {
		ctx->err = ret;
	}
}

/* Resolve by UUID when set, else by attr (mirrors bt_gatt_indicate()). */
static int acs_send_protected_output_lookup(struct bt_conn *conn, const struct bt_uuid *char_uuid,
					    const struct bt_gatt_attr *attr,
					    const struct acs_output_spec *spec)
{
	const struct bt_acs_restriction_map *active_map;
	const struct bt_acs_rmap_resource *resource;
	struct output_fanout_ctx ctx;
	uint16_t att_opcode;
	uint16_t isc_id;

	if ((char_uuid == NULL && attr == NULL) || (spec->data == NULL && spec->len > 0U)) {
		return -EINVAL;
	}

	active_map = acs_rmap_get_active_map();
	if (active_map == NULL) {
		return -ENOENT;
	}

	if (char_uuid != NULL) {
		resource = acs_rmap_resource_by_uuid(active_map, char_uuid);
	} else {
		uint16_t attr_handle = bt_gatt_attr_value_handle(attr);

		if (attr_handle == 0U) {
			attr_handle = bt_gatt_attr_get_handle(attr);
		}
		resource = acs_rmap_resource_by_attr_handle(active_map, attr_handle);
	}
	if (resource == NULL) {
		return -ENOENT;
	}

	/* The governing ISC is map policy, the same for every peer. */
	att_opcode = (spec->channel == ACS_REPLY_DON) ? BT_ACS_RMAP_OP_ATT_NOTIFY
						      : BT_ACS_RMAP_OP_ATT_INDICATE;
	isc_id = output_isc_id(resource, att_opcode);
	if (isc_id == BT_ACS_ISC_ID_NONE) {
		return -ENOENT;
	}

	if (conn != NULL) {
		return acs_send_protected_output_to_conn(acs_conn_lookup(conn), resource, isc_id,
							 spec);
	}

	ctx = (struct output_fanout_ctx){
		.resource = resource,
		.spec = spec,
		.isc_id = isc_id,
		.err = -ENOTCONN,
	};
	bt_conn_foreach(BT_CONN_TYPE_LE, send_output_to_conn, &ctx);

	return ctx.err;
}

int bt_acs_notify_cb(struct bt_conn *conn, struct bt_acs_notify_params *params)
{
	if (params == NULL) {
		LOG_ERR("params is NULL");
		return -EINVAL;
	}

	const struct acs_output_spec spec = {
		.data = params->data,
		.len = params->len,
		.channel = ACS_REPLY_DON,
		.func = params->func,
		.user_data = params->user_data,
	};

	return acs_send_protected_output_lookup(conn, params->uuid, params->attr, &spec);
}

int bt_acs_indicate(struct bt_conn *conn, struct bt_acs_indicate_params *params)
{
	if (params == NULL) {
		LOG_ERR("indication params is NULL");
		return -EINVAL;
	}

	const struct acs_output_spec spec = {
		.data = params->data,
		.len = params->len,
		.channel = ACS_REPLY_DOI,
		.func = params->func,
		.user_data = params->user_data,
	};

	return acs_send_protected_output_lookup(conn, params->uuid, params->attr, &spec);
}

int bt_acs_notify_uuid(struct bt_conn *conn, const struct bt_uuid *char_uuid, const void *data,
		       uint16_t len)
{
	const struct acs_output_spec spec = {
		.data = data,
		.len = len,
		.channel = ACS_REPLY_DON,
	};

	return acs_send_protected_output_lookup(conn, char_uuid, NULL, &spec);
}
#else
int bt_acs_notify_cb(struct bt_conn *conn, struct bt_acs_notify_params *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	return -ENOTSUP;
}

int bt_acs_indicate(struct bt_conn *conn, struct bt_acs_indicate_params *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	return -ENOTSUP;
}

int bt_acs_notify_uuid(struct bt_conn *conn, const struct bt_uuid *char_uuid, const void *data,
		       uint16_t len)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(char_uuid);
	ARG_UNUSED(data);
	ARG_UNUSED(len);

	return -ENOTSUP;
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION && CONFIG_BT_ACS_FEAT_AUTHORIZATION */
