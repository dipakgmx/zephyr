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
#include "acs_rhandle.h"
#include "acs_rmap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
static void data_tx_drain_work(struct k_work *work);
static void don_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
			void *user_data);
static void doi_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
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

static int data_tx_encrypt_in_place(struct acs_reply *reply, struct net_buf *buf)
{
	struct bt_acs_conn *acs_conn = reply->conn;
	uint16_t isc_id = reply->isc_id;
	struct bt_acs_key_desc_runtime *key_desc_runtime = reply->key_runtime;
	const struct bt_acs_key_desc_record *key_desc;
	uint16_t plain_len;
	uint16_t cipher_len = 0;
	uint64_t tx_counter;
	uint8_t nonce_var_size;
	uint8_t auth_tag_size;
	uint16_t crypto_headroom;
	uint8_t *hdr;
	int err;

	__ASSERT_NO_MSG(buf->len > 0);

	/*
	 * Resolved when the request was decrypted; the key may have been
	 * invalidated while the reply was queued.
	 */
	if (!key_desc_runtime || key_desc_runtime->psa_key_id == 0U) {
		LOG_ERR("no key runtime for isc_id 0x%04x", isc_id);
		return -EACCES;
	}
	key_desc = key_desc_runtime->key_desc;

	nonce_var_size = acs_key_desc_nonce_var_size(key_desc);
	auth_tag_size = acs_key_desc_auth_tag_size(key_desc);
	crypto_headroom = nonce_var_size + sizeof(uint16_t) + ACS_SEG_HDR_SIZE;
	if (net_buf_headroom(buf) < crypto_headroom) {
		LOG_ERR("insufficient headroom for encrypt (%zu < %u)", net_buf_headroom(buf),
			crypto_headroom);
		return -ENOMEM;
	}

	if (net_buf_tailroom(buf) < auth_tag_size) {
		LOG_ERR("insufficient tailroom for auth tag (%zu < %u)", net_buf_tailroom(buf),
			auth_tag_size);
		return -ENOMEM;
	}

	plain_len = buf->len;

	tx_counter = key_desc_runtime->tx_nonce_counter;

	/* acs_crypto_encrypt() consumes and produces wire-order buffers. */
	err = acs_crypto_encrypt(key_desc_runtime, buf->data, plain_len, &cipher_len);
	if (err) {
		if (err == -ENOSPC) {
			LOG_WRN("nonce exhausted on encrypt, invalidating security");
			bt_acs_invalidate_security(acs_conn->conn);
		}
		LOG_ERR("data encryption failed for isc_id 0x%04x: %d", isc_id, err);
		return err;
	}

	__ASSERT_NO_MSG(cipher_len == plain_len + auth_tag_size);

	net_buf_add(buf, auth_tag_size);

	hdr = net_buf_push(buf, nonce_var_size + sizeof(uint16_t));
	sys_put_le16(isc_id, hdr);
	sys_put_le(&hdr[sizeof(uint16_t)], &tx_counter, nonce_var_size);

	return 0;
}

/* Send queued replies one at a time on a Data Out channel. */
static void data_tx_drain(struct acs_tx_channel *chan)
{
	struct bt_acs_conn *acs_conn = chan->conn;
	sys_snode_t *snode;
	struct acs_reply *reply;
	int err;

	if (acs_conn->conn == NULL) {
		return;
	}

	while (chan->active == NULL) {
		snode = k_fifo_get(&chan->fifo, K_NO_WAIT);
		if (snode == NULL) {
			return;
		}

		reply = CONTAINER_OF(snode, struct acs_reply, node);
		chan->active = reply;

		err = data_tx_encrypt_in_place(reply, reply->response);
		if (err) {
			LOG_WRN("%s encrypt failed: %d (handle 0x%04x)", chan->name, err,
				reply->resource_handle);
			chan->active = NULL;
			acs_reply_output_complete(reply, err);
			acs_reply_free(reply);
			continue;
		}

		err = acs_seg_tx_send(&chan->tx, acs_conn->conn, chan->attr, reply->response,
				      chan->done_cb, reply);
		if (!err) {
			return;
		}

		chan->active = NULL;
		LOG_WRN("%s send failed: %d (handle 0x%04x)", chan->name, err,
			reply->resource_handle);
		acs_reply_output_complete(reply, err);
		acs_reply_free(reply);
	}
}

static void data_tx_drain_work(struct k_work *work)
{
	struct acs_tx_channel *chan = CONTAINER_OF(work, struct acs_tx_channel, drain_work);

	data_tx_drain(chan);
}

static void don_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
			void *user_data)
{
	struct acs_reply *reply = user_data;
	struct acs_tx_channel *chan = &reply->conn->don;

	ARG_UNUSED(bt_conn);
	ARG_UNUSED(attr);

	chan->active = NULL;

	if (err) {
		LOG_WRN("DON notification failed for handle 0x%04x: %d", reply->resource_handle,
			err);
	}

	acs_reply_output_complete(reply, err);
	acs_reply_free(reply);

	k_work_submit_to_queue(acs_get_wq(), &chan->drain_work);
}

static void doi_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
			void *user_data)
{
	struct acs_reply *reply = user_data;
	struct acs_tx_channel *chan = &reply->conn->doi;

	ARG_UNUSED(bt_conn);
	ARG_UNUSED(attr);

	chan->active = NULL;

	if (err) {
		LOG_WRN("DOI indication failed for handle 0x%04x: %d", reply->resource_handle, err);
	}

	if (reply->aborted || err || !acs_reply_continue(reply)) {
		acs_reply_output_complete(reply, err);
		acs_reply_free(reply);
	}

	k_work_submit_to_queue(acs_get_wq(), &chan->drain_work);
}

static void acs_tx_channel_init(struct acs_tx_channel *chan, struct bt_acs_conn *conn,
				const struct bt_gatt_attr *attr, acs_seg_tx_completion_cb_t done_cb,
				const char *name)
{
	chan->conn = conn;
	chan->attr = attr;
	chan->done_cb = done_cb;
	chan->name = name;
	chan->active = NULL;

	k_fifo_init(&chan->fifo);
	k_work_init(&chan->drain_work, data_tx_drain_work);
}

void acs_data_out_init_conn(struct bt_acs_conn *conn)
{
	acs_tx_channel_init(&conn->don, conn, acs_attr_don(), don_tx_done, "DON");
	acs_tx_channel_init(&conn->doi, conn, acs_attr_doi(), doi_tx_done, "DOI");
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION) && IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
static int acs_reply_resolve_output_key(struct bt_acs_conn *acs_conn, uint16_t resource_handle,
					uint16_t rmap_opcode, uint16_t *isc_id,
					struct bt_acs_key_desc_runtime **key_runtime)
{
	const struct bt_acs_rmap_entry *entry;
	const struct bt_acs_rmap_protected *record;
	uint16_t op_isc_id = BT_ACS_ISC_ID_NONE;
	int err;

	if (!acs_security_switch_get() ||
	    !atomic_test_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED)) {
		return -EACCES;
	}

	err = acs_rmap_find_entry_by_resource_handle(acs_rmap_active(), resource_handle, &entry);
	if (err || entry->record->kind != BT_ACS_RMAP_RESOURCE_CHAR) {
		return err ? err : -ENOENT;
	}

	record = entry->record;
	for (uint8_t i = 0; i < record->num_ops; i++) {
		if (record->ops[i].opcode == rmap_opcode &&
		    record->ops[i].isc_id != BT_ACS_ISC_ID_NONE) {
			op_isc_id = record->ops[i].isc_id;
			break;
		}
	}

	if (op_isc_id == BT_ACS_ISC_ID_NONE) {
		return -ENOENT;
	}

	err = acs_resolve_isc_key(acs_conn, op_isc_id, key_runtime);
	if (err) {
		return err;
	}

	*isc_id = op_isc_id;
	return 0;
}

/* DON carries notifications, DOI indications; the ATT opcode follows. */
static uint16_t acs_channel_att_opcode(enum acs_reply_channel channel)
{
	return channel == ACS_REPLY_DON ? BT_ACS_RMAP_OP_ATT_NOTIFY : BT_ACS_RMAP_OP_ATT_INDICATE;
}

static int acs_send_protected_output(struct bt_acs_conn *acs_conn, uint16_t resource_handle,
				     const struct acs_output_spec *spec)
{
	struct bt_acs_key_desc_runtime *key_runtime;
	struct acs_reply *reply;
	struct net_buf *buf;
	uint16_t isc_id;
	uint8_t auth_tag_size;
	int err;

	if (acs_conn == NULL || acs_conn->conn == NULL) {
		return -ENOTCONN;
	}

	err = acs_reply_resolve_output_key(acs_conn, resource_handle,
					   acs_channel_att_opcode(spec->channel), &isc_id,
					   &key_runtime);
	if (err) {
		return err;
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
	reply->resource_handle = resource_handle;
	reply->isc_id = isc_id;
	reply->key_runtime = key_runtime;
	reply->output_func = spec->func;
	reply->output_user_data = spec->user_data;

	buf = acs_prepare_reply_buf(reply);
	if (buf == NULL) {
		acs_reply_free(reply);
		return -ENOMEM;
	}

	/* Check the payload before adding it to the channel queue. */
	auth_tag_size = acs_key_desc_auth_tag_size(key_runtime->key_desc);
	if (net_buf_tailroom(buf) < (size_t)spec->len + auth_tag_size) {
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
	uint16_t resource_handle;
	const struct acs_output_spec *spec;
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

	ret = acs_send_protected_output(acs_conn, ctx->resource_handle, ctx->spec);
	if (ret == 0) {
		ctx->err = 0;
	} else if (ctx->err == -ENOTCONN) {
		ctx->err = ret;
	}
}

static int acs_send_protected_output_resolved(struct bt_conn *conn, uint16_t resource_handle,
					      const struct acs_output_spec *spec)
{
	struct output_fanout_ctx ctx = {
		.resource_handle = resource_handle,
		.spec = spec,
		.err = -ENOTCONN,
	};

	if (conn != NULL) {
		return acs_send_protected_output(acs_conn_lookup(conn), resource_handle, spec);
	}

	bt_conn_foreach(BT_CONN_TYPE_LE, send_output_to_conn, &ctx);

	return ctx.err;
}

/* Resolve by UUID when set, else by attr (mirrors bt_gatt_indicate()). */
static int acs_send_protected_output_lookup(struct bt_conn *conn, const struct bt_uuid *char_uuid,
					    const struct bt_gatt_attr *attr,
					    const struct acs_output_spec *spec)
{
	uint16_t resource_handle;
	uint16_t attr_handle;

	if ((char_uuid == NULL && attr == NULL) || (spec->data == NULL && spec->len > 0U)) {
		return -EINVAL;
	}

	if (char_uuid != NULL) {
		int err = acs_rhandle_find_char_attr_handles(char_uuid, &resource_handle,
							     &attr_handle);

		if (err) {
			return err;
		}
	} else {
		attr_handle = bt_gatt_attr_value_handle(attr);
		if (attr_handle == 0U) {
			attr_handle = bt_gatt_attr_get_handle(attr);
		}

		resource_handle = acs_rhandle_find_resource_handle(attr_handle);
		if (resource_handle == 0U) {
			return -ENOENT;
		}
	}

	return acs_send_protected_output_resolved(conn, resource_handle, spec);
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
