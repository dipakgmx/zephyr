/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Protected output over Data Out Notify and Data Out Indicate (§4.3). */

#include <errno.h>
#include <string.h>

#include <zephyr/spinlock.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_internal.h"
#include "acs_reply.h"
#include "acs_crypto.h"
#include "acs_isc.h"
#include "acs_data_out.h"
#include "acs_rhandle.h"
#include "acs_rmap.h"
#include "host/gatt_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
static void data_tx_drain_work(struct k_work *work);
static void data_tx_done(int err, void *user_data);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/* Serializes _ref of indicate params between callers and the ACS workqueue. */
static struct k_spinlock ind_params_lock;

/* Drop a reference to params; the last one destroys it, as in gatt_indicate_rsp(). */
static void ind_params_unref(struct bt_gatt_indicate_params *params)
{
	k_spinlock_key_t key = k_spin_lock(&ind_params_lock);
	bool last;

	__ASSERT_NO_MSG(params->_ref > 0U);
	params->_ref--;
	last = (params->_ref == 0U);
	k_spin_unlock(&ind_params_lock, key);

	if (last && params->destroy != NULL) {
		params->destroy(params);
	}
}

void acs_data_out_complete(struct acs_reply *reply, int err)
{
	struct bt_gatt_indicate_params *ind = reply->ind;

	if (ind == NULL) {
		/* A notification callback reports delivery only, as on ordinary GATT. */
		if (err == 0 && reply->ntf_func != NULL) {
			reply->ntf_func(reply->acs_conn->conn, reply->ntf_user_data);
		}
		return;
	}

	if (ind->func != NULL) {
		ind->func(reply->acs_conn->conn, ind,
			  (err == 0) ? BT_ATT_ERR_SUCCESS : BT_ATT_ERR_UNLIKELY);
	}
	ind_params_unref(ind);
}

/*
 * Encrypt Protected_Resource_Handle || message in place and prepend
 * ISC_ID || Nonce_Var (§4.3.2). The reply's key may have been invalidated while
 * it was queued; the crypto layer reports that as -EACCES.
 */
static int data_tx_encrypt_in_place(struct acs_reply *reply, struct net_buf *buf)
{
	/* The ISC of every protected reply names a compiled-in algorithm. */
	struct acs_sec_alg *alg = acs_isc_alg(reply->acs_conn, reply->isc_id);
	uint8_t nonce_var_size = acs_key_desc_nonce_var_size(alg->key_desc);
	uint64_t tx_counter = alg->tx_nonce_counter;
	uint16_t cipher_len;
	uint8_t *hdr;
	int err;

	/* Release the MAC room acs_reply_add_message() kept out of the message. */
	__ASSERT_NO_MSG(net_buf_headroom(buf) == ACS_BUF_HEADROOM);
	buf->size += ACS_PROTECTED_TAG_SIZE;

	net_buf_push_le16(buf, reply->resource_handle);

	err = acs_crypto_encrypt(alg, buf->data, buf->len, &cipher_len);
	if (err == -ENOSPC) {
		LOG_WRN("nonce exhausted on encrypt, invalidating security");
		acs_security_invalidate(reply->acs_conn);
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
		err = acs_seg_tx_send(&chan->tx, reply->tx_msg, reply);
	}

	return err;
}

/* Finish a reply on chan and let the channel take the next one. */
static void data_tx_finish(struct acs_tx_channel *chan, struct acs_reply *reply, int err)
{
	if (err) {
		LOG_WRN("%s reply for handle 0x%04x failed: %d",
			(reply->channel == ACS_REPLY_DOI) ? "DOI" : "DON", reply->resource_handle,
			err);
	}

	chan->active = NULL;
	acs_reply_finish(reply, err);
	k_work_submit_to_queue(acs_get_wq(), &chan->drain_work);
}

/* Send queued replies one at a time on a Data Out channel. */
static void data_tx_drain(struct acs_tx_channel *chan)
{
	struct acs_reply *reply;
	sys_snode_t *snode;
	int err;

	if (chan->active != NULL) {
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
static void data_tx_done(int err, void *user_data)
{
	struct acs_reply *reply = user_data;
	struct bt_acs_conn *acs_conn = reply->acs_conn;
	struct acs_tx_channel *chan =
		(reply->channel == ACS_REPLY_DON) ? &acs_conn->don : &acs_conn->doi;

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

static void acs_tx_channel_init(struct acs_tx_channel *chan)
{
	k_fifo_init(&chan->fifo);
	k_work_init(&chan->drain_work, data_tx_drain_work);
}

void acs_data_out_init_conn(struct bt_acs_conn *acs_conn)
{
	acs_tx_channel_init(&acs_conn->don);
	acs_seg_tx_init(&acs_conn->don.tx, acs_conn->conn, acs_attr_don(), false, true,
			data_tx_done, NULL);

	acs_tx_channel_init(&acs_conn->doi);
	acs_seg_tx_init(&acs_conn->doi.tx, acs_conn->conn, acs_attr_doi(), true, true, data_tx_done,
			acs_reply_message_sent);
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

/* Authorization depends on authentication. */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
/* ATT bearer option of notification or indication parameters. */
#if defined(CONFIG_BT_EATT)
#define PARAMS_CHAN_OPT(params) ((params)->chan_opt)
#else
#define PARAMS_CHAN_OPT(params) BT_ATT_CHAN_OPT_NONE
#endif

/*
 * One notification or indication sent on ACS Data Out Notify or ACS Data Out
 * Indicate (§4.3.2) instead of plain ATT: bt_gatt_notify_params and
 * bt_gatt_indicate_params in one.
 */
struct data_out_params {
	const struct bt_gatt_attr *attr;     /* Characteristic value attribute */
	const void *data;                    /* Value */
	uint16_t len;                        /* Value length */
	uint16_t resource_handle;            /* Resource Handle of attr, 0 until resolved */
	uint16_t isc_id;                     /* ISC protecting the operation */
	enum acs_reply_channel channel;      /* ACS_REPLY_DON or ACS_REPLY_DOI */
	struct bt_gatt_indicate_params *ind; /* Indication, NULL for a notification */
	bt_gatt_complete_func_t ntf_func;    /* Notification callback */
	void *ntf_user_data;                 /* Notification callback data */
	int err;                             /* Result of data_out_send() to every conn */
};

/* Take a reference to params for one more queued reply, as notify_cb() does per peer. */
static void ind_params_ref(struct bt_gatt_indicate_params *params)
{
	k_spinlock_key_t key = k_spin_lock(&ind_params_lock);

	params->_ref++;
	k_spin_unlock(&ind_params_lock, key);
}

/*
 * ISC protecting operation on attr in the active map, else BT_ACS_ISC_ID_NONE.
 * resource_handle as for acs_rmap_attr_isc().
 */
static uint16_t data_out_isc(const struct bt_gatt_attr *attr, uint8_t operation,
			     uint16_t *resource_handle)
{
	/*
	 * ACS's own characteristics are the protected transport. Service Changed reaches
	 * change-unaware clients on plain ATT only (Core Vol 3 Part G 2.5.3).
	 */
	if (!acs_security_switch_get() || acs_attr_is_own(attr) ||
	    bt_uuid_cmp(attr->uuid, BT_UUID_GATT_SC) == 0) {
		return BT_ACS_ISC_ID_NONE;
	}

	return acs_rmap_attr_isc(acs_rmap_get_active_map(), attr, operation, resource_handle);
}

/* Return 0 when acs_conn may receive out now, else why it may not. */
static int data_out_conn_check(struct bt_acs_conn *acs_conn, const struct data_out_params *out)
{
	const struct acs_sec_alg *alg;
	struct bt_conn *conn;
	int err;

	conn = acs_conn->conn;

	/* Core Vol 3 Part G 2.5.3, applied as the host applies it to ordinary delivery. */
	if (IS_ENABLED(CONFIG_BT_GATT_ENFORCE_CHANGE_UNAWARE) &&
	    !bt_gatt_change_aware(conn, false)) {
		return -EAGAIN;
	}

	if (!atomic_test_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED)) {
		return -EACCES;
	}

	alg = acs_isc_alg(acs_conn, out->isc_id);
	if (alg == NULL) {
		return -ENOENT;
	}
	if (!acs_sec_alg_ready(alg)) {
		return -EACCES;
	}

	/* Delivery follows the Data Out subscription (§4.3.2). */
	err = (out->channel == ACS_REPLY_DON) ? acs_don_ccc_check(conn) : acs_doi_ccc_check(conn);
	if (err != 0) {
		return -EACCES;
	}

	/* The resource's link security still applies on Data Out (Core Vol 3 Part F 3.2.5). */
	if (bt_gatt_check_perm(conn, out->attr, BT_GATT_PERM_READ_ENCRYPT_MASK) != 0U) {
		return -EPERM;
	}

	return 0;
}

/* Queue out as a reply on the Data Out channel of acs_conn; the reply reports the outcome. */
static int data_out_enqueue(struct bt_acs_conn *acs_conn, struct data_out_params *out)
{
	struct acs_reply *reply;
	struct net_buf *buf;
	int err;

	/* Resolved for the first eligible conn only: it walks the GATT table. */
	if (out->resource_handle == 0U) {
		out->resource_handle = acs_rhandle_from_attr(out->attr);
	}

	reply = acs_reply_alloc(acs_conn);
	if (reply == NULL) {
		return -ENOMEM;
	}

	buf = acs_reply_add_message(reply);
	if (buf == NULL) {
		acs_reply_free(reply);
		return -ENOMEM;
	}

	if (out->len > 0U) {
		net_buf_add_mem(buf, out->data, out->len);
	}

	reply->channel = out->channel;
	reply->resource_handle = out->resource_handle;
	reply->isc_id = out->isc_id;
	reply->ind = out->ind;
	reply->ntf_func = out->ntf_func;
	reply->ntf_user_data = out->ntf_user_data;

	/* The reference must exist before the reply can complete on the ACS workqueue. */
	if (out->ind != NULL) {
		ind_params_ref(out->ind);
	}

	err = acs_reply_submit(reply);
	if (err != 0) {
		acs_reply_free(reply);
		if (out->ind != NULL) {
			/* The reference data_out_intercept() holds keeps params alive. */
			ind_params_unref(out->ind);
		}
	}

	return err;
}

/* bt_conn_foreach() callback of data_out_send(): queue out for conn when it may receive it. */
static void data_out_send_conn(struct bt_conn *conn, void *user_data)
{
	struct data_out_params *out = user_data;
	struct bt_acs_conn *acs_conn = acs_conn_hold(conn);
	int err;

	if (acs_conn == NULL) {
		return;
	}

	if (data_out_conn_check(acs_conn, out) == 0) {
		err = data_out_enqueue(acs_conn, out);

		/* Queueing never fails with -ENOTCONN, so that value means nothing tried yet. */
		if (err == 0 || out->err == -ENOTCONN) {
			out->err = err;
		}
	}

	acs_conn_release(acs_conn);
}

/*
 * Queue out for conn, or for every eligible conn when conn is NULL. Returns 0
 * once a conn took it. Otherwise sending to every conn reports the first failure
 * to queue for an eligible conn, or -ENOTCONN when no conn was eligible.
 */
static int data_out_send(struct bt_conn *conn, struct data_out_params *out)
{
	struct bt_acs_conn *acs_conn;
	int err;

	if (conn == NULL) {
		out->err = -ENOTCONN;
		bt_conn_foreach(BT_CONN_TYPE_LE, data_out_send_conn, out);
		return out->err;
	}

	acs_conn = acs_conn_hold(conn);
	if (acs_conn == NULL) {
		return -ENOTCONN;
	}

	err = data_out_conn_check(acs_conn, out);
	if (err == 0) {
		err = data_out_enqueue(acs_conn, out);
	}

	acs_conn_release(acs_conn);

	return err;
}

/* Send out on ACS Data Out when the active map protects it; returns as bt_gatt_tx_router. */
static int data_out_intercept(struct bt_conn *conn, struct data_out_params *out,
			      enum bt_att_chan_opt chan_opt)
{
	uint8_t operation =
		(out->channel == ACS_REPLY_DOI) ? BT_GATT_CHRC_INDICATE : BT_GATT_CHRC_NOTIFY;
	int err;

	out->isc_id = data_out_isc(out->attr, operation, &out->resource_handle);
	if (out->isc_id == BT_ACS_ISC_ID_NONE) {
		return BT_GATT_TX_ROUTE_CONTINUE;
	}

	/* Data Out picks its own ATT bearer. */
	if (chan_opt != BT_ATT_CHAN_OPT_NONE) {
		return -ENOTSUP;
	}

	if (out->len > ACS_MESSAGE_MAX_SIZE) {
		return -EMSGSIZE;
	}

	if (out->ind == NULL) {
		err = data_out_send(conn, out);
		return (err == 0) ? BT_GATT_TX_ROUTE_CONSUMED : err;
	}

	/*
	 * Not shared yet, so set without the lock, as bt_gatt_indicate() does. Held
	 * across data_out_send() so an early completion cannot destroy params.
	 */
	out->ind->_ref = 1U;

	err = data_out_send(conn, out);
	if (err == 0) {
		ind_params_unref(out->ind);
		return BT_GATT_TX_ROUTE_CONSUMED;
	}

	/* No conn took it, so nothing else holds params; the call fails without destroy. */
	out->ind->_ref = 0U;
	return err;
}

static bool acs_gatt_tx_owns(const struct bt_gatt_attr *attr, uint8_t operation)
{
	return data_out_isc(attr, operation, NULL) != BT_ACS_ISC_ID_NONE;
}

static int acs_gatt_tx_notify(struct bt_conn *conn, const struct bt_gatt_notify_params *params)
{
	struct data_out_params out = {
		.attr = params->attr,
		.data = params->data,
		.len = params->len,
		.channel = ACS_REPLY_DON,
		.ntf_func = params->func,
		.ntf_user_data = params->user_data,
	};

	return data_out_intercept(conn, &out, PARAMS_CHAN_OPT(params));
}

static int acs_gatt_tx_indicate(struct bt_conn *conn, struct bt_gatt_indicate_params *params)
{
	struct data_out_params out = {
		.attr = params->attr,
		.data = params->data,
		.len = params->len,
		.channel = ACS_REPLY_DOI,
		.ind = params,
	};

	return data_out_intercept(conn, &out, PARAMS_CHAN_OPT(params));
}

int acs_data_out_router_register(void)
{
	static const struct bt_gatt_tx_router acs_tx_router = {
		.owns = acs_gatt_tx_owns,
		.notify = acs_gatt_tx_notify,
		.indicate = acs_gatt_tx_indicate,
	};

	return bt_gatt_tx_router_register(&acs_tx_router);
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */
