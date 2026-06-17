/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_internal.h"
#include "acs_isc.h"
#include "acs_key_exchange.h"
#include "acs_cp_handlers.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static void reply_continue_handler(struct k_work *work);

struct acs_reply *acs_reply_alloc(struct bt_acs_conn *conn)
{
	__ASSERT_NO_MSG(conn != NULL);

	for (int i = 0; i < ACS_REPLY_SLOTS; i++) {
		if (atomic_cas(&conn->reply_in_use[i], 0, 1)) {
			memset(&conn->replies[i], 0, sizeof(conn->replies[i]));
			conn->replies[i].conn = conn;
			return &conn->replies[i];
		}
	}

	LOG_WRN("reply pool exhausted for conn %p", (void *)conn);
	return NULL;
}

void acs_reply_free(struct acs_reply *reply)
{
	struct bt_acs_conn *conn;
	int idx;

	if (!reply) {
		return;
	}

	conn = reply->conn;
	idx = reply - conn->replies;

	__ASSERT(idx >= 0 && idx < ACS_REPLY_SLOTS, "acs_reply_free: invalid slot index %d", idx);
	__ASSERT(atomic_get(&conn->reply_in_use[idx]) == 1,
		 "acs_reply_free: double-free on slot %d", idx);

	if (reply->request) {
		acs_buf_free(reply->request);
	}
	if (reply->response) {
		acs_buf_free(reply->response);
	}
	reply->aborted = false;
	atomic_set(&conn->reply_in_use[idx], 0);
}

void acs_reply_init_conn(struct bt_acs_conn *conn)
{
	atomic_ptr_set(&conn->pending_continue, NULL);
	k_work_init(&conn->reply_continue_work, reply_continue_handler);
}

struct net_buf *acs_prepare_reply_buf(struct acs_reply *reply)
{
	struct net_buf *buf;

	__ASSERT_NO_MSG(reply != NULL);
	__ASSERT_NO_MSG(reply->conn != NULL);

	buf = reply->response;
	if (!buf) {
		buf = acs_buf_alloc(K_NO_WAIT);
		if (!buf) {
			LOG_ERR("buffer pool exhausted");
			return NULL;
		}
		reply->response = buf;
	}
	net_buf_reset(buf);

	if (reply->channel != ACS_REPLY_CP) {
		net_buf_reserve(buf, ACS_CRYPTO_HEADROOM);
		net_buf_add_le16(buf, reply->resource_handle);
	}

	return buf;
}

static void cp_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
		       void *user_data);

static int reply_submit_plain_cp(struct acs_reply *reply)
{
	struct bt_acs_conn *conn = reply->conn;
	struct net_buf *rsp_buf;
	int err;

	__ASSERT_NO_MSG(conn != NULL);
	__ASSERT_NO_MSG(conn->conn != NULL);

	rsp_buf = reply->response;
	reply->response = NULL;

	__ASSERT_NO_MSG(rsp_buf != NULL);

	err = acs_seg_tx_send(&conn->cp_tx, conn->conn, acs_attr_cp(), rsp_buf, cp_tx_done, reply);
	if (err) {
		atomic_set(&conn->cp_locked, 0);
		acs_buf_free(rsp_buf);
		acs_reply_free(reply);
	}
	return err;
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)

static void data_tx_format_wire(uint8_t *ciphertext_and_tag, uint16_t plain_len, uint8_t tag_size)
{
	uint8_t tag_tmp[ACS_MAX_AUTH_TAG_SIZE];

	memcpy(tag_tmp, ciphertext_and_tag + plain_len, tag_size);
	sys_mem_swap(tag_tmp, tag_size);
	sys_mem_swap(ciphertext_and_tag, plain_len);
	memmove(ciphertext_and_tag + tag_size, ciphertext_and_tag, plain_len);
	memcpy(ciphertext_and_tag, tag_tmp, tag_size);
}

static void data_tx_put_nonce_var(uint8_t *dst,
				  const struct bt_acs_key_desc_runtime *key_desc_runtime,
				  uint64_t tx_counter)
{
	const struct bt_acs_key_desc_record *key_desc = key_desc_runtime->key_desc;
	uint8_t counter_size = acs_key_desc_nonce_counter_size(key_desc);
	uint8_t prefix_size = acs_key_desc_nonce_prefix_size(key_desc);

	sys_put_le(dst, &tx_counter, counter_size);

	if (key_desc->aes.nonce_type == ACS_NONCE_SEQ_EVEN_ODD && prefix_size > 0U) {
		memcpy(&dst[counter_size], key_desc_runtime->server_nonce_fixed, prefix_size);
		sys_mem_swap(&dst[counter_size], prefix_size);
	}
}

static int data_tx_encrypt_in_place(struct bt_acs_conn *acs_conn, uint16_t isc_id,
				    struct net_buf *buf)
{
	uint8_t *plaintext;
	uint16_t plain_len;
	uint16_t cipher_len = 0;
	uint64_t tx_counter;
	const struct bt_acs_isc_record *isc;
	struct bt_acs_key_desc_runtime *key_desc_runtime;
	const struct bt_acs_key_desc_record *key_desc;
	uint8_t nonce_var_size;
	uint8_t auth_tag_size;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(buf->len > 0);

	if (net_buf_headroom(buf) < ACS_CRYPTO_HEADROOM) {
		LOG_ERR("insufficient headroom for encrypt (%zu < %u)", net_buf_headroom(buf),
			ACS_CRYPTO_HEADROOM);
		return -ENOMEM;
	}

	isc = acs_isc_lookup(isc_id);
	if (!isc) {
		return -EINVAL;
	}

	key_desc = acs_key_desc_lookup(isc->key_id);
	if (!key_desc) {
		return -EINVAL;
	}
	nonce_var_size = acs_key_desc_nonce_var_size(key_desc);
	auth_tag_size = acs_key_desc_auth_tag_size(key_desc);

	if (net_buf_tailroom(buf) < auth_tag_size) {
		LOG_ERR("insufficient tailroom for auth tag (%zu < %u)", net_buf_tailroom(buf),
			auth_tag_size);
		return -ENOMEM;
	}

	plaintext = buf->data;
	plain_len = buf->len;

	err = acs_crypto_key_runtime_lookup(acs_conn, isc->key_id, &key_desc_runtime);
	if (err || key_desc_runtime->psa_key_id == 0U) {
		LOG_ERR("no key runtime for isc_id 0x%04x", isc_id);
		return -EACCES;
	}

	tx_counter = key_desc_runtime->tx_nonce_counter;

	sys_mem_swap(plaintext, plain_len);

	err = acs_crypto_encrypt(key_desc_runtime, plaintext, plain_len, plaintext, &cipher_len);
	if (err) {
		if (err == -ENOSPC) {
			LOG_WRN("nonce exhausted on encrypt, invalidating security");
			bt_acs_invalidate_security(acs_conn->conn);
		}
		LOG_ERR("data encryption failed for isc_id 0x%04x: %d", isc_id, err);
		return err;
	}

	__ASSERT_NO_MSG(cipher_len == plain_len + auth_tag_size);

	data_tx_format_wire(plaintext, plain_len, auth_tag_size);
	net_buf_add(buf, auth_tag_size);

	uint8_t *hdr = net_buf_push(buf, nonce_var_size + sizeof(uint16_t));

	sys_put_le16(isc_id, hdr);
	data_tx_put_nonce_var(hdr + sizeof(uint16_t), key_desc_runtime, tx_counter);

	return 0;
}

#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)

static void don_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
			void *user_data);

static void acs_don_queue_submit(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);
	k_work_submit_to_queue(acs_get_wq(), &acs_conn->don_drain_work);
}

static void data_tx_drain_don_queue(struct bt_acs_conn *acs_conn)
{
	sys_snode_t *snode;
	struct acs_reply *reply;
	int err;

	if (!acs_conn || !acs_conn->conn) {
		return;
	}

	while (true) {
		if (atomic_ptr_get(&acs_conn->active_notification) != NULL) {
			return;
		}

		snode = k_fifo_get(&acs_conn->tx_notify_fifo, K_NO_WAIT);
		if (!snode) {
			return;
		}

		reply = CONTAINER_OF(snode, struct acs_reply, node);

		if (!atomic_ptr_cas(&acs_conn->active_notification, NULL, reply)) {
			k_fifo_put(&acs_conn->tx_notify_fifo, reply);
			return;
		}

		err = acs_seg_notify_async_send(&acs_conn->notify_tx, acs_conn->conn,
						acs_attr_don(), reply->response, don_tx_done,
						reply);
		if (!err) {
			return;
		}

		atomic_ptr_cas(&acs_conn->active_notification, reply, NULL);
		LOG_WRN("DON async send failed: %d (handle 0x%04x)", err, reply->resource_handle);
		acs_reply_free(reply);
	}
}

static void data_tx_don_drain_work(struct k_work *work)
{
	struct bt_acs_conn *acs_conn = CONTAINER_OF(work, struct bt_acs_conn, don_drain_work);
	data_tx_drain_don_queue(acs_conn);
}

void acs_don_queue_init(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);
	k_fifo_init(&acs_conn->tx_notify_fifo);
	k_work_init(&acs_conn->don_drain_work, data_tx_don_drain_work);
}

static void don_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
			void *user_data)
{
	struct acs_reply *reply = user_data;
	struct bt_acs_conn *acs_conn = reply->conn;
	bool was_aborted = reply->aborted;

	ARG_UNUSED(bt_conn);
	ARG_UNUSED(attr);

	atomic_ptr_cas(&acs_conn->active_notification, reply, NULL);
	acs_reply_free(reply);

	if (!was_aborted) {
		acs_don_queue_submit(acs_conn);
	}
}

static int data_tx_send_notify(struct acs_reply *reply)
{
	struct bt_acs_conn *acs_conn;
	struct net_buf *buf;
	int err;

	__ASSERT_NO_MSG(reply != NULL);
	__ASSERT_NO_MSG(reply->conn != NULL);
	__ASSERT_NO_MSG(reply->conn->conn != NULL);
	__ASSERT_NO_MSG(reply->response != NULL);
	__ASSERT_NO_MSG(reply->response->len > 0);

	acs_conn = reply->conn;
	buf = reply->response;

	err = data_tx_encrypt_in_place(acs_conn, reply->isc_id, buf);
	if (err) {
		LOG_WRN("DON encrypt failed for handle 0x%04x: %d", reply->resource_handle, err);
		return err;
	}

	k_fifo_put(&acs_conn->tx_notify_fifo, reply);
	acs_don_queue_submit(acs_conn);
	return 0;
}

#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION */

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)

static void doi_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
			void *user_data);

void acs_doi_queue_submit(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);
	k_work_submit_to_queue(acs_get_wq(), &acs_conn->doi_drain_work);
}

static void data_tx_drain_doi_queue(struct bt_acs_conn *acs_conn)
{
	sys_snode_t *snode;
	struct acs_reply *reply;
	int err;

	if (!acs_conn || !acs_conn->conn) {
		return;
	}

	while (true) {
		if (atomic_ptr_get(&acs_conn->active_indication) != NULL) {
			return;
		}

		snode = k_fifo_get(&acs_conn->tx_indicate_fifo, K_NO_WAIT);
		if (!snode) {
			return;
		}

		reply = CONTAINER_OF(snode, struct acs_reply, node);

		if (!atomic_ptr_cas(&acs_conn->active_indication, NULL, reply)) {
			k_fifo_put(&acs_conn->tx_indicate_fifo, reply);
			return;
		}

		err = acs_seg_tx_send(&acs_conn->indicate_tx, acs_conn->conn, acs_attr_doi(),
				      reply->response, doi_tx_done, reply);
		if (!err) {
			return;
		}
		atomic_ptr_cas(&acs_conn->active_indication, reply, NULL);
		LOG_WRN("DOI seg_tx_send failed: %d (handle 0x%04x)", err, reply->resource_handle);
		acs_reply_free(reply);
	}
}

static void data_tx_doi_drain_work(struct k_work *work)
{
	struct bt_acs_conn *acs_conn = CONTAINER_OF(work, struct bt_acs_conn, doi_drain_work);
	data_tx_drain_doi_queue(acs_conn);
}

void acs_doi_queue_init(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);
	k_fifo_init(&acs_conn->tx_indicate_fifo);
	k_work_init(&acs_conn->doi_drain_work, data_tx_doi_drain_work);
}

static void doi_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
			void *user_data)
{
	struct acs_reply *reply = user_data;
	struct bt_acs_conn *acs_conn = reply->conn;

	ARG_UNUSED(bt_conn);
	ARG_UNUSED(attr);

	atomic_ptr_cas(&acs_conn->active_indication, reply, NULL);

	if (reply->aborted || err) {
		if (err) {
			LOG_WRN("DOI indication failed for handle 0x%04x: %d",
				reply->resource_handle, err);
		}
		acs_reply_free(reply);
		acs_doi_queue_submit(acs_conn);
		return;
	}

	if (reply->step == ACS_REPLY_DONE) {
		acs_reply_free(reply);
		acs_doi_queue_submit(acs_conn);
		return;
	}

	atomic_ptr_set(&acs_conn->pending_continue, reply);
	k_work_submit_to_queue(acs_get_wq(), &acs_conn->reply_continue_work);

	acs_doi_queue_submit(acs_conn);
}

static int data_tx_send_indicate(struct acs_reply *reply)
{
	struct bt_acs_conn *acs_conn;
	struct net_buf *buf;
	int err;

	__ASSERT_NO_MSG(reply != NULL);
	__ASSERT_NO_MSG(reply->conn != NULL);
	__ASSERT_NO_MSG(reply->conn->conn != NULL);
	__ASSERT_NO_MSG(reply->response != NULL);
	__ASSERT_NO_MSG(reply->response->len > 0);

	acs_conn = reply->conn;
	buf = reply->response;

	err = data_tx_encrypt_in_place(acs_conn, reply->isc_id, buf);
	if (err) {
		LOG_WRN("DOI encrypt failed for handle 0x%04x: %d", reply->resource_handle, err);
		return err;
	}

	k_fifo_put(&acs_conn->tx_indicate_fifo, reply);
	acs_doi_queue_submit(acs_conn);
	return 0;
}

#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION */

int acs_reply_submit(struct acs_reply *reply)
{
	if (!reply || !reply->response) {
		return -EINVAL;
	}

	switch (reply->channel) {
	case ACS_REPLY_CP:
		return reply_submit_plain_cp(reply);
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	case ACS_REPLY_DON:
		return data_tx_send_notify(reply);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	case ACS_REPLY_DOI:
		return data_tx_send_indicate(reply);
#endif
	default:
		LOG_ERR("invalid channel %d", (int)reply->channel);
		return -EINVAL;
	}
}

static void acs_reply_continue(struct acs_reply *reply)
{
	int err;

	switch (reply->step) {
#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
	case ACS_REPLY_KEX_OK:
		reply->step = ACS_REPLY_KEX_COMPLETE;
		err = acs_kex_send_result(reply, 0x00);
		break;
	case ACS_REPLY_KEX_COMPLETE:
		acs_kex_finalize_success(reply->conn);
		reply->step = ACS_REPLY_DONE;
		return;
	case ACS_REPLY_KEX_FAIL:
		reply->step = ACS_REPLY_KEX_CLEANUP;
		err = acs_kex_send_result(reply, 0x01);
		break;
	case ACS_REPLY_KEX_CLEANUP:
		acs_key_exchange_abort(reply->conn);
		reply->step = ACS_REPLY_DONE;
		return;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS)
	case ACS_REPLY_DESCS_ISC:
		reply->step = ACS_REPLY_DESCS_KEY;
		err = acs_all_active_step_isc(reply);
		break;
	case ACS_REPLY_DESCS_KEY:
		reply->step = ACS_REPLY_DESCS_RC;
		err = acs_all_active_step_key(reply);
		break;
	case ACS_REPLY_DESCS_RC:
		reply->step = ACS_REPLY_DONE;
		err = acs_all_active_step_rc(reply);
		break;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_INVALIDATE_ESTABLISHED_SECURITY)
	case ACS_REPLY_INVALIDATE:
		bt_acs_invalidate_security(reply->conn->conn);
		reply->step = ACS_REPLY_DONE;
		return;
#endif
	default:
		reply->step = ACS_REPLY_DONE;
		return;
	}

	if (err) {
		reply->step = ACS_REPLY_DONE;
	}
}

static void reply_continue_handler(struct k_work *work)
{
	struct bt_acs_conn *conn = CONTAINER_OF(work, struct bt_acs_conn, reply_continue_work);
	struct acs_reply *reply = (struct acs_reply *)atomic_ptr_set(&conn->pending_continue, NULL);

	if (!reply) {
		return;
	}

	if (reply->aborted) {
		acs_reply_free(reply);
		return;
	}

	acs_reply_continue(reply);

	if (reply->step == ACS_REPLY_DONE) {
		bool is_cp = (reply->channel == ACS_REPLY_CP);

		acs_reply_free(reply);
		if (is_cp) {
			atomic_set(&conn->cp_locked, 0);
		}
	}
}

void acs_abort_commit(struct bt_acs_conn *conn);

static void cp_tx_done(struct bt_conn *bt_conn, const struct bt_gatt_attr *attr, int err,
		       void *user_data)
{
	struct acs_reply *reply = user_data;
	struct bt_acs_conn *acs_conn = reply->conn;

	ARG_UNUSED(bt_conn);
	ARG_UNUSED(attr);

	if (err) {
		LOG_WRN("Plain CP indication failed: %d", err);
		acs_reply_free(reply);
		acs_conn->cp_abort_pending = false;
		atomic_set(&acs_conn->cp_locked, 0);
		return;
	}

	if (acs_conn->cp_abort_pending) {
		acs_reply_free(reply);
		acs_abort_commit(acs_conn);
		return;
	}

	if (reply->step == ACS_REPLY_DONE) {
		acs_reply_free(reply);
		atomic_set(&acs_conn->cp_locked, 0);
		return;
	}

	atomic_ptr_set(&acs_conn->pending_continue, reply);
	k_work_submit_to_queue(acs_get_wq(), &acs_conn->reply_continue_work);
}

void acs_abort_commit(struct bt_acs_conn *conn)
{
	struct k_work_sync sync;
	struct acs_reply *reply;

	conn->cp_abort_pending = false;

	k_work_cancel_sync(&conn->cp_tx.tx_work, &sync);

	if (acs_kex_in_progress(conn)) {
		acs_key_exchange_abort(conn);
	}

	acs_reply_abort_all(conn);

	LOG_DBG("Deferred abort committed - sending ABORT SUCCESS");

	reply = acs_reply_alloc(conn);
	if (!reply) {
		LOG_ERR("Deferred ABORT response alloc failed");
		atomic_set(&conn->cp_locked, 0);
		return;
	}
	reply->channel = ACS_REPLY_CP;

	if (acs_cp_rsp_status(reply, BT_ACS_CP_OPCODE_ABORT, BT_ACS_CP_RESPONSE_SUCCESS)) {
		LOG_ERR("Deferred ABORT response send failed");
		atomic_set(&conn->cp_locked, 0);
	}
}

void acs_reply_abort_all(struct bt_acs_conn *conn)
{
	struct k_work_sync sync;
	sys_snode_t *snode;

	if (!conn) {
		return;
	}

	k_work_cancel_sync(&conn->request_work, &sync);

	while ((snode = k_fifo_get(&conn->request_fifo, K_NO_WAIT)) != NULL) {
		acs_reply_free(CONTAINER_OF(snode, struct acs_reply, node));
	}

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	{
		struct k_work_sync doi_sync;

		k_work_cancel_sync(&conn->doi_drain_work, &doi_sync);
	}

	while ((snode = k_fifo_get(&conn->tx_indicate_fifo, K_NO_WAIT)) != NULL) {
		acs_reply_free(CONTAINER_OF(snode, struct acs_reply, node));
	}

	{
		struct acs_reply *active;

		active = (struct acs_reply *)atomic_ptr_set(&conn->active_indication, NULL);
		if (active) {
			active->aborted = true;
		}
	}
#endif

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	{
		struct k_work_sync don_sync;

		k_work_cancel_sync(&conn->don_drain_work, &don_sync);
	}

	while ((snode = k_fifo_get(&conn->tx_notify_fifo, K_NO_WAIT)) != NULL) {
		acs_reply_free(CONTAINER_OF(snode, struct acs_reply, node));
	}

	{
		struct acs_reply *active;

		active = (struct acs_reply *)atomic_ptr_set(&conn->active_notification, NULL);
		if (active) {
			active->aborted = true;
		}
	}
#endif

	{
		struct acs_reply *pending;

		pending = (struct acs_reply *)atomic_ptr_set(&conn->pending_continue, NULL);
		if (pending) {
			pending->aborted = true;
		}
	}
}
