/*
 * Copyright (c) 2025 Dipak Shetty
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

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/*
 * Serialise PSA output [Ciphertext-or-Data(MSO) || Tag(MSO)] into wire format
 * MAC(LSO) || Ciphertext-or-Data(LSO). All currently supported algorithms
 * (CCM, GCM, GMAC, CMAC) place the MAC before the payload (spec §3.2).
 */
static void data_tx_format_wire(uint8_t *ciphertext_and_tag, uint16_t plain_len, uint8_t tag_size)
{
	uint8_t tag_tmp[ACS_MAX_AUTH_TAG_SIZE];

	memcpy(tag_tmp, ciphertext_and_tag + plain_len, tag_size);
	sys_mem_swap(tag_tmp, tag_size);
	sys_mem_swap(ciphertext_and_tag, plain_len);
	memmove(ciphertext_and_tag + tag_size, ciphertext_and_tag, plain_len);
	memcpy(ciphertext_and_tag, tag_tmp, tag_size);
}

/**
 * @brief Encrypt the response buffer in-place for wire transmission.
 *
 * The buffer was allocated through @ref acs_prepare_reply_buf with
 * ACS_CRYPTO_HEADROOM reserved (protected paths only — plain CP never
 * encrypts) and the inner resource_handle prefix already pushed.
 * Layout on entry:
 *
 *   [reserved headroom: ISC_ID(2) + Nonce_Var(N)] [ resource_handle(2) | payload... ]
 *
 * Layout on success:
 *
 *   [ ISC_ID(2) | Nonce_Var(N) | MAC(T) | Ciphertext(LSO, same len as plaintext) ]
 */
static int data_tx_encrypt_in_place(struct bt_acs_conn *acs_conn, uint16_t isc_id,
				    struct net_buf *buf)
{
	uint8_t *plaintext;
	uint16_t plain_len;
	uint16_t cipher_len = 0;
	uint64_t tx_counter;
	const struct bt_acs_isc_record *isc;
	struct bt_acs_key_desc_runtime *record_state;
	const struct bt_acs_key_desc_record *key_desc;
	uint8_t nonce_var_size;
	uint8_t auth_tag_size;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(buf->len > 0);

	if (net_buf_headroom(buf) < ACS_CRYPTO_HEADROOM) {
		LOG_ERR("encrypt_in_place: insufficient headroom (%zu < %u)", net_buf_headroom(buf),
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
		LOG_ERR("encrypt_in_place: insufficient tailroom for auth tag (%zu < %u)",
			net_buf_tailroom(buf), auth_tag_size);
		return -ENOMEM;
	}

	plaintext = buf->data;
	plain_len = buf->len;

	err = acs_crypto_key_desc_runtime_lookup(acs_conn, isc->key_id, &record_state);
	if (err || record_state->psa_key_id == 0U) {
		LOG_ERR("encrypt_in_place: no record state for isc_id 0x%04x", isc_id);
		return -EACCES;
	}

	/* Capture the nonce variable BEFORE encrypt advances the counter. */
	tx_counter = record_state->tx_nonce_counter;

	/* Reverse plaintext to LSO order for wire (spec §3.2). */
	sys_mem_swap(plaintext, plain_len);

	err = acs_crypto_encrypt(record_state, plaintext, plain_len, plaintext, &cipher_len);
	if (err) {
		if (err == -ENOSPC) {
			LOG_WRN("Nonce exhausted on encrypt — invalidating security");
			bt_acs_invalidate_security(acs_conn->conn);
		}
		LOG_ERR("Data encryption failed (isc_id 0x%04x, err %d)", isc_id, err);
		return err;
	}

	/* Reorder PSA output to wire: MAC(LSO) || Ciphertext(LSO). */
	data_tx_format_wire(plaintext, plain_len, auth_tag_size);

	/* Extend buffer length to include the auth tag. */
	net_buf_add(buf, auth_tag_size);

	/* Push Nonce_Var and ISC_ID into the reserved headroom. */
	uint8_t *hdr = net_buf_push(buf, nonce_var_size + sizeof(uint16_t));

	sys_put_le16(isc_id, hdr);
	sys_put_le(hdr + sizeof(uint16_t), &tx_counter, nonce_var_size);

	return 0;
}

/* Forward declarations for the DOI drain loop and dispatch helpers. */
static void data_tx_completion_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, int err,
				  void *user_data);
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
static int data_tx_send_notify(struct acs_procedure *req);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
static int data_tx_send_indicate(struct acs_procedure *req);
static void data_tx_doi_drain_work(struct k_work *work);
#endif

/**
 * @brief Drain the next queued indication if the DOI channel is idle.
 *
 * Called after posting a new indication and after each indication completes.
 * Only one indication may be in-flight per connection (BLE ATT constraint);
 * the FIFO serialises the rest.  On send failure the failing request is
 * released and the loop advances to the next queued entry.
 */
static void data_tx_drain_doi_queue(struct bt_acs_conn *acs_conn)
{
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	sys_snode_t *snode;
	struct acs_procedure *req;
	int err;

	if (!acs_conn || !acs_conn->conn) {
		return;
	}

	while (true) {
		if (atomic_ptr_get(&acs_conn->active_indication) != NULL) {
			return;
		}

		snode = k_fifo_get(&acs_conn->indicate_fifo, K_NO_WAIT);
		if (!snode) {
			return;
		}

		req = CONTAINER_OF(snode, struct acs_procedure, node);

		if (!atomic_ptr_cas(&acs_conn->active_indication, NULL, req)) {
			k_fifo_put(&acs_conn->indicate_fifo, req);
			return;
		}

		/* The seg-TX engine borrows the buffer for segmented transfer
		 * but does not own it.  req->buffers.response_buf stays alive for the
		 * lifetime of the request — multi-step sequences reuse it.
		 */
		err = acs_seg_tx_send(&acs_conn->indicate_tx, acs_conn->conn, acs_conn->attr_doi,
				      req->buffers.response_buf, data_tx_completion_cb, req);
		if (!err) {
			return;
		}
		atomic_ptr_cas(&acs_conn->active_indication, req, NULL);
		LOG_WRN("DOI seg_tx_send failed: %d (handle 0x%04x)", err,
			req->route.resource_handle);
		acs_procedure_release_tx(req);
	}
#else
	ARG_UNUSED(acs_conn);
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION */
}

/**
 * @brief Deferred continuation handler for multi-step reply sequences.
 *
 * Each step in a protected CP reply sequence (e.g. Get All Active Descriptors)
 * builds a response, runs PSA crypto, and queues a new indication.  Running
 * this synchronously inside the ATT indication confirm callback would nest
 * multiple crypto frames on the ACS workqueue stack.  Instead, the confirm
 * callback schedules connection-owned queue work so each step executes in its
 * own stack frame.
 */
static void data_tx_doi_drain_work(struct k_work *work)
{
	struct bt_acs_conn *acs_conn = CONTAINER_OF(work, struct bt_acs_conn, doi_drain_work);
	struct acs_procedure *req =
		(struct acs_procedure *)atomic_ptr_set(&acs_conn->pending_seq_continue, NULL);
	struct bt_conn *conn = acs_conn->conn;

	if (req) {
		if (!conn || !req->reply_seq.desc) {
			/* Sequence was aborted or connection lost — the ALLOC ref was
			 * deferred from acs_runtime_dispatch_protected_cp_frame(), release it here.
			 */
			acs_procedure_release_owner(req);
		} else {
			acs_seq_on_confirm(req);
		}
	}

	data_tx_drain_doi_queue(acs_conn);
}

void acs_doi_queue_init(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);

	k_fifo_init(&acs_conn->indicate_fifo);
	atomic_ptr_set(&acs_conn->pending_seq_continue, NULL);
	k_work_init(&acs_conn->doi_drain_work, data_tx_doi_drain_work);
}

void acs_doi_queue_submit(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);

	k_work_submit_to_queue(acs_get_wq(), &acs_conn->doi_drain_work);
}

/**
 * @brief DOI indication completion callback.
 *
 * Runs when the ATT layer confirms (or fails) a Data Out Indication segment
 * batch.  Releases the TX reference from the completed indication, then either
 * defers the next reply-sequence step to a work item or drains the next
 * independent request from the FIFO.
 */
static void data_tx_completion_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, int err,
				  void *user_data)
{
	struct acs_procedure *req = user_data;
	struct bt_acs_conn *acs_conn = req ? req->acs_conn : NULL;
	bool continue_reply_seq = req && req->reply_seq.desc != NULL;

	ARG_UNUSED(attr);

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	if (acs_conn) {
		atomic_ptr_cas(&acs_conn->active_indication, req, NULL);
	}
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION */

	/* Drop the TX reference before the next step can acquire a new one. */
	acs_procedure_release_tx(req);

	if (continue_reply_seq) {
		if (!err) {
			if (!acs_conn) {
				acs_procedure_release_owner(req);
				return;
			}
			atomic_ptr_set(&acs_conn->pending_seq_continue, req);
			acs_doi_queue_submit(acs_conn);
			return;
		}

		LOG_WRN("Protected CP indication failed: %d", err);
		acs_seq_abort(req);
	}

	if (acs_conn) {
		acs_doi_queue_submit(acs_conn);
	}
}

/*
 * Internal helpers for acs_tx_submit. Notify and indicate diverge:
 *   - notify   : synchronous one-shot send via acs_seg_notify; releases TX ref
 *                immediately on completion or error.
 *   - indicate : queued via FIFO and drained by data_tx_drain_doi_queue; TX ref
 *                released asynchronously by data_tx_completion_cb.
 * They are not merged because their concurrency models differ.
 */
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
static int data_tx_send_notify(struct acs_procedure *req)
{
	struct bt_acs_conn *acs_conn;
	struct net_buf *buf;
	int err;

	__ASSERT_NO_MSG(req != NULL);
	__ASSERT_NO_MSG(req->acs_conn != NULL);
	__ASSERT_NO_MSG(req->acs_conn->conn != NULL);
	__ASSERT_NO_MSG(req->buffers.response_buf != NULL);
	__ASSERT_NO_MSG(req->buffers.response_buf->len > 0);

	acs_conn = req->acs_conn;
	buf = req->buffers.response_buf;

	err = data_tx_encrypt_in_place(acs_conn, req->route.isc_id, buf);
	if (err) {
		LOG_WRN("DON encrypt failed for handle 0x%04x: %d", req->route.resource_handle,
			err);
		acs_procedure_release_tx(req);
		return err;
	}

	err = acs_seg_notify(acs_conn->conn, acs_conn->attr_don, buf->data, buf->len);
	if (err) {
		LOG_WRN("DON send failed for handle 0x%04x: %d", req->route.resource_handle, err);
	}

	acs_procedure_release_tx(req);
	return err;
}
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION */

struct net_buf *acs_prepare_reply_buf(struct acs_procedure *proc, bool encrypted)
{
	struct net_buf *buf;

	__ASSERT_NO_MSG(proc != NULL);
	__ASSERT_NO_MSG(proc->acs_conn != NULL);

	if (proc->kind == ACS_PROC_KIND_PLAIN_CP) {
		__ASSERT_NO_MSG(!encrypted);
	} else {
		__ASSERT_NO_MSG(encrypted);
	}

	buf = proc->buffers.response_buf;
	if (!buf) {
		buf = acs_buf_alloc(K_NO_WAIT);
		if (!buf) {
			LOG_ERR("buffer pool exhausted");
			return NULL;
		}
		proc->buffers.response_buf = buf;
	}
	net_buf_reset(buf);

	if (encrypted) {
		net_buf_reserve(buf, ACS_CRYPTO_HEADROOM);
	}

	if (proc->kind == ACS_PROC_KIND_PROTECTED_REQ) {
		net_buf_add_le16(buf, proc->route.resource_handle);
	}

	return buf;
}

/* Plain-CP final-mile send. Caller must already hold the busy gate. */
static int acs_tx_submit_plain_cp(struct acs_procedure *proc)
{
	struct bt_acs_conn *acs_conn = proc->acs_conn;
	struct net_buf *rsp_buf;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->conn != NULL);

	rsp_buf = acs_conn->plain_cp_proc.buffers.response_buf;
	acs_conn->plain_cp_proc.buffers.response_buf = NULL;

	__ASSERT_NO_MSG(rsp_buf != NULL);

	/* Pass the buffer via user_data so the completion callback can free
	 * it.  The seg-TX engine borrows the buffer but does not own it.
	 */
	err = acs_seg_tx_send(&acs_conn->cp_tx, acs_conn->conn, acs_conn->attr_cp, rsp_buf,
			      acs_cp_completion_cb, rsp_buf);
	if (err) {
		atomic_set(&acs_conn->plain_cp_proc.plain_cp.locked, 0);
		acs_buf_free(rsp_buf);
	}
	return err;
}

int acs_tx_submit(struct acs_procedure *proc, const struct acs_reply *reply)
{
	if (!proc || !reply || !reply->plaintext) {
		return -EINVAL;
	}

	/* Contract assertions — keep callers honest about every reply field, so
	 * future call sites cannot drift channel/proc ownership out of sync
	 * without immediately tripping a debug build.
	 */
	switch (reply->channel) {
	case ACS_REPLY_CP:
		__ASSERT(proc->kind == ACS_PROC_KIND_PLAIN_CP,
			 "ACS_REPLY_CP requires plain-CP proc");
		__ASSERT(proc != NULL, "plain-CP proc missing procedure");
		__ASSERT(reply->plaintext == proc->buffers.response_buf,
			 "ACS_REPLY_CP plaintext must be the staged "
			 "plain_cp_proc.buffers.response_buf");
		return acs_tx_submit_plain_cp(proc);
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	case ACS_REPLY_DON:
		__ASSERT(proc->kind == ACS_PROC_KIND_PROTECTED_REQ,
			 "ACS_REPLY_DON requires protected-request proc");
		__ASSERT(proc != NULL, "protected-request proc missing req");
		__ASSERT(reply->plaintext == proc->buffers.response_buf,
			 "ACS_REPLY_DON plaintext must be the staged req->buffers.response_buf");
		return data_tx_send_notify(proc);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	case ACS_REPLY_DOI:
		__ASSERT(proc->kind == ACS_PROC_KIND_PROTECTED_REQ,
			 "ACS_REPLY_DOI requires protected-request proc");
		__ASSERT(proc != NULL, "protected-request proc missing req");
		__ASSERT(reply->plaintext == proc->buffers.response_buf,
			 "ACS_REPLY_DOI plaintext must be the staged req->buffers.response_buf");
		return data_tx_send_indicate(proc);
#endif
	default:
		LOG_ERR("invalid channel %d", (int)reply->channel);
		return -EINVAL;
	}
}

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
static int data_tx_send_indicate(struct acs_procedure *req)
{
	struct bt_acs_conn *acs_conn;
	struct net_buf *buf;
	int err;

	__ASSERT_NO_MSG(req != NULL);
	__ASSERT_NO_MSG(req->acs_conn != NULL);
	__ASSERT_NO_MSG(req->acs_conn->conn != NULL);
	__ASSERT_NO_MSG(req->buffers.response_buf != NULL);
	__ASSERT_NO_MSG(req->buffers.response_buf->len > 0);

	acs_conn = req->acs_conn;
	buf = req->buffers.response_buf;

	err = data_tx_encrypt_in_place(acs_conn, req->route.isc_id, buf);
	if (err) {
		LOG_WRN("DOI encrypt failed for handle 0x%04x: %d", req->route.resource_handle,
			err);
		return err;
	}

	acs_procedure_ref(req, ACS_PROCEDURE_REF_TX);
	k_fifo_put(&acs_conn->indicate_fifo, req);
	acs_doi_queue_submit(acs_conn);
	return 0;
}
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION */
