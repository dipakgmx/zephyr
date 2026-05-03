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

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/**
 * @brief Recover a request context from its k_fifo snode linkage.
 */
static inline struct bt_acs_prot_resource_req *acs_req_from_node(sys_snode_t *snode)
{
	return CONTAINER_OF(snode, struct bt_acs_prot_resource_req, node);
}

static void acs_put_nonce_var(uint8_t *secure_data, uint8_t nonce_var_size, uint32_t tx_counter)
{
	memset(secure_data, 0, nonce_var_size);
	for (size_t i = 0; i < MIN((uint8_t)sizeof(tx_counter), nonce_var_size); i++) {
		secure_data[i] = (tx_counter >> (BITS_PER_BYTE * i)) & UINT8_MAX;
	}
}

/*
 * Serialise PSA output [Ciphertext-or-Data(MSO) || Tag(MSO)] into wire format
 * MAC(LSO) || Ciphertext-or-Data(LSO). All currently supported algorithms
 * (CCM, GCM, GMAC, CMAC) place the MAC before the payload (spec §3.2).
 */
static void acs_format_secure_data_wire(uint8_t *ciphertext_and_tag, uint16_t plain_len)
{
#if defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM) ||                                              \
	defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) ||                                          \
	defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC) ||                                         \
	defined(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	uint8_t tag_tmp[ACS_ACTIVE_AUTH_TAG_SIZE];

	memcpy(tag_tmp, ciphertext_and_tag + plain_len, ACS_ACTIVE_AUTH_TAG_SIZE);
	sys_mem_swap(tag_tmp, ACS_ACTIVE_AUTH_TAG_SIZE);
	sys_mem_swap(ciphertext_and_tag, plain_len);
	memmove(ciphertext_and_tag + ACS_ACTIVE_AUTH_TAG_SIZE, ciphertext_and_tag, plain_len);
	memcpy(ciphertext_and_tag, tag_tmp, ACS_ACTIVE_AUTH_TAG_SIZE);
#else
	sys_mem_swap(ciphertext_and_tag, plain_len);
	sys_mem_swap(ciphertext_and_tag + plain_len, ACS_ACTIVE_AUTH_TAG_SIZE);
#endif
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
static int acs_data_out_encrypt_in_place(struct bt_acs_conn *acs_conn, uint16_t isc_id,
					 struct net_buf *buf)
{
	uint8_t *plaintext;
	uint16_t plain_len;
	uint16_t cipher_len = 0;
	uint32_t tx_counter;
	int err;

	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(buf != NULL);
	__ASSERT_NO_MSG(buf->len > 0);

	if (net_buf_headroom(buf) < ACS_CRYPTO_HEADROOM) {
		LOG_ERR("encrypt_in_place: insufficient headroom (%zu < %u)", net_buf_headroom(buf),
			ACS_CRYPTO_HEADROOM);
		return -ENOMEM;
	}

	if (net_buf_tailroom(buf) < ACS_ACTIVE_AUTH_TAG_SIZE) {
		LOG_ERR("encrypt_in_place: insufficient tailroom for auth tag (%zu < %u)",
			net_buf_tailroom(buf), ACS_ACTIVE_AUTH_TAG_SIZE);
		return -ENOMEM;
	}

	plaintext = buf->data;
	plain_len = buf->len;

	/* Capture the nonce variable BEFORE encrypt advances the counter. */
	tx_counter = acs_conn->crypto.tx_nonce_counter;

	/* Reverse plaintext to LSO order for wire (spec §3.2). */
	sys_mem_swap(plaintext, plain_len);

	err = acs_crypto_encrypt(acs_conn, isc_id, plaintext, plain_len, plaintext, &cipher_len);
	if (err) {
		if (err == -ENOSPC) {
			LOG_WRN("Nonce exhausted on encrypt — invalidating security");
			bt_acs_invalidate_security(acs_conn->conn);
		}
		LOG_ERR("Data encryption failed (isc_id 0x%04x, err %d)", isc_id, err);
		return err;
	}

	/* Reorder PSA output to wire: MAC(LSO) || Ciphertext(LSO). */
	acs_format_secure_data_wire(plaintext, plain_len);

	/* Extend buffer length to include the auth tag. */
	net_buf_add(buf, ACS_ACTIVE_AUTH_TAG_SIZE);

	/* Push Nonce_Var and ISC_ID into the reserved headroom. */
	{
		uint8_t *hdr = net_buf_push(buf, ACS_ACTIVE_NONCE_VAR_SIZE + 2U);

		sys_put_le16(isc_id, hdr);
		acs_put_nonce_var(hdr + 2U, ACS_ACTIVE_NONCE_VAR_SIZE, tx_counter);
	}

	return 0;
}

/* Forward declaration for try_send_next. */
static void acs_data_out_on_indicate_done(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					  int err, void *user_data);

/**
 * @brief Drain the next queued indication if the DOI channel is idle.
 *
 * Called after posting a new indication and after each indication completes.
 * Only one indication may be in-flight per connection (BLE ATT constraint);
 * the FIFO serialises the rest.  On send failure the failing request is
 * released and the loop advances to the next queued entry.
 */
static void try_send_next(struct bt_acs_conn *acs_conn)
{
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	sys_snode_t *snode;
	struct bt_acs_prot_resource_req *req;
	int err;

	if (!acs_conn || !acs_conn->conn) {
		return;
	}

	for (uint8_t attempts = 0; attempts < CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN; attempts++) {
		if (atomic_ptr_get(&acs_conn->active_indication) != NULL) {
			return;
		}

		snode = k_fifo_get(&acs_conn->indicate_fifo, K_NO_WAIT);
		if (!snode) {
			return;
		}

		req = acs_req_from_node(snode);

		if (!atomic_ptr_cas(&acs_conn->active_indication, NULL, req)) {
			k_fifo_put(&acs_conn->indicate_fifo, req);
			return;
		}

		/* The seg-TX engine borrows the buffer for segmented transfer
		 * but does not own it.  req->response stays alive for the
		 * lifetime of the request — multi-step sequences reuse it.
		 */
		err = acs_seg_tx_send(&acs_conn->indicate_tx, acs_conn->conn, acs_conn->attr_doi,
				      req->response, acs_data_out_on_indicate_done, req);
		if (!err) {
			return;
		}
		atomic_ptr_cas(&acs_conn->active_indication, req, NULL);
		LOG_WRN("DOI seg_tx_send failed: %d (handle 0x%04x)", err, req->resource_handle);
		acs_prot_resource_req_tx_done(req);
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
 * multiple crypto frames on the sysworkq stack.  Instead, the confirm callback
 * schedules this work item so each step executes in its own stack frame.
 *
 * req->work is safe to reuse here: protected CP requests are dispatched
 * synchronously via acs_cp_dispatch(), not through the work queue, so the
 * work item is idle by the time the reply sequence runs.
 */
static void acs_seq_continue_work_handler(struct k_work *work)
{
	struct bt_acs_prot_resource_req *req =
		CONTAINER_OF(work, struct bt_acs_prot_resource_req, work);
	struct bt_acs_conn *acs_conn = req->acs_conn;
	struct bt_conn *conn = acs_conn ? acs_conn->conn : NULL;
	const struct bt_gatt_attr *attr = acs_conn ? acs_conn->attr_doi : NULL;

	if (!conn || !req->reply_seq.desc) {
		/* Sequence was aborted or connection lost — the ALLOC ref was
		 * deferred from acs_runtime_dispatch_protected_cp_frame(), release it here.
		 */
		acs_prot_resource_req_release_owner(req);
		goto drain;
	}

	acs_seq_on_req_confirm(req, conn, attr);

drain:
	if (acs_conn) {
		try_send_next(acs_conn);
	}
}

/**
 * @brief DOI indication completion callback.
 *
 * Runs when the ATT layer confirms (or fails) a Data Out Indication segment
 * batch.  Releases the TX reference from the completed indication, then either
 * defers the next reply-sequence step to a work item or drains the next
 * independent request from the FIFO.
 */
static void acs_data_out_on_indicate_done(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					  int err, void *user_data)
{
	struct bt_acs_prot_resource_req *req = user_data;
	struct bt_acs_conn *acs_conn = req ? req->acs_conn : NULL;
	bool continue_reply_seq = req && req->reply_seq.desc != NULL;

	ARG_UNUSED(attr);

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	if (acs_conn) {
		atomic_ptr_cas(&acs_conn->active_indication, req, NULL);
	}
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION */

	/* Drop the TX reference before the next step can acquire a new one. */
	acs_prot_resource_req_tx_done(req);

	if (continue_reply_seq) {
		if (!err) {
			k_work_init(&req->work, acs_seq_continue_work_handler);
			k_work_submit(&req->work);
			return;
		}

		struct acs_exec_owner owner = acs_exec_owner_protected(req);

		/* req->acs_conn may be NULL post-disconnect; preserve the legacy
		 * fallback so abort accounting still resolves the connection.
		 */
		if (!owner.acs_conn) {
			owner.acs_conn = acs_conn ? acs_conn
						  : (conn ? acs_conn_lookup(conn) : NULL);
		}

		LOG_WRN("Protected CP indication failed: %d", err);
		acs_seq_abort(&owner);
	}

	if (acs_conn) {
		try_send_next(acs_conn);
	}
}

int acs_prot_resource_rsp_notify(struct bt_acs_prot_resource_req *req, struct net_buf *plaintext)
{
#if !IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	ARG_UNUSED(req);
	ARG_UNUSED(plaintext);
	return -ENOTSUP;
#else
	struct bt_acs_conn *acs_conn;
	int err;

	__ASSERT_NO_MSG(req != NULL);
	__ASSERT_NO_MSG(req->acs_conn != NULL);
	__ASSERT_NO_MSG(req->acs_conn->conn != NULL);
	__ASSERT_NO_MSG(plaintext != NULL);
	__ASSERT_NO_MSG(plaintext->len > 0);
	__ASSERT_NO_MSG(plaintext == req->response);

	acs_conn = req->acs_conn;
	req->send_method = ACS_PROT_RESOURCE_SEND_NOTIFY;

	err = acs_data_out_encrypt_in_place(acs_conn, req->isc_id, plaintext);
	if (err) {
		LOG_WRN("DON encrypt failed for handle 0x%04x: %d", req->resource_handle, err);
		acs_prot_resource_req_tx_done(req);
		return err;
	}

	err = acs_seg_notify(acs_conn->conn, acs_conn->attr_don, plaintext->data, plaintext->len);
	if (err) {
		LOG_WRN("DON send failed for handle 0x%04x: %d", req->resource_handle, err);
	}

	acs_prot_resource_req_tx_done(req);
	return err;
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION */
}

struct net_buf *acs_prepare_reply_buf(const struct acs_exec_owner *owner,
				      enum acs_reply_channel channel, bool encrypted)
{
	struct net_buf **slot;
	struct net_buf *buf;
	uint16_t resource_handle = 0U;

	__ASSERT_NO_MSG(owner != NULL);
	__ASSERT_NO_MSG(owner->acs_conn != NULL);

	if (owner->kind == ACS_EXEC_OWNER_PLAIN_CP) {
		__ASSERT_NO_MSG(channel == ACS_REPLY_CP);
		__ASSERT_NO_MSG(!encrypted);
		__ASSERT_NO_MSG(owner->plain_cp != NULL);
		slot = &owner->plain_cp->response;
	} else {
		__ASSERT_NO_MSG(channel == ACS_REPLY_DOI || channel == ACS_REPLY_DON);
		__ASSERT_NO_MSG(owner->req != NULL);
		slot = &owner->req->response;
		resource_handle = owner->req->resource_handle;
	}

	if (!*slot) {
		*slot = acs_buf_alloc(K_NO_WAIT);
		if (!*slot) {
			LOG_ERR("acs_prepare_reply_buf: buffer pool exhausted");
			return NULL;
		}
	} else {
		net_buf_reset(*slot);
	}
	buf = *slot;

	if (encrypted) {
		/* Reserve room for the secure-transport prefix added by
		 * acs_data_out_encrypt_in_place: ISC_ID + Nonce_Var.
		 */
		net_buf_reserve(buf, ACS_CRYPTO_HEADROOM);
	}

	if (owner->kind == ACS_EXEC_OWNER_PROTECTED_REQ) {
		/* Protected wire format: inner [Resource_Handle | payload] */
		net_buf_add_le16(buf, resource_handle);
	}

	return buf;
}

int acs_cp_rsp_status(const struct acs_exec_owner *owner, uint8_t req_opcode, uint8_t code)
{
	struct net_buf *buf;
	struct acs_reply reply;
	struct acs_reply_mode reply_mode;
	bool plain_cp;
	int err;

	__ASSERT_NO_MSG(owner != NULL);
	__ASSERT_NO_MSG(owner->acs_conn != NULL);

	reply_mode = acs_owner_reply_mode(owner);
	plain_cp = acs_owner_is_plain_cp(owner);
	buf = acs_prepare_reply_buf(owner, reply_mode.channel, reply_mode.encrypted);
	if (!buf) {
		acs_seq_abort(owner);
		if (plain_cp) {
			atomic_set(&owner->acs_conn->plain_cp_proc.locked, 0);
		}
		return -ENOMEM;
	}

	net_buf_add_u8(buf, BT_ACS_CP_OPCODE_RESPONSE_CODE);
	net_buf_add_u8(buf, req_opcode);
	net_buf_add_u8(buf, code);

	acs_cp_build_reply(owner, &reply);

	err = acs_tx_submit(owner, &reply);
	if (err) {
		/* Status replies are still replies — same failure contract as
		 * acs_cp_send_reply: tear down any active sequence so we don't
		 * leak deferred ALLOC refs or leave stale step state behind.
		 * The plain-CP busy-gate release for submit failure is handled
		 * inside acs_tx_submit_plain_cp itself, so we don't double up.
		 */
		acs_seq_abort(owner);
		if (owner->kind == ACS_EXEC_OWNER_PROTECTED_REQ) {
			LOG_WRN("Protected CP status indication failed for handle 0x%04x: %d",
				owner->req->resource_handle, err);
		} else {
			LOG_WRN("Plain CP status indication failed: %d", err);
		}
	}

	return err;
}

/* Plain-CP final-mile send. Caller must already hold the busy gate. */
static int acs_tx_submit_plain_cp(const struct acs_exec_owner *owner,
				  const struct acs_reply *reply)
{
	struct bt_acs_conn *acs_conn = owner->acs_conn;
	struct net_buf *rsp_buf;
	int err;

	__ASSERT_NO_MSG(reply->channel == ACS_REPLY_CP);
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(acs_conn->conn != NULL);

	rsp_buf = acs_conn->plain_cp_proc.response;
	acs_conn->plain_cp_proc.response = NULL;

	__ASSERT_NO_MSG(rsp_buf != NULL);
	__ASSERT_NO_MSG(rsp_buf == reply->plaintext);

	/* Pass the buffer via user_data so the completion callback can free
	 * it.  The seg-TX engine borrows the buffer but does not own it.
	 */
	err = acs_seg_tx_send(&acs_conn->cp_tx, acs_conn->conn, acs_conn->attr_cp, rsp_buf,
			      acs_cp_on_indicate_done, rsp_buf);
	if (err) {
		atomic_set(&acs_conn->plain_cp_proc.locked, 0);
		acs_buf_free(rsp_buf);
	}
	return err;
}

int acs_tx_submit(const struct acs_exec_owner *owner, const struct acs_reply *reply)
{
	if (!owner || !reply || !reply->plaintext) {
		return -EINVAL;
	}

	/* Contract assertions — keep callers honest about every reply field, so
	 * future call sites cannot drift channel/owner/encrypted/needs_confirm
	 * out of sync without immediately tripping a debug build.
	 */
	switch (reply->channel) {
	case ACS_REPLY_CP:
		__ASSERT(owner->kind == ACS_EXEC_OWNER_PLAIN_CP,
			 "ACS_REPLY_CP requires plain-CP owner");
		__ASSERT(owner->plain_cp != NULL, "plain-CP owner missing procedure");
		__ASSERT(reply->plaintext == owner->plain_cp->response,
			 "ACS_REPLY_CP plaintext must be the staged plain_cp_proc.response");
		__ASSERT(!reply->encrypted, "ACS_REPLY_CP must be unencrypted (plain transport)");
		__ASSERT(reply->needs_confirm,
			 "ACS_REPLY_CP is always confirmed (segmented indication)");
		return acs_tx_submit_plain_cp(owner, reply);
	case ACS_REPLY_DON:
		__ASSERT(owner->kind == ACS_EXEC_OWNER_PROTECTED_REQ,
			 "ACS_REPLY_DON requires protected-request owner");
		__ASSERT(owner->req != NULL, "protected-request owner missing req");
		__ASSERT(reply->plaintext == owner->req->response,
			 "ACS_REPLY_DON plaintext must be the staged req->response");
		__ASSERT(reply->encrypted, "ACS_REPLY_DON must be encrypted");
		__ASSERT(!reply->needs_confirm,
			 "ACS_REPLY_DON is unconfirmed (notification)");
		return acs_prot_resource_rsp_notify(owner->req, reply->plaintext);
	case ACS_REPLY_DOI:
		__ASSERT(owner->kind == ACS_EXEC_OWNER_PROTECTED_REQ,
			 "ACS_REPLY_DOI requires protected-request owner");
		__ASSERT(owner->req != NULL, "protected-request owner missing req");
		__ASSERT(reply->plaintext == owner->req->response,
			 "ACS_REPLY_DOI plaintext must be the staged req->response");
		__ASSERT(reply->encrypted, "ACS_REPLY_DOI must be encrypted");
		__ASSERT(reply->needs_confirm,
			 "ACS_REPLY_DOI is always confirmed (segmented indication)");
		return acs_prot_resource_rsp_indicate(owner->req, reply->plaintext);
	default:
		LOG_ERR("acs_tx_submit: invalid channel %d", (int)reply->channel);
		return -EINVAL;
	}
}


int acs_prot_resource_rsp_indicate(struct bt_acs_prot_resource_req *req, struct net_buf *plaintext)
{
#if !IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	ARG_UNUSED(req);
	ARG_UNUSED(plaintext);
	return -ENOTSUP;
#else
	struct bt_acs_conn *acs_conn;
	int err;

	__ASSERT_NO_MSG(req != NULL);
	__ASSERT_NO_MSG(req->acs_conn != NULL);
	__ASSERT_NO_MSG(req->acs_conn->conn != NULL);
	__ASSERT_NO_MSG(plaintext != NULL);
	__ASSERT_NO_MSG(plaintext->len > 0);
	__ASSERT_NO_MSG(plaintext == req->response);

	acs_conn = req->acs_conn;
	req->send_method = ACS_PROT_RESOURCE_SEND_INDICATE;

	err = acs_data_out_encrypt_in_place(acs_conn, req->isc_id, plaintext);
	if (err) {
		LOG_WRN("DOI encrypt failed for handle 0x%04x: %d", req->resource_handle, err);
		return err;
	}

	acs_prot_resource_req_ref(req, PROT_RESOURCE_REF_TX);
	k_fifo_put(&acs_conn->indicate_fifo, req);
	try_send_next(acs_conn);
	return 0;
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION */
}
