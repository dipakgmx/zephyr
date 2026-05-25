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
#include "acs_rhandle.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#define ACS_REQ_CTX_COUNT (CONFIG_BT_MAX_CONN * CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN)

K_MEM_SLAB_DEFINE_STATIC(acs_req_ctx_slab, sizeof(struct acs_procedure), ACS_REQ_CTX_COUNT,
			 __alignof__(struct acs_procedure));

static void acs_req_work_handler(struct k_work *work);

void acs_procedure_ref(struct acs_procedure *req, enum acs_procedure_ref_who who)
{
	if (!req) {
		return;
	}

	__ASSERT(!atomic_test_bit(req->lifetime.ref_flags, who),
		 "acs_procedure_ref: double-ref who=%d", who);
	atomic_set_bit(req->lifetime.ref_flags, who);
	atomic_inc(&req->lifetime.ref_count);
}

static void acs_procedure_destroy(struct acs_procedure *req)
{
	if (req->registration.pending_slot) {
		atomic_ptr_set(req->registration.pending_slot, NULL);
		req->registration.pending_slot = NULL;
	}

	if (req->buffers.request_buf) {
		acs_buf_free(req->buffers.request_buf);
	}

	if (req->buffers.response_buf) {
		acs_buf_free(req->buffers.response_buf);
	}

	k_mem_slab_free(&acs_req_ctx_slab, req);
}

void acs_procedure_unref(struct acs_procedure *req, enum acs_procedure_ref_who who)
{
	if (!req) {
		return;
	}

	__ASSERT(atomic_test_bit(req->lifetime.ref_flags, who),
		 "acs_procedure_unref: double-release who=%d", who);
	atomic_clear_bit(req->lifetime.ref_flags, who);

	if (atomic_dec(&req->lifetime.ref_count) != 1) {
		return;
	}

	acs_procedure_destroy(req);
}

struct bt_conn *acs_procedure_conn(const struct acs_procedure *req)
{
	if (!req || !req->acs_conn) {
		return NULL;
	}

	return req->acs_conn->conn;
}

uint16_t acs_procedure_resource_handle(const struct acs_procedure *req)
{
	return req ? req->route.resource_handle : 0U;
}

struct acs_procedure *acs_procedure_alloc(struct bt_acs_conn *acs_conn, uint16_t resource_handle,
					  uint16_t isc_id)
{
	struct acs_procedure *req;

	__ASSERT_NO_MSG(acs_conn != NULL);

	if (k_mem_slab_alloc(&acs_req_ctx_slab, (void **)&req, K_NO_WAIT) != 0) {
		LOG_ERR("No free ACS request context for handle 0x%04x (isc_id=0x%04x)",
			resource_handle, isc_id);
		return NULL;
	}

	memset(req, 0, sizeof(*req));
	atomic_set(&req->lifetime.ref_count, 1);
	atomic_set_bit(req->lifetime.ref_flags, ACS_PROCEDURE_REF_ALLOC);
	req->acs_conn = acs_conn;
	req->kind = ACS_PROC_KIND_PROTECTED_REQ;
	req->route.resource_handle = resource_handle;
	req->route.isc_id = isc_id;

	/* Response buffer is allocated lazily in acs_prepare_reply_buf
	 * (called directly from CP / DOI / DON reply builders, or from
	 * acs_auto_respond).
	 */
	req->buffers.response_buf = NULL;

	/* Claim a slot in the connection's active request array */
	for (uint8_t i = 0; i < CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN; i++) {
		if (atomic_ptr_cas(&acs_conn->inflight_reqs[i], NULL, req)) {
			req->registration.pending_slot = &acs_conn->inflight_reqs[i];
			return req;
		}
	}

	/* No free slot */
	LOG_WRN("No free ACS request slot for conn %p", (void *)acs_conn);
	k_mem_slab_free(&acs_req_ctx_slab, req);
	return NULL;
}

void acs_request_queue_init(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);

	k_fifo_init(&acs_conn->request_fifo);
	k_work_init(&acs_conn->request_work, acs_req_work_handler);
}

void acs_request_queue_submit(struct bt_acs_conn *acs_conn, struct acs_procedure *req)
{
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(req != NULL);

	k_fifo_put(&acs_conn->request_fifo, req);
	k_work_submit_to_queue(acs_get_wq(), &acs_conn->request_work);
}

void acs_procedure_release_owner(struct acs_procedure *req)
{
	__ASSERT_NO_MSG(req != NULL);

	if (atomic_test_bit(req->lifetime.ref_flags, ACS_PROCEDURE_REF_ALLOC)) {
		acs_procedure_unref(req, ACS_PROCEDURE_REF_ALLOC);
	}
}

void acs_procedure_release_tx(struct acs_procedure *req)
{
	__ASSERT_NO_MSG(req != NULL);

	if (atomic_test_bit(req->lifetime.ref_flags, ACS_PROCEDURE_REF_TX)) {
		acs_procedure_unref(req, ACS_PROCEDURE_REF_TX);
	}
}

/* Auto-respond to a secure request when no application handler is registered. */
static int acs_auto_respond(struct acs_procedure *req)
{
	const uint8_t *data = req->buffers.request_buf ? req->buffers.request_buf->data : NULL;
	uint16_t len = req->buffers.request_buf ? req->buffers.request_buf->len : 0U;
	ssize_t n;
	uint8_t props = 0;
	struct bt_conn *conn = acs_procedure_conn(req);
	uint16_t resource_handle = acs_procedure_resource_handle(req);
	struct acs_char_attr_ctx ctx = {
		.value_handle = resource_handle,
		.decl = NULL,
		.value = NULL,
	};

	/* Locate both declaration (for properties) and value attribute */
	bt_gatt_foreach_attr(resource_handle - 1U, resource_handle, acs_find_char_attrs_cb, &ctx);

	if (!ctx.value) {
		LOG_WRN("auto_respond: no GATT attr at handle 0x%04x", resource_handle);
		return -ENOENT;
	}

	if (ctx.decl && ctx.decl->user_data) {
		props = ((const struct bt_gatt_chrc *)ctx.decl->user_data)->properties;
	}

	if (req->route.access == ACS_REQ_ACCESS_WRITE) {
		if (!ctx.value->write) {
			LOG_WRN("attr 0x%04x has no write handler", resource_handle);
			return -ENOTSUP;
		}
		ssize_t written = ctx.value->write(conn, ctx.value, data, len, 0, 0);

		if (written < 0) {
			LOG_ERR("auto_respond: write handler error %d", (int)written);
			return written;
		}
	}

	{
		enum acs_reply_channel channel =
			(props & BT_GATT_CHRC_INDICATE) ? ACS_REPLY_DOI : ACS_REPLY_DON;
		struct net_buf *rsp_buf = acs_prepare_reply_buf(req, true);
		struct acs_reply reply;

		if (!rsp_buf) {
			LOG_ERR("auto_respond: response pool exhausted (handle 0x%04x)",
				resource_handle);
			return -ENOMEM;
		}

		if (props & BT_GATT_CHRC_READ) {
			if (!ctx.value->read) {
				LOG_WRN("auto_respond: attr 0x%04x has no read handler",
					resource_handle);
				return -ENOTSUP;
			}

			n = ctx.value->read(conn, ctx.value, net_buf_tail(rsp_buf),
					    net_buf_tailroom(rsp_buf), 0);
			if (n < 0) {
				LOG_ERR("auto_respond: read handler error %d", (int)n);
				return n;
			}

			net_buf_add(rsp_buf, n);
		}

		reply = (struct acs_reply){
			.channel = channel,
			.plaintext = rsp_buf,
		};

		return acs_tx_submit(req, &reply);
	}
}

void acs_procedure_abort_all(struct bt_acs_conn *acs_conn)
{
	struct acs_procedure *req;
	uint16_t req_count = 0U;
	struct k_work_sync request_sync;
	uint16_t queued_count = 0U;
	sys_snode_t *snode;

	if (!acs_conn) {
		return;
	}

	k_work_cancel_sync(&acs_conn->request_work, &request_sync);

	while ((snode = k_fifo_get(&acs_conn->request_fifo, K_NO_WAIT)) != NULL) {
		ARG_UNUSED(snode);
	}

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	{
		struct k_work_sync doi_sync;

		k_work_cancel_sync(&acs_conn->doi_drain_work, &doi_sync);
	}

	atomic_ptr_set(&acs_conn->active_indication, NULL);
	atomic_ptr_set(&acs_conn->pending_seq_continue, NULL);

	/* Flush pending indications from the send FIFO, releasing their TX refs. */
	while ((snode = k_fifo_get(&acs_conn->indicate_fifo, K_NO_WAIT)) != NULL) {
		struct acs_procedure *queued = CONTAINER_OF(snode, struct acs_procedure, node);

		acs_procedure_release_tx(queued);
		queued_count++;
	}
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION */

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	{
		struct k_work_sync don_sync;

		k_work_cancel_sync(&acs_conn->don_drain_work, &don_sync);
	}

	atomic_ptr_set(&acs_conn->active_notification, NULL);

	while ((snode = k_fifo_get(&acs_conn->notify_fifo, K_NO_WAIT)) != NULL) {
		struct acs_procedure *queued = CONTAINER_OF(snode, struct acs_procedure, node);

		acs_procedure_release_tx(queued);
		queued_count++;
	}
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION */

	/* Atomically detach each request and drop held references. */
	for (uint8_t i = 0; i < CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN; i++) {
		req = atomic_ptr_set(&acs_conn->inflight_reqs[i], NULL);
		if (!req) {
			continue;
		}

		req_count++;
		req->acs_conn = NULL;

		/* Drop each held reference using the unified API */
		if (atomic_test_bit(req->lifetime.ref_flags, ACS_PROCEDURE_REF_ALLOC)) {
			acs_procedure_unref(req, ACS_PROCEDURE_REF_ALLOC);
		}
		if (atomic_test_bit(req->lifetime.ref_flags, ACS_PROCEDURE_REF_TX)) {
			acs_procedure_unref(req, ACS_PROCEDURE_REF_TX);
		}
	}

	if (req_count > 0U || queued_count > 0U) {
		LOG_DBG("Aborted %u ACS request(s), dropped %u queued response(s)", req_count,
			queued_count);
	}
}

static void acs_req_work_handler(struct k_work *work)
{
	struct bt_acs_conn *acs_conn = CONTAINER_OF(work, struct bt_acs_conn, request_work);
	sys_snode_t *snode;

	while ((snode = k_fifo_get(&acs_conn->request_fifo, K_NO_WAIT)) != NULL) {
		struct acs_procedure *req = CONTAINER_OF(snode, struct acs_procedure, node);
		struct bt_conn const *conn = acs_procedure_conn(req);
		int err;

		if (!conn || !req->acs_conn) {
			acs_procedure_release_owner(req);
			continue;
		}

		err = acs_auto_respond(req);
		if (err) {
			LOG_WRN("ACS auto-response failed for handle 0x%04x: %d",
				req->route.resource_handle, err);
		}

		/* The handler / auto_respond consumed the input bytes already.
		 * Drop decrypted_request now so it returns to the pool while we wait
		 * for the indication / notification to confirm - acs_req_free will see
		 * NULL and skip the free at refcount-zero.
		 */
		if (req->buffers.request_buf) {
			acs_buf_free(req->buffers.request_buf);
			req->buffers.request_buf = NULL;
		}

		acs_procedure_release_owner(req);
	}
}
