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
#include "acs_rmap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#define ACS_REQ_CTX_COUNT (CONFIG_BT_MAX_CONN * CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN)

K_MEM_SLAB_DEFINE_STATIC(acs_req_ctx_slab, sizeof(struct bt_acs_prot_resource_req),
			 ACS_REQ_CTX_COUNT, __alignof__(struct bt_acs_prot_resource_req));

STRUCT_SECTION_ITERABLE(bt_acs_prot_resource_handler_entry, acs_req_handler_sentinel) = {
	.char_uuid = NULL,
	.handler = NULL,
};

static void acs_req_work_handler(struct k_work *work);

/**
 * Context for locating a characteristic declaration and value attribute by
 * their ATT handles via bt_gatt_foreach_attr().
 */
struct acs_attr_ctx {
	uint16_t value_handle;
	const struct bt_gatt_attr *decl;  /**< declaration attribute (value_handle - 1) */
	const struct bt_gatt_attr *value; /**< value attribute (value_handle) */
};

static uint8_t acs_find_char_attrs_cb(const struct bt_gatt_attr *attr, uint16_t handle,
				      void *user_data)
{
	struct acs_attr_ctx *ctx = user_data;

	if (handle == ctx->value_handle - 1U) {
		ctx->decl = attr;
	} else if (handle == ctx->value_handle) {
		ctx->value = attr;
	}
	return BT_GATT_ITER_CONTINUE;
}

void acs_prot_resource_req_ref(struct bt_acs_prot_resource_req *req,
			       enum acs_prot_resource_ref_who who)
{
	if (!req) {
		return;
	}

	__ASSERT(!atomic_test_bit(req->ref_flags, who),
		 "acs_prot_resource_req_ref: double-ref who=%d", who);
	atomic_set_bit(req->ref_flags, who);
	atomic_inc(&req->ref_count);
}

/**
 * @brief Return a request context to the correct slab once all references are gone.
 */
static void acs_req_free(struct bt_acs_prot_resource_req *req)
{
	/* Release borrowed input buffer ref if we own it */
	if (req->input_owned && req->decrypted_request) {
		acs_buf_free(req->decrypted_request);
	}

	/* Release response staging buffer */
	if (req->response) {
		acs_buf_free(req->response);
	}

	k_mem_slab_free(&acs_req_ctx_slab, req);
}

void acs_prot_resource_req_unref(struct bt_acs_prot_resource_req *req,
				 enum acs_prot_resource_ref_who who)
{
	if (!req) {
		return;
	}

	__ASSERT(atomic_test_bit(req->ref_flags, who),
		 "acs_prot_resource_req_unref: double-release who=%d", who);
	atomic_clear_bit(req->ref_flags, who);

	if (atomic_dec(&req->ref_count) != 1) {
		return;
	}

	if (req->acs_conn) {
		atomic_ptr_set(&req->acs_conn->pending_reqs[req->req_slot], NULL);
	}

	acs_req_free(req);
}

struct bt_conn *acs_prot_resource_req_conn(const struct bt_acs_prot_resource_req *req)
{
	if (!req || !req->acs_conn) {
		return NULL;
	}

	return req->acs_conn->conn;
}

uint16_t acs_prot_resource_req_handle(const struct bt_acs_prot_resource_req *req)
{
	return req ? req->resource_handle : 0U;
}

struct bt_acs_prot_resource_req *acs_prot_resource_req_alloc(struct bt_acs_conn *acs_conn,
							     uint16_t resource_handle,
							     uint16_t isc_id, uint16_t data_offset,
							     uint16_t data_length)
{
	struct bt_acs_prot_resource_req *req;

	__ASSERT_NO_MSG(acs_conn != NULL);

	if (k_mem_slab_alloc(&acs_req_ctx_slab, (void **)&req, K_NO_WAIT) != 0) {
		LOG_ERR("No free ACS request context for handle 0x%04x (isc_id=0x%04x)",
			resource_handle, isc_id);
		return NULL;
	}

	memset(req, 0, sizeof(*req));
	atomic_set(&req->ref_count, 1);
	atomic_set_bit(req->ref_flags, PROT_RESOURCE_REF_ALLOC);
	k_work_init(&req->work, acs_req_work_handler);
	req->acs_conn = acs_conn;
	req->resource_handle = resource_handle;
	req->isc_id = isc_id;
	req->data_length = data_length;
	req->data_offset = data_offset;

	/* Response buffer is allocated lazily in acs_cp_rsp_alloc */
	req->response = NULL;

	/* Claim a slot in the connection's active request array */
	for (uint8_t i = 0; i < CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN; i++) {
		if (atomic_ptr_cas(&acs_conn->pending_reqs[i], NULL, req)) {
			req->req_slot = i;
			return req;
		}
	}

	/* No free slot */
	LOG_WRN("No free ACS request slot for conn %p", (void *)acs_conn);
	k_mem_slab_free(&acs_req_ctx_slab, req);
	return NULL;
}

void acs_prot_resource_req_release_owner(struct bt_acs_prot_resource_req *req)
{
	if (!req) {
		return;
	}

	if (atomic_test_bit(req->ref_flags, PROT_RESOURCE_REF_ALLOC)) {
		acs_prot_resource_req_unref(req, PROT_RESOURCE_REF_ALLOC);
	}
}

void acs_prot_resource_req_release_tx(struct bt_acs_prot_resource_req *req)
{
	if (!req) {
		return;
	}

	if (atomic_test_bit(req->ref_flags, PROT_RESOURCE_REF_TX)) {
		acs_prot_resource_req_unref(req, PROT_RESOURCE_REF_TX);
	}
}

void acs_prot_resource_req_tx_done(struct bt_acs_prot_resource_req *req)
{
	if (!req) {
		return;
	}

	req->send_method = ACS_PROT_RESOURCE_SEND_NONE;
	acs_prot_resource_req_release_tx(req);
}

/* Auto-respond to a secure request when no application handler is registered. */
static int acs_auto_respond(struct bt_acs_prot_resource_req *req)
{
	const uint8_t *data =
		req->decrypted_request ? req->decrypted_request->data + req->data_offset : NULL;
	uint16_t len = req->data_length;
	ssize_t n;
	uint8_t props = 0;
	struct bt_conn *conn = acs_prot_resource_req_conn(req);
	uint16_t resource_handle = acs_prot_resource_req_handle(req);
	struct acs_attr_ctx ctx = {
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

	/* Secure write: push the request data through the attribute's write handler */
	if (len > 0) {
		if (!ctx.value->write) {
			LOG_WRN("auto_respond: attr 0x%04x has no write handler", resource_handle);
			return -ENOTSUP;
		}
		ssize_t written = ctx.value->write(conn, ctx.value, data, len, 0, 0);

		if (written < 0) {
			LOG_ERR("auto_respond: write handler error %d", (int)written);
			return written;
		}
	}

	/* Determine characteristic properties for routing and read-back decisions. */
	if (ctx.decl && ctx.decl->user_data) {
		props = ((const struct bt_gatt_chrc *)ctx.decl->user_data)->properties;
	}

	/* Lazy-alloc the response buffer and reserve crypto headroom. */
	if (!req->response) {
		req->response = acs_buf_alloc(K_NO_WAIT);
		if (!req->response) {
			LOG_ERR("auto_respond: response pool exhausted (handle 0x%04x)",
				resource_handle);
			return -ENOMEM;
		}
	} else {
		net_buf_reset(req->response);
	}
	net_buf_reserve(req->response, ACS_CRYPTO_HEADROOM);

	/* Build response: [resource_handle_le16][attribute_value...] */
	net_buf_add_le16(req->response, resource_handle);

	/* Read back the current attribute value, unless the characteristic is write-only. */
	if (props & BT_GATT_CHRC_READ) {
		if (!ctx.value->read) {
			LOG_WRN("auto_respond: attr 0x%04x has no read handler", resource_handle);
			return -ENOTSUP;
		}

		n = ctx.value->read(conn, ctx.value, net_buf_tail(req->response),
				    net_buf_tailroom(req->response), 0);

		if (n < 0) {
			LOG_ERR("auto_respond: read handler error %d", (int)n);
			return n;
		}

		net_buf_add(req->response, n);
	}

	/* Route: Indicate → DOI, Notify/Read → DON. */
	if (props & BT_GATT_CHRC_INDICATE) {
		return acs_prot_resource_rsp_indicate(req);
	}

	return acs_prot_resource_rsp_notify(req);
}

void acs_prot_resource_req_abort_all(struct bt_acs_conn *acs_conn)
{
	struct bt_acs_prot_resource_req *req;
	uint16_t req_count = 0U;
	struct k_work_sync req_sync;
	uint16_t queued_count = 0U;

	if (!acs_conn) {
		return;
	}

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	atomic_ptr_set(&acs_conn->active_indication, NULL);

	/* Flush pending indications from the send FIFO, releasing their TX refs. */
	{
		sys_snode_t *snode;

		while ((snode = k_fifo_get(&acs_conn->indicate_fifo, K_NO_WAIT)) != NULL) {
			struct bt_acs_prot_resource_req *queued =
				CONTAINER_OF(snode, struct bt_acs_prot_resource_req, node);

			acs_prot_resource_req_tx_done(queued);
			queued_count++;
		}
	}
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION */

	/* Atomically detach each request, cancel its work item, and drop held references. */
	for (uint8_t i = 0; i < CONFIG_BT_ACS_MAX_INFLIGHT_REQ_PER_CONN; i++) {
		req = atomic_ptr_set(&acs_conn->pending_reqs[i], NULL);
		if (!req) {
			continue;
		}

		k_work_cancel_sync(&req->work, &req_sync);
		req_count++;
		req->acs_conn = NULL;

		/* Drop each held reference using the unified API */
		if (atomic_test_bit(req->ref_flags, PROT_RESOURCE_REF_ALLOC)) {
			acs_prot_resource_req_unref(req, PROT_RESOURCE_REF_ALLOC);
		}
		if (atomic_test_bit(req->ref_flags, PROT_RESOURCE_REF_TX)) {
			acs_prot_resource_req_unref(req, PROT_RESOURCE_REF_TX);
		}
	}

	if (req_count > 0U || queued_count > 0U) {
		LOG_DBG("Aborted %u ACS request(s), dropped %u queued response(s)", req_count,
			queued_count);
	}
}

static void acs_req_work_handler(struct k_work *work)
{
	struct bt_acs_prot_resource_req *req =
		CONTAINER_OF(work, struct bt_acs_prot_resource_req, work);
	struct bt_conn const *conn = acs_prot_resource_req_conn(req);
	bool handled = false;
	int err = 0;
	uint16_t h;

	if (!conn || !req->acs_conn) {
		acs_prot_resource_req_release_owner(req);
		return;
	}

	STRUCT_SECTION_FOREACH(bt_acs_prot_resource_handler_entry, entry) {
		if (!entry->handler) {
			continue;
		}

		h = acs_rhandle_find_char_handle(entry->char_uuid);
		if (h != 0U && h == req->resource_handle) {
			entry->handler(req);
			handled = true;
			break;
		}
	}

	if (!handled) {
		err = acs_auto_respond(req);
		if (err) {
			LOG_WRN("ACS auto-response failed for handle 0x%04x: %d",
				req->resource_handle, err);
		}
	}

	acs_prot_resource_req_release_owner(req);
}
