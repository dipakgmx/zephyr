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

static void acs_req_work_handler(struct k_work *work);

void acs_request_queue_init(struct bt_acs_conn *acs_conn)
{
	__ASSERT_NO_MSG(acs_conn != NULL);

	k_fifo_init(&acs_conn->request_fifo);
	k_work_init(&acs_conn->request_work, acs_req_work_handler);
}

void acs_request_queue_submit(struct bt_acs_conn *acs_conn, struct acs_reply *reply)
{
	__ASSERT_NO_MSG(acs_conn != NULL);
	__ASSERT_NO_MSG(reply != NULL);

	k_fifo_put(&acs_conn->request_fifo, reply);
	k_work_submit_to_queue(acs_get_wq(), &acs_conn->request_work);
}

static int acs_auto_respond(struct acs_reply *reply)
{
	const uint8_t *data = reply->request ? reply->request->data : NULL;
	uint16_t len = reply->request ? reply->request->len : 0U;
	ssize_t n;
	uint8_t props = 0;
	struct bt_conn *conn = reply->conn->conn;
	uint16_t resource_handle = reply->resource_handle;
	struct acs_char_attr_ctx ctx = {
		.value_handle = resource_handle,
		.decl = NULL,
		.value = NULL,
	};

	bt_gatt_foreach_attr(resource_handle - 1U, resource_handle, acs_find_char_attrs_cb, &ctx);

	if (!ctx.value) {
		LOG_WRN("auto_respond: no GATT attr at handle 0x%04x", resource_handle);
		return -ENOENT;
	}

	if (ctx.decl && ctx.decl->user_data) {
		props = ((const struct bt_gatt_chrc *)ctx.decl->user_data)->properties;
	}

	if (reply->access == ACS_REQ_ACCESS_WRITE) {
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
		reply->channel = (props & BT_GATT_CHRC_INDICATE) ? ACS_REPLY_DOI : ACS_REPLY_DON;
		struct net_buf *rsp_buf = acs_prepare_reply_buf(reply);

		if (!rsp_buf) {
			LOG_ERR("auto_respond: response pool exhausted "
				"(handle 0x%04x)",
				resource_handle);
			return -ENOMEM;
		}

		if (props & BT_GATT_CHRC_READ) {
			if (!ctx.value->read) {
				LOG_WRN("auto_respond: attr 0x%04x has no read "
					"handler",
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

		return acs_reply_submit(reply);
	}
}

static void acs_req_work_handler(struct k_work *work)
{
	struct bt_acs_conn *acs_conn = CONTAINER_OF(work, struct bt_acs_conn, request_work);
	sys_snode_t *snode;

	while ((snode = k_fifo_get(&acs_conn->request_fifo, K_NO_WAIT)) != NULL) {
		struct acs_reply *reply = CONTAINER_OF(snode, struct acs_reply, node);
		int err;

		if (!reply->conn || !reply->conn->conn) {
			acs_reply_free(reply);
			continue;
		}

		err = acs_auto_respond(reply);
		if (err) {
			LOG_ERR("ACS auto-response failed for handle 0x%04x: %d",
				reply->resource_handle, err);
			acs_reply_free(reply);
			continue;
		}

		if (reply->request) {
			acs_buf_free(reply->request);
			reply->request = NULL;
		}
	}
}
