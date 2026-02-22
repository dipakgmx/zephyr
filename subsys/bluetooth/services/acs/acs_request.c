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
#include "acs_reply.h"
#include "acs_request.h"
#include "acs_key_desc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static void acs_req_work_handler(struct k_work *work);

void acs_request_queue_init(struct bt_acs_conn *acs_conn)
{
	k_fifo_init(&acs_conn->request_fifo);
	k_work_init(&acs_conn->request_work, acs_req_work_handler);
}

void acs_request_queue_submit(struct bt_acs_conn *acs_conn, struct acs_reply *reply)
{
	k_fifo_put(&acs_conn->request_fifo, reply);
	k_work_submit_to_queue(acs_get_wq(), &acs_conn->request_work);
}

static int acs_execute_protected_access(struct acs_reply *reply)
{
	const struct bt_gatt_attr *value = reply->value_attr;
	struct bt_conn *conn = reply->conn->conn;
	struct net_buf *rsp_buf;
	uint8_t auth_tag_size;
	size_t read_room;
	ssize_t n;

	if (reply->route == ACS_ROUTE_PROTECTED_WRITE) {
		if (value->write == NULL) {
			LOG_WRN("resource 0x%04x has no write handler", reply->resource_handle);
			return -ENOTSUP;
		}

		n = value->write(conn, value, reply->request->data, reply->request->len, 0, 0);
		if (n < 0) {
			LOG_ERR("write handler error %d", (int)n);
			return n;
		}
	}

	/* reply->channel was selected before this request was queued. */
	rsp_buf = acs_reply_add_message(reply);
	if (rsp_buf == NULL) {
		LOG_ERR("response pool exhausted (resource 0x%04x)", reply->resource_handle);
		return -ENOMEM;
	}

	if (!(reply->value_props & BT_GATT_CHRC_READ)) {
		return acs_reply_submit(reply);
	}

	if (value->read == NULL) {
		LOG_WRN("resource 0x%04x has no read handler", reply->resource_handle);
		return -ENOTSUP;
	}

	/* Encryption appends the authentication tag to this buffer. */
	read_room = net_buf_tailroom(rsp_buf);
	auth_tag_size = acs_key_desc_auth_tag_size(reply->alg->key_desc);
	if (read_room < auth_tag_size) {
		LOG_ERR("no room for auth tag (resource 0x%04x)", reply->resource_handle);
		return -ENOMEM;
	}
	read_room -= auth_tag_size;

	n = value->read(conn, value, net_buf_tail(rsp_buf), read_room, 0);
	if (n < 0) {
		LOG_ERR("read handler error %d", (int)n);
		return n;
	}

	net_buf_add(rsp_buf, n);

	return acs_reply_submit(reply);
}

/*
 * External-service Control Point write: run the target's write handler with no
 * ACS Data Out response.
 */
static int acs_execute_external_cp(struct acs_reply *reply)
{
	const struct bt_gatt_attr *value = reply->value_attr;
	ssize_t written;

	if (value->write == NULL) {
		LOG_WRN("external CP 0x%04x has no write handler", reply->resource_handle);
		return -ENOTSUP;
	}

	written = value->write(reply->conn->conn, value, reply->request->data,
			       reply->request->len, 0, 0);
	if (written < 0) {
		LOG_ERR("external CP write handler error %d", (int)written);
		return written;
	}

	return 0;
}

static void acs_req_work_handler(struct k_work *work)
{
	struct bt_acs_conn *acs_conn = CONTAINER_OF(work, struct bt_acs_conn, request_work);
	sys_snode_t *snode;

	while ((snode = k_fifo_get(&acs_conn->request_fifo, K_NO_WAIT)) != NULL) {
		struct acs_reply *reply = CONTAINER_OF(snode, struct acs_reply, node);
		bool external_cp;
		int err;

		if (reply->conn->conn == NULL) {
			acs_reply_free(reply);
			continue;
		}

		external_cp = reply->route == ACS_ROUTE_PROTECTED_EXTERNAL_CP;
		if (external_cp) {
			err = acs_execute_external_cp(reply);
			if (err) {
				LOG_ERR("external CP failed for handle 0x%04x: %d",
					reply->resource_handle, err);
			}
		} else {
			err = acs_execute_protected_access(reply);
			if (err) {
				LOG_ERR("ACS protected access failed for handle 0x%04x: %d",
					reply->resource_handle, err);
			}
		}

		/* Dispatch is synchronous; the worker retains and releases the request buffer. */
		acs_buf_free(reply->request);
		reply->request = NULL;

		if (external_cp || err) {
			acs_reply_free(reply);
		}
	}
}
