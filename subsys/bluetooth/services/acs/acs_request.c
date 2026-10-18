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

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

static void acs_req_work_handler(struct k_work *work);

void acs_request_queue_init(struct bt_acs_conn *acs_conn)
{
	k_fifo_init(&acs_conn->request_fifo);
	k_work_init(&acs_conn->request_work, acs_req_work_handler);
}

void acs_request_queue_submit(struct acs_reply *reply)
{
	k_fifo_put(&reply->acs_conn->request_fifo, reply);
	k_work_submit_to_queue(acs_get_wq(), &reply->acs_conn->request_work);
}

/* Run the target's write handler with the whole request value; a partial write fails. */
static int acs_request_write(struct acs_reply *reply)
{
	const struct bt_gatt_attr *value = reply->value_attr;
	ssize_t len;

	if (value->write == NULL) {
		LOG_WRN("resource 0x%04x has no write handler", reply->resource_handle);
		return -ENOTSUP;
	}

	len = value->write(reply->acs_conn->conn, value, reply->request->data, reply->request->len,
			   0, 0);
	if (len < 0) {
		LOG_ERR("write handler error %d", (int)len);
		return len;
	}

	if ((size_t)len != reply->request->len) {
		LOG_ERR("write handler accepted %d of %u bytes", (int)len, reply->request->len);
		return -EIO;
	}

	return 0;
}

static int acs_execute_protected_access(struct acs_reply *reply)
{
	const struct bt_gatt_attr *value = reply->value_attr;
	struct bt_conn *conn = reply->acs_conn->conn;
	struct net_buf *rsp_buf;
	size_t read_room;
	ssize_t len;

	/* A nonempty payload is a secure write, an empty one a secure read (Table 4.10). */
	if (reply->request->len > 0U) {
		int err = acs_request_write(reply);

		if (err) {
			return err;
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

	/* The tailroom is the message room; the MAC has its own. */
	read_room = net_buf_tailroom(rsp_buf);

	len = value->read(conn, value, net_buf_tail(rsp_buf), read_room, 0);
	if (len < 0) {
		LOG_ERR("read handler error %d", (int)len);
		return len;
	}

	/* A value longer than the room left would be sent truncated. */
	if ((size_t)len == read_room) {
		uint8_t probe;

		if (value->read(conn, value, &probe, sizeof(probe), (uint16_t)len) > 0) {
			LOG_ERR("resource 0x%04x value too long for ACS", reply->resource_handle);
			return -EMSGSIZE;
		}
	}

	net_buf_add(rsp_buf, len);

	return acs_reply_submit(reply);
}

static void acs_req_work_handler(struct k_work *work)
{
	struct bt_acs_conn *acs_conn = CONTAINER_OF(work, struct bt_acs_conn, request_work);
	sys_snode_t *snode;

	while ((snode = k_fifo_get(&acs_conn->request_fifo, K_NO_WAIT)) != NULL) {
		struct acs_reply *reply = CONTAINER_OF(snode, struct acs_reply, node);
		int err = acs_execute_protected_access(reply);

		if (err) {
			LOG_ERR("ACS protected access failed for handle 0x%04x: %d",
				reply->resource_handle, err);
		}

		/* Dispatch is synchronous; the worker retains and releases the request buffer. */
		net_buf_drop(&reply->request);

		if (err) {
			acs_reply_free(reply);
		}
	}
}
