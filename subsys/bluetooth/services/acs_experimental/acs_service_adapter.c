/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include "acs_internal.h"
#include "acs_wire_constants.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

struct acs_attr_find_ctx {
	const struct bt_gatt_attr *attr;
	uint16_t handle;
};

static uint8_t acs_service_find_attr_cb(const struct bt_gatt_attr *attr, uint16_t handle,
					void *user_data)
{
	struct acs_attr_find_ctx *ctx = user_data;

	if (handle == ctx->handle) {
		ctx->attr = attr;
		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_CONTINUE;
}

static const struct bt_gatt_attr *acs_service_find_attr(uint16_t handle)
{
	struct acs_attr_find_ctx ctx = {
		.attr = NULL,
		.handle = handle,
	};

	bt_gatt_foreach_attr(handle, handle, acs_service_find_attr_cb, &ctx);
	return ctx.attr;
}

static int acs_service_send_read_reply(struct acs_procedure *proc, const uint8_t *data, size_t len)
{
	struct net_buf *buf;

	buf = acs_channel_buf_alloc();
	if (!buf) {
		return -ENOMEM;
	}

	if (net_buf_tailroom(buf) < len) {
		acs_channel_buf_free(buf);
		return -ENOBUFS;
	}

	net_buf_add_mem(buf, data, len);

	return acs_data_out_channel_send(proc, &(struct acs_reply){
					 .channel = ACS_REPLY_DOI,
					 .plaintext = buf,
					 .encrypted = true,
					 .needs_confirm = true,
				       });
}

static int acs_service_handle_read(struct acs_procedure *proc, const struct acs_frame *frame,
				   const struct bt_gatt_attr *attr)
{
	uint8_t value[ACS_BUF_SIZE];
	ssize_t rc;

	if (!IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_READ)) {
		return -ENOTSUP;
	}

	if (!bt_acs_policy_is_permitted(proc->conn, frame->resource_handle, BT_ACS_DIRECTION_READ)) {
		LOG_WRN("ACS protected read denied: handle=0x%04x", frame->resource_handle);
		return -EACCES;
	}

	if (!attr->read) {
		return -ENOTSUP;
	}

	rc = attr->read(proc->conn, attr, value, sizeof(value), 0U);
	if (rc < 0) {
		LOG_WRN("ACS protected read callback failed: handle=0x%04x rc=%d",
			frame->resource_handle, (int)rc);
		return -EIO;
	}

	return acs_service_send_read_reply(proc, value, (size_t)rc);
}

static int acs_service_handle_write(struct acs_procedure *proc, const struct acs_frame *frame,
				    const struct bt_gatt_attr *attr)
{
	ssize_t rc;

	if (!IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_WRITE)) {
		return -ENOTSUP;
	}

	if (!bt_acs_policy_is_permitted(proc->conn, frame->resource_handle, BT_ACS_DIRECTION_WRITE)) {
		LOG_WRN("ACS protected write denied: handle=0x%04x", frame->resource_handle);
		return -EACCES;
	}

	if (!attr->write) {
		return -ENOTSUP;
	}

	rc = attr->write(proc->conn, attr, frame->payload, frame->payload_len, 0U, 0U);
	if (rc < 0) {
		LOG_WRN("ACS protected write callback failed: handle=0x%04x rc=%d",
			frame->resource_handle, (int)rc);
		return -EIO;
	}

	return ACS_PROC_RES_COMPLETE;
}

int acs_service_adapter_dispatch(struct acs_procedure *proc, const struct acs_frame *frame)
{
	const struct bt_gatt_attr *attr;

	if (!proc || !frame) {
		return -EINVAL;
	}

	LOG_DBG("service adapter: handle=0x%04x payload_len=%u", frame->resource_handle,
		frame->payload_len);

	attr = acs_service_find_attr(frame->resource_handle);
	if (!attr) {
		LOG_WRN("ACS protected resource not found: handle=0x%04x", frame->resource_handle);
		return -ENOENT;
	}

	if (frame->payload_len == 0U) {
		return acs_service_handle_read(proc, frame, attr);
	}

	return acs_service_handle_write(proc, frame, attr);
}
