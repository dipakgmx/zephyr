/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>

#include "acs_cp.h"
#include "acs_internal.h"
#include "acs_isc.h"
#include "acs_key_desc.h"
#include "acs_rhandle.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if IS_ENABLED(CONFIG_BT_ACS_ANY_KEY_EXCHANGE)
int acs_cp_handle_get_key_descriptor(struct acs_procedure *proc, struct net_buf_simple *buf)
{
	int err = acs_key_desc_build_response(buf, &proc->buffers.response_buf->b, proc->acs_conn);

	if (err) {
		return errno_to_acs_status(err);
	}
	return ACS_CP_RESULT_STAGED_REPLY;
}
#endif /* CONFIG_BT_ACS_ANY_KEY_EXCHANGE */

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
int acs_cp_handle_get_isc_descriptor(struct acs_procedure *proc, struct net_buf_simple *buf)
{
	int err = acs_isc_build_response(buf, &proc->buffers.response_buf->b);

	if (err) {
		return errno_to_acs_status(err);
	}
	return ACS_CP_RESULT_STAGED_REPLY;
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

#if IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP)
int acs_cp_handle_get_resource_handle_uuid_map(struct acs_procedure *proc)
{
	int err = acs_rhandle_build_map_response(&proc->buffers.response_buf->b);

	if (err) {
		return errno_to_acs_status(err);
	}
	return ACS_CP_RESULT_STAGED_REPLY;
}
#endif /* CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP */

int acs_cp_handle_get_svc_char_uuids(struct acs_procedure *proc, struct net_buf_simple *buf)
{
	uint16_t resource_handle = net_buf_simple_pull_le16(buf);
	int err;

	err = acs_rhandle_lookup_svc_char(resource_handle, &proc->buffers.response_buf->b);

	if (err == -ENOENT) {
		return BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE;
	} else if (err) {
		return errno_to_acs_status(err);
	}
	return ACS_CP_RESULT_STAGED_REPLY;
}
