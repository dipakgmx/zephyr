/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "acs_internal.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/** Start hook for ACS-owned CP procedures. */
static int acs_cp_procedure_start(struct acs_procedure *proc, const struct acs_frame *frame);
/** Start hook for protected characteristic procedures. */
static int acs_char_procedure_start(struct acs_procedure *proc, const struct acs_frame *frame);
/** Start hook for protected service-owned control-point procedures. */
static int acs_service_cp_procedure_start(struct acs_procedure *proc,
					  const struct acs_frame *frame);
/** Default confirm hook for single-confirm stub procedures. */
static int acs_procedure_on_confirm_default(struct acs_procedure *proc);
/** Default destroy hook for stateless stub procedures. */
static void acs_procedure_destroy_default(struct acs_procedure *proc);

static const struct acs_proc_ops cp_ops = {
	.start = acs_cp_procedure_start,
	.on_confirm = acs_procedure_on_confirm_default,
	.destroy = acs_procedure_destroy_default,
};

static const struct acs_proc_ops char_ops = {
	.start = acs_char_procedure_start,
	.on_confirm = acs_procedure_on_confirm_default,
	.destroy = acs_procedure_destroy_default,
};

static const struct acs_proc_ops service_cp_ops = {
	.start = acs_service_cp_procedure_start,
	.on_confirm = acs_procedure_on_confirm_default,
	.destroy = acs_procedure_destroy_default,
};

/** See @ref acs_protected_resource_route_frame. */
int acs_classify_frame(const struct acs_frame *frame, struct acs_route *route)
{
	uint16_t acs_cp_handle;

	if (!frame || !route) {
		return -EINVAL;
	}

	route->resource_handle = frame->resource_handle;
	route->isc_id = frame->isc_id;
	route->encrypted = frame->encrypted;

	if (frame->source_channel == ACS_SOURCE_CP_CHANNEL) {
		route->kind = ACS_ROUTE_ACS_CP;
		return 0;
	}

	acs_cp_handle = acs_service_handle_cp();
	if (acs_cp_handle != 0U && frame->resource_handle == acs_cp_handle) {
		route->kind = ACS_ROUTE_ACS_CP;
		return 0;
	}

	if ((frame->resource_handle & 0x1U) == 0U) {
		route->kind = ACS_ROUTE_PROTECTED_CHAR;
	} else {
		route->kind = ACS_ROUTE_PROTECTED_SERVICE_CP;
	}

	return 0;
}

/** See @ref acs_protected_resource_build_procedure. */
int acs_build_procedure_for_route(const struct acs_frame *frame,
					   const struct acs_route *route,
					   struct acs_procedure *proc)
{
	if (!frame || !route || !proc) {
		return -EINVAL;
	}

	proc->conn = frame->conn;
	proc->resource_handle = route->resource_handle;
	proc->isc_id = route->isc_id;
	proc->flags = route->encrypted ? ACS_PROC_FLAG_SECURE_TRANSPORT : 0U;
	proc->status = ACS_PROC_IDLE;

	switch (route->kind) {
	case ACS_ROUTE_ACS_CP:
		proc->ops = &cp_ops;
		break;
	case ACS_ROUTE_PROTECTED_CHAR:
		proc->ops = &char_ops;
		break;
	case ACS_ROUTE_PROTECTED_SERVICE_CP:
		proc->ops = &service_cp_ops;
		break;
	default:
	__ASSERT_NO_MSG(false);
		return -EINVAL;
	}

	return 0;
}


/** ACS CP procedure start dispatches into the CP domain handler. */
static int acs_cp_procedure_start(struct acs_procedure *proc, const struct acs_frame *frame)
{
	uint8_t opcode = frame->payload_len > 0U ? frame->payload[0] : 0U;

	LOG_DBG("route: ACS CP opcode=0x%02x", opcode);
	return acs_cp_domain_handle(proc, frame);
}

/** Stub protected-characteristic procedure start using the service adapter. */
static int acs_char_procedure_start(struct acs_procedure *proc, const struct acs_frame *frame)
{
	LOG_DBG("route: protected char handle=0x%04x", frame->resource_handle);
	return acs_service_adapter_dispatch(proc, frame);
}

/** Stub protected service-owned CP proacs_protected_resource_route_framecedure start. */
static int acs_service_cp_procedure_start(struct acs_procedure *proc,
					  const struct acs_frame *frame)
{
	LOG_DBG("route: protected service CP handle=0x%04x", frame->resource_handle);
	return acs_service_adapter_dispatch(proc, frame);
}

/** Default confirm completion for one-reply stub procedures. */
static int acs_procedure_on_confirm_default(struct acs_procedure *proc)
{
	ARG_UNUSED(proc);
	return ACS_PROC_RES_COMPLETE;
}

/** Default destroy hook for procedures with no private state. */
static void acs_procedure_destroy_default(struct acs_procedure *proc)
{
	ARG_UNUSED(proc);
}
