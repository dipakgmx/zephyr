/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>

#include "acs_internal.h"
#include "acs_protected_resource_router.h"
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#include "acs_rmap.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
static bool router_char_match_cb(const struct bt_acs_rmap_protected *prot, void *user_data)
{
	uint16_t *target = user_data;

	if (prot->resource_handle == *target) {
		*target = 0;
		return false;
	}
	return true;
}

struct router_cp_match_ctx {
	uint16_t resource_handle;
	bool found;
};

static bool router_cp_match_cb(const struct bt_acs_rmap_protected *entry, void *user_data)
{
	struct router_cp_match_ctx *ctx = user_data;

	if (entry->resource_handle == ctx->resource_handle) {
		ctx->found = true;
		return false;
	}
	return true;
}

static bool router_handle_in_chars(uint16_t map_id, uint16_t resource_handle)
{
	struct bt_acs_restriction_map map;
	uint16_t target = resource_handle;

	if (acs_rmap_lookup(map_id, &map) != 0) {
		return false;
	}
	acs_rmap_foreach_char(&map, router_char_match_cb, &target);
	return target == 0;
}

static bool router_handle_in_cps(uint16_t map_id, uint16_t resource_handle)
{
	struct bt_acs_restriction_map map;
	struct router_cp_match_ctx ctx;

	if (acs_rmap_lookup(map_id, &map) != 0) {
		return false;
	}

	ctx.resource_handle = resource_handle;
	ctx.found = false;
	acs_rmap_foreach_cp(&map, router_cp_match_cb, &ctx);
	return ctx.found;
}

#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

int acs_classify_frame(const struct acs_frame *frame, uint16_t map_id, struct acs_route *route)
{
	enum acs_route_kind kind;
	if (!frame || !route) {
		return -EINVAL;
	}

	if (frame->source_channel == ACS_SRC_CP) {
		kind = ACS_ROUTE_ACS_CP;
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	} else if (router_handle_in_cps(map_id, frame->resource_handle)) {
		kind = ACS_ROUTE_PROTECTED_SERVICE_CP;
	} else if (router_handle_in_chars(map_id, frame->resource_handle)) {
		kind = ACS_ROUTE_PROTECTED_CHAR;
#else
		ARG_UNUSED(map_id);
#endif
	} else {
		LOG_WRN("Frame source channel %d is not CP or Data In", frame->source_channel);
		return -ENOENT;
	}

	switch (kind) {
	case ACS_ROUTE_ACS_CP:
		route->kind = kind;
		break;
	case ACS_ROUTE_PROTECTED_SERVICE_CP:
	case ACS_ROUTE_PROTECTED_CHAR:
		route->kind = kind;
		route->resource_handle = frame->resource_handle;
		route->isc_id = frame->isc_id;
		route->encrypted = true;
		break;
	default:
		__ASSERT_NO_MSG(false);
		break;
	}
	return 0;
}
