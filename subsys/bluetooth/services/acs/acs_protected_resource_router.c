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

enum router_hit {
	ROUTER_HIT_NONE,
	ROUTER_HIT_CP,
	ROUTER_HIT_CHAR,
};

struct router_match_ctx {
	uint16_t resource_handle;
	bool found;
};

static bool router_match_cb(const struct bt_acs_rmap_protected *entry, void *user_data)
{
	struct router_match_ctx *ctx = user_data;

	if (entry->resource_handle == ctx->resource_handle) {
		ctx->found = true;
		return false;
	}
	return true;
}

/* One pass, one lookup: classify @p resource_handle as CP, characteristic,
 * or absent in the active restriction map.
 */
static enum router_hit router_classify_handle(uint16_t map_id, uint16_t resource_handle)
{
	struct bt_acs_restriction_map map;
	struct router_match_ctx ctx = { .resource_handle = resource_handle };

	if (acs_rmap_lookup(map_id, &map) != 0) {
		return ROUTER_HIT_NONE;
	}

	acs_rmap_foreach_cp(&map, router_match_cb, &ctx);
	if (ctx.found) {
		return ROUTER_HIT_CP;
	}

	acs_rmap_foreach_char(&map, router_match_cb, &ctx);
	if (ctx.found) {
		return ROUTER_HIT_CHAR;
	}

	return ROUTER_HIT_NONE;
}

#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

int acs_classify_frame(const struct acs_frame *frame, uint16_t map_id, struct acs_route *route)
{
	if (!frame || !route) {
		return -EINVAL;
	}

	if (frame->source_channel == ACS_SRC_CP) {
		*route = (struct acs_route){ .kind = ACS_ROUTE_ACS_CP };
		return 0;
	}

#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	enum router_hit hit = router_classify_handle(map_id, frame->resource_handle);
	enum acs_route_kind kind;

	switch (hit) {
	case ROUTER_HIT_CP:
		kind = ACS_ROUTE_PROTECTED_SERVICE_CP;
		break;
	case ROUTER_HIT_CHAR:
		kind = ACS_ROUTE_PROTECTED_CHAR;
		break;
	default:
		LOG_WRN("Data In handle 0x%04x not in restriction map %u",
			frame->resource_handle, map_id);
		return -ENOENT;
	}

	*route = (struct acs_route){
		.kind = kind,
		.resource_handle = frame->resource_handle,
		.isc_id = frame->isc_id,
		.encrypted = true,
	};
	return 0;
#else
	ARG_UNUSED(map_id);
	LOG_WRN("Data In received but authorization disabled");
	return -ENOENT;
#endif
}
