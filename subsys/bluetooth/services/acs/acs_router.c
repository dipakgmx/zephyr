/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>

#include "acs_internal.h"
#include "acs_router.h"
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#include "acs_rmap.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

int acs_router_classify_frame(const struct acs_frame *frame, uint16_t map_id,
			      struct acs_route *route)
{
	__ASSERT_NO_MSG(frame != NULL);
	__ASSERT_NO_MSG(route != NULL);

	if (frame->source_channel == ACS_SRC_CP) {
		route->kind = ACS_ROUTE_ACS_CP;
		return 0;
	}

#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	enum acs_rmap_resource_kind hit;
	const struct bt_acs_rmap_protected *entry;

	if (acs_rmap_find_protected(map_id, frame->resource_handle, &hit, &entry) != 0) {
		LOG_WRN("Data In handle 0x%04x not in restriction map %u", frame->resource_handle,
			map_id);
		return -ENOENT;
	}
	ARG_UNUSED(entry);

	switch (hit) {
	case ACS_RMAP_RESOURCE_CP:
		route->kind = ACS_ROUTE_PROTECTED_SERVICE_CP;
		break;
	case ACS_RMAP_RESOURCE_CHAR:
		route->kind = ACS_ROUTE_PROTECTED_CHAR;
		break;
	default:
		LOG_WRN("Data In handle 0x%04x not in restriction map %u", frame->resource_handle,
			map_id);
		return -ENOENT;
	}

	route->encrypted = true;
	route->resource_handle = frame->resource_handle;
	route->isc_id = frame->isc_id;

	return 0;
#else
	ARG_UNUSED(map_id);
	LOG_WRN("Data In received but authorization disabled");
	return -ENOENT;
#endif
}
