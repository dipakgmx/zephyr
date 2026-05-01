/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_PROTECTED_RESOURCE_ROUTER_H_
#define ACS_PROTECTED_RESOURCE_ROUTER_H_

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Classify a normalized inbound frame.
 *
 * For ACS CP source: always @ref ACS_ROUTE_ACS_CP.
 *
 * For Data In source (post-unwrap): looks up @c frame->resource_handle in the
 * active restriction map and returns @ref ACS_ROUTE_PROTECTED_SERVICE_CP if it
 * matches a Protected CP entry, @ref ACS_ROUTE_PROTECTED_CHAR if it matches a
 * Protected Characteristic entry, or -ENOENT if the handle is not in the map.
 *
 * This function does not allocate state and does not perform any send-side or
 * permission checks; the caller is responsible for those.
 *
 * @param frame    Normalized inbound frame (must already be post-unwrap for Data In).
 * @param map_id   Active restriction map ID (used only for Data In classification).
 * @param route    [out] Populated on success.
 *
 * @return 0 on success, -ENOENT if the handle is not in the active map,
 *         -EINVAL on bad input.
 */
int acs_classify_frame(const struct acs_frame *frame, uint16_t map_id, struct acs_route *route);

#ifdef __cplusplus
}
#endif

#endif /* ACS_PROTECTED_RESOURCE_ROUTER_H_ */
