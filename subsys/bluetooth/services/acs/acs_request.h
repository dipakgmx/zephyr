/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_REQUEST_H_
#define ACS_REQUEST_H_

/* Protected requests executed on the ACS workqueue. */

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the per-connection protected-request queue. */
void acs_request_queue_init(struct bt_acs_conn *acs_conn);

/* Queue a protected request on the ACS workqueue. */
void acs_request_queue_submit(struct bt_acs_conn *acs_conn, struct acs_reply *reply);

#ifdef __cplusplus
}
#endif

#endif /* ACS_REQUEST_H_ */
