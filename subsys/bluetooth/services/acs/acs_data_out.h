/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_DATA_OUT_H_
#define ACS_DATA_OUT_H_

/* Protected output over Data Out Notify and Data Out Indicate (§4.3). */

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
/* Initialize the per-connection Data Out Notify and Data Out Indicate channels. */
void acs_data_out_init_conn(struct bt_acs_conn *acs_conn);
#endif

/*
 * Report the outcome of the protected notification or indication reply carries
 * to its GATT callbacks; err is 0 when it was delivered. A reply without one has
 * no callback and is left alone.
 */
void acs_data_out_complete(struct acs_reply *reply, int err);

/* Send notifications and indications of protected resources on ACS Data Out (§4.3.2). */
int acs_data_out_router_register(void);

#ifdef __cplusplus
}
#endif

#endif /* ACS_DATA_OUT_H_ */
