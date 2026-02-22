/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_DESCS_H_
#define ACS_DESCS_H_

/* Get All Active Descriptors procedure (§4.4.3.1). */

#include "acs_types.h"
#include "acs_cp_operands.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Handle the Get All Active Descriptors procedure (§4.4.3.1). */
struct acs_cp_result acs_cp_all_active_get(struct acs_reply *reply, struct net_buf_simple *payload);

/* Build the ISC response in the Get All Active Descriptors sequence. */
int acs_all_active_step_isc(struct acs_reply *reply);

/* Build the Key Descriptor response in the sequence. */
int acs_all_active_step_key(struct acs_reply *reply);

/* Finish the sequence with a successful Response Code. */
int acs_all_active_finish_success(struct acs_reply *reply);

#ifdef __cplusplus
}
#endif

#endif /* ACS_DESCS_H_ */
