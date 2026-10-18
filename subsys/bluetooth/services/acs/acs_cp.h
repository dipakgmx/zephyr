/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_CP_H_
#define ACS_CP_H_

/* ACS Control Point procedures (§4.4.3). */

#include <stdbool.h>
#include <stdint.h>

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Check and queue a plain Control Point write. Always reset its RX context.
 * Return the ATT error that rejects the write, or BT_ATT_ERR_SUCCESS.
 */
uint8_t acs_cp_queue_plain(struct bt_acs_conn *acs_conn);

/*
 * Check and queue the protected Control Point request in reply->request,
 * received under reply->isc_id. Free reply on error.
 * Return the ATT error that rejects the write, or BT_ATT_ERR_SUCCESS.
 */
uint8_t acs_cp_queue_protected(struct acs_reply *reply);

/* Build and submit a Response Code for req_opcode. */
int acs_cp_rsp_status(struct acs_reply *reply, uint8_t req_opcode, uint8_t code);

/* Indicate the current ATT_MTU on the ACS Control Point after an MTU change. */
void acs_cp_indicate_att_mtu(struct bt_acs_conn *acs_conn);

/* Lock the Control Point for reply. Return false if it is already locked. */
bool acs_cp_lock(struct acs_reply *reply);

/* Unlock the Control Point if reply holds its lock. */
void acs_cp_unlock(struct acs_reply *reply);

/* Initialize the per-connection ACS Control Point execution queue. */
void acs_cp_exec_queue_init(struct bt_acs_conn *acs_conn);

#ifdef __cplusplus
}
#endif

#endif /* ACS_CP_H_ */
