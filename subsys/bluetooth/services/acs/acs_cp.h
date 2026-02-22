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

/* Check and queue a plain Control Point write. Always reset its RX context. */
int acs_cp_queue_plain(struct bt_acs_conn *acs_conn);

/* Check and queue a protected Control Point request. Free reply on error. */
int acs_cp_queue_protected(const struct acs_frame *frame, struct bt_acs_conn *acs_conn,
			   struct acs_reply *reply);

/* Execute a queued Control Point procedure and release its request buffer. */
int acs_cp_execute(const struct acs_frame *frame, struct bt_acs_conn *acs_conn,
		   struct acs_reply *reply);

/* Build and submit a Response Code for req_opcode. */
int acs_cp_rsp_status(struct acs_reply *reply, uint8_t req_opcode, uint8_t code);

/* Indicate the current ATT_MTU on the ACS Control Point after an MTU change. */
void acs_cp_indicate_att_mtu(struct bt_acs_conn *acs_conn);

/* Lock the Control Point for reply. Return false if it is already locked. */
bool acs_cp_lock(struct bt_acs_conn *conn, struct acs_reply *reply);

/* Unlock the Control Point if reply holds its lock. */
void acs_cp_unlock(struct bt_acs_conn *conn, struct acs_reply *reply);

/* Initialize the per-connection ACS Control Point execution queue. */
void acs_cp_exec_queue_init(struct bt_acs_conn *acs_conn);

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
/*
 * Return true when the requested procedure must use Data In under active_rmap
 * (§3.1).
 */
bool acs_cp_plain_requires_data_in(const struct bt_acs_rmap_runtime *active_rmap,
				   const uint8_t *payload, uint16_t payload_len);
#else
static inline bool acs_cp_plain_requires_data_in(const struct bt_acs_rmap_runtime *active_rmap,
						 const uint8_t *payload, uint16_t payload_len)
{
	ARG_UNUSED(active_rmap);
	ARG_UNUSED(payload);
	ARG_UNUSED(payload_len);
	return false;
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* ACS_CP_H_ */
