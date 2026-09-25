/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_ISC_H_
#define BT_GATT_ACS_ISC_H_

#include <zephyr/types.h>
#include <zephyr/net_buf.h>

#include "acs_wire_constants.h"
#include "acs_types.h"

/* ISC record Type_ID (Table 4.31). */
#define BT_ACS_RECORD_TYPE_ISC_ID 0x00

/*
 * Reserved ISC ID used as a filter to request all ISC records (§4.4.4.12.1).
 * This value does not appear in an ISC descriptor record.
 */
#define BT_ACS_ISC_ALL_RECORDS_FILTER 0xFFFF

/*
 * Information_Security_Controls field values (Table 4.33). Each names how the
 * Protected Resource Request Or Response is protected.
 */
enum acs_sec_control_type {
	ACS_CTRL_NONCE = 0x00,       /* Nonce */
	ACS_CTRL_AUTH = 0x01,        /* Authenticated Protected Resource Request Or Response */
	ACS_CTRL_ENC = 0x02,         /* Encrypted Protected Resource Request Or Response */
	ACS_CTRL_AUTH_ENC = 0x03,    /* Authenticated And Encrypted Protected Resource Request Or Response */
	ACS_CTRL_AUTH_ENC_AD = 0x04, /* Authenticated And Encrypted Protected Resource Request Or Response With Associated Data */
	ACS_CTRL_UNENC = 0x05,       /* Unencrypted Protected Resource Request Or Response */
	ACS_CTRL_MAC = 0x06          /* MAC */
};

/* Most Information_Security_Controls in one of this server's ISC records. This is limited to 3
 * in the current implementation, for example:
 * - GCM, CCM: [Nonce, MAC, Authenticated And Encrypted] - 3 controls
 * - GMAC: [Nonce, MAC, Authenticated] - 3 controls
 * - CMAC: [MAC, Authenticated …] - 2 controls
 */
#define ACS_ISC_MAX_CONTROLS 3

/*
 * Information Security Configuration record (Table 4.32): a set of security
 * controls and the key they use.
 */
struct bt_acs_isc_record {
	uint16_t isc_id; /* See Information Security Configuration IDs (BT_ACS_ISC_ID_*) */
	uint8_t num_controls; /* Number of controls */
	uint8_t controls[ACS_ISC_MAX_CONTROLS]; /* Actual controls like mac, nonce, authenticated, encrypted, etc. */
	uint16_t key_id; /* Key_ID of the security algorithm used for this ISC record */
};

/* Find an ISC record by ISC_ID, or return NULL when this build has none. */
const struct bt_acs_isc_record *acs_isc_lookup(uint16_t isc_id);

/*
 * Add the ISC records selected by filter_id to buf. Return Success, No Records
 * Found, or Procedure Not Completed when buf is full.
 */
uint8_t acs_isc_build_response(uint16_t filter_id, struct net_buf *buf);

/* Security algorithm that isc_id uses on acs_conn, or NULL if either record is missing. */
struct acs_sec_alg *acs_isc_alg(struct bt_acs_conn *acs_conn, uint16_t isc_id);

#endif /* BT_GATT_ACS_ISC_H_ */
