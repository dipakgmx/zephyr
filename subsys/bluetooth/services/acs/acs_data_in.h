/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_DATA_IN_H_
#define BT_GATT_ACS_DATA_IN_H_

struct bt_acs_conn;

#include <stdint.h>

/*
 * Authenticate, decrypt, and route the Data In payload reassembled in
 * acs_conn->data_rx. Return the ATT error that rejects the write (§4.3.2), or
 * BT_ATT_ERR_SUCCESS.
 */
uint8_t acs_data_in_unwrap_and_route(struct bt_acs_conn *acs_conn);

#endif /* BT_GATT_ACS_DATA_IN_H_ */
