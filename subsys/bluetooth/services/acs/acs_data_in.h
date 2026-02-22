/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_DATA_IN_H_
#define BT_GATT_ACS_DATA_IN_H_

struct bt_acs_conn;
struct net_buf;

#include <stdint.h>

/*
 * Authenticate, decrypt, and route a reassembled Data In payload. Return the
 * ATT error that rejects the write (§4.3.2), or BT_ATT_ERR_SUCCESS.
 */
uint8_t acs_data_in_unwrap_and_route(struct bt_acs_conn *acs_conn, struct net_buf *buf);

#endif /* BT_GATT_ACS_DATA_IN_H_ */
