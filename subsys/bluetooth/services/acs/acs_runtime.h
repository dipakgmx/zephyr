/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_RUNTIME_H_
#define ACS_RUNTIME_H_

/* ACS Control Point and Data In write handling. */

#include <stddef.h>
#include <stdint.h>
#include <errno.h>

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/net_buf.h>

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* GATT write handler for the ACS Control Point characteristic. */
ssize_t acs_cp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
		     uint16_t len, uint16_t offset, uint8_t flags);

/* GATT write handler for the ACS Data In characteristic. */
ssize_t acs_data_in_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags);

/*
 * Classify a decrypted Data In frame and dispatch it to the matching handler.
 * Return the ATT error that rejects the write, or BT_ATT_ERR_SUCCESS.
 */
uint8_t acs_runtime_dispatch_frame(const struct acs_frame *frame, struct bt_acs_conn *acs_conn);

#ifdef __cplusplus
}
#endif

#endif /* ACS_RUNTIME_H_ */
