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

/* ACS write errors converted to ATT errors by acs_write_err_to_att(). */
#define ACS_DATA_ERR_CCC_IMPROPER_CONF      (-EPIPE)  /* -> BT_ATT_ERR_CCC_IMPROPER_CONF */
#define ACS_DATA_ERR_INVALID_KEY            (-EACCES) /* -> BT_ACS_ATT_ERR_INVALID_KEY (0x80) */
#define ACS_DATA_ERR_RESOURCE_NOT_PROTECTED (-ENOENT) /* -> ..._RESOURCE_NOT_PROTECTED (0x81) */
#define ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG                                                     \
	(-EPROTO)                                  /* -> ..._INCORRECT_SECURITY_CONFIG (0x82) */
#define ACS_DATA_ERR_INVALID_SEG_COUNTER (-EILSEQ) /* -> ..._INVALID_SEG_COUNTER (0x83) */
#define ACS_DATA_ERR_NO_RESOURCES        (-ENOMEM) /* -> BT_ATT_ERR_INSUFFICIENT_RESOURCES */

/* GATT write handler for the ACS Control Point characteristic. */
ssize_t acs_cp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
		     uint16_t len, uint16_t offset, uint8_t flags);

/* GATT write handler for the ACS Data In characteristic. */
ssize_t acs_data_in_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags);

/* Authenticate, decrypt, and route a reassembled Data In payload. */
int acs_data_in_unwrap_and_route(struct bt_acs_conn *acs_conn, struct net_buf *buf);

/* Classify a reassembled Data In frame and dispatch it to the matching handler. */
int acs_runtime_dispatch_frame(const struct acs_frame *frame, struct bt_acs_conn *acs_conn);

#ifdef __cplusplus
}
#endif

#endif /* ACS_RUNTIME_H_ */
