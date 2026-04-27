/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_WIRE_CONSTANTS_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_WIRE_CONSTANTS_H_

#include <stdint.h>

#include <zephyr/sys/util.h>

/*
 * ACS segmentation header layout:
 * bit 0: first segment
 * bit 1: last segment
 * bit 2..7: rolling segment counter
 */
#define ACS_SEG_FIRST_MASK   BIT(0)
#define ACS_SEG_LAST_MASK    BIT(1)
#define ACS_SEG_COUNTER_MASK GENMASK(7, 2)
#define ACS_SEG_COUNTER_MAX  64U
#define ACS_SEG_HEADER_SIZE  1U
#define ACS_SEG_SINGLE_PDU   (ACS_SEG_FIRST_MASK | ACS_SEG_LAST_MASK)

/* ATT opcode(1) + handle(2) */
#define ACS_SEG_ATT_HDR_SIZE 3U

/* ISC_ID at the front of the reassembled Data In payload. */
#define ACS_DATA_IN_HDR_SIZE 2U

/* ACS Status characteristic payload: flags(1) + active map id(2). */
#define ACS_STATUS_SIZE 3U

/* ACS-specific ATT error used for out-of-sequence segment counters. */
#define BT_ACS_ATT_ERR_INVALID_KEY         0x80U
#define BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED 0x81U
#define BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIGURATION 0x82U
#define BT_ACS_ATT_ERR_INVALID_SEG_COUNTER 0x83U

/*
 * Conservative working buffer size for the experimental stack. This matches
 * the Kconfig-owned logical segment ceiling rather than a fixed constant.
 */
#define ACS_BUF_SIZE MAX(CONFIG_BT_ACS_MAX_SEGMENT_SIZE, CONFIG_BT_ACS_CP_BUF_SIZE)

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_SERVICES_ACS_EXPERIMENTAL_WIRE_CONSTANTS_H_ */
