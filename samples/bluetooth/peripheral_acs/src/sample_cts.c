/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/services/cts.h>
#include <zephyr/bluetooth/services/acs.h>

#include "sample_acs_isc.h"
#include "sample_cts.h"

/* The ISC differs per operation, so the shared-ISC shorthand cannot express
 * this. Read is stated as ISC 0 rather than omitted so a client need not infer
 * it. Both maps carry the same resource policy.
 */
#define CTS_CURRENT_TIME_OPS                                                                       \
	BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_READ_REQ, BT_ACS_ISC_ID_NONE),                     \
	BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_WRITE_REQ, SAMPLE_ACS_ISC_ID),                     \
	BT_ACS_RMAP_OP_ENTRY(BT_ACS_RMAP_OP_ATT_NOTIFY, SAMPLE_ACS_ISC_ID)

BT_ACS_RMAP_DECLARE_CHAR(cts_public, SAMPLE_ACS_RMAP_PUBLIC_ID, BT_UUID_CTS_CURRENT_TIME,
			 CTS_CURRENT_TIME_OPS);
BT_ACS_RMAP_DECLARE_CHAR(cts_secret, SAMPLE_ACS_RMAP_SECRET_ID, BT_UUID_CTS_CURRENT_TIME,
			 CTS_CURRENT_TIME_OPS);

static struct bt_cts_time_format stored_time;
static bool time_is_set;

static int fill_current_cts_time(struct bt_cts_time_format *cts_time)
{
	if (time_is_set) {
		*cts_time = stored_time;
		return 0;
	}

	static uint8_t seconds;

	cts_time->year = sys_cpu_to_le16(2025);
	cts_time->mon = 1;
	cts_time->mday = 15;
	cts_time->hours = 12;
	cts_time->min = 0;
	cts_time->sec = seconds++;
	cts_time->wday = 3;
	cts_time->fractions256 = 0;
	cts_time->reason = BT_CTS_UPDATE_REASON_UNKNOWN;

	if (seconds >= 60) {
		seconds = 0;
	}

	return 0;
}

static int cts_time_write_cb(struct bt_cts_time_format *cts_time)
{
	stored_time = *cts_time;
	time_is_set = true;
	printk("CTS time updated: %04u-%02u-%02u %02u:%02u:%02u\n", sys_le16_to_cpu(cts_time->year),
	       cts_time->mon, cts_time->mday, cts_time->hours, cts_time->min, cts_time->sec);
	return 0;
}

static int fill_current_cts_local_time(struct bt_cts_local_time *local_time)
{
	local_time->timezone_offset = BT_CTS_TIMEZONE_DEFAULT_VALUE;
	local_time->dst_offset = BT_CTS_DST_OFFSET_UNKNOWN;
	return 0;
}

static const struct bt_cts_cb cts_cb = {
	.fill_current_cts_time = fill_current_cts_time,
	.cts_time_write = cts_time_write_cb,
	.fill_current_cts_local_time = fill_current_cts_local_time,
};

int sample_cts_init(void)
{
	return bt_cts_init(&cts_cb);
}

void sample_cts_notify(uint8_t reason)
{
	struct bt_cts_time_format cts_time;

	if (fill_current_cts_time(&cts_time) != 0) {
		return;
	}

	cts_time.reason = reason;
	(void)bt_acs_notify_uuid(NULL, BT_UUID_CTS_CURRENT_TIME, &cts_time, sizeof(cts_time));
}
