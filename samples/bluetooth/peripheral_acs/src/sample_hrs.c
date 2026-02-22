/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/services/hrs.h>
#include <zephyr/bluetooth/services/acs.h>

#include "sample_acs_isc.h"
#include "sample_hrs.h"

/* Heart Rate Measurement Flags, bit 3 (HRS 1.0 §3.1.1.1). */
#define HRS_FLAG_ENERGY_EXPENDED_PRESENT 0x08U

/* Both maps carry the same resource policy. */
BT_ACS_RMAP_DECLARE_CHAR_OPS(hrs_body_sensor_public, SAMPLE_ACS_RMAP_PUBLIC_ID,
			     BT_UUID_HRS_BODY_SENSOR, SAMPLE_ACS_ISC_ID,
			     BT_ACS_RMAP_OP_ATT_READ_REQ);
BT_ACS_RMAP_DECLARE_CHAR_OPS(hrs_body_sensor_secret, SAMPLE_ACS_RMAP_SECRET_ID,
			     BT_UUID_HRS_BODY_SENSOR, SAMPLE_ACS_ISC_ID,
			     BT_ACS_RMAP_OP_ATT_READ_REQ);

BT_ACS_RMAP_DECLARE_CHAR_OPS(hrs_measurement_public, SAMPLE_ACS_RMAP_PUBLIC_ID,
			     BT_UUID_HRS_MEASUREMENT, SAMPLE_ACS_ISC_ID,
			     BT_ACS_RMAP_OP_ATT_NOTIFY);
BT_ACS_RMAP_DECLARE_CHAR_OPS(hrs_measurement_secret, SAMPLE_ACS_RMAP_SECRET_ID,
			     BT_UUID_HRS_MEASUREMENT, SAMPLE_ACS_ISC_ID,
			     BT_ACS_RMAP_OP_ATT_NOTIFY);

BT_ACS_RMAP_DECLARE_EXTERNAL_CP_OPS(hrs_cp_public, SAMPLE_ACS_RMAP_PUBLIC_ID,
				    BT_UUID_HRS_CONTROL_POINT, SAMPLE_ACS_ISC_ID,
				    BT_HRS_CONTROL_POINT_RESET_ENERGY_EXPANDED_REQ);
BT_ACS_RMAP_DECLARE_EXTERNAL_CP_OPS(hrs_cp_secret, SAMPLE_ACS_RMAP_SECRET_ID,
				    BT_UUID_HRS_CONTROL_POINT, SAMPLE_ACS_ISC_ID,
				    BT_HRS_CONTROL_POINT_RESET_ENERGY_EXPANDED_REQ);

static uint32_t energy_expended;

static int hrs_ctrl_point_write(uint8_t request)
{
	if (request == BT_HRS_CONTROL_POINT_RESET_ENERGY_EXPANDED_REQ) {
		energy_expended = 0;
		printk("HRS: Energy Expended reset by client\n");
		return 0;
	}
	return -ENOTSUP;
}

static void hrs_ntf_changed(bool enabled)
{
	printk("HRS notifications %s\n", enabled ? "enabled" : "disabled");
}

static struct bt_hrs_cb hrs_cb = {
	.ctrl_point_write = hrs_ctrl_point_write,
	.ntf_changed = hrs_ntf_changed,
};

int sample_hrs_init(void)
{
	return bt_hrs_cb_register(&hrs_cb);
}

void sample_hrs_notify(uint16_t bpm, uint32_t ee_kj)
{
	uint8_t measurement[4];

	energy_expended = MIN(energy_expended + ee_kj, UINT16_MAX);

	measurement[0] = HRS_FLAG_ENERGY_EXPENDED_PRESENT;
	measurement[1] = (uint8_t)bpm;
	sys_put_le16((uint16_t)energy_expended, &measurement[2]);

	(void)bt_acs_notify_uuid(NULL, BT_UUID_HRS_MEASUREMENT, measurement, sizeof(measurement));
}
