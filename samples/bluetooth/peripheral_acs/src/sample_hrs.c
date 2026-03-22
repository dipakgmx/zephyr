/* sample_hrs.c - HRS service setup for the peripheral_acs sample */

/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * HRS restriction map entries:
 *
 *   HR Measurement  → Notify, HIGH_SEC:  encrypted HR stream via Data Out (DON)
 *   HR Control Point → Write, HIGH_SEC:  Reset Energy Expended via Data In
 */

#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/services/hrs.h>
#include <zephyr/bluetooth/services/acs.h>

#include "sample_hrs.h"

/* ACS restriction map entries for HRS — auto-registered to the protected map */
BT_ACS_PROTECT_CHAR_R_IN_MAP(hrs_body_sensor, CONFIG_BT_ACS_ACTIVE_RMAP_ID, BT_UUID_HRS_BODY_SENSOR,
			     BT_ACS_ISC_ID_DEFAULT);

BT_ACS_PROTECT_CHAR_N_IN_MAP(hrs_measurement, CONFIG_BT_ACS_ACTIVE_RMAP_ID, BT_UUID_HRS_MEASUREMENT,
			     BT_ACS_ISC_ID_DEFAULT);

BT_ACS_PROTECT_CHAR_W_IN_MAP(hrs_control_point, CONFIG_BT_ACS_ACTIVE_RMAP_ID,
			     BT_UUID_HRS_CONTROL_POINT, BT_ACS_ISC_ID_DEFAULT);

/* --- HRS callbacks --- */

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
	energy_expended += ee_kj;
	bt_hrs_notify(bpm);
}
