/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/bluetooth/services/cts.h>

#include "sample_acs.h"
#include "sample_cts.h"
#include "sample_hrs.h"

/* Locked map, active at start: every CTS and HRS resource needs ACS Data In.
 * It is mapped to ISC 0 in the Restriction Map ID List, so its descriptor is
 * read on the plain ACS CP and a client without keys can learn what is
 * protected and which ISC it needs.
 */
BT_ACS_RESTRICTION_MAP_DEFINE(locked_map, SAMPLE_ACS_RMAP_LOCKED_ID,
			      BT_ACS_ISC_ID_NONE,  /* map descriptor */
			      BT_ACS_ISC_ID_NONE); /* resources not listed */
BT_ACS_INITIAL_RESTRICTION_MAP(locked_map);

/* Relaxed map: of the CTS and HRS resources, only reading and writing the
 * current time need ACS Data In. Get Restriction Map Descriptor and Activate
 * Restriction Map for it, and Get All Active Descriptors while it is active,
 * need ACS Data In (ACS 1.0 Sections 4.4.3.1, 4.4.3.3).
 */
BT_ACS_RESTRICTION_MAP_DEFINE(relaxed_map, SAMPLE_ACS_RMAP_RELAXED_ID,
			      SAMPLE_ACS_ISC_ID,   /* map descriptor */
			      BT_ACS_ISC_ID_NONE); /* resources not listed */

/* Invalidate All Established Security and Set Security Controls Switch need ACS
 * Data In under the relaxed map only. The locked map lists no ACS CP
 * procedure, so its default ISC leaves both on the plain ACS CP.
 *
 * Neither map lists Get ISC Descriptor, Get Key Descriptor, the key exchange
 * or Invalidate Key, so a client without keys can always establish security
 * and discard keys it can no longer use.
 */
BT_ACS_PROTECTED_CONTROL_POINT_DEFINE(acs_cp_relaxed, relaxed_map, SAMPLE_ACS_ISC_ID,
				      BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY,
				      BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH);

static void acs_security_established(struct bt_conn *conn)
{
	printk("ACS security established with %s\n", bt_conn_dst_str(conn));
}

static void acs_security_invalidated(struct bt_conn *conn)
{
	printk("ACS security invalidated for %s\n", bt_conn_dst_str(conn));
}

static void acs_output_oob_number(struct bt_conn *conn, uint32_t number)
{
	printk("ACS confirmation for %s: show %u to the AC Client\n", bt_conn_dst_str(conn),
	       number);
}

#if IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_NUMERIC) ||                                        \
	IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_PUSH)
/* No input device, so the tester enters this fixed number on the AC Client. */
#define SAMPLE_ACS_INPUT_OOB_NUMBER 1U

static void acs_input_oob_request(struct bt_conn *conn, enum bt_acs_input_oob_action action)
{
	int err;

	err = bt_acs_input_oob_number(conn, SAMPLE_ACS_INPUT_OOB_NUMBER);
	if (err) {
		printk("ACS confirmation input for %s failed: %d\n", bt_conn_dst_str(conn), err);
		return;
	}

	printk("ACS confirmation input for %s (%s): answered %u\n", bt_conn_dst_str(conn),
	       action == BT_ACS_INPUT_OOB_PUSH ? "push" : "numeric", SAMPLE_ACS_INPUT_OOB_NUMBER);
}
#endif

static const struct bt_acs_cb acs_cb = {
	.security_established = acs_security_established,
	.security_invalidated = acs_security_invalidated,
	.output_oob_number = acs_output_oob_number,
#if IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_NUMERIC) ||                                        \
	IS_ENABLED(CONFIG_BT_ACS_CONFIRMATION_INPUT_PUSH)
	.input_oob_request = acs_input_oob_request,
#endif
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_CTS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL), BT_UUID_16_ENCODE(BT_UUID_ACLS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static bool has_bond;

static void bond_check_cb(const struct bt_bond_info *info, void *user_data)
{
	has_bond = true;
}

static int start_advertising(void)
{
	return bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}

static void recycled_cb(void)
{
	printk("Connection object recycled. Restarting advertising.\n");
	int err = start_advertising();

	if (err) {
		printk("Re-advertising failed (err %d)\n", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		return;
	}

	printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected from %s, reason 0x%02x %s\n", bt_conn_dst_str(conn), reason,
	       bt_hci_err_to_str(reason));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled_cb,
};

#if IS_ENABLED(CONFIG_BT_SMP_APP_PAIRING_ACCEPT)
static enum bt_security_err pairing_accept(struct bt_conn *conn,
					   const struct bt_conn_pairing_feat *const feat)
{
	has_bond = false;
	bt_foreach_bond(BT_ID_DEFAULT, bond_check_cb, NULL);

	if (has_bond) {
		printk("Pairing rejected: already bonded\n");
		return BT_SECURITY_ERR_PAIR_NOT_ALLOWED;
	}

	printk("Pairing accepted\n");
	return BT_SECURITY_ERR_SUCCESS;
}
#endif

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	printk("Passkey for %s: %06u\n", bt_conn_dst_str(conn), passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	printk("Pairing cancelled: %s\n", bt_conn_dst_str(conn));
}

static void auth_pairing_complete(struct bt_conn *conn, bool bonded)
{
	printk("Pairing complete: %s, bonded: %s\n", bt_conn_dst_str(conn), bonded ? "yes" : "no");
}

static void auth_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	printk("Pairing failed: %s, reason %d\n", bt_conn_dst_str(conn), reason);
}

static struct bt_conn_auth_cb auth_cb_display = {
#if IS_ENABLED(CONFIG_BT_SMP_APP_PAIRING_ACCEPT)
	.pairing_accept = pairing_accept,
#endif
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb auth_info_cb = {
	.pairing_complete = auth_pairing_complete,
	.pairing_failed = auth_pairing_failed,
};

int main(void)
{
	int err;

	printk("ACS + CTS + HRS Peripheral Sample\n");

	err = sample_cts_init();
	if (err) {
		printk("CTS init failed (err %d)\n", err);
		return 0;
	}

	err = sample_hrs_init();
	if (err) {
		printk("HRS init failed (err %d)\n", err);
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	settings_load();

	err = bt_acs_init(&acs_cb);
	if (err) {
		printk("ACS init failed (err %d)\n", err);
		return 0;
	}

	bt_conn_auth_cb_register(&auth_cb_display);
	bt_conn_auth_info_cb_register(&auth_info_cb);

	err = start_advertising();
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

	printk("ACS map 0x%04x (locked, active) protects CTS and HRS; map 0x%04x (relaxed) "
	       "protects only Current Time read and write\n",
	       SAMPLE_ACS_RMAP_LOCKED_ID, SAMPLE_ACS_RMAP_RELAXED_ID);
	printk("Clients must complete key exchange before accessing either service\n");

	/* HR every 2 s, CTS every 10 s, both as encrypted Data Out. */
	uint8_t cts_tick = 0;

	while (1) {
		k_sleep(K_SECONDS(2));

		uint16_t hr = 60 + (uint16_t)((k_uptime_get_32() / 2000) % 40);

		sample_hrs_notify(hr, hr / 30);

		if (++cts_tick >= 5) {
			cts_tick = 0;
			sample_cts_notify(BT_CTS_UPDATE_REASON_UNKNOWN);
		}
	}

	return 0;
}
