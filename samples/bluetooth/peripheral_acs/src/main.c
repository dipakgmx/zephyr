/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/bluetooth/services/cts.h>

#include "sample_acs.h"
#include "sample_cts.h"
#include "sample_hrs.h"

/* Map 1, active at start: protects CTS Current Time, every HRS characteristic and, without
 * CONFIG_SAMPLE_ACS_PTS, the two ACS CP procedures below. Its map ISC is 0, so a client
 * without keys reads its descriptor on the plain ACS CP.
 */
BT_ACS_RESTRICTION_MAP_DEFINE(locked_map, SAMPLE_ACS_RMAP_LOCKED_ID,
			      BT_ACS_ISC_ID_NONE,  /* map descriptor */
			      BT_ACS_ISC_ID_NONE); /* resources not listed */
BT_ACS_INITIAL_RESTRICTION_MAP(locked_map);

/* Both procedures act on every client: one ends all established security, the other
 * can turn all protection off (ACS 1.0 Section 4.4.3.14), so only clients with
 * established security may run them. No map protects the procedures a client without
 * keys needs to establish security.
 */
#if IS_ENABLED(CONFIG_SAMPLE_ACS_PTS)
/* PTS layout: map 1 leaves both procedures on the plain ACS CP, and map 2 protects them
 * and Current Time read and write. Requests about map 2 need ACS Data In (ACS 1.0
 * Sections 4.4.3.1, 4.4.3.3).
 */
BT_ACS_RESTRICTION_MAP_DEFINE(relaxed_map, SAMPLE_ACS_RMAP_RELAXED_ID,
			      SAMPLE_ACS_ISC_ID,   /* map descriptor */
			      BT_ACS_ISC_ID_NONE); /* resources not listed */
BT_ACS_PROTECTED_CONTROL_POINT_DEFINE(acs_cp_relaxed, relaxed_map, SAMPLE_ACS_ISC_ID,
				      BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY,
				      BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH);
#else
BT_ACS_PROTECTED_CONTROL_POINT_DEFINE(acs_cp_locked, locked_map, SAMPLE_ACS_ISC_ID,
				      BT_ACS_CP_OPCODE_INVALIDATE_ALL_ESTABLISHED_SECURITY,
				      BT_ACS_CP_OPCODE_SET_SECURITY_CONTROLS_SWITCH);
#endif

/* Connections with ACS security established, by bt_conn_index(). */
static ATOMIC_DEFINE(secured_conns, CONFIG_BT_MAX_CONN);

static bool any_conn_secured(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(secured_conns); i++) {
		if (atomic_get(&secured_conns[i]) != 0) {
			return true;
		}
	}

	return false;
}

static void acs_security_established(struct bt_conn *conn)
{
	printk("ACS security established with %s\n", bt_conn_dst_str(conn));
	atomic_set_bit(secured_conns, bt_conn_index(conn));
}

static void acs_security_invalidated(struct bt_conn *conn)
{
	printk("ACS security invalidated for %s\n", bt_conn_dst_str(conn));
	atomic_clear_bit(secured_conns, bt_conn_index(conn));
}

static void acs_output_oob_number(struct bt_conn *conn, uint32_t number)
{
	printk("ACS confirmation for %s: show %u to the AC Client\n", bt_conn_dst_str(conn),
	       number);
}

/* No input device, so the tester enters this fixed number on the AC Client. */
#define SAMPLE_ACS_INPUT_OOB_NUMBER 1U

static void acs_input_oob_request(struct bt_conn *conn, enum bt_acs_input_oob_action action)
{
	int err;

	err = bt_acs_input_oob_number(conn, SAMPLE_ACS_INPUT_OOB_NUMBER);
	if (err != 0) {
		printk("ACS confirmation input for %s failed: %d\n", bt_conn_dst_str(conn), err);
		return;
	}

	printk("ACS confirmation input for %s (%s): answered %u\n", bt_conn_dst_str(conn),
	       action == BT_ACS_INPUT_OOB_PUSH ? "push" : "numeric", SAMPLE_ACS_INPUT_OOB_NUMBER);
}

/* ACS calls the confirmation callbacks only for the methods enabled in Kconfig. */
static const struct bt_acs_cb acs_cb = {
	.security_established = acs_security_established,
	.security_invalidated = acs_security_invalidated,
	.output_oob_number = acs_output_oob_number,
	.input_oob_request = acs_input_oob_request,
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

static int start_advertising(void)
{
	return bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}

static void recycled_cb(void)
{
	int err;

	printk("Connection object recycled. Restarting advertising.\n");

	err = start_advertising();
	if (err != 0) {
		printk("Re-advertising failed (err %d)\n", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != BT_HCI_ERR_SUCCESS) {
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
	if (err != 0) {
		printk("CTS init failed (err %d)\n", err);
		return 0;
	}

	err = sample_hrs_init();
	if (err != 0) {
		printk("HRS init failed (err %d)\n", err);
		return 0;
	}

	err = bt_enable(NULL);
	if (err != 0) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	err = settings_load();
	if (err != 0) {
		printk("Settings load failed (err %d)\n", err);
		return 0;
	}

	err = bt_acs_init(&acs_cb);
	if (err != 0) {
		printk("ACS init failed (err %d)\n", err);
		return 0;
	}

	err = bt_conn_auth_cb_register(&auth_cb_display);
	if (err != 0) {
		printk("Auth callback registration failed (err %d)\n", err);
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&auth_info_cb);
	if (err != 0) {
		printk("Auth info callback registration failed (err %d)\n", err);
		return 0;
	}

	err = start_advertising();
	if (err != 0) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

	printk("ACS map %u active: CTS Current Time and HRS need ACS security\n",
	       SAMPLE_ACS_RMAP_LOCKED_ID);

	/* HR every 2 s, CTS every 10 s, while a client has ACS security. ACS sends each value
	 * on Data Out while the active map protects it, as a plain notification otherwise.
	 */
	bool sending = false;
	uint8_t cts_tick = 0U;

	while (true) {
		k_sleep(K_SECONDS(2));

		if (any_conn_secured() != sending) {
			sending = !sending;
			printk("%s heart rate and current time\n",
			       sending ? "Sending" : "Stopped sending");
		}

		if (!sending) {
			continue;
		}

		uint16_t hr = 60 + (uint16_t)((k_uptime_get_32() / 2000) % 40);

		sample_hrs_notify(hr, hr / 30);

		if (++cts_tick >= 5) {
			cts_tick = 0;
			sample_cts_notify(BT_CTS_UPDATE_REASON_UNKNOWN);
		}
	}

	return 0;
}
