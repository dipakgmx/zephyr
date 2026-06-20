/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This sample demonstrates protecting two standard BLE services behind a
 * single ACS secured restriction map.
 *
 * Services and ACS access paths:
 *
 *   CTS Current Time  (mixed ISC per opcode)
 *     - Read   → ISC NONE:     plain GATT read, no key exchange needed
 *     - Write  → ISC HIGH_SEC: client sends SET via Data In (encrypted)
 *     - Notify → ISC HIGH_SEC: device sends time update via Data Out (DON, encrypted)
 *
 *   HRS Measurement   (Notify, ISC HIGH_SEC)
 *     - Notify → device sends HR data via Data Out (DON, encrypted)
 *
 *   HRS Control Point (CP procedure 0x01, ISC HIGH_SEC)
 *     - Reset Energy Expended → client sends via Data In (encrypted)
 *
 * Unprotected characteristics (GAP Device Name, Body Sensor Location, DIS)
 * are not listed in any restriction map — plain GATT, always accessible.
 *
 * The BT_ACS_RMAP_PROTECT_CHAR entries for CTS and HRS live in sample_cts.c
 * and sample_hrs.c respectively.  The linker collects them all into the
 * bt_acs_rmap_char_reg iterable section; they are referenced by pointer from
 * the map arrays below.
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

#include "sample_cts.h"
#include "sample_hrs.h"

/*
 * RMAP 0 — native-service resources (spec §3.5.3).
 *
 * Restriction Map ID 0 declares characteristics that are accessible directly
 * via their own GATT service without an ACS session.  The DIS Manufacturer
 * Name and Model Number strings are read-only, publicly accessible, and
 * require no key exchange — ideal representatives for this map.
 *
 * This entry is required for PTS tests BV-05-C and BV-06-C, which exercise
 * the Get Restriction Map Descriptor procedure against an *unprotected* RMAP
 * (i.e., a map whose map_isc_id = BT_ACS_ISC_ID_NONE).
 */
BT_ACS_PROTECT_CHAR_R_IN_MAP(dis_manuf_native, 0x0000, BT_UUID_DIS_MANUFACTURER_NAME,
			     BT_ACS_ISC_ID_NONE);

BT_ACS_PROTECT_CHAR_R_IN_MAP(dis_model_native, 0x0000, BT_UUID_DIS_MODEL_NUMBER,
			     BT_ACS_ISC_ID_NONE);

BT_ACS_RESTRICTION_MAP_DEFINE(native_map, .map_id = 0x0000, .map_isc_id = BT_ACS_ISC_ID_NONE,
			      .default_isc_id = BT_ACS_ISC_ID_NONE);

/*
 * RMAP CONFIG_BT_ACS_ACTIVE_RMAP_ID — protected resources.
 *
 * Characteristic entries are declared with convenience macros
 * in sample_cts.c and sample_hrs.c tagged with CONFIG_BT_ACS_ACTIVE_RMAP_ID.
 * The library discovers them by scanning the bt_acs_rmap_char_reg iterable
 * section — no extern declarations or explicit pointer arrays needed here.
 *
 * Increment CONFIG_BT_ACS_ACTIVE_RMAP_ID in Kconfig whenever the protected
 * resource set changes so clients know to re-fetch the RMAP.
 *
 * Get Restriction Map Descriptor (0x02) and Activate Restriction Map (0x06)
 * are not listed here — their protection is per-map (driven by map_isc_id)
 * and the handlers enforce that directly.
 */
BT_ACS_RESTRICTION_MAP_DEFINE(secured_map, .map_id = CONFIG_BT_ACS_ACTIVE_RMAP_ID,
			      .map_isc_id = BT_ACS_ISC_ID_DEFAULT,
			      .default_isc_id = BT_ACS_ISC_ID_NONE);

/* ACS callbacks */

static void acs_security_established(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("ACS security established with %s\n", addr);
}

static void acs_security_invalidated(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("ACS security invalidated for %s\n", addr);
}

static const struct bt_acs_cb acs_cb = {
	.security_established = acs_security_established,
	.security_invalidated = acs_security_invalidated,
};

/* BLE advertising */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_CTS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL), BT_UUID_16_ENCODE(BT_UUID_ACLS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Bonding helpers */

static bool has_bond;

static void bond_check_cb(const struct bt_bond_info *info, void *user_data)
{
	has_bond = true;
}

static int start_advertising(void)
{
	return bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}

/* Connection callbacks */

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
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Disconnected from %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));
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
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing cancelled: %s\n", addr);
}

static void auth_pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing complete: %s, bonded: %s\n", addr, bonded ? "yes" : "no");
}

static void auth_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing failed: %s, reason %d\n", addr, reason);
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

	err = bt_acs_init(&acs_cb);
	if (err) {
		printk("ACS init failed (err %d)\n", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		settings_load();
	}

	bt_conn_auth_cb_register(&auth_cb_display);
	bt_conn_auth_info_cb_register(&auth_info_cb);

	err = start_advertising();
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

	printk("CTS and HRS are protected by ACS (map_id=0x%04x)\n", CONFIG_BT_ACS_ACTIVE_RMAP_ID);
	printk("Clients must complete key exchange before accessing either service\n");

	/*
	 * Main loop: send HR notifications every 2 s, CTS notifications every 10 s.
	 * Both are intercepted by ACS and delivered as encrypted Data Out (DON).
	 */
	uint8_t cts_tick = 0;

	while (1) {
		k_sleep(K_SECONDS(2));

		uint16_t hr = 60 + (uint16_t)((k_uptime_get_32() / 2000) % 40);

		sample_hrs_notify(hr, hr / 30);

		if (++cts_tick >= 5) {
			cts_tick = 0;
			bt_cts_send_notification(BT_CTS_UPDATE_REASON_UNKNOWN);
		}
	}

	return 0;
}
