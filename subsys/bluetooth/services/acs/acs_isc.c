/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "acs_internal.h"
#include "acs_isc.h"
#include "acs_key_desc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Catch misconfigured builds: confidentiality bit advertised but no encrypting cipher selected. */
BUILD_ASSERT(!(IS_ENABLED(CONFIG_BT_ACS_FEAT_CONFIDENTIALITY) &&
	       !IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM) &&
	       !IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)),
	     "CONFIG_BT_ACS_FEAT_CONFIDENTIALITY requires AES-GCM or AES-CCM data protection");

/*
 * Default ISC records — compiled in when the corresponding algorithm Kconfig is enabled.
 * Applications may register additional ISC records with BT_ACS_ISC_DEFINE() in their
 * own source files; the library discovers all records via the bt_acs_isc_record iterable section.
 */
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
BT_ACS_ISC_DEFINE(acs_isc_high_sec_gcm, .isc_id = ACS_ISC_ID_HIGH_SEC, .num_controls = 3,
		  .controls = {ACS_CTRL_NONCE, ACS_CTRL_MAC, ACS_CTRL_AUTH_ENC},
		  .key_id = ACS_KEY_ID_GCM);
#elif IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
BT_ACS_ISC_DEFINE(acs_isc_high_sec_ccm, .isc_id = ACS_ISC_ID_HIGH_SEC, .num_controls = 3,
		  .controls = {ACS_CTRL_NONCE, ACS_CTRL_MAC, ACS_CTRL_AUTH_ENC},
		  .key_id = ACS_KEY_ID_CCM);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
BT_ACS_ISC_DEFINE(acs_isc_integrity_gmac, .isc_id = ACS_ISC_ID_INTEGRITY, .num_controls = 3,
		  .controls = {ACS_CTRL_NONCE, ACS_CTRL_MAC, ACS_CTRL_AUTH_ENC},
		  .key_id = ACS_KEY_ID_GMAC);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
BT_ACS_ISC_DEFINE(acs_isc_mac_only_cmac, .isc_id = ACS_ISC_ID_MAC_ONLY, .num_controls = 1,
		  .controls = {ACS_CTRL_MAC}, .key_id = ACS_KEY_ID_CMAC);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_KEY_EXCHANGE_ECDH)
BT_ACS_ISC_DEFINE(acs_isc_auth_ecdh, .isc_id = ACS_ISC_ID_AUTH, .num_controls = 2,
		  .controls = {ACS_CTRL_NONCE, ACS_CTRL_MAC}, .key_id = ACS_KEY_ID_ECDH);
#endif

/* Unencrypted fallback — always present */
BT_ACS_ISC_DEFINE(acs_isc_unenc, .isc_id = ACS_ISC_ID_UNENC, .num_controls = 1,
		  .controls = {ACS_CTRL_UNENC}, .key_id = 0x0000);

const struct bt_acs_isc_record *acs_isc_lookup(uint16_t isc_id)
{
	STRUCT_SECTION_FOREACH(bt_acs_isc_record, rec) {
		if (rec->isc_id == isc_id) {
			return rec;
		}
	}
	return NULL;
}

static bool requires_key_id(const uint8_t *controls, uint8_t num)
{
	for (int i = 0; i < num; i++) {
		if (controls[i] == ACS_CTRL_AUTH || controls[i] == ACS_CTRL_ENC ||
		    controls[i] == ACS_CTRL_AUTH_ENC || controls[i] == ACS_CTRL_AUTH_ENC_AD ||
		    controls[i] == ACS_CTRL_MAC) {
			return true;
		}
	}
	return false;
}

int acs_isc_build_response(struct net_buf_simple *operand, struct net_buf_simple *buf)
{
	uint16_t filter_id;

	if (operand->len < sizeof(filter_id)) {
		LOG_ERR("ISC operand too short: %u bytes (expected at least %zu)", operand->len,
			sizeof(filter_id));
		return -EINVAL;
	}

	filter_id = net_buf_simple_pull_le16(operand);
	bool record_found = false;

	LOG_DBG("ACS ISC: Querying Filter ID 0x%04X", filter_id);

	/*
	 * ISC ID 0x0000 is reserved to mean "no security controls required".
	 * It is used only as a reference in restriction map descriptors and
	 * shall never appear in an ISC descriptor record (spec §4.4.x).
	 * Treat a request for it the same as any unknown ID: no records found.
	 */
	if (filter_id == BT_ACS_ISC_UNPROTECTED_ID) {
		LOG_WRN("ACS ISC: filter 0x0000 is reserved; no records to return");
		return -ENOENT;
	}

	STRUCT_SECTION_FOREACH(bt_acs_isc_record, rec) {
		/* Defensively skip any record that carries a reserved ID. */
		if (rec->isc_id == BT_ACS_ISC_UNPROTECTED_ID ||
		    rec->isc_id == BT_ACS_ISC_ALL_RECORDS_FILTER) {
			LOG_ERR("ISC record has reserved id 0x%04x; skipping", rec->isc_id);
			continue;
		}

		if (filter_id != BT_ACS_ISC_ALL_RECORDS_FILTER && filter_id != rec->isc_id) {
			continue;
		}

		record_found = true;

		if (rec->num_controls > CONFIG_BT_ACS_ISC_MAX_CONTROLS) {
			LOG_ERR("ISC record 0x%04X: num_controls %u exceeds max %u", rec->isc_id,
				rec->num_controls, CONFIG_BT_ACS_ISC_MAX_CONTROLS);
			return -EINVAL;
		}

		bool needs_key = requires_key_id(rec->controls, rec->num_controls);

		LOG_DBG("ISC record: isc_id=0x%04x num_controls=%u needs_key=%d "
			"key_id=0x%04x",
			rec->isc_id, rec->num_controls, (int)needs_key,
			needs_key ? rec->key_id : 0);
		for (uint8_t k = 0; k < rec->num_controls; k++) {
			LOG_DBG("  control[%u]=0x%02x", k, rec->controls[k]);
		}

		/* record_size: num_controls byte (1) + actual controls array (N) + optional
		 * key (2) */
		uint8_t record_size = ACS_ISC_NUM_CTRL_FIELD_SIZE + rec->num_controls +
				      (needs_key ? ACS_ISC_KEY_ID_FIELD_SIZE : 0);

		/* Check if the buffer has enough space: header (4) + record data */
		if (net_buf_simple_tailroom(buf) < (sizeof(struct acs_isc_rec_hdr) + record_size)) {
			return -ENOMEM;
		}

		/* ISC record TLV header (Table 4.5) */
		struct acs_isc_rec_hdr hdr = {
			.type_id = BT_ACS_RECORD_TYPE_ISC_ID,
			.type_value = sys_cpu_to_le16(rec->isc_id),
			.data_size = record_size,
		};
		net_buf_simple_add_mem(buf, &hdr, sizeof(hdr));

		net_buf_simple_add_u8(buf, rec->num_controls);
		net_buf_simple_add_mem(buf, rec->controls, rec->num_controls);

		if (needs_key) {
			net_buf_simple_add_le16(buf, rec->key_id);
		}
	}

	if (!record_found) {
		LOG_WRN("ACS ISC: no record found for filter 0x%04X", filter_id);
		return -ENOENT;
	}

	return 0;
}
