/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "acs_cp_operands.h"
#include "acs_internal.h"
#include "acs_reply.h"
#include "acs_crypto.h"
#include "acs_isc.h"
#include "acs_key_desc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Default ISC records selected by Kconfig. */
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
BT_ACS_ISC_DEFINE(acs_isc_high_sec_gcm, .isc_id = BT_ACS_ISC_ID_HIGH_SEC_GCM, .num_controls = 3,
		  .controls = {ACS_CTRL_NONCE, ACS_CTRL_MAC, ACS_CTRL_AUTH_ENC},
		  .key_id = ACS_KEY_ID_GCM);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
BT_ACS_ISC_DEFINE(acs_isc_high_sec_ccm, .isc_id = BT_ACS_ISC_ID_HIGH_SEC_CCM, .num_controls = 3,
		  .controls = {ACS_CTRL_NONCE, ACS_CTRL_MAC, ACS_CTRL_AUTH_ENC},
		  .key_id = ACS_KEY_ID_CCM);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
BT_ACS_ISC_DEFINE(acs_isc_integrity_gmac, .isc_id = BT_ACS_ISC_ID_INTEGRITY_GMAC, .num_controls = 3,
		  .controls = {ACS_CTRL_NONCE, ACS_CTRL_MAC, ACS_CTRL_AUTH},
		  .key_id = ACS_KEY_ID_GMAC);
#endif

#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
/* Controls are listed in wire order: MAC, then authenticated payload. */
BT_ACS_ISC_DEFINE(acs_isc_mac_only_cmac, .isc_id = BT_ACS_ISC_ID_MAC_ONLY_CMAC, .num_controls = 2,
		  .controls = {ACS_CTRL_MAC, ACS_CTRL_AUTH}, .key_id = ACS_KEY_ID_CMAC);
#endif

const struct bt_acs_isc_record *acs_isc_lookup(uint16_t isc_id)
{
	STRUCT_SECTION_FOREACH(bt_acs_isc_record, rec) {
		if (rec->isc_id == isc_id) {
			return rec;
		}
	}
	return NULL;
}

int acs_resolve_isc_slot(struct bt_acs_conn *acs_conn, uint16_t isc_id,
			 struct bt_acs_key_desc_runtime **key_runtime)
{
	const struct bt_acs_isc_record *isc;
	int err;

	*key_runtime = NULL;

	isc = acs_isc_lookup(isc_id);
	if (isc == NULL) {
		LOG_WRN("unknown ISC_ID 0x%04x", isc_id);
		return -ENOENT;
	}

	err = acs_crypto_key_runtime_lookup(acs_conn, isc->key_id, key_runtime);
	if (err) {
		LOG_WRN("no key descriptor runtime for ISC_ID 0x%04x", isc_id);
	}

	return err;
}

int acs_resolve_isc_key(struct bt_acs_conn *acs_conn, uint16_t isc_id,
			struct bt_acs_key_desc_runtime **key_runtime)
{
	int err;

	err = acs_resolve_isc_slot(acs_conn, isc_id, key_runtime);
	if (err) {
		return err;
	}

	if (!acs_current_key_installed(*key_runtime)) {
		LOG_WRN("key for ISC_ID 0x%04x not provisioned", isc_id);
		return -EACCES;
	}

	return 0;
}

int acs_isc_validate_records(void)
{
	STRUCT_SECTION_FOREACH(bt_acs_isc_record, rec) {
		if (rec->isc_id == BT_ACS_ISC_ID_NONE ||
		    rec->isc_id == BT_ACS_ISC_ALL_RECORDS_FILTER) {
			LOG_ERR("ISC record uses reserved ISC_ID 0x%04x", rec->isc_id);
			return -EINVAL;
		}

		if (rec->num_controls == 0U ||
		    rec->num_controls > CONFIG_BT_ACS_ISC_MAX_CONTROLS) {
			LOG_ERR("ISC_ID 0x%04x declares %u controls, expected 1..%u", rec->isc_id,
				rec->num_controls, CONFIG_BT_ACS_ISC_MAX_CONTROLS);
			return -EINVAL;
		}

		/* Lookup returns the first match, so duplicates resolve by link order. */
		if (acs_isc_lookup(rec->isc_id) != rec) {
			LOG_ERR("duplicate ISC_ID 0x%04x", rec->isc_id);
			return -EINVAL;
		}
	}

	return 0;
}

int acs_isc_build_response(struct net_buf_simple *operand, struct net_buf *buf)
{
	bool record_found = false;
	uint16_t filter_id;

	if (operand->len < sizeof(struct acs_cp_get_isc_descriptor_req)) {
		LOG_ERR("ISC operand too short: %u bytes (expected at least %zu)", operand->len,
			sizeof(struct acs_cp_get_isc_descriptor_req));
		return -EINVAL;
	}

	filter_id = net_buf_simple_pull_le16(operand);

	LOG_DBG("ACS ISC: Querying Filter ID 0x%04X", filter_id);

	/* ISC_ID 0 means no security controls and has no descriptor record. */
	if (filter_id == BT_ACS_ISC_ID_NONE) {
		LOG_WRN("ACS ISC: filter 0x0000 is reserved; no records to return");
		return -ENOENT;
	}

	STRUCT_SECTION_FOREACH(bt_acs_isc_record, rec) {
		bool needs_key = false;
		uint8_t record_size;

		if (filter_id != BT_ACS_ISC_ALL_RECORDS_FILTER && filter_id != rec->isc_id) {
			continue;
		}

		record_found = true;

		for (uint8_t k = 0U; k < rec->num_controls; k++) {
			/* Table 4.32 excludes Key_ID only for the unencrypted control. */
			if (rec->controls[k] != ACS_CTRL_UNENC) {
				needs_key = true;
				break;
			}
		}

		record_size = ACS_ISC_NUM_CTRL_FIELD_SIZE + rec->num_controls +
			      (needs_key ? ACS_ISC_KEY_ID_FIELD_SIZE : 0U);

		LOG_DBG("ISC record: isc_id=0x%04x num_controls=%u needs_key=%d "
			"key_id=0x%04x",
			rec->isc_id, rec->num_controls, (int)needs_key,
			needs_key ? rec->key_id : 0);
		for (uint8_t k = 0; k < rec->num_controls; k++) {
			LOG_DBG("  control[%u]=0x%02x", k, rec->controls[k]);
		}

		if (net_buf_tailroom(buf) < (sizeof(struct acs_desc_rec_hdr) + record_size)) {
			return -ENOMEM;
		}

		/* ISC record TLV header (Table 4.5). */
		struct acs_desc_rec_hdr hdr = {
			.type_id = BT_ACS_RECORD_TYPE_ISC_ID,
			.type_value = sys_cpu_to_le16(rec->isc_id),
			.data_size = record_size,
		};
		net_buf_add_mem(buf, &hdr, sizeof(hdr));

		net_buf_add_u8(buf, rec->num_controls);
		net_buf_add_mem(buf, rec->controls, rec->num_controls);

		if (needs_key) {
			net_buf_add_le16(buf, rec->key_id);
		}
	}

	if (!record_found) {
		LOG_WRN("ACS ISC: no record found for filter 0x%04X", filter_id);
		return -ENOENT;
	}

	return 0;
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
struct acs_cp_result acs_cp_handle_get_isc_descriptor(struct acs_reply *reply,
						      struct net_buf_simple *buf)
{
	int err = acs_isc_build_response(buf, reply->response);

	if (err) {
		return acs_cp_status(errno_to_acs_status(err));
	}

	return acs_cp_reply();
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */
