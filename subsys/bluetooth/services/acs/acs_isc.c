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
#include "acs_keys.h"
#include "acs_isc.h"
#include "acs_key_desc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Static ISC records used by the server. See @Z_BT_ACS_ISC_ENABLED() when modifying this table*/
static const struct bt_acs_isc_record acs_isc_records[] = {
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GCM)
	{
		.isc_id = BT_ACS_ISC_ID_HIGH_SEC_GCM,
		.num_controls = 3,
		.controls = {ACS_CTRL_NONCE, ACS_CTRL_MAC, ACS_CTRL_AUTH_ENC},
		.key_id = ACS_KEY_ID_GCM,
	},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CCM)
	{
		.isc_id = BT_ACS_ISC_ID_HIGH_SEC_CCM,
		.num_controls = 3,
		.controls = {ACS_CTRL_NONCE, ACS_CTRL_MAC, ACS_CTRL_AUTH_ENC},
		.key_id = ACS_KEY_ID_CCM,
	},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_GMAC)
	{
		.isc_id = BT_ACS_ISC_ID_INTEGRITY_GMAC,
		.num_controls = 3,
		.controls = {ACS_CTRL_NONCE, ACS_CTRL_MAC, ACS_CTRL_AUTH},
		.key_id = ACS_KEY_ID_GMAC,
	},
#endif
#if IS_ENABLED(CONFIG_BT_ACS_DATA_PROTECTION_AES_CMAC)
	/* Controls are listed in wire order: MAC, then authenticated payload. */
	{
		.isc_id = BT_ACS_ISC_ID_MAC_ONLY_CMAC,
		.num_controls = 2,
		.controls = {ACS_CTRL_MAC, ACS_CTRL_AUTH},
		.key_id = ACS_KEY_ID_CMAC,
	},
#endif
};

/* Get ISC Descriptor for all records fits one message (§4.4.3.7). */
BUILD_ASSERT(ARRAY_SIZE(acs_isc_records) *
			     (sizeof(struct acs_desc_rec_hdr) + 1U + ACS_ISC_MAX_CONTROLS +
			      sizeof(uint16_t)) <=
		     ACS_MESSAGE_MAX_OPERAND,
	     "ISC Descriptor Response exceeds one message");

const struct bt_acs_isc_record *acs_isc_lookup(uint16_t isc_id)
{
	ARRAY_FOR_EACH_PTR(acs_isc_records, rec) {
		if (rec->isc_id == isc_id) {
			return rec;
		}
	}
	return NULL;
}

struct acs_sec_alg *acs_isc_alg(struct bt_acs_conn *acs_conn, uint16_t isc_id)
{
	const struct bt_acs_isc_record *isc = acs_isc_lookup(isc_id);
	struct acs_sec_alg *alg;

	if (isc == NULL) {
		LOG_WRN("unknown ISC_ID 0x%04x", isc_id);
		return NULL;
	}

	alg = acs_keys_alg(acs_conn, isc->key_id);
	if (alg == NULL) {
		LOG_WRN("ISC_ID 0x%04x Key_ID 0x%04x is not an algorithm record", isc_id,
			isc->key_id);
	}

	return alg;
}

uint8_t acs_isc_build_response(uint16_t filter_id, struct net_buf *buf)
{
	bool record_found = false;

	LOG_DBG("Querying Filter ID 0x%04X", filter_id);

	/* ISC_ID 0 means no security controls and has no descriptor record. (see §4.4.3.7) */
	if (filter_id == BT_ACS_ISC_ID_NONE) {
		LOG_WRN("filter 0x0000 is reserved; no records to return");
		return BT_ACS_CP_RESPONSE_NO_RECORDS_FOUND;
	}

	ARRAY_FOR_EACH_PTR(acs_isc_records, rec) {
		bool needs_key = false;
		uint8_t record_size;
		int err;

		if (filter_id != BT_ACS_ISC_ALL_RECORDS_FILTER && filter_id != rec->isc_id) {
			continue;
		}

		record_found = true;

		/* Determine if the ISC record needs a Key_ID field. The Key_ID is omitted for the
		 * unencrypted control (ACS_CTRL_UNENC) only. See Table 4.32.
		 */
		for (uint8_t k = 0U; k < rec->num_controls; k++) {
			if (rec->controls[k] != ACS_CTRL_UNENC) {
				needs_key = true;
				break;
			}
		}

		/* Number_Of_Controls (1) | Controls (N: 2 or 3 here) | Key_ID (2, optional) */
		record_size = ACS_ISC_NUM_CTRL_FIELD_SIZE + rec->num_controls +
			      (needs_key ? ACS_ISC_KEY_ID_FIELD_SIZE : 0U);

		LOG_DBG("ISC record: isc_id=0x%04x num_controls=%u needs_key=%d "
			"key_id=0x%04x",
			rec->isc_id, rec->num_controls, (int)needs_key,
			needs_key ? rec->key_id : 0);

		/* ISC record (Table 4.5). */
		err = acs_desc_add_record_header(buf, BT_ACS_RECORD_TYPE_ISC_ID, rec->isc_id,
						 record_size);
		if (err) {
			return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
		}

		(void)net_buf_add_u8(buf, rec->num_controls);
		(void)net_buf_add_mem(buf, rec->controls, rec->num_controls);

		if (needs_key) {
			net_buf_add_le16(buf, rec->key_id);
		}
	}

	if (!record_found) {
		LOG_WRN("no record found for filter 0x%04X", filter_id);
		return BT_ACS_CP_RESPONSE_NO_RECORDS_FOUND;
	}

	return BT_ACS_CP_RESPONSE_SUCCESS;
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
uint8_t acs_cp_handle_get_isc_descriptor(struct acs_reply *reply, struct net_buf_simple *buf)
{
	uint16_t filter_id;

	if (buf->len < sizeof(struct acs_cp_get_isc_descriptor_req)) {
		LOG_ERR("ISC operand too short: %u bytes (expected at least %zu)", buf->len,
			sizeof(struct acs_cp_get_isc_descriptor_req));
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	}

	filter_id = net_buf_simple_pull_le16(buf);

	return acs_isc_build_response(filter_id, reply->response);
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */
