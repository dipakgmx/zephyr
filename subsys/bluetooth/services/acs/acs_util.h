/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_ACS_UTIL_H_
#define BT_GATT_ACS_UTIL_H_

#include <errno.h>

#include "acs_types.h"
#include "acs_wire_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Map a response-building errno to an ACS Control Point Response Code. */
static inline enum bt_acs_cp_response_code errno_to_acs_status(int err)
{
	switch (err) {
	case 0:
		return BT_ACS_CP_RESPONSE_SUCCESS;
	case -EINVAL:
		return BT_ACS_CP_RESPONSE_INVALID_OPERAND;
	case -ENOENT:
		return BT_ACS_CP_RESPONSE_NO_RECORDS_FOUND;
	case -ERANGE:
		return BT_ACS_CP_RESPONSE_PARAMETER_OUT_OF_RANGE;
	case -EALREADY:
	case -EAGAIN:
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_APPLICABLE;
	case -ENOTSUP:
		return BT_ACS_CP_RESPONSE_OPCODE_NOT_SUPPORTED;
	case -ENOSPC:
	case -EACCES:
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	default:
		return BT_ACS_CP_RESPONSE_PROCEDURE_NOT_COMPLETED;
	}
}

/*
 * Add a descriptor record header (Table 4.4) if buf has room for it and for
 * data_size octets of Data, which the caller adds next. Return -ENOMEM and add
 * nothing otherwise.
 */
static inline int acs_desc_add_record_header(struct net_buf *buf, uint8_t type_id,
					     uint16_t type_value, uint8_t data_size)
{
	if (net_buf_tailroom(buf) < sizeof(struct acs_desc_rec_hdr) + data_size) {
		return -ENOMEM;
	}

	net_buf_add_u8(buf, type_id);
	net_buf_add_le16(buf, type_value);
	net_buf_add_u8(buf, data_size);

	return 0;
}

/* Return true while a key exchange is in progress. */
static inline bool acs_kex_in_progress(const struct bt_acs_conn *acs_conn)
{
	return acs_conn && acs_conn->kex != NULL;
}

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_UTIL_H_ */
