/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file acs_util.h
 * @brief ACS inline utility functions (errno → CP response code mapping).
 *
 * Included transitively via acs_internal.h - not intended for direct inclusion.
 */

#ifndef BT_GATT_ACS_UTIL_H_
#define BT_GATT_ACS_UTIL_H_

#include <errno.h>

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Map a build-function errno to an ACS CP response code.
 */
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

static inline bool acs_current_key_installed(const struct bt_acs_key_desc_runtime *key_runtime)
{
	return key_runtime && key_runtime->psa_key_id != 0U;
}

static inline bool acs_session_established(const struct bt_acs_conn *acs_conn)
{
	return acs_conn && ((acs_conn->status_flags & BT_ACS_STATUS_SECURITY_ESTABLISHED) != 0U);
}

static inline bool acs_kex_in_progress(const struct bt_acs_conn *acs_conn)
{
	return acs_conn && acs_conn->kex.active;
}

static inline bool acs_kex_expects(const struct bt_acs_conn *acs_conn, uint8_t opcode)
{
	return acs_kex_in_progress(acs_conn) && acs_conn->kex.next_expected_opcode == opcode;
}

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_ACS_UTIL_H_ */
