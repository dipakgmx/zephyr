/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "acs_internal.h"
#include "acs_wire_constants.h"

LOG_MODULE_DECLARE(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

#ifdef CONFIG_BT_ACS_ACTIVE_RMAP_ID
#define ACS_SERVICE_DEFAULT_ACTIVE_MAP_ID CONFIG_BT_ACS_ACTIVE_RMAP_ID
#else
#define ACS_SERVICE_DEFAULT_ACTIVE_MAP_ID 0U
#endif

/*
 * Spec Table 4.1 characteristic presence conditions:
 * - Data In (C.1): mandatory when protected write or read is supported
 * - Data Out Notify (C.2): mandatory when protected read or notification is supported
 * - Data Out Indicate (C.3): mandatory when protected indication is supported
 */
#if defined(CONFIG_BT_ACS_PROTECTED_RESOURCE_WRITE) || defined(CONFIG_BT_ACS_PROTECTED_RESOURCE_READ)
#define ACS_SERVICE_HAS_DATA_IN 1
#else
#define ACS_SERVICE_HAS_DATA_IN 0
#endif

#if defined(CONFIG_BT_ACS_PROTECTED_RESOURCE_READ) || \
	defined(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
#define ACS_SERVICE_HAS_DON 1
#else
#define ACS_SERVICE_HAS_DON 0
#endif

#if defined(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
#define ACS_SERVICE_HAS_DOI 1
#else
#define ACS_SERVICE_HAS_DOI 0
#endif

static struct {
	const struct bt_gatt_attr *status;
	const struct bt_gatt_attr *cp;
#if ACS_SERVICE_HAS_DON
	const struct bt_gatt_attr *don;
#endif
#if ACS_SERVICE_HAS_DOI
	const struct bt_gatt_attr *doi;
#endif
} acs_attrs;

static void acs_status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	LOG_DBG("ACS Status CCC: %s",
		(value == BT_GATT_CCC_INDICATE) ? "enabled" : "disabled");
}

#if ACS_SERVICE_HAS_DON
static void acs_data_out_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	LOG_DBG("ACS Data Out Notify CCC: %s",
		(value == BT_GATT_CCC_NOTIFY) ? "enabled" : "disabled");
}
#endif

#if ACS_SERVICE_HAS_DOI
static void acs_data_out_ind_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	LOG_DBG("ACS Data Out Indicate CCC: %s",
		(value == BT_GATT_CCC_INDICATE) ? "enabled" : "disabled");
}
#endif

static void acs_cp_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	LOG_DBG("ACS Control Point CCC: %s",
		(value == BT_GATT_CCC_INDICATE) ? "enabled" : "disabled");
}

static ssize_t acs_status_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	uint8_t status_data[ACS_STATUS_SIZE];
	struct acs_conn_ctx *conn_ctx = acs_runtime_lookup_conn(conn);

	if (conn_ctx) {
		status_data[0] = conn_ctx->status_flags;
		sys_put_le16(conn_ctx->active_map_id, &status_data[1]);
	} else {
		status_data[0] = bt_acs_status_get(conn);
		sys_put_le16(IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION) ?
				     ACS_SERVICE_DEFAULT_ACTIVE_MAP_ID :
				     0U,
			     &status_data[1]);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, status_data, sizeof(status_data));
}

static ssize_t acs_att_err_from_runtime(int err)
{
	switch (err) {
	case 0:
		return 0;
	case ACS_SEG_RX_ERR_COUNTER:
		return BT_GATT_ERR(BT_ACS_ATT_ERR_INVALID_SEG_COUNTER);
	case ACS_SEG_RX_ERR_OVERFLOW:
	case -ENOMEM:
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	case -EBUSY:
		return BT_GATT_ERR(BT_ATT_ERR_PROCEDURE_IN_PROGRESS);
	case -EACCES:
		return BT_GATT_ERR(BT_ACS_ATT_ERR_INVALID_KEY);
	case -EPERM:
		return BT_GATT_ERR(BT_ACS_ATT_ERR_INCORRECT_SECURITY_CONFIGURATION);
	case -ENOENT:
	case -ENOTSUP:
		return BT_GATT_ERR(BT_ACS_ATT_ERR_RESOURCE_NOT_PROTECTED);
	case -EINVAL:
		return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
	case ACS_SEG_RX_ERR_TIMEOUT:
	case ACS_SEG_RX_ERR_ORPHAN:
	case ACS_SEG_RX_ERR_LEN:
	default:
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
}

#if ACS_SERVICE_HAS_DATA_IN
static ssize_t acs_data_in_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	int err;

	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		LOG_WRN("ACS Data In write rejected: invalid offset %u", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	LOG_DBG("ACS Data In write: len=%u flags=0x%02x", len, flags);
	err = acs_runtime_handle_data_in(conn, buf, len);
	if (err == ACS_PROC_STEP_WAIT_IND_CONFIRM) {
		LOG_WRN("ACS Data In accepted: waiting for confirm-driven completion");
		return len;
	}

	if (err) {
		LOG_WRN("ACS Data In handling failed: %d", err);
	}

	return err ? acs_att_err_from_runtime(err) : len;
}
#endif

static ssize_t acs_cp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			    uint16_t len, uint16_t offset, uint8_t flags)
{
	int err;

	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		LOG_WRN("ACS CP write rejected: invalid offset %u", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	LOG_DBG("ACS CP write: len=%u flags=0x%02x opcode=0x%02x", len, flags,
		len > 0U ? ((const uint8_t *)buf)[0] : 0U);
	err = acs_runtime_handle_cp_write(conn, buf, len);
	if (err == ACS_PROC_STEP_WAIT_IND_CONFIRM) {
		LOG_DBG("ACS CP accepted: waiting for confirm-driven completion");
		return len;
	}

	if (err) {
		LOG_WRN("ACS CP handling failed: %d", err);
	}

	return err ? acs_att_err_from_runtime(err) : len;
}

#if ACS_SERVICE_HAS_DATA_IN
#define ACS_DATA_IN_ATTRS                                                                         \
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_ACS_DI,                                              \
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,             \
			       BT_GATT_PERM_WRITE, NULL, acs_data_in_write, NULL),
#else
#define ACS_DATA_IN_ATTRS
#endif

#if ACS_SERVICE_HAS_DON
#define ACS_DON_ATTRS                                                                             \
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_ACS_DON, BT_GATT_CHRC_NOTIFY,                      \
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),                           \
	BT_GATT_CCC(acs_data_out_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#else
#define ACS_DON_ATTRS
#endif

#if ACS_SERVICE_HAS_DOI
#define ACS_DOI_ATTRS                                                                             \
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_ACS_DOI, BT_GATT_CHRC_INDICATE,                   \
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),                           \
	BT_GATT_CCC(acs_data_out_ind_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#else
#define ACS_DOI_ATTRS
#endif

BT_GATT_SERVICE_DEFINE(acs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_ACLS),
		       BT_GATT_CHARACTERISTIC(BT_UUID_GATT_ACS_S,
					      BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE,
					      BT_GATT_PERM_READ, acs_status_read, NULL, NULL),
		       BT_GATT_CCC(acs_status_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
		       ACS_DATA_IN_ATTRS ACS_DON_ATTRS ACS_DOI_ATTRS
			       BT_GATT_CHARACTERISTIC(BT_UUID_GATT_ACS_CP,
						      BT_GATT_CHRC_WRITE | BT_GATT_CHRC_INDICATE,
						      BT_GATT_PERM_WRITE, NULL, acs_cp_write, NULL),
			       BT_GATT_CCC(acs_cp_ccc_changed,
					   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),);

int acs_service_cache_attrs(void)
{
	acs_attrs.status =
		bt_gatt_find_by_uuid(acs_svc.attrs, acs_svc.attr_count, BT_UUID_GATT_ACS_S);
	acs_attrs.cp = bt_gatt_find_by_uuid(acs_svc.attrs, acs_svc.attr_count, BT_UUID_GATT_ACS_CP);
#if ACS_SERVICE_HAS_DON
	acs_attrs.don =
		bt_gatt_find_by_uuid(acs_svc.attrs, acs_svc.attr_count, BT_UUID_GATT_ACS_DON);
#endif
#if ACS_SERVICE_HAS_DOI
	acs_attrs.doi =
		bt_gatt_find_by_uuid(acs_svc.attrs, acs_svc.attr_count, BT_UUID_GATT_ACS_DOI);
#endif

	if (!acs_attrs.status || !acs_attrs.cp) {
		return -ENOENT;
	}

#if ACS_SERVICE_HAS_DON
	if (!acs_attrs.don) {
		return -ENOENT;
	}
#endif
#if ACS_SERVICE_HAS_DOI
	if (!acs_attrs.doi) {
		return -ENOENT;
	}
#endif

	return 0;
}

const struct bt_gatt_attr *acs_service_attr_cp(void)
{
	__ASSERT_NO_MSG(acs_attrs.cp != NULL);
	return acs_attrs.cp;
}

uint16_t acs_service_handle_cp(void)
{
	const struct bt_gatt_attr *attr = acs_service_attr_cp();

	return attr ? bt_gatt_attr_get_handle(attr) : 0U;
}

const struct bt_gatt_attr *acs_service_attr_don(void)
{
#if ACS_SERVICE_HAS_DON
	__ASSERT_NO_MSG(acs_attrs.don != NULL);
	return acs_attrs.don;
#else
	return NULL;
#endif
}

const struct bt_gatt_attr *acs_service_attr_doi(void)
{
#if ACS_SERVICE_HAS_DOI
	__ASSERT_NO_MSG(acs_attrs.doi != NULL);
	return acs_attrs.doi;
#else
	return NULL;
#endif
}
