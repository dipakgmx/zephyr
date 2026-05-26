/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/acs.h>
#include <zephyr/logging/log.h>

#include "acs_internal.h"
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#include "acs_rmap.h"
#endif

LOG_MODULE_REGISTER(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

BUILD_ASSERT(IS_ENABLED(CONFIG_BT_ACS_DESCRIPTORS) ||
		     IS_ENABLED(CONFIG_BT_ACS_RESOURCE_HANDLE_UUID_MAP),
	     "ACS requires descriptors or resource-handle UUID mapping support");

static const struct bt_acs_cb *acs_cb;
static bool acs_initialized;

static struct k_work_q acs_work_q;
K_THREAD_STACK_DEFINE(acs_work_q_stack, CONFIG_BT_ACS_WORKQUEUE_STACK_SIZE);
static const struct k_work_queue_config acs_work_q_config = {
	.name = "BT_ACS_WQ",
};

bool acs_is_initialized(void)
{
	return acs_initialized;
}

const struct bt_acs_cb *acs_cb_get(void)
{
	return acs_cb;
}

struct k_work_q *acs_get_wq(void)
{
	return &acs_work_q;
}

static ssize_t acs_status_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	uint8_t status_data[ACS_STATUS_SIZE];
	struct bt_acs_conn const *acs_conn = acs_conn_lookup(conn);

	if (acs_conn != NULL) {
		status_data[0] = acs_conn->status_flags;
		sys_put_le16(acs_conn->restriction_map_id, &status_data[1]);
	} else {
		status_data[0] = BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
		sys_put_le16(IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
				     ? CONFIG_BT_ACS_ACTIVE_RMAP_ID
				     : 0,
			     &status_data[1]);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, status_data, sizeof(status_data));
}

static void acs_status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_DBG("status CCC %s", (value == BT_GATT_CCC_INDICATE) ? "enabled" : "disabled");
}

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
static void acs_don_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_DBG("data-out notify CCC %s", (value == BT_GATT_CCC_NOTIFY) ? "enabled" : "disabled");
}
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION */

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
static void acs_doi_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_DBG("data-out indicate CCC %s",
		(value == BT_GATT_CCC_INDICATE) ? "enabled" : "disabled");
}
#endif /* CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION */

static void acs_cp_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_DBG("control-point CCC %s", (value == BT_GATT_CCC_INDICATE) ? "enabled" : "disabled");
}

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_WRITE) ||                                          \
	IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_READ)
#define ACS_DATA_IN_ATTRS                                                                          \
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_ACS_DI, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL,  \
			       acs_data_in_write, NULL),
#else
#define ACS_DATA_IN_ATTRS
#endif

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION) ||                                   \
	IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_READ)
#define ACS_DON_ATTRS                                                                              \
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_ACS_DON, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, \
			       NULL, NULL),                                                        \
		BT_GATT_CCC(acs_don_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#else
#define ACS_DON_ATTRS
#endif

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
#define ACS_DOI_ATTRS                                                                              \
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_ACS_DOI, BT_GATT_CHRC_INDICATE, BT_GATT_PERM_NONE,     \
			       NULL, NULL, NULL),                                                  \
		BT_GATT_CCC(acs_doi_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#else
#define ACS_DOI_ATTRS
#endif

/* clang-format off */
BT_GATT_SERVICE_DEFINE(
	acs_svc,
	/* Primary Service: Authorization Control Service */
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ACLS),

	/* Status characteristic: Read + Indicate */
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_ACS_S, BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_READ, acs_status_read, NULL, NULL),
	BT_GATT_CCC(acs_status_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* Data In: conditional on protected write support (spec Table 4.1, C.1) */
	ACS_DATA_IN_ATTRS

	/* Data Out Notify: conditional on notification protection (C.2) */
	ACS_DON_ATTRS

	/* Data Out Indicate: conditional on indication protection (C.3) */
	ACS_DOI_ATTRS

	/* Control Point characteristic: Write + Indicate */
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_ACS_CP,
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_WRITE, NULL, acs_cp_write,
			       NULL),
	BT_GATT_CCC(acs_cp_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

/* clang-format on */

/** Cached GATT attribute pointers, populated once during bt_acs_init(). */
static struct {
	const struct bt_gatt_attr *status;
	const struct bt_gatt_attr *cp;
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	const struct bt_gatt_attr *don;
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	const struct bt_gatt_attr *doi;
#endif
} acs_attrs;

const struct bt_gatt_attr *acs_attr_status(void)
{
	return acs_attrs.status;
}

const struct bt_gatt_attr *acs_attr_cp(void)
{
	return acs_attrs.cp;
}

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
const struct bt_gatt_attr *acs_attr_don(void)
{
	return acs_attrs.don;
}
#endif

#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
const struct bt_gatt_attr *acs_attr_doi(void)
{
	return acs_attrs.doi;
}
#endif

static int acs_gatt_attrs_cache(void)
{
	acs_attrs.status =
		bt_gatt_find_by_uuid(acs_svc.attrs, acs_svc.attr_count, BT_UUID_GATT_ACS_S);
	acs_attrs.cp = bt_gatt_find_by_uuid(acs_svc.attrs, acs_svc.attr_count, BT_UUID_GATT_ACS_CP);
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	acs_attrs.don =
		bt_gatt_find_by_uuid(acs_svc.attrs, acs_svc.attr_count, BT_UUID_GATT_ACS_DON);
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	acs_attrs.doi =
		bt_gatt_find_by_uuid(acs_svc.attrs, acs_svc.attr_count, BT_UUID_GATT_ACS_DOI);
#endif

	if (!acs_attrs.status || !acs_attrs.cp
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	    || !acs_attrs.don
#endif
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	    || !acs_attrs.doi
#endif
	) {
		return -ENOENT;
	}

	return 0;
}

int acs_cp_ccc_check(struct bt_conn *conn)
{
	__ASSERT_NO_MSG(conn != NULL);
	__ASSERT_NO_MSG(acs_attrs.cp != NULL);
	return bt_gatt_is_subscribed(conn, acs_attrs.cp, BT_GATT_CCC_INDICATE) ? 0 : -EINVAL;
}

int acs_don_ccc_check(struct bt_conn *conn)
{
	__ASSERT_NO_MSG(conn != NULL);
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_NOTIFICATION)
	__ASSERT_NO_MSG(acs_attrs.don != NULL);
	return bt_gatt_is_subscribed(conn, acs_attrs.don, BT_GATT_CCC_NOTIFY) ? 0 : -EINVAL;
#else
	return -ENOTSUP;
#endif
}

int acs_doi_ccc_check(struct bt_conn *conn)
{
	__ASSERT_NO_MSG(conn != NULL);
#if IS_ENABLED(CONFIG_BT_ACS_PROTECTED_RESOURCE_INDICATION)
	__ASSERT_NO_MSG(acs_attrs.doi != NULL);
	return bt_gatt_is_subscribed(conn, acs_attrs.doi, BT_GATT_CCC_INDICATE) ? 0 : -EINVAL;
#else
	return -ENOTSUP;
#endif
}

static void acs_status_indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params,
				   uint8_t err)
{
	if (err) {
		LOG_WRN("Status indication complete with error: %u", err);
	}

	params->attr = NULL;
}

void acs_status_indicate(struct bt_conn *conn)
{
	struct bt_acs_conn *acs_conn;
	int ret;

	acs_conn = acs_conn_lookup(conn);

	if (!acs_conn) {
		LOG_WRN("No ACS connection");
		return;
	}

	if (acs_conn->status_indicate_params.attr != NULL) {
		LOG_WRN("Status indication already in flight");
		return;
	}

	acs_conn->status_data[0] = acs_conn->status_flags;
	sys_put_le16(acs_conn->restriction_map_id, &acs_conn->status_data[1]);

	memset(&acs_conn->status_indicate_params, 0, sizeof(acs_conn->status_indicate_params));
	acs_conn->status_indicate_params.attr = acs_conn->attr_status;
	acs_conn->status_indicate_params.func = acs_status_indicate_cb;
	acs_conn->status_indicate_params.data = acs_conn->status_data;
	acs_conn->status_indicate_params.len = ACS_STATUS_SIZE;

	ret = bt_gatt_indicate(conn, &acs_conn->status_indicate_params);
	if (ret) {
		acs_conn->status_indicate_params.attr = NULL;
		LOG_WRN("Status indication failed: %d", ret);
	}
}

int bt_acs_init(const struct bt_acs_cb *cb)
{
	int ret;

	__ASSERT(cb != NULL, "ACS callback struct pointer cannot be NULL");

	if (acs_initialized) {
		LOG_WRN("ACS callback already initialized");
		return -EALREADY;
	}

	acs_cb = cb;

	k_work_queue_init(&acs_work_q);
	k_work_queue_start(&acs_work_q, acs_work_q_stack, K_THREAD_STACK_SIZEOF(acs_work_q_stack),
			   CONFIG_BT_ACS_WORKQUEUE_THREAD_PRIO, &acs_work_q_config);

	ret = acs_gatt_attrs_cache();
	if (ret) {
		LOG_ERR("Failed to cache ACS GATT attributes: %d", ret);
		return ret;
	}

#if IS_ENABLED(CONFIG_BT_ACS_GATT_AUTHORIZATION)
	ret = acs_policy_register_gatt_auth_cb();
	if (ret) {
		LOG_ERR("Failed to register GATT authorization callback: %d", ret);
		return ret;
	}
#endif

#if defined(CONFIG_BT_SETTINGS)
	acs_session_register_auth_info_cb();
#endif

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	/* Resolve ATT handles for all RMAP protection entries. */
	ret = acs_rmap_resolve_handles();
	if (ret) {
		LOG_ERR("ACS rmap handle resolution failed: %d", ret);
		return ret;
	}
#endif /* CONFIG_BT_ACS_FEAT_AUTHORIZATION */

#if IS_ENABLED(CONFIG_BT_ACS_GATT_AUTHORIZATION)
	acs_policy_resolve_protected_cccds();
#endif

	acs_initialized = true;

	return 0;
}

int bt_acs_set_oob_number(struct bt_conn *conn, const uint8_t *oob, uint16_t len)
{
	struct bt_acs_conn *acs_conn;

	if (!conn || !oob || len == 0 || len > ACS_CONFIRM_VALUE_SIZE) {
		return -EINVAL;
	}

	acs_conn = acs_conn_lookup(conn);
	if (!acs_conn) {
		return -ENOTCONN;
	}

	if (!acs_conn->crypto.kex) {
		LOG_WRN("No key exchange in progress");
		return -ESRCH;
	}

	if (acs_conn->crypto.kex->start_kex.confirmation_method !=
	    BT_ACS_CONFIRM_METHOD_INPUT_OOB) {
		LOG_WRN("Confirmation method is not Input OOB");
		return -EPERM;
	}

	memset(acs_conn->crypto.kex->auth_value, 0, sizeof(acs_conn->crypto.kex->auth_value));
	memcpy(&acs_conn->crypto.kex->auth_value[ACS_CONFIRM_VALUE_SIZE - len], oob, len);

	return 0;
}

uint8_t bt_acs_status_get(struct bt_conn *conn)
{
	struct bt_acs_conn const *acs_conn = acs_conn_lookup(conn);

	if (!acs_conn) {
		return BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	}

	return acs_conn->status_flags;
}

int bt_acs_set_restriction_map(struct bt_conn *conn, uint16_t map_id)
{
	struct bt_acs_conn *acs_conn;

	if (!acs_initialized || !conn) {
		return -EINVAL;
	}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	{
		struct bt_acs_restriction_map map;

		if (acs_rmap_lookup(map_id, &map) != 0) {
			return -EINVAL;
		}
	}
#endif

	acs_conn = acs_conn_lookup(conn);

	if (!acs_conn) {
		return -ENOTCONN;
	}

	acs_conn->restriction_map_id = map_id;

	return 0;
}
