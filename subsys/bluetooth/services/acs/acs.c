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
#include "acs_runtime.h"
#include "acs_cp.h"
#include "acs_rhandle.h"
#if defined(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
#include "acs_isc.h"
#include "acs_key_desc.h"
#endif
#if defined(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
#include "acs_rmap.h"
#endif

LOG_MODULE_REGISTER(bt_acs, CONFIG_BT_ACS_LOG_LEVEL);

/* Authentication requires at least one data-protection algorithm. */
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION) ||
		     IS_ENABLED(CONFIG_BT_ACS_ANY_DATA_PROTECTION),
	     "ACS authentication requires at least one data-protection algorithm");

/* Delay before re-attempting a Status indication. */
#define ACS_STATUS_RETRY_DELAY K_MSEC(50)

static void acs_att_mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx);

static const struct bt_acs_cb *acs_cb;
static bool acs_initialized;
static atomic_t acs_security_controls_enabled = ATOMIC_INIT(1);
static struct k_work_q acs_work_q;
K_THREAD_STACK_DEFINE(acs_work_q_stack, CONFIG_BT_ACS_WORKQUEUE_STACK_SIZE);
static const struct k_work_queue_config acs_work_q_config = {
	.name = "BT_ACS_WQ",
};
static struct bt_gatt_cb acs_gatt_cb = {
	.att_mtu_updated = acs_att_mtu_updated,
};

/* Data In and Data Out are present only when authentication is enabled. */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
#define ACS_DATA_IN_ATTRS                                                                          \
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_ACS_DI, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL,  \
			       acs_data_in_write, NULL),
#define ACS_DON_ATTRS                                                                              \
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_ACS_DON, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, \
			       NULL, NULL),                                                        \
		BT_GATT_CCC(acs_don_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#define ACS_DOI_ATTRS                                                                              \
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_ACS_DOI, BT_GATT_CHRC_INDICATE, BT_GATT_PERM_NONE,     \
			       NULL, NULL, NULL),                                                  \
		BT_GATT_CCC(acs_doi_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#else
#define ACS_DATA_IN_ATTRS
#define ACS_DON_ATTRS
#define ACS_DOI_ATTRS
#endif

/* GATT attributes and handles cached during bt_acs_init(). */
static struct {
	const struct bt_gatt_attr *status;
	const struct bt_gatt_attr *cp;
	uint16_t cp_attr_handle;     /* ACS Control Point GATT Attribute Handle */
	uint16_t cp_resource_handle; /* ACS Control Point Resource Handle */
	uint16_t first_attr_handle;  /* Service declaration GATT Attribute Handle */
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	const struct bt_gatt_attr *don;
	const struct bt_gatt_attr *doi;
#endif
} acs_attrs;

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

/* Build the Status_Flags field (Table 4.7); acs_conn may be NULL. */
static uint8_t acs_status_flags(const struct bt_acs_conn *acs_conn)
{
	bool established = acs_conn != NULL &&
			   atomic_test_bit(&acs_conn->state, ACS_STATE_SECURITY_ESTABLISHED);
	uint8_t flags = established ? BT_ACS_STATUS_SECURITY_ESTABLISHED : 0U;

	if (acs_security_switch_get()) {
		flags |= BT_ACS_STATUS_SECURITY_CONTROLS_ENABLED;
	}
	return flags;
}

/* Build the Status value (Table 4.7); acs_conn may be NULL. */
static void acs_status_fill(const struct bt_acs_conn *acs_conn, uint8_t out[ACS_STATUS_SIZE])
{
	uint16_t map_id = acs_rmap_active_id();

	out[0] = acs_status_flags(acs_conn);
	sys_put_le16(map_id, &out[1]);
}

static ssize_t acs_status_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	uint8_t status_data[ACS_STATUS_SIZE];

	acs_status_fill(acs_conn_lookup(conn), status_data);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, status_data, sizeof(status_data));
}

static void acs_status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_DBG("status CCC %s", (value == BT_GATT_CCC_INDICATE) ? "enabled" : "disabled");
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
static void acs_don_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_DBG("data-out notify CCC %s", (value == BT_GATT_CCC_NOTIFY) ? "enabled" : "disabled");
}

static void acs_doi_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_DBG("data-out indicate CCC %s",
		(value == BT_GATT_CCC_INDICATE) ? "enabled" : "disabled");
}
#endif /* CONFIG_BT_ACS_FEAT_AUTHENTICATION */

static void acs_cp_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_DBG("control-point CCC %s", (value == BT_GATT_CCC_INDICATE) ? "enabled" : "disabled");
}

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

static const struct bt_gatt_attr *acs_attr_status(void)
{
	return acs_attrs.status;
}

const struct bt_gatt_attr *acs_attr_cp(void)
{
	return acs_attrs.cp;
}

uint16_t acs_cp_attr_handle(void)
{
	return acs_attrs.cp_attr_handle;
}

uint16_t acs_cp_resource_handle(void)
{
	return acs_attrs.cp_resource_handle;
}

bool acs_handle_is_own(uint16_t handle)
{
	uint16_t first = acs_attrs.first_attr_handle;

	return first != 0U && handle >= first && handle < first + acs_svc.attr_count;
}

#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
const struct bt_gatt_attr *acs_attr_don(void)
{
	return acs_attrs.don;
}

const struct bt_gatt_attr *acs_attr_doi(void)
{
	return acs_attrs.doi;
}
#endif

static int acs_gatt_attrs_cache(void)
{
	uint16_t found_attr_handle = 0U;
	int err;

	acs_attrs.status =
		bt_gatt_find_by_uuid(acs_svc.attrs, acs_svc.attr_count, BT_UUID_GATT_ACS_S);
	acs_attrs.cp = bt_gatt_find_by_uuid(acs_svc.attrs, acs_svc.attr_count, BT_UUID_GATT_ACS_CP);
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	acs_attrs.don =
		bt_gatt_find_by_uuid(acs_svc.attrs, acs_svc.attr_count, BT_UUID_GATT_ACS_DON);
	acs_attrs.doi =
		bt_gatt_find_by_uuid(acs_svc.attrs, acs_svc.attr_count, BT_UUID_GATT_ACS_DOI);
#endif

	if (acs_attrs.status == NULL || acs_attrs.cp == NULL
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	    || acs_attrs.don == NULL || acs_attrs.doi == NULL
#endif
	) {
		return -ENOENT;
	}

	acs_attrs.cp_attr_handle = bt_gatt_attr_get_handle(acs_attrs.cp);

	/* Cache the service range for ATT authorization checks. */
	acs_attrs.first_attr_handle =
		acs_svc.attr_count ? bt_gatt_attr_get_handle(&acs_svc.attrs[0]) : 0U;

	err = acs_rhandle_find_char_attr_handles(BT_UUID_GATT_ACS_CP, &acs_attrs.cp_resource_handle,
						 &found_attr_handle);
	if (err != 0) {
		LOG_ERR("Failed to resolve ACS CP resource handle: err=%d", err);
		return err;
	}

	if (found_attr_handle != acs_cp_attr_handle()) {
		LOG_ERR("ACS CP handle mismatch: att=0x%04x expected=0x%04x", found_attr_handle,
			acs_cp_attr_handle());
		return -EINVAL;
	}

	return 0;
}

static int acs_ccc_check(struct bt_conn *conn, const struct bt_gatt_attr *attr, uint16_t ccc_value)
{
	__ASSERT_NO_MSG(conn != NULL);
	__ASSERT_NO_MSG(attr != NULL);
	return bt_gatt_is_subscribed(conn, attr, ccc_value) ? 0 : -EINVAL;
}

int acs_cp_ccc_check(struct bt_conn *conn)
{
	return acs_ccc_check(conn, acs_attrs.cp, BT_GATT_CCC_INDICATE);
}

int acs_don_ccc_check(struct bt_conn *conn)
{
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	return acs_ccc_check(conn, acs_attrs.don, BT_GATT_CCC_NOTIFY);
#else
	return -ENOTSUP;
#endif
}

int acs_doi_ccc_check(struct bt_conn *conn)
{
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	return acs_ccc_check(conn, acs_attrs.doi, BT_GATT_CCC_INDICATE);
#else
	return -ENOTSUP;
#endif
}

static void acs_status_schedule_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params,
				   uint8_t err)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (err) {
		LOG_WRN("Status indication complete with error: %u", err);
	}
}

static void acs_status_schedule_destroy(struct bt_gatt_indicate_params *params)
{
	struct bt_acs_conn *acs_conn =
		CONTAINER_OF(params, struct bt_acs_conn, status_indicate_params);

	atomic_set_bit_to(&acs_conn->state, ACS_STATE_STATUS_BUSY, false);
	if (atomic_test_bit(&acs_conn->state, ACS_STATE_STATUS_PENDING)) {
		k_work_reschedule_for_queue(acs_get_wq(), &acs_conn->status_work, K_NO_WAIT);
	}
}

static void acs_status_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct bt_acs_conn *acs_conn = CONTAINER_OF(dwork, struct bt_acs_conn, status_work);
	struct bt_conn *conn = acs_conn->conn;
	int err;

	if (conn == NULL || atomic_test_bit(&acs_conn->state, ACS_STATE_STATUS_BUSY)) {
		return;
	}

	if (!atomic_test_and_clear_bit(&acs_conn->state, ACS_STATE_STATUS_PENDING)) {
		return;
	}

	atomic_set_bit_to(&acs_conn->state, ACS_STATE_STATUS_BUSY, true);
	acs_status_fill(acs_conn, acs_conn->status_data);

	memset(&acs_conn->status_indicate_params, 0, sizeof(acs_conn->status_indicate_params));
	acs_conn->status_indicate_params.attr = acs_attr_status();
	acs_conn->status_indicate_params.func = acs_status_schedule_cb;
	acs_conn->status_indicate_params.destroy = acs_status_schedule_destroy;
	acs_conn->status_indicate_params.data = acs_conn->status_data;
	acs_conn->status_indicate_params.len = ACS_STATUS_SIZE;

	err = bt_gatt_indicate(conn, &acs_conn->status_indicate_params);
	if (err != 0) {
		/* A rejected submission has no destroy callback to schedule the retry. */
		LOG_WRN("Status indication failed: %d", err);
		atomic_set_bit_to(&acs_conn->state, ACS_STATE_STATUS_BUSY, false);
		atomic_set_bit(&acs_conn->state, ACS_STATE_STATUS_PENDING);
		k_work_reschedule_for_queue(acs_get_wq(), &acs_conn->status_work,
					    ACS_STATUS_RETRY_DELAY);
	}
}

void acs_status_schedule(struct bt_conn *conn)
{
	struct bt_acs_conn *acs_conn = acs_conn_lookup(conn);

	if (acs_conn == NULL) {
		LOG_WRN("No ACS connection");
		return;
	}

	atomic_set_bit(&acs_conn->state, ACS_STATE_STATUS_PENDING);
	k_work_reschedule_for_queue(acs_get_wq(), &acs_conn->status_work, K_NO_WAIT);
}

void acs_status_work_init(struct bt_acs_conn *acs_conn)
{
	k_work_init_delayable(&acs_conn->status_work, acs_status_work_handler);
}

static void acs_att_mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	struct bt_acs_conn *acs_conn = acs_conn_lookup(conn);

	ARG_UNUSED(tx);
	ARG_UNUSED(rx);

	if (acs_conn != NULL) {
		acs_cp_indicate_att_mtu(acs_conn);
	}
}

static int acs_init_authentication(void)
{
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHENTICATION)
	int ret;

	ret = acs_isc_validate_records();
	if (ret != 0) {
		LOG_ERR("ISC record validation failed: %d", ret);
		return ret;
	}

	ret = acs_key_desc_validate_records();
	if (ret != 0) {
		LOG_ERR("Key descriptor record validation failed: %d", ret);
		return ret;
	}

	acs_key_store_init();
#endif
	return 0;
}

/* Fail initialization if configured protection cannot be enforced (§3.1). */
static int acs_init_authorization(void)
{
#if IS_ENABLED(CONFIG_BT_ACS_FEAT_AUTHORIZATION)
	int ret;

	ret = acs_rmap_init_runtime();
	if (ret != 0) {
		LOG_ERR("Restriction map handle resolution failed: %d", ret);
		return ret;
	}

	ret = acs_rmap_activate(CONFIG_BT_ACS_ACTIVE_RMAP_ID);
	if (ret != 0) {
		LOG_ERR("Active restriction map 0x%04x is not registered",
			CONFIG_BT_ACS_ACTIVE_RMAP_ID);
		return ret;
	}

	ret = acs_policy_register_gatt_auth_cb();
	if (ret != 0) {
		LOG_ERR("Failed to register GATT authorization callback: %d", ret);
		return ret;
	}
#endif
	return 0;
}

int bt_acs_init(const struct bt_acs_cb *cb)
{
	int ret;

	__ASSERT(cb != NULL, "ACS callback struct pointer cannot be NULL");

	if (acs_initialized) {
		LOG_WRN("ACS already initialized");
		return -EALREADY;
	}

	ret = acs_gatt_attrs_cache();
	if (ret != 0) {
		LOG_ERR("Failed to cache ACS GATT attributes: %d", ret);
		return ret;
	}

	ret = acs_init_authentication();
	if (ret != 0) {
		return ret;
	}

	ret = acs_init_authorization();
	if (ret != 0) {
		return ret;
	}
	acs_cp_feature_init();

	bt_gatt_cb_register(&acs_gatt_cb);

	k_work_queue_init(&acs_work_q);
	k_work_queue_start(&acs_work_q, acs_work_q_stack, K_THREAD_STACK_SIZEOF(acs_work_q_stack),
			   CONFIG_BT_ACS_WORKQUEUE_THREAD_PRIO, &acs_work_q_config);

	acs_cb = cb;
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
	if (acs_conn == NULL) {
		return -ENOTCONN;
	}

	if (!acs_kex_in_progress(acs_conn)) {
		LOG_WRN("No key exchange in progress");
		return -ESRCH;
	}

	if (acs_conn->kex->start_kex.confirmation_method != BT_ACS_CONFIRM_METHOD_INPUT_OOB) {
		LOG_WRN("Confirmation method is not Input OOB");
		return -EPERM;
	}

	memset(acs_conn->kex->auth_value, 0, sizeof(acs_conn->kex->auth_value));
	memcpy(&acs_conn->kex->auth_value[ACS_CONFIRM_VALUE_SIZE - len], oob, len);

	return 0;
}

bool acs_security_switch_get(void)
{
	return atomic_get(&acs_security_controls_enabled) != 0;
}

static void acs_status_schedule_conn(struct bt_conn *conn, void *data)
{
	ARG_UNUSED(data);

	acs_status_schedule(conn);
}

void acs_status_schedule_all(void)
{
	bt_conn_foreach(BT_CONN_TYPE_LE, acs_status_schedule_conn, NULL);
}

void acs_security_switch_set(bool enabled)
{
	if (atomic_set(&acs_security_controls_enabled, enabled ? 1 : 0) == (enabled ? 1 : 0)) {
		return;
	}

	acs_status_schedule_all();
}

uint8_t bt_acs_status_get(struct bt_conn *conn)
{
	return acs_status_flags(conn != NULL ? acs_conn_lookup(conn) : NULL);
}
