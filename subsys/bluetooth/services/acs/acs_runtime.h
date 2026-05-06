/*
 * Copyright (c) 2026 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACS_RUNTIME_H_
#define ACS_RUNTIME_H_

/*
 * The runtime layer hosts the GATT-write entry points for the ACS service:
 *
 *   acs_cp_write       — ACS Control Point characteristic write handler
 *   acs_data_in_write  — ACS Data In characteristic write handler
 *
 * Both drive the same pipeline: input checks → channel-RX feed → frame
 * construction → unwrap (Data In only) → classify → procedure build/start →
 * error mapping. The dispatch helpers and the Data-In unwrap helper are
 * declared here too so the layer is self-contained.
 */

#include <stddef.h>
#include <stdint.h>
#include <errno.h>

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/net_buf.h>

#include "acs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Data In pipeline error codes (returned by @ref acs_data_in_unwrap_and_route).
 *
 * Mapped by the runtime entry layer to ATT error codes.
 * @{
 */
#define ACS_DATA_ERR_CCC_IMPROPER_CONF         (-EPIPE)
#define ACS_DATA_ERR_NOT_AUTHORIZED            (-EPERM)
#define ACS_DATA_ERR_INVALID_KEY               (-EACCES)
#define ACS_DATA_ERR_RESOURCE_NOT_PROTECTED    (-ENOENT)
#define ACS_DATA_ERR_INCORRECT_SECURITY_CONFIG (-EPROTO)
/** @} */

/** @brief GATT write handler for the ACS Control Point characteristic. */
ssize_t acs_cp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
		     uint16_t len, uint16_t offset, uint8_t flags);

/** @brief GATT write handler for the ACS Data In characteristic. */
ssize_t acs_data_in_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags);

/**
 * @brief Unwrap a complete reassembled Data In payload and route it.
 *
 * Validates the ISC_ID/nonce header, decrypts the payload in place, builds a
 * normalized @ref acs_frame, then hands it to @ref acs_runtime_dispatch_frame.
 *
 * @return 0 on success, or one of @ref ACS_DATA_ERR_* on failure.
 */
int acs_data_in_unwrap_and_route(struct bt_conn *conn, struct bt_acs_conn *acs_conn,
				 struct net_buf_simple *buf);

/**
 * @brief Common normalized-frame dispatch entry point.
 *
 * Single seam between the GATT-write entrypoints and the per-route execution
 * helpers. Classifies @p frame against @p acs_conn's active restriction map
 * (or trivially as @ref ACS_ROUTE_ACS_CP for CP-source frames) and forwards
 * to the matching @c acs_runtime_dispatch_*_frame helper.
 *
 * @return 0 on success, negative errno or @ref ACS_DATA_ERR_* on failure.
 */
int acs_runtime_dispatch_frame(struct acs_frame *frame, struct bt_acs_conn *acs_conn);

/**
 * @brief Dispatch a Data-In frame that targets a protected service CP.
 *
 * Performs the DOI CCC check, allocates a request context, transfers ownership
 * of @c acs_conn->data_rx.buf into the context, then forwards to
 * @ref acs_cp_dispatch. Drops the proc reference here unless a
 * multi-step reply sequence took it over.
 */
int acs_runtime_dispatch_protected_cp_frame(struct acs_frame *frame, struct bt_acs_conn *acs_conn);

/**
 * @brief Dispatch a Data-In frame that targets a protected characteristic.
 *
 * Resolves the required Data Out subscription, allocates a request context,
 * transfers ownership of @c acs_conn->data_rx.buf into the context, then
 * queues the work item that runs the application/auto-respond handler.
 */
int acs_runtime_dispatch_protected_char_frame(struct acs_frame *frame,
					      struct bt_acs_conn *acs_conn);

/**
 * @brief Resolve the Data Out subscription for a protected resource handle.
 *
 * Picks DON or DOI based on the characteristic's properties; for zero-length
 * payloads only DON is checked. Returns 0 if the required CCC is configured,
 * @c -EINVAL when the CCC is missing, or other negative errno on lookup failure.
 */
int acs_require_data_out_subscription(struct bt_conn *conn, uint16_t resource_handle,
				      uint16_t data_length);

#ifdef __cplusplus
}
#endif

#endif /* ACS_RUNTIME_H_ */
