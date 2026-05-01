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
 * Both are declared in acs_internal.h (so the GATT macros in acs.c can
 * reference them by name) and defined in acs_runtime.c. They drive the
 * pipeline: input checks → channel-RX feed → frame construction → unwrap
 * (Data In only) → classify → procedure build/start → error mapping.
 *
 * No public API beyond those two symbols lives here today.
 */

#endif /* ACS_RUNTIME_H_ */
