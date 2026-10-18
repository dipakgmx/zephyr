/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SAMPLE_ACS_H_
#define SAMPLE_ACS_H_

#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/services/acs.h>

/* Each choice option depends on the Kconfig enabling its algorithm, so the
 * selected ISC is always compiled in.
 */
#if IS_ENABLED(CONFIG_SAMPLE_ACS_ISC_GCM)
#define SAMPLE_ACS_ISC_ID BT_ACS_ISC_ID_HIGH_SEC_GCM
#elif IS_ENABLED(CONFIG_SAMPLE_ACS_ISC_CCM)
#define SAMPLE_ACS_ISC_ID BT_ACS_ISC_ID_HIGH_SEC_CCM
#elif IS_ENABLED(CONFIG_SAMPLE_ACS_ISC_GMAC)
#define SAMPLE_ACS_ISC_ID BT_ACS_ISC_ID_INTEGRITY_GMAC
#elif IS_ENABLED(CONFIG_SAMPLE_ACS_ISC_CMAC)
#define SAMPLE_ACS_ISC_ID BT_ACS_ISC_ID_MAC_ONLY_CMAC
#else
#error "peripheral_acs requires one of CONFIG_SAMPLE_ACS_ISC_{GCM,CCM,GMAC,CMAC}"
#endif

/* The sample's two restriction maps, defined in main.c. The locked map protects
 * every CTS and HRS resource, the relaxed map only Current Time read and write.
 * Only ID 0 is reserved (ACS 1.0 Section 4.4.3.2).
 */
#define SAMPLE_ACS_RMAP_LOCKED_ID  1
#define SAMPLE_ACS_RMAP_RELAXED_ID 2

BT_ACS_RESTRICTION_MAP_EXTERN(locked_map);
BT_ACS_RESTRICTION_MAP_EXTERN(relaxed_map);

#endif /* SAMPLE_ACS_H_ */
