/*
 * Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SAMPLE_HRS_H_
#define SAMPLE_HRS_H_

#include <stdint.h>

int sample_hrs_init(void);

/* Notify a heart rate in bpm, adding ee_kj to the energy-expended counter. */
void sample_hrs_notify(uint16_t bpm, uint32_t ee_kj);

#endif /* SAMPLE_HRS_H_ */
