/* sample_hrs.h - HRS callbacks and ACS restriction map entries for the peripheral_acs sample */

/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SAMPLE_HRS_H_
#define SAMPLE_HRS_H_

#include <stdint.h>

int sample_hrs_init(void);

/**
 * @brief Notify a new heart rate measurement.
 *
 * @param bpm   Heart rate in beats per minute.
 * @param ee_kj Energy to add to the running energy-expended counter (kJ).
 */
void sample_hrs_notify(uint16_t bpm, uint32_t ee_kj);

#endif /* SAMPLE_HRS_H_ */
