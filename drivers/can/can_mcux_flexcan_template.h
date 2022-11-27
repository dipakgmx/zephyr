/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2022 Carl Zeiss Meditec AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef MCUX_FLEXCAN_TYPE__
#include "can_mcux_flexcan_common.h"

#define GEN_NAME_(a, b) a##b
#define GEN_NAME(a, b)	GEN_NAME_(a, b)
#define GENERATE_FUNCTION_NAME(name)                                                               \
	GEN_NAME(mcux_flexcan_, GEN_NAME(MCUX_FLEXCAN_TYPE__, GEN_NAME(_, name)))

void GENERATE_FUNCTION_NAME(isr)(const struct device *dev);

int GENERATE_FUNCTION_NAME(set_timing)(const struct device *dev, const struct can_timing *timing);

int GENERATE_FUNCTION_NAME(start)(const struct device *dev);

int GENERATE_FUNCTION_NAME(stop)(const struct device *dev);

int GENERATE_FUNCTION_NAME(set_mode)(const struct device *dev, can_mode_t mode);

int GENERATE_FUNCTION_NAME(get_state)(const struct device *dev, enum can_state *state,
				      struct can_bus_err_cnt *err_cnt);

int GENERATE_FUNCTION_NAME(send)(const struct device *dev, const struct can_frame *frame,
				 k_timeout_t timeout, can_tx_callback_t callback, void *user_data);

int GENERATE_FUNCTION_NAME(add_rx_filter)(const struct device *dev, can_rx_callback_t callback,
					  void *user_data, const struct can_filter *filter);

void GENERATE_FUNCTION_NAME(set_state_change_callback)(const struct device *dev,
						       can_state_change_callback_t callback,
						       void *user_data);

void GENERATE_FUNCTION_NAME(remove_rx_filter)(const struct device *dev, int filter_id);

void GENERATE_FUNCTION_NAME(transfer_error_status)(const struct device *dev, uint64_t error);

void GENERATE_FUNCTION_NAME(transfer_tx_idle)(const struct device *dev, uint32_t mb);

void GENERATE_FUNCTION_NAME(transfer_rx_idle)(const struct device *dev, uint32_t mb);

int GENERATE_FUNCTION_NAME(init)(const struct device *dev);


#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY

int CONCAT(mcux_flexcan_, CONCAT(CAN_TYPE__, _recover))(const struct device *dev,
							k_timeout_t timeout);

#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */

#endif /* MCUX_FLEXCAN_TYPE__ */
