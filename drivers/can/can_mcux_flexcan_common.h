/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2022 Carl Zeiss Meditec AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CAN_MCUX_FLEXCAN_COMMON_H
#define CAN_MCUX_FLEXCAN_COMMON_H

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <fsl_flexcan.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#define SP_IS_SET(inst) DT_INST_NODE_HAS_PROP(inst, sample_point) ||

/* Macro to exclude the sample point algorithm from compilation if not used
 * Without the macro, the algorithm would always waste ROM
 */
#define USE_SP_ALGO (DT_INST_FOREACH_STATUS_OKAY(SP_IS_SET) 0)

#define SP_AND_TIMING_NOT_SET(inst)                                                                \
	(!DT_INST_NODE_HAS_PROP(inst, sample_point) &&                                             \
	 !(DT_INST_NODE_HAS_PROP(inst, prop_seg) && DT_INST_NODE_HAS_PROP(inst, phase_seg1) &&     \
	   DT_INST_NODE_HAS_PROP(inst, phase_seg2))) ||

#if DT_INST_FOREACH_STATUS_OKAY(SP_AND_TIMING_NOT_SET) 0
#error You must either set a sampling-point or timings (phase-seg* and prop-seg)
#endif

#if ((defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) || \
	(defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829))
/* the first valid MB should be occupied by ERRATA 5461 or 5829. */
#define RX_START_IDX 1
#else
#define RX_START_IDX 0
#endif

/*
 * RX message buffers (filters) will take up the first N message
 * buffers. The rest are available for TX use.
 */
#define MCUX_FLEXCAN_MAX_RX (CONFIG_CAN_MAX_FILTER + RX_START_IDX)

/*
 * Convert from RX message buffer index to allocated filter ID and
 * vice versa.
 */
#define RX_MBIDX_TO_ALLOC_IDX(x) (x)
#define ALLOC_IDX_TO_RXMB_IDX(x) (x)

/*
 * Convert from TX message buffer index to allocated TX ID and vice
 * versa.
 */
#define TX_MBIDX_TO_ALLOC_IDX(x) (x - MCUX_FLEXCAN_MAX_RX)
#define ALLOC_IDX_TO_TXMB_IDX(x) (x + MCUX_FLEXCAN_MAX_RX)

/* Convert from back from FLEXCAN IDs to Zephyr CAN IDs. */
#define FLEXCAN_ID_TO_CAN_ID_STD(id)                                                               \
	((uint32_t)((((uint32_t)(id)) & CAN_ID_STD_MASK) >> CAN_ID_STD_SHIFT))
#define FLEXCAN_ID_TO_CAN_ID_EXT(id)                                                               \
	((uint32_t)((((uint32_t)(id)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK)) >> CAN_ID_EXT_SHIFT))

/*
 * The TX buffer differs when FlexCAN-FD is active. This is due to the reduced number of message
 * buffers being available. The buffers are distinguished between one another.,
 * MCUX_FLEXCAN_FD_MAX_TX for FlexCAN-FD and MCUX_FLEXCAN_MAX_TX, when classic can driver is used.
 * This issue is further complicated when FlexCAN-FD driver is selected, but CAN-FD is unselected.
 * The below block goes about configuring the buffer indices.
 */
#ifdef CONFIG_CAN_MCUX_FLEXCAN_FD
#ifdef CONFIG_CAN_FD_MODE
/*
 * Total message buffer count based on the calculated value is the sum of MBDSR0 & MBDSR1 registers
 * which configures the message buffers for Region 0 & Region 1.
 */
#define FLEXCAN_FD_MESSAGE_BUFFER_MAX_NUMBER 14U

/*
 * Tx buffer index count in the case of a FlexCAN-FD driver.
 */
#define MCUX_FLEXCAN_FD_MAX_TX                                            \
	(FLEXCAN_FD_MESSAGE_BUFFER_MAX_NUMBER - MCUX_FLEXCAN_MAX_RX)
#else
/*
 * Tx buffer index count in the case of a FlexCAN-FD driver configured as CAN 2.0 driver.
 */
#define MCUX_FLEXCAN_FD_MAX_TX \
	(FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(0) \
	- MCUX_FLEXCAN_MAX_RX)
#endif /* CONFIG_CAN_FD_MODE */
#endif /* CONFIG_CAN_MCUX_FLEXCAN_FD */

/*
 * Tx buffer index count in the case of a classic FlexCAN driver.
 */
#define MCUX_FLEXCAN_MAX_TX \
	(FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(0) \
	- MCUX_FLEXCAN_MAX_RX)

struct mcux_flexcan_generic_config {
	CAN_Type *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	int clk_source;
	uint32_t bitrate;
	uint32_t sample_point;
	uint32_t sjw;
	uint32_t prop_seg;
	uint32_t phase_seg1;
	uint32_t phase_seg2;
	uint32_t bus_speed_data;
	uint32_t sjw_data;
	uint32_t sample_point_data;
	void (*irq_config_func)(const struct device *dev);
	void (*irq_enable_func)(void);
	void (*irq_disable_func)(void);
	const struct device *phy;
	uint32_t max_bitrate;
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pincfg;
#endif
};

/*
 * RX-Callback struct for FlexCAN classic.
 */
struct mcux_flexcan_classic_rx_callback {
	flexcan_rx_mb_config_t mb_config;
	flexcan_frame_t frame;
	can_rx_callback_t function;
	void *arg;
};

/*
 * TX-Callback struct for FlexCAN classic.
 */
struct mcux_flexcan_classic_tx_callback {
	flexcan_frame_t frame;
	can_tx_callback_t function;
	void *arg;
};

/*
 * Data struct for FlexCAN classic.
 */
struct mcux_flexcan_classic_data {
	const struct device *dev;
	flexcan_handle_t handle;

	ATOMIC_DEFINE(rx_allocs, MCUX_FLEXCAN_MAX_RX);
	struct k_mutex rx_mutex;
	struct mcux_flexcan_classic_rx_callback rx_cbs[MCUX_FLEXCAN_MAX_RX];

	ATOMIC_DEFINE(tx_allocs, MCUX_FLEXCAN_MAX_TX);
	struct k_sem tx_allocs_sem;
	struct k_mutex tx_mutex;
	struct mcux_flexcan_classic_tx_callback tx_cbs[MCUX_FLEXCAN_MAX_TX];
	enum can_state state;
	can_state_change_callback_t state_change_cb;
	void *state_change_cb_data;
	struct can_timing timing;
	bool started;
};

/*
 * Structs for FlexCAN FD are wrapped within compiler guards since the FlexCAN FD types are only
 * visible when CONFIG_CAN_MCUX_FLEXCAN_FD is active.
 */

#ifdef CONFIG_CAN_MCUX_FLEXCAN_FD
/*
 * RX-Callback struct for FlexCAN FD.
 */
struct mcux_flexcan_fd_rx_callback {
	flexcan_rx_mb_config_t mb_config;
/*
 * Select frame type based on CAN_FD mode.
 */
#ifdef CONFIG_CAN_FD_MODE
	flexcan_fd_frame_t frame;
#else
	flexcan_frame_t frame;
#endif /*CONFIG_CAN_FD_MODE */
	can_rx_callback_t function;
	void *arg;
};

/*
 * TX-Callback struct for FlexCAN FD.
 */
struct mcux_flexcan_fd_tx_callback {
/*
 * Select frame type based on CAN_FD mode.
 */
#ifdef CONFIG_CAN_FD_MODE
	flexcan_fd_frame_t frame;
#else
	flexcan_frame_t frame;
#endif
	can_tx_callback_t function;
	void *arg;
};

/*
 * Data struct for FlexCAN FD.
 */
struct mcux_flexcan_fd_data {
	const struct device *dev;
	flexcan_handle_t handle;

	ATOMIC_DEFINE(rx_allocs, MCUX_FLEXCAN_MAX_RX);
	struct k_mutex rx_mutex;
	struct mcux_flexcan_fd_rx_callback rx_cbs[MCUX_FLEXCAN_MAX_RX];

	ATOMIC_DEFINE(tx_allocs, MCUX_FLEXCAN_FD_MAX_TX);
	struct k_sem tx_allocs_sem;
	struct k_mutex tx_mutex;
	struct mcux_flexcan_fd_tx_callback tx_cbs[MCUX_FLEXCAN_FD_MAX_TX];
	enum can_state state;
	can_state_change_callback_t state_change_cb;
	void *state_change_cb_data;
	struct can_timing timing;
#ifdef CONFIG_CAN_FD_MODE
	struct can_timing timing_data;
#endif
	bool started;
};

#endif /* CONFIG_CAN_MCUX_FLEXCAN_FD */

int mcux_flexcan_get_max_filters(const struct device *dev, bool ide);

int mcux_flexcan_get_max_bitrate(const struct device *dev, uint32_t *max_bitrate);

void mcux_flexcan_config_ctrl1(can_mode_t mode, CAN_Type *can_base);

void mcux_flexcan_config_mcr(can_mode_t mode, CAN_Type *can_base);

void mcux_flexcan_can_filter_to_mbconfig(const struct can_filter *src, flexcan_rx_mb_config_t *dest,
					 uint32_t *mask);

int mcux_flexcan_get_core_clock(const struct device *dev, uint32_t *rate);

enum can_state mcux_flexcan_read_status_flags(const struct mcux_flexcan_generic_config *config);

int mcux_flexcan_config_can_calc_prescaler(const struct device *dev,
					   const struct mcux_flexcan_generic_config *const config,
					   struct can_timing *const timing);

void mcux_flexcan_init_common_config(flexcan_config_t *const flexcan_config,
				     const struct can_timing *const timing,
				     const uint32_t clock_freq, const int clock_source,
				     const uint8_t max_mb);

void increment_error_counters(const struct device *dev, uint64_t error);

/* Function for copying CAN 2.0 frames */
void mcux_flexcan_from_can_frame(const struct can_frame *src, flexcan_frame_t *dest);

void mcux_flexcan_to_can_frame(const flexcan_frame_t *src, struct can_frame *dest);

/* Function for copying CAN FD frames */
#ifdef CONFIG_CAN_FD_MODE
void mcux_flexcan_fd_from_can_frame(const struct can_frame *src, flexcan_fd_frame_t *dest);

void mcux_flexcan_fd_to_can_frame(const flexcan_fd_frame_t *src, struct can_frame *dest);
#endif
#endif /* CAN_MCUX_FLEXCAN_COMMON_H */
