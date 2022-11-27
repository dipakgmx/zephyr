/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kinetis_flexcan

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
#include "can_mcux_flexcan_common.h"
#define MCUX_FLEXCAN_TYPE__ classic
#include "can_mcux_flexcan_template.c"
#undef MCUX_FLEXCAN_TYPE__
#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

static int mcux_flexcan_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_3_SAMPLES;

	return 0;
}

static const struct can_driver_api mcux_flexcan_driver_api = {
	.get_capabilities = mcux_flexcan_get_capabilities,
	.start = mcux_flexcan_classic_start,
	.stop = mcux_flexcan_classic_stop,
	.set_mode = mcux_flexcan_classic_set_mode,
	.set_timing = mcux_flexcan_classic_set_timing,
	.send = mcux_flexcan_classic_send,
	.add_rx_filter = mcux_flexcan_classic_add_rx_filter,
	.remove_rx_filter = mcux_flexcan_classic_remove_rx_filter,
	.get_state = mcux_flexcan_classic_get_state,
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover = mcux_flexcan_classic_recover,
#endif
	.set_state_change_callback = mcux_flexcan_classic_set_state_change_callback,
	.get_core_clock = mcux_flexcan_get_core_clock,
	.get_max_filters = mcux_flexcan_get_max_filters,
	.get_max_bitrate = mcux_flexcan_get_max_bitrate,
	/*
	 * FlexCAN timing limits are specified in the "FLEXCANx_CTRL1 field
	 * descriptions" table in the SoC reference manual.
	 *
	 * Note that the values here are the "physical" timing limits, whereas
	 * the register field limits are physical values minus 1 (which is
	 * handled by the flexcan_config_t field assignments elsewhere in this
	 * driver).
	 */
	.timing_min = {
		.sjw = 0x01,
		.prop_seg = 0x01,
		.phase_seg1 = 0x01,
		.phase_seg2 = 0x02,
		.prescaler = 0x01
	},
	.timing_max = {
		.sjw = 0x04,
		.prop_seg = 0x08,
		.phase_seg1 = 0x08,
		.phase_seg2 = 0x08,
		.prescaler = 0x100
	}
};

#define FLEXCAN_IRQ_BY_IDX(node_id, prop, idx, cell) \
	DT_IRQ_BY_NAME(node_id, \
		DT_STRING_TOKEN_BY_IDX(node_id, prop, idx), cell)

#define FLEXCAN_IRQ_ENABLE_CODE(node_id, prop, idx) \
	irq_enable(FLEXCAN_IRQ_BY_IDX(node_id, prop, idx, irq));

#define FLEXCAN_IRQ_DISABLE_CODE(node_id, prop, idx) \
	irq_disable(FLEXCAN_IRQ_BY_IDX(node_id, prop, idx, irq));

#define FLEXCAN_IRQ_CONFIG_CODE(node_id, prop, idx) \
	do {								\
		IRQ_CONNECT(FLEXCAN_IRQ_BY_IDX(node_id, prop, idx, irq), \
		FLEXCAN_IRQ_BY_IDX(node_id, prop, idx, priority), \
		mcux_flexcan_classic_isr, \
		DEVICE_DT_GET(node_id), 0); \
		FLEXCAN_IRQ_ENABLE_CODE(node_id, prop, idx); \
	} while (false);

#ifdef CONFIG_PINCTRL
#define FLEXCAN_PINCTRL_DEFINE(id) PINCTRL_DT_INST_DEFINE(id);
#define FLEXCAN_PINCTRL_INIT(id) .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(id),
#else
#define FLEXCAN_PINCTRL_DEFINE(id)
#define FLEXCAN_PINCTRL_INIT(id)
#endif /* CONFIG_PINCTRL */

#define FLEXCAN_DEVICE_INIT_MCUX(id)					\
	FLEXCAN_PINCTRL_DEFINE(id)					\
									\
	static void mcux_flexcan_irq_config_##id(const struct device *dev); \
	static void mcux_flexcan_irq_enable_##id(void); \
	static void mcux_flexcan_irq_disable_##id(void); \
									\
	static const struct mcux_flexcan_generic_config mcux_flexcan_generic_config_##id = { \
		.base = (CAN_Type *)DT_INST_REG_ADDR(id),		\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(id)),	\
		.clock_subsys = (clock_control_subsys_t)		\
			DT_INST_CLOCKS_CELL(id, name),			\
		.clk_source = DT_INST_PROP(id, clk_source),		\
		.bitrate = DT_INST_PROP(id, bus_speed),			\
		.sjw = DT_INST_PROP(id, sjw),				\
		.prop_seg = DT_INST_PROP_OR(id, prop_seg, 0),		\
		.phase_seg1 = DT_INST_PROP_OR(id, phase_seg1, 0),	\
		.phase_seg2 = DT_INST_PROP_OR(id, phase_seg2, 0),	\
		.sample_point = DT_INST_PROP_OR(id, sample_point, 0),	\
		.irq_config_func = mcux_flexcan_irq_config_##id,	\
		.irq_enable_func = mcux_flexcan_irq_enable_##id,	\
		.irq_disable_func = mcux_flexcan_irq_disable_##id,	\
		.phy = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(id, phys)), \
		.max_bitrate = DT_INST_CAN_TRANSCEIVER_MAX_BITRATE(id, 1000000), \
		FLEXCAN_PINCTRL_INIT(id)				\
	};								\
									\
	static struct mcux_flexcan_classic_data mcux_flexcan_data_##id;		\
									\
	CAN_DEVICE_DT_INST_DEFINE(id, mcux_flexcan_classic_init,		\
				  NULL, &mcux_flexcan_data_##id,	\
				  &mcux_flexcan_generic_config_##id,		\
				  POST_KERNEL, CONFIG_CAN_INIT_PRIORITY, \
				  &mcux_flexcan_driver_api);		\
									\
	static void mcux_flexcan_irq_config_##id(const struct device *dev) \
	{								\
		DT_INST_FOREACH_PROP_ELEM(id, interrupt_names, FLEXCAN_IRQ_CONFIG_CODE); \
	} \
									\
	static void mcux_flexcan_irq_enable_##id(void) \
	{								\
		DT_INST_FOREACH_PROP_ELEM(id, interrupt_names, FLEXCAN_IRQ_ENABLE_CODE); \
	} \
									\
	static void mcux_flexcan_irq_disable_##id(void) \
	{								\
		DT_INST_FOREACH_PROP_ELEM(id, interrupt_names, FLEXCAN_IRQ_DISABLE_CODE); \
	}

DT_INST_FOREACH_STATUS_OKAY(FLEXCAN_DEVICE_INIT_MCUX)
