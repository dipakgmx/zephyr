/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2022 Carl Zeiss Meditec AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_mcux_flexcan_common.h"


int mcux_flexcan_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(ide);

	return CONFIG_CAN_MAX_FILTER;
}

int mcux_flexcan_get_max_bitrate(const struct device *dev, uint32_t *max_bitrate)
{
	const struct mcux_flexcan_generic_config *config = dev->config;

	*max_bitrate = config->max_bitrate;

	return 0;
}

void mcux_flexcan_config_ctrl1(can_mode_t mode, CAN_Type *can_base)
{
	uint32_t ctrl1 = can_base->CTRL1;

	if ((mode & CAN_MODE_LOOPBACK) != 0) {
		/* Enable loopback and self-reception */
		ctrl1 |= CAN_CTRL1_LPB_MASK;
	} else {
		/* Disable loopback and self-reception */
		ctrl1 &= ~(CAN_CTRL1_LPB_MASK);
	}

	if ((mode & CAN_MODE_LISTENONLY) != 0) {
		/* Enable listen-only mode */
		ctrl1 |= CAN_CTRL1_LOM_MASK;
	} else {
		/* Disable listen-only mode */
		ctrl1 &= ~(CAN_CTRL1_LOM_MASK);
	}

	if ((mode & CAN_MODE_3_SAMPLES) != 0) {
		/* Enable triple sampling mode */
		ctrl1 |= CAN_CTRL1_SMP_MASK;
	} else {
		/* Disable triple sampling mode */
		ctrl1 &= ~(CAN_CTRL1_SMP_MASK);
	}

	can_base->CTRL1 = ctrl1;
}

void mcux_flexcan_config_mcr(can_mode_t mode, CAN_Type *can_base)
{
	uint32_t mcr = can_base->MCR;

	if ((mode & CAN_MODE_LOOPBACK) != 0) {
		/* Enable loopback and self-reception */
		mcr &= ~(CAN_MCR_SRXDIS_MASK);
	} else {
		/* Disable loopback and self-reception */
		mcr |= CAN_MCR_SRXDIS_MASK;
	}
	can_base->MCR = mcr;
}

void mcux_flexcan_can_filter_to_mbconfig(const struct can_filter *src, flexcan_rx_mb_config_t *dest,
					 uint32_t *mask)
{
	static const uint32_t ide_mask = 1U;
	uint32_t rtr_mask = (src->flags & (CAN_FILTER_DATA | CAN_FILTER_RTR)) !=
					    (CAN_FILTER_DATA | CAN_FILTER_RTR)
				    ? 1U
				    : 0U;

	if ((src->flags & CAN_FILTER_IDE) != 0) {
		dest->format = kFLEXCAN_FrameFormatExtend;
		dest->id = FLEXCAN_ID_EXT(src->id);
		*mask = FLEXCAN_RX_MB_EXT_MASK(src->mask, rtr_mask, ide_mask);
	} else {
		dest->format = kFLEXCAN_FrameFormatStandard;
		dest->id = FLEXCAN_ID_STD(src->id);
		*mask = FLEXCAN_RX_MB_STD_MASK(src->mask, rtr_mask, ide_mask);
	}

	if ((src->flags & CAN_FILTER_RTR) != 0) {
		dest->type = kFLEXCAN_FrameTypeRemote;
	} else {
		dest->type = kFLEXCAN_FrameTypeData;
	}
}

int mcux_flexcan_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct mcux_flexcan_generic_config *config = dev->config;

	return clock_control_get_rate(config->clock_dev, config->clock_subsys, rate);
}

enum can_state mcux_flexcan_read_status_flags(const struct mcux_flexcan_generic_config *config)
{
	enum can_state state;
	uint64_t status_flags = FLEXCAN_GetStatusFlags(config->base);

	if ((status_flags & CAN_ESR1_FLTCONF(2)) != 0U) {
		state = CAN_STATE_BUS_OFF;
	} else if ((status_flags & CAN_ESR1_FLTCONF(1)) != 0U) {
		state = CAN_STATE_ERROR_PASSIVE;
	} else if ((status_flags & (kFLEXCAN_TxErrorWarningFlag | kFLEXCAN_RxErrorWarningFlag)) !=
		   0) {
		state = CAN_STATE_ERROR_WARNING;
	} else {
		state = CAN_STATE_ERROR_ACTIVE;
	}
	return state;
}

int mcux_flexcan_config_can_calc_prescaler(const struct device *dev,
					   const struct mcux_flexcan_generic_config *const config,
					   struct can_timing *const timing)
{
	timing->prop_seg = config->prop_seg;
	timing->phase_seg1 = config->phase_seg1;
	timing->phase_seg2 = config->phase_seg2;
	return can_calc_prescaler(dev, timing, config->bitrate);
}

void increment_error_counters(const struct device *dev, const uint64_t error)
{
	if ((error & kFLEXCAN_Bit0Error) != 0U) {
		CAN_STATS_BIT0_ERROR_INC(dev);
	}

	if ((error & kFLEXCAN_Bit1Error) != 0U) {
		CAN_STATS_BIT1_ERROR_INC(dev);
	}

	if ((error & kFLEXCAN_AckError) != 0U) {
		CAN_STATS_ACK_ERROR_INC(dev);
	}

	if ((error & kFLEXCAN_StuffingError) != 0U) {
		CAN_STATS_STUFF_ERROR_INC(dev);
	}

	if ((error & kFLEXCAN_FormError) != 0U) {
		CAN_STATS_FORM_ERROR_INC(dev);
	}

	if ((error & kFLEXCAN_CrcError) != 0U) {
		CAN_STATS_CRC_ERROR_INC(dev);
	}

	/* Handle FlexCAN FD errors */
#ifdef CONFIG_CAN_FD_MODE
	if ((error & kFLEXCAN_FDBit0Error) != 0U) {
		CAN_STATS_BIT0_ERROR_INC(dev);
	}

	if ((error & kFLEXCAN_FDBit1Error) != 0U) {
		CAN_STATS_BIT1_ERROR_INC(dev);
	}

	if ((error & kFLEXCAN_FDStuffingError) != 0U) {
		CAN_STATS_STUFF_ERROR_INC(dev);
	}

	if ((error & kFLEXCAN_FDFormError) != 0U) {
		CAN_STATS_FORM_ERROR_INC(dev);
	}

	if ((error & kFLEXCAN_FDCrcError) != 0U) {
		CAN_STATS_CRC_ERROR_INC(dev);
	}
#endif /* CONFIG_CAN_FD_MODE */
}

void mcux_flexcan_init_common_config(flexcan_config_t *const flexcan_config,
				     const struct can_timing *const timing,
				     const uint32_t clock_freq, const int clock_source,
				     const uint8_t max_mb)
{
	FLEXCAN_GetDefaultConfig(flexcan_config);
	flexcan_config->maxMbNum = max_mb;
	flexcan_config->clkSrc = clock_source;
	flexcan_config->baudRate =
		clock_freq / (1U + timing->prop_seg + timing->phase_seg1 + timing->phase_seg2) /
		timing->prescaler;
	flexcan_config->enableIndividMask = true;
	flexcan_config->enableLoopBack = false;
	flexcan_config->disableSelfReception = true;
	flexcan_config->enableListenOnlyMode = true;

	flexcan_config->timingConfig.rJumpwidth = timing->sjw - 1U;
	flexcan_config->timingConfig.propSeg = timing->prop_seg - 1U;
	flexcan_config->timingConfig.phaseSeg1 = timing->phase_seg1 - 1U;
	flexcan_config->timingConfig.phaseSeg2 = timing->phase_seg2 - 1U;
}

void mcux_flexcan_from_can_frame(const struct can_frame *src,
					flexcan_frame_t *dest)
{
	memset(dest, 0, sizeof(*dest));

	if ((src->flags & CAN_FRAME_IDE) != 0U) {
		dest->format = kFLEXCAN_FrameFormatExtend;
		dest->id = FLEXCAN_ID_EXT(src->id);
	} else {
		dest->format = kFLEXCAN_FrameFormatStandard;
		dest->id = FLEXCAN_ID_STD(src->id);
	}

	if ((src->flags & CAN_FRAME_RTR) != 0U) {
		dest->type = kFLEXCAN_FrameTypeRemote;
	} else {
		dest->type = kFLEXCAN_FrameTypeData;
	}

	dest->length = src->dlc;
	dest->dataWord0 = sys_cpu_to_be32(src->data_32[0]);
	dest->dataWord1 = sys_cpu_to_be32(src->data_32[1]);
}

void mcux_flexcan_to_can_frame(const flexcan_frame_t *src,
				      struct can_frame *dest)
{
	memset(dest, 0, sizeof(*dest));

	if (src->format == kFLEXCAN_FrameFormatStandard) {
		dest->id = FLEXCAN_ID_TO_CAN_ID_STD(src->id);
	} else {
		dest->flags |= CAN_FRAME_IDE;
		dest->id = FLEXCAN_ID_TO_CAN_ID_EXT(src->id);
	}

	if (src->type == kFLEXCAN_FrameTypeRemote) {
		dest->flags |= CAN_FRAME_RTR;
	}

	dest->dlc = src->length;
	dest->data_32[0] = sys_be32_to_cpu(src->dataWord0);
	dest->data_32[1] = sys_be32_to_cpu(src->dataWord1);
#ifdef CONFIG_CAN_RX_TIMESTAMP
	dest->timestamp = src->timestamp;
#endif /* CONFIG_CAN_RX_TIMESTAMP */
}

#ifdef CONFIG_CAN_FD_MODE
void mcux_flexcan_fd_from_can_frame(const struct can_frame *src,
					   flexcan_fd_frame_t *dest)
{
	memset(dest, 0, sizeof(*dest));

	if ((src->flags & CAN_FRAME_IDE) != 0U) {
		dest->format = kFLEXCAN_FrameFormatExtend;
		dest->id = FLEXCAN_ID_EXT(src->id);
	} else {
		dest->format = kFLEXCAN_FrameFormatStandard;
		dest->id = FLEXCAN_ID_STD(src->id);
	}

	if ((src->flags & CAN_FRAME_RTR) != 0U) {
		dest->type = kFLEXCAN_FrameTypeRemote;
	} else {
		dest->type = kFLEXCAN_FrameTypeData;
	}

	if ((src->flags & CAN_FRAME_FDF) != 0U) {
		dest->edl = 1U;
	}

	if ((CAN_FRAME_BRS & src->flags) != 0) {
		dest->brs = 1U;
	}

	dest->length = src->dlc;

	for (uint8_t index = 0U; index <= CANFD_MAX_DLC; index++) {
		dest->dataWord[index] = sys_cpu_to_be32(src->data_32[index]);
	}
}

void mcux_flexcan_fd_to_can_frame(const flexcan_fd_frame_t *src,
					 struct can_frame *dest)
{
	memset(dest, 0, sizeof(*dest));

	if (src->format == kFLEXCAN_FrameFormatStandard) {
		dest->id = FLEXCAN_ID_TO_CAN_ID_STD(src->id);
	} else {
		dest->flags |= CAN_FRAME_IDE;
		dest->id = FLEXCAN_ID_TO_CAN_ID_EXT(src->id);
	}

	if (src->type == kFLEXCAN_FrameTypeRemote) {
		dest->flags |= CAN_FRAME_RTR;
	}

	dest->dlc = src->length;
	if (true == src->edl) {
		dest->flags |= CAN_FRAME_FDF;
	}
	if (true == src->brs) {
		dest->flags |= CAN_FRAME_BRS;
	}
	for (uint8_t index = 0U; index <= CANFD_MAX_DLC; index++) {
		dest->data_32[index] = sys_cpu_to_be32(src->dataWord[index]);
	}
#ifdef CONFIG_CAN_RX_TIMESTAMP
	dest->timestamp = src->timestamp;
#endif /* CONFIG_CAN_RX_TIMESTAMP */
}
#endif /* CONFIG_CAN_FD_MODE */
