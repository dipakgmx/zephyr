/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2022 Carl Zeiss Meditec AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef MCUX_FLEXCAN_TYPE__
#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif
#include "can_mcux_flexcan_template.h"
LOG_MODULE_REGISTER(can_mcux_flexcan_template, CONFIG_CAN_LOG_LEVEL);

/*
 * Helper macro to generate the mcux_flexcan_<>_data type based on the MCUX_FLEXCAN_TYPE__ flag.
 * This macro would either generate type mcux_flexcan_classic_data or mcux_flexcan_fd_data.
 */
#define FLEXCAN_DATA_TYPE() CONCAT(mcux_flexcan_, CONCAT(MCUX_FLEXCAN_TYPE__, _data))

/*
 * Function template would yield either mcux_flexcan_classic_set_timing or
 * mcux_flexcan_fd_set_timing.
 */
int GENERATE_FUNCTION_NAME(set_timing)(const struct device *dev,
							   const struct can_timing *timing)
{
	struct FLEXCAN_DATA_TYPE() *data = dev->data;
	uint8_t sjw_backup = data->timing.sjw;
#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	uint8_t sjw_data_backup = data->timing_data.sjw;
#endif /* CONFIG_CAN_FD_MODE */

	if (!timing) {
		return -EINVAL;
	}

	if (data->started) {
		return -EBUSY;
	}

	data->timing = *timing;
	if (timing->sjw == CAN_SJW_NO_CHANGE) {
		data->timing.sjw = sjw_backup;
	}

#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	data->timing_data = *timing;
	if (timing->sjw == CAN_SJW_NO_CHANGE) {
		data->timing_data.sjw = sjw_data_backup;
	}
#endif /* CONFIG_CAN_FD_MODE */

	return 0;
}

/*
 * Function template would yield either mcux_flexcan_classic_start or mcux_flexcan_fd_start.
 */
int GENERATE_FUNCTION_NAME(start)(const struct device *dev)
{
	const struct mcux_flexcan_generic_config *config = dev->config;

	struct FLEXCAN_DATA_TYPE() *data = dev->data;
	flexcan_timing_config_t timing;
	int err;

	if (data->started) {
		return -EALREADY;
	}

	if (config->phy != NULL) {
		err = can_transceiver_enable(config->phy);
		if (err != 0) {
			LOG_ERR("failed to enable CAN transceiver (err %d)", err);
			return err;
		}
	}

	/* Clear error counters */
	config->base->ECR &= ~(CAN_ECR_TXERRCNT_MASK | CAN_ECR_RXERRCNT_MASK);

	/* Delay this until start since setting the timing automatically exits freeze mode */
	timing.preDivider = data->timing.prescaler - 1U;
	timing.rJumpwidth = data->timing.sjw - 1U;
	timing.phaseSeg1 = data->timing.phase_seg1 - 1U;
	timing.phaseSeg2 = data->timing.phase_seg2 - 1U;
	timing.propSeg = data->timing.prop_seg - 1U;

#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	timing.fpreDivider = data->timing_data.prescaler - 1U;
	timing.frJumpwidth = data->timing_data.sjw - 1U;
	timing.fphaseSeg1 = data->timing_data.phase_seg1 - 1U;
	timing.fphaseSeg2 = data->timing_data.phase_seg2 - 1U;
	timing.fpropSeg = data->timing_data.prop_seg - 1U;
	FLEXCAN_SetFDTimingConfig(config->base, &timing);
#else
	FLEXCAN_SetTimingConfig(config->base, &timing);
#endif
	data->started = true;

	return 0;
}

/*
 * Function template would yield either mcux_flexcan_classic_stop or mcux_flexcan_fd_stop.
 */
int GENERATE_FUNCTION_NAME(stop)(const struct device *dev)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct FLEXCAN_DATA_TYPE() *data = dev->data;
	can_tx_callback_t function;
	void *arg;
	int alloc;
	int err;
#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	const int flexcan_max_tx = MCUX_FLEXCAN_FD_MAX_TX;
#else
	const int flexcan_max_tx = MCUX_FLEXCAN_MAX_TX;
#endif /* CONFIG_CAN_FD_MODE */

	if (!data->started) {
		return -EALREADY;
	}

	data->started = false;

	/* Abort any pending TX frames before entering freeze mode */
	for (alloc = 0; alloc < flexcan_max_tx; alloc++) {
		function = data->tx_cbs[alloc].function;
		arg = data->tx_cbs[alloc].arg;

		if (atomic_test_and_clear_bit(data->tx_allocs, alloc)) {
#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
			FLEXCAN_TransferFDAbortSend(config->base, &data->handle,
						    ALLOC_IDX_TO_TXMB_IDX(alloc));
#else
			FLEXCAN_TransferAbortSend(config->base, &data->handle,
						  ALLOC_IDX_TO_TXMB_IDX(alloc));
#endif
			function(dev, -ENETDOWN, arg);
			k_sem_give(&data->tx_allocs_sem);
		}
	}

	FLEXCAN_EnterFreezeMode(config->base);

	if (config->phy != NULL) {
		err = can_transceiver_disable(config->phy);
		if (err != 0) {
			LOG_ERR("failed to disable CAN transceiver (err %d)", err);
			return err;
		}
	}

	return 0;
}

/*
 * Function template would yield either mcux_flexcan_classic_set_mode or mcux_flexcan_fd_set_mode.
 */
int GENERATE_FUNCTION_NAME(set_mode)(const struct device *dev, can_mode_t mode)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct FLEXCAN_DATA_TYPE() *data = dev->data;

	if (data->started) {
		return -EBUSY;
	}

	if ((mode &
#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	     ~(CAN_MODE_FD | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_3_SAMPLES)) != 0) {
#else
	     ~(CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_3_SAMPLES)) != 0) {
#endif
		LOG_ERR("unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}

	mcux_flexcan_config_ctrl1(mode, config->base);
	mcux_flexcan_config_mcr(mode, config->base);

#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)

	uint32_t fdctrl = config->base->FDCTRL;

	if ((mode & CAN_MODE_LOOPBACK) != 0) {
		/* TDC must be disabled when Loop Back mode is enabled */
		fdctrl &= ~(CAN_FDCTRL_TDCEN_MASK);
	} else {
		/* TDC is enabled */
		fdctrl |= CAN_FDCTRL_TDCEN_MASK;
	}
	config->base->FDCTRL = fdctrl;
#endif /* CONFIG_CAN_FD_MODE */

	return 0;
}

/*
 * Function template would yield either mcux_flexcan_classic_get_state or mcux_flexcan_fd_get_state.
 */
int GENERATE_FUNCTION_NAME(get_state)(const struct device *dev,
							  enum can_state *state,
							  struct can_bus_err_cnt *err_cnt)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct FLEXCAN_DATA_TYPE() *data = dev->data;

	if (state != NULL) {
		if (!data->started) {
			*state = CAN_STATE_STOPPED;
		} else {
			*state = mcux_flexcan_read_status_flags(config);
		}
	}

	if (err_cnt != NULL) {
		FLEXCAN_GetBusErrCount(config->base, &err_cnt->tx_err_cnt, &err_cnt->rx_err_cnt);
	}

	return 0;
}

/*
 * Function template would yield either mcux_flexcan_classic_send or mcux_flexcan_fd_send.
 */
int GENERATE_FUNCTION_NAME(send)(const struct device *dev, const struct can_frame *frame,
				 k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct FLEXCAN_DATA_TYPE() *data = dev->data;
	flexcan_mb_transfer_t xfer;
	enum can_state state;
	status_t status;
	int alloc;

	__ASSERT_NO_MSG(callback != NULL);

#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	const int flexcan_max_tx = MCUX_FLEXCAN_FD_MAX_TX;

	if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF | CAN_FRAME_BRS)) !=
	    0U) {
		LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
		return -ENOTSUP;
	}

	if (frame->dlc > CANFD_MAX_DLC) {
		LOG_ERR("CAN-FD frame with incorrect dlc %d", frame->dlc);
		return -EINVAL;
	}

#else
	const int flexcan_max_tx = MCUX_FLEXCAN_MAX_TX;

	if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0) {
		LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
		return -ENOTSUP;
	}

	if (frame->dlc > CAN_MAX_DLC) {
		LOG_ERR("CAN frame with incorrect dlc %d", frame->dlc);
		return -EINVAL;
	}

#endif /* CONFIG_CAN_FD_MODE */

	if (((frame->flags & CAN_FRAME_IDE) != 0U) && (frame->id <= 0x7FF)) {
		LOG_ERR("Standard frame id used with frame tagged Extended Frame id: 0x%x",
			frame->id);
		return -EINVAL;
	}

	if (((frame->flags & CAN_FRAME_IDE) == 0U) && (frame->id >= 0x7FF)) {
		LOG_ERR("Extended Frame id used with frame tagged Standard Frame id: 0x%x",
			frame->id);
		return -EINVAL;
	}

	if (!data->started) {
		return -ENETDOWN;
	}
	(void)CONCAT(mcux_flexcan_, CONCAT(MCUX_FLEXCAN_TYPE__, _get_state))(dev, &state, NULL);
	if (state == CAN_STATE_BUS_OFF) {
		LOG_DBG("Transmit failed, bus-off");
		return -ENETUNREACH;
	}

	if (k_sem_take(&data->tx_allocs_sem, timeout) != 0) {
		return -EAGAIN;
	}

	for (alloc = 0; alloc < flexcan_max_tx; alloc++) {
		if (!atomic_test_and_set_bit(data->tx_allocs, alloc)) {
			break;
		}
	}

#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	mcux_flexcan_fd_from_can_frame(frame, &data->tx_cbs[alloc].frame);
#else
	mcux_flexcan_from_can_frame(frame, &data->tx_cbs[alloc].frame);
#endif /* CONFIG_CAN_FD_MODE */
	data->tx_cbs[alloc].function = callback;
	data->tx_cbs[alloc].arg = user_data;
#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	xfer.framefd = &data->tx_cbs[alloc].frame;
#else
	xfer.frame = &data->tx_cbs[alloc].frame;
#endif /* CONFIG_CAN_FD_MODE */
	xfer.mbIdx = ALLOC_IDX_TO_TXMB_IDX(alloc);

#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	FLEXCAN_SetFDTxMbConfig(config->base, xfer.mbIdx, true);
#else
	FLEXCAN_SetTxMbConfig(config->base, xfer.mbIdx, true);
#endif /* CONFIG_CAN_FD_MODE */
	k_mutex_lock(&data->tx_mutex, K_FOREVER);
	config->irq_disable_func();
#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	status = FLEXCAN_TransferFDSendNonBlocking(config->base, &data->handle, &xfer);
#else
	status = FLEXCAN_TransferSendNonBlocking(config->base, &data->handle, &xfer);
#endif /* CONFIG_CAN_FD_MODE */

	config->irq_enable_func();
	k_mutex_unlock(&data->tx_mutex);

	if (status != kStatus_Success) {
		return -EIO;
	}

	return 0;
}

/*
 * Function template would yield either mcux_flexcan_classic_add_rx_filter or
 * mcux_flexcan_fd_add_rx_filter.
 */
int GENERATE_FUNCTION_NAME(add_rx_filter)(const struct device *dev, can_rx_callback_t callback,
					  void *user_data, const struct can_filter *filter)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct FLEXCAN_DATA_TYPE() *data = dev->data;
	flexcan_mb_transfer_t xfer;
	status_t status;
	uint32_t mask;
	int alloc = -ENOSPC;
	int i;

	__ASSERT_NO_MSG(callback);

#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	if ((filter->flags & ~(CAN_FILTER_IDE | CAN_FILTER_DATA |
			       CAN_FILTER_RTR | CAN_FILTER_FDF)) != 0) {
#else
	if ((filter->flags & ~(CAN_FILTER_IDE | CAN_FILTER_DATA | CAN_FILTER_RTR)) != 0) {
#endif
		LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
		return -ENOTSUP;
	}

	k_mutex_lock(&data->rx_mutex, K_FOREVER);

	/* Find and allocate RX message buffer */
	for (i = RX_START_IDX; i < MCUX_FLEXCAN_MAX_RX; i++) {
		if (!atomic_test_and_set_bit(data->rx_allocs, i)) {
			alloc = i;
			break;
		}
	}

	if (alloc == -ENOSPC) {
		return alloc;
	}

	mcux_flexcan_can_filter_to_mbconfig(filter, &data->rx_cbs[alloc].mb_config, &mask);

	data->rx_cbs[alloc].arg = user_data;
	data->rx_cbs[alloc].function = callback;

	/* The individual RX mask registers can only be written in freeze mode */
	FLEXCAN_EnterFreezeMode(config->base);
	config->base->RXIMR[ALLOC_IDX_TO_RXMB_IDX(alloc)] = mask;
	if (data->started) {
		FLEXCAN_ExitFreezeMode(config->base);
	}

#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	FLEXCAN_SetFDRxMbConfig(config->base, ALLOC_IDX_TO_RXMB_IDX(alloc),
				&data->rx_cbs[alloc].mb_config, true);

	xfer.framefd = &data->rx_cbs[alloc].frame;
	xfer.mbIdx = ALLOC_IDX_TO_RXMB_IDX(alloc);
	status = FLEXCAN_TransferFDReceiveNonBlocking(config->base, &data->handle, &xfer);
#else
	FLEXCAN_SetRxMbConfig(config->base, ALLOC_IDX_TO_RXMB_IDX(alloc),
			      &data->rx_cbs[alloc].mb_config, true);

	xfer.frame = &data->rx_cbs[alloc].frame;
	xfer.mbIdx = ALLOC_IDX_TO_RXMB_IDX(alloc);
	status = FLEXCAN_TransferReceiveNonBlocking(config->base, &data->handle, &xfer);
#endif /* CONFIG_CAN_FD_MODE */

	if (status != kStatus_Success) {
		LOG_ERR("Failed to start rx for filter id %d (err = %d)", alloc, status);
		alloc = -ENOSPC;
	}

	k_mutex_unlock(&data->rx_mutex);

	return alloc;
}

/*
 * Function template would yield either mcux_flexcan_classic_set_state_change_callback or
 * mcux_flexcan_fd_set_state_change_callback.
 */
void GENERATE_FUNCTION_NAME(set_state_change_callback)(const struct device *dev,
							    can_state_change_callback_t callback,
							    void *user_data)
{
	struct FLEXCAN_DATA_TYPE() *data = dev->data;

	data->state_change_cb = callback;
	data->state_change_cb_data = user_data;
}

/*
 * Function template would yield either mcux_flexcan_classic_remove_rx_filter or
 * mcux_flexcan_fd_remove_rx_filter.
 */
void GENERATE_FUNCTION_NAME(remove_rx_filter)(const struct device *dev, int filter_id)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct FLEXCAN_DATA_TYPE() *data = dev->data;

	if (filter_id >= MCUX_FLEXCAN_MAX_RX) {
		LOG_ERR("Detach: Filter id >= MAX_RX (%d >= %d)", filter_id, MCUX_FLEXCAN_MAX_RX);
		return;
	}

	k_mutex_lock(&data->rx_mutex, K_FOREVER);

	if (atomic_test_and_clear_bit(data->rx_allocs, filter_id)) {
#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
		FLEXCAN_TransferFDAbortReceive(config->base, &data->handle,
					       ALLOC_IDX_TO_RXMB_IDX(filter_id));
		FLEXCAN_SetFDRxMbConfig(config->base, ALLOC_IDX_TO_RXMB_IDX(filter_id), NULL,
					false);
#else
		FLEXCAN_TransferAbortReceive(config->base, &data->handle,
					     ALLOC_IDX_TO_RXMB_IDX(filter_id));
		FLEXCAN_SetRxMbConfig(config->base, ALLOC_IDX_TO_RXMB_IDX(filter_id), NULL, false);
#endif /* CONFIG_CAN_FD_MODE */
		data->rx_cbs[filter_id].function = NULL;
		data->rx_cbs[filter_id].arg = NULL;
	} else {
		LOG_WRN("Filter ID %d already detached", filter_id);
	}

	k_mutex_unlock(&data->rx_mutex);
}

/*
 * Function template would yield either mcux_flexcan_classic_transfer_error_status or
 * mcux_flexcan_fd_transfer_error_status.
 */
void GENERATE_FUNCTION_NAME(transfer_error_status)(const struct device *dev, uint64_t error)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct FLEXCAN_DATA_TYPE() *data = dev->data;
	const can_state_change_callback_t cb = data->state_change_cb;
	void *cb_data = data->state_change_cb_data;
	can_tx_callback_t function;
	void *arg;
	int alloc;
	enum can_state state;
	struct can_bus_err_cnt err_cnt;
#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	const int flexcan_max_tx = MCUX_FLEXCAN_FD_MAX_TX;
#else
	const int flexcan_max_tx = MCUX_FLEXCAN_MAX_TX;
#endif /* CONFIG_CAN_FD_MODE */

	increment_error_counters(dev, error);

	(void)CONCAT(mcux_flexcan_, CONCAT(MCUX_FLEXCAN_TYPE__, _get_state))(dev, &state, &err_cnt);
	if (data->state != state) {
		data->state = state;

		if (cb != NULL) {
			cb(dev, state, err_cnt, cb_data);
		}
	}

	if (state == CAN_STATE_BUS_OFF) {
		/* Abort any pending TX frames in case of bus-off */
		for (alloc = 0; alloc < flexcan_max_tx; alloc++) {
			/* Copy callback function and argument before clearing bit */
			function = data->tx_cbs[alloc].function;
			arg = data->tx_cbs[alloc].arg;

			if (atomic_test_and_clear_bit(data->tx_allocs, alloc)) {
#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
				FLEXCAN_TransferFDAbortSend(config->base, &data->handle,
							    ALLOC_IDX_TO_TXMB_IDX(alloc));
#else
				FLEXCAN_TransferAbortSend(config->base, &data->handle,
							  ALLOC_IDX_TO_TXMB_IDX(alloc));
#endif
				function(dev, -ENETUNREACH, arg);
				k_sem_give(&data->tx_allocs_sem);
			}
		}
	}
}

/*
 * Function template would yield either mcux_flexcan_classic_transfer_tx_idle or
 * mcux_flexcan_fd_transfer_tx_idle.
 */
void GENERATE_FUNCTION_NAME(transfer_tx_idle)(const struct device *dev, uint32_t mb)

{
	struct FLEXCAN_DATA_TYPE() *data = dev->data;
	can_tx_callback_t function;
	void *arg;
	int alloc;

	alloc = TX_MBIDX_TO_ALLOC_IDX(mb);

	/* Copy callback function and argument before clearing bit */
	function = data->tx_cbs[alloc].function;
	arg = data->tx_cbs[alloc].arg;

	if (atomic_test_and_clear_bit(data->tx_allocs, alloc)) {
		function(dev, 0, arg);
		k_sem_give(&data->tx_allocs_sem);
	}
}

/*
 * Function template would yield either mcux_flexcan_classic_transfer_rx_idle or
 * mcux_flexcan_fd_transfer_rx_idle.
 */
void GENERATE_FUNCTION_NAME(transfer_rx_idle)(const struct device *dev, uint32_t mb)

{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct FLEXCAN_DATA_TYPE() *data = dev->data;
	can_rx_callback_t function;
	flexcan_mb_transfer_t xfer;
	struct can_frame frame;
	status_t status;
	void *arg;
	int alloc;

	alloc = RX_MBIDX_TO_ALLOC_IDX(mb);
	function = data->rx_cbs[alloc].function;
	arg = data->rx_cbs[alloc].arg;

	if (atomic_test_bit(data->rx_allocs, alloc)) {
#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
		mcux_flexcan_fd_to_can_frame(&data->rx_cbs[alloc].frame, &frame);
#else
		mcux_flexcan_to_can_frame(&data->rx_cbs[alloc].frame, &frame);
#endif /* CONFIG_CAN_FD_MODE */

		function(dev, &frame, arg);

		/* Setup RX message buffer to receive next message */
#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
		FLEXCAN_SetFDRxMbConfig(config->base, mb, &data->rx_cbs[alloc].mb_config, true);
		xfer.framefd = &data->rx_cbs[alloc].frame;
		xfer.mbIdx = mb;
		status = FLEXCAN_TransferFDReceiveNonBlocking(config->base, &data->handle, &xfer);
#else
		FLEXCAN_SetRxMbConfig(config->base, mb, &data->rx_cbs[alloc].mb_config, true);
		xfer.frame = &data->rx_cbs[alloc].frame;
		xfer.mbIdx = mb;
		status = FLEXCAN_TransferReceiveNonBlocking(config->base, &data->handle, &xfer);
#endif
		if (status != kStatus_Success) {
			LOG_ERR("Failed to restart rx for filter id %d "
				"(err = %d)",
				alloc, status);
		}
	}
}

static FLEXCAN_CALLBACK(mcux_flexcan_transfer_callback)
{
	struct FLEXCAN_DATA_TYPE() *data =
		(struct FLEXCAN_DATA_TYPE() *)(userData);
	const struct mcux_flexcan_generic_config *config = data->dev->config;
	/*
	 * The result field can either be a MB index (which is limited to 32 bit
	 * value) or a status flags value, which is 32 bit on some platforms but
	 * 64 on others. To decouple the remaining functions from this, the
	 * result field is always promoted to uint64_t.
	 */
	uint32_t mb = (uint32_t)result;
	uint64_t status_flags = result;

	ARG_UNUSED(base);

	switch (status) {
	case kStatus_FLEXCAN_UnHandled:
		/* Not all fault confinement state changes are handled by the HAL */
		__fallthrough;
	case kStatus_FLEXCAN_ErrorStatus:
		GENERATE_FUNCTION_NAME(transfer_error_status)(data->dev, status_flags);
		break;
	case kStatus_FLEXCAN_TxSwitchToRx:
#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
		FLEXCAN_TransferFDAbortReceive(config->base, &data->handle, mb);
#else
		FLEXCAN_TransferAbortReceive(config->base, &data->handle, mb);
#endif /* CONFIG_CAN_FD_MODE */
		__fallthrough;
	case kStatus_FLEXCAN_TxIdle:
		GENERATE_FUNCTION_NAME(transfer_tx_idle)(data->dev, mb);
		break;
	case kStatus_FLEXCAN_RxOverflow:
		__fallthrough;
	case kStatus_FLEXCAN_RxIdle:
		GENERATE_FUNCTION_NAME(transfer_rx_idle)(data->dev, mb);
		break;
	default:
		LOG_WRN("Unhandled status 0x%08x (result = 0x%016llx)", status, status_flags);
	}
}

/*
 * Function template would yield either mcux_flexcan_classic_isr or mcux_flexcan_fd_isr.
 */
void GENERATE_FUNCTION_NAME(isr)(const struct device *dev)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct FLEXCAN_DATA_TYPE() *data = dev->data;

	FLEXCAN_TransferHandleIRQ(config->base, &data->handle);
}

int GENERATE_FUNCTION_NAME(init)(const struct device *dev)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct FLEXCAN_DATA_TYPE() *data = dev->data;
	flexcan_config_t flexcan_config;
	uint32_t clock_freq;
	int err;
	can_mode_t mode = CAN_MODE_NORMAL;
	uint8_t max_mb_size = FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(0);

	/*
	 * Set the mode if the flexcan_fd mode is set to CAN FD and also the message buffer size
	 * accordingly. Else, the previously set values would be used.
	 */
#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	mode = CAN_MODE_FD;
	max_mb_size = FLEXCAN_FD_MESSAGE_BUFFER_MAX_NUMBER;
#endif /* CONFIG_CAN_FD_MODE */

	if (config->phy != NULL) {
		if (!device_is_ready(config->phy)) {
			LOG_ERR("CAN transceiver not ready");
			return -ENODEV;
		}
	}

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock device not ready");
		return -ENODEV;
	}

	k_mutex_init(&data->rx_mutex);
	k_mutex_init(&data->tx_mutex);

	k_sem_init(&data->tx_allocs_sem, MCUX_FLEXCAN_MAX_TX,
		   MCUX_FLEXCAN_MAX_TX);

	data->timing.sjw = config->sjw;
	if (config->sample_point && USE_SP_ALGO) {
		err = can_calc_timing(dev, &data->timing, config->bitrate,
				      config->sample_point);
		if (err == -EINVAL) {
			LOG_ERR("Can't find timing for given param");
			return -EIO;
		}
		LOG_DBG("Presc: %d, Seg1S1: %d, Seg2: %d",
			data->timing.prescaler, data->timing.phase_seg1,
			data->timing.phase_seg2);
		LOG_DBG("Sample-point err : %d", err);

#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
		data->timing_data.sjw = config->sjw_data;
		err = can_calc_timing_data(dev, &data->timing_data, config->bus_speed_data,
					   config->sample_point_data);
		if (err == -EINVAL) {
			LOG_ERR("Can't find timing for given data phase param");
			return -EIO;
		}
		LOG_DBG("Presc data phase: %d, Seg1S1 data phase: %d, Seg2 data phase: %d",
			data->timing_data.prescaler, data->timing_data.phase_seg1,
			data->timing_data.phase_seg2);
		LOG_DBG("Sample-point err data phase: %d", err);
#endif /* CONFIG_CAN_FD_MODE */
	} else {
		err = mcux_flexcan_config_can_calc_prescaler(dev, config, &data->timing);
		if (err) {
			LOG_WRN("Bitrate error: %d", err);
		}

#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
		data->timing_data.prop_seg = config->prop_seg;
		data->timing_data.phase_seg1 = config->phase_seg1;
		data->timing_data.phase_seg2 = config->phase_seg2;
		err = can_calc_prescaler(dev, &data->timing_data, config->bus_speed_data);
		if (err) {
			LOG_WRN("Data phase bitrate error: %d", err);
		}
#endif /* CONFIG_CAN_FD_MODE */
	}

#ifdef CONFIG_PINCTRL
	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}
#endif

	err = mcux_flexcan_get_core_clock(dev, &clock_freq);
	if (err != 0) {
		return -EIO;
	}

	data->dev = dev;

	mcux_flexcan_init_common_config(&flexcan_config, &data->timing, clock_freq,
					config->clk_source, max_mb_size);

#if defined(CONFIG_CAN_FD_MODE) && (fd == MCUX_FLEXCAN_TYPE__)
	flexcan_config.baudRateFD = clock_freq /
				    (1U + data->timing_data.prop_seg +
				     data->timing_data.phase_seg1 + data->timing_data.phase_seg2) /
				    data->timing_data.prescaler;

	flexcan_config.timingConfig.frJumpwidth = data->timing_data.sjw - 1U;
	flexcan_config.timingConfig.fpropSeg = data->timing_data.prop_seg - 1U;
	flexcan_config.timingConfig.fphaseSeg1 = data->timing_data.phase_seg1 - 1U;
	flexcan_config.timingConfig.fphaseSeg2 = data->timing_data.phase_seg2 - 1U;

	/* Initialize in listen-only mode since FLEXCAN_FDInit() exits freeze mode */
	FLEXCAN_FDInit(config->base, &flexcan_config, clock_freq,
		       kFLEXCAN_64BperMB, true);
#else
	/* Initialize in listen-only mode since FLEXCAN_Init() exits freeze mode */
	FLEXCAN_Init(config->base, &flexcan_config, clock_freq);
#endif /* CONFIG_CAN_FD_MODE */
	FLEXCAN_TransferCreateHandle(config->base, &data->handle, mcux_flexcan_transfer_callback,
				     data);

	/* Manually enter freeze mode, set normal mode, and clear error counters */
	FLEXCAN_EnterFreezeMode(config->base);
	(void)GENERATE_FUNCTION_NAME(set_mode)(dev, mode);
	config->base->ECR &= ~(CAN_ECR_TXERRCNT_MASK | CAN_ECR_RXERRCNT_MASK);

	config->irq_config_func(dev);

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	config->base->CTRL1 |= CAN_CTRL1_BOFFREC_MASK;
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */

	(void)GENERATE_FUNCTION_NAME(get_state)(dev, &data->state, NULL);

	return 0;
}

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY

/*
 * Function template would yield either mcux_flexcan_classic_recover or mcux_flexcan_fd_recover.
 */
int GENERATE_FUNCTION_NAME(recover)(const struct device *dev, k_timeout_t timeout)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct FLEXCAN_DATA_TYPE() *data = dev->data;
	enum can_state state;
	uint64_t start_time;
	int ret = 0;

	if (!data->started) {
		return -ENETDOWN;
	}

	(void)GENERATE_FUNCTION_NAME(get_state)(dev, &state, NULL);
	if (state != CAN_STATE_BUS_OFF) {
		return 0;
	}

	start_time = k_uptime_ticks();
	config->base->CTRL1 &= ~CAN_CTRL1_BOFFREC_MASK;

	if (!K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
		(void)GENERATE_FUNCTION_NAME(get_state)(dev, &state, NULL);

		while (state == CAN_STATE_BUS_OFF) {
			if (!K_TIMEOUT_EQ(timeout, K_FOREVER) &&
			    k_uptime_ticks() - start_time >= timeout.ticks) {
				ret = -EAGAIN;
			}

			(void)GENERATE_FUNCTION_NAME(get_state)(dev, &state, NULL);
		}
	}

	config->base->CTRL1 |= CAN_CTRL1_BOFFREC_MASK;

	return ret;
}
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */

#endif /* MCUX_FLEXCAN_TYPE__ */
