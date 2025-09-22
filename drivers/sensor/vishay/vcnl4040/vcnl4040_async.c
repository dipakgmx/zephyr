/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/rtio/work.h>
#include <zephyr/logging/log.h>

#include "vcnl4040.h"

LOG_MODULE_DECLARE(vcnl4040, CONFIG_SENSOR_LOG_LEVEL);

void vcnl4040_submit_sync(struct rtio_iodev_sqe *iodev_sqe)
{
	uint32_t min_buf_len = sizeof(struct vcnl4040_encoded_data);
	int rc;
	uint8_t *buf;
	uint32_t buf_len;

	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	const struct device *dev = cfg->sensor;
	const struct sensor_chan_spec *const channels = cfg->channels;
	const size_t num_channels = cfg->count;

	rc = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, &buf, &buf_len);
	if (rc != 0) {
		LOG_ERR("Failed to get a read buffer of size %u bytes", min_buf_len);
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	struct vcnl4040_encoded_data *edata;

	edata = (struct vcnl4040_encoded_data *)buf;
	edata->header.timestamp = k_ticks_to_ns_floor64(k_uptime_ticks());
	edata->has_prox = 0;
	edata->has_light = 0;

	/* Check if the requested channels are supported */
	for (size_t i = 0; i < num_channels; i++) {
		switch (channels[i].chan_type) {
		case SENSOR_CHAN_PROX:
			edata->has_prox = 1;
			break;
		case SENSOR_CHAN_LIGHT:
#ifdef CONFIG_VCNL4040_ENABLE_ALS
			edata->has_light = 1;
#endif
			break;
		case SENSOR_CHAN_ALL:
			edata->has_prox = 1;
#ifdef CONFIG_VCNL4040_ENABLE_ALS
			edata->has_light = 1;
#endif
			break;
		default:
			continue;
		}
	}

	/* Determine which channel to fetch */
	enum sensor_channel fetch_chan;
	if (edata->has_prox && edata->has_light) {
		fetch_chan = SENSOR_CHAN_ALL;
	} else if (edata->has_light) {
		fetch_chan = SENSOR_CHAN_LIGHT;
	} else {
		fetch_chan = SENSOR_CHAN_PROX;
	}

	rc = vcnl4040_sample_fetch(dev, fetch_chan);
	if (rc != 0) {
		LOG_ERR("Failed to fetch samples");
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	/* Copy sensor data to encoded structure */
	struct vcnl4040_data const *data = dev->data;
	if (edata->has_prox) {
		edata->reading.proximity = data->proximity;
	}
#ifdef CONFIG_VCNL4040_ENABLE_ALS
	if (edata->has_light) {
		edata->reading.light = data->light;
	}
#endif

	rtio_iodev_sqe_ok(iodev_sqe, 0);
}

void vcnl4040_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct rtio_work_req *req = rtio_work_req_alloc();

	__ASSERT_NO_MSG(req);

	rtio_work_req_submit(req, iodev_sqe, vcnl4040_submit_sync);
}
