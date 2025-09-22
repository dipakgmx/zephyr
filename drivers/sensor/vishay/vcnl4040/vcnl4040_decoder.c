/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include "vcnl4040.h"

static int vcnl4040_decoder_get_frame_count(const uint8_t *buffer,
					     struct sensor_chan_spec chan_spec,
					     uint16_t *frame_count)
{
	const struct vcnl4040_encoded_data *edata = (const struct vcnl4040_encoded_data *)buffer;
	int32_t ret = -ENOTSUP;

	if (chan_spec.chan_idx != 0) {
		return ret;
	}

	/* This sensor lacks a FIFO; there will always only be one frame at a time. */
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_PROX:
		*frame_count = edata->has_prox ? 1 : 0;
		break;
#ifdef CONFIG_VCNL4040_ENABLE_ALS
	case SENSOR_CHAN_LIGHT:
		*frame_count = edata->has_light ? 1 : 0;
		break;
#endif
	default:
		return ret;
	}

	if (*frame_count > 0) {
		ret = 0;
	}

	return ret;
}

static int vcnl4040_decoder_get_size_info(struct sensor_chan_spec chan_spec, size_t *base_size,
					   size_t *frame_size)
{
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_PROX:
		*base_size = sizeof(struct sensor_q31_sample_data);
		*frame_size = sizeof(struct sensor_q31_sample_data);
		return 0;
#ifdef CONFIG_VCNL4040_ENABLE_ALS
	case SENSOR_CHAN_LIGHT:
		*base_size = sizeof(struct sensor_q31_sample_data);
		*frame_size = sizeof(struct sensor_q31_sample_data);
		return 0;
#endif
	default:
		return -ENOTSUP;
	}
}

#define VCNL4040_PROX_SHIFT (16)
#define VCNL4040_LIGHT_SHIFT (16)

static void vcnl4040_convert_uint16_to_q31(uint16_t reading, int32_t shift, q31_t *out)
{
	/* Convert raw uint16 reading to Q31 format */
	uint64_t scaled = ((uint64_t)reading) << (31 - shift);
	*out = (q31_t)MIN(scaled, INT32_MAX);
}

static int vcnl4040_decoder_decode(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				   uint32_t *fit, uint16_t max_count, void *data_out)
{
	const struct vcnl4040_encoded_data *edata = (const struct vcnl4040_encoded_data *)buffer;

	if (*fit != 0) {
		return 0;
	}

	struct sensor_q31_data *out = data_out;

	out->header.base_timestamp_ns = edata->header.timestamp;
	out->header.reading_count = 1;

	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_PROX:
		if (edata->has_prox) {
			vcnl4040_convert_uint16_to_q31(edata->reading.proximity,
				VCNL4040_PROX_SHIFT, &out->readings[0].distance);
			out->shift = VCNL4040_PROX_SHIFT;
		} else {
			return -ENODATA;
		}
		break;
#ifdef CONFIG_VCNL4040_ENABLE_ALS
	case SENSOR_CHAN_LIGHT:
		if (edata->has_light) {
			vcnl4040_convert_uint16_to_q31(edata->reading.light,
				VCNL4040_LIGHT_SHIFT, &out->readings[0].light);
			out->shift = VCNL4040_LIGHT_SHIFT;
		} else {
			return -ENODATA;
		}
		break;
#endif
	default:
		return -EINVAL;
	}

	*fit = 1;

	return 1;
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = vcnl4040_decoder_get_frame_count,
	.get_size_info = vcnl4040_decoder_get_size_info,
	.decode = vcnl4040_decoder_decode,
};

int vcnl4040_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &SENSOR_DECODER_NAME();

	return 0;
}