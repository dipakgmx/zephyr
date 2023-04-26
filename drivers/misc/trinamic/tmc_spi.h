/*
 * Copyright (c) 2023 Carl Zeiss Meditec AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_TMC_SPI_H
#define ZEPHYR_DRIVERS_TMC_SPI_H

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

int tmc_spi_read_register(const struct spi_dt_spec *bus, const uint8_t reg, uint32_t *data);
int tmc_spi_write_register(const struct spi_dt_spec *bus, const uint8_t reg, const uint32_t data);

#endif // ZEPHYR_DRIVERS_TMC_SPI_H
