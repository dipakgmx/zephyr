/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Dipak Shetty
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_STEPPER_ADI_TMC51XX_H
#define ZEPHYR_DRIVERS_STEPPER_ADI_TMC51XX_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper/stepper_trinamic.h>

#include <adi_tmc_bus.h>

#define DT_DRV_COMPAT adi_tmc51xx

/* Check for supported bus types */
#define TMC51XX_BUS_SPI  DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#define TMC51XX_BUS_UART DT_ANY_INST_ON_BUS_STATUS_OKAY(uart)

#if TMC51XX_BUS_SPI
/* SPI bus I/O operations for TMC51xx devices */
extern const struct tmc_bus_io tmc51xx_spi_bus_io;
#endif

#if TMC51XX_BUS_UART
/* UART bus I/O operations for TMC51xx devices */
extern const struct tmc_bus_io tmc51xx_uart_bus_io;
#endif

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_TMC51XX_H */
