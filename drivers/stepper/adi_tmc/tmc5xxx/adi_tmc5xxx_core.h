/**
 * @file drivers/stepper/adi_tmc/tmc5xxx/adi_tmc5xxx_core.h
 * @brief Common core for TMC5xxx stepper controllers
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Dipak Shetty
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_STEPPER_ADI_TMC_ADI_TMC5XXX_CORE_H_
#define ZEPHYR_DRIVERS_STEPPER_ADI_TMC_ADI_TMC5XXX_CORE_H_

#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/gpio.h>
#include <adi_tmc_bus.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Core context for stepper motor operations
 *
 * This structure contains all the necessary information to operate
 * a stepper motor with a TMC5xxx controller.
 */
struct tmc5xxx_core_context {
	const struct device *dev;              /* Stepper device */
	const struct device *controller_dev;   /* Parent controller device */
	uint8_t motor_index;                   /* Motor index (0 or 1) */
	const struct tmc5xxx_reg_map *reg_map; /* Register map for this motor */
	struct k_sem *controller_sem;          /* Reference to controller's semaphore */
};

/**
 * @brief Controller data structure
 */
struct tmc5xxx_controller_data {
	struct k_sem bus_sem;           /* Semaphore for bus synchronization */
	void *controller_specific_data; /* Controller-specific data */
};

/**
 * @brief Controller configuration structure
 */
struct tmc5xxx_controller_config {
	union tmc_bus bus;               /* Bus connection (SPI/UART) */
	const struct tmc_bus_io *bus_io; /* Bus I/O operations */
	enum tmc_comm_type comm_type;    /* Communication type */
	uint32_t gconf;                  /* Global configuration register value */
	uint32_t clock_frequency;        /* Clock frequency in Hz */
	struct gpio_dt_spec *gpio_specs; /* GPIO specifications */
};

/**
 * @brief Stepper data structure
 */
struct tmc5xxx_stepper_data {
	struct tmc5xxx_core_context core;                /* Core context for this stepper */
	struct k_work_delayable stallguard_dwork;        /* StallGuard work */
	struct k_work_delayable rampstat_callback_dwork; /* Rampstat work */
	struct gpio_callback diag0_cb;                   /* DIAG0 GPIO callback */
	stepper_event_callback_t callback;               /* Event callback function */
	void *callback_user_data;                        /* User data for callback */
};

/**
 * @brief Stepper configuration structure
 */
struct tmc5xxx_stepper_config {
	uint16_t default_micro_step_res;                    /* Default microstepping resolution */
	int8_t sg_threshold;                                /* StallGuard threshold */
	bool is_sg_enabled;                                 /* StallGuard enabled flag */
	uint32_t sg_velocity_check_interval_ms;             /* StallGuard velocity check interval */
	uint32_t sg_threshold_velocity;                     /* StallGuard threshold velocity */
	struct tmc_ramp_generator_data default_ramp_config; /* Default ramp configuration */
};

/**
 * @brief Initialize core context from device configuration
 *
 * @param ctx Core context to initialize
 * @param dev Device pointer for the stepper
 * @param controller_dev Parent controller device pointer
 * @param motor_index Index of the motor (0 or 1)
 * @param reg_map Register map for the specific TMC variant
 * @return 0 on success, negative error code otherwise
 */
int tmc5xxx_init_core_context(struct tmc5xxx_core_context *ctx, const struct device *dev,
			      const struct device *controller_dev, uint8_t motor_index,
			      const struct tmc5xxx_reg_map *reg_map);

/**
 * @brief Common register I/O functions
 */
int tmc5xxx_write_reg(const struct tmc5xxx_core_context *ctx, uint8_t reg, uint32_t value);
int tmc5xxx_read_reg(const struct tmc5xxx_core_context *ctx, uint8_t reg, uint32_t *value);

/**
 * @brief Common stepper motor control functions
 */
int tmc5xxx_enable(const struct device *dev);
int tmc5xxx_disable(const struct device *dev);
int tmc5xxx_is_moving(const struct device *dev, bool *is_moving);
int tmc5xxx_get_actual_position(const struct device *dev, int32_t *position);
int tmc5xxx_set_reference_position(const struct device *dev, int32_t position);
int tmc5xxx_set_max_velocity(const struct device *dev, uint32_t velocity);
int tmc5xxx_move_to(const struct device *dev, int32_t position);
int tmc5xxx_move_by(const struct device *dev, int32_t steps);
int tmc5xxx_run(const struct device *dev, enum stepper_direction direction);
int tmc5xxx_set_micro_step_res(const struct device *dev, enum stepper_micro_step_resolution res);
int tmc5xxx_get_micro_step_res(const struct device *dev, enum stepper_micro_step_resolution *res);

int tmc5xxx_stallguard_enable(const struct device *dev, bool enable);

/**
 * @brief StallGuard and event handling functions
 */
void tmc5xxx_stallguard_work_handler(struct k_work *work);
void tmc5xxx_rampstat_work_handler(struct k_work *work);
int tmc5xxx_rampstat_read_clear(const struct device *dev, uint32_t *rampstat);

/**
 * @brief Set up event callback handling for a stepper
 *
 * @param ctx Core context for the stepper
 * @param callback_data Callback data structure for the stepper
 * @param callback Event callback function
 * @param user_data User data for the callback
 * @return 0 on success, negative error code otherwise
 */
int tmc5xxx_set_event_callback(const struct device *dev, stepper_event_callback_t callback,
			       void *user_data);

/**
 * @brief Ramp generator functions
 */
#ifdef CONFIG_STEPPER_ADI_TMC_RAMP_GEN
int tmc5xxx_set_ramp(const struct tmc5xxx_core_context *ctx,
		     const struct tmc_ramp_generator_data *ramp_data);
#endif

/**
 * @brief Helper functions
 */
int tmc5xxx_read_vactual(const struct device *dev, int32_t *velocity);
void tmc5xxx_trigger_callback(const struct device *dev, enum stepper_event event);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_TMC_ADI_TMC5XXX_CORE_H_ */
