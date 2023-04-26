/*
* Copyright (c) 2023 Carl Zeiss Meditec AG
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <zephyr/device.h>
#include <zephyr/drivers/misc/trinamic/trinamic.h>

static int cmd_tmc_read(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	uint8_t reg_addr;
	uint32_t reg_val;
	char *endptr;
	int err;

	dev = device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(sh, "Device unknown (%s)", argv[1]);
		return -ENODEV;
	}

	reg_addr = (uint8_t)strtoul(argv[2], &endptr, 10);
	if (*endptr != '\0') {
		shell_error(sh, "failed to parse register address");
		return -EINVAL;
	}

	err = trinamic_read(dev, reg_addr, &reg_val);
	if (err < 0) {
		shell_error(sh, "failed to read register (err %d)", err);
		return err;
	}
	shell_print(sh, "reg <%d> value:      %d", reg_addr, reg_val);

	return 0;
}

static int cmd_tmc_write(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	uint8_t reg_addr;
	uint32_t reg_val;
	char *endptr;
	int err;

	dev = device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(sh, "Device unknown (%s)", argv[1]);
		return -ENODEV;
	}

	reg_addr = (uint8_t)strtoul(argv[2], &endptr, 10);
	if (*endptr != '\0') {
		shell_error(sh, "failed to parse register address");
		return -EINVAL;
	}

	reg_val = (uint8_t)strtoul(argv[2], &endptr, 10);
	if (*endptr != '\0') {
		shell_error(sh, "failed to parse write value");
		return -EINVAL;
	}

	err = trinamic_write(dev, reg_addr, reg_val);
	if (err < 0) {
		shell_error(sh, "failed to read register (err %d)", err);
		return err;
	}
	shell_print(sh, "write success: reg <%d> value:      %d", reg_addr, reg_val);

	return 0;
}

static void device_name_get(size_t idx, struct shell_static_entry *entry);

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help  = NULL;
	entry->subcmd = NULL;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_trinamic_cmds,
			       SHELL_CMD_ARG(read,
					     &dsub_device_name,
					     "Read register value\n"
					     "Usage: tmc read <device> <reg_addr>",
					     cmd_tmc_read,
					     3,
					     0),

			       SHELL_CMD_ARG(write,
					     &dsub_device_name,
					     "Write value into register\n"
					     "Usage: tmc write <device> <reg_addr> <value>",
					     cmd_tmc_write,
					     4,
					     0),
			       SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(tmc, &sub_trinamic_cmds, "Trinamic motor controller commands", NULL);

