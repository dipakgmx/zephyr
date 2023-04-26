# SPDX-License-Identifier: Apache-2.0

zephyr_library()

add_subdirectory_ifdef(CONFIG_TRINAMIC_TMC5041          tmc5041)

zephyr_library_sources_ifdef(CONFIG_TMC_SPI             tmc_spi.c)
zephyr_library_sources_ifdef(CONFIG_TRINAMIC_SHELL      tmc_shell.c)
