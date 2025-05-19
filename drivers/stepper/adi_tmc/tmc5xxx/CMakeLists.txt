# SPDX-FileCopyrightText: Copyright (c) 2025 Dipak Shetty
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources(adi_tmc5xxx_core.c)

add_subdirectory_ifdef(CONFIG_STEPPER_ADI_TMC51XX tmc51xx)
