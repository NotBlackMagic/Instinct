/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    tx_pluman6_sd_driver.h
 * Author:  NotBlackMagic
 * Brief:   FileX glue logic, interface to SD card driver.
 */

#ifndef FX_STM32_SD_DRIVER_H
#define FX_STM32_SD_DRIVER_H

#include "hardware.hpp"
#include "sd.hpp"
#include "status.hpp"

#include "fx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

// The entry point for FileX to talk to your SD Driver
void PlumaSDDriver(FX_MEDIA *media_ptr);

#ifdef __cplusplus
}
#endif

#endif // FX_STM32_SD_DRIVER_H