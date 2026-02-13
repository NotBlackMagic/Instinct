/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Board/PlumaN6.hpp
 * Author:  NotBlackMagic
 * Brief:   File contains board specific initializations.
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_exti.h"
#include "stm32n6xx_ll_gpio.h"

void BoardGPIOInit();
void BoardCAN1Init();
void BoardCAN2Init();
void BoardCAN3Init();
void BoardDCMIInit();
void BoardI2C1Init();
void BoardI2C2Init();
void BoardI2C3Init();
void BoardI2C4Init();
void BoardI3C1Init();
void BoardI3C2Init();
void BoardPWM2Init();
void BoardPWM8Init();
void BoardSDMMC1Init();
void BoardSDMMC2Init();
void BoardSPI1Init();
void BoardSPI2Init();
void BoardSPI4Init();
void BoardSPI5Init();
void BoardUART3Init();
void BoardUART4Init();
void BoardUART6Init();
void BoardUART7Init();
void BoardUART8Init();
void BoardXSPI1Init();
void BoardXSPI2Init();