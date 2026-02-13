/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/dcmi.hpp
 * Author:  NotBlackMagic
 * Brief:   DCMI (parallel camera interface) driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_exti.h"
#include "stm32n6xx_ll_gpio.h"

#include "status.hpp"

#include "tx_api.h"

/// @brief Driver for the DCMI peripheral.
class Dcmi {
	public:
		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., DCMI).
		Dcmi(DCMI_TypeDef *instance);

		/// @brief Initializes the peripheral clock, the peripheral itself, and interrupts.
		void Init();

		void StartContinuous();
		void Stop();
		bool WaitForFrame(uint32_t timeoutTicks);

		void InterruptHandler();

	private:
		DCMI_TypeDef *instance;
		IRQn_Type irqCall;
		uint8_t irqPriority;
};