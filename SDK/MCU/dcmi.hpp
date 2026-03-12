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
		/// @brief DCMI frame capture mode.
		enum class CaptureMode {
			Continuous,
			Snapshot
		};

		/// @brief DCMI V/H Sync polarity.
		enum class SyncPolarity : uint8_t {
			Low = 0,
			High = 1
		};

		/// @brief DCMI pixel clock polarity.
		enum class ClockPolarity : uint8_t {
			Falling = 0,
			Rising = 1
		};

		/// @brief DCMI peripheral configuration structure.
		struct Config {
			SyncPolarity vSyncPolarity;
			SyncPolarity hSyncPolarity;
			ClockPolarity pxClkPolarity;
			bool embeddedSync;
		};

		// Delete copy constructors
		Dcmi(const Dcmi&) = delete;
		Dcmi& operator=(const Dcmi&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., DCMI).
		Dcmi(DCMI_TypeDef *instance);

		/// @brief Initializes the peripheral clock, the peripheral itself, and interrupts.
		/// @param config DCMI configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		/// @brief Starts capturing frames.
		/// @param mode: Capturing mode.
		/// @return Status::Ok
		Status Start(CaptureMode mode);

		void Stop();

		bool WaitForFrame(uint32_t timeoutTicks);

		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler();

	private:
		DCMI_TypeDef *instance;
		IRQn_Type irqCall;
		uint8_t irqPriority;
};