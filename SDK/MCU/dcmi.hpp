/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	SDK/MCU/dcmi.hpp
 * Author:	NotBlackMagic
 * Brief:	DCMI (parallel camera interface) driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
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
			SyncPolarity vSyncPolarity;		// When data is NOT valid
			SyncPolarity hSyncPolarity;		// When data is NOT valid
			ClockPolarity pxClkPolarity;	// When data is NOT valid
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

		/// @brief Enables and sets frame crop mode.
		/// @param x Vertical start line
		/// @param y Horizontal start line
		/// @param w Vertical line count (vertical resolution)
		/// @param h Horizontal capture count (horizontal resolution)
		/// @return Status::Ok
		Status ConfigureCrop(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
		
		/// @brief Enables/disables frame decimation by 2 mode. Only works for 8-bit data width
		/// @param halfV 	Capture every other line (vertical resolution decimation by 2)
		/// @param skipCnt	Skip N bytes. For 8-bit pixel data use 1 (skip every other), for 16-bit pixel data use 2 (skip 2 bytes, capture 2 bytes)
		/// @return Status::Ok
		Status ConfigureDecimation(bool halfV, uint8_t skipCnt);

		/// @brief Starts capturing frames.
		/// @param mode: Capturing mode.
		/// @return Status::Ok
		Status Start(CaptureMode mode);

		/// @brief Aborts the current capture.
		/// @return Status::Ok.
		Status Stop();

		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler();

	private:
		DCMI_TypeDef *instance;
		IRQn_Type irqCall;
		uint8_t irqPriority;
};