/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/timer.hpp
 * Author:  NotBlackMagic
 * Brief:   Timer driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_tim.h"

#include "status.hpp"

/// @brief Driver for STM32 hardware timers acting as a shared timebase.
/// @note  This class handles the core clocking, prescaler, and auto-reload values.
///        It provides a foundation for dependent classes like PWM or Input Capture.
class Timer {
	public:
		/// @brief Configuration structure for the timer base.
		struct Config {
			uint32_t sourceClockHz; ///< Peripheral source clock frequency in Hz
			uint32_t frequencyHz;   ///< Desired timer update frequency
		};

		// Delete copy constructors
		Timer(const Timer&) = delete;
		Timer& operator=(const Timer&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware timer instance (e.g., TIM1, TIM2).
		Timer(TIM_TypeDef *instance);

		/// @brief Initializes the timer base.
		/// @param config Timer configuration.
		/// @return Status::Ok on success, Status::Error if parameters are invalid.
		Status Init(const Config &config);

		/// @brief Starts the timer counter (sets the CEN bit).
		void Start();

		/// @brief Stops the timer counter (clears the CEN bit).
		void Stop();

		/// @brief Forces the counter back to zero and generates an update event.
		void Reset();

		/// @brief Gets the underlying hardware instance.
		/// @note Required by dependent classes (like PWM) to configure their specific channels.
		/// @return Pointer to the TIM_TypeDef.
		TIM_TypeDef* GetInstance() const;

		/// @brief Gets the calculated Auto-Reload value.
		/// @note Required by dependent classes (like PWM) to calculate exact CCR ratios for duty cycles.
		/// @return The ARR value currently configured in the hardware.
		uint32_t GetAutoReload() const;

		/// @brief Gets the core tick frequency of the timer (after the prescaler).
		/// @return The timer tick frequency in Hz.
		uint32_t GetFrequencyHz() const;

	private:
		TIM_TypeDef *instance;
		uint32_t tickFrequencyHz;
		uint32_t autoReloadValue;
		bool isInitialized;
};