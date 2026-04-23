/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/pwm.hpp
 * Author:  NotBlackMagic
 * Brief:   PWM driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

 #pragma once

#include <stdint.h>

#include "timer.hpp"

#include "status.hpp"

/// @brief PWM driver mapped to a specific Timer channel.
/// @note  Multiple PWM objects can share the same timer instance, but they MUST share the same frequency.
class PWM {
	public:
		/// @brief Defines the Timer Channel.
		enum class Channel : uint32_t {
			Ch1 = LL_TIM_CHANNEL_CH1,
			Ch2 = LL_TIM_CHANNEL_CH2,
			Ch3 = LL_TIM_CHANNEL_CH3,
			Ch4 = LL_TIM_CHANNEL_CH4,
			Ch5 = LL_TIM_CHANNEL_CH5,
			Ch6 = LL_TIM_CHANNEL_CH6,
		};

		/// @brief PWM output polarity.
		enum class Polarity : uint32_t {
			High = LL_TIM_OCPOLARITY_HIGH,	///< Active high (duty cycle represents high time)
			Low = LL_TIM_OCPOLARITY_LOW		///< Active low (duty cycle represents low time)
		};

		/// @brief PWM configuration structure.
		struct Config {
			Polarity polarity = Polarity::High; ///< Default to active high
		};

		// Delete copy constructors
		PWM(const PWM&) = delete;
		PWM& operator=(const PWM&) = delete;

		/// @brief Constructor.
		/// @param timer	Reference to the initialized hardware timer base.
		/// @param channel	The specific timer channel to output PWM on.
		PWM(Timer &timer, Channel channel);

		/// @brief Initialize the PWM channel and adjusts the underlying timer configuration.
		/// @param config PWM configuration.
		/// @return Status::Ok on success.
		Status Init(const Config &config);

		/// @brief Enables the PWM output on this specific channel.
		void Start();

		/// @brief Disables the PWM output on this specific channel.
		void Stop();

		/// @brief Sets the duty cycle as a percentage.
		/// @param dutyCycle Value from 0.0f (0%) to 1.0f (100%).
		void SetDutyCycle(float dutyCycle);

		/// @brief Set the PWM pulse width in microseconds.
		/// @param pulseWidthUs The pulse width in microseconds.
		void SetPulseWidth(uint32_t pulseWidthUs);

	private:
		Timer &timer;		// Reference to the explicit timebase
		Channel channel;	// The target channel

		void SetCompareValue(uint32_t value);
};