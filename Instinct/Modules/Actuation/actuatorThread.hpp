/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Actuation/actuatorThread.hpp
 * Author:	NotBlackMagic
 * Brief:	Hardware output layer. Maps unified physics outputs to physical PWM signals.
 */

#pragma once

#include "hardware.hpp"

#include "logger.hpp"
#include "pubSub.hpp"

#include "tx_api.h"

class ActuatorThread {
	public:
		/// @brief Initializes the Actuator ThreadX loop.
		static void Init();

	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[4096];

		// Subscribers
		static Subscriber<ActuatorMsg> subActuators;
		static Topic<ActuatorMsg>* topicActuators;

		// Hardware Configuration
		enum class ChannelType {
			Motor,	// Unidirectional: 0.0 to 1.0
			Servo	// Bidirectional: -1.0 to 1.0
		};

		struct PinConfig {
			ChannelType type;
			uint16_t min;		// Typically 1000
			uint16_t max;		// Typically 2000
			uint16_t center;	// 1500 for Servos, 1000 for Motors
			uint16_t idle;		// Value to send when disarmed or failsafed (e.g., 900 for motors to stop spinning)
		};

		// Configuration for the 16 possible outputs
		static PinConfig pinConfigs[16];
		static PWM* hwPWM[16];

		/// @brief Main output loop, synchronized to the Controller's 1000Hz publication.
		static void Run(ULONG input);

		/// @brief Maps the normalized math output [-1.0 to 1.0] to a microsecond pulse width.
		static uint16_t MapToMicroseconds(float output, const PinConfig& config);
		
		/// @brief Instantly forces all configured hardware pins to their safe 'disarmUs' value.
		static void ForceSafeOutputs();
};