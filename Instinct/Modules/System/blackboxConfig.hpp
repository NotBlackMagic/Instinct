/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/blackboxConfig.hpp
 * Author:  NotBlackMagic
 * Brief:   Centralized configuration for Blackbox SD logging rates and topic selection.
 */

#pragma once

#include <stdint.h>

#include "broker.hpp"
#include "logger.hpp"
#include "topicIDs.hpp"

class BlackboxConfig {
	public:
		// Delete copy constructors.
		BlackboxConfig() = delete;
		BlackboxConfig(const BlackboxConfig&) = delete;
		void operator=(const BlackboxConfig&) = delete;

		/// @brief Applies the standard flight logging profile (Sensors, State, RC at moderate rates)
		static void ApplyStandardProfile();

		/// @brief Applies high-frequency logging for PID and filter tuning (Native IMU rates)
		static void ApplyTuningProfile();

		/// @brief Disables all SD logging for all topics
		static void DisableAll();
};