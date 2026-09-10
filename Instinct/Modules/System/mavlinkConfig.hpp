/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/mavlinkConfig.hpp
 * Author:  NotBlackMagic
 * Brief:   Centralized configuration for MAVLink telemetry stream rates.
 */

#pragma once

#include <stdint.h>

#include "mavlinkThread.hpp"
#include "logger.hpp"

class MavlinkConfig {
	public:
		// Delete copy constructors.
		MavlinkConfig() = delete;
		MavlinkConfig(const MavlinkConfig&) = delete;
		void operator=(const MavlinkConfig&) = delete;

		/// @brief Standard telemetry profile for basic QGroundControl flight
		static void ApplyStandardProfile();

		/// @brief High-bandwidth profile (requires USB or high-speed radio)
		static void ApplyHighSpeedProfile();

		/// @brief Disables all streams except the 1Hz Heartbeat (Failsafe mode)
		static void ApplyQuietProfile();

		/// @brief Disables all streams
		static void DisableAll();
};