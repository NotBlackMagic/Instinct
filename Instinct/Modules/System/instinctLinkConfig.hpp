/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Messaging/instinctLinkConfig.hpp
 * Author:	NotBlackMagic
 * Brief:	Centralized configuration for InstinctLink streaming profiles and topic rates.
 */

#pragma once

#include <stdint.h>

#include "instinctLink.hpp"
#include "logger.hpp"
#include "topicIDs.hpp"

class InstinctLinkConfig {
	public:
		InstinctLinkConfig() = delete;
		InstinctLinkConfig(const InstinctLinkConfig&) = delete;
		void operator=(const InstinctLinkConfig&) = delete;

		/// @brief Live PID & filter tuning profile (high-rate setpoints, state, and actuators)
		static void ApplyTuningProfile();

		/// @brief Hardware-in-the-loop simulation profile (maximum rate actuator streaming)
		static void ApplyHITLProfile();

		/// @brief Moderate rate monitoring profile for desktop telemetry / diagnostics
		static void ApplyMonitorProfile();

		/// @brief Disables all periodic topic streams
		static void DisableAll();
};