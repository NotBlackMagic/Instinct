/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/PubSub/Messages/topicIDs.hpp
 * Author:  NotBlackMagic
 * Brief:   List of all topic IDs available.
 */

#pragma once

#include <stdint.h>

enum class TopicID : uint8_t {
	// Inertial Sensors
	Accel = 1,
	Gyro = 2,
	Mag = 3,
	Baro = 4,
	Imu = 5,

	// GNSS Sensors
	NavSat = 10,
	NavRTCM = 11,

	// Environment/Ranging Sensors
	Range = 20,
	Tacho = 21,

	// Power Sensors
	Power = 30,
	Battery = 31,

	// Inputs
	RC = 40
};