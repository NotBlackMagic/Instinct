/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/PubSub/Messages/vehicleStatus.hpp
 * Author:  NotBlackMagic
 * Brief:   Topic message definition for the vehicle status messages.
 */

#pragma once

#include "common.hpp"

// Enforce byte-alignment
#pragma pack(push, 1)

enum class ArmingStatus : uint8_t {
	Init = 0,
	Disarmed = 1,
	Armed = 2,
	Failsafe = 3
};

enum class ControlMode : uint8_t {
	ManualRate = 0,		// Direct angular rate control (Replaces Acro)
	ManualAngle = 1,	// Stabilized attitude control
	VerticalHold = 2,	// Altitude (Air) or Depth (Submarine) hold
	PositionHold = 3,	// 3D coordinate hold
	Autonomous = 4,		// Path following / Waypoint navigation
	ReturnToHome = 5	// Return to launch/rally point
};

struct VehicleStatusMsg {
	Timestamp timestamp;	// Microseconds
	ArmingStatus arming;	// Vehicle armed status
	ControlMode mode;		// Active control mode
};

#pragma pack(pop) // Restore default compiler alignment