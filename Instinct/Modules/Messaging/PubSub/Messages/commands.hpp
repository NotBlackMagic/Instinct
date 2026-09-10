/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/PubSub/Messages/commands.hpp
 * Author:  NotBlackMagic
 * Brief:   Topic message definition for control command messages.
 */

#pragma once

#include "common.hpp"

// Bitmask flags to tell the Controller exactly which fields are active
enum class SetpointMask : uint32_t {
	None			= 0,
	PositionXY		= (1 << 0),
	PositionZ		= (1 << 1),
	VelocityXY		= (1 << 2),
	VelocityZ		= (1 << 3),
	LinearEffortXY	= (1 << 4),		// e.g., Rover forward/reverse or Plane throttle
	LinearEffortZ	= (1 << 5),		// e.g., Drone direct thrust
	Attitude		= (1 << 6),
	AngularRate		= (1 << 7)
};

// Overload bitwise operators for the enum class
inline SetpointMask operator|(SetpointMask a, SetpointMask b) {
	return static_cast<SetpointMask>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
inline bool operator&(SetpointMask a, SetpointMask b) {
	return (static_cast<uint32_t>(a) & static_cast<uint32_t>(b)) != 0;
}

// Enforce byte-alignment
#pragma pack(push, 1)

// Structures are organized to be memory boundary aligned
struct ControlSetpointMsg {
	Timestamp timestamp;		// Microseconds

	SetpointMask activeMask;	// Which of the fields below should be used

	// Translation / Linear Setpoints
	Vector3f position;			// Local NED/ENU (meters)
	Vector3f velocity;			// Local NED/ENU (m/s)
	Vector3f linearEffort;		// Normalized [-1.0f to 1.0f] (Direct thrust/throttle)

	// Rotation / Angular Setpoints
	Quaternion attitude;		// Target Orientation
	Vector3f angularRates;		// Target Rates (rad/s)

	// Auxiliary / Passthrough
	float auxiliary[4];			// Normalized [-1.0f to 1.0f] (Gimbals, Flaps, Landing Gear)

	uint8_t controlSource;		// 0: RC, 1: MAVLink, 2: VSLAM/Companion
	uint8_t _padding[3];		// Explicitly pad to reach a 4-byte boundary for CPU efficiency
};

#pragma pack(pop) // Restore default compiler alignment