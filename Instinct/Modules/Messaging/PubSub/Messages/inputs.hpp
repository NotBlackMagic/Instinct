/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/PubSub/Messages/inputs.hpp
 * Author:  NotBlackMagic
 * Brief:   Topic messages definitions for input (RC/Gamepad) messages.
 */

#pragma once

#include "common.hpp"

struct RCMsg {
	Timestamp timestamp;	// Microseconds (Timestamp of last valid frame)
	uint16_t channels[16];	// 1000-2000us standardized range
	uint16_t channelCount;

	uint8_t linkState;		// 0: Disconnected, 1: Connected, 2: LinkLost, 3: Failsafe
	uint8_t linkQuality;	// 0-100% (CRSF/ELRS primarily)
	int8_t rssi;			// Negative dBm
};

struct ManualControlMsg {
	Timestamp timestamp;	// Microseconds

	// Flight Axes: Normalized from -1.0f to 1.0f
	float roll;				// Positive = Right (Or steering)
	float pitch;			// Positive = Forward (Nose down)
	float yaw;				// Positive = Right (Clockwise) (or Turn)

	// Throttle: Normalized from -1.0f to 1.0f
	float throttle;			// Flight only positive, others -1.0 (Reverse) to 1.0 (Forward)

	// Auxiliary Switches: Normalized from -1.0f to 1.0f
	float aux1;
	float aux2;
	float aux3;
	float aux4;

	uint8_t linkState;
};