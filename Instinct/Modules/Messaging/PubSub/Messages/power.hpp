/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/PubSub/Messages/power.hpp
 * Author:  NotBlackMagic
 * Brief:   Topic messages definitions for power messages.
 */

#pragma once

#include "common.hpp"

struct PowerMsg {
	Timestamp timestamp;	// Microseconds
	float voltage;			// Volts (V)
	float current;			// Amperes (A)
	float power;			// Watts (W)
};

struct BatteryMsg {
	Timestamp timestamp;	// Microseconds
	float capacityDrawn;	// Milliampere-hours (mAh)
	uint8_t stateOfCharge;	// Percentage (0-100%)
	bool isLowVoltage;		// Flag for failsafe triggers
};