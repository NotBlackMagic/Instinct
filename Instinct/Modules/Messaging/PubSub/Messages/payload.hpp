/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/PubSub/Messages/payload.hpp
 * Author:  NotBlackMagic
 * Brief:   Topic message definition for payload messages.
 */

#pragma once

#include "common.hpp"

// Enforce byte-alignment
#pragma pack(push, 1)

struct GimbalMsg {
	Timestamp timestamp;	// Microseconds

	float roll;				// radians
	float pitch;			// radians
	float yaw;				// radians

	float pitchRate;		// rad/s
	float yawRate;			// rad/s
};

#pragma pack(pop) // Restore default compiler alignment