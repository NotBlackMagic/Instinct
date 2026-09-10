/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/PubSub/Messages/actuators.hpp
 * Author:  NotBlackMagic
 * Brief:   Topic message definition for actuator messages.
 */

#pragma once

#include "common.hpp"

// Enforce byte-alignment
#pragma pack(push, 1)

struct ActuatorMsg {
	Timestamp timestamp;	// Microseconds (Syncs with the StateMsg timestamp)

	bool isArmed;			// Hardware-level failsafe
	float outputs[16];		// Universal outputs [-1.0 to 1.0], hardware maps these to PWM/DShot
};

#pragma pack(pop) // Restore default compiler alignment