/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/PubSub/Messages/common.hpp
 * Author:  NotBlackMagic
 * Brief:   Common structure definitions for topic messages.
 */

#pragma once

#include <stdint.h>

struct Vector3f {
	float x, y, z;
};

struct Quaternion {
	float x, y, z, w;
};

struct ControlWrench {
	Vector3f force;		// Linear effort: X, Y, Z [-1.0f to 1.0f]
	Vector3f torque;	// Angular effort: Roll, Pitch, Yaw [-1.0f to 1.0f]
};

using Timestamp = uint64_t;