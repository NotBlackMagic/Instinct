/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Messaging/PubSub/Messages/parameters.hpp
 * Author:	NotBlackMagic
 * Brief:	Topic message definition for parameter update notifications.
 */

 #pragma once

#include "common.hpp"

// Enforce byte-alignment
#pragma pack(push, 1)

struct ParamUpdateMsg {
	Timestamp timestamp;	// Microseconds
	char name[16];			// Parameter name (MAVLink standard 16-char max)
	uint32_t hash;			// 32-bit FNV-1a hash of paramter name (0xFFFFFFFF = bulk reset)
	uint16_t paramIndex;	// Sequential index for MAVLink (0xFFFF = bulk reset)
	uint8_t type;			// 0: Float, 1: Int32
	union {
		float fVal;
		int32_t iVal;
	} value;				// New value
};

#pragma pack(pop)