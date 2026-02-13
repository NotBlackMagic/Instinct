/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/status.hpp
 * Author:  NotBlackMagic
 * Brief:   Global status codes for all system modules and drivers.
 */

#pragma once

#include <stdint.h>

/// @brief Global return codes for system operations.
enum class Status : uint8_t {
	Ok = 0x00,	///< Operation completed successfully.
	Busy = 0x01,	///< Resource is currently locked or busy.
	Timeout = 0x02,	///< Operation timed out.
	Error = 0xFF	///< Generic failure.
};