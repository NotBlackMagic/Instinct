/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/patternGenerator.hpp
 * Author:  NotBlackMagic
 * Brief:   Utility to generate synthetic video frames for pipeline testing.
 */

#pragma once

#include "status.hpp"
#include "visionTypes.hpp"

class PatternGenerator {
	public:
		/// @brief Fills the provided VisionFrame with SMPTE-style vertical color bars.
		/// @param frame The frame buffer to populate. Metadata (width, height, format) must be pre-configured.
		/// @return Status::Ok on success, Status::Error if the pixel format is unsupported.
		static Status ColorBar(VisionFrame& frame);

		/// @brief Fills the provided VisionFrame with a BW checkerboard pattern.
		/// @param frame The frame buffer to populate. Metadata (width, height, format) must be pre-configured.
		/// @return Status::Ok on success, Status::Error if the pixel format is unsupported.
		static Status Checkerboard(VisionFrame& frame);
};