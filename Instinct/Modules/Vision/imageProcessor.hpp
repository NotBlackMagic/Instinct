/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/imageProcessor.hpp
 * Author:  NotBlackMagic
 * Brief:   Stateless utility class for formatting and processing images/frames.
 */

#pragma once

#include <stdint.h>

#include "logger.hpp"
#include "status.hpp"
#include "visionTypes.hpp"

#include "fx_api.h"

class ImageProcessor {
	public:
		// Delete constructor.
		ImageProcessor() = delete;

		/// @brief Unified entry point. Converts supported raw formats into JPEG-ready MCU blocks.
		/// @param input	The raw input frame, to be converted/transformed.
		/// @param output	The pre-allocated buffer, target location, to hold the MCU blocks.
		static Status ConvertToMCU(const VisionFrame& input, VisionFrame& output);

		static Status ConvertFormat(const VisionFrame& input, VisionFrame& output);

	private:
        // Format-specific highly optimized conversion loops
		static Status ConvertYVYUToYUYV(const VisionFrame& input, VisionFrame& output);

		// Interleaved 4:2:2 (Outputs 16x8 MCUs)
		static Status YUYVToMCU422(const VisionFrame& input, VisionFrame& output);
		static Status YVYUToMCU422(const VisionFrame& input, VisionFrame& output);
		static Status UYVYToMCU422(const VisionFrame& input, VisionFrame& output);
		// Semi-Planar 4:2:0 (Outputs 16x16 MCUs)
		static Status NV12ToMCU420(const VisionFrame& input, VisionFrame& output);
		static Status NV21ToMCU420(const VisionFrame& input, VisionFrame& output);
		// Interleaved 4:4:4 (Outputs 8x8 MCUs)
		static Status YUV444ToMCU444(const VisionFrame& input, VisionFrame& output);
		// RGB Formats
		static Status RGB565ToMCU(const VisionFrame& input, VisionFrame& output);
		static Status RGB888ToMCU(const VisionFrame& input, VisionFrame& output);
		// Single Channel
		static Status GrayscaleToMCU(const VisionFrame& input, VisionFrame& output);
};