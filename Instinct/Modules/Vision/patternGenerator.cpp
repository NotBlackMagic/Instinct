/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/patternGenerator.cpp
 */

#include "patternGenerator.hpp"

Status PatternGenerator::ColorBar(VisionFrame& frame) {
	if(frame.startAddress == nullptr || frame.width == 0 || frame.height == 0) {
		return Status::Error;
	}

	uint32_t barWidth = frame.width / 8;
	uint8_t* ptr = frame.startAddress;
	uint32_t bytesWritten = 0;

	// SMPTE Color Bars: White, Yellow, Cyan, Green, Magenta, Red, Blue, Black
	// Standard YUV values
	const uint8_t yVals[8] = { 255, 226, 179, 150, 105, 76, 29, 0};
	const uint8_t uVals[8] = { 128, 0, 170, 43, 212, 84, 255, 128};
	const uint8_t vVals[8] = { 128, 149, 0, 21, 234, 255, 107, 128};

	// RGB565 values
	const uint16_t rgb565Vals[8] = {
		0xFFFF,		// White
		0xFFE0,		// Yellow
		0x07FF,		// Cyan
		0x07E0,		// Green
		0xF81F,		// Magenta
		0xF800,		// Red
		0x001F,		// Blue
		0x0000		// Black
	};

	for(uint16_t y = 0; y < frame.height; y++) {
		for(uint16_t x = 0; x < frame.width; x++) {
			uint8_t barIdx = x / barWidth;
			if(barIdx > 7) {
				barIdx = 7;
			}

			switch(frame.format) {
				case PixelFormat::YUV422_YVYU:
					// YVYU is MACROPIXEL: Y0 V0 Y1 U0
					if (x % 2 == 0) {
						*ptr++ = yVals[barIdx];	// Y0
						*ptr++ = vVals[barIdx];	// V
					} else {
						*ptr++ = yVals[barIdx];	// Y1
						*ptr++ = uVals[barIdx];	// U
					}
					bytesWritten += 2;
					break;

				case PixelFormat::YUV422_YUYV:
					// YUYV is MACROPIXEL: Y0 U0 Y1 V0
					if (x % 2 == 0) {
						*ptr++ = yVals[barIdx];	// Y0
						*ptr++ = uVals[barIdx];	// U
					} else {
						*ptr++ = yVals[barIdx];	// Y1
						*ptr++ = vVals[barIdx];	// V
					}
					bytesWritten += 2;
					break;

				case PixelFormat::RGB565: {
					uint16_t color = rgb565Vals[barIdx];
					*ptr++ = (uint8_t)(color & 0xFF);			// Low byte
					*ptr++ = (uint8_t)((color >> 8) & 0xFF);	// High byte
					bytesWritten += 2;
					break;
				}

				case PixelFormat::Grayscale:
					*ptr++ = yVals[barIdx]; // Luma (Y) channel directly maps to Grayscale
					bytesWritten += 1;
					break;

				default:
					return Status::Error; // Unsupported format
			}
		}
	}

	frame.payloadSize = bytesWritten;
	return Status::Ok;
}