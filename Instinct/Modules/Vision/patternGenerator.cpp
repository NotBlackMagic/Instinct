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

Status PatternGenerator::Checkerboard(VisionFrame& frame) {
	if(frame.startAddress == nullptr || frame.width == 0 || frame.height == 0) {
		return Status::Error;
	}

	const uint32_t squareSize = 16; // 16x16 pixel squares
	uint8_t* ptr = frame.startAddress;
	uint32_t bytesWritten = 0;

	for(uint16_t y = 0; y < frame.height; y++) {
		for(uint16_t x = 0; x < frame.width; x += 2) {
			// Determine square color
			bool isWhiteSquare = ((x / squareSize) % 2) == ((y / squareSize) % 2);

			// Use safe video-level Luma (16 to 235) to prevent math overflows
			uint16_t rgb565Val = isWhiteSquare ? 0xFFFF : 0x0000;
			uint8_t yVal = isWhiteSquare ? 235 : 16;
			uint8_t uVal = 128; // Neutral Chroma
			uint8_t vVal = 128; // Neutral Chroma

			switch(frame.format) {
				case PixelFormat::YUV422_YVYU:
					// YVYU is MACROPIXEL: Y0 V0 Y1 U0
					if (x % 2 == 0) {
						*ptr++ = yVal;	// Y0
						*ptr++ = vVal;	// V
					} else {
						*ptr++ = yVal;	// Y1
						*ptr++ = uVal;	// U
					}
					bytesWritten += 2;
					break;

				case PixelFormat::YUV422_YUYV:
					// YUYV is MACROPIXEL: Y0 U0 Y1 V0
					if (x % 2 == 0) {
						*ptr++ = yVal;	// Y0
						*ptr++ = uVal;	// U
					} else {
						*ptr++ = yVal;	// Y1
						*ptr++ = vVal;	// V
					}
					bytesWritten += 2;
					break;

				case PixelFormat::RGB565: {
					uint16_t color = rgb565Val;
					*ptr++ = (uint8_t)(color & 0xFF);			// Low byte
					*ptr++ = (uint8_t)((color >> 8) & 0xFF);	// High byte
					bytesWritten += 2;
					break;
				}

				case PixelFormat::Grayscale:
					*ptr++ = yVal; // Luma (Y) channel directly maps to Grayscale
					bytesWritten += 1;
					break;

				default:
					return Status::Error; // Unsupported format
			}
			
			bytesWritten += 4;
		}
	}

	frame.payloadSize = bytesWritten;
	return Status::Ok;
}

// Status PatternGenerator::HighEntropyNoise(VisionFrame& frame) {
// 	if(frame.startAddress == nullptr || frame.width == 0 || frame.height == 0) {
// 		return Status::Error;
// 	}

// 	uint8_t* ptr = frame.startAddress;
// 	uint32_t bytesWritten = 0;

// 	// Fast inline pseudo-random number generator (Xorshift)
// 	// Avoids the overhead of rand()
// 	static uint32_t prngState = 123456789;
// 	auto fastRand8 = [&]() -> uint8_t {
// 		prngState ^= prngState << 13;
// 		prngState ^= prngState >> 17;
// 		prngState ^= prngState << 5;
// 		return static_cast<uint8_t>(prngState & 0xFF);
// 	};

// 	// We only need to handle YUYV since we proved the camera is outputting that
// 	if (frame.format != PixelFormat::YUV422_YVYU) {
// 		return Status::Error; 
// 	}

// 	for(uint16_t y = 0; y < frame.height; y++) {
// 		// x increments by 2 because YUV422 packs 2 pixels into 4 bytes (Macropixel)
// 		for(uint16_t x = 0; x < frame.width; x += 2) {
			
// 			// Generate heavy Luma (Y) noise to destroy DCT compression
// 			uint8_t y0 = 30 + (fastRand8() % 190); // Random brightness between 30 and 220
// 			uint8_t y1 = 30 + (fastRand8() % 190);
			
// 			// Generate mild Chroma (U/V) noise around the neutral 128 point
// 			uint8_t u0 = 100 + (fastRand8() % 56); // Random color drift
// 			uint8_t v0 = 100 + (fastRand8() % 56);

// 			// Pack the YUYV Macropixel: Y0 U0 Y1 V0
// 			*ptr++ = y0;
// 			*ptr++ = v0;
// 			*ptr++ = y1;
// 			*ptr++ = u0;
			
// 			bytesWritten += 4;
// 		}
// 	}

// 	frame.payloadSize = bytesWritten;
// 	return Status::Ok;
// }