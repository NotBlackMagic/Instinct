/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/imageProcessor.cpp
 */

#include "imageProcessor.hpp"

Status ImageProcessor::ConvertToMCU(const VisionFrame& input, VisionFrame& output) {
	if(input.startAddress == nullptr || output.startAddress == nullptr) {
		return Status::Error;
	}

	switch (input.format) {
		// Interleaved 4:2:2 (Outputs 16x8 MCUs)
		case PixelFormat::YUV422_YUYV:
			return YUYVToMCU422(input, output);
		case PixelFormat::YUV422_YVYU:
			return YVYUToMCU422(input, output);
		case PixelFormat::YUV422_UYVY:
			return UYVYToMCU422(input, output);
		// Semi-Planar 4:2:0 (Outputs 16x16 MCUs)
		case PixelFormat::YUV420_NV12:
			return NV12ToMCU420(input, output);
		case PixelFormat::YUV420_NV21:
			return NV21ToMCU420(input, output);
		// Interleaved 4:4:4 (Outputs 8x8 MCUs)
		case PixelFormat::YUV444:
			return YUV444ToMCU444(input, output);
		// RGB Formats (Requires Color Space Conversion -> 4:2:2 or 4:4:4 MCUs)
		case PixelFormat::RGB565:
			return RGB565ToMCU(input, output);
		case PixelFormat::RGB888:
			return RGB888ToMCU(input, output);
		// Single Channel (Outputs 8x8 Luma-only MCUs)
		case PixelFormat::Grayscale:
			GrayscaleToMCU(input, output);
		// Hardware-Delegated or Invalid Formats
		case PixelFormat::BayerRaw8:
		case PixelFormat::BayerRaw10:
		case PixelFormat::Unknown:
		default:
			return Status::Error; // Unsupported format
	}
}

Status ImageProcessor::YUYVToMCU422(const VisionFrame& input, VisionFrame& output) {
	uint32_t width = input.width;
	uint32_t height = input.height;

	uint8_t* inBuf = input.startAddress;
	uint8_t* outBuf = output.startAddress;

	// YCbCr 4:2:2 MCU is: Two 8x8 Y block + one 8x8 Cb block + one 8x8 Cr block = Total 256 bytes
	// YUV 4:2:2: Cb and Cr are horizontally sampled at the half compared to the Y component (keeping only the chrominance information of one pixel over two horizontally adjacent pixels).
	//    Luminance Y      Chrominance (Cb and Cr)
	// | A | B | C | D |     |   A   |   C   |
	// -----------------  +  -----------------
	// | E | F | G | H |     |   E   |   G   |
	uint32_t hMCUCount = width / 16;
	uint32_t vMCUCount = height / 8;
	uint32_t mcuIndex = 0;

	for(uint32_t mcuY = 0; mcuY < vMCUCount; mcuY++) {
		for(uint32_t mcuX = 0; mcuX < hMCUCount; mcuX++) {
			
			uint8_t* currentMcuBase = outBuf + (mcuIndex * 256);
			uint8_t* destY0 = currentMcuBase;			// First 64 bytes (Left Y)
			uint8_t* destY1 = currentMcuBase + 64;		// Next 64 bytes (Right Y)
			uint8_t* destCb = currentMcuBase + 128;		// Next 64 bytes
			uint8_t* destCr = currentMcuBase + 192;		// Final 64 bytes

			// Read the 16x8 block out of the interleaved image
			for(uint32_t row = 0; row < 8; row++) {
				uint32_t globalY = (mcuY * 8) + row;
				uint32_t globalX = (mcuX * 16);
				
				// YVYU is 2 bytes per pixel
				uint32_t inOffset = (globalY * width * 2) + (globalX * 2);
				
				for(uint32_t col = 0; col < 16; col += 2) {
					// Extract the 4 bytes for this macropixel
					uint8_t y0 = inBuf[inOffset];		// Y0
					uint8_t u  = inBuf[inOffset + 1];	// U
					uint8_t y1 = inBuf[inOffset + 2];	// Y1
					uint8_t v  = inBuf[inOffset + 3];	// V

					// Route the Y pixels to the correct 8x8 block
					if(col < 8) {
						*destY0++ = y0;
						*destY0++ = y1;
					}
					else {
						*destY1++ = y0;
						*destY1++ = y1;
					}

					// Route Chroma (Cb/Cr only have one 8x8 block per MCU)
					*destCb++ = u;
					*destCr++ = v;

					inOffset += 4;
				}
			}
			mcuIndex++;
		}
	}

	output.width = width;
	output.height = height;
	output.format = PixelFormat::YUV422_YUYV;
	output.payloadSize = mcuIndex * 256;

	return Status::Ok;
}

Status ImageProcessor::UYVYToMCU422(const VisionFrame& input, VisionFrame& output) {
	uint32_t width = input.width;
	uint32_t height = input.height;

	uint8_t* inBuf = input.startAddress;
	uint8_t* outBuf = output.startAddress;

	// YCbCr 4:2:2 MCU is: Two 8x8 Y block + one 8x8 Cb block + one 8x8 Cr block = Total 256 bytes
	// YUV 4:2:2: Cb and Cr are horizontally sampled at the half compared to the Y component (keeping only the chrominance information of one pixel over two horizontally adjacent pixels).
	//    Luminance Y      Chrominance (Cb and Cr)
	// | A | B | C | D |     |   A   |   C   |
	// -----------------  +  -----------------
	// | E | F | G | H |     |   E   |   G   |
	uint32_t hMCUCount = width / 16;
	uint32_t vMCUCount = height / 8;
	uint32_t mcuIndex = 0;

	for(uint32_t mcuY = 0; mcuY < vMCUCount; mcuY++) {
		for(uint32_t mcuX = 0; mcuX < hMCUCount; mcuX++) {
			
			uint8_t* currentMcuBase = outBuf + (mcuIndex * 256);
			uint8_t* destY0 = currentMcuBase;			// First 64 bytes (Left Y)
			uint8_t* destY1 = currentMcuBase + 64;		// Next 64 bytes (Right Y)
			uint8_t* destCb = currentMcuBase + 128;		// Next 64 bytes
			uint8_t* destCr = currentMcuBase + 192;		// Final 64 bytes

			// Read the 16x8 block out of the interleaved image
			for(uint32_t row = 0; row < 8; row++) {
				uint32_t globalY = (mcuY * 8) + row;
				uint32_t globalX = (mcuX * 16);
				
				// YVYU is 2 bytes per pixel
				uint32_t inOffset = (globalY * width * 2) + (globalX * 2);
				
				for(uint32_t col = 0; col < 16; col += 2) {
					// Extract the 4 bytes for this macropixel
					uint8_t u = inBuf[inOffset];		// U
					uint8_t y0  = inBuf[inOffset + 1];	// Y0
					uint8_t v = inBuf[inOffset + 2];	// V
					uint8_t y1  = inBuf[inOffset + 3];	// Y1

					// Route the Y pixels to the correct 8x8 block
					if(col < 8) {
						*destY0++ = y0;
						*destY0++ = y1;
					}
					else {
						*destY1++ = y0;
						*destY1++ = y1;
					}

					// Route Chroma (Cb/Cr only have one 8x8 block per MCU)
					*destCb++ = u;
					*destCr++ = v;

					inOffset += 4;
				}
			}
			mcuIndex++;
		}
	}

	output.width = width;
	output.height = height;
	output.format = PixelFormat::YUV422_YUYV;
	output.payloadSize = mcuIndex * 256;

	return Status::Ok;
}

Status ImageProcessor::YVYUToMCU422(const VisionFrame& input, VisionFrame& output) {
	uint32_t width = input.width;
	uint32_t height = input.height;

	uint8_t* inBuf = input.startAddress;
	uint8_t* outBuf = output.startAddress;

	// YCbCr 4:2:2 MCU is: Two 8x8 Y block + one 8x8 Cb block + one 8x8 Cr block = Total 256 bytes
	// YUV 4:2:2: Cb and Cr are horizontally sampled at the half compared to the Y component (keeping only the chrominance information of one pixel over two horizontally adjacent pixels).
	//    Luminance Y      Chrominance (Cb and Cr)
	// | A | B | C | D |     |   A   |   C   |
	// -----------------  +  -----------------
	// | E | F | G | H |     |   E   |   G   |
	uint32_t hMCUCount = width / 16;
	uint32_t vMCUCount = height / 8;
	uint32_t mcuIndex = 0;

	for(uint32_t mcuY = 0; mcuY < vMCUCount; mcuY++) {
		for(uint32_t mcuX = 0; mcuX < hMCUCount; mcuX++) {
			
			uint8_t* currentMcuBase = outBuf + (mcuIndex * 256);
			uint8_t* destY0 = currentMcuBase;			// First 64 bytes (Left Y)
			uint8_t* destY1 = currentMcuBase + 64;		// Next 64 bytes (Right Y)
			uint8_t* destCb = currentMcuBase + 128;		// Next 64 bytes
			uint8_t* destCr = currentMcuBase + 192;		// Final 64 bytes

			// Read the 16x8 block out of the interleaved image
			for(uint32_t row = 0; row < 8; row++) {
				uint32_t globalY = (mcuY * 8) + row;
				uint32_t globalX = (mcuX * 16);
				
				// YVYU is 2 bytes per pixel
				uint32_t inOffset = (globalY * width * 2) + (globalX * 2);
				
				for(uint32_t col = 0; col < 16; col += 2) {
					// Extract the 4 bytes for this macropixel
					uint8_t y0 = inBuf[inOffset];		// Y0
					uint8_t v  = inBuf[inOffset + 1];	// V
					uint8_t y1 = inBuf[inOffset + 2];	// Y1
					uint8_t u  = inBuf[inOffset + 3];	// U

					// Route the Y pixels to the correct 8x8 block
					if(col < 8) {
						*destY0++ = y0;
						*destY0++ = y1;
					}
					else {
						*destY1++ = y0;
						*destY1++ = y1;
					}

					// Route Chroma (Cb/Cr only have one 8x8 block per MCU)
					*destCb++ = u;
					*destCr++ = v;

					inOffset += 4;
				}
			}
			mcuIndex++;
		}
	}

	output.width = width;
	output.height = height;
	output.format = PixelFormat::YUV422_YUYV;
	output.payloadSize = mcuIndex * 256;

	return Status::Ok;
}

Status ImageProcessor::NV12ToMCU420(const VisionFrame& input, VisionFrame& output) {
	uint32_t width = input.width;
	uint32_t height = input.height;
	uint8_t* inBuf = input.startAddress;
	uint8_t* outBuf = output.startAddress;

	// 4:2:0 MCU is: Four 8x8 Y block + one 8x8 Cb block + one 8x8 Cr block = Total 384 bytes
	// YUV 4:2:0: b and Cr are horizontally and vertically sampled at the half compared to the Y component (keeping only the chrominance information of one pixel over four adjacent pixels).
	//    Luminance Y      Chrominance (Cb and Cr)
	// | A | B | C | D |     |       |       |
	// -----------------  +  |   A   |   C   |
	// | E | F | G | H |     |       |       |
	uint32_t hMCUCount = width / 16;
	uint32_t vMCUCount = height / 16;
	uint32_t mcuIndex = 0;

	// NV12 Memory Layout:
	// 1. Contiguous Y plane (width * height bytes)
	// 2. Interleaved UV plane (width * height / 2 bytes)
	uint8_t* yPlane = inBuf;
	uint8_t* uvPlane = inBuf + (width * height);

	for(uint32_t mcuY = 0; mcuY < vMCUCount; mcuY++) {
		for(uint32_t mcuX = 0; mcuX < hMCUCount; mcuX++) {
			
			uint8_t* currentMcuBase = outBuf + (mcuIndex * 384);
			uint8_t* destY0 = currentMcuBase;             // Top-left Y (64 bytes)
			uint8_t* destY1 = currentMcuBase + 64;        // Top-right Y (64 bytes)
			uint8_t* destY2 = currentMcuBase + 128;       // Bottom-left Y (64 bytes)
			uint8_t* destY3 = currentMcuBase + 192;       // Bottom-right Y (64 bytes)
			uint8_t* destCb = currentMcuBase + 256;       // Cb (64 bytes)
			uint8_t* destCr = currentMcuBase + 320;       // Cr (64 bytes)

			// Extract the 16x16 Y component into four 8x8 blocks
			for(uint32_t row = 0; row < 16; row++) {
				uint32_t globalY = (mcuY * 16) + row;
				uint32_t globalX = (mcuX * 16);
				uint32_t yOffset = (globalY * width) + globalX;

				for(uint32_t col = 0; col < 8; col++) { 
					// Left half
					if(row < 8) {
						*destY0++ = yPlane[yOffset + col];
					}
					else {
						*destY2++ = yPlane[yOffset + col];
					}
				}
				for(uint32_t col = 8; col < 16; col++) { 
					// Right half
					if(row < 8) {
						*destY1++ = yPlane[yOffset + col];
					}
					else {
						*destY3++ = yPlane[yOffset + col];
					}
				}
			}

			// Extract the 8x8 UV component
			for(uint32_t row = 0; row < 8; row++) {
				uint32_t uvGlobalY = (mcuY * 8) + row;
				uint32_t uvGlobalX = (mcuX * 8); 
				uint32_t uvOffset = (uvGlobalY * width) + (uvGlobalX * 2);

				for(uint32_t col = 0; col < 8; col++) {
					// NV12 Layout: U, V, U, V...
					*destCb++ = uvPlane[uvOffset + (col * 2)];      // U
					*destCr++ = uvPlane[uvOffset + (col * 2) + 1];  // V
				}
			}
			mcuIndex++;
		}
	}

	output.width = width;
	output.height = height;

	output.format = PixelFormat::YUV420_NV12; 

	output.payloadSize = mcuIndex * 384;

	return Status::Ok;
}

Status ImageProcessor::NV21ToMCU420(const VisionFrame& input, VisionFrame& output) {
	uint32_t width = input.width;
	uint32_t height = input.height;
	uint8_t* inBuf = input.startAddress;
	uint8_t* outBuf = output.startAddress;

	// 4:2:0 MCU is: Four 8x8 Y block + one 8x8 Cb block + one 8x8 Cr block = Total 384 bytes
	// YUV 4:2:0: b and Cr are horizontally and vertically sampled at the half compared to the Y component (keeping only the chrominance information of one pixel over four adjacent pixels).
	//    Luminance Y      Chrominance (Cb and Cr)
	// | A | B | C | D |     |       |       |
	// -----------------  +  |   A   |   C   |
	// | E | F | G | H |     |       |       |
	uint32_t hMCUCount = width / 16;
	uint32_t vMCUCount = height / 16;
	uint32_t mcuIndex = 0;

	// NV21 Memory Layout:
	// 1. Contiguous Y plane (width * height bytes)
	// 2. Interleaved VU plane (width * height / 2 bytes)
	uint8_t* yPlane = inBuf;
	uint8_t* uvPlane = inBuf + (width * height);

	for(uint32_t mcuY = 0; mcuY < vMCUCount; mcuY++) {
		for(uint32_t mcuX = 0; mcuX < hMCUCount; mcuX++) {
			
			uint8_t* currentMcuBase = outBuf + (mcuIndex * 384);
			uint8_t* destY0 = currentMcuBase;             // Top-left Y (64 bytes)
			uint8_t* destY1 = currentMcuBase + 64;        // Top-right Y (64 bytes)
			uint8_t* destY2 = currentMcuBase + 128;       // Bottom-left Y (64 bytes)
			uint8_t* destY3 = currentMcuBase + 192;       // Bottom-right Y (64 bytes)
			uint8_t* destCb = currentMcuBase + 256;       // Cb (64 bytes)
			uint8_t* destCr = currentMcuBase + 320;       // Cr (64 bytes)

			// Extract the 16x16 Y component into four 8x8 blocks
			for(uint32_t row = 0; row < 16; row++) {
				uint32_t globalY = (mcuY * 16) + row;
				uint32_t globalX = (mcuX * 16);
				uint32_t yOffset = (globalY * width) + globalX;

				for(uint32_t col = 0; col < 8; col++) {
					// Left half
					if(row < 8) {
						*destY0++ = yPlane[yOffset + col];
					}
					else {
						*destY2++ = yPlane[yOffset + col];
					}
				}
				for(uint32_t col = 8; col < 16; col++) {
					// Right half
					if(row < 8) {
						*destY1++ = yPlane[yOffset + col];
					}
					else {
						*destY3++ = yPlane[yOffset + col];
					}
				}
			}

			// Extract the 8x8 VU component (Swapped compared to NV12)
			for(uint32_t row = 0; row < 8; row++) {
				uint32_t uvGlobalY = (mcuY * 8) + row;
				uint32_t uvGlobalX = (mcuX * 8); 
				uint32_t uvOffset = (uvGlobalY * width) + (uvGlobalX * 2);

				for(uint32_t col = 0; col < 8; col++) {
					// NV21 Layout: V, U, V, U...
					*destCr++ = uvPlane[uvOffset + (col * 2)];      // V (Cr) is first
					*destCb++ = uvPlane[uvOffset + (col * 2) + 1];  // U (Cb) is second
				}
			}
			mcuIndex++;
		}
	}

	output.width = width;
	output.height = height;
	output.format = PixelFormat::YUV420_NV12; 
	output.payloadSize = mcuIndex * 384;

	return Status::Ok;
}

Status ImageProcessor::YUV444ToMCU444(const VisionFrame& input, VisionFrame& output) {
	uint32_t width = input.width;
	uint32_t height = input.height;
	uint8_t* inBuf = input.startAddress;
	uint8_t* outBuf = output.startAddress;

	// 4:4:4 MCU is: One 8x8 Y block + one 8x8 Cb block + one 8x8 Cr block = Total 192 bytes
	uint32_t hMCUCount = width / 8;
	uint32_t vMCUCount = height / 8;
	uint32_t mcuIndex = 0;

	for(uint32_t mcuY = 0; mcuY < vMCUCount; mcuY++) {
		for(uint32_t mcuX = 0; mcuX < hMCUCount; mcuX++) {
			
			uint8_t* currentMcuBase = outBuf + (mcuIndex * 192);
			uint8_t* destY  = currentMcuBase;             // First 64 bytes
			uint8_t* destCb = currentMcuBase + 64;        // Next 64 bytes
			uint8_t* destCr = currentMcuBase + 128;       // Final 64 bytes

			for(uint32_t row = 0; row < 8; row++) {
				uint32_t globalY = (mcuY * 8) + row;
				uint32_t globalX = (mcuX * 8);
				
				// YUV444 is 3 bytes per pixel (Y, U, V interleaved)
				uint32_t inOffset = (globalY * width * 3) + (globalX * 3);
				
				for(uint32_t col = 0; col < 8; col++) {
					*destY++  = inBuf[inOffset];
					*destCb++ = inBuf[inOffset + 1];
					*destCr++ = inBuf[inOffset + 2];
					inOffset += 3;
				}
			}
			mcuIndex++;
		}
	}

	output.width = width;
	output.height = height;
	output.format = PixelFormat::YUV444;
	output.payloadSize = mcuIndex * 192;

	return Status::Ok;
}

Status ImageProcessor::RGB565ToMCU(const VisionFrame& input, VisionFrame& output) {
}

Status ImageProcessor::RGB888ToMCU(const VisionFrame& input, VisionFrame& output) {
}

Status ImageProcessor::GrayscaleToMCU(const VisionFrame& input, VisionFrame& output) {
	uint32_t width = input.width;
	uint32_t height = input.height;
	uint8_t* inBuf = input.startAddress;
	uint8_t* outBuf = output.startAddress;

	// Grayscale MCU is a single 8x8 block = 64 bytes
	uint32_t hMCUCount = width / 8;
	uint32_t vMCUCount = height / 8;
	uint32_t mcuIndex = 0;

	for(uint32_t mcuY = 0; mcuY < vMCUCount; mcuY++) {
		for(uint32_t mcuX = 0; mcuX < hMCUCount; mcuX++) {
			
			uint8_t* destY = outBuf + (mcuIndex * 64);

			for(uint32_t row = 0; row < 8; row++) {
				uint32_t globalY = (mcuY * 8) + row;
				uint32_t globalX = (mcuX * 8);
				uint32_t inOffset = (globalY * width) + globalX;
				
				for(uint32_t col = 0; col < 8; col++) {
					*destY++ = inBuf[inOffset + col];
				}
			}
			mcuIndex++;
		}
	}

	output.width = width;
	output.height = height;
	output.format = PixelFormat::Grayscale;
	output.payloadSize = mcuIndex * 64;

	return Status::Ok;
}