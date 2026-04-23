/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/visionTypes.hpp
 * Author:  NotBlackMagic
 * Brief:   
 */

#pragma once

#include <stdint.h>

// Pixel Formats
// YUV 4:4:4: No chrominance sub-sampling keeps full information for all Y, Cb and Cr components.
//    Luminance Y      Chrominance (Cb and Cr)
// | A | B | C | D |     | A | B | C | D |
// -----------------  +  -----------------
// | E | F | G | H |     | A | B | C | D |
// YUV 4:2:2: Cb and Cr are horizontally sampled at the half compared to the Y component (keeping only the chrominance information of one pixel over two horizontally adjacent pixels).
//    Luminance Y      Chrominance (Cb and Cr)
// | A | B | C | D |     |   A   |   C   |
// -----------------  +  -----------------
// | E | F | G | H |     |   E   |   G   |
// YUV 4:2:0: b and Cr are horizontally and vertically sampled at the half compared to the Y component (keeping only the chrominance information of one pixel over four adjacent pixels).
//    Luminance Y      Chrominance (Cb and Cr)
// | A | B | C | D |     |       |       |
// -----------------  +  |   A   |   C   |
// | E | F | G | H |     |       |       |

/// @brief Camera pixel format.
enum class PixelFormat : uint8_t {
	// RGB Formats
	RGB565,
	RGB888,
	// 
	YUV444,			// No subsampling
	YUV422_YUYV,	// Standard Interleaved (YUYV)
	YUV422_YVYU,	// Interleaved Swapped Chroma (YVYU)
	YUV422_UYVY,	// 
	YUV420_NV12,	// Semi-planar (Required for VENC/H.264)
	YUV420_NV21,	// Semi-planar swapped
	// Raw formats
	BayerRaw8,
	BayerRaw10,
	// Fallback for compressed buffers
	Grayscale,
	Unknown
};

/// @brief Compression standard applied to the vision data.
enum class VisionCodec : uint8_t {
	None,	// Uncompressed raw data
	Jpeg,	// MJPEG stream or single static JPEG snapshot
	H264	// Encoded H.264 stream
};

/// @brief Universal frame token carrying hardware buffers and metadata through the pipeline.
struct VisionFrame {
	uint8_t* startAddress;
	uint32_t allocatedSize;	// Total buffer capacity (for memory pool management)
	uint32_t payloadSize;	// Actual bytes used (critical for Jpeg/H264 out and USB TX)

	uint16_t width;
	uint16_t height;
	uint16_t stride;		// Bytes per row in memory including hardware alignment padding

	PixelFormat format;		// The raw layout (if codec == None)
	VisionCodec codec;		// The compression applied (if format == Unknown)

	uint64_t timestampUs;	// Presentation timestamp for A/V sync and container muxing

	bool isStream;			// False for static pictures (snapshots), True for continuous video
	bool isKeyframe;		// True for static JPEGs, MJPEG frames, and H.264 I-Frames
};

/// @brief Defines a supported camera resolution and framerate for USB/Host negotiation.
struct FrameFormat {
	uint16_t width;
	uint16_t height;
	uint32_t frameInterval;	// 100ns units (e.g., 333333 for 30fps)
	PixelFormat format; 
	VisionCodec codec;		// Determines which UVC Format Descriptor to generate
};