/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/VENCEncoder.hpp
 * Author:  NotBlackMagic
 * Brief:   H.264 video stream encoder.
 */

#pragma once

#include <stdint.h>

#include "hardware.hpp"

#include "venc.hpp"
#include "logger.hpp"
#include "status.hpp"
#include "system.hpp"
#include "visionTypes.hpp"

#include "tx_api.h"

class VENCEncoder {
    public:
		/// @brief Constructor.
		/// @param venc		Reference to the low-level VENC driver
		VENCEncoder(Venc& venc) : vencInstance(venc), outFrameContext(nullptr), inFrameContext(nullptr) {};

		/// @brief Initializes the VENC.
		/// @param config VENC hardware configuration and memory pool pointers (passed to VENC driver).
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid or failed.
		Status Init(Venc::Config& config);

		/// @brief Generates the initial Sequence/Picture Parameter Sets (SPS/PPS).
		/// @param outputFrame Output frame buffer structure.
		/// @return Status::Ok if the encoding started successfully, or Status::Error if failed.
		Status EncodeStart(VisionFrame& outputFrame);
		
		/// @brief Pushes a VisionFrame to the encoder.
		/// @param inputFrame		Input frame buffer structure.
		/// @param outputFrame		Output frame buffer structure.
		/// @param requestKeyframe	AAaa
		/// @return Status::Ok if encoding frame successfully, or Status::Error if failed.
		Status EncodeAsync(const VisionFrame& inputFrame, VisionFrame& outputFrame, bool requestKeyframe = false);

		/// @brief Blocks until the encoder hardware completes the current frame.
		/// @param timeoutTicks Max wait time in OS ticks.
		/// @return Status::Ok if the transfer completed successfully, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status EncodeWait(uint32_t timeoutTicks);
		
		/// @brief Stops the hardware and generates the EOS marker.
		/// @param outputFrame	Output frame buffer structure.
		/// @return Status::Ok if the stopped and finished frame encoding successfully, or Status::Error if failed.
		Status EncodeStop(VisionFrame& outputFrame);

		/// @brief Hard resets the peripheral.
		/// @return Status::Ok
		Status EncodeAbort();

	private:
		Venc& vencInstance;
		VisionCodec codec;

		// Capture context
		VisionFrame* outFrameContext;
		const VisionFrame* inFrameContext;

		TX_EVENT_FLAGS_GROUP syncEvent;
		static constexpr uint32_t EVT_DONE = 0x01;
		static constexpr uint32_t EVT_ERR = 0x02;

		static void EventCallback(void* ctx, Venc::Event evt);
};