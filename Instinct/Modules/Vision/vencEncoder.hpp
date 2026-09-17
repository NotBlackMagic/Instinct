/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Vision/VENCEncoder.hpp
 * Author:	NotBlackMagic
 * Brief:	H.264 video stream encoder.
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

		/// Forward VENC configuration to underlying instance
		Status SetBitrate(uint32_t bitPerSecond) { return this->vencInstance.SetBitrate(bitPerSecond); }
		Status SetFixedQP(uint32_t qp) { return this->vencInstance.SetQPBounds(0, 51, static_cast<int32_t>(qp)); }
		Status SetAutoQP(uint32_t minQp = 10, uint32_t maxQp = 48) { return this->vencInstance.SetQPBounds(minQp, maxQp, -1); }
		Status SetGOPLength(uint32_t gopLen) { return this->vencInstance.SetGOPLength(gopLen); }
		Status InjectTelemetry(const uint8_t* telemetryPayload, uint32_t sizeBytes) { return this->vencInstance.SetSEIUserData(telemetryPayload, sizeBytes); }
		Status SetROI(uint32_t top, uint32_t left, uint32_t bottom, uint32_t right, int32_t delta = -8) { return this->vencInstance.SetROI(top, left, bottom, right, delta); }

		/// @brief Structural reconfiguration requiring SPS/PPS regeneration (called by pipeline coordinator)
		Status ReconfigureStreamParameters(uint32_t sliceSize, uint32_t gdrDuration, bool denoise, VisionFrame& headerFrame);

		/// @brief Retrieve active configuration snapshot without touching low-level driver.
		Venc::Config GetConfig() { return this->vencInstance.GetConfig(); }

		/// @brief Retrieve hardware-level timing and byte statistics from the VENC driver.
		Venc::Statistics GetStatistics() const { return vencInstance.GetStatistics(); }

	private:
		Venc& vencInstance;
		VisionCodec codec;

		// Capture context
		VisionFrame* outFrameContext;
		const VisionFrame* inFrameContext;
		bool streamingMode;
		uint32_t lineBufferLines;

		TX_EVENT_FLAGS_GROUP syncEvent;
		static constexpr uint32_t EVT_DONE = 0x01;
		static constexpr uint32_t EVT_ERR = 0x02;

		static void EventCallback(void* ctx, Venc::Event evt);
};