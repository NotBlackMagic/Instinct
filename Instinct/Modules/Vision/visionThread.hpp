/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/visionThread.hpp
 * Author:  NotBlackMagic
 * Brief:   
 */

#pragma once

#include "string.h"

#include "hardware.hpp"

#include "cameraDCMI.hpp"
#include "cameraMIPI.hpp"
#include "imageProcessor.hpp"
#include "imageWriter.hpp"
#include "jpegEncoder.hpp"
#include "logger.hpp"
#include "patternGenerator.hpp"
#include "storageThread.hpp"
#include "usbClassUVC.hpp"
#include "usbDevice.hpp"

#include "tx_api.h"
#include "fx_api.h"

#define USE_LINE_MODE						1	// 1 = Hardware line mode (64 lines), 0 = Full-frame ping-pong

class VisionThread {
	public:
		/// @brief Aggregated performance and hardware statistics for the vision pipeline
		struct Statistics {
			// Rate & Counters
			float fps = 0.0f;
			uint32_t totalFrames = 0;
			uint32_t uvcDroppedCount = 0;
			uint32_t pipelineRestarts = 0;

			// Frame Latencies (microseconds)
			uint32_t lastCapTimeUs = 0;
			uint32_t avgCapTimeUs = 0;
			uint32_t lastEncTimeUs = 0;
			uint32_t avgEncTimeUs = 0;
			uint32_t lastIOTimeUs = 0;
			uint32_t avgIOTimeUs = 0;

#if USE_LINE_MODE
			uint32_t lastSliceCapUs = 0;
			uint32_t avgSliceCapUs = 0;
			uint32_t lastSliceEncUs = 0;
			uint32_t avgSliceEncUs = 0;
#endif

			// Encoding Throughput
			uint32_t lastFrameBytes = 0;
			uint32_t bitrateBPS = 0;
			float compressionRatio = 0.0f;
			float avgCompressionRatio = 0.0f;

			// Ingress & Encoder Driver Stats
			Csi::Statistics csi;
			Dcmipp::Statistics dcmipp;
			Venc::Statistics venc;
		};
		
		static void Init();

		// Thread-safe telemetry snapshot accessors
		static void GetStatistics(Statistics& out);
		static void ResetStatistics();

		// Array of all supported controls
		static USBClassUVC::UVCControlState puControlRegistry[];
		static const uint8_t numPUControls;

		static USBClassUVC::UVCControlState itControlRegistry[];
		static const uint8_t numITControls;
	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[8196];

		// Telemetry synchronization
		static TX_MUTEX mutex;
		static Statistics stats;
		static uint32_t pipelineRestartCounter;

		static void Run(ULONG input);
		static void OnUVCStreamState(USBClassUVC::StreamEvent event);

		// Static callbacks to wrap your I2C hardware calls
		static void ExposureTimeControl(int16_t value);
		static void BrightnessControl(int16_t value);
		static void AutoWhiteBalanceControl(int16_t value);
		static void WhiteBalanceTemperatureControl(int16_t value);
};