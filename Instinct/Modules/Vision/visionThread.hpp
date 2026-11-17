/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Vision/visionThread.hpp
 * Author:	NotBlackMagic
 * Brief:	Handles MIPI-CSI (Main/HD camera) video feed.
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
			// Rates and Throughput
			float fps = 0.0f;
			uint32_t bitrateBps = 0;
			float compression = 0.0f;			///< Compression ratio: Raw size / Compressed size
			float compressionMin = 1000.0f;		///< Minimum compression ratio (worst case): Raw size / Compressed size
			uint32_t lastFrameBytes = 0;

			// Counters & Drops
			uint32_t totalFrames = 0;
			uint32_t uvcDroppedCount = 0;
			uint32_t pipelineRestarts = 0;

			// Frame Latencies
			uint32_t capTime = 0;		///< Last frame capture wait time
			uint32_t encTime = 0;		///< Last encode duration
			uint32_t maxEncTime = 0;	///< Peak encode time (VENC stall detector)
			uint32_t ioTime = 0;		///< Last SD / storage write time

#if USE_LINE_MODE
			// Line Mode Slice Metrics
			uint32_t sliceCap = 0;		///< Last slice capture duration
			uint32_t sliceEnc = 0;		///< Last slice encode tail duration
			uint32_t maxSliceEnc = 0;	///< Peak slice encode tail
#endif

			// Ingress & Encoder Driver Stats
			Csi::Statistics csi;
			Dcmipp::Statistics dcmipp;
			Venc::Statistics venc;
		};
		
		static void Init();

		// Capture & Recording controls
		static void SetCapture(bool enable);
		static bool IsCapturing();
		static void SetRecording(bool enable);
		static bool IsRecording();

		/// @brief Returns Vision performance statistics.
		static Statistics GetStatistics() { return stats; }

		/// @brief Resets Vision statistics counters.
		static void ResetStatistics() {
			stats = {};
			pipelineRestartCounter = 0;
		}

		// Array of all supported controls
		static USBClassUVC::UVCControlState puControlRegistry[];
		static const uint8_t numPUControls;

		static USBClassUVC::UVCControlState itControlRegistry[];
		static const uint8_t numITControls;
	private:
		static TX_THREAD threadPtr;
		static const uint16_t threadStackSize = 8196;
		__attribute__((aligned(32))) static uint8_t threadStack[threadStackSize];

		// Telemetry synchronization
		static Statistics stats;
		static uint32_t pipelineRestartCounter;

		// Pipeline state flags
		static volatile bool captureEnabled;
		static volatile bool recordingRequested;
		static volatile bool isRecording;

		static void Run(ULONG input);
		static void OnUVCStreamState(USBClassUVC::StreamEvent event);

		// Static callbacks to wrap your I2C hardware calls
		static void ExposureTimeControl(int16_t value);
		static void BrightnessControl(int16_t value);
		static void AutoWhiteBalanceControl(int16_t value);
		static void WhiteBalanceTemperatureControl(int16_t value);
};