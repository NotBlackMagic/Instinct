/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Vision/cameraMIPI.hpp
 * Author:	NotBlackMagic
 * Brief:	Helper class to acquire frames over MIPI-CSI and DCMIPP. Wraps camera sensor driver, CSI and DCMIPP under one master class.
 */

#pragma once

#include <stdint.h>

#include "hardware.hpp"

#include "logger.hpp"

#include "csi.hpp"
#include "dcmipp.hpp"
#include "dmaChannel.hpp"

#include "ov5645.hpp"

#include "visionTypes.hpp"

#include "status.hpp"

class CameraMIPI {
	public:
		/// @brief Camera pipeline and throughput statistics.
		struct Statistics {
			uint32_t frames = 0;
			uint32_t drops = 0;
			uint32_t errors = 0;
			float fps = 0.0f;
		};

		/// @brief Configuration structure for the HD/MIPI-CSI Camera Sensor.
		struct Config {
			uint16_t width;			///< Sensor output width in pixels.
			uint16_t height;		///< Sensor output height in pixels.
			PixelFormat format;		///< Default Main pipe pixel format.
			uint32_t fps;			///< Sensor frame rate.
		};

		/// @brief Per-pipe output stream configuration.
		struct StreamConfig {
			uint16_t width = 0;			///< Output width in pixels (0 = full active input width).
			uint16_t height = 0;		///< Output height in pixels (0 = full active input height).
			PixelFormat format = PixelFormat::YUV422_YUYV;
			Dcmipp::FrameRateDecimation frameRate = Dcmipp::FrameRateDecimation::All;
			uint16_t cropX = 0;			///< Optional crop X offset in pixels.
			uint16_t cropY = 0;			///< Optional crop Y offset in pixels.
			uint16_t cropWidth = 0;		///< Optional crop width in pixels (0 = no crop).
			uint16_t cropHeight = 0;	///< Optional crop height in pixels (0 = no crop).
		};

		// Delete copy constructors
		CameraMIPI(const CameraMIPI&) = delete;
		CameraMIPI& operator=(const CameraMIPI&) = delete;

		/// @brief Constructor.
		/// @param csi		Reference to the low-level bus driver.
		/// @param sensor	Reference to camera sensor driver. 
		CameraMIPI(Csi &csi, Dcmipp &dcmipp, OV5645 &sensor, DMAChannel &dma) : csiInterface(csi), dcmiPPInterface(dcmipp), sensor(sensor), dmaChannel(dma) {};

		/// @brief Initializes the sensor, CSI peripheral, DCMI-PP peripheral, and DMA pipelines.
		/// @param config The requested resolution and format settings.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid or failed.
		Status Init(const Config &config);

		/// @brief Configures resolution, format, cropping, and frame rate decimation for a specific DCMIPP pipe.
		/// @param pipe		Aaa
		/// @param config	aA
		/// @return Status::Ok
		Status ConfigurePipe(Dcmipp::PipeID pipe, const StreamConfig &config);

		/// @brief Starts a non-blocking single frame grab (Snapshot) on the specified pipe.
		/// @param pipe		DCMIPP Pipe to capture from.
		/// @param buffer	Frame buffer structure.
		/// @return Status::Ok if the transfer started, or Status::Busy if device is locked by another thread, or Status::Error if failed.
		Status CaptureAsync(Dcmipp::PipeID pipe, VisionFrame &buffer);

		/// @brief Starts continuous double-buffered capture on the specified pipe.
		/// @param pipe		DCMIPP Pipe to capture from.
		/// @param buffer0 Frame buffer A structure (double buffer).
		/// @param buffer1 Frame buffer B structure (double buffer).
		/// @return Status::Ok if the transfer started, or Status::Busy if device is locked by another thread, or Status::Error if failed.
		Status CaptureAsync(Dcmipp::PipeID pipe, VisionFrame &buffer0, VisionFrame &buffer1);

		/// @brief Starts continuous line-wrapping capture on the specified pipe (ONLY for Pipe 1 or Pipe 2).
		/// @param pipe			DCMIPP Pipe to capture from.
		/// @param buffer		Line buffer structure.
		/// @param lineMult		Aaa
		/// @param wrapAddress	Aaa
		/// @return Status::Ok if the transfer started, or Status::Busy if device is locked by another thread, or Status::Error if failed.
		Status CaptureAsync(Dcmipp::PipeID pipe, VisionFrame &buffer, Dcmipp::LineCount lineMult, Dcmipp::LineCount wrapAddress);

		/// @brief Blocks until the next frame completes on the specified pipe, handling cache invalidation automatically.
		/// @param timeout Max wait time in OS ticks.
		/// @return Status::Ok if the transfer completed successfully, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status CaptureWait(Dcmipp::PipeID pipe, uint32_t timeout);

		/// @brief Aborts an ongoing capture on the specified pipe.
		/// @return Status::Ok
		Status CaptureAbort(Dcmipp::PipeID pipe);

		/// @brief Expose a reference to the sensor
		/// @return Camera instance.
		OV5645& GetSensor() { return sensor; }

		/// @brief Retrieve hardware-level timing statistics for a specific pipe.
		Dcmipp::Statistics GetPipeStatistics(Dcmipp::PipeID pipe) const { return dcmiPPInterface.GetPipeStatistics(pipe); }

		/// @brief Retrieve hardware-level timing statistics for the CSI interface.
		Csi::Statistics GetCSIStatistics() const { return csiInterface.GetStatistics(); }

		/// @brief Returns pipeline performance statistics.
		Statistics GetStatistics(Dcmipp::PipeID pipe = Dcmipp::PipeID::Main) const { return this->pipeCtx[static_cast<uint8_t>(pipe)].stats; }

		/// @brief Resets pipeline statistics counters.
		void ResetStatistics(Dcmipp::PipeID pipe = Dcmipp::PipeID::Main) {
			uint8_t idx = static_cast<uint8_t>(pipe);
			this->pipeCtx[idx].stats = {};
			this->pipeCtx[idx].timestamp = 0;
		}

	private:
		Csi& csiInterface;
		Dcmipp& dcmiPPInterface;
		OV5645& sensor;
		DMAChannel& dmaChannel;

		Config sensorConfig = {};
		Csi::MIPIDataType mipiDataType = Csi::MIPIDataType::YUV422_8bit;
		uint8_t activePipeMask = 0;

		// Capture context
		enum class Mode { Idle, Snapshot, Continuous, Partial };

		struct PipeContext {
			Mode mode = Mode::Idle;
			uint8_t currentFrameIdx = 0;
			VisionFrame* activeFrames[2] = {nullptr, nullptr};
			Statistics stats = {};
			uint32_t timestamp = 0;
		};
		PipeContext pipeCtx[3];

		Status StartSensor(Dcmipp::PipeID pipe);
		void StopSensor(Dcmipp::PipeID pipe);
};