/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/cameraDCMI.hpp
 * Author:  NotBlackMagic
 * Brief:   
 */

#pragma once

#include <stdint.h>

#include "hardware.hpp"

#include "logger.hpp"

#include "dmaChannel.hpp"
#include "dcmi.hpp"

#include "ov7670.hpp"

#include "visionTypes.hpp"

#include "status.hpp"

class CameraDCMI {
	public:
		/// @brief Configuration structure for the SD/DCMI Camera Pipeline.
		struct Config {
			uint16_t width;			///< Image width in pixels.
			uint16_t height;		///< Image height in pixels.
			PixelFormat format;		///< Pixel format.
			uint32_t fps;			///< Frame rate.
		};

		// Delete copy constructors
		CameraDCMI(const CameraDCMI&) = delete;
		CameraDCMI& operator=(const CameraDCMI&) = delete;

		/// @brief Constructor.
		/// @param dcmi Reference to the low-level bus driver.
		/// @param sensor Reference to camera sensor driver. 
		CameraDCMI(Dcmi &dcmi, OV7670 &sensor, DMAChannel &dma) : dcmiInterface(dcmi), sensor(sensor), dmaChannel(dma) {};

		/// @brief Initializes the sensor, DCMI peripheral, and DMA pipelines.
		/// @param config The requested resolution and format settings.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid or failed.
		Status Init(const Config &config);

		/// @brief Starts a non-blocking single frame grab.
		/// @param buffer Frame buffer structure.
		/// @return Status::Ok if the transfer started, or Status::Busy if device is locked by another thread, or Status::Error if failed.
		Status CaptureAsync(VisionFrame &buffer);

		/// @brief Blocks the current thread until the Async capture completes.
		/// @param timeoutTicks Max wait time in OS ticks.
		/// @return Status::Ok if the transfer completed successfully, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status CaptureWait(uint32_t timeoutTicks);

		/// @brief Aborts a ongoing capture.
		/// @return Status::Ok
		Status CaptureAbort();

		/// @brief Expose a reference to the sensor
		/// @return Camera instance.
		OV7670& GetSensor() { return sensor; }

	private:
		Dcmi& dcmiInterface;
		OV7670& sensor;
		DMAChannel& dmaChannel;

		// Capture context
		VisionFrame* frame;
};