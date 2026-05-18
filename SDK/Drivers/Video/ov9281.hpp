/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Video/ov9281.hpp
 * Author:  NotBlackMagic
 * Brief:   OV9281 Camera driver class for STM32N6.
 */

#pragma once

#include <stdint.h>

#include "gpio.hpp"
#include "i3c.hpp"
#include "system.hpp"

#include "status.hpp"
#include "visionTypes.hpp"

class OV9281 {
	public:
		// Standard Chip identifications.
		static constexpr uint8_t i2cAddrPrimary = 0x21;
		static constexpr uint16_t chipID = 0x7673;

		/// @brief Device register map.
		enum class Register : uint8_t {
		};

		/// @brief OV9281 camera configuration structure.
		struct Config {
			const char* deviceName;		///< Free text name e.g. "OV9281".

			// Image/frame geometry
			uint16_t width;				///< Image width in pixels.
			uint16_t height;			///< Image height in pixels.
			PixelFormat format;			///< Pixel format.
			uint32_t fps;				///< Frame rate.

			GPIO* resetPin;				///< Optional reset pin object.
			GPIO* powerDownPin;			///< Optional power down pin object.
		};

		/// @brief Camera sensor information structure.
		struct SensorInfo {
			uint16_t id;
			uint16_t width;
			uint16_t height;
		};

		// Delete copy constructors
		OV9281(const OV9281&) = delete;
		OV9281& operator=(const OV9281&) = delete;

		/// @brief Constructor.
		/// @param i3c	Reference to the low-level bus driver.
		/// @param addr	Bus address.
		OV9281(I3C &i3c, uint8_t addr) : bus(i3c), addr(addr) {}

		/// @brief Initializes the OV9281 camera.
		/// @param config OV9281 camera configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid or failed.
		Status Init(const Config &config);

		/// @brief Resets the camera device, using either software reset or hardware reset (if present).
		/// @return Status::Ok if reset succeeded cleanly.
		Status Reset();

		/// @brief Starts the camera device, starting to stream frames from the camera. Using either software wakeup or hardware power up (if present).
		/// @return Status::Ok if started successfully.
		Status Start();

		/// @brief Stops the camera device, stopping to stream frames from the camera. Using either software sleep or hardware power down (if present).
		/// @return Status::Ok if started successfully.
		Status Stop();

		/// @brief Reads the Manufacturer and Device IDs.
		/// @param id Pointer to be filled with manufacturer ID (Product ID).
		/// @return Status::Ok if read succeeded, or Status::Error if failed.
		Status ReadID(uint16_t *id);

		/// @brief Sets the desired frame resolution, approximated to closed (lower) valid resolution.
		/// @param width	Target frame width.
		/// @param height	Target frame height.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetResolution(uint16_t width, uint16_t height);

		/// @brief Sets the desired pixel format.
		/// @param format Pixel format to use.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetFormat(PixelFormat format);

		/// @brief Sets the desired frame rate, approximated to closed (lower) valid frame rate.
		/// @param fps Target frame rate.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetFPS(uint32_t fps);

		/// @brief Gets the basic camera sensor information.
		/// @return Camera sensor information.
		SensorInfo GetInfo() { return sensorInfo; }

	private:
		I3C& bus;
		const uint8_t addr;
		Config config;

		SensorInfo sensorInfo;

		static constexpr uint16_t transferSize = 32;
		__attribute__((aligned(32))) uint8_t buffer[transferSize];

		/// @brief Implemented frame resolutions.
		enum class InternalResolution {
			VGA, QVGA, QQVGA, QQQVGA, CIF, QCIF, QQCIF, UNKNOWN
		};

		struct RegisterValuePair {
			OV9281::Register reg;
			uint8_t value;
		};

		InternalResolution FindNearestResolution(uint16_t w, uint16_t h);
		Status ApplyResolution(InternalResolution resolution);
		Status ApplyWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
		Status ApplyAWBTuning();

		Status WriteRegister(Register reg, uint8_t value);
		Status ReadRegister(Register reg, uint8_t& value);
		Status ModifyRegister(Register reg, uint8_t mask, uint8_t value);
};
