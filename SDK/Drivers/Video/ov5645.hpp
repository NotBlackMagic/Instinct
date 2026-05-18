/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Video/ov5645.hpp
 * Author:  NotBlackMagic
 * Brief:   OV5645 Camera driver class for STM32N6.
 */

#pragma once

#include <stdint.h>

#include "gpio.hpp"
#include "i3c.hpp"
#include "system.hpp"

#include "status.hpp"
#include "visionTypes.hpp"

class OV5645 {
	public:
		// Standard Chip identifications.
		static constexpr uint8_t i2cAddrPrimary = 0x3C;
		static constexpr uint16_t chipID = 0x5645;

		/// @brief Device register map.
		enum class Register : uint16_t {
			SYSTEM_CTRL0 = 0x3008,		// 
			CHIP_ID_HIGH = 0x300A,		// Product ID Number MSB (Read only = 0x56)
			CHIP_ID_LOW  = 0x300B,		// Product ID Number LSB (Read only = 0x45)
			IO_MIPI_CTRL00 = 0x300E,	// IO MIPI Control 00
			PAD_OUT_VAL00 = 0x3019,		// Pad Output Value
			FORMAT_CTRL00 = 0x4300,		// Format Control 00
			MIPI_CTRL00 = 0x4800,		// MIPI Control 00
			FORMAT_MUX_CTRL = 0x501f	// Format MUX Control
		};

		/// @brief Regsiter value pair, for initialization blocks.
		struct RegisterValuePair {
			OV5645::Register reg;
			uint8_t value;
		};

		/// @brief Supported banding filters for AC light flicker.
		enum class BandingFilter : uint8_t {
			Off,
			Hz50,
			Hz60,
			Auto
		};

		/// @brief Automatic Gain Control (AGC) maximum ceiling limits.
		enum class GainCeiling : uint8_t {
			x2 = 0x00,
			x4 = 0x01,
			x8 = 0x02,
			x16 = 0x03,
			x32 = 0x04,
			x64 = 0x05,
			x128 = 0x06
		};

		/// @brief OV5645 camera configuration structure.
		struct Config {
			const char* deviceName;		///< Free text name e.g. "OV5645".

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
		OV5645(const OV5645&) = delete;
		OV5645& operator=(const OV5645&) = delete;

		/// @brief Constructor.
		/// @param i3c	Reference to the low-level bus driver.
		/// @param addr	Bus address.
		OV5645(I3C &i3c, uint8_t addr) : bus(i3c), addr(addr) {}

		/// @brief Initializes the OV5645 camera.
		/// @param config OV5645 camera configuration.
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
		/// @param width  Target frame width.
		/// @param height Target frame height.
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

		/// @brief Sets the test pattern mode.
		/// @param enable If true test pattern is enabled.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetTestPattern(bool enable);

	private:
		I3C& bus;
		const uint8_t addr;
		Config config;

		SensorInfo sensorInfo;

		static constexpr uint16_t transferSize = 32;
		__attribute__((aligned(32))) uint8_t buffer[transferSize];

		/// @brief Implemented frame resolutions.
		enum class InternalResolution {
			QSXGA, FHD, SXGA, VGA, UNKNOWN
		};

		InternalResolution FindNearestResolution(uint16_t w, uint16_t h);
		Status ApplyResolution(InternalResolution resolution);
		Status ApplyWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

		Status WriteRegisterArray(const RegisterValuePair* blob, uint16_t size);
		Status WriteRegister(Register reg, uint8_t value);
		Status ReadRegister(Register reg, uint8_t& value);
		Status ModifyRegister(Register reg, uint8_t mask, uint8_t value);
};