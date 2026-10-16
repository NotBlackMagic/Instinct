/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	SDK/Drivers/Video/ov9281.hpp
 * Author:	NotBlackMagic
 * Brief:	OV9281 Camera driver class for STM32N6.
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
		static constexpr uint8_t i2cAddrPrimary = 0x60;
		static constexpr uint8_t i2cAddrSecondary = 0x70;
		static constexpr uint16_t chipID = 0x9281;

		/// @brief Device register map.
		enum class Register : uint16_t {
			// System control
			MODE_SELECT = 0x0100,
			SC_SOFTWARE_RESET = 0x0103,

			// Chip identification
			SC_CTRL_06 = 0x3006,
			SC_CHIP_ID_H = 0x300A,
			SC_CHIP_ID_L = 0x300B,
			SC_MIPI_SC_CTRL0 = 0x3014,

			// SCCB / system control
			SC_CTRL_2B = 0x302B,
			GROUP_ACCESS = 0x3208,

			// Exposure
			EXPO_H = 0x3500,
			EXPO_M = 0x3501,
			EXPO_L = 0x3502,
			AEC = 0x3503,

			// Analogue gain
			GAIN_SHIFT = 0x3507,
			DEBUG = 0x3508,
			GAIN = 0x3509,

			// Sensor timing / window
			X_START_H = 0x3800,
			X_START_L = 0x3801,
			Y_START_H = 0x3802,
			Y_START_L = 0x3803,
			X_END_H = 0x3804,
			X_END_L = 0x3805,
			Y_END_H = 0x3806,
			Y_END_L = 0x3807,
			X_OUTPUT_H = 0x3808,
			X_OUTPUT_L = 0x3809,
			Y_OUTPUT_H = 0x380A,
			Y_OUTPUT_L = 0x380B,
			HTS_H = 0x380C,
			HTS_L = 0x380D,
			VTS_H = 0x380E,
			VTS_L = 0x380F,
			X_WIN_H = 0x3810,
			X_WIN_L = 0x3811,
			Y_WIN_H = 0x3812,
			Y_WIN_L = 0x3813,
			X_INC = 0x3814,
			Y_INC = 0x3815,

			// Orientation / binning
			TIMING_FORMAT1 = 0x3820,
			TIMING_FORMAT2 = 0x3821,

			// Frame synchronization / trigger
			TIMING_REG23 = 0x3823,

			// DVP / output format
			DVP_MODE = 0x4317,
			POLARITY_CTRL = 0x4708,

			// Test pattern
			PRE_CTRL00 = 0x5E00,
		};

		/// @brief Register value pair, for initialization blocks.
		struct RegisterValuePair {
			OV9281::Register reg;
			uint8_t value;
		};

		/// @brief Frame capture synchronization mode.
		enum class TriggerMode : uint8_t {
			Continuous,		///< Free-running
			External		///< Frame synchronized to external trigger
		};

		/// @brief OV9281 camera configuration structure.
		struct Config {
			const char* deviceName;		///< Free text name e.g. "OV9281".

			// Image/frame geometry
			uint16_t width;				///< Image width in pixels.
			uint16_t height;			///< Image height in pixels.
			PixelFormat format;			///< Pixel format.
			uint32_t fps;				///< Frame rate.
			TriggerMode triggerMode;	///< Free running or hardware synced.

			GPIO* shutdownPin;			///< Optional shutdown pin object.
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

		/// @brief Sets exposure in whole sensor row periods.
		/// @param exposure Exposure time in sensor row periods.
		/// @return Status::Ok if set succeeded.
		Status SetExposure(uint32_t exposure);

		/// @brief Sets analogue gain using Q12.4 fixed-point representation.
		/// @param gain Gain in Q12.4 format i.e. 0x10 = 1.0x and 0x28 = 2.5x.
		/// @return Status::Ok if set succeeded.
		Status SetAnalogGain(uint16_t gain);

		/// @brief Sets frame orientation/mirroring.
		/// @param mirrorH	If truer mirror frame along the horizontal axis.
		/// @param flipV	If truer mirror frame along the vertical axis.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetOrientation(bool mirrorH, bool flipV);

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
			WXGA, HD, VGA, P256, UNKNOWN
		};

		InternalResolution FindNearestResolution(uint16_t w, uint16_t h);
		Status ApplyResolution(InternalResolution resolution);
		Status ApplyWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

		Status WriteRegisterArray(const RegisterValuePair* blob, uint16_t size);;
		Status WriteRegister(Register reg, uint8_t value);
		Status ReadRegister(Register reg, uint8_t& value);
		Status ModifyRegister(Register reg, uint8_t mask, uint8_t value);
};
