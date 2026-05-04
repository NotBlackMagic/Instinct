/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Video/ov7670.hpp
 * Author:  NotBlackMagic
 * Brief:   OV7670 Camera driver class for STM32N6.
 */

#pragma once

#include <stdint.h>

#include "gpio.hpp"
#include "i3c.hpp"
#include "system.hpp"

#include "status.hpp"
#include "visionTypes.hpp"

class OV7670 {
	public:
		// Standard Chip identifications.
		static constexpr uint8_t i2cAddrPrimary = 0x21;
		static constexpr uint16_t chipID = 0x7673;

		/// @brief Device register map.
		enum class Register : uint8_t {
			GAIN = 0x00,	// Gain control gain setting
			BLUE = 0x01,	// Blue channel gain setting
			RED = 0x02,		// Red channel gain setting
			VREF = 0x03,	// Vertical Frame Control
			COM1 = 0x04,	// Common Control 1 (AEC LSBs)
			AECHH = 0x07,	// AEC high 6 bits
			COM2 = 0x09,	// Common Control 2
			PID = 0x0A,		// Product ID Number MSB (Read only = 0x76)
			VER = 0x0B,		// Product ID Number LSB (Read only = 0x73)
			COM3 = 0x0C,	// Common Control 3
			AECH = 0x10,	// AEC middle 8 bits
			CLKRC = 0x11,	// Internal Clock
			COM7 = 0x12,	// Common Control 7
			COM8 = 0x13,	// Common Control 8 (AEC, AGC, AWB)
			COM9 = 0x14,	// Common Control 9 (AGC Ceiling)
			COM10 = 0x15,	// Common Control 10 (PCLK, HREF, VSYNC)
			HSTART = 0x17,	// Output Format - Horizontal Frame (HREF column) start high 8-bit (low 3 bits are at HREF[2:0])
			HSTOP = 0x18,	// Output Format - Horizontal Frame (HREF column) end high 8-bit (low 3 bits are at HREF[5:3])
			VSTART = 0x19,	// Output Format - Vertical Frame (row) start high 8-bit (low 2 bits are at VREF[1:0])
			VSTOP = 0x1A,	// Output Format - Vertical Frame (row) end high 8-bit (low 2 bits are at VREF[3:2])
			MVFP = 0x1E,	// Mirror/VFlip Enable
			HREF = 0x32,	// HREF Control
			TSLB = 0x3A,	// Line Buffer Test Option
			COM11 = 0x3B,	// Common Control 11 (Banding filter)
			COM13 = 0x3D,	// Common Control 13 (UV swap)
			COM14 = 0x3E,	// Common Control 14
			COM15 = 0x40,	// Common Control 15
			COM16 = 0x41,	// Common Control 16
			COM17 = 0x42,	// Common Control 17
			AWBC1 = 0x43,	// AWB Control 1
			AWBC2 = 0x44,	// AWB Control 2
			AWBC3 = 0x45,	// AWB Control 3
			AWBC4 = 0x46,	// AWB Control 4
			AWBC5 = 0x47,	// AWB Control 5
			AWBC6 = 0x48,	// AWB Control 6
			BRIGHT = 0x55,	// Brightness Control
			CONTRAS = 0x56,	// Contrast Control
			AWBC7 = 0x59,	// AWB Control 7
			AWBC8 = 0x5A,	// AWB Control 8
			AWBC9 = 0x5B,	// AWB Control 9
			AWBC10 = 0x5C,	// AWB Control 10
			AWBC11 = 0x5D,	// AWB Control 11
			AWBC12 = 0x5E,	// AWB Control 12
			DBLV = 0x6B,	//
			AWBCTR3 = 0x6C,	// AWB Control 3
			AWBCTR2 = 0x6D,	// AWB Control 2
			AWBCTR1 = 0x6E,	// AWB Control 1
			AWBCTR0 = 0x6F,	// AWB Control 0
			SCALING_XSC = 0x70,			//
			SCALING_YSC = 0x71,			//
			SCALING_DCWCTR = 0x72,		// DCW Control
			SCALING_PCLK_DIV = 0x73,	//
			RGB444 = 0x8C,
			SCALING_PCLK_DELAY = 0xA2,	// Pixel Clock Delay
			RSVD = 0xB0,	// Magic color register...
			ABLC1 = 0xB1	// ABLC control
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
			x4 = 0x10,
			x8 = 0x20,
			x16 = 0x30,
			x32 = 0x40,
			x64 = 0x50,
			x128 = 0x60
		};

		/// @brief OV7670 camera configuration structure.
		struct Config {
			const char* deviceName;		///< Free text name e.g. "OV7670".

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
		OV7670(const OV7670&) = delete;
		OV7670& operator=(const OV7670&) = delete;

		/// @brief Constructor.
		/// @param i3c	Reference to the low-level bus driver.
		/// @param addr	Bus address.
		OV7670(I3C &i3c, uint8_t addr) : bus(i3c), addr(addr) {}

		/// @brief Initializes the OV7670 camera.
		/// @param config OV7670 camera configuration.
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

		/// @brief Sets the desired brightness level.
		/// @param value Brightness level to use, signed value (default is 0x00).
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetBrightness(int8_t value);

		/// @brief Sets the desired contrast level.
		/// @param value Contrast level to use (default is 0x40).
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetContrast(uint8_t value);

		/// @brief Sets the desired white balance, when in manual mode.
		/// @param redGain	Red channel gain.
		/// @param blueGain	Blue channel gain.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetWhiteBalance(uint8_t redGain, uint8_t blueGain);

		/// @brief Disables auto exposure and sets a manual exposure time.
		/// @param exposure 16-bit exposure value (higher = longer exposure).
		/// @return Status::Ok if set succeeded.
		Status SetManualExposure(uint16_t exposure);

		/// @brief Sets the maximum limit the Auto Gain Control (AGC) can reach.
		/// @param ceiling The maximum multiplier.
		/// @return Status::Ok if set succeeded.
		Status SetMaxGain(GainCeiling ceiling);

		/// @brief Sets the auto exposure mode.
		/// @param enable If true auto exposure is enabled.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetAutoExposure(bool enable);

		/// @brief Sets the auto white balance mode.
		/// @param enable If true white balance is enabled.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetAutoWhiteBalance(bool enable);

		/// @brief Sets the anti-banding filter for indoor lighting.
		/// @param filter The AC frequency of the environment.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetBandingFilter(BandingFilter filter);

		/// @brief Sets frame orientation/mirroring.
		/// @param mirrorH If truer mirror frame along the horizontal axis.
		/// @param flipV   If truer mirror frame along the vertical axis.
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
			VGA, QVGA, QQVGA, QQQVGA, CIF, QCIF, QQCIF, UNKNOWN
		};

		struct RegisterValuePair {
			OV7670::Register reg;
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