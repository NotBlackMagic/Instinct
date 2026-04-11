/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Sensors/bmp581.hpp
 * Author:  NotBlackMagic
 * Brief:   BMP-581 Barometer driver class for STM32N6.
 */

#pragma once

#include <stdint.h>

#include "i2c.hpp"
#include "status.hpp"

class BMP581 {
	public:
		// Standard Chip identifications
		static constexpr uint8_t i2cAddrPrimary = 0x46;
		static constexpr uint8_t i2cAddrSecondary = 0x47;
		static constexpr uint8_t chipID = 0x50;

		enum class Register : uint8_t {
			CHIP_ID = 0x01,
			REV_ID = 0x02,
			CHIP_STATUS = 0x11,
			DRIVE_CONFIG = 0x13,
			INT_CONFIG = 0x14,
			INT_SOURCE = 0x15,
			FIFO_CONFIG = 0x16,
			FIFO_COUNT = 0x17,
			FIFO_SEL = 0x18,
			RESERVED_REG_0 =0x1C,
			TEMP_DATA_XLSB = 0x1D,
			TEMP_DATA_LSB = 0x1E,
			TEMP_DATA_MSB = 0x1F,
			PRESS_DATA_XLSB = 0x20,
			PRESS_DATA_LSB = 0x21,
			PRESS_DATA_MSB = 0x22,
			RESERVED_REG1 = 0x23,
			RESERVED_REG2 = 0x24,
			RESERVED_REG3 = 0x25,
			RESERVED_REG4 = 0x26,
			INT_STATUS = 0x27,
			STATUS = 0x28,
			FIFO_DATA = 0x29,
			NVM_ADDR = 0x2B,
			NVM_DATA_LSB = 0x2C,
			NVM_DATA_MSB = 0x2D,
			DSP_CONFIG = 0x30,
			DSP_IIR = 0x31,
			OOR_THR_P_LSB = 0x32,
			OOR_THR_P_MSB = 0x33,
			OOR_RANGE = 0x34,
			OOR_CONFIG = 0x35,
			OSR_CONFIG = 0x36,
			ODR_CONFIG = 0x37,
			OSR_EFF = 0x38,
			CMD = 0x7E
		};

		enum class OutputDataRate : uint8_t {
			Hz240 = 0x00,
			Hz219 = 0x01,
			Hz199 = 0x02,
			Hz179 = 0x03,
			Hz160 = 0x04,
			Hz149 = 0x05,
			Hz140 = 0x06,
			Hz130 = 0x07,
			Hz120 = 0x08,
			Hz110 = 0x09,
			Hz100 = 0x0A,
			Hz90 = 0x0B,
			Hz80 = 0x0C,
			Hz70 = 0x0D,
			Hz60 = 0x0E,
			Hz50 = 0x0F,
			Hz45 = 0x10,
			Hz40 = 0x11,
			Hz35 = 0x12,
			Hz30 = 0x13,
			Hz25 = 0x14,
			Hz20 = 0x15,
			Hz15 = 0x16,
			Hz10 = 0x17,
			Hz5 = 0x18,
			Hz4 = 0x19,
			Hz3 = 0x1A,
			Hz2 = 0x1B,
			Hz1 = 0x1C,
			Hz0_5 = 0x1D,
			Hz0_25 = 0x1E,
			Hz0_125 = 0x1F
		};

		enum class Oversampling : uint8_t {
			X1 = 0x00,
			X2 = 0x01,
			X4 = 0x02,
			X8 = 0x03,
			X16 = 0x04,
			X32 = 0x05,
			X64 = 0x06,
			X128 = 0x07
		};

		enum class IIRFilter : uint8_t {
			Bypass = 0x00,	///< Normalized BW (-3dB): Bypass
			Coef1 = 0x01,	///< Normalized BW (-3dB): 0.1147
			Coef3 = 0x02,	///< Normalized BW (-3dB): 0.0459
			Coef7 = 0x03,	///< Normalized BW (-3dB): 0.0212
			Coef15 = 0x04,	///< Normalized BW (-3dB): 0.01025
			Coef31 = 0x05,	///< Normalized BW (-3dB): 0.005041
			Coef63 = 0x06,	///< Normalized BW (-3dB): 0.00250
			Coef127 = 0x07	///< Normalized BW (-3dB): 0.00125
		};

		struct Config {
			OutputDataRate odr;
			Oversampling osrPressure;
			Oversampling osrTemp;
			IIRFilter iirFilter;
		};

		BMP581(I2C& i2c, uint8_t addr) : bus(i2c), addr(addr) {};

		Status Init(const Config& config);
		Status Reset();
		Status ReadID(uint8_t& id);

		Status SetOffset(float offsetPressure);

		Status RequestData();
		Status GetData(float* pressure, float* temp);

	private:
		I2C& bus;
		const uint8_t addr;
		Config config;

		static constexpr uint16_t transferSize = 32;
		__attribute__((aligned(32))) uint8_t buffer[transferSize];

		static constexpr float pressureSens = (1.0f/64);
		static constexpr float tempSens = (1.0f/65536);		
		
		float pressureOffset = 0.0f;
		static constexpr float tempOffset = 0.0f;

		Status WriteRegister(Register reg, uint8_t value);
		Status ReadRegister(Register reg, uint8_t& value);
		Status ModifyRegister(Register reg, uint8_t mask, uint8_t value);
};