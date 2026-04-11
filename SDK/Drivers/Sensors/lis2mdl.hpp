/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Sensors/lis2mdl.hpp
 * Author:  NotBlackMagic
 * Brief:   LIS2MDL Magnetometer driver class for STM32N6.
 */


#pragma once

#include <stdint.h>

#include "i2c.hpp"
#include "status.hpp"

class LIS2MDL {
	public:
		// Standard Chip identifications
		static constexpr uint8_t i2cAddrPrimary = 0x1E;
		static constexpr uint8_t chipID = 0x40;

		enum class Register : uint8_t {
			OFFSET_X_REG_L = 0x45,
			OFFSET_X_REG_H = 0x46,
			OFFSET_Y_REG_L = 0x47,
			OFFSET_Y_REG_H = 0x48,
			OFFSET_Z_REG_L = 0x49,
			OFFSET_Z_REG_H = 0x4A,
			WHO_AM_I = 0x4F,
			CFG_REG_A = 0x60,
			CFG_REG_B = 0x61,
			CFG_REG_C = 0x62,
			INT_CRTL_REG = 0x63,
			INT_SOURCE_REG = 0x64,
			INT_THS_L_REG = 0x65,
			INT_THS_H_REG = 0x66,
			STATUS_REG = 0x67,
			OUTX_L_REG = 0x68,
			OUTX_H_REG = 0x69,
			OUTY_L_REG = 0x6A,
			OUTY_H_REG = 0x6B,
			OUTZ_L_REG = 0x6C,
			OUTZ_H_REG = 0x6D,
			TEMP_OUT_L_REG = 0x6E,
			TEMP_OUT_H_REG = 0x6F
		};

		enum class OutputDataRate : uint8_t {
			Hz10 = 0x00,
			Hz20 = 0x01,
			Hz50 = 0x02,
			Hz100 = 0x03
		};

		struct Config {
			OutputDataRate odr;
			bool enableLPF;
		};

		LIS2MDL(I2C& i2c, uint8_t addr) : bus(i2c), addr(addr) {};

		Status Init(const Config& config);
		Status Reset();
		Status ReadID(uint8_t& id);

		Status SetOffsets(float offsetX, float offsetY, float offsetZ);
		Status RunHardwareSelfTest();

		Status RequestData();
		Status GetData(float* field, float* temp);
		Status ReadTemperature(float& value);

	private:
		I2C& bus;
		const uint8_t addr;
		Config config;

		static constexpr uint16_t transferSize = 32;
		__attribute__((aligned(32))) uint8_t buffer[transferSize];

		//8 LSB/C + 25C zero offset (https://github.com/STMicroelectronics/lis2mdl-pid/blob/master/lis2mdl_reg.c)
		static constexpr float magSens = 1.5f;
		static constexpr float tempSens = (1.0f/8);		

		float magOffset[3] = {0.0f, 0.0f, 0.0f};
		static constexpr float tempOffset = 25.0f;

		Status WriteRegister(Register reg, uint8_t value);
		Status ReadRegister(Register reg, uint8_t& value);
		Status ModifyRegister(Register reg, uint8_t mask, uint8_t value);
};