/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Sensors/bmm350.hpp
 * Author:  NotBlackMagic
 * Brief:   BMP-581 Barometer driver class for STM32N6.
 */

#pragma once

#include <stdint.h>

#include "i2c.hpp"
#include "status.hpp"

class BMM350 {
	public:
		enum class Register : uint8_t {
			CHIP_ID = 0x00,
			ERR_REG = 0x02,
			PAD_CTRL = 0x03,
			PMU_CONFIG = 0x04,
			PMU_AXIS_EN = 0x05,
			PMU_CMD = 0x06,
			PMU_CMD_STATUS0 = 0x07,
			PMU_CMD_STATUS1 = 0x08,
			I3C_ERR = 0x09,
			I2C_WDT_SET = 0x0A,
			INT_CTRL = 0x2E,
			INT_CTRL_IBI = 0x2F,
			INT_STATUS = 0x30,
			MAG_X_XLSB = 0x31,
			MAG_X_LSB = 0x32,
			MAG_X_MSB = 0x33,
			MAG_Y_XLSB = 0x34,
			MAG_Y_LSB = 0x35,
			MAG_Y_MSB = 0x36,
			MAG_Z_XLSB = 0x37,
			MAG_Z_LSB = 0x38,
			MAG_Z_MSB = 0x39,
			TEMP_XLSB = 0x3A,
			TEMP_LSB = 0x3B,
			TEMP_MSB = 0x3C,
			SENSORTIME_XLSB = 0x3D,
			SENSORTIME_TIM_LSB = 0x3E,
			SENSORTIME_TIM_MSB = 0x3F,
			OTP_CMD_REG = 0x50,
			OTP_DATA_MSB_REG = 0x52,
			OTP_DATA_LSB_REG = 0x53,
			OTP_STATUS_REG = 0x55,
			TMR_SELFTEST_USER = 0x60,
			CTRL_USER = 0x61,
			CMD = 0x7E
		};

		struct CompensationData {
			// Offsets
			float tempOffset;
			float offsetX;
			float offsetY;
			float offsetZ;
			// Sensitivities
			float tempSens;
			float sensX;
			float sensY;
			float sensZ;
			//TCO
			float tcoX;
			float tcoY;
			float tcoZ;
			//TCS
			float tcsX;
			float tcsY;
			float tcsZ;
			// Crossaxis
			float crossXY;
			float crossYX;
			float crossZX;
			float crossZY; 
		};

		enum class OutputDataRate : uint8_t {
			Hz400 = 0x02,
			Hz200 = 0x03,
			Hz100 = 0x04,
			Hz50 = 0x05,
			Hz25 = 0x06,
			Hz12_5 = 0x07,
			Hz6_25 = 0x08,
			Hz3_125 = 0x09,
			Hz1_5625 = 0x0A
		};

		enum class Averaging : uint8_t {
			NoAvg = 0x00,
			Avg2 = 0x01,
			Avg4 = 0x02,
			Avg8 = 0x03
		};

		struct Config {
			OutputDataRate odr;
			Averaging avg;
		};

		BMM350(I2C& i2c, uint8_t addr) : bus(i2c), addr(addr) {};

		Status Init(const Config& config);
		Status Reset();
		Status ReadID(uint8_t& id);

		Status SetOffsets(float offsetX, float offsetY, float offsetZ);
		Status RunHardwareSelfTest();

		Status RequestData();
		Status GetData(float* mag, float* temp);

	private:
		I2C& bus;
		const uint8_t addr;
		Config config;

		static constexpr uint16_t transferSize = 32;
		__attribute__((aligned(32))) uint8_t buffer[transferSize];

		CompensationData compData;
		uint16_t otpData[32];

		float magOffset[3] = {0.0f, 0.0f, 0.0f};

		Status WriteRegister(Register reg, uint8_t value);
		Status ReadRegister(Register reg, uint8_t& value);
		Status ModifyRegister(Register reg, uint8_t mask, uint8_t value);

		Status ReadOTPData();
};