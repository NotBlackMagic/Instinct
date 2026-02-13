#pragma once

#include <stdint.h>

#include "i2c.hpp"

class LIS2MDL {
	public:
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

		LIS2MDL(I2C& i2c, uint8_t addr) : bus(i2c), addr(addr) {};

		void Init();
		void ReadID(uint8_t& id);

		bool RequestData();
		bool GetData(float* field, float* temp);
		void ReadTemperature(float& value);

	private:
		I2C& bus;
		const uint8_t addr;

		uint8_t buffer[10];

		void WriteRegister(Register reg, uint8_t value);
		void ReadRegister(Register reg, uint8_t& value);
};