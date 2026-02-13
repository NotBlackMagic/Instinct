#pragma once

#include <stdint.h>

#include "i2c.hpp"

class BMP581 {
	public:
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

		BMP581(I2C& i2c, uint8_t addr) : bus(i2c), addr(addr) {};

		void Init();
		void ReadID(uint8_t& id);

		bool RequestData();
		bool GetData(float& pressure, float&  temp);

	private:
		I2C& bus;
		const uint8_t addr;

		uint8_t buffer[8];

		void WriteRegister(Register reg, uint8_t value);
		void ReadRegister(Register reg, uint8_t& value);
};