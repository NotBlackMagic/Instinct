#pragma once

#include <stdint.h>

#include "i2c.hpp"

class BMM350 {
	public:
		enum class Register : uint8_t {
		};

		BMM350(I2C& i2c, uint8_t addr) : bus(i2c), addr(addr) {};

		void Init();
		void ReadID(uint8_t& id);
		void ReadTemperature(float& value);

	private:
		I2C& bus;
		const uint8_t addr;

		void WriteRegister(Register reg, uint8_t value);
		void ReadRegister(Register reg, uint8_t& value);
};