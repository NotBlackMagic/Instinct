#pragma once

#include <stdint.h>

#include "i2c.hpp"

class INA700 {
	public:
		enum class Register : uint8_t {
			CONFIG = 0x00,			//Configuration, 16 Bits
			ADC_CONFIG = 0x01,		//ADC Configuration, 16 Bits
			VBUS = 0x05,			//Bus Voltage Measurement, 16 Bits
			DIETEMP = 0x06,			//Temperature Measurement, 16 Bits
			CURRENT = 0x07,			//Current Result, 16 Bits
			POWER = 0x08,			//Power Result, 24 Bits
			ENERGY = 0x09,			//Energy Result, 40 Bits
			CHARGE = 0x0A,			//Charge Result, 40 Bits
			ALERT_DIAG = 0x0B,		//Diagnostic Flags and Alert, 16 Bits
			COL = 0x0C,				//Current Over-Limit Threshold, 16 Bits
			CUL = 0x0D,				//Current Under-Limit Threshold, 16 Bits
			BOVL = 0x0E,			//Bus Overvoltage Threshold, 16 Bits
			BUVL = 0x0F,			//Bus Undervoltage Threshold, 16 Bits
			TEMP_LIMIT = 0x10,		//Temperature Over-Limit Threshold, 16 Bits
			PWR_LIMIT = 0x11,		//Power Over-Limit Threshold, 16 Bits
			MANUFACTURER_ID = 0x3E	//Manufacturer ID, 16 Bits
		};

		INA700(I2C& i2c, uint8_t addr) : bus(i2c), addr(addr) {};

		void Init();
		void ReadID(uint16_t& id);
		void ReadVoltage(float& value);
		void ReadCurrent(float& value);
		void ReadTemperature(float& value);

	private:
		I2C& bus;
		const uint8_t addr;

		uint8_t buffer[8];
		
		void WriteRegister(Register reg, uint16_t value);
		void ReadRegister(Register reg, uint16_t& value);
};