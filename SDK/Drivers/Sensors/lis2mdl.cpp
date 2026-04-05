#include "lis2mdl.hpp"
#include "status.hpp"
#include "stm32n657xx.h"

void LIS2MDL::Init(const Config& config) {
	this->config = config;
	
	this->WriteRegister(LIS2MDL::Register::CFG_REG_A, 0x8C);	//Enable temperature compensation, ODR = 100 Hz
	this->WriteRegister(LIS2MDL::Register::CFG_REG_C, 0x11);	//Enable BDU consistency, enable DRDY

	//Hard Iron offset calibration		https://appelsiini.net/2018/calibrate-magnetometer/
	//X: (0.8 + 0.08) / 2	= 0.44 	-> 440 / 1.5 = 293		|| Mean: 0.516
	//Y: (1.1 + 0.43) / 2	= 0.765	-> 765 / 1.5 = 510		|| Mean: 0.732
	//Z: (0.38 + -0.3) / 2	= 0.04	->  40 / 1.5 = 27		|| Mean: 0.057
	int16_t offsetX = 0;
	int16_t offsetY = 0;
	int16_t offsetZ = 0;
	this->WriteRegister(LIS2MDL::Register::OFFSET_X_REG_H, ((offsetX >> 8) & 0xFF));
	this->WriteRegister(LIS2MDL::Register::OFFSET_X_REG_L, ((offsetX) & 0xFF));
	this->WriteRegister(LIS2MDL::Register::OFFSET_Y_REG_H, ((offsetY >> 8) & 0xFF));
	this->WriteRegister(LIS2MDL::Register::OFFSET_Y_REG_L, ((offsetY) & 0xFF));
	this->WriteRegister(LIS2MDL::Register::OFFSET_Z_REG_H, ((offsetZ >> 8) & 0xFF));
	this->WriteRegister(LIS2MDL::Register::OFFSET_Z_REG_L, ((offsetZ) & 0xFF));
}

void LIS2MDL::ReadID(uint8_t& id) {
	this->ReadRegister(LIS2MDL::Register::WHO_AM_I, id);
}

bool LIS2MDL::RequestData() {
	this->buffer[0] = 0x80 | static_cast<uint8_t>(LIS2MDL::Register::OUTX_L_REG);	//Enable auto increment register addresss
	if(this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 6) == Status::Ok) {
		return true;
	}
	else {
		return false;
	}
}

bool LIS2MDL::GetData(float* field, float* temp) {
	if(this->bus.TransferWait(TX_WAIT_FOREVER) != Status::Ok) {
		return false;
	}

	int16_t rawMX = (int16_t)((this->buffer[1] << 8) + this->buffer[0]);
	int16_t rawMY = (int16_t)((this->buffer[3] << 8) + this->buffer[2]);
	int16_t rawMZ = (int16_t)((this->buffer[5] << 8) + this->buffer[4]);
	// int16_t rawTemp = (int16_t)((this->buffer[7] << 8) + this->buffer[6]);	//Can't read in burst (auto increment) mode

	//*temp = rawTemp * (1.0f/8) + 25.0f;		//8 LSB/C + 25C zero offset (https://github.com/STMicroelectronics/lis2mdl-pid/blob/master/lis2mdl_reg.c)

	field[0] = rawMX * 1.5f;
	field[1] = rawMY * 1.5f;
	field[2] = rawMZ * 1.5f;

	return true;
}

void LIS2MDL::ReadTemperature(float& value) {
	this->buffer[0] = static_cast<uint8_t>(LIS2MDL::Register::TEMP_OUT_L_REG);
	this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 2);
	this->bus.TransferWait(TX_WAIT_FOREVER);
	value = (int16_t)((this->buffer[1] << 8) + this->buffer[0]) * (1.0f/8) + 25.0f;		//8 LSB/C + 25C zero offset (https://github.com/STMicroelectronics/lis2mdl-pid/blob/master/lis2mdl_reg.c)
	// uint8_t data[2];
	// this->ReadRegister(LIS2MDL::Register::TEMP_OUT_L_REG, data[0]);
	// this->ReadRegister(LIS2MDL::Register::TEMP_OUT_H_REG, data[1]);
	// value = (int16_t)((data[1] << 8) + data[0]) * (1.0f/8) + 25.0f;			//8 LSB/C + 25C zero offset (https://github.com/STMicroelectronics/lis2mdl-pid/blob/master/lis2mdl_reg.c)
}

void LIS2MDL::WriteRegister(Register reg, uint8_t value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	this->buffer[1] = (value) & 0xFF;
	this->bus.TransferAsync(this->addr, this->buffer, 2, nullptr, 0);
	this->bus.TransferWait(TX_WAIT_FOREVER);
}

void LIS2MDL::ReadRegister(Register reg, uint8_t& value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 1);
	this->bus.TransferWait(TX_WAIT_FOREVER);
	value = this->buffer[0];
}