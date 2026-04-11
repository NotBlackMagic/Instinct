/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Sensors/lis3mdl.cpp
 */

#include "lis2mdl.hpp"

Status LIS2MDL::Init(const Config& config) {
	this->config = config;

	// Verify ID
	uint8_t id;
	Status status = this->ReadID(id);
	if(status != Status::Ok || id != this->chipID) {
		return Status::Error;
	}

	// Reset device
	status = this->Reset();
	if(status != Status::Ok) {
		return status;
	}
	
	// Configure device
	status = this->WriteRegister(LIS2MDL::Register::CFG_REG_A, 0x80 | (static_cast<uint8_t>(this->config.odr) << 2) | 0x00);	//Enable temperature compensation, ODR = 100 Hz, continuous mode
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(LIS2MDL::Register::CFG_REG_B, 0x02 | (this->config.enableLPF == true ? 0x01 : 0x00));
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(LIS2MDL::Register::CFG_REG_C, 0x11);	//Enable BDU consistency, enable DRDY
	if(status != Status::Ok) {
		return status;
	}

	return Status::Ok;
}

Status LIS2MDL::Reset() {
	// Soft reset via CFG_REG_A (Bit 5)
	Status status = this->WriteRegister(Register::CFG_REG_A, 0x20);
	if(status != Status::Ok) {
		return status;
	}

	// Wait for reboot to finish
	tx_thread_sleep(20);
	return Status::Ok;
}

Status LIS2MDL::ReadID(uint8_t& id) {
	// Chip ID: 0x40
	return this->ReadRegister(LIS2MDL::Register::WHO_AM_I, id);
}

Status LIS2MDL::SetOffsets(float offsetX, float offsetY, float offsetZ) {
	//Hard Iron offset calibration		https://appelsiini.net/2018/calibrate-magnetometer/
	//X: (0.8 + 0.08) / 2	= 0.44 	-> 440 / 1.5 = 293		|| Mean: 0.516
	//Y: (1.1 + 0.43) / 2	= 0.765	-> 765 / 1.5 = 510		|| Mean: 0.732
	//Z: (0.38 + -0.3) / 2	= 0.04	->  40 / 1.5 = 27		|| Mean: 0.057
	// int16_t offsetX = 0;
	// int16_t offsetY = 0;
	// int16_t offsetZ = 0;
	// this->WriteRegister(LIS2MDL::Register::OFFSET_X_REG_H, ((offsetX >> 8) & 0xFF));
	// this->WriteRegister(LIS2MDL::Register::OFFSET_X_REG_L, ((offsetX) & 0xFF));
	// this->WriteRegister(LIS2MDL::Register::OFFSET_Y_REG_H, ((offsetY >> 8) & 0xFF));
	// this->WriteRegister(LIS2MDL::Register::OFFSET_Y_REG_L, ((offsetY) & 0xFF));
	// this->WriteRegister(LIS2MDL::Register::OFFSET_Z_REG_H, ((offsetZ >> 8) & 0xFF));
	// this->WriteRegister(LIS2MDL::Register::OFFSET_Z_REG_L, ((offsetZ) & 0xFF));
	this->magOffset[0] = offsetX;
	this->magOffset[1] = offsetY;
	this->magOffset[2] = offsetZ;
	return Status::Ok;
}

Status LIS2MDL::RunHardwareSelfTest() {
	// Stub: The LIS2MDL supports a built-in self-test via CFG_REG_C bit 1.
	return Status::Ok; 
}

Status LIS2MDL::RequestData() {
	this->buffer[0] = 0x80 | static_cast<uint8_t>(LIS2MDL::Register::OUTX_L_REG);	//Enable auto increment register addresss
	return this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 6);
}

Status LIS2MDL::GetData(float* field, float* temp) {
	if(this->bus.TransferWait(TX_WAIT_FOREVER) != Status::Ok) {
		return Status::Error;
	}

	int16_t rawMX = (int16_t)((this->buffer[1] << 8) | this->buffer[0]);
	int16_t rawMY = (int16_t)((this->buffer[3] << 8) | this->buffer[2]);
	int16_t rawMZ = (int16_t)((this->buffer[5] << 8) | this->buffer[4]);
	// int16_t rawTemp = (int16_t)((this->buffer[7] << 8) + this->buffer[6]);	//Can't read in burst (auto increment) mode

	field[0] = (static_cast<float>(rawMX) * this->magSens) + this->magOffset[0];
	field[1] = (static_cast<float>(rawMY) * this->magSens) + this->magOffset[1];
	field[2] = (static_cast<float>(rawMZ) * this->magSens) + this->magOffset[2];

	return Status::Ok;
}

Status LIS2MDL::ReadTemperature(float& value) {
	this->buffer[0] = static_cast<uint8_t>(LIS2MDL::Register::TEMP_OUT_L_REG);
	Status status = this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 2);
	if(status != Status::Ok) {
		return status;
	}
	
	status = this->bus.TransferWait(TX_WAIT_FOREVER);
	if(status == Status::Ok) {
		value = (int16_t)((this->buffer[1] << 8) | this->buffer[0]) * this->tempSens + this->tempOffset;
	}

	return status;
}

Status LIS2MDL::WriteRegister(Register reg, uint8_t value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	this->buffer[1] = (value) & 0xFF;
	Status status = this->bus.TransferAsync(this->addr, this->buffer, 2, nullptr, 0);
	if(status != Status::Ok) {
		return status;
	}
	
	return this->bus.TransferWait(TX_WAIT_FOREVER);
}

Status LIS2MDL::ReadRegister(Register reg, uint8_t& value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	Status status = this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 1);
	if(status != Status::Ok) {
		return status;
	}
	
	status = this->bus.TransferWait(TX_WAIT_FOREVER);
	value = this->buffer[0];

	return status;
}

Status LIS2MDL::ModifyRegister(Register reg, uint8_t mask, uint8_t value) {
	uint8_t tmp;

	Status status = this->ReadRegister(reg, tmp);
	if(status != Status::Ok) {
		return status;
	}

	tmp &= ~mask;
	tmp |= value;

	return this->WriteRegister(reg, tmp);
}