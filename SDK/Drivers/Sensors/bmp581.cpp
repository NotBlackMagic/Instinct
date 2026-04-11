/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Sensors/bmp581.cpp
 */

#include "bmp581.hpp"

Status BMP581::Init(const Config& config) {
	this->config = config;

	// Verify ID
	uint8_t id;
	this->ReadID(id);
	if(id != this->chipID) {
		return Status::Error;
	}

	// Reset device
	Status status = this->Reset();
	if(status != Status::Ok) {
		return status;
	}

	// Configure device
	status = this->WriteRegister(BMP581::Register::DSP_CONFIG, 0xFF);	// Enable compensations, values taken after IIR
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(BMP581::Register::OSR_CONFIG, 0x40 | (static_cast<uint8_t>(this->config.osrPressure) << 3) | (static_cast<uint8_t>(this->config.osrTemp)));	// Enable Pressure measurements and set OSR
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(BMP581::Register::DSP_IIR, (static_cast<uint8_t>(this->config.iirFilter) << 3) | (static_cast<uint8_t>(this->config.iirFilter)));	// Set IIR filters
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(BMP581::Register::ODR_CONFIG, 0x03 | (static_cast<uint8_t>(this->config.odr) << 2));		//Power up: Normal mode and set ODR
	if(status != Status::Ok) {
		return status;
	}

	return Status::Ok;
}

Status BMP581::Reset() {
	Status status = this->WriteRegister(BMP581::Register::CMD, 0xB6);
	if(status != Status::Ok) {
		return status;
	}

	// Wait for reset, about 2ms
	tx_thread_sleep(2);

	return Status::Ok;
}

Status BMP581::ReadID(uint8_t& id) {
	// Chip ID: 0x50
	return this->ReadRegister(BMP581::Register::CHIP_ID, id);
}

Status BMP581::SetOffset(float offsetPressure) {
	this->pressureOffset = offsetPressure;
	return Status::Ok;
}

Status BMP581::RequestData() {
	this->buffer[0] = static_cast<uint8_t>(BMP581::Register::TEMP_DATA_XLSB);
	if(this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 6) == Status::Ok) {
		return Status::Ok;
	}
	else {
		return Status::Error;
	}
}

Status BMP581::GetData(float* pressure, float* temp) {
	if(this->bus.TransferWait(TX_WAIT_FOREVER) != Status::Ok) {
		return Status::Error;
	}

	uint32_t rawPress = static_cast<uint32_t>((this->buffer[5] << 16) | (this->buffer[4] << 8) | this->buffer[3]);
	*pressure = (static_cast<float>(rawPress) * this->pressureSens) + this->pressureOffset;

	int32_t rawValue = static_cast<int32_t>((this->buffer[2] << 24) | (this->buffer[1] << 16) | (this->buffer[0] << 8)) >> 8;
	*temp = (static_cast<float>(rawValue) * this->tempSens) + this->tempOffset;

	return Status::Ok;
}

Status BMP581::WriteRegister(Register reg, uint8_t value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	this->buffer[1] = (value) & 0xFF;
	Status status = this->bus.TransferAsync(this->addr, this->buffer, 2, nullptr, 0);
	if(status != Status::Ok) {
		return status;
	}
	
	return this->bus.TransferWait(TX_WAIT_FOREVER);
}

Status BMP581::ReadRegister(Register reg, uint8_t& value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	Status status = this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 1);
	if(status != Status::Ok) {
		return status;
	}

	status = this->bus.TransferWait(TX_WAIT_FOREVER);
	value = this->buffer[0];

	return status;
}

Status BMP581::ModifyRegister(Register reg, uint8_t mask, uint8_t value) {
	uint8_t tmp;

	Status status = this->ReadRegister(reg, tmp);
	if(status != Status::Ok) {
		return status;
	}

	tmp &= ~mask;
	tmp |= value;

	return this->WriteRegister(reg, tmp);
}