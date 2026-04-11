/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Sensors/icp20100.cpp
 */

#include "icp20100.hpp"

Status ICP20100::Init(const Config& config) {
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
	status = this->WriteRegister(ICP20100::Register::MODE_SELECT, (static_cast<uint8_t>(this->config.odr) << 5) | 0x0C);	// Continuous measurements, active mode
	if(status != Status::Ok) {
		return status;
	}

	return Status::Ok;
}

Status ICP20100::Reset() {
	Status status = this->WriteRegister(ICP20100::Register::OTP_DBG2, 0x80);
	if(status != Status::Ok) {
		return status;
	}
	tx_thread_sleep(2);	// From datasheet 2us is enough
	status = this->WriteRegister(ICP20100::Register::OTP_DBG2, 0x00);
	if(status != Status::Ok) {
		return status;
	}
	tx_thread_sleep(2);	// From datasheet 2us is enough
	return Status::Ok;
}

Status ICP20100::ReadID(uint8_t& id) {
	// Chip ID: 0x63
	return this->ReadRegister(ICP20100::Register::DEVICE_ID, id);
}

Status ICP20100::SetOffset(float offsetPressure) {
	this->pressureOffset = offsetPressure;
	return Status::Ok;
}

Status ICP20100::RequestData() {
	this->buffer[0] = static_cast<uint8_t>(ICP20100::Register::PRESS_DATA_0);
	return this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 6);
}

Status ICP20100::GetData(float* pressure, float* temp) {
	if(this->bus.TransferWait(TX_WAIT_FOREVER) != Status::Ok) {
		return Status::Error;
	}

	int32_t rawPress = static_cast<int32_t>((this->buffer[2] << 24) | (this->buffer[1] << 16) | (this->buffer[0] << 8)) >> 8;
	*pressure = (static_cast<float>(rawPress) * this->pressureSens) + this->pressureOffset;

	int32_t rawValue = static_cast<int32_t>((this->buffer[5] << 24) | (this->buffer[4] << 16) | (this->buffer[3] << 8)) >> 8;
	*temp = (static_cast<float>(rawValue) * this->tempSens) + this->tempOffset;

	return Status::Ok;
}

Status ICP20100::WriteRegister(Register reg, uint8_t value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	this->buffer[1] = (value) & 0xFF;
	Status status = this->bus.TransferAsync(this->addr, this->buffer, 2, nullptr, 0);
	if(status != Status::Ok) {
		return status;
	}
	
	return this->bus.TransferWait(TX_WAIT_FOREVER);
}

Status ICP20100::ReadRegister(Register reg, uint8_t& value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	Status status = this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 1);
	if(status != Status::Ok) {
		return status;
	}

	status = this->bus.TransferWait(TX_WAIT_FOREVER);
	value = this->buffer[0];

	return status;
}

Status ICP20100::ModifyRegister(Register reg, uint8_t mask, uint8_t value) {
	uint8_t tmp;

	Status status = this->ReadRegister(reg, tmp);
	if(status != Status::Ok) {
		return status;
	}

	tmp &= ~mask;
	tmp |= value;

	return this->WriteRegister(reg, tmp);
}