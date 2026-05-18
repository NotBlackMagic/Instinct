/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Video/ov9281.cpp
 */

#include "ov9281.hpp"

Status OV9281::Init(const Config &config) {
	this->config = config;

	// Verify ID
	uint16_t manID = 0;
	Status status = this->ReadID(&manID);
	if(status != Status::Ok || manID != this->chipID) {
		return Status::Error;
	}
	sensorInfo.id = manID;

	// Reset device
	status = this->Reset();
	if(status != Status::Ok) {
		return status;
	}

	return Status::Ok;
}

Status OV9281::Reset() {
	if(this->config.resetPin != nullptr) {
		this->config.resetPin->Write(0);
		tx_thread_sleep(10);
		this->config.resetPin->Write(1);
		tx_thread_sleep(10);
		return Status::Ok;
	}
	else {
		// Status status = this->WriteRegister(OV9281::Register::COM7, 0x80);
		// tx_thread_sleep(10);
		// return status;
	}
}

Status OV9281::Start() {
	if(config.powerDownPin != nullptr) {
		config.powerDownPin->Write(0);
		tx_thread_sleep(5);
		return Status::Ok;
	}
	else {
		// Fallback to Clear Software Sleep
		// return ModifyRegister(OV9281::Register::COM2, 0x10, 0x00);
	}
}

Status OV9281::Stop() {
	if(config.powerDownPin != nullptr) {
		config.powerDownPin->Write(1);
		return Status::Ok;
	}
	else {
		// Fallback to Software Sleep (COM2 bit 4)
		// return this->ModifyRegister(OV9281::Register::COM2, 0x10, 0x10);
	}
}

Status OV9281::ReadID(uint16_t *id) {
	uint8_t pid, vid;
	// if(this->ReadRegister(OV9281::Register::PID, pid) != Status::Ok) {
	// 	return Status::Error;
	// }
	// if(this->ReadRegister(OV9281::Register::VER, vid) != Status::Ok) {
	// 	return Status::Error;
	// }
	*id = (pid << 8) + vid;
	return Status::Ok;
}

Status OV9281::WriteRegister(Register reg, uint8_t value) {
	const uint8_t maxRetries = 3;
	Status status = Status::Error;

	// Do a few retires, due to using SCCB and not I2C there are issues with the ACK (Do-Not-Care in SCCB)
	for(uint8_t attempt = 0; attempt < maxRetries; attempt++) {
		this->buffer[0] = static_cast<uint8_t>(reg);
		this->buffer[1] = value;

		status = this->bus.TransferAsync(this->addr, I3C::TargetType::I2C, this->buffer, 2, nullptr, 0);
		if(status == Status::Ok) {
			status = this->bus.TransferWait(1000);
			if(status == Status::Ok) {
				return Status::Ok;
			}
		}

		tx_thread_sleep(1);
	}

	return status;
}

Status OV9281::ReadRegister(Register reg, uint8_t& value) {
	const uint8_t maxRetries = 3;
	Status status = Status::Error;

	// Do a few retires, due to using SCCB and not I2C there are issues with the ACK (Do-Not-Care in SCCB)
	for(uint8_t attempt = 0; attempt < maxRetries; attempt++) {
		this->buffer[0] = static_cast<uint8_t>(reg);
		status = this->bus.TransferAsync(this->addr, I3C::TargetType::I2C, this->buffer, 1, this->buffer, 0);
		if(status == Status::Ok) {
			status = this->bus.TransferWait(1000);
		}

		if(status != Status::Ok) {
			// Failed in write regsiter, loop around and retry
			tx_thread_sleep(1);
			continue;
		}

		status = this->bus.TransferAsync(this->addr, I3C::TargetType::I2C, nullptr, 0, this->buffer, 1);
		if(status == Status::Ok) {
			status = this->bus.TransferWait(1000);
			if(status == Status::Ok) {
				value = this->buffer[0];
				return Status::Ok;
			}
		}

		tx_thread_sleep(1);
	}
	
	return status;
}

Status OV9281::ModifyRegister(Register reg, uint8_t mask, uint8_t value) {
	uint8_t regVal;
	Status status = this->ReadRegister(reg, regVal);
	if(status != Status::Ok) {
		return status;
	}

	regVal &= ~mask;
	regVal |= (value & mask);

	return this->WriteRegister(reg, regVal);
}