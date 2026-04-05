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
	this->ReadID(id);
	if(id != 0xE9) {
		return Status::Error;
	}

	// Reset device
	Status status = this->Reset();
	if(status != Status::Ok) {
		return status;
	}

	return Status::Ok;
}

Status ICP20100::Reset() {
	return Status::Ok;
}

Status ICP20100::ReadID(uint8_t& id) {
	return Status::Ok;
}

Status ICP20100::RequestData() {
	return Status::Ok;
}

Status ICP20100::GetData() {
	return Status::Ok;
}

Status ICP20100::WriteRegister(Register reg, uint8_t value) {
	return Status::Ok;
}

Status ICP20100::ReadRegister(Register reg, uint8_t& value) {
	return Status::Ok;
}

Status ICP20100::ModifyRegister(Register reg, uint8_t mask, uint8_t value) {
	return Status::Ok;
}