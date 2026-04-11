/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Sensors/icm45686.cpp
 */

#include "icm45686.hpp"

Status ICM45686::Init(const Config& config) {
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

	// Turn on device in Low Noise Mode
	status = this->WriteRegister(ICM45686::Register::PWR_MGMT0, 0x0F);
	if(status != Status::Ok) {
		return status;
	}

	// Configure scales and rates
	status = this->WriteRegister(ICM45686::Register::ACCEL_CONFIG0, (static_cast<uint8_t>(this->config.accelScale) << 4) | static_cast<uint8_t>(this->config.accelOdr));
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(ICM45686::Register::GYRO_CONFIG0, (static_cast<uint8_t>(this->config.gyroScale) << 4) | static_cast<uint8_t>(this->config.gyroOdr));
	if(status != Status::Ok) {
		return status;
	}

	// Configure INT1 outputs
	status = this->WriteRegister(ICM45686::Register::INT1_CONFIG0, 0x04);	//INT1: Data Ready Interrupt
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(ICM45686::Register::INT1_CONFIG1, 0x00);
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(ICM45686::Register::INT1_CONFIG2, 0x01);	//INT1: Push-pull, pulse mode, active high
	if(status != Status::Ok) {
		return status;
	}

	return Status::Ok;
}

Status ICM45686::Reset() {
	uint8_t stat;

	Status status = this->WriteRegister(ICM45686::Register::REG_MISC2, 0x02);
	if(status != Status::Ok) {
		return status;
	}

	do {
		status = this->ReadRegister(ICM45686::Register::REG_MISC2, stat);
		if(status != Status::Ok) {
			return status;
		}
		// Yield for a bit
		tx_thread_sleep(2);
	} while((stat & 0x02) == 0x02);

	return Status::Ok;
}

Status ICM45686::ReadID(uint8_t& id) {
	return this->ReadRegister(ICM45686::Register::WHO_AM_I, id);
}

Status ICM45686::SetScales(AccelScale accelScale, GyroScale gyroScale) {
	Status status = this->ModifyRegister(ICM45686::Register::ACCEL_CONFIG0, 0x70, (static_cast<uint8_t>(accelScale) << 4));
	if(status != Status::Ok) {
		return status;
	}
	this->config.accelScale = accelScale;

	status = this->ModifyRegister(ICM45686::Register::GYRO_CONFIG0, 0xF0, (static_cast<uint8_t>(gyroScale) << 4));
	if(status != Status::Ok) {
		return status;
	}
	this->config.gyroScale = gyroScale;

	return Status::Ok;
}

Status ICM45686::SetAccelOffsets(float offsetX, float offsetY, float offsetZ) {
	this->accelOffset[0] = offsetX;
	this->accelOffset[1] = offsetY;
	this->accelOffset[2] = offsetZ;
	return Status::Ok;
}

Status ICM45686::SetGyroOffsets(float offsetX, float offsetY, float offsetZ) {
	this->gyroOffset[0] = offsetX;
	this->gyroOffset[1] = offsetY;
	this->gyroOffset[2] = offsetZ;
	return Status::Ok;
}

Status ICM45686::RunHardwareSelfTest() {
	return Status::Ok;
}

Status ICM45686::RequestData() {
	this->txBuffer[0] = 0x80 | static_cast<uint8_t>(ICM45686::Register::ACCEL_DATA_X1_UI);
	return this->bus.TransferAsync(this->txBuffer, this->rxBuffer, 15);
}

Status ICM45686::GetData(float* accel, float* gyro, float* temp) {
	if(this->bus.TransferWait(TX_WAIT_FOREVER) != Status::Ok) {
		return Status::Error;
	}

	float sens = accelSens[static_cast<uint8_t>(this->config.accelScale)];
	accel[0] = this->ParseAxis(rxBuffer[1], rxBuffer[2], sens, accelOffset[0]);
	accel[1] = this->ParseAxis(rxBuffer[3], rxBuffer[4], sens, accelOffset[1]);
	accel[2] = this->ParseAxis(rxBuffer[5], rxBuffer[6], sens, accelOffset[2]);

	sens = gyroSens[static_cast<uint8_t>(this->config.gyroScale)];
	gyro[0] = this->ParseAxis(rxBuffer[7], rxBuffer[8], sens, gyroOffset[0]);
	gyro[1] = this->ParseAxis(rxBuffer[9], rxBuffer[10], sens, gyroOffset[1]);
	gyro[2] = this->ParseAxis(rxBuffer[11], rxBuffer[12], sens, gyroOffset[2]);

	*temp = this->ParseAxis(rxBuffer[13], rxBuffer[14], tempSens, tempOffset);

	return Status::Ok;
}

float ICM45686::ParseAxis(uint8_t msb, uint8_t lsb, float scaleFactor, float offset) {
	int16_t rawValue = static_cast<int16_t>((msb << 8) | lsb);
	return (static_cast<float>(rawValue) * scaleFactor) + offset;
}

Status ICM45686::WriteRegister(Register reg, uint8_t value) {
	this->txBuffer[0] = static_cast<uint8_t>(reg);
	this->txBuffer[1] = value;
	Status status = this->bus.TransferAsync(this->txBuffer, this->rxBuffer, 2);
	if(status != Status::Ok) {
		return status;
	}

	return this->bus.TransferWait(TX_WAIT_FOREVER);
}

Status ICM45686::ReadRegister(Register reg, uint8_t& value) {
	this->txBuffer[0] = 0x80 | static_cast<uint8_t>(reg);
	this->txBuffer[1] = 0x00; // Dummy byte
	Status status = this->bus.TransferAsync(this->txBuffer, this->rxBuffer, 2);
	if(status != Status::Ok) {
		return status;
	}

	status = this->bus.TransferWait(TX_WAIT_FOREVER);
	value = this->rxBuffer[1];

	return status;
}

Status ICM45686::ModifyRegister(Register reg, uint8_t mask, uint8_t value) {
	uint8_t tmp;

	Status status = this->ReadRegister(reg, tmp);
	if(status != Status::Ok) {
		return status;
	}

	tmp &= ~mask;
	tmp |= value;

	return this->WriteRegister(reg, tmp);
}