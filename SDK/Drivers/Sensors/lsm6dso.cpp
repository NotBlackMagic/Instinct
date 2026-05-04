/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Sensors/lsm6dso.cpp
 */

#include "lsm6dso.hpp"

Status LSM6DSO::Init(const Config& config) {
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

	// Start with RESET command
	status = this->WriteRegister(LSM6DSO::Register::CTRL3_C, 0x01);
	tx_thread_sleep(1000);

	uint8_t regVal = 0;
	// Configure base configs like interfaces, interrupts, etc
	status = this->WriteRegister(LSM6DSO::Register::CTRL3_C, 0x04);			//Automatic increment register address
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(LSM6DSO::Register::CTRL9_XL, 0x02);			//Disable I3C Interface
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(LSM6DSO::Register::CTRL4_C, 0x04);			//Disable I2C Interface
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(LSM6DSO::Register::COUNTER_BDR_REG1, 0x80);	//Enable pulsed data-ready mode
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(LSM6DSO::Register::INT1_CTRL, 0x03);		//INT1 outputs Accelerometer and Gyro data-ready interrupt
	if(status != Status::Ok) {
		return status;
	}

	// Configure Accelerometer Chain
	uint8_t sampleRate = static_cast<uint8_t>(this->config.accelOdr);
	uint8_t scale = static_cast<uint8_t>(this->config.accelScale);
	uint8_t lpf = static_cast<uint8_t>(LSM6DSO::AccelLPF::LPF_ODR100);
	regVal = (sampleRate << 4) + (scale << 2) + 0x02;
	status = this->WriteRegister(LSM6DSO::Register::CTRL1_XL, regVal);
	if(status != Status::Ok) {
		return status;
	}
	regVal = (lpf << 5);
	status = this->WriteRegister(LSM6DSO::Register::CTRL8_XL, regVal);
	if(status != Status::Ok) {
		return status;
	}

	// Configure Gyroscope Chain
	sampleRate = static_cast<uint8_t>(this->config.gyroOdr);
	scale = static_cast<uint8_t>(this->config.gyroScale);
	lpf = static_cast<uint8_t>(LSM6DSO::GyroLPF::LPFType_0);
	regVal = (sampleRate << 4) + (scale << 2);
	status = this->WriteRegister(LSM6DSO::Register::CTRL2_G, regVal);
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(LSM6DSO::Register::CTRL4_C, 0x06);
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(LSM6DSO::Register::CTRL6_C, (lpf - 1));
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(LSM6DSO::Register::CTRL7_G, 0x00);
	if(status != Status::Ok) {
		return status;
	}

	// Configure FIFO
	//status = this->WriteRegister(LSM6DSO::Register::FIFO_CTRL1, 0x01);	//Set FIFO TH to 1 sensor packet
	//status = this->WriteRegister(LSM6DSO::Register::FIFO_CTRL3, 0x00);	//Accel BDR: 833 Hz; Gyro BDR: 833 Hz
	//status = this->WriteRegister(LSM6DSO::Register::FIFO_CTRL4, 0x00);	//Disable/bypass FIFO mode (Will be enabled when EXTI INT is ready/set-up)

	// Wait for LSM6DSO LPF stabilization: around 9 * Ts = 9*1/8.33 = 1080 ms (within 0.01% of final value)
	tx_thread_sleep(1080);

	return Status::Ok;
}

Status LSM6DSO::Reset() {
	return Status::Ok;
}

Status LSM6DSO::ReadID(uint8_t& id) {
	return this->ReadRegister(LSM6DSO::Register::WHO_AM_I, id);
}

Status LSM6DSO::ReadStatus(uint8_t& status) {
	return this->ReadRegister(LSM6DSO::Register::STATUS_REG, status);
}

Status LSM6DSO::SetScales(AccelScale accelScale, GyroScale gyroScale) {
	return Status::Ok;
}

Status LSM6DSO::SetAccelOffsets(float offsetX, float offsetY, float offsetZ) {
	this->accelOffset[0] = offsetX;
	this->accelOffset[1] = offsetY;
	this->accelOffset[2] = offsetZ;
	return Status::Ok;
}

Status LSM6DSO::SetGyroOffsets(float offsetX, float offsetY, float offsetZ) {
	this->gyroOffset[0] = offsetX;
	this->gyroOffset[1] = offsetY;
	this->gyroOffset[2] = offsetZ;
	return Status::Ok;
}

Status LSM6DSO::RunHardwareSelfTest() {
	return Status::Ok;
}

Status LSM6DSO::RequestData() {
	this->txBuffer[0] = 0x80 | static_cast<uint8_t>(LSM6DSO::Register::OUT_TEMP_L);
	return this->bus.TransferAsync(this->txBuffer, this->rxBuffer, 15);
}

Status LSM6DSO::GetData(float* accel, float* gyro, float* temp) {
	if(this->bus.TransferWait(TX_WAIT_FOREVER) != Status::Ok) {
		return Status::Error;
	}

	float sens = this->accelSens[static_cast<uint8_t>(this->config.accelScale)];
	accel[0] = this->ParseAxis(this->rxBuffer[10], this->rxBuffer[9], sens, this->accelOffset[0]);
	accel[1] = this->ParseAxis(this->rxBuffer[12], this->rxBuffer[11], sens, this->accelOffset[1]);
	accel[2] = this->ParseAxis(this->rxBuffer[14], this->rxBuffer[13], sens, this->accelOffset[2]);

	sens = this->gyroSens[static_cast<uint8_t>(this->config.gyroScale)];
	gyro[0] = this->ParseAxis(this->rxBuffer[4], this->rxBuffer[3], sens, this->gyroOffset[0]);
	gyro[1] = this->ParseAxis(this->rxBuffer[6], this->rxBuffer[5], sens, this->gyroOffset[1]);
	gyro[2] = this->ParseAxis(this->rxBuffer[8], this->rxBuffer[7], sens, this->gyroOffset[2]);

	*temp = this->ParseAxis(this->rxBuffer[2], this->rxBuffer[1], this->tempSens, this->tempOffset);

	return Status::Ok;
}

float LSM6DSO::ParseAxis(uint8_t msb, uint8_t lsb, float scaleFactor, float offset) {
	int16_t rawValue = static_cast<int16_t>((msb << 8) | lsb);
	return (static_cast<float>(rawValue) * scaleFactor) + offset;
}

Status LSM6DSO::WriteRegister(Register reg, uint8_t value) {
	this->txBuffer[0] = static_cast<uint8_t>(reg);
	this->txBuffer[1] = value;
	Status status = this->bus.TransferAsync(this->txBuffer, this->rxBuffer, 2);
	if(status != Status::Ok) {
		return status;
	}

	return this->bus.TransferWait(TX_WAIT_FOREVER);
}

Status LSM6DSO::ReadRegister(Register reg, uint8_t& value) {
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

Status LSM6DSO::ModifyRegister(Register reg, uint8_t mask, uint8_t value) {
	uint8_t tmp;

	Status status = this->ReadRegister(reg, tmp);
	if(status != Status::Ok) {
		return status;
	}

	tmp &= ~mask;
	tmp |= value;

	return this->WriteRegister(reg, tmp);
}