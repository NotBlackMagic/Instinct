/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Sensors/bmm350.cpp
 */

#include "bmm350.hpp"

Status BMM350::Init(const Config& config) {
	this->config = config;

	// Verify ID
	uint8_t id;
	this->ReadID(id);
	if(id != 0x33) {
		return Status::Error;
	}

	// Reset device
	Status status = this->Reset();
	if(status != Status::Ok) {
		return status;
	}

	// Configure device
	status = this->WriteRegister(BMM350::Register::PMU_CONFIG, (static_cast<uint8_t>(this->config.avg) << 4) | static_cast<uint8_t>(this->config.odr));		// Set ODR and sample averaging
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(BMM350::Register::PMU_CONFIG, 0x07);	// Enable all channels
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(BMM350::Register::PMU_CMD, 0x01);	// Enter normal mode
	if(status != Status::Ok) {
		return status;
	}
	// Wait for normal mode enter, can take up to 40ms
	tx_thread_sleep(40);

	return Status::Ok;
}

Status BMM350::Reset() {
	Status status = this->WriteRegister(BMM350::Register::CMD, 0xB6);	// Soft-reset
	if(status != Status::Ok) {
		return status;
	}
	// Wait for reset, can take up to 8ms
	tx_thread_sleep(8);

	// Get OTP data before power off OTP domain
	this->ReadOTPData();

	status = this->WriteRegister(BMM350::Register::OTP_CMD_REG, 0x80);	// Power off OTP
	if(status != Status::Ok) {
		return status;
	}
	// Wait for OTP power off
	tx_thread_sleep(8);

	// Perform the magnetic reset of the sensor

	uint8_t pmuStatus0;
	status = this->ReadRegister(BMM350::Register::PMU_CMD_STATUS0, pmuStatus0);
	if(status != Status::Ok) {
		return status;
	}
	if((pmuStatus0 & 0x08) == 0x08) {
		// Is PMU normal mode, enter suspend
		status = this->WriteRegister(BMM350::Register::PMU_CMD, 0x00);	// Enter normal mode
		if(status != Status::Ok) {
			return status;
		}
	}

	status = this->WriteRegister(BMM350::Register::PMU_CMD, 0x07);	// Reset with full CRST recharge
	if(status != Status::Ok) {
		return status;
	}
	// Wait for BR
	tx_thread_sleep(15);

	// Verify PMU CMD has BR set
	status = this->ReadRegister(BMM350::Register::PMU_CMD_STATUS0, pmuStatus0);
	if(status != Status::Ok || (pmuStatus0 >> 5) != 0x07) {
		return status;
	}

	status = this->WriteRegister(BMM350::Register::PMU_CMD, 0x05);	// Flux-guide reset with full CRST recharge
	if(status != Status::Ok) {
		return status;
	}
	// Wait for FGR
	tx_thread_sleep(20);

	// Verify PMU CMD has FGR set
	status = this->ReadRegister(BMM350::Register::PMU_CMD_STATUS0, pmuStatus0);
	if(status != Status::Ok || (pmuStatus0 >> 5) != 0x05) {
		return status;
	}

	return Status::Ok;
}

Status BMM350::ReadID(uint8_t& id) {
	// Chip ID: 0x33
	return this->ReadRegister(BMM350::Register::CHIP_ID, id);
}

Status BMM350::SetOffsets(float offsetX, float offsetY, float offsetZ) {
	this->magOffset[0] = offsetX;
	this->magOffset[1] = offsetY;
	this->magOffset[2] = offsetZ;
	return Status::Ok;
}

Status BMM350::RunHardwareSelfTest() {
	return Status::Ok;
}

Status BMM350::RequestData() {
	this->buffer[0] = static_cast<uint8_t>(BMM350::Register::MAG_X_XLSB);
	if(this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 12) == Status::Ok) {
		return Status::Ok;
	}
	else {
		return Status::Error;
	}
}

Status BMM350::GetData(float* mag, float* temp) {
	if(this->bus.TransferWait(TX_WAIT_FOREVER) != Status::Ok) {
		return Status::Error;
	}

	mag[0] = this->ParseMagAxis(this->buffer[2], this->buffer[1], this->buffer[0], this->magOffset[0]);
	mag[1] = this->ParseMagAxis(this->buffer[5], this->buffer[4], this->buffer[3], this->magOffset[1]);
	mag[2] = this->ParseMagAxis(this->buffer[8], this->buffer[7], this->buffer[6], this->magOffset[2]);

	*temp = this->ParseTemperature(this->buffer[10], this->buffer[10], this->buffer[9]);

	return Status::Ok;
}

inline int32_t SignExtend(uint32_t value, int bits) {
	int32_t power = 1 << (bits - 1);
	int32_t retval = static_cast<int32_t>(value);
	if (retval >= power) {
		retval -= (power * 2);
	}
	return retval;
}

Status BMM350::ReadOTPData() {
	// Code from Bosch API: https://github.com/boschsensortec/BMM350_SensorAPI/blob/main/bmm350.c
	// It is required to be in suspend/normal before reading OTP
	for (uint8_t i = 0; i < 32; i++) {
		Status status = this->WriteRegister(Register::OTP_CMD_REG, 0x20 | (i & 0x1F));
		if(status != Status::Ok) {
			return status;
		}

		uint8_t otpStatus = 0;
		uint8_t timeout = 10;
		do {
			tx_thread_sleep(1);

			status = this->ReadRegister(Register::OTP_STATUS_REG, otpStatus);
			if(status != Status::Ok || (otpStatus & 0xE0) != 0) {
				return Status::Error;
			}

			timeout = timeout - 1;
		} while(((otpStatus & 0x01) == 0x01) && (timeout > 0)); // Wait for CMD_DONE

		if (timeout == 0) {
			return Status::Error;
		}

		uint8_t msb = 0, lsb = 0;
		status = this->ReadRegister(Register::OTP_DATA_MSB_REG, msb);
		if(status == Status::Ok) {
			status = this->ReadRegister(Register::OTP_DATA_LSB_REG, lsb);
		} 
		if(status != Status::Ok) {
			return status;
		}

		this->otpData[i] = (static_cast<uint16_t>(msb) << 8) | lsb;
	}

	// Now parse raw OTP data to compensation struct
	uint16_t offX_lsbMsb = this->otpData[0x0E] & 0x0FFF;
	uint16_t offY_lsbMsb = ((this->otpData[0x0E] & 0xF000) >> 4) + (this->otpData[0x0F] & 0x00FF);
	uint16_t offZ_lsbMsb = (this->otpData[0x0F] & 0x0F00) + (this->otpData[0x10] & 0x00FF);
	uint16_t tOff = this->otpData[0x0D] & 0x00FF;

	this->compData.offsetX = static_cast<float>(SignExtend(offX_lsbMsb, 12));
	this->compData.offsetY = static_cast<float>(SignExtend(offY_lsbMsb, 12));
	this->compData.offsetZ = static_cast<float>(SignExtend(offZ_lsbMsb, 12));
	this->compData.tOffs   = static_cast<float>(SignExtend(tOff, 8)) / 5.0f;

	uint8_t sensX = (this->otpData[0x10] & 0xFF00) >> 8;
	uint8_t sensY = (this->otpData[0x11] & 0x00FF);
	uint8_t sensZ = (this->otpData[0x11] & 0xFF00) >> 8;
	uint8_t tSens = (this->otpData[0x0D] & 0xFF00) >> 8;

	this->compData.sensX = static_cast<float>(SignExtend(sensX, 8)) / 256.0f;
	this->compData.sensY = static_cast<float>(SignExtend(sensY, 8)) / 256.0f;
	this->compData.sensZ = static_cast<float>(SignExtend(sensZ, 8)) / 256.0f;
	this->compData.tSens = static_cast<float>(SignExtend(tSens, 8)) / 512.0f;

	uint8_t tcoX = (this->otpData[0x12] & 0x00FF);
	uint8_t tcoY = (this->otpData[0x13] & 0x00FF);
	uint8_t tcoZ = (this->otpData[0x14] & 0x00FF);

	this->compData.tcoX = static_cast<float>(SignExtend(tcoX, 8)) / 32.0f;
	this->compData.tcoY = static_cast<float>(SignExtend(tcoY, 8)) / 32.0f;
	this->compData.tcoZ = static_cast<float>(SignExtend(tcoZ, 8)) / 32.0f;

	uint8_t tcsX = (this->otpData[0x12] & 0xFF00) >> 8;
	uint8_t tcsY = (this->otpData[0x13] & 0xFF00) >> 8;
	uint8_t tcsZ = (this->otpData[0x14] & 0xFF00) >> 8;

	this->compData.tcsX = static_cast<float>(SignExtend(tcsX, 8)) / 16384.0f;
	this->compData.tcsY = static_cast<float>(SignExtend(tcsY, 8)) / 16384.0f;
	this->compData.tcsZ = static_cast<float>(SignExtend(tcsZ, 8)) / 16384.0f;

	this->compData.t0 = (static_cast<float>(SignExtend(this->otpData[0x18], 16)) / 512.0f) + 23.0f;

	uint8_t crossXY = (this->otpData[0x15] & 0x00FF);
	uint8_t crossYX = (this->otpData[0x15] & 0xFF00) >> 8;
	uint8_t crossZX = (this->otpData[0x16] & 0x00FF);
	uint8_t crossZY = (this->otpData[0x16] & 0xFF00) >> 8;

	this->compData.crossXY = static_cast<float>(SignExtend(crossXY, 8)) / 800.0f;
	this->compData.crossYX = static_cast<float>(SignExtend(crossYX, 8)) / 800.0f;
	this->compData.crossZX = static_cast<float>(SignExtend(crossZX, 8)) / 800.0f;
	this->compData.crossZY = static_cast<float>(SignExtend(crossZY, 8)) / 800.0f;

	return Status::Ok;
}

float BMM350::ParseMagAxis(uint8_t msb, uint8_t lsb, uint8_t xlsb, float offset) {
	int32_t rawValue = static_cast<int32_t>((msb << 24) | (lsb << 16) | (xlsb << 8)) >> 8;
	return (static_cast<float>(rawValue) * this->magSens) - offset;
}

float BMM350::ParseTemperature(uint8_t msb, uint8_t lsb, uint8_t xlsb) {
	int32_t rawValue = static_cast<int32_t>((msb << 24) | (lsb << 16) | (xlsb << 8)) >> 8;
	return (static_cast<float>(rawValue) * this->tempSens) - this->tempOffset;
}

Status BMM350::WriteRegister(Register reg, uint8_t value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	this->buffer[1] = (value) & 0xFF;
	Status status = this->bus.TransferAsync(this->addr, this->buffer, 2, nullptr, 0);
	if(status != Status::Ok) {
		return status;
	}
	
	return this->bus.TransferWait(TX_WAIT_FOREVER);
}

Status BMM350::ReadRegister(Register reg, uint8_t& value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	Status status = this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 1);
	if(status != Status::Ok) {
		return status;
	}

	status = this->bus.TransferWait(TX_WAIT_FOREVER);
	value = this->buffer[0];

	return status;
}

Status BMM350::ModifyRegister(Register reg, uint8_t mask, uint8_t value) {
	uint8_t tmp;

	Status status = this->ReadRegister(reg, tmp);
	if(status != Status::Ok) {
		return status;
	}

	tmp &= ~mask;
	tmp |= value;

	return this->WriteRegister(reg, tmp);
}