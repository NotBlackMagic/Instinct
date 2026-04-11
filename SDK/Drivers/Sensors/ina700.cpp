#include "ina700.hpp"

Status INA700::Init(const Config& config) {
	this->config = config;

	// Verify ID
	uint16_t id;
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
	status = this->WriteRegister(INA700::Register::ADC_CONFIG, 0xFB68);
	if(status != Status::Ok) {
		return status;
	}

	return Status::Ok;
}

Status INA700::Reset() {
	Status status = this->WriteRegister(INA700::Register::CONFIG, 0x80);
	if(status != Status::Ok) {
		return status;
	}

	// Wait for reset, about 2ms
	tx_thread_sleep(2);

	return Status::Ok;
}

Status INA700::ReadID(uint16_t& id) {
	// Chip ID: 0x5449
	return this->ReadRegister(INA700::Register::MANUFACTURER_ID, id);
}

// Status INA700::RequestData() {
// 	this->buffer[0] = static_cast<uint8_t>(INA700::Register::VBUS);
// 	if(this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 6) == Status::Ok) {
// 		return Status::Ok;
// 	}
// 	else {
// 		return Status::Error;
// 	}
// }

// Status INA700::GetData(float& volt, float& curr, float& temp) {
// 	if(this->bus.TransferWait(TX_WAIT_FOREVER) != Status::Ok) {
// 		return Status::Error;
// 	}

// 	uint16_t rawVolt = static_cast<uint16_t>((this->buffer[0] << 8) | this->buffer[1]);
// 	volt = rawVolt * this->voltSens;

// 	uint16_t rawTemp = static_cast<uint16_t>((this->buffer[2] << 8) | this->buffer[3]);
// 	temp = rawTemp * this->tempSens;

// 	uint16_t rawCurr = static_cast<uint16_t>((this->buffer[4] << 8) | this->buffer[5]);
// 	curr = rawCurr * this->currSens;

// 	return Status::Ok;
// }

Status INA700::ReadVoltage(float& value) {
	uint16_t raw;
	Status status = this->ReadRegister(INA700::Register::VBUS, raw);
	value = raw * this->voltSens;
	return status;
}

Status INA700::ReadCurrent(float& value) {
	uint16_t raw;
	Status status = this->ReadRegister(INA700::Register::CURRENT, raw);
	value = raw * this->currSens;
	return status;
}

Status INA700::ReadTemperature(float& value) {
	uint16_t raw;
	Status status = this->ReadRegister(INA700::Register::DIETEMP, raw);
	value = raw * this->tempSens;
	return status;
}

Status INA700::WriteRegister(Register reg, uint16_t value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	this->buffer[1] = (value >> 8) & 0xFF;
	this->buffer[2] = (value) & 0xFF;
	Status status = this->bus.TransferAsync(this->addr, this->buffer, 3, nullptr, 0);
	if(status != Status::Ok) {
		return status;
	}
	
	return this->bus.TransferWait(TX_WAIT_FOREVER);
}

Status INA700::ReadRegister(Register reg, uint16_t& value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	Status status = this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 2);
	if(status != Status::Ok) {
		return status;
	}

	status = this->bus.TransferWait(TX_WAIT_FOREVER);
	value = (this->buffer[0] << 8) + this->buffer[1];

	return status;
}

Status INA700::ModifyRegister(Register reg, uint16_t mask, uint16_t value) {
	uint16_t tmp;

	Status status = this->ReadRegister(reg, tmp);
	if(status != Status::Ok) {
		return status;
	}

	tmp &= ~mask;
	tmp |= value;

	return this->WriteRegister(reg, tmp);
}