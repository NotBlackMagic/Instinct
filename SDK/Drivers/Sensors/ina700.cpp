#include "ina700.hpp"

void INA700::Init() {
	// this->WriteRegister(INA700::Register::ADC_CONFIG, 0xFB6F);
}

void INA700::ReadID(uint16_t& id) {
	this->ReadRegister(INA700::Register::MANUFACTURER_ID, id);
}

void INA700::ReadVoltage(float& value) {
	uint16_t data;
	this->ReadRegister(INA700::Register::VBUS, data);
	value = (int16_t)data * 0.003125f;
}

void INA700::ReadCurrent(float& value) {
	uint16_t data;
	this->ReadRegister(INA700::Register::CURRENT, data);
	value = (int16_t)data * 0.000480f;
}

void INA700::ReadTemperature(float& value) {
	uint16_t data;
	this->ReadRegister(INA700::Register::DIETEMP, data);
	value = (int16_t)data * 0.0078125;
}

void INA700::WriteRegister(Register reg, uint16_t value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	this->buffer[1] = (value >> 8) & 0xFF;
	this->buffer[2] = (value) & 0xFF;
	this->bus.TransferAsync(this->addr, this->buffer, 3, nullptr, 0);
	this->bus.TransferWait(TX_WAIT_FOREVER);
}

void INA700::ReadRegister(Register reg, uint16_t& value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 2);
	this->bus.TransferWait(TX_WAIT_FOREVER);
	value = (this->buffer[0] << 8) + this->buffer[1];
}