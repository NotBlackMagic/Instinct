#include "icm45686.hpp"

void ICM45686::Init() {
}

void ICM45686::ReadID(uint8_t& id) {
	this->ReadRegister(ICM45686::Register::WHO_AM_I, id);
}

void ICM45686::WriteRegister(Register reg, uint8_t value) {
	uint8_t txData[2];
	uint8_t rxData[2];
	txData[0] = static_cast<uint8_t>(reg);
	txData[1] = (value) & 0xFF;
	this->bus.TransferAsync(txData, rxData, 2);
	this->bus.TransferWait(TX_WAIT_FOREVER);
}

void ICM45686::ReadRegister(Register reg, uint8_t& value) {
	uint8_t txData[2];
	uint8_t rxData[2];
	txData[0] = 0x80 | static_cast<uint8_t>(reg);
	this->bus.TransferAsync(txData, rxData, 2);
	this->bus.TransferWait(TX_WAIT_FOREVER);
	value = rxData[1];
}

void ICM45686::ModifyRegister(Register reg, uint8_t mask, uint8_t value) {
	uint8_t tmp;
	this->ReadRegister(reg, tmp);
	tmp &= ~mask;
	tmp |= value;
	this->WriteRegister(reg, tmp);
}