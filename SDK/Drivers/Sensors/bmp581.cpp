#include "bmp581.hpp"
#include "status.hpp"

void BMP581::Init() {
	this->WriteRegister(BMP581::Register::OSR_CONFIG, 0x7B);	//Enable Pressure measurements, OSR Pressure x128, OSR Temperature x8
	this->WriteRegister(BMP581::Register::ODR_CONFIG, 0x5D);	//Power up: Normal mode, ODR = 10 Hz
}

void BMP581::ReadID(uint8_t& id) {
	//Chip ID: 0x50
	this->ReadRegister(BMP581::Register::CHIP_ID, id);
}

bool BMP581::RequestData() {
	this->buffer[0] = static_cast<uint8_t>(BMP581::Register::TEMP_DATA_XLSB);
	if(this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 6) == Status::Ok) {
		return true;
	}
	else {
		return false;
	}
}

bool BMP581::GetData(float& pressure, float&  temp) {
	if(this->bus.TransferWait(TX_WAIT_FOREVER) != Status::Ok) {
		return false;
	}

	int32_t rawTemp = (int32_t)((this->buffer[2] << 16) + (this->buffer[1] << 8) + this->buffer[0]);
	int32_t rawPress = (int32_t)((this->buffer[5] << 16) + (this->buffer[4] << 8) + this->buffer[3]);

	temp = rawTemp * (1.0f/65536);
	pressure = rawPress * (1.0f/64);

	return true;
}

void BMP581::WriteRegister(Register reg, uint8_t value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	this->buffer[1] = (value) & 0xFF;
	this->bus.TransferAsync(this->addr, this->buffer, 2, nullptr, 0);
	this->bus.TransferWait(TX_WAIT_FOREVER);
}

void BMP581::ReadRegister(Register reg, uint8_t& value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	this->bus.TransferAsync(this->addr, this->buffer, 1, this->buffer, 1);
	this->bus.TransferWait(TX_WAIT_FOREVER);
	value = this->buffer[0];
}