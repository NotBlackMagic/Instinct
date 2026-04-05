#include "lsm6dso.hpp"

void LSM6DSO::Init() {
	//Start with RESET command
	this->WriteRegister(LSM6DSO::Register::CTRL3_C, 0x01);
	tx_thread_sleep(1000);

	uint8_t regVal = 0;
	//Configure base configs like interfaces, interrupts, etc
	this->WriteRegister(LSM6DSO::Register::CTRL3_C, 0x04);			//Automatic increment register address
	this->WriteRegister(LSM6DSO::Register::CTRL9_XL, 0x02);			//Disable I3C Interface
	this->WriteRegister(LSM6DSO::Register::CTRL4_C, 0x04);			//Disable I2C Interface
	this->WriteRegister(LSM6DSO::Register::COUNTER_BDR_REG1, 0x80);	//Enable pulsed data-ready mode
	this->WriteRegister(LSM6DSO::Register::INT1_CTRL, 0x03);		//INT1 outputs Accelerometer and Gyro data-ready interrupt

	//Configure Accelerometer Chain
	uint8_t sampleRate = static_cast<uint8_t>(LSM6DSO::SampleRate::SampleRate_833Hz);
	uint8_t scale = static_cast<uint8_t>(LSM6DSO::AccelFullScale::Scale_8g);
	uint8_t lpf = static_cast<uint8_t>(LSM6DSO::AccelLPF::LPF_ODR100);
	regVal = (sampleRate << 4) + (scale << 2) + 0x02;
	this->WriteRegister(LSM6DSO::Register::CTRL1_XL, regVal);
	regVal = (lpf << 5);
	this->WriteRegister(LSM6DSO::Register::CTRL8_XL, regVal);

	//Configure Gyroscope Chain
	sampleRate = static_cast<uint8_t>(LSM6DSO::SampleRate::SampleRate_833Hz);
	scale = static_cast<uint8_t>(LSM6DSO::GyroFullScale::Scale_1000dps);
	lpf = static_cast<uint8_t>(LSM6DSO::GyroLPF::LPFType_0);
	regVal = (sampleRate << 4) + (scale << 2);
	this->WriteRegister(LSM6DSO::Register::CTRL2_G, regVal);
	this->WriteRegister(LSM6DSO::Register::CTRL4_C, 0x06);
	this->WriteRegister(LSM6DSO::Register::CTRL6_C, (lpf - 1));
	this->WriteRegister(LSM6DSO::Register::CTRL7_G, 0x00);

	//Configure FIFO
	//this->WriteRegister(LSM6DSO::Register::FIFO_CTRL1, 0x01);	//Set FIFO TH to 1 sensor packet
	//this->WriteRegister(LSM6DSO::Register::FIFO_CTRL3, 0x00);	//Accel BDR: 833 Hz; Gyro BDR: 833 Hz
	//this->WriteRegister(LSM6DSO::Register::FIFO_CTRL4, 0x00);	//Disable/bypass FIFO mode (Will be enabled when EXTI INT is ready/set-up)

	//Wait for LSM6DSO LPF stabilization: around 9 * Ts = 9*1/8.33 = 1080 ms (within 0.01% of final value)
	tx_thread_sleep(1080);
}

void LSM6DSO::ReadID(uint8_t& id) {
	this->ReadRegister(LSM6DSO::Register::WHO_AM_I, id);
}

void LSM6DSO::ReadStatus(uint8_t& status) {
	this->ReadRegister(LSM6DSO::Register::STATUS_REG, status);
}

bool LSM6DSO::RequestData() {
	this->buffer[0] = 0x80 | static_cast<uint8_t>(LSM6DSO::Register::OUT_TEMP_L);
	if(this->bus.TransferAsync(this->buffer, this->buffer, 15) == Status::Ok) {
		return true;
	}
	else {
		return true;
	}
}

bool LSM6DSO::GetData(float* accel, float* gyro, float* temp) {
	if(this->bus.TransferWait(TX_WAIT_FOREVER) != Status::Ok) {
		return false;
	}

	int16_t rawTemp = (int16_t)((this->buffer[2] << 8) + this->buffer[1]);
	int16_t rawGX = (int16_t)((this->buffer[4] << 8) + this->buffer[3]);
	int16_t rawGY = (int16_t)((this->buffer[6] << 8) + this->buffer[5]);
	int16_t rawGZ = (int16_t)((this->buffer[8] << 8) + this->buffer[7]);
	int16_t rawAX = (int16_t)((this->buffer[10] << 8) + this->buffer[9]);
	int16_t rawAY = (int16_t)((this->buffer[12] << 8) + this->buffer[11]);
	int16_t rawAZ = (int16_t)((this->buffer[14] << 8) + this->buffer[13]);

	*temp = rawTemp * LSM6DSO::tempSens + LSM6DSO::tempOffset;

	float scaling = LSM6DSO::gyroSens[static_cast<uint8_t>(LSM6DSO::GyroFullScale::Scale_1000dps)];
	gyro[0] = rawGX * scaling * 0.001f;
	gyro[1] = rawGY * scaling * 0.001f;
	gyro[2] = rawGZ * scaling * 0.001f;

	scaling = LSM6DSO::accelSens[static_cast<uint8_t>(LSM6DSO::AccelFullScale::Scale_8g)];
	accel[0] = rawAX * scaling * 0.001f;
	accel[1] = rawAY * scaling * 0.001f;
	accel[2] = rawAZ * scaling * 0.001f;

	return true;
}

void LSM6DSO::WriteRegister(Register reg, uint8_t value) {
	this->buffer[0] = static_cast<uint8_t>(reg);
	this->buffer[1] = value;
	this->bus.TransferAsync(this->buffer, this->buffer, 2);
	this->bus.TransferWait(TX_WAIT_FOREVER);
}

void LSM6DSO::ReadRegister(Register reg, uint8_t& value) {
	this->buffer[0] = 0x80 | static_cast<uint8_t>(reg);
	this->bus.TransferAsync(this->buffer, this->buffer, 2);
	this->bus.TransferWait(TX_WAIT_FOREVER);
	value = this->buffer[1];
}