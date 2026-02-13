#include "auxiliaryThread.hpp"

TX_THREAD AuxiliaryThread::threadPtr;
uint8_t AuxiliaryThread::threadStack[4096];

void AuxiliaryThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("ACQ_Aux"),
											AuxiliaryThread::Run,
											0,
											threadStack,
											sizeof(threadStack),
											0,
											0,
											TX_NO_TIME_SLICE,
											TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX ACQ Aux Thread Create Failed.");
	}
}

void AuxiliaryThread::Run(ULONG input) {
	BaroMsg baroExt;
	MagMsg magInt;

	LOG_INFO("Auxiliary Thread Initialized.");

	volatile uint8_t probeI2C1 = i2c1.Probe(0x1E);
	volatile uint8_t probeI2C4 = i2c4.Probe(0x47);

	//Initialize
	if(i2c1.Probe(0x1E) == 0x01) {
		LOG_INFO("LIS2MDL I2C1 Probe Pass");
		uint8_t lis2mdlID;
		lis2mdl.ReadID(lis2mdlID);
		if(lis2mdlID == 0x40) {
			lis2mdl.Init();
			LOG_INFO("LIS2MDL Initialized");
		}
	}

	if(i2c4.Probe(0x47) == 0x01) {
		LOG_INFO("BMP581 I2C4 Probe Pass");
		uint8_t bmp581ID;
		bmp581.ReadID(bmp581ID);
		if(bmp581ID == 0x50) {
			bmp581.Init();
			LOG_INFO("BMP581 Initialized");
		}
	}

	while(1) {
		lis2mdl.RequestData();
		bmp581.RequestData();

		bmp581.GetData(baroExt.pressure, baroExt.temperature);
		topicBaro.Publish(baroExt);

		lis2mdl.GetData(magInt.values, &magInt.temperature);
		lis2mdl.ReadTemperature(magInt.temperature);
		topicMag.Publish(magInt);

		tx_thread_sleep(200);
	}
}