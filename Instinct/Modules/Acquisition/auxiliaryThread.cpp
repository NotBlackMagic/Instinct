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

	// Initialize
	if(i2c1.Probe(0x1E) == 0x01) {
		uint8_t lis2mdlID;
		onboardMag.ReadID(lis2mdlID);
		if(lis2mdlID == 0x40) {
			onboardMag.Init({});
			LOG_INFO("LIS2MDL Init OK");
		}
		else {
			LOG_WARN("LIS2MDL Init Failed!");
		}
	}
	else {
		LOG_WARN("LIS2MDL not found on I2C1!");
	}

	// External/offboard Barometer config
	if(i2c4.Probe(0x47) == 0x01) {
		// Recommended settings based on datasheet for "High resolution"
		BMP581::Config config = {
			.odr = BMP581::OutputDataRate::Hz25,
			.osrPressure = BMP581::Oversampling::X64,
			.osrTemp = BMP581::Oversampling::X4,
			.iirFilter = BMP581::IIRFilter::Bypass
		};
		if(extBaro.Init(config) != Status::Ok) {
			LOG_INFO("Ext Baro (BMP581) Init OK");
		}
		else {
			LOG_INFO("Ext Baro (BMP581) Init Failed!");
		}
	}
	else {
		LOG_WARN("Ext Baro (BMP581) not found on I2C4!");
	}

	if(i2c4.Probe(0x14) == 0x01) {
		uint8_t bmm350ID;
		// bmm350.ReadID(bmm350ID);
		if(bmm350ID == 0x50) {
			// bmm350.Init();
			LOG_INFO("BMM350 Initialized");
		}
		else {
			LOG_WARN("BMM350 Init Failed!");
		}
	}
	else {
		LOG_WARN("BMM350 not found on I2C4!");
	}

	while(1) {
		onboardMag.RequestData();
		extBaro.RequestData();

		extBaro.GetData(&baroExt.pressure, &baroExt.temperature);
		topicBaro.Publish(baroExt);

		onboardMag.GetData(magInt.values, &magInt.temperature);
		onboardMag.ReadTemperature(magInt.temperature);
		topicMag.Publish(magInt);

		// Match set ODR rates of 25Hz
		tx_thread_sleep(40);
	}
}