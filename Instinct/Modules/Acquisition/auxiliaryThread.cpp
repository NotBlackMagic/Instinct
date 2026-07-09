#include "auxiliaryThread.hpp"

TX_THREAD AuxiliaryThread::threadPtr;
uint8_t AuxiliaryThread::threadStack[4096];

Topic<BaroMsg> AuxiliaryThread::topicBaro[2] = {Topic<BaroMsg>("baro", static_cast<uint8_t>(TopicID::Baro), 0),
												Topic<BaroMsg>("baro", static_cast<uint8_t>(TopicID::Baro), 1)};
Topic<MagMsg> AuxiliaryThread::topicMag[2] = {	Topic<MagMsg>("mag", static_cast<uint8_t>(TopicID::Mag), 0),
												Topic<MagMsg>("mag", static_cast<uint8_t>(TopicID::Mag), 1)};

void AuxiliaryThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("ACQ_Aux"),
											AuxiliaryThread::Run, 0,
											threadStack, sizeof(threadStack),
											4, 0,
											TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX ACQ Aux Thread Create Failed.");
	}

	Broker::RegisterTopic(&topicBaro[0]);
	Broker::RegisterTopic(&topicBaro[1]);
	Broker::RegisterTopic(&topicMag[0]);
	Broker::RegisterTopic(&topicMag[1]);
}

void AuxiliaryThread::Run(ULONG input) {
	BaroMsg baroInt;
	BaroMsg baroExt;
	MagMsg magInt;
	MagMsg magExt;

	LOG_INFO("Auxiliary Thread Initialized.");

	volatile uint8_t probeI2C1 = i2c1.Probe(0x1E);
	volatile uint8_t probeI2C4 = i2c4.Probe(0x47);

	// Onboard Barometer config
	if(i2c1.Probe(0x63) == 0x01) {
		ICP20100::Config config = {
			.odr = ICP20100::OutputDataRate::Hz25
		};
		if(onboardBaro.Init(config) == Status::Ok) {
			LOG_INFO("Int Baro (ICP20100) Init OK");
		}
		else {
			LOG_WARN("Int Baro (ICP20100) Init Failed!");
		}
	}
	else {
		LOG_WARN("Int Baro (ICP20100) not found on I2C1!");
	}

	// Onboard Magnetometer config
	if(i2c1.Probe(0x1E) == 0x01) {
		LIS2MDL::Config config = {
			.odr = LIS2MDL::OutputDataRate::Hz50,
			.enableLPF = true
		};
		if(onboardMag.Init(config) == Status::Ok) {
			LOG_INFO("Int Mag (LIS2MDL) Init OK");
		}
		else {
			LOG_WARN("Int Mag (LIS2MDL) Init Failed!");
		}
	}
	else {
		LOG_WARN("Int Mag (LIS2MDL) not found on I2C1!");
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
		if(extBaro.Init(config) == Status::Ok) {
			LOG_INFO("Ext Baro (BMP581) Init OK");
		}
		else {
			LOG_WARN("Ext Baro (BMP581) Init Failed!");
		}
	}
	else {
		LOG_WARN("Ext Baro (BMP581) not found on I2C4!");
	}

	// External/offboard Magnetometer config
	if(i2c4.Probe(0x14) == 0x01) {
		BMM350::Config config = {
			.odr = BMM350::OutputDataRate::Hz25,
			.avg = BMM350::Averaging::Avg8
		};
		if(extMag.Init(config) == Status::Ok) {
			LOG_INFO("Ext Mag (BMM350) Init OK");
		}
		else {
			LOG_WARN("Ext Mag (BMM350) Init Failed!");
		}
	}
	else {
		LOG_WARN("Ext Mag (BMM350) not found on I2C4!");
	}

	while(1) {
		// Start request of sensors on DIFFERENT buses
		magInt.timestamp = Time::GetUs();
		magExt.timestamp = Time::GetUs();
		onboardMag.RequestData();
		extMag.RequestData();

		// Wait for all called/triggered requests
		onboardMag.GetData(magInt.values, &magInt.temperature);
		onboardMag.ReadTemperature(magInt.temperature);
		// Convert from sensor coordinate frame to FRD/NED
		// float tmp = magInt.values[0];
		// magInt.values[0] = -magInt.values[1];
		// magInt.values[1] = -tmp;
		// magInt.values[2] = -magInt.values[2];
		topicMag[0].Publish(magInt);

		// Wait for all called/triggered requests
		extMag.GetData(magExt.values, &magExt.temperature);
		// Convert from sensor coordinate frame to FRD/NED
		// tmp = extMag.values[0];
		// extMag.values[0] = -extMag.values[1];
		// extMag.values[1] = -tmp;
		// extMag.values[2] = -extMag.values[2];
		topicMag[1].Publish(magExt);

		// Start request of sensors on shared buse from previous request
		baroInt.timestamp = Time::GetUs();
		baroExt.timestamp = Time::GetUs();
		onboardBaro.RequestData();
		extBaro.RequestData();

		// Wait for all called/triggered requests
		onboardBaro.GetData(&baroInt.pressure, &baroInt.temperature);
		topicBaro[0].Publish(baroInt);

		extBaro.GetData(&baroExt.pressure, &baroExt.temperature);
		topicBaro[1].Publish(baroExt);

		// Match set ODR rates of 25Hz
		tx_thread_sleep(40);
	}
}