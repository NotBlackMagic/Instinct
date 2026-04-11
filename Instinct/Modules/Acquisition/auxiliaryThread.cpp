#include "auxiliaryThread.hpp"

TX_THREAD AuxiliaryThread::threadPtr;
uint8_t AuxiliaryThread::threadStack[4096];

static Topic<BaroMsg> topicBaroInt("baroInt", static_cast<uint8_t>(TopicID::Baro), 0);
static Topic<BaroMsg> topicBaroExt("baroExt", static_cast<uint8_t>(TopicID::Baro), 0);
static Topic<MagMsg> topicMagInt("magInt", static_cast<uint8_t>(TopicID::Mag), 1);
static Topic<MagMsg> topicMagExt("magExt", static_cast<uint8_t>(TopicID::Mag), 1);

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

	Broker::RegisterTopic(&topicBaroInt);
	Broker::RegisterTopic(&topicBaroExt);
	Broker::RegisterTopic(&topicMagInt);
	Broker::RegisterTopic(&topicMagExt);
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
		onboardMag.RequestData();
		extMag.RequestData();

		// Wait for all called/triggered requests
		magInt.timestamp = Time::GetUs();
		onboardMag.GetData(magInt.values, &magInt.temperature);
		onboardMag.ReadTemperature(magInt.temperature);
		topicMagInt.Publish(magInt);

		// Wait for all called/triggered requests
		magExt.timestamp = Time::GetUs();
		extMag.GetData(magExt.values, &magExt.temperature);
		topicMagExt.Publish(magExt);

		// Start request of sensors on shared buse from previous request
		onboardBaro.RequestData();
		extBaro.RequestData();

		// Wait for all called/triggered requests
		baroInt.timestamp = Time::GetUs();
		onboardBaro.GetData(&baroInt.pressure, &baroInt.temperature);
		topicBaroInt.Publish(baroInt);

		baroExt.timestamp = Time::GetUs();
		extBaro.GetData(&baroExt.pressure, &baroExt.temperature);
		topicBaroExt.Publish(baroExt);

		// Match set ODR rates of 25Hz
		tx_thread_sleep(40);
	}
}