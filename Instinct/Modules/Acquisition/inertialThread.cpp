#include "inertialThread.hpp"

#include <stdio.h>

// Allocate the static variables
TX_THREAD InertialThread::threadPtr;
uint8_t InertialThread::threadStack[4096];
TX_EVENT_FLAGS_GROUP InertialThread::event;

// Global/Shared timestamp for the EKF
volatile uint32_t imuTimestampUs = 0;

static Topic<ImuMsg> topicImu("imu", static_cast<uint8_t>(TopicID::Imu));

void InertialThread::OnboardIntCallback(void* context, EXTIManager::Edge edge) {
	// Get current sample timestamp, lowest latency/jitter
	// imuTimestampUs = Time::GetUs();

	// Wake up the Inertial Thread
	// tx_event_flags_set(&event, EVT_EXT2_IMU_DRDY, TX_OR);
}

void InertialThread::Ext2IntCallback(void* context, EXTIManager::Edge edge) {
	// Get current sample timestamp, lowest latency/jitter
	imuTimestampUs = Time::GetUs();

	// Wake up the Inertial Thread
	tx_event_flags_set(&event, EVT_EXT2_IMU_DRDY, TX_OR);
}

void InertialThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("ACQ_Inert"),
											InertialThread::Run,
											0,
											threadStack,
											sizeof(threadStack),
											0,
											0,
											TX_NO_TIME_SLICE,
											TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX ACQ Inert Thread Create Failed.");
	}

	Broker::RegisterTopic(&topicImu);
}

void InertialThread::Run(ULONG input) {
	ImuMsg imuInt;

	LOG_INFO("Inertial Thread Initialized.");

	// Create RTOS objects
	if(tx_event_flags_create(&event, const_cast<char*>("IMU_event")) != TX_SUCCESS) {
		return;
	}

	// Power up IMUs (enable LDOs)
	onboardIMUPwEn.Write(1);
	ext1IMUPwEn.Write(1);
	ext2IMUPwEn.Write(1);
	tx_thread_sleep(50);

	// Onboard IMU config
	LSM6DSO::Config lsmCfg = {
		.accelScale = LSM6DSO::AccelScale::G16,
		.gyroScale = LSM6DSO::GyroScale::DPS2000,
		.accelOdr = LSM6DSO::SampleRate::Hz26,
		.gyroOdr = LSM6DSO::SampleRate::Hz26
	};

	if(onboardIMU.Init(lsmCfg) == Status::Ok) {
		if(EXTIManager::RegisterCallback(onboardIMUInt.GetPinIndex(), OnboardIntCallback, nullptr) == Status::Ok) {
			// Enable the EXTI interrupt
			onboardIMUInt.EnableIRQ(GPIO::Interrupt::Rising, 0x0E);
			LOG_INFO("Onboard IMU (LSM6DSO) Init & EXTI Routed OK");
		}
		else {
			LOG_WARN("Onboard IMU EXTI Routing Failed!");
		}
	}
	else {
		LOG_WARN("Onboard IMU (LSM6DSO) Init Failed!");
	}

	// External/offboard IMU 2 config
	ICM45686::Config icmCfg = {
		.accelScale = ICM45686::AccelScale::G16,
		.gyroScale = ICM45686::GyroScale::DPS2000,
		.accelOdr = ICM45686::OutputDataRate::Hz25,
		.gyroOdr = ICM45686::OutputDataRate::Hz25
	};

	if(ext2IMU.Init(icmCfg) == Status::Ok) {
		if(EXTIManager::RegisterCallback(ext2IMUInt.GetPinIndex(), Ext2IntCallback, nullptr) == Status::Ok) {
			// Enable the EXTI interrupt
			ext2IMUInt.EnableIRQ(GPIO::Interrupt::Rising, 0x0E);
			LOG_INFO("Ext2 IMU (ICM45686) Init & EXTI Routed OK");
		}
		else {
			LOG_WARN("Ext2 IMU EXTI Routing Failed!");
		}
	}
	else {
		LOG_WARN("Ext2 IMU (ICM45686) Init Failed!");
	}

	ULONG events;
	UINT status;
	uint8_t reqICM = 0x00;
	while(1) {
		// Wait for event
		status = tx_event_flags_get(&event, EVT_EXT2_IMU_DRDY, TX_OR_CLEAR, &events, 1000);
		
		reqICM = 0x00;
		if(status == TX_SUCCESS && (events & EVT_EXT2_IMU_DRDY) == EVT_EXT2_IMU_DRDY) {
			// Get new data from IMU
			ext2IMU.RequestData();
			reqICM = 0x01;

			ledRed.Toggle();
		}
		else if(status != TX_SUCCESS) {
			// Handle sensor timeout (e.g., attempt software reset or trigger failsafe)
			LOG_WARN("IMU Interrupt Timeout!");
		}

		// Wait for all called/triggered requests
		if(reqICM == 0x01) {
			imuInt.timestamp = imuTimestampUs;
			ext2IMU.GetData(imuInt.accel, imuInt.gyro, &imuInt.temperature);
			topicImu.Publish(imuInt);
		}
	}
}