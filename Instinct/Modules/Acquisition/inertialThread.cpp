#include "inertialThread.hpp"

#include <stdio.h>

TX_THREAD InertialThread::threadPtr;
uint8_t InertialThread::threadStack[4096];

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
}

void InertialThread::Run(ULONG input) {
	ImuMsg imuInt;

	LOG_INFO("Inertial Thread Initialized.");

	//Initialize
	lsm6dso.Init();
	icm45686.Init();

	uint8_t lsm6dsoID;
	lsm6dso.ReadID(lsm6dsoID);
	LOG_INFO("LSM ID: 0x%02X [0x6C]", lsm6dsoID);

	uint8_t icm45686ID;
	icm45686.ReadID(icm45686ID);
	LOG_INFO("ICM ID: 0x%02X [0xE9]", icm45686ID);

	while(1) {
		lsm6dso.RequestData();

		lsm6dso.GetData(imuInt.accel, imuInt.gyro, &imuInt.temperature);
		topicImu.Publish(imuInt);

		tx_thread_sleep(200);
	}
}