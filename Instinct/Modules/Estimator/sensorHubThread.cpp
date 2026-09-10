/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Estimation/sensorHubThread.cpp
 */

#include "sensorHubThread.hpp"

#include <math.h>

TX_THREAD SensorHubThread::threadPtr;
uint8_t SensorHubThread::threadStack[8192];

// Virtual Publishers
Topic<IMUMsg> SensorHubThread::topicVirtualIMU("imu", static_cast<uint8_t>(TopicID::Imu), SensorHubThread::virtualInstanceID);
Topic<MagMsg> SensorHubThread::topicVirtualMag("mag", static_cast<uint8_t>(TopicID::Mag), SensorHubThread::virtualInstanceID);
Topic<BaroMsg> SensorHubThread::topicVirtualBaro("baro", static_cast<uint8_t>(TopicID::Baro), SensorHubThread::virtualInstanceID);

// Subscribers
Subscriber<IMUMsg> SensorHubThread::subIMU[SensorHubThread::maxIMUs];
Subscriber<MagMsg> SensorHubThread::subMag[SensorHubThread::maxMags];
Subscriber<BaroMsg> SensorHubThread::subBaro[SensorHubThread::maxBaros];

Topic<IMUMsg>* SensorHubThread::topicsIMU[SensorHubThread::maxIMUs] = {nullptr};
Topic<MagMsg>* SensorHubThread::topicsMag[SensorHubThread::maxMags] = {nullptr};
Topic<BaroMsg>* SensorHubThread::topicsBaro[SensorHubThread::maxBaros] = {nullptr};

// Tracking & Health
uint8_t SensorHubThread::activeIMUCount = 0;
uint8_t SensorHubThread::activeMagCount = 0;
uint8_t SensorHubThread::activeBaroCount = 0;
uint64_t SensorHubThread::lastIMUTime[SensorHubThread::maxIMUs] = {0};

// State Tracking
float SensorHubThread::imuMean[SensorHubThread::maxIMUs][3] = {0};
float SensorHubThread::imuVariance[SensorHubThread::maxIMUs][3] = {0};
float SensorHubThread::lastPressure[SensorHubThread::maxBaros] = {0};
uint64_t SensorHubThread::lastBaroTimeUs[SensorHubThread::maxBaros] = {0};

bool SensorHubThread::imuHealth[SensorHubThread::maxIMUs] = {false, false, false};
bool SensorHubThread::magHealth[SensorHubThread::maxMags] = {false, false};
bool SensorHubThread::baroHealth[SensorHubThread::maxBaros] = {false, false};

void SensorHubThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr,	const_cast<char*>("Sens_Hub"),
													SensorHubThread::Run, 0,
													threadStack, sizeof(threadStack),
													2, 0,
													TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Sensor Hub Thread Create Failed.");
	}

	// Register the virtual topics to the broker
	Broker::RegisterTopic(&topicVirtualIMU);
	Broker::RegisterTopic(&topicVirtualMag);
	Broker::RegisterTopic(&topicVirtualBaro);
}

void SensorHubThread::Run(ULONG input) {
	LOG_INFO("Sensor Hub Thread Initialized.");

	// Wait for all the sensor threads to initialize. They are lower priority so we ahve to wait here.
	tx_thread_sleep(1000);

	// Bind to available raw sensors
	// IMUs
	for(uint8_t i = 0; i < maxIMUs; i++) {
		Topic<IMUMsg>* topic = Broker::GetTopic<IMUMsg>(static_cast<uint8_t>(TopicID::Imu), i);
		if(topic != nullptr) {
			topicsIMU[activeIMUCount] = topic;
			topicsIMU[activeIMUCount]->Subscribe(&subIMU[activeIMUCount]);
			activeIMUCount++;
		}
	}
	// Magnetometers
	for(uint8_t i = 0; i < maxMags; i++) {
		Topic<MagMsg>* topic = Broker::GetTopic<MagMsg>(static_cast<uint8_t>(TopicID::Mag), i);
		if(topic != nullptr) {
			topicsMag[activeMagCount] = topic;
			topicsMag[activeMagCount]->Subscribe(&subMag[activeMagCount]);
			activeMagCount++;
		}
	}
	// Barometers
	for(uint8_t i = 0; i < maxBaros; i++) {
		Topic<BaroMsg>* topic = Broker::GetTopic<BaroMsg>(static_cast<uint8_t>(TopicID::Baro), i);
		if(topic != nullptr) {
			topicsBaro[activeBaroCount] = topic;
			topicsBaro[activeBaroCount]->Subscribe(&subBaro[activeBaroCount]);
			activeBaroCount++;
		}
	}

	if(activeIMUCount == 0) {
		LOG_ERR("Sensor Hub: No IMU Topic found!");
		return;
	}
	if(activeMagCount == 0) {
		LOG_WARN("Sensor Hub: No Mag Topic found!");
	}
	if(activeBaroCount == 0) {
		LOG_WARN("Sensor Hub: No Baro Topic found!");
	}

	uint8_t masterIMUIx = 0; 
	const ULONG imuTimeout = 100;

	IMUMsg imuMsgs[maxIMUs];
	MagMsg magMsgs[maxMags];
	BaroMsg baroMsgs[maxBaros];

	while(1) {
		// The sampling reference point: Specifically on IMU0 (consider this the master/main)
		if(topicsIMU[masterIMUIx] != nullptr && topicsIMU[masterIMUIx]->Take(&subIMU[masterIMUIx], imuMsgs[masterIMUIx], imuTimeout) == true) {
			// Use the Master IMU's timestamp as the synchronized "now" for the whole system
			uint64_t curSysTime = imuMsgs[masterIMUIx].timestamp;

			// Non-blocking read of the remaining IMUs using Peek
			for(uint8_t i = 0; i < activeIMUCount; i++) {
				if(i != masterIMUIx) {
					topicsIMU[i]->Peek(imuMsgs[i]);

					// Health Monitoring: 2000us (2ms) timeout threshold
					imuHealth[i] = SensorHubThread::CheckHealth(imuMsgs[i].timestamp, curSysTime, 2000);
				}
			}
			imuHealth[masterIMUIx] = true;

			// Blend and Publish Virtual IMU
			IMUMsg virtualImu = SensorHubThread::AggregateIMUs(imuMsgs, activeIMUCount);
			topicVirtualIMU.Publish(virtualImu);

			// Auxiliary Sensor Processing (Non-Blocking). If updated process it, else skip
			// Magnetometer update check
			bool magUpdated = false;
			for(uint8_t i = 0; i < activeMagCount; i++) {
				if(topicsMag[i] != nullptr && topicsMag[i]->Take(&subMag[i], magMsgs[i], TX_NO_WAIT) == true) {
					magUpdated = true;
				}

				// 100000us (100ms) timeout for low-rate Mags
				magHealth[i] = SensorHubThread::CheckHealth(magMsgs[i].timestamp, curSysTime, 100000);
			}

			if(magUpdated == true) {
				// MagMsg virtualMag = SelectMag(rawMagOn, rawMagOff);
				MagMsg virtualMag = SensorHubThread::SelectMag(magMsgs, activeMagCount);
				topicVirtualMag.Publish(magMsgs[0]);
			}

			// Barometer update check
			bool baroUpdated = false;
			for(uint8_t i = 0; i < activeBaroCount; i++) {
				if(topicsBaro[i] != nullptr && topicsBaro[i]->Take(&subBaro[i], baroMsgs[i], TX_NO_WAIT) == true) {
					baroUpdated = true;
				}

				// 100000us (100ms) timeout for low-rate Baros
				baroHealth[i] = SensorHubThread::CheckHealth(baroMsgs[i].timestamp, curSysTime, 100000);
			}

			if(baroUpdated == true) {
				// BaroMsg virtualBaro = BlendBaros(rawBaro1, rawBaro2);
				BaroMsg virtualBaro = SensorHubThread::AggregateBaros(baroMsgs, activeBaroCount);
				topicVirtualBaro.Publish(virtualBaro);
			}
		}
		else {
			// Master IMU timeout -> Hardware failure (probably)
			LOG_WARN("Sensor Hub: Master IMU %d lost!", masterIMUIx);
			imuHealth[masterIMUIx] = false;

			// Find next best living IMU
			bool newMaster = false;
			for(uint8_t i = 0; i < activeIMUCount; i++) {
				if(imuHealth[i] == true) {
					masterIMUIx = i;
					newMaster = true;
					LOG_WARN("Sensor Hub: Failover to IMU %d successful.", masterIMUIx);
					break;
				}
			}

			if(newMaster == false) {
				// No new master found, no valid/alive IMU. Catastrophic total failure. 
				// Trigger emergency motor kill / parachute deployment here.
				LOG_FATAL("Sensor Hub: ALL IMUs lost!");

				// Failsafe: All IMUs are dead.
				IMUMsg deadMsg{};
				deadMsg.timestamp = 0;
				topicVirtualIMU.Publish(deadMsg);

				while(1) {
					tx_thread_sleep(100);
				} 
			}
		}
	}
}

IMUMsg SensorHubThread::AggregateIMUs(IMUMsg* rawMsgs, uint8_t count) {
	if(count == 0) {
		IMUMsg deadMsg{}; 
		deadMsg.timestamp = 0;
		return deadMsg;
	}

	IMUMsg virtualIMU = rawMsgs[0];		// Get base stuff from master IMU (e.g. timestamp)

	// Spatial Translation (Move all IMUs to the CoG)
	float transAccel[maxIMUs][3];
	float transGyro[maxIMUs][3];
	for(uint8_t i = 0; i < count; i++) {
		if(imuHealth[i] == false) {
			continue;
		}

		// Gyro is invariant to translation on a rigid body
		transGyro[i][0] = rawMsgs[i].gyro[0];
		transGyro[i][1] = rawMsgs[i].gyro[1];
		transGyro[i][2] = rawMsgs[i].gyro[2];

		// Lever Arm Compensation for Accelerometer
		float r[3] = {imuOffsetCoG[i].x, imuOffsetCoG[i].y, imuOffsetCoG[i].z};
		
		// Calculate w x r
		float w_cross_r[3];
		CrossProduct(rawMsgs[i].gyro, r, w_cross_r);
		
		// Calculate w x (w x r), the centripetal acceleration
		float centripetal[3];
		CrossProduct(rawMsgs[i].gyro, w_cross_r, centripetal);

		// Note: Tangential acceleration (w_dot x r) is omitted here to avoid noise amplification. It can be added with a heavily filtered angular acceleration estimate.

		transAccel[i][0] = rawMsgs[i].accel[0] - centripetal[0];
		transAccel[i][1] = rawMsgs[i].accel[1] - centripetal[1];
		transAccel[i][2] = rawMsgs[i].accel[2] - centripetal[2];
	}

	// Dynamic Variance Calculation (EMV filter)
	float totalInvVar[3] = {0.0f, 0.0f, 0.0f};

	for(uint8_t i = 0; i < count; i++) {
		if(imuHealth[i] == false) {
			continue;
		}

		bool isNewData = false;
		if(rawMsgs[i].timestamp != lastIMUTime[i]) {
			isNewData = true;
		}
		lastIMUTime[i] = rawMsgs[i].timestamp;

		for(uint8_t axis = 0; axis < 3; axis++) {
			float currAccel = transAccel[i][axis];

			// Only update the statistical model if the data changed
			if(isNewData == true) {
				// Update running mean
				imuMean[i][axis] = (alpha * currAccel) + ((1.0f - alpha) * imuMean[i][axis]);
				
				// Update running variance
				float diff = currAccel - imuMean[i][axis];
				imuVariance[i][axis] = (alpha * diff * diff) + ((1.0f - alpha) * imuVariance[i][axis]);
				
				// Prevent division by zero if variance drops to absolute zero
				if(imuVariance[i][axis] < 1e-4f) {
					imuVariance[i][axis] = 1e-4f;
				}
			}
			
			totalInvVar[axis] += (1.0f / imuVariance[i][axis]);
		}
	}

	// Inverse-Variance Blending
	float finalAccel[3] = {0.0f, 0.0f, 0.0f};
	float finalGyro[3] = {0.0f, 0.0f, 0.0f};

	for (uint8_t i = 0; i < count; i++) {
		if(imuHealth[i] == false) {
			continue;
		}

		for (uint8_t axis = 0; axis < 3; axis++) {
			float weight = 0.0f;
			if(totalInvVar[axis] > 0.0f) {
				weight = (1.0f / imuVariance[i][axis]) / totalInvVar[axis];
			}
			
			finalAccel[axis] += transAccel[i][axis] * weight;
			finalGyro[axis] += transGyro[i][axis] * weight;
		}
	}

	// Apply the perfectly aggregated math back to the Virtual IMU payload
	virtualIMU.accel[0] = finalAccel[0];
	virtualIMU.accel[1] = finalAccel[1];
	virtualIMU.accel[2] = finalAccel[2];

	virtualIMU.gyro[0] = finalGyro[0];
	virtualIMU.gyro[1] = finalGyro[1];
	virtualIMU.gyro[2] = finalGyro[2];

	return virtualIMU;
}

BaroMsg SensorHubThread::AggregateBaros(BaroMsg* rawMsgs, uint8_t count) {
	if(count == 0) {
		BaroMsg deadMsg{};
		deadMsg.timestamp = 0;
		return deadMsg;
	}

	bool useSensor[maxBaros] = {false};
	float rates[maxBaros] = {0.0f};
	uint8_t validCount = 0;

	// Rate-of-Change Outlier Rejection
	for(uint8_t i = 0; i < count; i++) {
		if(baroHealth[i] == false) {
			continue;
		}

		float dt = 0.0f;
		if(lastBaroTimeUs[i] != 0) {
			dt = (float)(rawMsgs[i].timestamp - lastBaroTimeUs[i]) / 1000000.0f;
		}

		// Calculate rate of pressure change (Pascals per second)
		if(dt > 0.0f) {
			rates[i] = fabsf(rawMsgs[i].pressure - lastPressure[i]) / dt;
		}

		// Update state tracking for the next loop
		lastPressure[i] = rawMsgs[i].pressure;
		lastBaroTimeUs[i] = rawMsgs[i].timestamp;

		// If it's the first reading (dt=0) or the rate is physically realistic, it survives
		if(dt == 0.0f || rates[i] < maxPressRate) {
			useSensor[i] = true;
			validCount++;
		}
	}

	// Failsafe checks
	if(validCount == 0) {
		BaroMsg deadMsg{};
		deadMsg.timestamp = 0;
		return deadMsg;
	}

	if(validCount == 1) {
		for(uint8_t i = 0; i < count; i++) {
			if(useSensor[i]) {
				return rawMsgs[i];
			}
		}
	}

	// Threshold-Gating (Optimized for 2 Baros)
	if(count >= 2 && useSensor[0] == true && useSensor[1] == true) {
		float diff = fabsf(rawMsgs[0].pressure - rawMsgs[1].pressure);
		
		if(diff < baroAgreeThres) {
			// They agree! Average them to drop the noise floor by sqrt(2)
			BaroMsg blended = rawMsgs[0];
			blended.pressure = (rawMsgs[0].pressure + rawMsgs[1].pressure) * 0.5f;
			blended.temperature = (rawMsgs[0].temperature + rawMsgs[1].temperature) * 0.5f;
			return blended;
		}
		else {
			// They disagree, but both passed the rate check. 
			// The one with the LOWER rate of change is experiencing less turbulence. Pick it.
			uint8_t bestIndex = (rates[0] < rates[1]) ? 0 : 1;
			return rawMsgs[bestIndex];
		}
	}

	// Fallback for >2 baros edge cases (returns the first surviving sensor)
	for(uint8_t i = 0; i < count; i++) {
		if(useSensor[i] == true) {
			return rawMsgs[i];
		}
	}

	// Failsafe: All Baros are dead.
	BaroMsg deadMsg{};
	deadMsg.timestamp = 0;
	return deadMsg;
}

MagMsg SensorHubThread::SelectMag(MagMsg* rawMsgs, uint8_t count) {
	if(count == 0) {
		MagMsg deadMsg{}; 
		deadMsg.timestamp = 0;
		return deadMsg;
	}

	float bestError = 999999.0f;
	int8_t bestIndex = -1;
	for (uint8_t i = 0; i < count; i++) {
		if(magHealth[i] == false) {
			continue;
		}

		float x = rawMsgs[i].values[0];
		float y = rawMsgs[i].values[1];
		float z = rawMsgs[i].values[2];
		
		// Calculate the 3D Vector Magnitude (Norm)
		float norm = sqrtf(x*x + y*y + z*z);
		
		// Calculate how heavily distorted the field is
		float error = fabsf(norm - referenceMagNorm);

		// Keep the sensor with the cleanest field
		if(error < bestError) {
			bestError = error;
			bestIndex = i;
		}
	}

	if(bestIndex >= 0) {
		return rawMsgs[bestIndex];
	}
	
	// Failsafe: All Mags are dead.
	MagMsg deadMsg{}; 
	deadMsg.timestamp = 0;
	return deadMsg;
}

bool SensorHubThread::CheckHealth(uint64_t lastTime, uint64_t refTime, uint32_t timeout) {
	if(lastTime == 0) {
		return false; // Never initialized
	}

	// Check if the sensor data is stale compared to our reference time
	if(refTime > lastTime) {
		if((refTime - lastTime) > timeout) {
			return false;
		}
	}
	return true;
}

void SensorHubThread::CrossProduct(const float a[3], const float b[3], float out[3]) {
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}