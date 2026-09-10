/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Estimation/estimatorThread.cpp
 */

#include "estimatorThread.hpp"

TX_THREAD EstimatorThread::threadPtr;
uint8_t EstimatorThread::threadStack[16384];

// Instantiate the Math Engine
InEKF EstimatorThread::filter;

// Output Publisher
Topic<StateMsg> EstimatorThread::topicState("state", static_cast<uint8_t>(TopicID::State));

// Subscribers
Subscriber<IMUMsg> EstimatorThread::subVirtualIMU;
Subscriber<MagMsg> EstimatorThread::subVirtualMag;
Subscriber<BaroMsg> EstimatorThread::subVirtualBaro;

Topic<IMUMsg>* EstimatorThread::topicVirtualIMU = nullptr;
Topic<MagMsg>* EstimatorThread::topicVirtualMag = nullptr;
Topic<BaroMsg>* EstimatorThread::topicVirtualBaro = nullptr;

void EstimatorThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr,	const_cast<char*>("EST_InEKF"),
													EstimatorThread::Run, 0,
													threadStack, sizeof(threadStack),
													1, 0,
													TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX EST InEKF Thread Create Failed.");
	}

	// Register the output topic to the Broker
	Broker::RegisterTopic(&topicState);
}

void EstimatorThread::Run(ULONG input) {
	LOG_INFO("Estimator InEKF Thread Initialized.");

	// Enforce Hardware Flush-to-Zero (FZ) for this specific thread's FPU context
	// Not having this code resulted in >60% CPU utilization by the EKF at 100Hz, with it down to 6%
	uint32_t fpscr = __get_FPSCR();
	fpscr |= (1 << 24);			// Set bit 24
	__set_FPSCR(fpscr);

	// Boot Synchronization: Wait for SensorHub to be online
	while (topicVirtualIMU == nullptr) {
		topicVirtualIMU = Broker::GetTopic<IMUMsg>(static_cast<uint8_t>(TopicID::Imu), virtualInstanceID);
		if(topicVirtualIMU == nullptr) {
			tx_thread_sleep(10); // Sleep for 10 ticks and check again
		}
	}
	topicVirtualIMU->Subscribe(&subVirtualIMU);

	topicVirtualMag = Broker::GetTopic<MagMsg>(static_cast<uint8_t>(TopicID::Mag), virtualInstanceID);
	if(topicVirtualMag != nullptr) {
		topicVirtualMag->Subscribe(&subVirtualMag);
	}
	else {
		LOG_WARN("Estimator: No Virtual Mag Topic found!");
	}

	topicVirtualBaro = Broker::GetTopic<BaroMsg>(static_cast<uint8_t>(TopicID::Baro), virtualInstanceID);
	if(topicVirtualBaro != nullptr) {
		topicVirtualBaro->Subscribe(&subVirtualBaro);
	}
	else {
		LOG_WARN("Estimator: No Virtual Baro Topic found!");
	}

	IMUMsg imuMsg;
	MagMsg magMsg;
	BaroMsg baroMsg;
	bool magInitialized = false;

	StateMsg currentState{};
	currentState.status = EstimatorState::Uninitialized;

	uint64_t cycleTimestamp = Time::GetUs();
	uint64_t timestamp = Time::GetUs();
	uint64_t deltaTime = 0;
	uint32_t timePred, timePeriod, timeUpd, timeCycle;

	while(1) {
		// EKF Heartbeat: Virtual IMU stream drives the prediction step
		if(topicVirtualIMU->Take(&subVirtualIMU, imuMsg, TX_WAIT_FOREVER) == true) {
			timePeriod = Time::GetUs() - cycleTimestamp;
			timePeriod = timePeriod + timeCycle;
			cycleTimestamp = Time::GetUs();

			// Hardware failure check: SensorHub passes timestamp = 0 if all IMUs are dead.
			if(imuMsg.timestamp == 0) {
				LOG_FATAL("Estimator: SensorHub declared total IMU loss! Halting.");
				
				currentState.status = EstimatorState::Diverged;
				currentState.timestamp = 0;		// Downstream modules (PID) will see this and trigger failsafes
				topicState.Publish(currentState);
				
				while(1) {
					tx_thread_sleep(100);
				} 
			}

			// PROPAGATION (PREDICT STEP)
			// Integrating the IMU across the SE_2(3) Manifold
			timestamp = Time::GetUs();
			filter.Predict(imuMsg.accel, imuMsg.gyro, imuMsg.timestamp);
			timePred = Time::GetUs() - timestamp;
			timestamp = Time::GetUs();

			// CORRECTION (UPDATE STEPS)

			// Horizon observation to bound Roll/Pitch drift
			filter.UpdateGravity(imuMsg.accel);
			
			// Magnetometer Right-Invariant Update
			if(topicVirtualMag != nullptr && topicVirtualMag->Take(&subVirtualMag, magMsg, TX_NO_WAIT) == true) {
				if(magMsg.timestamp != 0) {
					if(filter.IsConverged() == true && magInitialized == false) {
						// Drone is level and stable. Lock/save local magnetic field
						filter.InitializeMagReference(magMsg.values);
						magInitialized = true;
						LOG_INFO("Estimator: Magnetometer Reference Initialized (%.2f, %.2f, %.2f).", magMsg.values[0], magMsg.values[1], magMsg.values[2]);
					}
					// Normal operation
					// filter.UpdateMag(magMsg.values);
				}
			}

			// Barometer Right-Invariant Update
			if(topicVirtualBaro != nullptr && topicVirtualBaro->Take(&subVirtualBaro, baroMsg, TX_NO_WAIT) == true) {
				if(baroMsg.timestamp != 0) {
					filter.UpdateBaro(baroMsg.pressure);
				}
			}

			timeUpd = Time::GetUs() - timestamp;
			timestamp = Time::GetUs();
			if(deltaTime < Time::GetMs()) {
				// Current state is: "EST Thread: TOTAL 97/10091 us, PRED 57 us, UPT G 35 us" (A HUGE upgrade from starting point of "PRED 280 us, UPT G 341 us"!)
				// LOG_INFO("EST Thread: TOTAL %d/%d us, PRED %d us, UPT G %d us", timeCycle, timePeriod, timePred, timeUpd);
				deltaTime = Time::GetMs() + 5000;
			}

			// EXTRACTION & PUBLISHING
			
			// Pull the latest decoupled kinematics from the 5x5 Manifold
			currentState.attitude = filter.GetQuaternion();
			currentState.velocity = filter.GetVelocity();
			currentState.position = filter.GetPosition();
			currentState.gyroBias = filter.GetGyroBias();
			currentState.accelBias = filter.GetAccelBias();
			
			// Sync the state timestamp perfectly with the IMU sample that generated it
			currentState.timestamp = imuMsg.timestamp;
			
			// Once the EKF has processed enough samples to converge, update status
			if(filter.IsConverged() == true) {
				currentState.status = EstimatorState::FullyConverged;
			}
			else {
				currentState.status = EstimatorState::Uninitialized;
			}

			topicState.Publish(currentState);

			timeCycle = Time::GetUs() - cycleTimestamp;
			cycleTimestamp = Time::GetUs();
		}
		else {
			// Safeguard: If the middleware queue fails, force a yield.
			tx_thread_sleep(1);
        }
	}
}