/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/telemetryThread.cpp
 */

#include "telemetryThread.hpp"

TX_THREAD TelemetryThread::threadPtr;
uint8_t TelemetryThread::threadStack[4096];

// Subscribers
Subscriber<StateMsg> TelemetryThread::subState;
Subscriber<IMUMsg> TelemetryThread::subVirtualIMU;
Subscriber<MagMsg> TelemetryThread::subVirtualMag;
Subscriber<BaroMsg> TelemetryThread::subVirtualBaro;

// Topic Pointers
Topic<StateMsg>* TelemetryThread::topicState = nullptr;
Topic<IMUMsg>* TelemetryThread::topicVirtualIMU = nullptr;
Topic<MagMsg>* TelemetryThread::topicVirtualMag = nullptr;
Topic<BaroMsg>* TelemetryThread::topicVirtualBaro = nullptr;

void TelemetryThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("Telem"),
										TelemetryThread::Run, 0,
										threadStack, sizeof(threadStack),
										5, 0,
										TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Telemetry Thread Create Failed.");
	}
}

void TelemetryThread::Run(ULONG input) {
	LOG_INFO("Telemetry Thread Initialized.");

	// Boot Synchronization: Wait for Estimator and SensorHub to be online
	while(topicState == nullptr || topicVirtualIMU == nullptr) {
		if(topicState == nullptr) {
			topicState = Broker::GetTopic<StateMsg>(static_cast<uint8_t>(TopicID::State), 0);
		}
		if(topicVirtualIMU == nullptr) {
			topicVirtualIMU = Broker::GetTopic<IMUMsg>(static_cast<uint8_t>(TopicID::Imu), 255);
		}
		tx_thread_sleep(10); // Sleep for 10 ticks and check again
	}
	topicState->Subscribe(&subState);
	topicVirtualIMU->Subscribe(&subVirtualIMU);

	topicVirtualMag = Broker::GetTopic<MagMsg>(static_cast<uint8_t>(TopicID::Mag), 255);
	if(topicVirtualMag != nullptr) {
		topicVirtualMag->Subscribe(&subVirtualMag);
	}

	topicVirtualBaro = Broker::GetTopic<BaroMsg>(static_cast<uint8_t>(TopicID::Baro), 255);
	if(topicVirtualBaro != nullptr) {
		topicVirtualBaro->Subscribe(&subVirtualBaro);
	}

	StateMsg stateMsg;
	stateMsg.attitude = {1.0f, 0.0f, 0.0f, 0.0f};
	stateMsg.angularVelocity = {0.0f, 0.0f, 0.0f};

	IMUMsg imuMsg{};
	MagMsg magMsg{};
	BaroMsg baroMsg{};

	uint64_t lastHeartbeatUs = 0;
	uint64_t lastAttitudeUs = 0;
	uint64_t lastSensorUs = 0;
	while(1) {
		uint64_t now = Time::GetUs();

		// Send Heartbeat at 1 Hz
		if((now - lastHeartbeatUs) >= 1000000) {
			lastHeartbeatUs = now;
			SendHeartbeat(hdrUART);
		}

		// Send Attitude at ~20Hz
		if((now - lastAttitudeUs) >= 50000) {
			lastAttitudeUs = now;
			topicState->Peek(stateMsg);
			SendAttitude(hdrUART, stateMsg);
		}

		// Send HighRes Sensor Data at 50Hz (20000us) for QGC charting
		if((now - lastSensorUs) >= 20000) {
			lastSensorUs = now;
			if(topicVirtualIMU->Peek(imuMsg) == true) {
				if(topicVirtualMag != nullptr) {
					topicVirtualMag->Peek(magMsg);
				}
				if(topicVirtualBaro != nullptr) {
					topicVirtualBaro->Peek(baroMsg);
				}
				SendHighresIMU(hdrUART, imuMsg, magMsg, baroMsg);
			}
		}

		// TODO Handle inbound MAVLink parameter requests from QGC here.

		tx_thread_sleep(10);
	}
}

void TelemetryThread::SendHeartbeat(UART& uart) {
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	// Pack the heartbeat message
	mavlink_msg_heartbeat_pack(systemId, componentId, &msg, MAV_TYPE_QUADROTOR, 
															MAV_AUTOPILOT_GENERIC, 
															MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 
															0,
															MAV_STATE_STANDBY);

	uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
	uart.Transmit(buffer, len);
}

void TelemetryThread::SendAttitude(UART& uart, const StateMsg& state) {
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

	// Convert core Quaternion to MAVLink Euler
	EulerAngles euler = QuaternionToEuler(state.attitude);

	// QGroundControl expects the uptime in milliseconds
	uint32_t bootTimeMs = static_cast<uint32_t>(Time::GetUs() / 1000);

	// Pack the attitude message
	mavlink_msg_attitude_pack(systemId, componentId, &msg, 
								bootTimeMs, 
								euler.roll, euler.pitch, euler.yaw, 
								state.angularVelocity.x, state.angularVelocity.y, state.angularVelocity.z);

	uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
	uart.Transmit(buffer, len);
}

void TelemetryThread::SendHighresIMU(UART& uart, const IMUMsg& imu, const MagMsg& mag, const BaroMsg& baro) {
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

	// MAVLink HIGHRES_IMU expects pressure in millibars (hPa), but internal is Pascals: 1 hPa = 100 Pascals
	float pressureMbar = baro.pressure / 100.0f;
	float diffPressure = 0.0f;
	float alt = 0.0f;

	// Bitmask (HIGHRES_IMU_UPDATED_FLAGS) indicating which fields are valid (0x7F = Accel, Gyro, Mag, Baro, Temp are all valid).
	uint16_t fieldsUpdated = 0x7F; 

	mavlink_msg_highres_imu_pack(systemId, componentId, &msg,
										imu.timestamp, 
										imu.accel[0], imu.accel[1], imu.accel[2],
										imu.gyro[0], imu.gyro[1], imu.gyro[2],
										mag.values[0], mag.values[1], mag.values[2],
										pressureMbar, diffPressure, alt,
										imu.temperature, 
										fieldsUpdated, 0);

	uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
	uart.Transmit(buffer, len);
}

// Standard Aerospace ZYX Euler extraction from a Quaternion
TelemetryThread::EulerAngles TelemetryThread::QuaternionToEuler(const Quaternion& q) {
	EulerAngles angles;

	// Roll (x-axis rotation)
	float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
	float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
	angles.roll = atan2f(sinr_cosp, cosr_cosp);

	// Pitch (y-axis rotation)
	float sinp = 2.0f * (q.w * q.y - q.z * q.x);
	if(fabsf(sinp) >= 1.0f) {
		// Use 90 degrees if out of range to prevent NaN
		angles.pitch = copysignf(3.14159265f / 2.0f, sinp); 
	}
	else {
		angles.pitch = asinf(sinp);
	}

	// Yaw (z-axis rotation)
	float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
	float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
	angles.yaw = atan2f(siny_cosp, cosy_cosp);

	return angles;
}