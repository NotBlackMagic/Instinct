/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Messaging/mavlinkThread.cpp
 */

#include "mavlinkThread.hpp"

TX_THREAD MavlinkThread::threadPtr;
__attribute__((aligned(32))) uint8_t MavlinkThread::threadStack[MavlinkThread::threadStackSize];

// Subscribers
Subscriber<StateMsg> MavlinkThread::subState;
Subscriber<IMUMsg> MavlinkThread::subVirtualIMU;
Subscriber<MagMsg> MavlinkThread::subVirtualMag;
Subscriber<BaroMsg> MavlinkThread::subVirtualBaro;
Subscriber<RangeMsg> MavlinkThread::subRange;
Subscriber<FlowMsg> MavlinkThread::subFlow;

// Topic Pointers
Topic<StateMsg>* MavlinkThread::topicState = nullptr;
Topic<IMUMsg>* MavlinkThread::topicVirtualIMU = nullptr;
Topic<MagMsg>* MavlinkThread::topicVirtualMag = nullptr;
Topic<BaroMsg>* MavlinkThread::topicVirtualBaro = nullptr;
Topic<RangeMsg>* MavlinkThread::topicRange = nullptr;
Topic<FlowMsg>* MavlinkThread::topicFlow = nullptr;

MavlinkThread::StreamConfig MavlinkThread::streams[static_cast<uint8_t>(MavlinkThread::Stream::Count)];

void MavlinkThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("Mavlink"),
										MavlinkThread::Run, 0,
										threadStack, sizeof(threadStack),
										5, 0,
										TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Mavlink Thread Create Failed.");
	}
}

void MavlinkThread::Run(ULONG input) {
	(void)input;
	LOG_INFO("Mavlink Thread Initialized.");

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

	topicRange = Broker::GetTopic<RangeMsg>(static_cast<uint8_t>(TopicID::Range), 0);
	if(topicRange != nullptr) {
		topicRange->Subscribe(&subRange);
	}

	topicFlow = Broker::GetTopic<FlowMsg>(static_cast<uint8_t>(TopicID::Flow), 0);
	if(topicFlow != nullptr) {
		topicFlow->Subscribe(&subFlow);
	}

	StateMsg stateMsg;
	stateMsg.attitude = {1.0f, 0.0f, 0.0f, 0.0f};
	stateMsg.angularVelocity = {0.0f, 0.0f, 0.0f};

	IMUMsg imuMsg;
	MagMsg magMsg;
	BaroMsg baroMsg;
	RangeMsg rangeMsg;
	FlowMsg flowMsg;

	while(1) {
		uint64_t now = Time::GetUs();

		// Message scheduler, loop through all configured streams and see which must be sent now
		for(uint8_t i = 0; i < static_cast<uint8_t>(Stream::Count); i++) {
			if(streams[i].intervalUs > 0 && (now - streams[i].lastSentUs) >= streams[i].intervalUs) {
				streams[i].lastSentUs = now;

				switch(static_cast<Stream>(i)) {
					case Stream::Heartbeat:
						SendHeartbeat(telem2);
						break;
					case Stream::Attitude:
						if(topicState->Peek(stateMsg)) {
							SendAttitude(telem2, stateMsg);
						}
						break;
					case Stream::HighresIMU:
						if(topicVirtualIMU->Peek(imuMsg)) {
							if(topicVirtualMag != nullptr) {
								topicVirtualMag->Peek(magMsg);
							}
							if(topicVirtualBaro != nullptr) {
								topicVirtualBaro->Peek(baroMsg);
							}
							SendHighresIMU(telem2, imuMsg, magMsg, baroMsg);
						}
						break;
					case Stream::DistanceSensor:
						if(topicRange != nullptr && topicRange->Peek(rangeMsg)) {
							SendDistanceSensor(telem2, rangeMsg);
						}
						break;
					case Stream::OpticalFlow:
						if(topicFlow != nullptr && topicFlow->Peek(flowMsg)) {
							float groundDistance = -1.0f;
							if(topicRange != nullptr && topicRange->Peek(rangeMsg)) {
								groundDistance = rangeMsg.distance;
							}
							SendOpticalFlow(telem2, flowMsg, groundDistance);
						}
						break;
					default:
						break;
				}
			}
		}

		// Handle inbound MAVLink parameter requests from QGC (Mavlink RX parsing)
		uint32_t bytesAvailable = telem2.Available();
		if(bytesAvailable > 0) {
			uint8_t rxBuffer[128];
			uint32_t bytesToRead = (bytesAvailable > sizeof(rxBuffer)) ? sizeof(rxBuffer) : bytesAvailable;
			
			// Get/read bytes from the UART buffer
			uint32_t bytesRead = telem2.Receive(rxBuffer, bytesToRead);

			// Feed received buffer byte-by-byte into the official MAVLink parser
			mavlink_message_t rxMsg;
			mavlink_status_t rxStatus;
			for(uint32_t j = 0; j < bytesRead; j++) {
				if(mavlink_parse_char(MAVLINK_COMM_0, rxBuffer[j], &rxMsg, &rxStatus) == 1) {
					// Successfully decoded new message
					switch(rxMsg.msgid) {
						case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
							// QGC wants to know parameters
							// HandleParamRequestList(rxMsg);
							break;
						case MAVLINK_MSG_ID_COMMAND_LONG:
							// QGC sent a command (e.g. ARM, DISARM, CALIBRATE)
							// HandleCommandLong(rxMsg);
							break;
						case MAVLINK_MSG_ID_MANUAL_CONTROL:
							// QGC is sending virtual joystick inputs
							// HandleManualControl(rxMsg);
							break;
						default:
							// Unhandled message
							break;
					}
				}
			}
		}

		tx_thread_sleep(10);
	}
}

void MavlinkThread::SetRate(Stream stream, float rateHz) {
	uint8_t idx = static_cast<uint8_t>(stream);
	if(rateHz > 0.0f) {
		streams[idx].intervalUs = static_cast<uint32_t>(1000000.0f / rateHz);
	}
	else {
		// Disable the stream
		streams[idx].intervalUs = 0;
	}
	streams[idx].lastSentUs = 0;
}

void MavlinkThread::SendHeartbeat(UART& uart) {
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

void MavlinkThread::SendAttitude(UART& uart, const StateMsg& state) {
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

	// Convert core Quaternion to MAVLink Euler
	Vector3f euler = Rotations::QuaternionToEuler(state.attitude);

	// QGroundControl expects the uptime in milliseconds
	uint32_t bootTimeMs = static_cast<uint32_t>(Time::GetUs() / 1000);

	// Pack the attitude message
	mavlink_msg_attitude_pack(systemId, componentId, &msg, 
								bootTimeMs, 
								euler.x, euler.y, euler.z, 
								state.angularVelocity.x, state.angularVelocity.y, state.angularVelocity.z);

	uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
	uart.Transmit(buffer, len);
}

void MavlinkThread::SendHighresIMU(UART& uart, const IMUMsg& imu, const MagMsg& mag, const BaroMsg& baro) {
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

void MavlinkThread::SendDistanceSensor(UART& uart, const RangeMsg& range) {
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

	uint32_t timeBootMs = static_cast<uint32_t>(range.timestamp / 1000);

	// Sensor operational bounds in centimeters
	constexpr uint16_t minDistanceCm = 4;	// e.g., 0.04m
	constexpr uint16_t maxDistanceCm = 400;	// e.g., 4.00m

	// Convert distance from meters to cm
	uint16_t currentDistanceCm = 0;
	if(range.distance >= 0.0f) {
		float distanceCm = range.distance * 100.0f;
		if(distanceCm < static_cast<float>(minDistanceCm)) {
			currentDistanceCm = minDistanceCm;
		}
		else if(distanceCm > static_cast<float>(maxDistanceCm)) {
			currentDistanceCm = maxDistanceCm;
		}
		else {
			currentDistanceCm = static_cast<uint16_t>(distanceCm);
		}
	}

	// 0 = Unknown covariance; orientation = 25 (MAV_SENSOR_ROTATION_PITCH_270, Z-down facing)
	mavlink_msg_distance_sensor_pack(systemId, componentId, &msg,
		timeBootMs,
		minDistanceCm, maxDistanceCm, currentDistanceCm,
		MAV_DISTANCE_SENSOR_LASER, 0, MAV_SENSOR_ROTATION_PITCH_270,
		0,
		0.0f, 0.0f, 
		nullptr,
		0
	);

	uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
	uart.Transmit(buffer, len);
}

void MavlinkThread::SendOpticalFlow(UART& uart, const FlowMsg& flow, float groundDistance) {
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

	// Scale quality from [0.0f, 1.0f] to [0, 255]
	float clampedQuality = (flow.quality < 0.0f) ? 0.0f : ((flow.quality > 1.0f) ? 1.0f : flow.quality);
	uint8_t qualityByte = static_cast<uint8_t>(clampedQuality * 255.0f);

	mavlink_msg_optical_flow_pack(systemId, componentId, &msg,
		flow.timestamp,
		0,
		0, 0,
		0.0f, 0.0f,
		qualityByte,
		groundDistance,
		flow.flowRate[0], flow.flowRate[1]
	);

	uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
	uart.Transmit(buffer, len);
}