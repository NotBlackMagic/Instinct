/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Messaging/instinctLinkConfig.cpp
 */

#include "instinctLinkConfig.hpp"

void InstinctLinkConfig::ApplyTuningProfile() {
	// Disable any existing streams to ensure a clean slot table
	DisableAll();

	// High-rate state and rate-loop tracking for step response plotting (200-250 Hz)
	InstinctLink::ConfigureStream(static_cast<uint8_t>(TopicID::State), 0, 200.0f);
	InstinctLink::ConfigureStream(static_cast<uint8_t>(TopicID::ControlSetpoint), 0, 200.0f);
	InstinctLink::ConfigureStream(static_cast<uint8_t>(TopicID::Actuator), 0, 200.0f);

	// Filter verification (IMU / Gyro)
	InstinctLink::ConfigureStream(static_cast<uint8_t>(TopicID::Imu), 255, 100.0f);

	// Low-rate vehicle status context (Arming, flight modes, failsafes)
	InstinctLink::ConfigureStream(static_cast<uint8_t>(TopicID::VehicleStatus), 0, 10.0f);

	LOG_INFO("InstinctLink: Tuning Profile Applied (High-Rate Control Telemetry)");
}

void InstinctLinkConfig::ApplyHITLProfile() {
	DisableAll();

	// For HITL, stream actuator outputs as fast as possible to feed the simulator physics engine
	InstinctLink::ConfigureStream(static_cast<uint8_t>(TopicID::Actuator), 0, 500.0f);

	// Heartbeat status for link health and watchdog tracking
	InstinctLink::ConfigureStream(static_cast<uint8_t>(TopicID::VehicleStatus), 0, 20.0f);

	LOG_INFO("InstinctLink: HITL Profile Applied (500Hz Actuator Feedback)");
}

void InstinctLinkConfig::ApplyMonitorProfile() {
	DisableAll();

	// Safe rates for bench diagnostics or wireless bridge telemetry
	InstinctLink::ConfigureStream(static_cast<uint8_t>(TopicID::State), 0, 50.0f);
	InstinctLink::ConfigureStream(static_cast<uint8_t>(TopicID::ControlSetpoint), 0, 20.0f);
	InstinctLink::ConfigureStream(static_cast<uint8_t>(TopicID::Actuator), 0, 20.0f);
	InstinctLink::ConfigureStream(static_cast<uint8_t>(TopicID::VehicleStatus), 0, 5.0f);

	LOG_INFO("InstinctLink: Monitor Profile Applied");
}

void InstinctLinkConfig::DisableAll() {
	// Clear all configured streams by querying the Broker's registered topics
	TopicBase* curr = Broker::GetTopicList();
	while(curr != nullptr) {
		InstinctLink::ConfigureStream(curr->GetTopicID(), curr->GetInstance(), 0.0f);
		curr = curr->nextTopic;
	}

	LOG_INFO("InstinctLink: All Streams Disabled");
}