/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/blackboxConfig.cpp
 */

#include "blackboxConfig.hpp"

void BlackboxConfig::ApplyStandardProfile() {
	// Standard flight use low/basic rate for IMU data
	Broker::EnableLogging(static_cast<uint8_t>(TopicID::Imu), Broker::allInstancesID, 50); 

	// Low rate environment and navigation sensors
	Broker::EnableLogging(static_cast<uint8_t>(TopicID::Baro), Broker::allInstancesID, 20); 
	Broker::EnableLogging(static_cast<uint8_t>(TopicID::NavSat), Broker::allInstancesID, 5); 

	// Core state and inputs
	Broker::EnableLogging(static_cast<uint8_t>(TopicID::State), 0, 50);
	Broker::EnableLogging(static_cast<uint8_t>(TopicID::RC), Broker::allInstancesID, 20);

	LOG_INFO("Blackbox: Standard Profile Applied");
}

void BlackboxConfig::ApplyTuningProfile() {
	// Tuning flight: Log IMUs at their native rate (0 = No limit)
	Broker::EnableLogging(static_cast<uint8_t>(TopicID::Imu), Broker::allInstancesID, 0); 
	Broker::EnableLogging(static_cast<uint8_t>(TopicID::Gyro), Broker::allInstancesID, 0); 

	// High-rate control loop diagnostics
	Broker::EnableLogging(static_cast<uint8_t>(TopicID::Actuator), 0, 0);
	Broker::EnableLogging(static_cast<uint8_t>(TopicID::ControlSetpoint), 0, 0);
	Broker::EnableLogging(static_cast<uint8_t>(TopicID::State), 0, 250);

	// Also log standard slow sensors to give full context to the tuning flight
	Broker::EnableLogging(static_cast<uint8_t>(TopicID::Baro), Broker::allInstancesID, 20); 
	Broker::EnableLogging(static_cast<uint8_t>(TopicID::NavSat), Broker::allInstancesID, 5);

	LOG_INFO("Blackbox: Tuning Profile Applied (High Bandwidth)");
}

void BlackboxConfig::DisableAll() {
	TopicBase* curr = Broker::GetTopicList();
	while(curr != nullptr) {
		curr->DisableLogging();
		curr = curr->nextTopic;
	}
	LOG_INFO("Blackbox: All Data Logging Disabled");
}