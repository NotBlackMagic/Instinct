/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Messaging/PubSub/Messages/topicIDs.hpp
 * Author:	NotBlackMagic
 * Brief:	List of all topic IDs available.
 */

#pragma once

#include <stdint.h>

struct TopicInstance {
	// 0x00 .. 0xEF : Physical/indexed instances
	static constexpr uint8_t Default = 0x00;
	static constexpr uint8_t MaxPhysical = 0xEF;

	// 0xF0 .. 0xFD : Virtual/synthetic topic instances
	static constexpr uint8_t Virtual = 0xF0;	// Voted/calibrated/fused sensor output
	static constexpr uint8_t Simulated = 0xF1;	// HITL/SITL injected data
	static constexpr uint8_t MaxValid = 0xFD;

	// 0xFE .. 0xFF : Non-topic reserved planes
	static constexpr uint8_t All = 0xFE;		// Wildcard for Broker/Logger/Link operations
	static constexpr uint8_t Protocol = 0xFF;	// InstinctLink RPCs & SD Logger meta packets
};

enum class TopicID : uint8_t {
	// Inertial Sensors
	Accel = 1,
	Gyro = 2,
	Mag = 3,
	Baro = 4,
	Imu = 5,

	// GNSS Sensors
	NavSat = 10,
	NavRTCM = 11,

	// Environment/Ranging Sensors
	Range = 20,
	Tacho = 21,
	Flow = 22,

	// Power Sensors
	Power = 30,
	Battery = 31,

	// Inputs
	RC = 40,
	ManualControl = 41,

	// Estimation / State
	State = 50,
	VehicleStatus = 51,

	// Control Commands
	ControlSetpoint = 60,

	// Actuators / Hardware Outputs
	Actuator = 70,

	// Payloads
	Gimbal = 80,

	// System / Configuration
	ParamUpdate = 200
};

class TopicAddress {
	public:
		static constexpr uint16_t Pack(uint8_t topicID, uint8_t instance = TopicInstance::Default) {
			return (static_cast<uint16_t>(instance) << 8) | topicID;
		}

		static constexpr uint16_t Pack(TopicID topicID, uint8_t instance = TopicInstance::Default) {
			return Pack(static_cast<uint8_t>(topicID), instance);
		}

		static constexpr uint16_t MakeProtocolID(uint8_t commandID) {
			return Pack(commandID, TopicInstance::Protocol);
		}

		static constexpr uint8_t GetTopicID(uint16_t fullID) {
			return static_cast<uint8_t>(fullID & 0xFF);
		}

		static constexpr uint8_t GetInstance(uint16_t fullID) {
			return static_cast<uint8_t>((fullID >> 8) & 0xFF);
		}

		static constexpr bool IsProtocol(uint16_t fullID) {
			return GetInstance(fullID) == TopicInstance::Protocol;
		}

		static constexpr bool IsValidTopicInstance(uint8_t instance) {
			return instance <= TopicInstance::MaxValid;
		}
};