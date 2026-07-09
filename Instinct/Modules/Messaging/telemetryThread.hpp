/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/telemetryThread.hpp
 * Author:  NotBlackMagic
 * Brief:   
 */

#pragma once

#include "math.h"

#include "hardware.hpp"

#include "hardware.hpp"
#include "logger.hpp"
#include "pubSub.hpp"
#include "state.hpp"
#include "system.hpp"

#include "tx_api.h"

// MAVLink includes
#include "mavlink_types.h"
#define MAVLINK_USE_MESSAGE_INFO
#include "common/mavlink.h"

class TelemetryThread {
	public:
		static void Init();

	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[4096];

		// Subscribers
		static Subscriber<StateMsg> subState;
		static Subscriber<IMUMsg> subVirtualIMU;
		static Subscriber<MagMsg> subVirtualMag;
		static Subscriber<BaroMsg> subVirtualBaro;

		// Topic Pointers
		static Topic<StateMsg>* topicState;
		static Topic<IMUMsg>* topicVirtualIMU;
		static Topic<MagMsg>* topicVirtualMag;
		static Topic<BaroMsg>* topicVirtualBaro;

		// MAVLink System Configuration
		static constexpr uint8_t systemId = 1;		// ID of this vehicle
		static constexpr uint8_t componentId = 1;	// ID of the flight controller (Autopilot)

		static void Run(ULONG input);
		static void SendHeartbeat(UART& uart);
		static void SendAttitude(UART& uart, const StateMsg& state);
		static void SendHighresIMU(UART& uart, const IMUMsg& imu, const MagMsg& mag, const BaroMsg& baro);

		// Helper structure for Euler angles
		struct EulerAngles {
			float roll;
			float pitch;
			float yaw;
		};

		// Mathematical helper to translate internal Quaternions to MAVLink Euler angles
		static EulerAngles QuaternionToEuler(const Quaternion& q);
};