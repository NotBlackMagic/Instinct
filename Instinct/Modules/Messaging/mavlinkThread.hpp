/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Messaging/mavlinkThread.hpp
 * Author:	NotBlackMagic
 * Brief:	Aaa
 */

#pragma once

#include "math.h"

#include "hardware.hpp"

#include "hardware.hpp"
#include "logger.hpp"
#include "rotations.hpp"
#include "pubSub.hpp"
#include "state.hpp"
#include "system.hpp"

#include "tx_api.h"

// MAVLink includes
#include "mavlink_types.h"
#define MAVLINK_USE_MESSAGE_INFO
#include "common/mavlink.h"

class MavlinkThread {
	public:
		enum class Stream : uint8_t {
			Heartbeat = 0,
			Attitude,
			HighresIMU,
			DistanceSensor,
			OpticalFlow,
			Count
		};

		static void Init();

		static void SetRate(Stream stream, float rateHz);

	private:
		static TX_THREAD threadPtr;
		static const uint16_t threadStackSize = 4096;
		__attribute__((aligned(32))) static uint8_t threadStack[threadStackSize];

		// Subscribers
		static Subscriber<StateMsg> subState;
		static Subscriber<IMUMsg> subVirtualIMU;
		static Subscriber<MagMsg> subVirtualMag;
		static Subscriber<BaroMsg> subVirtualBaro;
		static Subscriber<RangeMsg> subRange;
		static Subscriber<FlowMsg> subFlow;

		// Topic Pointers
		static Topic<StateMsg>* topicState;
		static Topic<IMUMsg>* topicVirtualIMU;
		static Topic<MagMsg>* topicVirtualMag;
		static Topic<BaroMsg>* topicVirtualBaro;
		static Topic<RangeMsg>* topicRange;
		static Topic<FlowMsg>* topicFlow;

		// MAVLink System Configuration
		static constexpr uint8_t systemId = 1;		// ID of this vehicle
		static constexpr uint8_t componentId = 1;	// ID of the flight controller (Autopilot)

		// Stream/message scheduler

		struct StreamConfig {
			uint32_t intervalUs;
			uint64_t lastSentUs;
		};

		static StreamConfig streams[static_cast<uint8_t>(Stream::Count)];

		static void Run(ULONG input);
		static void SendHeartbeat(UART& uart);
		static void SendAttitude(UART& uart, const StateMsg& state);
		static void SendHighresIMU(UART& uart, const IMUMsg& imu, const MagMsg& mag, const BaroMsg& baro);
		static void SendDistanceSensor(UART& uart, const RangeMsg& range);
		static void SendOpticalFlow(UART& uart, const FlowMsg& flow, float groundDistance = -1.0f);
};