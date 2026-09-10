/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Commander/commanderThread.hpp
 * Author:	NotBlackMagic
 * Brief:	Evaluates pilot/system intent and translates inputs into kinematic commands.
 */

#pragma once

#include "hardware.hpp"

#include "logger.hpp"
#include "pubSub.hpp"
#include "rotations.hpp"

#include "tx_api.h"

class CommanderThread {
	public:
		static void Init();

	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[8192];

		struct VehicleConfig {
			uint8_t thrustAxis;	// 0 = X-Axis (Rover/Plane), 2 = Z-Axis (Drone)
			float rollScale;	// 1.0f for Drone/Plane, 0.0f for Rover
			float pitchScale;	// 1.0f for Drone/Plane, 0.0f for Rover
			float yawScale;		// 1.0f for all (Usually 1.0 for Drone Yaw or Rover Steering)
		};
		static VehicleConfig config;

		// Publishers
		static Topic<VehicleStatusMsg> topicStatus;
		static Topic<ControlSetpointMsg> topicSetpoint;

		// Subscribers
		static Subscriber<ManualControlMsg> subManualControl;
		static Subscriber<StateMsg> subState;

		// Topic Pointers
		static Topic<ManualControlMsg>* topicManualControl;
		static Topic<StateMsg>* topicState;

		// Internal State
		static VehicleStatusMsg currentStatus;
		static StateMsg currentState;
		static float targetYaw;

		static uint64_t lastInputTimestamp;
		static uint64_t lastRunTimestamp;

		// Vehicle Configuration / Scaling
		static constexpr float maxRollPitchRate = 8.72f;	// ~500 deg/s in rad/s
		static constexpr float maxYawRate = 5.23f;			// ~300 deg/s in rad/s
		static constexpr float maxBankAngle = 0.785f;		// ~45 degrees in rad

		static void Run(ULONG input);

		// Core Logic Steps
		static void UpdateStateMachine(const ManualControlMsg& input, const StateMsg& state, bool hasNewInput);
		static void GenerateCommands(const ManualControlMsg& input, const StateMsg& state, float dt);
};