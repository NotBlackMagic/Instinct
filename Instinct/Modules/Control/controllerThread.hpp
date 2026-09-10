/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Control/controllerThread.hpp
 * Author:	NotBlackMagic
 * Brief:	Generalized kinematic controller.
 */

#pragma once

#include "hardware.hpp"

#include "logger.hpp"
#include "matrix.hpp"
#include "pid.hpp"
#include "pubSub.hpp"
#include "rotations.hpp"

#include "tx_api.h"

class ControllerThread {
	public:
		/// @brief Initializes the Controller ThreadX loop and PIDs, and registers topics.
		static void Init();

	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[8192];

		// Publishers
		static Topic<ActuatorMsg> topicActuators;

		// Subscribers
		static Subscriber<StateMsg> subState;
		static Subscriber<VehicleStatusMsg> subStatus;
		static Subscriber<ControlSetpointMsg> subSetpoint;

		// Topic Pointers
		static Topic<StateMsg>* topicState;
		static Topic<VehicleStatusMsg>* topicStatus;
		static Topic<ControlSetpointMsg>* topicSetpoint;

		// Internal State
		static VehicleStatusMsg currentStatus;
		static ControlSetpointMsg currentSetpoint;
		static ControlWrench currentWrench;
		static uint64_t lastStateTimestamp;

		// PID Controllers[0=X, 1=Y, 2=Z]
		static PID pidPosition[3];
		static PID pidVelocity[3];
		static PID pidRate[3];

		// Physical Constants
		static constexpr float gravity = 9.81f; 
		static constexpr float hoverThrust = 0.35f; // Throttle to hover [0.0 to 1.0]

		// Defines how the 6-DOF Wrench maps to physical actuators
		struct MixerGeometry {
			// 16 actuators x 6 DOFs (Fx, Fy, Fz, Tx, Ty, Tz)
			Matrix<16, 6> geometry;
			float outMin[16]; // Minimum hardware limit (e.g., 0.0f for motors, -1.0f for servos)
			float outMax[16]; // Maximum hardware limit (e.g., 1.0f)
		};

		static MixerGeometry currentMixer;

		/// @brief Main 1000Hz RTOS loop, synchronized to EKF StateMsg.
		static void Run(ULONG input);
		
		// Cascaded Math Blocks
		/// @brief Converts Position errors (m) into Velocity setpoints (m/s).
		static void RunPositionController(const StateMsg& state, ControlSetpointMsg& setpoint, float dt);

		/// @brief Converts Velocity errors (m/s) into Attitude/Thrust setpoints.
		static void RunVelocityController(const StateMsg& state, ControlSetpointMsg& setpoint, float dt);

		/// @brief Converts Attitude quaternion errors into Angular Rate setpoints (rad/s).
		static void RunAttitudeController(const StateMsg& state, ControlSetpointMsg& setpoint, float dt);

		/// @brief Converts Angular Rate errors (rad/s) into physical Torques.
		static void RunRateController(const StateMsg& state, const ControlSetpointMsg& setpoint, ControlWrench& outputWrench, float dt);
		
		/// @brief Matrix Mixer mapping a 6-DOF Wrench to hardware actuator outputs.
		static void AllocateActuators(const ControlWrench& wrench, ActuatorMsg& actuators);
};