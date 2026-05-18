/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/PubSub/Messages/state.hpp
 * Author:  NotBlackMagic
 * Brief:   Topic message definition for the vehicle's estimated kinematic state.
 */

#pragma once

#include "common.hpp"

// Defines the current health and convergence level of the InEKF
enum class EstimatorState : uint8_t {
	Uninitialized = 0,	// Waiting for sensors / IMU alignment
	AttitudeOnly = 1,	// Roll/Pitch/Yaw are valid, but no velocity/position (e.g., no GPS/Flow yet)
	FullyConverged = 2,	// 3D Position and Velocity are valid
	DeadReckoning = 3,	// Lost external references (GPS/Vision), drifting on IMU
	Diverged = 4		// Math fault, do not trust state
};

struct StateMsg {
	Timestamp timestamp;	// Microseconds (Syncs with the Virtual IMU that triggered this state)

	// Core Kinematic State: NED (North-East-Down) coordinate frame here.
	Vector3f angularVelocity;	// Bias-corrected angular rates in Body frame (rad/s)
	Quaternion attitude;		// Rotation from Body frame to Local Earth frame
	Vector3f velocity;			// Linear velocity in Local Earth frame (m/s)
	Vector3f position;			// Local position relative to home/boot location (meters)

	// Estimated Sensor Biases
	Vector3f gyroBias;		// rad/s [Roll, Pitch, Yaw axes]
	Vector3f accelBias;		// m/s^2 [X, Y, Z axes]

	// Estimator Filter Health
	EstimatorState status;	// Enum indicating which fields downstream nodes can trust
};