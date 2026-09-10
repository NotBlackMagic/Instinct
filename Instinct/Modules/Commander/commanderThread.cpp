/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Commander/commanderThread.cpp
 */

#include "commanderThread.hpp"
#include <math.h>

TX_THREAD CommanderThread::threadPtr;
uint8_t CommanderThread::threadStack[8192];

// Output Publishers
Topic<VehicleStatusMsg> CommanderThread::topicStatus("vehicle_status", static_cast<uint8_t>(TopicID::VehicleStatus), 0);
Topic<ControlSetpointMsg> CommanderThread::topicSetpoint("control_setpoint", static_cast<uint8_t>(TopicID::ControlSetpoint), 0);

// Subscribers
Subscriber<ManualControlMsg> CommanderThread::subManualControl;
Subscriber<StateMsg> CommanderThread::subState;

// Topic Pointers
Topic<ManualControlMsg>* CommanderThread::topicManualControl = nullptr;
Topic<StateMsg>* CommanderThread::topicState = nullptr;

// Internal State
VehicleStatusMsg CommanderThread::currentStatus;
StateMsg CommanderThread::currentState{};
float CommanderThread::targetYaw = 0.0f;
uint64_t CommanderThread::lastInputTimestamp = 0;
uint64_t CommanderThread::lastRunTimestamp = 0;

// Initialize the vehicle config (Defaulting to Drone)
CommanderThread::VehicleConfig CommanderThread::config = { 2, 1.0f, 1.0f, 1.0f }; // Z-Thrust, Full Roll/Pitch/Yaw

void CommanderThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr,	const_cast<char*>("CMD_Logic"),
													CommanderThread::Run, 0,
													threadStack, sizeof(threadStack),
													3, 0, 
													TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX CMD Logic Thread Create Failed.");
	}

	Broker::RegisterTopic(&topicStatus);
	Broker::RegisterTopic(&topicSetpoint);
}

void CommanderThread::Run(ULONG input) {
	(void)input;
	LOG_INFO("Commander Thread Initialized.");

	// Boot Synchronization: Wait for the Acquisition module to start publishing
	while(topicManualControl == nullptr || topicState == nullptr) {
		topicManualControl = Broker::GetTopic<ManualControlMsg>(static_cast<uint8_t>(TopicID::ManualControl), 0);
		topicState = Broker::GetTopic<StateMsg>(static_cast<uint8_t>(TopicID::State), 0);
		tx_thread_sleep(10);
	}

	topicManualControl->Subscribe(&subManualControl);
	topicState->Subscribe(&subState);

	ManualControlMsg currentInput{};

	// Initialize default safe status
	currentStatus.arming = ArmingStatus::Disarmed;
	currentStatus.mode = ControlMode::ManualRate;
	while(1) {
		// Run at a fixed ~50Hz (Assuming 1 tick = 1ms) to ensures the commander logic keeps running even if the RC link drops
		tx_thread_sleep(20);

		// Non-blocking pub-sub read
		bool hasNewInput = topicManualControl->Take(&subManualControl, currentInput, TX_NO_WAIT);
		topicState->Take(&subState, currentState, TX_NO_WAIT);

		// Calculate dt for integrators
		uint64_t timestamp = Time::GetUs(); 
		float dt = (float)(timestamp - lastRunTimestamp) / 1000000.0f;
		if(dt <= 0.0f || dt > 0.1f) {
			dt = 0.02f; // Fallback to 50Hz nominal
		}
		lastRunTimestamp = timestamp;

		// Process State Machine (Arming/Failsafe/Mode)
		UpdateStateMachine(currentInput, currentState, hasNewInput);
		
		currentStatus.timestamp = timestamp;
		topicStatus.Publish(currentStatus);

		// Generate Kinematic Setpoints if armed
		if(currentStatus.arming == ArmingStatus::Armed) {
			GenerateCommands(currentInput, currentState, dt);
		}
	}
}

void CommanderThread::UpdateStateMachine(const ManualControlMsg& input, const StateMsg& state, bool hasNewInput) {
	// Failsafe Logic: If not received an RC input in 500ms, or link state says lost
	// Check RC Link state (0: Disconnected, 1: Connected, 2: Lost, 3: Failsafe)
	if(hasNewInput == true) {
		lastInputTimestamp = input.timestamp;
	}

	if((Time::GetUs() - lastInputTimestamp) > 500000 || input.linkState >= 2) {
		currentStatus.arming = ArmingStatus::Failsafe;
		return;
	}

	bool armingRequested = false;
	if(input.aux1 > 0.5f) {
		armingRequested = true;
	}

	if(armingRequested == true && currentStatus.arming != ArmingStatus::Armed) {
		// Check throttle (needs be at rest)
		bool throttleIdle = false;
		if(abs(input.throttle) <= 0.05f) {
			throttleIdle = true;
		}

		// Check estimator status
		bool estimatorReady = false;
		if(state.status == EstimatorState::FullyConverged || state.status == EstimatorState::AttitudeOnly) {
			estimatorReady = true;
		}

		if(throttleIdle == true && estimatorReady == true) {
			LOG_INFO("Commander: Vehicle ARMED.");
			currentStatus.arming = ArmingStatus::Armed;
			targetYaw = Rotations::ExtractYawFromQuaternion(state.attitude);
		}
		else {
			// Reject arming request
			if(throttleIdle == false) {
				LOG_WARN("Commander: Arming rejected: Throttle not at zero!");
			}
			if(estimatorReady == false) {
				LOG_WARN("Commander: Arming rejected: Estimator not converged!");
			}
			currentStatus.arming = ArmingStatus::Disarmed;
		}
	}
	else if(armingRequested == false && currentStatus.arming == ArmingStatus::Armed) {
		LOG_INFO("Commander: Vehicle DISARMED");
		currentStatus.arming = ArmingStatus::Disarmed;
	}

	// Flight Mode Logic using a 3-position switch (Aux2)
	if(input.aux2 < -0.5f) {
		currentStatus.mode = ControlMode::ManualRate;	// Acro
	} 
	else if(input.aux2 >= -0.5f && input.aux2 <= 0.5f) {
		currentStatus.mode = ControlMode::ManualAngle;	// Angle/Level
	} 
	else {
		currentStatus.mode = ControlMode::PositionHold;	// GPS/Nav hold
	}
}

void CommanderThread::GenerateCommands(const ManualControlMsg& input, const StateMsg& state, float dt) {
	ControlSetpointMsg setpoint{};
	setpoint.timestamp = Time::GetUs();
	setpoint.controlSource = 0;		// 0 = RC Pilot
	setpoint.activeMask = SetpointMask::None;

	// Apply center deadband to sticks to prevent drifting
	float roll = Rotations::ApplyDeadband(input.roll, 0.05f);
	float pitch = Rotations::ApplyDeadband(input.pitch, 0.05f);
	float yaw = Rotations::ApplyDeadband(input.yaw, 0.05f);

	// Manual modes
	if(currentStatus.mode == ControlMode::ManualRate || currentStatus.mode == ControlMode::ManualAngle) {
		// Map Linear Effort (Throttle) dynamically based on Vehicle Configuration
		if(config.thrustAxis == 2) {
			setpoint.activeMask = setpoint.activeMask | SetpointMask::LinearEffortZ;
			setpoint.linearEffort.z = input.throttle;
		}
		else if(config.thrustAxis == 0) {
			setpoint.activeMask = setpoint.activeMask | SetpointMask::LinearEffortXY;
			setpoint.linearEffort.x = input.throttle;		// Forward/Reverse for Rover/Plane
		}

		// Map Rotational Intent
		if(currentStatus.mode == ControlMode::ManualRate) {
			setpoint.activeMask = setpoint.activeMask | SetpointMask::AngularRate;
			
			// For rover, config.rollScale and pitchScale are 0.0f, killing/removing the output
			setpoint.angularRates.x = roll * maxRollPitchRate * config.rollScale;
			setpoint.angularRates.y = pitch * maxRollPitchRate * config.pitchScale;
			setpoint.angularRates.z = yaw * maxYawRate * config.yawScale;
		}
		else if(currentStatus.mode == ControlMode::ManualAngle) {
			setpoint.activeMask = setpoint.activeMask | SetpointMask::Attitude;

			float targetRoll = roll * maxBankAngle * config.rollScale;
			float targetPitch = pitch * maxBankAngle * config.pitchScale;
			
			targetYaw += yaw * maxYawRate * dt * config.yawScale;
			targetYaw = Rotations::WrapAngle(targetYaw);
			
			setpoint.attitude = Rotations::EulerToQuaternion(targetRoll, targetPitch, targetYaw);
		}
	}
	// Assisted mode
	else if(currentStatus.mode == ControlMode::PositionHold) {
		// In Position Hold, sticks command 3D Velocity and Yaw Rate
		setpoint.activeMask = SetpointMask::VelocityXY | SetpointMask::VelocityZ | SetpointMask::AngularRate;
		
		float maxSpeedXY = 5.0f;	// m/s
		float maxSpeedZ = 2.0f;		// m/s

		// Map Pitch/Roll to XY Velocity
		setpoint.velocity.x = pitch * maxSpeedXY * config.pitchScale;
		setpoint.velocity.y = roll * maxSpeedXY * config.rollScale;
		
		// Map Throttle to Z Velocity
		if(config.thrustAxis == 2) {
			setpoint.velocity.z = input.throttle * maxSpeedZ;
		}
		else {
			setpoint.velocity.x = input.throttle * maxSpeedXY;	// Rover uses throttle for X velocity
		}

		setpoint.angularRates.z = yaw * maxYawRate * config.yawScale;
	}
	// Autonomous mode
	else if(currentStatus.mode == ControlMode::Autonomous) {
		//TODO
	}

	topicSetpoint.Publish(setpoint);
}