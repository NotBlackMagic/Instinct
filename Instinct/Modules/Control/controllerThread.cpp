/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Control/controllerThread.cpp
 */

#include "controllerThread.hpp"

TX_THREAD ControllerThread::threadPtr;
uint8_t ControllerThread::threadStack[8192];

// Output Publisher
Topic<ActuatorMsg> ControllerThread::topicActuators("actuators", static_cast<uint8_t>(TopicID::Actuator), 0);

// Subscribers
Subscriber<StateMsg> ControllerThread::subState;
Subscriber<VehicleStatusMsg> ControllerThread::subStatus;
Subscriber<ControlSetpointMsg> ControllerThread::subSetpoint;

// Topic Pointers
Topic<StateMsg>* ControllerThread::topicState = nullptr;
Topic<VehicleStatusMsg>* ControllerThread::topicStatus = nullptr;
Topic<ControlSetpointMsg>* ControllerThread::topicSetpoint = nullptr;

// Internal State
VehicleStatusMsg ControllerThread::currentStatus{};
ControlSetpointMsg ControllerThread::currentSetpoint{};
ControlWrench ControllerThread::currentWrench{};
uint64_t ControllerThread::lastStateTimestamp = 0;

PID ControllerThread::pidPosition[3];
PID ControllerThread::pidVelocity[3];
PID ControllerThread::pidRate[3];
ControllerThread::MixerGeometry ControllerThread::currentMixer{};

void ControllerThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("CTRL_Loop"),
													ControllerThread::Run, 0,
													threadStack, sizeof(threadStack),
													2, 0, 
													TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX CTRL Loop Thread Create Failed.");
	}

	// Register the output topic to the Broker
	Broker::RegisterTopic(&topicActuators);

	// Initialize PIDs (Drone Defaults): { kp, ki, kd, kf, integLim, outLim, diffFcHz }
	PID::Config posCfg = { 1.5f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f, 0.0f };			// P-only, max 5 m/s
	PID::Config velCfg = { 2.0f, 0.5f, 0.1f, 0.0f, 2.0f, 10.0f, 20.0f };		// Full PID for velocity
	PID::Config rateCfg = { 0.15f, 0.05f, 0.005f, 0.0f, 0.5f, 1.0f, 40.0f };	// Full PID for rates, output clamped to -1.0 to 1.0
	for(uint8_t i = 0; i < 3; i++) {
		pidPosition[i].Init(posCfg);
		pidVelocity[i].Init(velCfg);
		pidRate[i].Init(rateCfg);
	}

	// Initialize the mixer matrix, default mapping to a drone
	// Clear the mixer matrix and limits
	currentMixer.geometry = Matrix<16, 6>();
	for(uint8_t i = 0; i < 16; i++) {
		currentMixer.outMin[i] = 0.0f;
		currentMixer.outMax[i] = 0.0f;
	}

	// Set/"load" mixer matrix values for a "X"-Quadcopter geometry
	// Columns: 0=Fx, 1=Fy, 2=Fz, 3=Tx, 4=Ty, 5=Tz
	// Indexing: [motorIndex * 6 + column]

	// Motor 0: Front Right (CCW)
	currentMixer.geometry.data[0 * 6 + 2] = 1.0f;	// +Z Thrust
	currentMixer.geometry.data[0 * 6 + 3] = -1.0f;	// -X Roll
	currentMixer.geometry.data[0 * 6 + 4] = -1.0f;	// -Y Pitch
	currentMixer.geometry.data[0 * 6 + 5] = 1.0f;	// +Z Yaw

	// Motor 1: Rear Left (CCW)
	currentMixer.geometry.data[1 * 6 + 2] = 1.0f;
	currentMixer.geometry.data[1 * 6 + 3] = 1.0f;
	currentMixer.geometry.data[1 * 6 + 4] = 1.0f;
	currentMixer.geometry.data[1 * 6 + 5] = 1.0f;

	// Motor 2: Front Left (CW)
	currentMixer.geometry.data[2 * 6 + 2] = 1.0f;
	currentMixer.geometry.data[2 * 6 + 3] = 1.0f;
	currentMixer.geometry.data[2 * 6 + 4] = -1.0f; 
	currentMixer.geometry.data[2 * 6 + 5] = -1.0f; 

	// Motor 3: Rear Right (CW)
	currentMixer.geometry.data[3 * 6 + 2] = 1.0f;
	currentMixer.geometry.data[3 * 6 + 3] = -1.0f; 
	currentMixer.geometry.data[3 * 6 + 4] = 1.0f;
	currentMixer.geometry.data[3 * 6 + 5] = -1.0f; 

	// Set motor limits (Unidirectional: 0% to 100%)
	for(uint8_t i = 0; i < 4; i++) {
		currentMixer.outMin[i] = 0.0f;
		currentMixer.outMax[i] = 1.0f;
	}
}

void ControllerThread::Run(ULONG input) {
	(void)input;
	LOG_INFO("Controller Thread Initialized.");

	// Boot Synchronization: Wait for Estimator to come online
	while(topicState == nullptr) {
		topicState = Broker::GetTopic<StateMsg>(static_cast<uint8_t>(TopicID::State), 0);
		tx_thread_sleep(10);
	}
	topicState->Subscribe(&subState);

	// Bind to Status and Command topics (Non-blocking)
	topicStatus = Broker::GetTopic<VehicleStatusMsg>(static_cast<uint8_t>(TopicID::VehicleStatus), 0);
	if(topicStatus != nullptr) {
		topicStatus->Subscribe(&subStatus);
	}

	topicSetpoint = Broker::GetTopic<ControlSetpointMsg>(static_cast<uint8_t>(TopicID::ControlSetpoint), 0);
	if(topicSetpoint != nullptr) {
		topicSetpoint->Subscribe(&subSetpoint);
	}

	StateMsg stateMsg{};
	ActuatorMsg actuatorMsg{};

	// Initialize default safe status
	currentStatus.arming = ArmingStatus::Disarmed;
	currentStatus.mode = ControlMode::ManualRate;
	// Tracking for state changes (edge-case transitions)
	bool wasArmed = false;
	ControlMode previousMode = ControlMode::ManualRate;

	while(1) {
		// Control Heartbeat: Synchronize with the EKF state output
		if(topicState->Take(&subState, stateMsg, TX_WAIT_FOREVER) == true) {
			// Calculate dt based on the EKF state timestamps
			float dt = (float)(stateMsg.timestamp - lastStateTimestamp) / 1000000.0f;
			if(dt <= 0.0f || dt > 0.01f) {
				dt = 0.001f; // Fallback to nominal 1kHz loop time
			}
			lastStateTimestamp = stateMsg.timestamp;


			// Update Arming Status and Setpoint (Only updates variables if new data arrived)
			if(topicStatus != nullptr) {
				topicStatus->Take(&subStatus, currentStatus, TX_NO_WAIT);
			}
			if(topicSetpoint != nullptr) {
				topicSetpoint->Take(&subSetpoint, currentSetpoint, TX_NO_WAIT);
			}

			// Catch Arming Transition or Mode Switch
			bool justArmed = (!wasArmed && currentStatus.arming == ArmingStatus::Armed);
			bool modeChanged = (previousMode != currentStatus.mode);
			wasArmed = (currentStatus.arming == ArmingStatus::Armed);
			previousMode = currentStatus.mode;

			if(justArmed == true || modeChanged == true) {
				// Reset PID memory to prevent spikes, smooth out transfer/transitions
				for(uint8_t i = 0; i < 3; i++) {
					pidPosition[i].Reset();
					pidVelocity[i].Reset();
					pidRate[i].Reset();
				}
				LOG_INFO("Controller: PIDs reset due to state/mode transition.");
			}

			// Zero out the outputs
			currentWrench.force = {0.0f, 0.0f, 0.0f};
			currentWrench.torque = {0.0f, 0.0f, 0.0f};

			// Execute Control Math
			if(currentStatus.arming == ArmingStatus::Armed) {
				// Cascade from highest active mode down to rates
				if(currentSetpoint.activeMask & (SetpointMask::PositionXY | SetpointMask::PositionZ)) {
					RunPositionController(stateMsg, currentSetpoint, dt);
				}

				if(currentSetpoint.activeMask & (SetpointMask::VelocityXY | SetpointMask::VelocityZ)) {
					RunVelocityController(stateMsg, currentSetpoint, dt);
				}

				if(currentSetpoint.activeMask & SetpointMask::Attitude) {
					RunAttitudeController(stateMsg, currentSetpoint, dt);
				}

				if(currentSetpoint.activeMask & SetpointMask::AngularRate) {
					RunRateController(stateMsg, currentSetpoint, currentWrench, dt);
				}

				// Direct linear effort, e.g. Throttle
				if(currentSetpoint.activeMask & SetpointMask::LinearEffortXY) {
					currentWrench.force.x = currentSetpoint.linearEffort.x;
					currentWrench.force.y = currentSetpoint.linearEffort.y;
				}
				if(currentSetpoint.activeMask & SetpointMask::LinearEffortZ) {
					currentWrench.force.z = currentSetpoint.linearEffort.z;
				}

				// Actuator output mixer
				actuatorMsg.isArmed = true;
				AllocateActuators(currentWrench, actuatorMsg);
			}
			else {
				// Disarmed or Failsafe -> Safe actuators
				actuatorMsg.isArmed = false;
				for(uint8_t i = 0; i < 16; i++) {
					actuatorMsg.outputs[i] = 0.0f;
				}
			}

			// Publish
			actuatorMsg.timestamp = stateMsg.timestamp;
			topicActuators.Publish(actuatorMsg);
		}
	}
}

void ControllerThread::RunPositionController(const StateMsg& state, ControlSetpointMsg& setpoint, float dt) {
	setpoint.velocity.x = pidPosition[0].Update(setpoint.position.x, state.position.x, dt);
	setpoint.velocity.y = pidPosition[1].Update(setpoint.position.y, state.position.y, dt);
	setpoint.velocity.z = pidPosition[2].Update(setpoint.position.z, state.position.z, dt);

	setpoint.activeMask = setpoint.activeMask | SetpointMask::VelocityXY | SetpointMask::VelocityZ;
}

void ControllerThread::RunVelocityController(const StateMsg& state, ControlSetpointMsg& setpoint, float dt) {
	// Calculate desired accelerations using your PID class
	Vector3f targetAccel;
	targetAccel.x = pidVelocity[0].Update(setpoint.velocity.x, state.velocity.x, dt);
	targetAccel.y = pidVelocity[1].Update(setpoint.velocity.y, state.velocity.y, dt);
	targetAccel.z = pidVelocity[2].Update(setpoint.velocity.z, state.velocity.z, dt);

	// Drone Specific Kinematics: Z-Accel to Throttle
	float totalZAccel = targetAccel.z - gravity; // Gravity is positive Z (Down)
	setpoint.linearEffort.z = hoverThrust * (totalZAccel / -gravity); 

	// Clamp
	if(setpoint.linearEffort.z > 1.0f) {
		setpoint.linearEffort.z = 1.0f;
	}
	if(setpoint.linearEffort.z < 0.0f) {
		setpoint.linearEffort.z = 0.0f;
	}

	// Drone Specific Kinematics: XY-Accel to Pitch/Roll
	float currentYaw = Rotations::ExtractYawFromQuaternion(state.attitude);
	float cosYaw = cosf(currentYaw);
	float sinYaw = sinf(currentYaw);

	// Rotate world-frame accel into body-frame
	float accelBodyX = targetAccel.x * cosYaw + targetAccel.y * sinYaw;
	float accelBodyY = -targetAccel.x * sinYaw + targetAccel.y * cosYaw;

	// Convert lateral acceleration to bank angles (Small angle approx: accel/gravity = angle in rad)
	float targetPitch = -accelBodyX / gravity; 
	float targetRoll = accelBodyY / gravity;
	float targetYaw = Rotations::ExtractYawFromQuaternion(setpoint.attitude);

	setpoint.attitude = Rotations::EulerToQuaternion(targetRoll, targetPitch, targetYaw);

	setpoint.activeMask = setpoint.activeMask | SetpointMask::Attitude | SetpointMask::LinearEffortZ;
}

void ControllerThread::RunAttitudeController(const StateMsg& state, ControlSetpointMsg& setpoint, float dt) {
	(void)dt;
	// Calculate quaternion error: q_error = q_current^-1 * q_target
	Quaternion currentInv = Rotations::Inverse(state.attitude);
	Quaternion qErr = Rotations::Multiply(currentInv, setpoint.attitude);

	// Normalize error to shortest path
	if(qErr.w < 0.0f) {
		qErr.x = -qErr.x;
		qErr.y = -qErr.y;
		qErr.z = -qErr.z;
	}

	// Convert vector part to angular rates using a simple P-gain (e.g., 4.5f)
	float attP = 4.5f; 
	setpoint.angularRates.x = 2.0f * attP * qErr.x;
	setpoint.angularRates.y = 2.0f * attP * qErr.y;
	setpoint.angularRates.z = 2.0f * attP * qErr.z;

	setpoint.activeMask = setpoint.activeMask | SetpointMask::AngularRate;
}

void ControllerThread::RunRateController(const StateMsg& state, const ControlSetpointMsg& setpoint, ControlWrench& outputWrench, float dt) {
	// The PID class outputs the exact normalized torque [-1.0 to 1.0] required
	outputWrench.torque.x = pidRate[0].Update(setpoint.angularRates.x, state.angularVelocity.x, dt);
	outputWrench.torque.y = pidRate[1].Update(setpoint.angularRates.y, state.angularVelocity.y, dt);
	outputWrench.torque.z = pidRate[2].Update(setpoint.angularRates.z, state.angularVelocity.z, dt);
}

void ControllerThread::AllocateActuators(const ControlWrench& wrench, ActuatorMsg& actuators) {
	// Pack the wrench into a 6x1 column vector
	Matrix<6, 1> wrenchVec(InitType::Uninitialized);
	wrenchVec.data[0] = wrench.force.x;
	wrenchVec.data[1] = wrench.force.y;
	wrenchVec.data[2] = wrench.force.z;
	wrenchVec.data[3] = wrench.torque.x;
	wrenchVec.data[4] = wrench.torque.y;
	wrenchVec.data[5] = wrench.torque.z;

	// Hardware-Accelerated Matrix Multiplication: [16x1 Outputs] = [16x6 Geometry] * [6x1 Wrench]
	Matrix<16, 1> outVec = currentMixer.geometry * wrenchVec;

	// Find the maximum overflow (Desaturation Logic). Handles cases where desired output overlows, so compensate (desaturate) other outputs together
	float maxOverflow = 0.0f;
	float maxUnderflow = 0.0f;
	for(uint8_t i = 0; i < 16; i++) {
		// Only apply desaturation to motors (outMin == 0.0)
		if(currentMixer.outMin[i] >= 0.0f) { 
			if(outVec.data[i] > currentMixer.outMax[i]) {
				float overflow = outVec.data[i] - currentMixer.outMax[i];
				if(overflow > maxOverflow) {
					maxOverflow = overflow;
				}
			}
			else if(outVec.data[i] < currentMixer.outMin[i]) {
				float underflow = currentMixer.outMin[i] - outVec.data[i];
				if(underflow > maxUnderflow) {
					maxUnderflow = underflow;
				}
			}
		}
	}

	// Apply outputs with desaturation and clamping
	for(uint8_t i = 0; i < 16; i++) {
		float output = outVec.data[i];
		
		// Shift motor outputs if exceeded limits (desaturation)
		if(currentMixer.outMin[i] >= 0.0f) {
			output -= maxOverflow;
			output += maxUnderflow;
		}

		// Absolute clamp
		if(output > currentMixer.outMax[i]) {
			output = currentMixer.outMax[i];
		}
		if(output < currentMixer.outMin[i]) {
			output = currentMixer.outMin[i];
		}
		actuators.outputs[i] = output;
	}
}