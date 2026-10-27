/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Control/controllerThread.cpp
 */

#include "controllerThread.hpp"

TX_THREAD ControllerThread::threadPtr;
__attribute__((aligned(32))) uint8_t ControllerThread::threadStack[ControllerThread::threadStackSize];

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

// Parameters regsitered in the parameter service
ControllerThread::ControllerParams ControllerThread::params{};
ParamService::ParamList ControllerThread::paramList{};
// Null-terminated parameter definition table placed in Flash (.rodata)
const ParamService::ParamEntry ControllerThread::paramEntries[] = {
	// Name, Pointer, Default, Min, Max, Flags
	// Attitude
	ParamService::ParamEntry("MC_ATT_P", &ControllerThread::params.attP, 0.00f, 0.00f, 15.00f),

	// Position Loop
	ParamService::ParamEntry("MPC_POS_P", &ControllerThread::params.posKp, 0.00f, 0.00f, 5.00f),
	ParamService::ParamEntry("MPC_POS_VEL_LIM", &ControllerThread::params.posVelLimit, 0.00f, 1.00f, 20.00f),

	// Velocity Loop
	ParamService::ParamEntry("MPC_VEL_P", &ControllerThread::params.velKp, 0.00f, 0.00f, 10.00f),
	ParamService::ParamEntry("MPC_VEL_I", &ControllerThread::params.velKi, 0.00f, 0.00f, 5.00f),
	ParamService::ParamEntry("MPC_VEL_D", &ControllerThread::params.velKd, 0.00f, 0.00f, 1.00f),
	ParamService::ParamEntry("MPC_VEL_ILIM", &ControllerThread::params.velIntLim, 0.00f, 0.00f, 5.00f),
	ParamService::ParamEntry("MPC_VEL_ACC_LIM", &ControllerThread::params.velAccelLimit, 0.00f, 0.00f, 30.00f),
	ParamService::ParamEntry("MPC_VEL_D_LPF", &ControllerThread::params.velDlpHz, 0.00f, 0.00f, 60.00f),
	ParamService::ParamEntry("MPC_THR_HOVER", &ControllerThread::params.hoverThrust, 0.00f, 0.00f, 0.80f),

	// Roll Rate PID
	ParamService::ParamEntry("MC_ROLLRATE_P", &ControllerThread::params.rollRateKp, 0.00f, 0.00f, 2.00f),
	ParamService::ParamEntry("MC_ROLLRATE_I", &ControllerThread::params.rollRateKi, 0.00f, 0.00f, 2.00f),
	ParamService::ParamEntry("MC_ROLLRATE_D", &ControllerThread::params.rollRateKd, 0.00f, 0.00f, 0.10f),
	ParamService::ParamEntry("MC_ROLLRATE_ILIM", &ControllerThread::params.rollRateIntLim, 0.00f, 0.00f, 1.00f),

	// Pitch Rate PID
	ParamService::ParamEntry("MC_PITCHRATE_P", &ControllerThread::params.pitchRateKp, 0.00f, 0.00f, 2.00f),
	ParamService::ParamEntry("MC_PITCHRATE_I", &ControllerThread::params.pitchRateKi, 0.00f, 0.00f, 2.00f),
	ParamService::ParamEntry("MC_PITCHRATE_D", &ControllerThread::params.pitchRateKd, 0.00f, 0.00f, 0.10f),
	ParamService::ParamEntry("MC_PITCHRATE_ILIM", &ControllerThread::params.pitchRateIntLim,0.00f, 0.00f, 1.00f),

	// Yaw Rate PID
	ParamService::ParamEntry("MC_YAWRATE_P", &ControllerThread::params.yawRateKp, 0.00f, 0.00f, 2.00f),
	ParamService::ParamEntry("MC_YAWRATE_I", &ControllerThread::params.yawRateKi, 0.00f, 0.00f, 2.00f),
	ParamService::ParamEntry("MC_YAWRATE_D", &ControllerThread::params.yawRateKd, 0.00f, 0.00f, 0.05f),
	ParamService::ParamEntry("MC_YAWRATE_ILIM", &ControllerThread::params.yawRateIntLim, 0.00f, 0.00f, 1.00f),

	ParamService::ParamEntry("MC_RATE_D_LPF", &ControllerThread::params.rateDlpHz, 0.00f, 0.00f, 100.00f),

	// Motor Limits
	ParamService::ParamEntry("MOT_SPIN_MIN",&ControllerThread::params.motorSpinMin,0.00f, 0.00f,0.20f, ParamFlags::Protected),
	ParamService::ParamEntry("MOT_SPIN_MAX",&ControllerThread::params.motorSpinMax,1.00f, 0.50f,1.00f, ParamFlags::Protected),

	// Sentinel to mark end of array
	ParamService::ParamEntry()
};

void ControllerThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("Controller"),
													ControllerThread::Run, 0,
													threadStack, sizeof(threadStack),
													3, 3, 
													TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX CTRL Loop Thread Create Failed.");
	}

	// Register the output topic to the Broker
	Broker::RegisterTopic(&topicActuators);

	// Register parameters and load factory defaults into ControllerThread::params
	ParamService::RegisterParams(&paramList, paramEntries);

	// Set motor limits from loaded parameters
	currentMixer.geometry = Matrix<16, 6>();
	for(uint8_t i = 0; i < 16; i++) {
		currentMixer.outMin[i] = 0.0f;
		currentMixer.outMax[i] = 0.0f;
	}

	// Initialize Rate PIDs using the registered defaults
	ReloadParams();

	// Initialize the mixer matrix, default mapping to a drone

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
			// Check if any controller parameters changed (single-cycle check when false)
			if(ParamService::HasUpdate(&paramList) == true) {
				ReloadParams();
			}

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
			bool isArmed = (currentStatus.arming == ArmingStatus::Armed);
			bool justArmed = (!wasArmed && isArmed);
			bool modeChanged = (previousMode != currentStatus.mode);

			if(wasArmed != isArmed) {
				// Lock protected parameters while armed, unlock when disarmed
				ParamService::SetProtectedLock(isArmed);
			}

			wasArmed = isArmed;
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
			if(isArmed == true) {
				// Copy to a working struct so cascade overwrites don't corrupt currentSetpoint
				ControlSetpointMsg workingSetpoint = currentSetpoint;
				
				// Cascade from highest active mode down to rates
				if(workingSetpoint.activeMask & (SetpointMask::PositionXY | SetpointMask::PositionZ)) {
					RunPositionController(stateMsg, workingSetpoint, dt);
				}

				if(workingSetpoint.activeMask & (SetpointMask::VelocityXY | SetpointMask::VelocityZ)) {
					RunVelocityController(stateMsg, workingSetpoint, dt);
				}

				if(workingSetpoint.activeMask & SetpointMask::Attitude) {
					RunAttitudeController(stateMsg, workingSetpoint, dt);
				}

				if(workingSetpoint.activeMask & SetpointMask::AngularRate) {
					RunRateController(stateMsg, workingSetpoint, currentWrench, dt);
				}

				// Direct linear effort, e.g. Throttle
				if(workingSetpoint.activeMask & SetpointMask::LinearEffortXY) {
					currentWrench.force.x = workingSetpoint.linearEffort.x;
					currentWrench.force.y = workingSetpoint.linearEffort.y;
				}
				if(workingSetpoint.activeMask & SetpointMask::LinearEffortZ) {
					currentWrench.force.z = workingSetpoint.linearEffort.z;
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
		else {
			// Safeguard: If the middleware queue fails, force a yield.
			tx_thread_sleep(1);
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
	float totalZAccel = targetAccel.z - gravity;	// Gravity is positive Z (Down)
	setpoint.linearEffort.z = params.hoverThrust * (totalZAccel / -gravity);

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

	// Convert vector part to angular rates
	setpoint.angularRates.x = 2.0f * params.attP * qErr.x;
	setpoint.angularRates.y = 2.0f * params.attP * qErr.y;
	setpoint.angularRates.z = 2.0f * params.attP * qErr.z;

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

void ControllerThread::ReloadParams() {
	// Position Controllers
	PID::Config posCfg = { params.posKp, 0.0f, 0.0f, 0.0f, 0.0f, params.posVelLimit, 0.0f };
	pidPosition[0].SetConfig(posCfg);
	pidPosition[1].SetConfig(posCfg);
	pidPosition[2].SetConfig(posCfg);

	// Velocity Controllers
	PID::Config velCfg = {params.velKp, params.velKi, params.velKd, 0.0f, params.velIntLim, params.velAccelLimit, params.velDlpHz};
	pidVelocity[0].SetConfig(velCfg);
	pidVelocity[1].SetConfig(velCfg);
	pidVelocity[2].SetConfig(velCfg);

	// Roll Rate PID Config
	PID::Config rollRateCfg = {params.rollRateKp, params.rollRateKi, params.rollRateKd, 0.0f, params.rollRateIntLim, 1.0f, params.rateDlpHz};
	pidRate[0].SetConfig(rollRateCfg);

	// Pitch Rate PID Config
	PID::Config pitchRateCfg = {params.pitchRateKp, params.pitchRateKi, params.pitchRateKd, 0.0f, params.pitchRateIntLim, 1.0f, params.rateDlpHz};
	pidRate[1].SetConfig(pitchRateCfg);

	// Yaw Rate PID Config
	PID::Config yawRateCfg = {params.yawRateKp, params.yawRateKi, params.yawRateKd, 0.0f, params.yawRateIntLim, 1.0f, params.rateDlpHz};
	pidRate[2].SetConfig(yawRateCfg);

	// Set motor limits (Unidirectional: 0% to 100%)
	for(uint8_t i = 0; i < 4; i++) {
		currentMixer.outMin[i] = params.motorSpinMin;
		currentMixer.outMax[i] = params.motorSpinMax;
	}

	LOG_INFO("Controller: Parameters reloaded.");
}