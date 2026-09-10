/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Actuation/actuatorThread.cpp
 */

#include "actuatorThread.hpp"

TX_THREAD ActuatorThread::threadPtr;
uint8_t ActuatorThread::threadStack[4096];

Subscriber<ActuatorMsg> ActuatorThread::subActuators;
Topic<ActuatorMsg>* ActuatorThread::topicActuators = nullptr;

// Default configuration (e.g., 4 motors, 12 disabled/unused)
ActuatorThread::PinConfig ActuatorThread::pinConfigs[16];
PWM* ActuatorThread::hwPWM[16] = {nullptr};

void ActuatorThread::Init() {
	// Map logical output to physical (hardware) PWM channels
	hwPWM[0] = &pwm1Ch1;
	hwPWM[1] = &pwm1Ch2;
	hwPWM[2] = &pwm1Ch3;
	hwPWM[3] = &pwm1Ch4;
	hwPWM[4] = &pwm2Ch1;
	hwPWM[5] = &pwm2Ch2;
	hwPWM[6] = &pwm2Ch3;
	hwPWM[7] = &pwm2Ch4;

	// Initialize outputs/pin/pwm (Drone Defaults): Defaulting to 4 Quadcopter motors.
	for (uint8_t i = 0; i < 4; i++) {
		pinConfigs[i].type = ChannelType::Motor;
		pinConfigs[i].min = 1000;
		pinConfigs[i].max = 2000;
		pinConfigs[i].center = 1000;
		pinConfigs[i].idle = 900;
	}
	// Set the rest as centered servos
	for (uint8_t i = 4; i < 16; i++) {
		pinConfigs[i].type = ChannelType::Servo;
		pinConfigs[i].min = 1000;
		pinConfigs[i].max = 2000;
		pinConfigs[i].center = 1500;
		pinConfigs[i].idle = 1500;
	}

	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("ACT_Loop"),
													ActuatorThread::Run, 0,
													threadStack, sizeof(threadStack),
													1, 0,
													TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX ACT Loop Thread Create Failed.");
	}
}

void ActuatorThread::Run(ULONG input) {
	(void)input;
	LOG_INFO("Actuator Thread Initialized.");

	// Boot Synchronization: Wait for the Controller to register the topic
	while(topicActuators == nullptr) {
		topicActuators = Broker::GetTopic<ActuatorMsg>(static_cast<uint8_t>(TopicID::Actuator), 0);
		tx_thread_sleep(10);
	}
	topicActuators->Subscribe(&subActuators);

	ActuatorMsg actuatorMsg{};

	// Initialize default safe status
	ForceSafeOutputs();

	while(1) {
		// Deadman switch failsafe: if no message in 50ms assume controller thread crashed -> go to failsafe
		if(topicActuators->Take(&subActuators, actuatorMsg, 50) == true) {
			if(actuatorMsg.isArmed == true) {
				// Map and apply active outputs
				for(uint8_t i = 0; i < 16; i++) {
					uint16_t pulseUs = MapToMicroseconds(actuatorMsg.outputs[i], pinConfigs[i]);
					if(hwPWM[i] != nullptr) {
						hwPWM[i]->SetPulseWidth(pulseUs);
					}
				}
			} 
			else {
				// Message received, but system is disarmed
				ForceSafeOutputs();
			}
		}
		else {
			// Timeout crash, the controller stopped sending messages.
			LOG_ERR("ActuatorThread: Controller IPC Timeout! Force HW failsafe.");
			ForceSafeOutputs();
		}
	}
}

uint16_t ActuatorThread::MapToMicroseconds(float output, const PinConfig& config) {
	if(config.type == ChannelType::Motor) {
		// Output is [0.0 to 1.0]. Clamp for safety.
		if(output < 0.0f) {
			output = 0.0f;
		}
		if(output > 1.0f) {
			output = 1.0f;
		}
		return config.min + (uint16_t)(output * (config.max - config.min));
	} 
	else {
		// Servo: Output is [-1.0 to 1.0]. 
		if(output < -1.0f) {
			output = -1.0f;
		}
		if(output > 1.0f) {
			output = 1.0f;
		}

		if(output >= 0.0f) {
			return config.center + (uint16_t)(output * (config.max - config.center));
		}
		else {
			return config.center - (uint16_t)(-output * (config.center - config.min));
		}
	}
}

void ActuatorThread::ForceSafeOutputs() {
	for(uint8_t i = 0; i < 16; i++) {
		if(hwPWM[i] != nullptr) {
			hwPWM[i]->SetPulseWidth(pinConfigs[i].idle);
		}
	}
}