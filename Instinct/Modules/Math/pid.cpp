/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Math/pid.cpp
 */

#include "pid.hpp"
#include <math.h>

PID::PID() {
	Reset();
}

Status PID::Init(const Config& config) {
	this->config = config;
	Reset();
}

void PID::Reset() {
	integSum = 0.0f;
	prevErr = 0.0f;
	prevMeas = 0.0f;
	prevD = 0.0f;
}

float PID::Update(float target, float measurement, float dt) {
	if(dt <= 0.0f) {
		return 0.0f;
	}

	float error = target - measurement;

	// Proportional
	float pTerm = config.kp * error;

	// Integral
	integSum += config.ki * error * dt;

	// Anti-windup: Clamp the integral
	if(integSum > config.integLim) {
		integSum = config.integLim;
	}
	else if(integSum < -config.integLim) {
		integSum = -config.integLim;
	}

	float iTerm = integSum;

	// Derivative (Measurement-based to prevent setpoint kick?)
	float rawDerivative = -(measurement - prevMeas) / dt;

	// Low-Pass Filter on the D-Term
	float alpha = CalculateLowPassAlpha(dt, config.diffFcHz);
	float filteredDerivative = prevD + alpha * (rawDerivative - prevD);
	float dTerm = config.kd * filteredDerivative;

	// Feed-Forward
	float fTerm = config.kf * target;

	// State Updates
	prevErr = error;
	prevMeas = measurement;
	prevD = filteredDerivative;

	// Sum and Saturate
	float output = pTerm + iTerm + dTerm + fTerm;
	if(output > config.outLim) {
		return config.outLim;
	}
	else if(output < -config.outLim) {
		return -config.outLim;
	}

	return output;
}

float PID::CalculateLowPassAlpha(float dt, float cutoffHz) {
	if(cutoffHz <= 0.0f) {
		return 1.0f; // Filter disabled
	}
	float rc = 1.0f / (6.283185307f * cutoffHz);
	return dt / (rc + dt);
}