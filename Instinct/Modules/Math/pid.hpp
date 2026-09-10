/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Math/pid.hpp
 * Author:	NotBlackMagic
 * Brief:	Proportional-Integral-Derivative controller with anti-windup and D-term filtering.
 */

#pragma once

#include <stdint.h>

#include "status.hpp"

class PID {
	public:
		struct Config {
			float kp;		// Proportional gain
			float ki;		// Integral gain
			float kd;		// Differential gain
			float kf;		// Feed-Forward gain
			
			float integLim;	// Anti-windup clamping limit
			float outLim;	// Absolute maximum output of the controller
			float diffFcHz;	// Low-Pass Filter cutoff for the D-term
		};

		PID();

		/// @brief EKF prediction.
		/// @param config PID controller configuration.
		/// @return Status::Ok if initialization succeeded.
		Status Init(const Config& config);

		/// @brief PID controller update step (runs once).
		/// @param target 		Desired target or set point.
		/// @param measurement	Current measurment.
		/// @param dt			Time delta in seconds
		/// @return New control value
		float Update(float target, float measurement, float dt);

		/// @brief Flushes the integral accumulator and filter history.
		void Reset();

	private:
		// PID filter configurations
		Config config;

		// Internal PID state tracking
		float integSum;
		float prevErr;
		float prevMeas;

		// For the D-Term low pass filter
		float prevD;

		// Private Helper Functions
		float CalculateLowPassAlpha(float dt, float cutoffHz);
};