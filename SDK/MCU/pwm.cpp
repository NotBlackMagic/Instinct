/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/pwm.cpp
 */

#include "pwm.hpp"

PWM::PWM(Timer &timer, Channel channel) : timer(timer) {
	this->channel = channel;
}

Status PWM::Init(const Config &config) {
	// Configure TIM for PWM mode
	LL_TIM_OC_SetMode(this->timer.GetInstance(), static_cast<uint32_t>(this->channel), LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetPolarity(this->timer.GetInstance(), static_cast<uint32_t>(this->channel), static_cast<uint32_t>(config.polarity));
	LL_TIM_OC_EnablePreload(this->timer.GetInstance(), static_cast<uint32_t>(this->channel));
	LL_TIM_EnableAllOutputs(this->timer.GetInstance());

	// Initialize with 0% duty cycle to be safe
	SetCompareValue(0);

	return Status::Ok;
}

void PWM::Start() {
	LL_TIM_CC_EnableChannel(this->timer.GetInstance(), static_cast<uint32_t>(this->channel));
}

void PWM::Stop() {
	LL_TIM_CC_DisableChannel(this->timer.GetInstance(), static_cast<uint32_t>(this->channel));
}

void PWM::SetDutyCycle(float dutyCycle) {
	// Clamp to valid percentages
	if(dutyCycle < 0.0f) {
		dutyCycle = 0.0f;
	}
	if(dutyCycle > 1.0f) {
		dutyCycle = 1.0f;
	}

	// Calculate the raw Capture/Compare value based on the Timer's Auto-Reload Register (ARR)
	// 100% duty cycle requires CCR to be ARR + 1
	uint32_t arr = this->timer.GetAutoReload();
	uint32_t ccr = static_cast<uint32_t>(dutyCycle * (arr + 1));

	SetCompareValue(ccr);
}

void PWM::SetPulseWidth(uint32_t pulseWidthUs) {
	uint32_t timFreq = this->timer.GetFrequencyHz();

	uint64_t exactTicks = (static_cast<uint64_t>(pulseWidthUs) * timFreq) / 1000000ULL;

	uint32_t arr = this->timer.GetAutoReload();
	uint32_t ccr = static_cast<uint32_t>(exactTicks);
	// limit to 100% duty cycle
	if(ccr > (arr + 1)) {
		ccr = arr + 1;
	}

	SetCompareValue(ccr);
}

// Private helper to route the calculated CCR to the correct hardware channel
void PWM::SetCompareValue(uint32_t value) {
	switch(this->channel) {
		case Channel::Ch1:
			LL_TIM_OC_SetCompareCH1(this->timer.GetInstance(), value);
			break;
		case Channel::Ch2:
			LL_TIM_OC_SetCompareCH2(this->timer.GetInstance(), value);
			break;
		case Channel::Ch3:
			LL_TIM_OC_SetCompareCH3(this->timer.GetInstance(), value);
			break;
		case Channel::Ch4:
			LL_TIM_OC_SetCompareCH4(this->timer.GetInstance(), value);
			break;
		case Channel::Ch5:
			LL_TIM_OC_SetCompareCH5(this->timer.GetInstance(), value);
			break;
		case Channel::Ch6:
			LL_TIM_OC_SetCompareCH6(this->timer.GetInstance(), value);
			break;
	}
}