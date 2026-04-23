/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/timer.cpp
 */

#include "timer.hpp"

Timer::Timer(TIM_TypeDef *instance) {
	this->instance = instance;
	this->tickFrequencyHz = 0;
	this->autoReloadValue = 0;
	this->isInitialized = false;
}

Status Timer::Init(const Config &config) {
	if(config.frequencyHz == 0 || config.sourceClockHz == 0) {
		return Status::Error;
	}

	// Enable bus clocks
	if(this->instance == TIM1) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
	}
	else if(this->instance == TIM2) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	}
	else if(this->instance == TIM3) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	}
	else if(this->instance == TIM4) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	}
	else if(this->instance == TIM5) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
	}
	else if(this->instance == TIM6) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
	}
	else if(this->instance == TIM7) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);
	}
	else if(this->instance == TIM8) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
	}
	else if(this->instance == TIM9) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM9);
	}
	else if(this->instance == TIM10) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM10);
	}
	else if(this->instance == TIM11) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM11);
	}
	else if(this->instance == TIM12) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM12);
	}
	else if(this->instance == TIM13) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM13);
	}
	else if(this->instance == TIM14) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);
	}
	else if(this->instance == TIM15) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM15);
	}
	else if(this->instance == TIM16) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);
	}
	else if(this->instance == TIM17) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);
	}
	else if(this->instance == TIM18) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM18);
	}
	else {
		return Status::Error;
	}

	// Configure TIM
	LL_TIM_SetCounterMode(this->instance, LL_TIM_COUNTERMODE_UP);

	// Calculate Prescaler and ARR for maximum PWM resolution
	uint32_t totalTicks = config.sourceClockHz / config.frequencyHz;
	uint32_t prescaler = totalTicks / 65536;		// Get prescaler for highest ARR value (max. 65536)
	uint32_t arr = (totalTicks / (prescaler + 1));	// Get real ARR value based on prescaler, required for non integer divisions
	LL_TIM_SetPrescaler(this->instance, prescaler);
	LL_TIM_SetAutoReload(this->instance, arr);
	LL_TIM_EnableARRPreload(this->instance);

	this->tickFrequencyHz = config.sourceClockHz / (prescaler + 1);
	this->autoReloadValue = arr;

	this->isInitialized = true;
	return Status::Ok;
}

void Timer::Start() {
	LL_TIM_EnableCounter(this->instance);
}

void Timer::Stop() {
	LL_TIM_DisableCounter(this->instance);
}

void Timer::Reset() {
	// Generate an update event to force counter to 0 and apply PSC/ARR immediately
	LL_TIM_GenerateEvent_UPDATE(this->instance);
}

TIM_TypeDef* Timer::GetInstance() const {
	return this->instance;
}

uint32_t Timer::GetAutoReload() const {
	return this->autoReloadValue;
}

uint32_t Timer::GetFrequencyHz() const {
	return this->tickFrequencyHz;
}