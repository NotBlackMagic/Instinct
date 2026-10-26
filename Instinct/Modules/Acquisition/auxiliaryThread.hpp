/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Acquisitions/auxiliaryThread.hpp
 * Author:	NotBlackMagic
 * Brief:	Low rate sensors polling and acquisition i.e. barometer and magnetometer.
 */

#pragma once

#include "hardware.hpp"

#include "logger.hpp"
#include "pubSub.hpp"

#include "tx_api.h"

class AuxiliaryThread {
	public:
		static void Init();

	private:
		static TX_THREAD threadPtr;
		static const uint16_t threadStackSize = 4096;
		__attribute__((aligned(32))) static uint8_t threadStack[threadStackSize];

		// Sensor Publishers
		static Topic<BaroMsg> topicBaro[2];
		static Topic<MagMsg> topicMag[2];

		static void Run(ULONG input);
};