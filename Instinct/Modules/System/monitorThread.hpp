/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/System/monitorThread.hpp
 * Author:	NotBlackMagic
 * Brief:	General system monitor thread, acquire system sensors (power, temperature, etc...).
 */

#pragma once

#include "hardware.hpp"

#include "logger.hpp"
#include "pubSub.hpp"

#include "tx_api.h"

class MonitorThread {
	public:
		static void Init();

	private:
		static TX_THREAD threadPtr;
		static const uint16_t threadStackSize = 4096;
		__attribute__((aligned(32))) static uint8_t threadStack[threadStackSize];

		static void Run(ULONG input);
};