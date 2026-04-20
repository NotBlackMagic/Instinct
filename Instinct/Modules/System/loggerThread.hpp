/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/loggerThread.hpp
 * Author:  NotBlackMagic
 * Brief:   Thread handling writing log buffers to SD card.
 */

#pragma once

#include "logger.hpp"
#include "storageThread.hpp"

#include "tx_api.h"
#include "fx_api.h"

class LoggerThread {
	public:
		static void Init();

	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[4096];

		__attribute__((aligned(32))) static uint8_t writeBuffer[512];

		static void Run(ULONG input);
};