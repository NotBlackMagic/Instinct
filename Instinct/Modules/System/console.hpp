/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/System/console.hpp
 * Author:	NotBlackMagic
 * Brief:	System console/shell thread.
 */

#pragma once

#include "tx_api.h"

#include "hardware.hpp"

#include "shell.hpp"
#include "shellModules.hpp"

class Console {
	public:
		static void Init(UART* uart);
		static void Run(ULONG input);

	private:
		static TX_THREAD threadPtr;
		static const uint16_t threadStackSize = 4096;
		__attribute__((aligned(32))) static uint8_t threadStack[threadStackSize];

		static UART* consolePort;
		static Shell shell;
};