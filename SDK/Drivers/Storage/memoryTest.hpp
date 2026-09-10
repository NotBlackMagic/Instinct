/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	SDK/Drivers/Storage/memoryTest.hpp
 * Author:	NotBlackMagic
 * Brief:	AA
 */


#pragma once

#include <stdint.h>

#include "hardware.hpp"
#include "logger.hpp"
#include "status.hpp"

class MemoryTest {
	public:
		/// @brief Fast verification (e.g., 16 KB) for boot-time POST in HardwareInit()
		static Status RunQuickTest(uint32_t sizeBytes = 16 * 1024);

		/// @brief Full benchmark (Bus, Memory-Mapped, DMA) for Console CLI
		static void RunBenchmark(uint32_t sizeBytes = 16 * 1024);
};