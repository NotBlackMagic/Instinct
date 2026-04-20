/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/storageThread.hpp
 * Author:  NotBlackMagic
 * Brief:   Handles storage systems i.e. the SD card and FileX for it.
 */

#pragma once

#include "hardware.hpp"
#include "sdmmc.hpp"
#include "sd.hpp"

#include "tx_pluman6_sd_driver.h"

#include "logger.hpp"
#include "pubSub.hpp"

#include "tx_api.h"
#include "fx_api.h"

class StorageThread {
	public:
		static void Init();

		static bool IsReady();
		static FX_MEDIA* GetMedia();

	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[4096];

		// FileX Memory
		static FX_MEDIA sdMedia;
		__attribute__((aligned(32))) static uint8_t sdMediaPool[4096]; // Minimum 512 bytes for sector buffer

		// Thread-safe state flag
		static std::atomic<bool> storageReady;

		static void Run(ULONG input);
};