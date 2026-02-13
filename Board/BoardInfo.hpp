/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Board/BoardInfo.hpp
 * Author:  NotBlackMagic
 * Brief:   File contains static board (hardware) version information.
 */

#pragma once

#include <stdint.h>

struct BoardInfo {
    static constexpr const char* Name = "PlumaN6";
    static constexpr const char* Revision = "Rev A";
    static constexpr const char* DesignDate = "2025-09-01";
    static constexpr const char* DesignLocation = "Lisbon, PT";
    static constexpr const char* Designer = "NotBlackMagic";
    
    // Hardware Capabilities
    static constexpr bool HasExternalFlash = true;
    static constexpr bool HasWiFi = false;

	// Memory Sizes
    static constexpr uint32_t SizeROM = 255 * 1024; // 255KB
    static constexpr uint32_t SizeRAM = 256 * 1024; // 256KB
    
    // External Memory
    static constexpr uint32_t SizeExtFlash = 64 * 1024 * 1024; 
    static constexpr uint32_t SizeExtRAM   = 32 * 1024 * 1024;
};