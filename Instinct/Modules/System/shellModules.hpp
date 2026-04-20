/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/shellModules.hpp
 * Author:  NotBlackMagic
 * Brief:   
 */

#pragma once

#include "BoardInfo.hpp"
#include "hardware.hpp"
#include "logger.hpp"
#include "shell.hpp"
#include "version.hpp"

#include "system_stm32n6xx.h"
#include "tx_thread.h"

// Core System
void RegisterSystemCommands();

// Hardware Peripherals
void RegisterI2CCommands();
void RegisterGPIOCommands();
void RegisterSerialCommands();
void RegisterCameraCommands();
void RegisterPWMCommands();
void RegisterFlashCommands();
void RegisterPubSubCommands();