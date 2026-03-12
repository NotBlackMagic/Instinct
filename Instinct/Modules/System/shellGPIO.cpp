/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/shellGPIO.cpp
 */

#include "shellModules.hpp"

static bool CommandGPIO(const char* args) {
	if (args == nullptr || args[0] == '\0') {
		Logger::Instance().Write("Usage: gpio <read|write|toggle> <port> <pin> [val]\r\n");
		return true;
	}
	return true;
}

static const Shell::CommandEntry gpioCommands[] = {
	{ "gpio", CommandGPIO, "GPIO Read/Write/Toggle" },
	{ nullptr, nullptr, nullptr } // Terminator
};

static Shell::CommandList gpioShellNode;

void RegisterGPIOCommands() {
	Shell::RegisterCommands(&gpioShellNode, gpioCommands);
}