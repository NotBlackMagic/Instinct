/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:    Instinct/Modules/System/shellGPIO.cpp
 */

#include "shellModules.hpp"

static void PrintHelp() {
	Logger::Instance().Write("--- GPIO Commands ---\r\n");
	Logger::Instance().Write("  read <port> <pin> [val]   - Read GPIO output/insput state\r\n");
	Logger::Instance().Write("  write <port> <pin> [val]  - Write GPIO output state\r\n");
	Logger::Instance().Write("  toggle <port> <pin>       - Toggle GPIO output state\r\n");
	Logger::Instance().Write("------------------------\r\n");
}

static bool CommandGPIO(const char* args) {
	if (args == nullptr || args[0] == '\0') {
		PrintHelp();
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