/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/System/console.cpp
 */

#include "console.hpp"

TX_THREAD Console::threadPtr;
__attribute__((aligned(32))) uint8_t Console::threadStack[Console::threadStackSize];

UART* Console::consolePort = nullptr;
Shell Console::shell;

void Console::Init(UART* uart) {
	consolePort = uart;

	Console::shell.Init();

	// Register all active command modules
	RegisterSystemCommands();
	RegisterCameraCommands();
	RegisterFDCANCommands();
	// RegisterFlashCommands();
	RegisterI2CCommands();
	RegisterI3CCommands();
	RegisterGPIOCommands();
	RegisterParamCommands();
	RegisterPubSubCommands();
	RegisterPWMCommands();
	// RegisterSerialCommands();
	RegisterStorageCommands();
	RegisterUSBCommands();
	RegisterVisionCommands();

	uint32_t status = tx_thread_create(&threadPtr,	const_cast<char*>("Console"),
													Console::Run, 0,
													threadStack, sizeof(threadStack),
													15, 15,
													TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Console Thread Create Failed.");
	}
}

void Console::Run(ULONG input) {
	uint8_t rxBuffer[16];

	while(1) {
		bool active = false;

		uint16_t len = consolePort->Receive(rxBuffer, sizeof(rxBuffer));

		if(len > 0) {
			shell.Input(rxBuffer, len);
			active = true;
		}

		if(!active) {
			tx_thread_sleep(2);
		}
	}
}