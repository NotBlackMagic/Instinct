#pragma once

#include "tx_api.h"

#include "hardware.hpp"

#include "shell.hpp"

class Console {
	public:
		static void Init(UART* uart);
		static void Run(ULONG input);

	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[6144];

		static UART* consolePort;
		static Shell shell;
};