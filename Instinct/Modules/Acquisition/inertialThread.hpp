#pragma once

#include "hardware.hpp"

#include "logger.hpp"
#include "topics.hpp"

#include "tx_api.h"

class InertialThread {
	public:
		static void Init();
		static void Run(ULONG input);

	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[4096];
};