#pragma once

#include "hardware.hpp"

#include "logger.hpp"
#include "pubSub.hpp"

#include "tx_api.h"

class AuxiliaryThread {
	public:
		static void Init();

	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[4096];

		static void Run(ULONG input);
};