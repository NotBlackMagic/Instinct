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

		// Sensor Publishers
		static Topic<BaroMsg> topicBaro[2];
		static Topic<MagMsg> topicMag[2];

		static void Run(ULONG input);
};