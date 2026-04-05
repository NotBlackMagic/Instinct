#pragma once

#include "hardware.hpp"

#include "exti.hpp"
#include "gpio.hpp"
#include "logger.hpp"
#include "topics.hpp"

#include "tx_api.h"

class InertialThread {
	public:
		static void Init();

	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[4096];

		// Synchronization
		static TX_EVENT_FLAGS_GROUP event;

		// Event Flags Definitions
		static constexpr uint32_t EVT_ONBOARD_IMU_DRDY = 0x01;
		static constexpr uint32_t EVT_EXT1_IMU_DRDY = 0x02;
		static constexpr uint32_t EVT_EXT2_IMU_DRDY = 0x04;

		static void Run(ULONG input);

		static void Ext2IntCallback(void* context, EXTIManager::Edge edge);
};