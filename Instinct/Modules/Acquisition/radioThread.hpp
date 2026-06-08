/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Acquisitions/radioThread.hpp
 * Author:  NotBlackMagic
 * Brief:   Radio receiver polling and publishing thread.
 */

#pragma once

#include "hardware.hpp"

#include "inputs.hpp"
#include "logger.hpp"
#include "pubSub.hpp"
#include "rcReceiver.hpp"

#include "tx_api.h"

class RadioThread {
	public:
		enum class ChannelMap : uint8_t {
			AETR,	// Aileron(Roll), Elevator(Pitch), Throttle, Rudder(Yaw) -> Standard FlySky/Futaba
			TAER,	// Throttle, Aileron(Roll), Elevator(Pitch), Rudder(Yaw) -> Standard Spektrum/FrSky
			Surface	// Car/Rover (CH1: Steering, CH2: Drive)
		};

		// Configuration struct for the thread
		struct Config {
			RCReceiver::Protocol protocol;
			int8_t rssiChannelIndex;
			ChannelMap channelMap;
			uint16_t deadband; // In microseconds (e.g., 3us)
		};

		static void Init();

	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[4096];

		static void Run(ULONG input);

        static float NormalizeBidirectional(uint16_t raw, uint16_t deadband);
        static float NormalizeUnidirectional(uint16_t raw);
};