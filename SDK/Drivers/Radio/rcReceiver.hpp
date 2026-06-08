/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Radio/rcReceiver.hpp
 * Author:  NotBlackMagic
 * Brief:   Multi-protocol RC Receiver driver class for STM32N6.
 */

#pragma once

#include <stdint.h>

#include "status.hpp"
#include "system.hpp"
#include "uart.hpp"

#include "tx_api.h"

/// @brief Driver for handling serial RC receiver protocols (i-Bus, S.Bus, CRSF).
class RCReceiver {
	public:
		static constexpr uint8_t MAX_CHANNELS = 16;

		/// @brief Supported RC protocols.
		enum class Protocol : uint8_t {
			None = 0,
			IBus,
			SBus,
			CRSF
		};

		/// @brief Current state of the RC link.
		enum class LinkState : uint8_t {
			Disconnected = 0,
			Connected,
			LinkLost,
			Failsafe
		};

		/// @brief Standardized RC channel output and telemetry data.
		struct ChannelData {
			uint16_t channels[MAX_CHANNELS];	///< Normalized to 1000-2000us
			uint16_t channelCount;

			LinkState linkState;
			uint32_t timestamp;

			uint8_t linkQuality;			///< 0-100%
			int8_t rssi;					///< Negative dBm
		};

		/// @brief Receiver configuration structure.
		struct Config {
			Protocol protocol;
			int8_t rssiChannelIndex;
		};

		// Delete copy constructors
		RCReceiver(const RCReceiver&) = delete;
		RCReceiver& operator=(const RCReceiver&) = delete;

		/// @brief Constructor.
        /// @param uart Reference to the underlying UART driver instance.
        RCReceiver(UART& uart);

		/// @brief Initializes the receiver parser state.
		/// @param config Receiver configuration.
		/// @return Status::Ok if initialization succeeded.
		Status Init(const Config& config);

		/// @brief Drains the UART buffer and processes the protocol state machine.
		Status ProcessStream();

		/// @brief Retrieves the latest parsed channel data.
		/// @param data Reference to the struct to be filled.
		/// @return Status::Ok if data is valid.
		Status GetChannelData(ChannelData& data);

		/// @brief Transmits telemetry payloads back to the receiver (CRSF/ELRS).
		/// @param payload Pointer to the telemetry frame.
		/// @param length Length of the payload.
		/// @return Status::Ok if transfer started, or Status::Busy.
		Status SendTelemetry(uint8_t* payload, uint16_t length);

	private:
		UART& bus;
		Config config;
		ChannelData channelData;

		static constexpr uint16_t bufferSize = 64;
		__attribute__((aligned(32))) uint8_t buffer[bufferSize];
		
		// Transaction Context
		uint16_t frameIndex;
		uint64_t timestamp;

		// Timeout defines
		static constexpr uint32_t FAILSAFE_TIMEOUT = 500000;
		static constexpr uint32_t BYTE_TIMEOUT = 5000;

		bool ParseIBus(uint8_t byte);
		bool ParseSBus(uint8_t byte);
		bool ParseCRSF(uint8_t byte);

		uint8_t CalculateCRC(const uint8_t* data, uint16_t length);
		void ApplyDefaultChannels();
};