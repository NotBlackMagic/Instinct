/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Messaging/instinctLink.hpp
 * Author:	NotBlackMagic
 * Brief:	High-speed native binary bridge for PubSub streaming, parameter tuning, and topic injection.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "broker.hpp"
#include "logger.hpp"
#include "paramService.hpp"
#include "system.hpp"
#include "tx_api.h"
#include "uart.hpp"

class InstinctLink {
	public:
		enum class CommandID : uint16_t {
			StreamConfig = 0xFF00,		// Configure topic stream rate
			ParamRequestList = 0xFF01,	// Request full parameter table dump
			ParamRequestRead = 0xFF02,	// Request single parameter by index
			ParamSet = 0xFF03,			// Set parameter value by index
			ParamReset = 0xFF04,		// Reset all parameters to factory defaults
			TopicRequestList = 0xFF05	// Request list of all registered Broker topics
		};

		struct __attribute__((packed)) StreamConfigPayload {
			uint8_t topicID;
			uint8_t instance;
			uint16_t rate;
		};

		struct __attribute__((packed)) ParamReadPayload {
			uint16_t index;
		};

		struct __attribute__((packed)) ParamSetPayload {
			uint16_t index;
			uint8_t type;
			ParamValue value;
		};

		struct __attribute__((packed)) ParamInfoPayload {
			uint16_t index;
			uint32_t hash;
			uint8_t type;
			uint8_t flags;
			ParamValue currentVal;
			ParamValue defaultVal;
			ParamValue minVal;
			ParamValue maxVal;
			char name[16];
		};

		struct __attribute__((packed)) TopicInfoPayload {
			uint8_t topicID;
			uint8_t instance;
			uint8_t payloadSize;
			char name[16];
		};

		struct StreamSlot {
			TopicBase* topic;
			uint32_t interval;
			uint64_t timestamp;
			uint32_t version;
		};

		/// @brief Initializes the InstinctLink thread and binds to the target UART.
		static void Init(UART* uart);

		/// @brief Configures a topic stream rate (can be called from firmware or via PC command).
		static bool ConfigureStream(uint8_t topicId, uint8_t instance, float rateHz);

	private:
		static constexpr uint32_t threadStackSize = 4096;
		static TX_THREAD threadPtr;
		__attribute__((aligned(32))) static uint8_t threadStack[threadStackSize];

		static constexpr uint16_t syncWord = 0x55AA;
		static constexpr uint8_t maxStreams = 16;
		static constexpr uint8_t maxPayloadSize = 255;

		static UART* linkPort;
		static StreamSlot streams[maxStreams];

		// Parameter update subscriber for instant ACK forwarding
		static Subscriber<ParamUpdateMsg> subParamUpdate;
		static Topic<ParamUpdateMsg>* topicParamUpdate;

		// Non-blocking parameter table dump state
		static bool isDumpingParams;
		static uint16_t dumpParamIdx;

		static void Run(ULONG input);
		static void ProcessPacket(uint16_t packetID, const uint8_t* payload, uint8_t len);
		static bool SendPacket(uint16_t packetID, const uint8_t* payload, uint8_t len);
		static bool SendParamInfo(uint16_t index);
		static void SendTopicList();
};