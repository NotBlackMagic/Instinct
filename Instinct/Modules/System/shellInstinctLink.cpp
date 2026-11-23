/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/System/shellInstinctLink.cpp
 * Author:	NotBlackMagic
 * Brief:	Shell commands for InstinctLink stream inspection, rate configuration, and link diagnostics.
 */

#include <stdlib.h>
#include <string.h>

#include "broker.hpp"
#include "instinctLink.hpp"
#include "shellModules.hpp"

static void PrintHelp() {
	Logger::Instance().Write("--- InstinctLink Commands ---\r\n");
	Logger::Instance().Write("  ilink stats                     - Display packet counters, drops, and throughput\r\n");
	Logger::Instance().Write("  ilink streams                   - List active topic stream slots and measured rates\r\n");
	Logger::Instance().Write("  ilink stream <id> <rate> [inst] - Set stream rate in Hz (rate=0 to stop, default inst=0)\r\n");
	Logger::Instance().Write("  ilink stop <id> [inst]          - Stop streaming a topic\r\n");
	Logger::Instance().Write("  ilink topics                    - List all registered Broker topics with IDs and sizes\r\n");
	Logger::Instance().Write("  ilink reset                     - Reset link and stream statistics counters\r\n");
	Logger::Instance().Write("-----------------------------\r\n");
}

static void FormatDataRate(float bytesPerSec, char* outBuf, size_t maxLen) {
	if(bytesPerSec >= 1024.0f * 1024.0f) {
		snprintf(outBuf, maxLen, "%.2f MB/s", bytesPerSec / (1024.0f * 1024.0f));
	}
	else if(bytesPerSec >= 1024.0f) {
		snprintf(outBuf, maxLen, "%.1f kB/s", bytesPerSec / 1024.0f);
	}
	else {
		snprintf(outBuf, maxLen, "%.0f B/s", bytesPerSec);
	}
}

static bool CommandInstinctLink(const char* args) {
	if(args == nullptr || args[0] == '\0') {
		PrintHelp();
		return true;
	}

	char action[16] = {0};
	// Parse target action
	int32_t count = sscanf(args, "%15s", action);
	if(count < 1) {
		PrintHelp();
		return true;
	}

	if(strcmp(action, "stats") == 0) {
		InstinctLink::Statistics stats = InstinctLink::GetStatistics();

		char txRateBuf[16];
		char rxRateBuf[16];
		FormatDataRate(stats.txByteRate, txRateBuf, sizeof(txRateBuf));
		FormatDataRate(stats.rxByteRate, rxRateBuf, sizeof(rxRateBuf));

		Logger::Instance().Write("\r\n--- InstinctLink Statistics ---\r\n");
		Logger::Instance().Printf("  TX Packets   : %lu\r\n", stats.txPackets);
		Logger::Instance().Printf("  RX Packets   : %lu\r\n", stats.rxPackets);
		Logger::Instance().Printf("  TX Drops     : %lu\r\n", stats.txDrops);
		Logger::Instance().Printf("  CRC Errors   : %lu\r\n", stats.crcErrors);
		Logger::Instance().Printf("  TX Throughput: %s\r\n", txRateBuf);
		Logger::Instance().Printf("  RX Throughput: %s\r\n", rxRateBuf);
		Logger::Instance().Write("-------------------------------\r\n");
		return true;
	}

	if(strcmp(action, "streams") == 0) {
		Logger::Instance().Write("--------------------------------------------------------------------------------\r\n");
		Logger::Instance().Write(" SLOT  TOPIC                ID   INST  TARGET(Hz)  ACTUAL(Hz)  TX PKTS   DATA RATE\r\n");
		Logger::Instance().Write("--------------------------------------------------------------------------------\r\n");

		uint8_t activeCount = 0;
		char buffer[128];
		char rateBuf[16];

		for(uint8_t i = 0; i < InstinctLink::maxStreams; i++) {
			InstinctLink::StreamStatistics sStats;
			if(InstinctLink::GetStreamStatistics(i, sStats) == true) {
				TopicBase* topic = Broker::GetTopicBase(sStats.topicID, sStats.instance);
				const char* topicName = (topic != nullptr && topic->GetName() != nullptr) ? topic->GetName() : "unknown";

				FormatDataRate(sStats.byteRate, rateBuf, sizeof(rateBuf));

				snprintf(buffer, sizeof(buffer), "  %-4u %-20.20s %-4u %-5u %-11.1f %-11.1f %-9lu %-10s\r\n",
					i,
					topicName,
					sStats.topicID,
					sStats.instance,
					sStats.targetRate,
					sStats.actualRate,
					sStats.txPackets,
					rateBuf
				);

				Logger::Instance().Write(buffer);
				activeCount++;
			}
		}

		if(activeCount == 0) {
			Logger::Instance().Write("  No active topic streams.\r\n");
		}

		Logger::Instance().Write("--------------------------------------------------------------------------------\r\n");
		return true;
	}

	if(strcmp(action, "stream") == 0) {
		uint32_t topicID = 0;
		float rateHz = 0.0f;
		uint32_t instance = 0;

		int32_t parsed = sscanf(args, "%*s %u %f %u", &topicID, &rateHz, &instance);
		if(parsed < 2) {
			Logger::Instance().Write("Usage: ilink stream <topicID> <rateHz> [instance (default 0)]\r\n");
			return true;
		}

		TopicBase* topic = Broker::GetTopicBase(static_cast<uint8_t>(topicID), static_cast<uint8_t>(instance));
		if(topic == nullptr) {
			Logger::Instance().Printf("Error: Topic ID %u (instance %u) not found in Broker.\r\n", topicID, instance);
			return true;
		}

		bool success = InstinctLink::ConfigureStream(static_cast<uint8_t>(topicID), static_cast<uint8_t>(instance), rateHz);
		if(success == true) {
			if(rateHz > 0.0f) {
				Logger::Instance().Printf("Stream configured: '%s' (ID %u, inst %u) @ %.1f Hz\r\n",
					topic->GetName() ? topic->GetName() : "", topicID, instance, rateHz);
			}
			else {
				Logger::Instance().Printf("Stream disabled: '%s' (ID %u, inst %u)\r\n",
					topic->GetName() ? topic->GetName() : "", topicID, instance);
			}
		}
		else {
			Logger::Instance().Write("Error: Failed to configure stream (all slots full).\r\n");
		}
		return true;
	}

	if(strcmp(action, "stop") == 0) {
		uint32_t topicID = 0;
		uint32_t instance = 0;

		int32_t parsed = sscanf(args, "%*s %u %u", &topicID, &instance);
		if(parsed < 1) {
			Logger::Instance().Write("Usage: ilink stop <topicID> [instance (default 0)]\r\n");
			return true;
		}

		InstinctLink::ConfigureStream(static_cast<uint8_t>(topicID), static_cast<uint8_t>(instance), 0.0f);
		Logger::Instance().Printf("Stream stopped for Topic ID %u (instance %u)\r\n", topicID, instance);
		return true;
	}

	if(strcmp(action, "topics") == 0) {
		Logger::Instance().Write("--------------------------------------------------\r\n");
		Logger::Instance().Write(" ID   INST  PAYLOAD (B)  TOPIC NAME\r\n");
		Logger::Instance().Write("--------------------------------------------------\r\n");

		TopicBase* curr = Broker::GetTopicList();
		if(curr == nullptr) {
			Logger::Instance().Write("  No topics registered in Broker.\r\n");
		}

		char buffer[96];
		while(curr != nullptr) {
			snprintf(buffer, sizeof(buffer), " %-4u %-5u %-12u %s\r\n",
				curr->GetTopicID(),
				curr->GetInstance(),
				curr->GetPayloadSize(),
				curr->GetName() ? curr->GetName() : ""
			);
			Logger::Instance().Write(buffer);
			curr = curr->nextTopic;
		}

		Logger::Instance().Write("--------------------------------------------------\r\n");
		return true;
	}

	if(strcmp(action, "reset") == 0) {
		InstinctLink::ResetStatistics();
		Logger::Instance().Write("InstinctLink statistics reset.\r\n");
		return true;
	}

	Logger::Instance().Printf("Unknown InstinctLink action '%s'.\r\n", action);
	PrintHelp();
	return true;
}

static const Shell::CommandEntry instinctLinkCommands[] = {
	{ "ilink", CommandInstinctLink, "InstinctLink stream management, throughput, and link diagnostics" },
	{ "link", CommandInstinctLink, "Alias for ilink" },
	{ nullptr, nullptr, nullptr }
};

static Shell::CommandList instinctLinkShellNode;

void RegisterInstinctLinkCommands() {
	Shell::RegisterCommands(&instinctLinkShellNode, instinctLinkCommands);
}