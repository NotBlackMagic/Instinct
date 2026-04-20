/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/shellPubSub.cpp
 */

 #include "shellModules.hpp"

 #include "broker.hpp"

 static bool CommandPubSub(const char* args) {
	(void)args; // Ignore arguments for the basic 'topics' command

	Logger::Instance().Write("------------------------------------------------------------------------\r\n");
	Logger::Instance().Write(" ID   INST  NAME                   SUBS  MESSAGES    RATE (Hz)\r\n");
	Logger::Instance().Write("------------------------------------------------------------------------\r\n");

	TopicBase* currentTopic = Broker::GetTopicList();

	if(currentTopic == nullptr) {
		Logger::Instance().Write(" No topics registered.\r\n");
		Logger::Instance().Write("------------------------------------------------------------------------\r\n");
		return true;
	}

	char buffer[128];

	// Iterate through the linked list of topics
	while(currentTopic != nullptr) {
		uint32_t msgCount = 0;
		uint32_t timestamp = 0;
		uint32_t subCount = 0;
		float rate = 0.0f;

		// Fetch stats directly from the base class
		currentTopic->GetStats(msgCount, timestamp, subCount, rate);

		// Format and print the row. 
		snprintf(buffer, sizeof(buffer), " %-4u %-5u %-22.22s %-5lu %-11lu %5.1f\r\n",
			currentTopic->GetTopicID(),
			currentTopic->GetInstance(),
			currentTopic->GetName(),
			(unsigned long)subCount,
			(unsigned long)msgCount,
			rate
		);

		Logger::Instance().Write(buffer);

		// Move to the next topic in the broker's list
		currentTopic = currentTopic->nextTopic;
	}

	Logger::Instance().Write("------------------------------------------------------------------------\r\n");
	return true;
}

static const Shell::CommandEntry pubSubCommands[] = {
	{ "topics", CommandPubSub, "Lists all PubSub topics and stats" },
	{ nullptr, nullptr, nullptr } // Terminator
};

static Shell::CommandList pubSubShellNode;

void RegisterPubSubCommands() {
    Shell::RegisterCommands(&pubSubShellNode, pubSubCommands);
}