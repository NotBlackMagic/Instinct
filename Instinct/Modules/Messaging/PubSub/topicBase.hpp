/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/PubSub/topicBase.hpp
 * Author:  NotBlackMagic
 * Brief:   PubSub TopicBase structure, required base class for broker registration system.
 */

#pragma once

#include <stdint.h>

class TopicBase {
	public:
		TopicBase(const char* name, uint8_t topicID, uint8_t instance) : nextTopic(nullptr), name(name), topicID(topicID), instance(instance), msgCount(0), publishTimestamp(0), subCount(0), averagePeriodUs(0) {}
		
		virtual ~TopicBase() = default;

		const char* GetName() const { return name; }
		uint8_t GetTopicID() const { return topicID; }
		uint8_t GetInstance() const { return instance; }

		void GetStats(uint32_t& msgCount, uint32_t& publishTimestamp, uint32_t& subCount, float& rate) {
			msgCount = this->msgCount;
			publishTimestamp = this->publishTimestamp;
			subCount = this->subCount;
			if (averagePeriodUs == 0) {
				rate = 0.0f;
			}
			rate = 1000000.0f / static_cast<float>(averagePeriodUs);
		}

		// List pointer used by the Broker
		TopicBase* nextTopic;

	protected:
		const char* name;
		uint8_t topicID;
		uint8_t instance;

		// Stats variables
		volatile uint32_t msgCount;
		volatile uint64_t publishTimestamp;
		volatile uint8_t subCount;
		uint32_t averagePeriodUs;
};