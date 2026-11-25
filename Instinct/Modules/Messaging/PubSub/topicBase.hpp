/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Messaging/PubSub/topicBase.hpp
 * Author:	NotBlackMagic
 * Brief:	PubSub TopicBase structure, required base class for broker registration system.
 */

#pragma once

#include <stdint.h>

class TopicBase {
	public:
		TopicBase(const char* name, uint8_t topicID, uint8_t instance)
			: 	nextTopic(nullptr), 
				name(name),
				topicID(topicID),
				instance(instance),
				msgCount(0),
				timestamp(0),
				subCount(0),
				averagePeriodUs(0),
				logEnabled(false),
				logPeriodUs(0),
				lastLogTimestamp(0) {}
		
		virtual ~TopicBase() = default;

		/// @brief Getter for topic name.
		const char* GetName() const { return name; }

		/// @brief Getter for topic ID.
		uint8_t GetTopicID() const { return topicID; }

		/// @brief Getter for topic instance.
		uint8_t GetInstance() const { return instance; }

		/// @brief Virtual function getter for topic message size (payload size), overwritten in topic.hpp.
		virtual uint8_t GetPayloadSize() const = 0;

		/// @brief Virtual function to byte copy topic message, overwritten in topic.hpp.
		virtual bool CopyPayload(uint8_t* outBuf, uint8_t maxLen, uint32_t* outVersion = nullptr) = 0;

		/// @brief Virtual function to publish topic message from byte array, overwritten in topic.hpp.
		virtual bool PublishRaw(const uint8_t* inBuf, uint8_t len) = 0;

		/// @brief Enable binary SD logging for this topic.
		/// @param rateHz Target log rate in Hz. Set to 0 to log at native publish rate.
		/// @return None.
		void EnableLogging(uint32_t rateHz = 0) {
			logEnabled = true;
			if(rateHz > 0) {
				logPeriodUs = 1000000 / rateHz;
			}
			else {
				logPeriodUs = 0; // 0 = every publish
			}
		}

		/// @brief Disable binary SD logging for this topic.
		/// @return None.
		void DisableLogging() {
			logEnabled = false;
		}

		/// @brief Check if binary SD logging is enabled for this topic.
		/// @return None.
		bool IsLoggingEnabled() const { return logEnabled; }

		/// @brief Get basic statistics of this topic.
		/// @param msgCount		Number of messages published.
		/// @param timestamp	Last published message timestamp.
		/// @param subCount		Subscriber count to this topic.
		/// @param rate			Average message publishing rate.
		/// @return None.
		void GetStats(uint32_t& msgCount, uint64_t& timestamp, uint32_t& subCount, float& rate) {
			msgCount = this->msgCount;
			timestamp = this->timestamp;
			subCount = this->subCount;
			if (averagePeriodUs == 0) {
				rate = 0.0f;
			}
			else {
				rate = 1000000.0f / static_cast<float>(averagePeriodUs);
			}
		}

		// List pointer used by the Broker
		TopicBase* nextTopic;

	protected:
		const char* name;
		uint8_t topicID;
		uint8_t instance;

		// Stats variables
		volatile uint32_t msgCount;
		volatile uint64_t timestamp;
		volatile uint8_t subCount;
		uint32_t averagePeriodUs;

		// Logging control variables
		bool logEnabled;
		uint32_t logPeriodUs;
		uint64_t lastLogTimestamp;
};