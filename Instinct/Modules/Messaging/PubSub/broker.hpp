/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:    Instinct/Modules/Messaging/PubSub/broker.hpp
 * Author:  NotBlackMagic
 * Brief:   PubSub topic broker, to register and get all topics.
 */

#pragma once

#include <string.h>

#include "Topic.hpp"

class Broker {
	public:
		// Called once when a topic is instantiated
		static bool RegisterTopic(TopicBase* topic) {
			if(topic == nullptr) {
				return false;
			}
			if(!TopicAddress::IsValidTopicInstance(topic->GetInstance())) {
				// Reject All (0xFE) and Protocol (0xFF)
				return false;
			}
			if(GetTopicBase(topic->GetTopicID(), topic->GetInstance()) != nullptr) {
				// Prevent duplicate (topicID, instance) registration
				return false;
			}

			// Insert at the head of the list (simple, fast, O(1))
			topic->nextTopic = head;
			head = topic;
			return true;
		}

		// Fast type-safe lookup for flight code
		template <typename T>
		static Topic<T>* GetTopic(TopicID topicID, uint8_t instance = TopicInstance::Default) {
			return GetTopic<T>(static_cast<uint8_t>(topicID), instance);
		}

		template <typename T>
		static Topic<T>* GetTopic(uint8_t topicID, uint8_t instance = TopicInstance::Default) {
			TopicBase* base = GetTopicBase(topicID, instance);
			return static_cast<Topic<T>*>(base);
		}

		/// @brief Type-erased lookup used by InstinctLink for dynamic streaming and injection
		static TopicBase* GetTopicBase(uint8_t topicID, uint8_t instance = 0) {
			TopicBase* curr = head;
			while(curr != nullptr) {
				if(curr->GetTopicID() == topicID && curr->GetInstance() == instance) {
					return curr;
				}
				curr = curr->nextTopic;
			}
			return nullptr;
		}

		// Used by 3rd parties to find a topic by name
		template <typename T>
		static Topic<T>* GetTopicByName(const char* name) {
			TopicBase* curr = head;
			while(curr != nullptr) {
				if(strcmp(curr->GetName(), name) == 0) {
					return static_cast<Topic<T>*>(curr);
				}
				curr = curr->nextTopic;
			}
			return nullptr; // Topic not found
		}

		static void EnableLogging(TopicID topicID, uint8_t instance = TopicInstance::All, uint32_t rateHz = 0) {
			EnableLogging(static_cast<uint8_t>(topicID), instance, rateHz);
		}

		static void EnableLogging(uint8_t topicID, uint8_t instance = TopicInstance::All, uint32_t rateHz = 0) {
			TopicBase* curr = head;
			while(curr != nullptr) {
				// If instance matches OR wildcard (all instances of a topic type) is used
				if(curr->GetTopicID() == topicID && (instance == TopicInstance::All || curr->GetInstance() == instance)) {
					curr->EnableLogging(rateHz);
					if(instance != TopicInstance::All) {
						return;
					}
				}
				curr = curr->nextTopic;
			}
		}

		static void DisableLogging(TopicID topicID, uint8_t instance = TopicInstance::All) {
			DisableLogging(static_cast<uint8_t>(topicID), instance);
		}

		static void DisableLogging(uint8_t topicID, uint16_t instance = TopicInstance::All) {
			TopicBase* curr = head;
			while(curr != nullptr) {
				// If instance matches OR wildcard (all instances of a topic type) is used
				if(curr->GetTopicID() == topicID && (instance == TopicInstance::All || curr->GetInstance() == instance)) {
					curr->DisableLogging();
					if(instance != TopicInstance::All) {
						return;
					}
				}
				curr = curr->nextTopic;
			}
		}

		static TopicBase* GetTopicList() { return head; }

	private:
		static inline TopicBase* head = nullptr; 
};