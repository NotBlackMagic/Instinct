/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
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
		static void RegisterTopic(TopicBase* topic) {
			if(topic == nullptr) {
				return;
			}
			
			// Insert at the head of the list (simple, fast, O(1))
			topic->nextTopic = head;
			head = topic;
		}

		// Fast integer lookup (Recommended for flight code)
		template <typename T>
		static Topic<T>* GetTopic(uint8_t topicId, uint8_t instance = 0) {
			TopicBase* curr = head;
			while (curr != nullptr) {
				if (curr->GetTopicID() == topicId && curr->GetInstance() == instance) {
					return static_cast<Topic<T>*>(curr);
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
					// Safe in embedded if we trust the developer to use the right type.
					// (RTTI/dynamic_cast is usually disabled in embedded C++)
					return static_cast<Topic<T>*>(curr);
				}
				curr = curr->nextTopic;
			}
			return nullptr; // Topic not found
		}

		static TopicBase* GetTopicList() { return head; }

	private:
		static inline TopicBase* head = nullptr; 
};