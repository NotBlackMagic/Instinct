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

#include "topic.hpp"

class Broker {
	public:
		/// @brief Registers a new topic, adds it to the end of the linked list.
		/// @note Called once when a topic is instantiated.
		/// @param topic	Pointer of the topic to register.
		/// @return Returns true on successful registration, false otherwise.
		static bool RegisterTopic(TopicBase* topic) {
			if(topic == nullptr) {
				return false;
			}
			if(TopicAddress::IsValidTopicInstance(topic->GetInstance()) == false) {
				// Reject All (0xFE) and Protocol (0xFF)
				return false;
			}
			if(GetTopicBase(topic->GetTopicID(), topic->GetInstance()) != nullptr) {
				// Prevent duplicate, based on topicID and instanceID, registration
				return false;
			}

			// Insert at the head of the list
			topic->nextTopic = head;
			head = topic;
			return true;
		}

		/// @brief Fast type-safe lookup for flight code.
		/// @param topicID	Topic ID of the topic to get.
		/// @param instance	Optional topic instance of the topic to get, TopicInstance::Default (0x00) otherwise.
		/// @return Returns the pointer to the topic based on the parameter, nullptr otherwise. 
		template <typename T>
		static Topic<T>* GetTopic(TopicID topicID, uint8_t instance = TopicInstance::Default) {
			return GetTopic<T>(static_cast<uint8_t>(topicID), instance);
		}

		template <typename T>
		static Topic<T>* GetTopic(uint8_t topicID, uint8_t instance = TopicInstance::Default) {
			TopicBase* base = GetTopicBase(topicID, instance);
			return static_cast<Topic<T>*>(base);
		}

		/// @brief Type-erased lookup used by InstinctLink for dynamic streaming and injection.
		/// @param topicID	Topic ID of the topic to get.
		/// @param instance	Topic instance of the topic to get.
		/// @return Returns the pointer to the topic based on the parameter, nullptr otherwise. 
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

		/// @brief Find and return topic by name.
		/// @param name		Topic name of the topic to get.
		/// @return Returns the pointer to the topic based on the parameter, nullptr otherwise. 
		template <typename T>
		static Topic<T>* GetTopicByName(const char* name) {
			TopicBase* curr = head;
			while(curr != nullptr) {
				if(strcmp(curr->GetName(), name) == 0) {
					return static_cast<Topic<T>*>(curr);
				}
				curr = curr->nextTopic;
			}
			return nullptr;	// Topic not found
		}

		/// @brief Enables binary logging, typically to SD card, of selected topic at defined rate.
		/// @param topicID	Topic ID
		/// @param instance Optional topic instance ID, TopicInstance::All (0xFE) otherwise.
		/// @param rateHz	Optional target log rate in Hz. By default is 0 to log at native publish rate.
		/// @return None.
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

		/// @brief Disables binary logging of selected topic.
		/// @param topicID	Topic ID
		/// @param instance Optional topic instance ID, TopicInstance::All (0xFE) otherwise.
		/// @return None.
		static void DisableLogging(TopicID topicID, uint8_t instance = TopicInstance::All) {
			DisableLogging(static_cast<uint8_t>(topicID), instance);
		}

		static void DisableLogging(uint8_t topicID, uint8_t instance = TopicInstance::All) {
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

		/// @brief Returns the base/head pointer to the topic list.
		/// @return Topic list head pointer.
		static TopicBase* GetTopicList() { return head; }

	private:
		static inline TopicBase* head = nullptr; 
};