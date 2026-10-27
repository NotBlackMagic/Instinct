/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Messaging/PubSub/topic.hpp
 * Author:	NotBlackMagic
 * Brief:	PubSub topic logic (main logic for the PubSub). Inherits from TopicBase class (for broker registration system).
 */

#pragma once

#include <atomic>
#include <stdint.h>
#include <string.h>

#include "logger.hpp"
#include "system.hpp"

#include "tx_api.h"

#include "subscriber.hpp"
#include "topicBase.hpp"

// Hardware Data Memory Barrier + Compiler Memory Clobber
#define BARRIER() __asm volatile( "dmb" ::: "memory" )

template <typename T>
class Topic : public TopicBase {
	public:
		Topic(const char* name, uint8_t topicId, uint8_t instance = 0) : TopicBase(name, topicId, instance), version(0), head(nullptr) {}

		// Delete copy constructors
		Topic(const Topic&) = delete;
		Topic& operator=(const Topic&) = delete;

		/// @brief Subscribe to this topic.
		/// @param sub Pointer to subscriber instance.
		/// @param enableEvent Set true if thread blocks with TX_WAIT_FOREVER/timeout; false if only polling with TX_NO_WAIT.
		/// @return True if subscribed successfully, false if not.
		bool Subscribe(Subscriber<T>* sub, bool enableEvent = true) {
			if(sub == nullptr) {
				return false;
			}

			if(sub->Init(enableEvent) == false) {
				return false;
			}

			// Enter Critical Section
			UINT state = tx_interrupt_control(TX_INT_DISABLE);

			// Prevent duplicate subscriptions
			Subscriber<T>* curr = head;
			while(curr) {
				if(curr == sub) {
					tx_interrupt_control(state);
					return true; 
				}
				curr = curr->next;
			}

			sub->next = head;
			head = sub;

			// Update Stats
			subCount += 1;

			// Exit Critical Section
			tx_interrupt_control(state);

			return true;
		}

		void Unsubscribe(Subscriber<T>* sub) {
			if(sub == nullptr) {
				return;
			}

			// Enter Critical Section
			UINT state = tx_interrupt_control(TX_INT_DISABLE);

			if(head == nullptr) {
				// No subscribers, list is empty
				tx_interrupt_control(state);
				return;
			}

			bool found = false;

			if(head == sub) {
				// Subscriber is head (last added)
				head = sub->next;	// Move head to the next one
				found = true;
			}
			else {
				Subscriber<T>* prev = head;
				while(prev->next != nullptr && prev->next != sub) {
					prev = prev->next;
				}

				if(prev->next == sub) {
					// Bypass the node
					prev->next = sub->next;
					found = true;
				}
			}

			if(found == true) {
				// Clear the subscriber's next pointer for safety
				sub->next = nullptr;

				// Update Stats
				subCount = subCount - 1; 
				
				// Clean up the event group
				if(sub->isValid == true) {
					// This will wake up any thread waiting on this event with TX_DELETED
					tx_event_flags_delete(&sub->eventGroup);
					sub->isValid = false;
				}
			}

			// Exit Critical Section
			tx_interrupt_control(state);
		}

		void Publish(const T& msg) {
			// Capture time
			uint64_t now = Time::GetUs();

			// Enter Critical Section
			UINT state = tx_interrupt_control(TX_INT_DISABLE);

			// Calculate Rate (Period)
			if(publishTimestamp != 0) {
				uint32_t delta = (uint32_t)(now - publishTimestamp);
				if(averagePeriodUs == 0) {
					averagePeriodUs = delta; // Seed the average on first real delta
				}
				else {
					// Moving average Filter, (using fast bitwise shifts
					averagePeriodUs = averagePeriodUs - (averagePeriodUs >> 3) + (delta >> 3);
				}
			}

			// Increment Version (Odd number = "I am writing")
			version.fetch_add(1, std::memory_order_relaxed);

			BARRIER();

			// Copy Data
			data = msg;
			// Update Stats
			msgCount = msgCount + 1;
			publishTimestamp = now;

			BARRIER();

			// Increment Version (Even number = "I am done")
			version.fetch_add(1, std::memory_order_relaxed);

			// Exit Critical Section
			tx_interrupt_control(state);

			// Binary SD Logging hook
			if(logEnabled == true) {
				if(logPeriodUs == 0 || (now - lastLogTimestamp) >= logPeriodUs) {
					lastLogTimestamp = now;
					uint16_t fullTopicId = (static_cast<uint16_t>(instance) << 8) | topicID;
					Logger::Instance().LogData(fullTopicId, reinterpret_cast<const uint8_t*>(&data), sizeof(T));
				}
			}

			// Notify blocking subscribers only
			Subscriber<T>* curr = head;
			while (curr != nullptr) {
				if(curr->useEvent == true && curr->isValid == true) {
					tx_event_flags_set(&curr->eventGroup, 0x01, TX_OR);
				}
				curr = curr->next;
			}
		}

		bool Peek(T& msg, uint32_t* outVersion = nullptr) {
			uint32_t v1, v2;
			uint8_t retries = 0;
			const uint8_t maxRetries = 5;

			while(retries < maxRetries) {
				retries += 1;
								
				v1 = version.load(std::memory_order_acquire);
				// If odd (writing) or 0 (never published), abort/retry
				if(v1 == 0) {
					return false;
				}
				if(v1 & 1) {
					BARRIER();
					continue;
				}

				BARRIER();
				msg = data;
				BARRIER();

				v2 = version.load(std::memory_order_relaxed);
				// If versions match, the read was clean!
				if (v1 == v2) {
					if(outVersion != nullptr) {
						*outVersion = v1;
					}
					return true;
				}
			}

			return false;
		}

		/// @brief Lock-free check if new data was published since last check (consumes update).
		bool HasUpdate(Subscriber<T>* sub) {
			if(sub == nullptr) {
				return false;
			}

			uint32_t currentVer = version.load(std::memory_order_acquire);
			// Return false if never published (0), unchanged, or currently mid-write (odd)
			if(currentVer == 0 || currentVer == sub->lastVersion || (currentVer & 1)) {
				return false;
			}

			sub->missedUpdates = (sub->lastVersion != 0) && ((currentVer - sub->lastVersion) > 2);
			sub->lastVersion = currentVer;
			return true;
		}

		bool Take(Subscriber<T>* sub, T& msg, ULONG ticks) {
			if(sub == nullptr) {
				return false;
			}

			// Fast Lock-Free Path for TX_NO_WAIT (Zero RTOS Kernel Overhead)
			if(ticks == TX_NO_WAIT) {
				// Check if have new version since last call (subscriber saves its last version read)
				uint32_t currentVer = version.load(std::memory_order_relaxed);
				if(currentVer == sub->lastVersion || currentVer == 0) {
					return false;
				}

				uint32_t readVer = 0;
				if(Peek(msg, &readVer) == true) {
					// Each Publish() increments version by 2. A jump > 2 means multiple publishes occurred.
					sub->missedUpdates = (sub->lastVersion != 0) && ((readVer - sub->lastVersion) > 2);
					sub->lastVersion = readVer;
					return true;
				}
				return false;
			}

			// Blocking Path for threads synchronizing on this topic (e.g., TX_WAIT_FOREVER)
			if(sub->useEvent == false || sub->isValid == false) {
				return false;
			}

			ULONG flags;
			UINT status = tx_event_flags_get(&sub->eventGroup, 0x01, TX_OR_CLEAR, &flags, ticks);
		
			if(status == TX_SUCCESS) {
				uint32_t readVer = 0;
				if(Peek(msg, &readVer) == true) {
					sub->missedUpdates = (sub->lastVersion != 0) && ((readVer - sub->lastVersion) > 2);
					sub->lastVersion = readVer;
					return true;
				}
			}

			return false;
		}

		uint8_t GetPayloadSize() const override {
			return static_cast<uint8_t>(sizeof(T));
		}

		bool CopyPayload(uint8_t* outBuf, uint8_t maxLen, uint32_t* outVersion = nullptr) override {
			if(outBuf == nullptr || maxLen < sizeof(T)) {
				return false;
			}

			// Read directly into destination buffer under the seqlock
			return Peek(*reinterpret_cast<T*>(outBuf), outVersion);
		}

		bool PublishRaw(const uint8_t* inBuf, uint8_t len) override {
			if(inBuf == nullptr || len != sizeof(T)) {
				return false;
			}
			Publish(*reinterpret_cast<const T*>(inBuf));
			return true;
		}

	private:
		std::atomic<uint32_t> version;
		T data;
		Subscriber<T>* head;
};