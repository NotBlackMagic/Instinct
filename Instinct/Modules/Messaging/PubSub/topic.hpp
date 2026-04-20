/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/PubSub/topic.hpp
 * Author:  NotBlackMagic
 * Brief:   PubSub topic logic (main logic for the PubSub). Inherits from TopicBase class (for broker registration system).
 */

#pragma once

#include <atomic>
#include <stdint.h>
#include <string.h>

#include "system.hpp"

#include "tx_api.h"

#include "subscriber.hpp"
#include "topicBase.hpp"

// #define BARRIER() __asm volatile( "dmb" ::: "memory" )

template <typename T>
class Topic : public TopicBase {
	public:
		Topic(const char* name, uint8_t topicId, uint8_t instance = 0) : TopicBase(name, topicId, instance), version(0), head(nullptr) {}

		// Delete copy constructors
		Topic(const Topic&) = delete;
		Topic& operator=(const Topic&) = delete;

		void Subscribe(Subscriber<T>* sub) {
			if(sub == nullptr) {
				return;
			}

			sub->init();

			// Enter Critical Section
			UINT state = tx_interrupt_control(TX_INT_DISABLE);

			// Prevent duplicate subscriptions
			Subscriber<T>* curr = head;
			while(curr) {
				if(curr == sub) {
					tx_interrupt_control(state);
					return; 
				}
				curr = curr->next;
			}

			sub->next = head;
			head = sub;

			// Update Stats
			subCount += 1;

			// Exit Critical Section
			tx_interrupt_control(state);
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
				
				// Clean up the semaphore
				// This will wake up any thread waiting on this semaphore with TX_DELETED
				tx_semaphore_delete(&sub->semaphore);
				sub->isValid = false;
			}

			// Exit Critical Section
			tx_interrupt_control(state);
		}

		void Publish(const T& msg) {
			// Capture time
			uint64_t now = Time::GetUs();

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
			// version = version + 1;
			version.fetch_add(1, std::memory_order_relaxed);

			// Barrier: Ensure version increment lands before data write starts
			// BARRIER();
			std::atomic_thread_fence(std::memory_order_release);
			// Copy Data
			data = msg;
			// Update Stats
			msgCount += 1;
			publishTimestamp = now;
			// Barrier: Ensure data write finishes before version increments again
			// BARRIER();
			std::atomic_thread_fence(std::memory_order_release);

			// Increment Version (Even number = "I am done")
			// version += 1;
			version.fetch_add(1, std::memory_order_relaxed);

			// Notify all subscribers
			Subscriber<T>* curr = head;
			while (curr != nullptr) {
				tx_semaphore_put(&curr->semaphore);
				curr = curr->next;
			}
		}

		bool Peek(T& msg) {
			uint32_t v1, v2;
			uint8_t retries = 0;
			const uint8_t maxRetries = 5;

			do {
				retries += 1;
				if(retries > maxRetries) {
					return false;
				}
				
				// v1 = version;
				v1 = version.load(std::memory_order_acquire);
				// If odd, publisher is currently writing. Try again.
				if(v1 & 1) {
					continue;
				} 

				// BARRIER();
				msg = data;
				// BARRIER();

				std::atomic_thread_fence(std::memory_order_acquire);

				// v2 = version;
				v2 = version.load(std::memory_order_relaxed);
			} while (v1 != v2);

			return true;
		}

		bool Take(Subscriber<T>* sub, T& msg, ULONG ticks) {
			if(sub == nullptr) {
				return false;
			}

			UINT status = tx_semaphore_get(&sub->semaphore, ticks);
		
			if(status == TX_SUCCESS) {
				Peek(msg); // Reuse the read function
				return true;
			}

			return false;
		}

	private:
		// volatile uint32_t version;
		std::atomic<uint32_t> version;
		T data;
		Subscriber<T>* head;
};