#pragma once

#include <stdint.h>
#include <string.h>

#include "system.hpp"

#include "tx_api.h"

#include "subscriber.hpp"

#define BARRIER() __asm volatile( "dmb" ::: "memory" )

template <typename T>
class Topic {
	public:
		Topic(const char* name, uint8_t id) : name(name), id(id), version(0), head(nullptr), msgCount(0), msgTimestamp(0), subCount(0) {}

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
				while (prev->next != nullptr && prev->next != sub) {
					prev = prev->next;
				}

				if (prev->next == sub) {
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

			// Increment Version (Odd number = "I am writing")
			version = version + 1;

			// Barrier: Ensure version increment lands before data write starts
			BARRIER();
			// Copy Data
			data = msg;
			// Update Stats
			msgCount += 1;
			msgTimestamp = now;
			// Barrier: Ensure data write finishes before version increments again
			BARRIER();

			// Increment Version (Even number = "I am done")
			version += 1;

			// Notify all subscribers
			Subscriber<T>* curr = head;
			while (curr != nullptr) {
				tx_semaphore_put(&curr->semaphore);
				curr = curr->next;
			}
		}

		bool Peak(T& msg) {
			uint32_t v1, v2;
			uint8_t retries = 0;
			const uint8_t maxRetries = 5;

			do {
				retries += 1;
				if(retries > maxRetries) {
					return false;
				}
				
				v1 = version;
				if (v1 & 1) {
					continue;
				} 

				BARRIER();
				msg = data;
				BARRIER();

				v2 = version;
			} while (v1 != v2);

			return true;
		}

		bool Take(Subscriber<T>* sub, T& msg, ULONG ticks) {
			if(sub == nullptr) {
				return false;
			}

			UINT status = tx_semaphore_get(&sub->semaphore, ticks);
		
			if(status == TX_SUCCESS) {
				Peak(msg); // Reuse the read function
				return true;
			}

			return false;
		}

		const char* GetName() const { return name; }
		uint8_t GetID() const { return id; }
		void GetStats(uint32_t& msgCount, uint32_t& msgTimestamp, uint32_t& subCount) {
			msgCount = this->msgCount;
			msgTimestamp = this->msgTimestamp;
			subCount = this->subCount;
		}

	private:
		const char* name;
		uint8_t id;
		volatile uint32_t version;
		T data;
		Subscriber<T>* head;

		// Stats variables
		volatile uint32_t msgCount;
		volatile uint64_t msgTimestamp;
		volatile uint8_t subCount;
};