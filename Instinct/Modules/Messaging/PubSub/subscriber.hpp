#pragma once

#include "tx_api.h"

template <typename T>
class Subscriber {
	public:
		Subscriber() : next(nullptr), isValid(false) {};

		// The link to the next subscriber in the chain
		Subscriber<T>* next;

		// The signal used to wake up the thread
		TX_SEMAPHORE semaphore;

		// Flag to check if semaphore is created
		bool isValid;

		void init() {
			if (!isValid) {
				tx_semaphore_create(&semaphore, (char*)"SUB", 0);
				isValid = true;
			}
		}
};