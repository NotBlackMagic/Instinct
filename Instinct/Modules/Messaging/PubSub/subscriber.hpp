/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/PubSub/subscriber.hpp
 * Author:  NotBlackMagic
 * Brief:   PubSub topic subscriber logic.
 */

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