/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Messaging/PubSub/subscriber.hpp
 * Author:	NotBlackMagic
 * Brief:	PubSub topic subscriber logic.
 */

#pragma once

#include "tx_api.h"

template <typename T>
class Subscriber {
	public:
		Subscriber() : next(nullptr), lastVersion(0), missedUpdates(false), isValid(false), useEvent(false) {};

		// The link to the next subscriber in the chain
		Subscriber<T>* next;

		// Tracks the last read Topic version for lock-free TX_NO_WAIT polling
		uint32_t lastVersion;

		// True if more than 1 message was published since the last Take()
		bool missedUpdates;

		// The signal used to wake up the thread
		TX_EVENT_FLAGS_GROUP eventGroup;

		// Flag to check if event group is created
		bool isValid;
		bool useEvent;

		bool Init(bool enableEvent = true) {
			useEvent = enableEvent;
			if(useEvent == true && isValid == false) {
				if(tx_event_flags_create(&eventGroup, (char*)"sub evt") != TX_SUCCESS) {
					isValid = false;
					return false;
				}
				isValid = true;
				return true;
			}
			return true;
		}
};