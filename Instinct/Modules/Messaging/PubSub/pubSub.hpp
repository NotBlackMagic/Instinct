/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Messaging/PubSub/pubSub.hpp
 * Author:  NotBlackMagic
 * Brief:   Global/general include for the PubSub system, only need to include this file to use PubSub.
 */

#pragma once

// Core PubSub Infrastructure
#include "topicBase.hpp"
#include "subscriber.hpp"
#include "topic.hpp"
#include "broker.hpp"

// Messaging headers/defines
#include "common.hpp"
#include "power.hpp"
#include "sensors.hpp"
#include "state.hpp"
#include "topicIDs.hpp"