/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/usbpd.hpp
 * Author:  NotBlackMagic
 * Brief:   USB-PD driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */
 
#pragma once

#include <stdint.h>

#include "stm32n657xx.h"

#include "tx_api.h"

class USBPD {
	public:
		USBPD();
	private:

};