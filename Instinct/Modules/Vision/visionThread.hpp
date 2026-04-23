/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/visionThread.hpp
 * Author:  NotBlackMagic
 * Brief:   
 */

#pragma once

#include "string.h"

#include "hardware.hpp"

#include "cameraDCMI.hpp"
#include "imageProcessor.hpp"
#include "imageWriter.hpp"
#include "jpegEncoder.hpp"
#include "logger.hpp"
#include "patternGenerator.hpp"
#include "storageThread.hpp"
#include "usbClassUVC.hpp"
#include "usbDevice.hpp"

#include "tx_api.h"
#include "fx_api.h"

class VisionThread {
	public:
		static void Init();

		// Array of all supported controls
		static USBClassUVC::UVCControlState puControlRegistry[];
		static const uint8_t numPUControls;

		static USBClassUVC::UVCControlState itControlRegistry[];
		static const uint8_t numITControls;
	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[4096];

		static void Run(ULONG input);

		// Static callbacks to wrap your I2C hardware calls
		static void BrightnessControl(int16_t value);
};