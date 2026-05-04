/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/JPEGEncoder.hpp
 * Author:  NotBlackMagic
 * Brief:   JPEG frame encoder.
 */

#pragma once

#include <stdint.h>

#include "hardware.hpp"

#include "dmaChannel.hpp"
#include "jpeg.hpp"
#include "logger.hpp"
#include "status.hpp"
#include "system.hpp"
#include "visionTypes.hpp"

#include "tx_api.h"

class JPEGEncoder {
	public:
		JPEGEncoder(Jpeg& jpegInterface, DMAChannel& dmaIn, DMAChannel& dmaOut) : jpegInstance(jpegInterface), dmaIn(dmaIn), dmaOut(dmaOut) {};

		Status Init();
		Status EncodeAsync(const VisionFrame& inputFrame, VisionFrame& outputFrame, uint32_t quality);
		Status EncodeWait(uint32_t timeoutTicks);
		Status EncodeAbort();

	private:
		Jpeg& jpegInstance;
		DMAChannel& dmaIn;
		DMAChannel& dmaOut;

		// Capture context
		VisionFrame* outFrameContext;
		const VisionFrame* inFrameContext;

		TX_EVENT_FLAGS_GROUP syncEvent;
		static constexpr uint32_t EVT_DONE = 0x01;
		static constexpr uint32_t EVT_ERR  = 0x02;

		// Static callbacks to pass into the DMA Config
		static void DMAInStateCallback(void* ctx, bool enable);
		static void DMAOutStateCallback(void* ctx, bool enable);

		static void EventCallback(void* ctx, Jpeg::Event evt);
};