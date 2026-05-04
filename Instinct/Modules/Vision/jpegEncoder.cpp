/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/JPEGEncoder.cpp
 */

 // NOTE ON DMA TRANSFERS: https://community.st.com/t5/stm32-mcus-embedded-software/struggle-using-gpdma-linkedlist-on-jpeg-gpdma-input-ll-stalls/td-p/842221

#include "JPEGEncoder.hpp"

Status JPEGEncoder::Init() {
	if(tx_event_flags_create(&this->syncEvent, const_cast<char*>("JpegSync")) != TX_SUCCESS) {
		return Status::Error;
	}

	// Configure JPEG
	Jpeg::Config config;
	config.EventCallback = JPEGEncoder::EventCallback;
	config.callbackContext = this;

	// Initialize JPEG Hardware
	Status status = this->jpegInstance.Init(config);
	if(status != Status::Ok) {
		return status;
	}

	// Initialize the IN DMA Channel (Memory to Peripheral)
	DMAChannel::Config dmaInCfg;
	dmaInCfg.direction = DMAChannel::Direction::MemoryToPeripheral;
	dmaInCfg.dstWidth = DMAChannel::DataWidth::Word;
	dmaInCfg.srcWidth = DMAChannel::DataWidth::Word;
	dmaInCfg.incDst = false;
	dmaInCfg.incSrc = true;
	dmaInCfg.priority = DMAChannel::Priority::High;
	dmaInCfg.requestLine = LL_HPDMA1_REQUEST_JPEG_RX;
	dmaInCfg.srcBurstLength = 8;
	dmaInCfg.dstBurstLength = 8;
	dmaInCfg.StateCallback = JPEGEncoder::DMAInStateCallback;
	dmaInCfg.callbackContext = this;

	status = this->dmaIn.Init(dmaInCfg);
	if(status != Status::Ok) {
		return status;
	}

	// Initialize the OUT DMA Channel (Peripheral to Memory)
	DMAChannel::Config dmaOutCfg;
	dmaOutCfg.direction = DMAChannel::Direction::PeripheralToMemory;
	dmaOutCfg.dstWidth = DMAChannel::DataWidth::Word;
	dmaOutCfg.srcWidth = DMAChannel::DataWidth::Word;
	dmaOutCfg.incDst = true;
	dmaOutCfg.incSrc = false;
	dmaOutCfg.priority = DMAChannel::Priority::High;
	dmaOutCfg.requestLine = LL_HPDMA1_REQUEST_JPEG_TX;
	dmaOutCfg.srcBurstLength = 8;
	dmaOutCfg.dstBurstLength = 8;
	// On second try/test, this is not required: Disable DMA while reloading buffer
	dmaOutCfg.StateCallback = nullptr;		// JPEGEncoder::DMAOutStateCallback;
	dmaOutCfg.callbackContext = nullptr;	// this;

	return this->dmaOut.Init(dmaOutCfg);
}

Status JPEGEncoder::EncodeAsync(const VisionFrame& inputFrame, VisionFrame& outputFrame, uint32_t quality) {
	// Cache alignment checks
	if(((uint32_t)(inputFrame.startAddress) & 0x1F) != 0 || ((uint32_t)(outputFrame.startAddress) & 0x1F) != 0) {
		return Status::Error;
	}

	// Save capture context
	this->outFrameContext = &outputFrame;
	this->inFrameContext = &inputFrame;
	tx_event_flags_set(&this->syncEvent, 0, TX_AND); // Clear events

	// Handle cache coherency
	System::CleanCache((uint32_t*)inputFrame.startAddress, inputFrame.payloadSize);
	System::InvalidateCache((uint32_t*)outputFrame.startAddress, outputFrame.allocatedSize);

	// Map PixelFormat to Jpeg Format
	Jpeg::ImageParams params;
	params.width = inputFrame.width;
	params.height = inputFrame.height;
	params.quality = quality;

	if(inputFrame.format == PixelFormat::YUV422_YUYV || inputFrame.format == PixelFormat::YUV422_YVYU || inputFrame.format == PixelFormat::YUV422_UYVY) {
		params.colorSpace = Jpeg::ColorSpace::YCbCr;
		params.subsampling = Jpeg::Subsampling::YUV422;
	}
	else if(inputFrame.format == PixelFormat::Grayscale) {
		params.colorSpace = Jpeg::ColorSpace::Grayscale;
		params.subsampling = Jpeg::Subsampling::None;
	}
	else {
		return Status::Error;
	}

	// Start OUT DMA (Providing the max allocated size)
	Status status = this->dmaOut.Transfer((uint32_t)&JPEG->DOR, (uint32_t)outputFrame.startAddress, outputFrame.allocatedSize);
	if(status != Status::Ok) {
		return status;
	}

	// Start IN DMA
	status = this->dmaIn.Transfer((uint32_t)inputFrame.startAddress, (uint32_t)&JPEG->DIR, inputFrame.payloadSize);
	if(status != Status::Ok) {
		this->dmaOut.TransferStop();
		return status;
	}

	// Start Hardware
	return this->jpegInstance.Start(params);
}

Status JPEGEncoder::EncodeWait(uint32_t timeoutTicks) {
	if(this->outFrameContext == nullptr) {
		return Status::Error;
	}

	// Wait for IN DMA to finish feeding all bytes
	Status status = this->dmaIn.TransferWait(timeoutTicks);
	if(status != Status::Ok) {
		this->EncodeAbort();
		return status;
	}

	// Wait for the JPEG core to generate the EOI marker and raise EOCF
	ULONG actualEvents = 0;
	uint32_t waitStatus = tx_event_flags_get(&this->syncEvent, EVT_DONE | EVT_ERR, TX_OR_CLEAR, &actualEvents, timeoutTicks);

	// Stop hardware and DMAs regardless of outcome
	this->jpegInstance.Stop();
	this->dmaIn.TransferStop();
	this->dmaOut.TransferStop();

	status = Status::Ok;
	if(waitStatus == TX_SUCCESS && (actualEvents & EVT_DONE)) {
		// Compute actual payload size.
		uint32_t remainingBytes = this->dmaOut.GetRemaining();
		this->outFrameContext->payloadSize = this->outFrameContext->allocatedSize - remainingBytes;

		System::InvalidateCache((uint32_t*)this->outFrameContext->startAddress, this->outFrameContext->allocatedSize);

		// Read remaining bytes from JPEG OUT FIFO, bytes below the DMA trigger threshold.
		while(READ_BIT(JPEG->SR, JPEG_SR_OFNEF) == JPEG_SR_OFNEF) {
			*((uint32_t*)(this->outFrameContext->startAddress + this->outFrameContext->payloadSize)) = JPEG->DOR;
			this->outFrameContext->payloadSize += 4;
		}
		
		this->outFrameContext->codec = VisionCodec::Jpeg;
	
		// Handle cache coherency
		System::CleanCache((uint32_t*)this->outFrameContext->startAddress, this->outFrameContext->payloadSize);
	}
	else {
		status = Status::Timeout;
	}

	this->outFrameContext = nullptr;
	return status;
}

Status JPEGEncoder::EncodeAbort() {
	this->jpegInstance.Stop();
	this->dmaIn.TransferStop();
	this->dmaOut.TransferStop();
	this->outFrameContext = nullptr;
	return Status::Ok;
}

void JPEGEncoder::DMAInStateCallback(void* ctx, bool enable) {
	JPEGEncoder* encoder = static_cast<JPEGEncoder*>(ctx);
	encoder->jpegInstance.EnableRxDMA(enable);
}

void JPEGEncoder::DMAOutStateCallback(void* ctx, bool enable) {
	JPEGEncoder* encoder = static_cast<JPEGEncoder*>(ctx);
	encoder->jpegInstance.EnableTxDMA(enable);
}

void JPEGEncoder::EventCallback(void* ctx, Jpeg::Event evt) {
	JPEGEncoder* encoder = static_cast<JPEGEncoder*>(ctx);
	if(evt == Jpeg::Event::EncodeComplete) {
		tx_event_flags_set(&encoder->syncEvent, EVT_DONE, TX_OR);
	}
	else {
		tx_event_flags_set(&encoder->syncEvent, EVT_ERR, TX_OR);
	}
}