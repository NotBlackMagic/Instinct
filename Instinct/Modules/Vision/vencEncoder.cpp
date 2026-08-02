/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/vencEncoder.cpp
 */

#include "vencEncoder.hpp"

Status VENCEncoder::Init(Venc::Config& config) {
	if(tx_event_flags_create(&this->syncEvent, const_cast<char*>("VencSync")) != TX_SUCCESS) {
		return Status::Error;
	}

	// Cache the active format for payload tagging
	this->codec = (config.codec == Venc::Codec::H264) ? VisionCodec::H264 : VisionCodec::Jpeg;

	// Configure VENC Event bindings
	config.EventCallback = VENCEncoder::EventCallback;
	config.callbackContext = this;

	// Initialize VENC Hardware
	return this->vencInstance.Init(config);
}

Status VENCEncoder::EncodeStart(VisionFrame& outputFrame) {
	// Cache alignment checks
	if(((uint32_t)(outputFrame.startAddress) & 0x1F) != 0) {
		return Status::Error;
	}

	uint32_t generatedBytes = 0;
	Status status = this->vencInstance.Start((uint32_t*)outputFrame.startAddress, outputFrame.allocatedSize, generatedBytes);
	if(status == Status::Ok) {
		outputFrame.payloadSize = generatedBytes;
		outputFrame.codec = this->codec;
		uint32_t alignedSize = (generatedBytes + 31U) & 0xFFFFFFE0;
		System::InvalidateCache((uint32_t*)outputFrame.startAddress, alignedSize);
	}

	return status;
}

Status VENCEncoder::EncodeAsync(const VisionFrame& inputFrame, VisionFrame& outputFrame, bool requestKeyframe) {
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

	// Setup input/output buffering. VENC features built-in AXI bus mastering (internal DMA)
	Venc::FrameBuffer params = {};
	params.busLuma = (const uint32_t*)inputFrame.startAddress;
	params.busChromaU = params.busLuma; 
	params.busChromaV = params.busLuma;
	params.outBuffer = (uint32_t*)outputFrame.startAddress;
	params.outBufSize = outputFrame.allocatedSize;
	params.frameType = requestKeyframe ? Venc::FrameType::Intra : Venc::FrameType::Predicted;

	uint32_t generatedBytes = 0;
	Venc::FrameType outType;

	// Calling thread yields to RTOS here while hardware encodes
	Status status = this->vencInstance.EncodeFrame(params, generatedBytes, outType);

	if(status == Status::Ok) {
		this->outFrameContext->payloadSize = generatedBytes;
		this->outFrameContext->codec = this->codec;
		this->outFrameContext->isKeyframe = (outType == Venc::FrameType::Intra);
		tx_event_flags_set(&this->syncEvent, EVT_DONE, TX_OR);
	} 
	else {
		tx_event_flags_set(&this->syncEvent, EVT_ERR, TX_OR);
	}

	return status;
}

Status VENCEncoder::EncodeWait(uint32_t timeoutTicks) {
	if(this->outFrameContext == nullptr) {
		return Status::Error;
	}

	ULONG actualEvents = 0;
	uint32_t waitStatus = tx_event_flags_get(&this->syncEvent, EVT_DONE | EVT_ERR, TX_OR_CLEAR, &actualEvents, timeoutTicks);

	Status status = Status::Ok;
	if(waitStatus == TX_SUCCESS && (actualEvents & EVT_DONE)) {
		// Handle cache coherency
		uint32_t alignedSize = (this->outFrameContext->payloadSize + 31U) & 0xFFFFFFE0;
		System::InvalidateCache((uint32_t*)this->outFrameContext->startAddress, alignedSize);
	}
	else {
		status = Status::Timeout;
		this->EncodeAbort();
	}

	this->outFrameContext = nullptr;
	this->inFrameContext = nullptr;
	return status;
}

Status VENCEncoder::EncodeStop(VisionFrame& outputFrame) {
	if(((uint32_t)(outputFrame.startAddress) & 0x1F) != 0) {
		return Status::Error;
	}

	uint32_t generatedBytes = 0;
	Status status = this->vencInstance.Stop((uint32_t*)outputFrame.startAddress, outputFrame.allocatedSize, generatedBytes);
	if(status == Status::Ok) {
		outputFrame.payloadSize = generatedBytes;
		outputFrame.codec = this->codec;
		uint32_t alignedSize = (this->outFrameContext->payloadSize + 31U) & 0xFFFFFFE0;
		System::InvalidateCache((uint32_t*)outputFrame.startAddress, alignedSize);
	}

	return status;
}

Status VENCEncoder::EncodeAbort() {
	this->vencInstance.Reset();
	this->outFrameContext = nullptr;
	this->inFrameContext = nullptr;
	return Status::Ok;
}

void VENCEncoder::EventCallback(void* ctx, Venc::Event evt) {
	VENCEncoder* encoder = static_cast<VENCEncoder*>(ctx);
	if(evt == Venc::Event::FrameReady) {
		tx_event_flags_set(&encoder->syncEvent, EVT_DONE, TX_OR);
	}
	else {
		tx_event_flags_set(&encoder->syncEvent, EVT_ERR, TX_OR);
	}
}