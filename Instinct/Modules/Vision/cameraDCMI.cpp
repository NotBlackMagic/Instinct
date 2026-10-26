/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Vision/cameraDCMI.cpp
 */

#include "cameraDCMI.hpp"

Status CameraDCMI::Init(const Config &config) {
	Status status;
	this->config = config;

	// Powerup sequence
	camPwdn.Write(1);
	tx_thread_sleep(10);

	// Configure the Sensor (OV7670 or OV9281)
	OV9281::Config sensorCnfg{};
	sensorCnfg.width = config.width;
	sensorCnfg.height = config.height;
	sensorCnfg.format = config.format;
	sensorCnfg.fps = config.fps;
	sensorCnfg.triggerMode = OV9281::TriggerMode::Continuous;
	sensorCnfg.shutdownPin = nullptr;

	// Initialize Sensor (OV7670 or OV9281)
	status = this->sensor.Init(sensorCnfg);
	if(status != Status::Ok) {
		return status;
	}

	// Configure the Interface (DCMI)
	Dcmi::Config dcmiCfg{};
	dcmiCfg.vSyncPolarity = Dcmi::SyncPolarity::High;
	dcmiCfg.hSyncPolarity = Dcmi::SyncPolarity::Low;
	dcmiCfg.pxClkPolarity = Dcmi::ClockPolarity::Rising;
	dcmiCfg.embeddedSync = false;

	// Initialize the Interface Hardware
	status = this->dcmiInterface.Init(dcmiCfg);
	if(status != Status::Ok) {
		return status;
	}
	// Set hardware cropping, even if no "real" cropping used to prevent any HSYNC glitch from shifting rows
	this->dcmiInterface.ConfigureCrop(0, 0, config.width, config.height);

	// Initialize the DMA Channel
	DMAChannel::Config dmaConfig{};
	dmaConfig.direction = DMAChannel::Direction::PeripheralToMemory;
	dmaConfig.dstWidth = DMAChannel::DataWidth::Word;
	dmaConfig.srcWidth = DMAChannel::DataWidth::Word;
	dmaConfig.incDst = true;
	dmaConfig.incSrc = false;
	dmaConfig.priority = DMAChannel::Priority::High;
	dmaConfig.requestLine = LL_HPDMA1_REQUEST_DCMI_PSSI;
	dmaConfig.CompleteCallback = CameraDCMI::OnDMAComplete;
	dmaConfig.callbackContext = this;

	return this->dmaChannel.Init(dmaConfig);
}

Status CameraDCMI::CaptureAsync(VisionFrame &buffer) {
	if(this->frame != nullptr) {
		this->stats.drops = this->stats.drops + 1;
		return Status::Error;
	}
	
	// Alignment check, for DMA cache stuff
	if(((uint32_t)(buffer.startAddress) & 0x1F) != 0) {
		return Status::Error;
	}

	// Save capture context
	frame = &buffer;

	// Handle cache coherency
	System::CleanCache((uint32_t*)buffer.startAddress, buffer.payloadSize);

	Status status = this->dmaChannel.TransferAsync((uint32_t)&DCMI->DR, (uint32_t)buffer.startAddress, buffer.payloadSize);
	if(status != Status::Ok) {
		this->frame = nullptr;
		this->stats.dmaErrors = this->stats.dmaErrors + 1;
		return status;
	}

	// Enable interface
	return dcmiInterface.Start(Dcmi::CaptureMode::Snapshot);
}

Status CameraDCMI::CaptureWait(uint32_t timeoutTicks) {
	if(this->frame == nullptr) {
		return Status::Error;
	}

	Status status = this->dmaChannel.TransferWait(timeoutTicks);
	if(status == Status::Ok) {
		// Handle cache coherency
		System::InvalidateCache((uint32_t*)this->frame->startAddress, this->frame->payloadSize);
	}
	else {
		// Disable Interface
		dcmiInterface.Stop();
	}

	// Clear capture context
	this->frame = nullptr;

	return status;
}

Status CameraDCMI::CaptureAbort() {
	// Stop the DCMI peripheral from generating requests
	dcmiInterface.Stop();

	// Abort the ongoing DMA transfer
	this->dmaChannel.TransferStop();

	if(this->frame != nullptr) {
		this->stats.drops = this->stats.drops + 1;
	}

	// Clear capture context
	this->frame = nullptr;
	
	return Status::Ok;
}

void CameraDCMI::OnDMAComplete(void* ctx, Status status) {
	if(ctx == nullptr) {
		return;
	}

	CameraDCMI* self = static_cast<CameraDCMI*>(ctx);
	if(self->frame == nullptr) {
		return;
	}

	VisionFrame* completedFrame = self->frame;
	self->frame = nullptr;

	// Stop DCMI CAPTURE & ENABLE are 0 before the trailing VSYNC arrives
	self->dcmiInterface.Stop();

	if(status == Status::Ok) {
		// Handle cache coherency
		System::InvalidateCache((uint32_t*)completedFrame->startAddress, completedFrame->payloadSize);

		// Frame timestamp
		completedFrame->timestampUs = Time::GetUs();
		self->stats.frames = self->stats.frames + 1;

		if(self->frameTimestamp != 0) {
			uint32_t dt = completedFrame->timestampUs - self->frameTimestamp;
			if(dt > 0) {
				float instantFps = 1000000.0f / static_cast<float>(dt);
				self->stats.fps = (self->stats.fps == 0.0f) ? instantFps : (0.8f * self->stats.fps + 0.2f * instantFps);
			}
		}
		self->frameTimestamp = completedFrame->timestampUs;

		if(self->config.EventCallback != nullptr) {
			self->config.EventCallback(self->config.callbackContext, Event::FrameComplete);
		}
	}
	else {
		self->stats.dmaErrors = self->stats.dmaErrors + 1;

		// Harvest any pending DCMI status flags before disabling the peripheral
		self->dcmiInterface.InterruptHandler();
		self->dcmiInterface.Stop();

		if(self->config.EventCallback != nullptr) {
			self->config.EventCallback(self->config.callbackContext, Event::Error);
		}
	}
}