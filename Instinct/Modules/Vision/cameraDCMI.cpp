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
	dcmiCfg.EventCallback = CameraDCMI::OnDCMIEvent;
	dcmiCfg.callbackContext = this;

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
	dmaConfig.dstWidth = DMAChannel::DataWidth::DoubleWord;
	dmaConfig.srcWidth = DMAChannel::DataWidth::Word;
	dmaConfig.incDst = true;
	dmaConfig.incSrc = false;
	dmaConfig.priority = DMAChannel::Priority::High;
	dmaConfig.srcBurstLength = 1;	// From DCMI, burst NOT supported
	dmaConfig.dstBurstLength = 2;	// To AXISRAM, 2x 64-bit (double word) = 16-Bytes AXI bursts
	dmaConfig.requestLine = LL_HPDMA1_REQUEST_DCMI_PSSI;
	dmaConfig.EventCallback = CameraDCMI::OnDMAComplete;
	dmaConfig.callbackContext = this;

	return this->dmaChannel.Init(dmaConfig);
}

Status CameraDCMI::CaptureAsync(VisionFrame &buffer) {
	// Empty pointer check
	if(buffer.startAddress == nullptr) {
		return Status::Error;
	}
	
	// Alignment check, for DMA cache stuff
	if(((uint32_t)(buffer.startAddress) & 0x1F) != 0) {
		return Status::Error;
	}

	// Save capture context
	this->currentMode = Mode::Snapshot;
	this->currentFrameIdx = 0;
	this->activeFrames[0] = &buffer;
	this->activeFrames[1] = nullptr;

	// Handle cache coherency
	// System::CleanCache((uint32_t*)buffer.startAddress, buffer.payloadSize);

	Status status = this->dmaChannel.TransferAsync((uint32_t)&DCMI->DR, (uint32_t)buffer.startAddress, buffer.payloadSize);
	if(status != Status::Ok) {
		this->currentMode = Mode::Idle;
		this->stats.errors = this->stats.errors + 1;
		return status;
	}

	// Enable interface
	return dcmiInterface.Start(Dcmi::CaptureMode::Snapshot);
}

Status CameraDCMI::CaptureAsync(VisionFrame &buffer0, VisionFrame &buffer1) {
	// Empty pointer check
	if(buffer0.startAddress == nullptr || buffer1.startAddress == nullptr) {
		return Status::Error;
	}

	// Alignment check, for DMA cache stuff
	if(((uint32_t)(buffer0.startAddress) & 0x1F) != 0 || ((uint32_t)(buffer1.startAddress) & 0x1F) != 0) {
		return Status::Error;
	}

	// Save capture context
	this->currentMode = Mode::Continuous;
	this->currentFrameIdx = 0;
	this->activeFrames[0] = &buffer0;
	this->activeFrames[1] = &buffer1;

	// Handle cache coherency
	System::CleanCache((uint32_t*)buffer0.startAddress, buffer0.payloadSize);
	System::CleanCache((uint32_t*)buffer1.startAddress, buffer1.payloadSize);

	// Prime the pipeline with the first buffer (uses 2D mode automatically on CH12-15)
	Status status = this->dmaChannel.TransferAsync((uint32_t)&DCMI->DR, (uint32_t)buffer0.startAddress, buffer0.payloadSize);
	if(status != Status::Ok) {
		this->currentMode = Mode::Idle;
		this->stats.errors = this->stats.errors + 1;
		return status;
	}

	// Start DCMI in continuous mode
	return this->dcmiInterface.Start(Dcmi::CaptureMode::Continuous);
}

Status CameraDCMI::CaptureWait(uint32_t timeoutTicks) {
	// Block on DMA transfer completion
	Status status = this->dmaChannel.TransferWait(timeoutTicks);
	if(status == Status::Ok) {
		// In Continuous mode, OnDMAComplete already flipped currentFrameIdx to the next buffer,
		// so the completed buffer is at (1 - currentFrameIdx).
		uint8_t completedIdx = (this->currentMode == Mode::Continuous) ? (1U - this->currentFrameIdx) : this->currentFrameIdx;

		VisionFrame* readyFrame = this->activeFrames[completedIdx];
		if(readyFrame != nullptr) {
			// Invalidate cache on the thread, not inside the DMA ISR
			System::InvalidateCache((uint32_t*)readyFrame->startAddress, readyFrame->payloadSize);
		}

		if(this->currentMode == Mode::Snapshot) {
			this->dcmiInterface.Stop();
			this->currentMode = Mode::Idle;
		}
	}
	else {
		this->stats.errors++;
		this->CaptureAbort();
	}

	return status;
}

Status CameraDCMI::CaptureAbort() {
	// Stop the DCMI peripheral from generating requests
	dcmiInterface.Stop();

	// Abort the ongoing DMA transfer
	this->dmaChannel.TransferStop();

	if(this->currentMode != Mode::Idle) {
		this->stats.drops++;
	}

	// Clear capture context
	this->currentMode = Mode::Idle;
	this->activeFrames[0] = nullptr;
	this->activeFrames[1] = nullptr;

	return Status::Ok;
}

void CameraDCMI::OnDCMIEvent(void* ctx, Dcmi::Event evt) {
	if(ctx == nullptr) {
		return;
	}

	CameraDCMI* self = static_cast<CameraDCMI*>(ctx);
	if(self->currentMode == Mode::Idle) {
		return;
	}

	if(evt == Dcmi::Event::OverrunError || evt == Dcmi::Event::SyncError) {
		// Stop DCMI from generating further requests, rest should be handled in application
		self->dcmiInterface.Stop();
		// Abort DMA too so CaptureWait() wakes up immediately instead of hanging until timeout
		self->dmaChannel.TransferStop();

		if(self->config.EventCallback != nullptr) {
			self->config.EventCallback(self->config.callbackContext, Event::Error);
		}
	}
}

void CameraDCMI::OnDMAComplete(void* ctx, DMAChannel::Event evt) {
	if(ctx == nullptr ) {
		return;
	}

	CameraDCMI* self = static_cast<CameraDCMI*>(ctx);
	if(self->currentMode == Mode::Idle) {
		return;
	}

	if(evt == DMAChannel::Event::TransferComplete) {
		VisionFrame* completedFrame = self->activeFrames[self->currentFrameIdx];

		if(self->currentMode == Mode::Continuous) {
			// Determine next buffer index
			uint8_t nextIdx = 1 - self->currentFrameIdx;
			VisionFrame* nextFrame = self->activeFrames[nextIdx];

			// Re-arm DMA immediately during the VBLANK period
			self->dmaChannel.TransferAsync((uint32_t)&DCMI->DR, (uint32_t)nextFrame->startAddress, nextFrame->payloadSize);

			// Advance ping-pong index
			self->currentFrameIdx = nextIdx;
		}
		else {
			self->currentMode = Mode::Idle;
			// self->dcmiInterface.Stop();
		}

		if(completedFrame != nullptr) {
			// Handle cache coherency
			// System::InvalidateCache((uint32_t*)completedFrame->startAddress, completedFrame->payloadSize);

			// Frame timestamp
			completedFrame->timestampUs = Time::GetUs();
			self->stats.frames = self->stats.frames + 1;

			if(self->timestamp != 0) {
				uint32_t dt = completedFrame->timestampUs - self->timestamp;
				if(dt > 0) {
					float instantFps = 1000000.0f / static_cast<float>(dt);
					self->stats.fps = (self->stats.fps == 0.0f) ? instantFps : (0.8f * self->stats.fps + 0.2f * instantFps);
				}
			}
			self->timestamp = completedFrame->timestampUs;
		}

		if(self->config.EventCallback != nullptr) {
			self->config.EventCallback(self->config.callbackContext, Event::FrameComplete);
		}
	}
	else {
		self->stats.errors = self->stats.errors + 1;
		self->currentMode = Mode::Idle;
		self->dcmiInterface.Stop();

		if(self->config.EventCallback != nullptr) {
			self->config.EventCallback(self->config.callbackContext, Event::Error);
		}
	}
}