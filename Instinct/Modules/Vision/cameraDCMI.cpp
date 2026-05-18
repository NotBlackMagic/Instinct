/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/cameraDCMI.cpp
 */

#include "cameraDCMI.hpp"

Status CameraDCMI::Init(const Config &config) {
	Status status;

	// Powerup sequence
	camPwdn.Write(1);
	tx_thread_sleep(10);

	// Configure the Sensor (OV7670)
	OV7670::Config ov7670Cnfg;
	ov7670Cnfg.width = config.width;
	ov7670Cnfg.height = config.height;
	ov7670Cnfg.format = config.format;
	ov7670Cnfg.fps = config.fps;
	ov7670Cnfg.resetPin = nullptr;
	ov7670Cnfg.powerDownPin = nullptr;

	// Initialize Sensor (OV7670)
	status = this->sensor.Init(ov7670Cnfg);
	if (status != Status::Ok) {
		return status;
	}

	// Configure the Interface (DCMI)
	Dcmi::Config dcmiCfg;
	dcmiCfg.vSyncPolarity = Dcmi::SyncPolarity::High;
	dcmiCfg.hSyncPolarity = Dcmi::SyncPolarity::Low;
	dcmiCfg.pxClkPolarity = Dcmi::ClockPolarity::Rising;
	dcmiCfg.embeddedSync = false;

	// Initialize the Interface Hardware
	status = this->dcmiInterface.Init(dcmiCfg);
	if (status != Status::Ok) {
		return status;
	}

	// Initialize the DMA Channel
	DMAChannel::Config dmaConfig;
	dmaConfig.direction = DMAChannel::Direction::PeripheralToMemory;
	dmaConfig.dstWidth = DMAChannel::DataWidth::Word;
	dmaConfig.srcWidth = DMAChannel::DataWidth::Word;
	dmaConfig.incDst = true;
	dmaConfig.incSrc = false;
	dmaConfig.priority = DMAChannel::Priority::High;
	dmaConfig.requestLine = LL_HPDMA1_REQUEST_DCMI_PSSI;

	return this->dmaChannel.Init(dmaConfig);
}

Status CameraDCMI::CaptureAsync(VisionFrame &buffer) {
	// Alignment check, for DMA cache stuff
	if(((uint32_t)(buffer.startAddress) & 0x1F) != 0) {
		return Status::Error;
	}

	// Save capture context
	frame = &buffer;

	// Handle cache coherency
	System::CleanCache((uint32_t*)buffer.startAddress, buffer.payloadSize);

	Status status = this->dmaChannel.Transfer((uint32_t)&DCMI->DR, (uint32_t)buffer.startAddress, buffer.payloadSize);
	if(status != Status::Ok) {
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
	if (status == Status::Ok) {
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

	// Clear capture context
	this->frame = nullptr;
	
	return Status::Ok;
}