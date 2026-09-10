/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Vision/cameraMIPI.cpp
 */

#include "cameraMIPI.hpp"

Status CameraMIPI::Init(const Config &config) {
	Status status;

	// Powerup sequence
	csiPwdn.Write(1);
	csiRst.Write(0);
	tx_thread_sleep(10);
	csiRst.Write(1);
	tx_thread_sleep(20);

	// Configure the Sensor (OV5645)
	OV5645::Config ov5645Cnfg;
	ov5645Cnfg.width = config.width;
	ov5645Cnfg.height = config.height;
	ov5645Cnfg.fps = config.fps;
	ov5645Cnfg.resetPin = nullptr;
	ov5645Cnfg.powerDownPin = nullptr;

	// MIPI CSI-2 requires that 8-bit YUV422 to be transmitted as UYVY. Overwrite all other formats to this and let the DCMIPP hardware handle translations
	if(config.format == PixelFormat::YUV422_YUYV || config.format == PixelFormat::YUV422_YVYU || config.format == PixelFormat::YUV422_UYVY) {
		ov5645Cnfg.format = PixelFormat::YUV422_UYVY;
	}
	else {
		ov5645Cnfg.format = config.format;
	}

	// Initialize Sensor (OV5645)
	status = this->sensor.Init(ov5645Cnfg);
	if (status != Status::Ok) {
		return status;
	}

	// Configure the Interface (CSI)
	Csi::Config csiCfg;
	csiCfg.bitrate = this->sensor.GetMIPIBitrate();
	csiCfg.laneMapping = Csi::LaneMapping::Direct;
	csiCfg.lanes = Csi::LaneCount::Two;

	// Initialize Interface Hardware (MIPI-CSI)
	status = this->csiInterface.Init(csiCfg);
	if (status != Status::Ok) {
		return status;
	}

	// Configure the Interface (DCMIPP)
	Dcmipp::PipeConfig dcmiPipe;
	dcmiPipe.frameRate = config.fps;

	// Find correct format
	Csi::MIPIDataType mipiDataType;
	switch (config.format) {
		case PixelFormat::RGB565:
			dcmiPipe.format = Dcmipp::OutputFormat::RGB565;
			dcmiPipe.swapRBUV = false;
			dcmiPipe.pixelPitch = config.width * 2;
			mipiDataType = Csi::MIPIDataType::RGB565;
			break;
		case PixelFormat::YUV422_YUYV:
			dcmiPipe.format = Dcmipp::OutputFormat::YUV422_YUYV;
			dcmiPipe.swapRBUV = false;
			dcmiPipe.pixelPitch = config.width * 2;
			mipiDataType = Csi::MIPIDataType::YUV422_8bit;
			break;
		case PixelFormat::YUV422_YVYU:
			dcmiPipe.format = Dcmipp::OutputFormat::YUV422_YUYV;
			dcmiPipe.swapRBUV = true;
			dcmiPipe.pixelPitch = config.width * 2;
			mipiDataType = Csi::MIPIDataType::YUV422_8bit;
			break;
		case PixelFormat::YUV422_UYVY:
			dcmiPipe.format = Dcmipp::OutputFormat::YUV422_UYVY;
			dcmiPipe.swapRBUV = false;
			dcmiPipe.pixelPitch = config.width * 2;
			mipiDataType = Csi::MIPIDataType::YUV422_8bit;
			break;
		case PixelFormat::YUV420_NV12:
			dcmiPipe.format = Dcmipp::OutputFormat::YUV420_NV21;
			dcmiPipe.swapRBUV = true;
			dcmiPipe.pixelPitch = config.width;
			mipiDataType = Csi::MIPIDataType::YUV422_8bit;
			break;
		default:
			return Status::Error; // Unsupported pipeline format
	}

	// Initialize Interface Hardware (DCMIPP)
	status = this->dcmiPPInterface.Init();
	if (status != Status::Ok) {
		return status;
	}

	status = this->csiInterface.ConfigureVirtualChannel(Csi::VirtualChannel::VC0, mipiDataType);
	if (status != Status::Ok) {
		return status;
	}

	status = this->dcmiPPInterface.LinkCSIVirtualChannel(Dcmipp::PipeID::Main, Csi::VirtualChannel::VC0, mipiDataType);
	if (status != Status::Ok) {
		return status;
	}

	status = this->dcmiPPInterface.ConfigurePipe(Dcmipp::PipeID::Main, dcmiPipe);
	if (status != Status::Ok) {
		return status;
	}

	return Status::Ok;
}

Status CameraMIPI::CaptureAsync(VisionFrame &buffer) {
	// Alignment check, for DMA cache stuff
	if(((uint32_t)(buffer.startAddress) & 0x1F) != 0) {
		return Status::Error;
	}

	// Save capture context
	this->currentMode = Mode::Snapshot;
	this->currentFrameIdx = 0;
	this->activeFrames[0] = &buffer;
	this->activeFrames[1] = nullptr;

	// Ensure line wrapping is disabled for full-frame snapshot
	this->dcmiPPInterface.DisableLineWrapping(Dcmipp::PipeID::Main);

	// Handle cache coherency
	System::CleanCache((uint32_t*)buffer.startAddress, buffer.payloadSize);

	// Configure DCMIPP Memory Destination
	Dcmipp::MemoryDestination dest;
	dest.primaryAddress = (uint32_t)buffer.startAddress;
	dest.isDoubleBuffered = false;
	dest.isFullPlanar = false;

	if(buffer.format == PixelFormat::YUV420_NV12) {
		dest.isSemiPlanar = true;
		uint32_t lumaSize = buffer.width * buffer.height;
		dest.uAddress = dest.primaryAddress + lumaSize;
	}
	else {
		dest.isSemiPlanar = false;
		dest.uAddress = 0;
	}

	// Enable interface (Receiver side first)
	Status status = this->dcmiPPInterface.CaptureAsync(Dcmipp::PipeID::Main, dest, Dcmipp::CaptureMode::Snapshot);
	if(status != Status::Ok) {
		return status;
	}

	status = this->csiInterface.Start(Csi::VirtualChannel::VC0);
	if(status != Status::Ok) {
		return status;
	}

	// Enable sensor (Transmitter side last)
	return this->sensor.Start();
}

Status CameraMIPI::CaptureAsync(VisionFrame &buffer0, VisionFrame &buffer1) {
	// Alignment check, for DMA cache stuff
	if(((uint32_t)(buffer0.startAddress) & 0x1F) != 0 || ((uint32_t)(buffer1.startAddress) & 0x1F) != 0) {
		return Status::Error;
	}

	// Save capture context
	this->currentMode = Mode::Continuous;
	this->currentFrameIdx = 0;
	this->activeFrames[0] = &buffer0;
	this->activeFrames[1] = &buffer1;

	// Ensure line wrapping is disabled for full-frame continuous mode
	this->dcmiPPInterface.DisableLineWrapping(Dcmipp::PipeID::Main);

	// Handle cache coherency
	System::CleanCache((uint32_t*)buffer0.startAddress, buffer0.payloadSize);
	System::CleanCache((uint32_t*)buffer1.startAddress, buffer1.payloadSize);

	// Configure DCMIPP Memory Destination
	Dcmipp::MemoryDestination dest;
	dest.primaryAddress = (uint32_t)buffer0.startAddress;
	dest.secondaryAddress = (uint32_t)buffer1.startAddress;
	dest.isDoubleBuffered = true;
	dest.isFullPlanar = false;

	if(buffer0.format == PixelFormat::YUV420_NV12) {
		dest.isSemiPlanar = true;
		dest.uAddress = dest.primaryAddress + (buffer0.width * buffer0.height);
	}
	else {
		dest.isSemiPlanar = false;
		dest.uAddress = 0;
	}

	// Enable interface (Receiver side first)
	Status status = this->dcmiPPInterface.CaptureAsync(Dcmipp::PipeID::Main, dest, Dcmipp::CaptureMode::Continuous);
	if(status != Status::Ok) {
		return status;
	}

	status = this->csiInterface.Start(Csi::VirtualChannel::VC0);
	if(status != Status::Ok) {
		return status;
	}

	// Enable sensor (Transmitter side last)
	return this->sensor.Start();
}

Status CameraMIPI::CaptureAsync(VisionFrame& buffer, Dcmipp::LineCount lineMult, Dcmipp::LineCount wrapAddress) {
	// Alignment check, for DMA cache stuff
	if(((uint32_t)(buffer.startAddress) & 0x1F) != 0) {
		return Status::Error;
	}

	// Save capture context
	this->currentMode = Mode::Partial;
	this->currentFrameIdx = 0;
	this->activeFrames[0] = &buffer;
	this->activeFrames[1] = nullptr;

	// Enable hardware line wrapping
	this->dcmiPPInterface.EnableLineWrapping(Dcmipp::PipeID::Main, wrapAddress, lineMult);

	// Configure DCMIPP Memory Destination
	Dcmipp::MemoryDestination dest;
	dest.primaryAddress = (uint32_t)buffer.startAddress;
	dest.isDoubleBuffered = false;
	dest.isFullPlanar = false;

	if(buffer.format == PixelFormat::YUV420_NV12) {
		dest.isSemiPlanar = true;
		uint32_t wrapLines = 1U << static_cast<uint32_t>(wrapAddress);
		uint32_t lumaSize = buffer.width * wrapLines;
		dest.uAddress = dest.primaryAddress + lumaSize;
	}
	else {
		dest.isSemiPlanar = false;
		dest.uAddress = 0;
	}

	Status status = this->dcmiPPInterface.CaptureAsync(Dcmipp::PipeID::Main, dest, Dcmipp::CaptureMode::Continuous);
	if(status != Status::Ok) {
		return status;
	}

	status = this->csiInterface.Start(Csi::VirtualChannel::VC0);
	if(status != Status::Ok) {
		return status;
	}

	return this->sensor.Start();
}

Status CameraMIPI::CaptureWait(uint32_t timeoutTicks) {
	if(this->currentMode == Mode::Idle) {
		return Status::Error;
	}

	Status status = this->dcmiPPInterface.CaptureWait(Dcmipp::PipeID::Main, timeoutTicks);
	if(status == Status::Ok) {
		// Automatically invalidate the cache for whichever buffer the hardware just completed
		if(this->currentMode != Mode::Partial) {
			VisionFrame* readyFrame = this->activeFrames[this->currentFrameIdx];
			System::InvalidateCache((uint32_t*)readyFrame->startAddress, readyFrame->payloadSize);
		}

		// State Machine Management
		if(this->currentMode == Mode::Continuous) {
			// Ping-pong the index for the next call
			this->currentFrameIdx = 1 - this->currentFrameIdx;
		}
		else if(this->currentMode == Mode::Partial) {
			// Line mode runs continuously into the single circular buffer
			this->currentFrameIdx = 0;
		}
		else {
			// Snapshots and Partials complete after one wait
			this->currentMode = Mode::Idle;
		}
	}
	else {
		// Disable Interface
		this->CaptureAbort();
	}

	return status;
}

Status CameraMIPI::CaptureAbort() {
	// Stop the sensor PHY first
	this->sensor.Stop();

	// Stop receivers
	this->csiInterface.Stop(Csi::VirtualChannel::VC0);
	this->dcmiPPInterface.CaptureAbort(Dcmipp::PipeID::Main);
	
	// Clear capture context
	this->currentMode = Mode::Idle;
	this->activeFrames[0] = nullptr;
	this->activeFrames[1] = nullptr;

	return Status::Ok;
}