/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/cameraMIPI.cpp
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
	ov5645Cnfg.format = config.format;
	ov5645Cnfg.fps = config.fps;
	ov5645Cnfg.resetPin = nullptr;
	ov5645Cnfg.powerDownPin = nullptr;

	// Initialize Sensor (OV5645)
	status = this->sensor.Init(ov5645Cnfg);
	if (status != Status::Ok) {
		return status;
	}

	// Configure the Interface (CSI)
	Csi::Config csiCfg;
	csiCfg.bitrate = 448 * 1000000;
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
	dcmiPipe.pixelPitch = config.width * 2;

	// Find correct format
	Csi::MIPIDataType mipiDataType;
	switch (config.format) {
		case PixelFormat::RGB565:
			dcmiPipe.format = Dcmipp::OutputFormat::RGB565;
			dcmiPipe.swapRBUV = false;
			mipiDataType = Csi::MIPIDataType::RGB565;
			break;
		case PixelFormat::YUV422_YUYV:
			dcmiPipe.format = Dcmipp::OutputFormat::YUV422_YUYV;
			dcmiPipe.swapRBUV = false;
			mipiDataType = Csi::MIPIDataType::YUV422_8bit;
			break;
		case PixelFormat::YUV422_YVYU:
			dcmiPipe.format = Dcmipp::OutputFormat::YUV422_YUYV;
			dcmiPipe.swapRBUV = true;
			mipiDataType = Csi::MIPIDataType::YUV422_8bit;
			break;
		case PixelFormat::YUV422_UYVY:
			dcmiPipe.format = Dcmipp::OutputFormat::YUV422_UYVY;
			dcmiPipe.swapRBUV = false;
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
	frame = &buffer;

	// Handle cache coherency
	System::CleanCache((uint32_t*)buffer.startAddress, buffer.payloadSize);

	// Configure DCMIPP Memory Destination
	Dcmipp::MemoryDestination dest;
    dest.primaryAddress = (uint32_t)buffer.startAddress;
    dest.isDoubleBuffered = false;
    dest.isSemiPlanar = false;
    dest.isFullPlanar = false;

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

Status CameraMIPI::CaptureWait(uint32_t timeoutTicks) {
	if(this->frame == nullptr) {
		return Status::Error;
	}

	Status status = this->dcmiPPInterface.CaptureWait(Dcmipp::PipeID::Main, timeoutTicks);
	if (status == Status::Ok) {
		// Handle cache coherency
		System::InvalidateCache((uint32_t*)this->frame->startAddress, this->frame->payloadSize);
	}
	else {
		// Disable Interface
		this->CaptureAbort();
	}

	// Clear capture context
	this->frame = nullptr;

	return status;
}

Status CameraMIPI::CaptureAbort() {
	// Stop the sensor PHY first
	this->sensor.Stop();

	// Stop receivers
	this->csiInterface.Stop(Csi::VirtualChannel::VC0);
	this->dcmiPPInterface.CaptureAbort(Dcmipp::PipeID::Main);
	
	// Clear capture context
	this->frame = nullptr;

	return Status::Ok;
}