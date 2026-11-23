/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Vision/cameraMIPI.cpp
 */

#include "cameraMIPI.hpp"

Status CameraMIPI::Init(const Config &config) {
	Status status;
	this->sensorConfig = config;

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
	if(config.format == PixelFormat::YUV422_YUYV || config.format == PixelFormat::YUV422_YVYU || config.format == PixelFormat::YUV422_UYVY || config.format == PixelFormat::YUV420_NV12 || config.format == PixelFormat::RGB888) {
		ov5645Cnfg.format = PixelFormat::YUV422_UYVY;
		this->mipiDataType = Csi::MIPIDataType::YUV422_8bit;
	}
	else if(config.format == PixelFormat::RGB565) {
		ov5645Cnfg.format = PixelFormat::RGB565;
		this->mipiDataType = Csi::MIPIDataType::RGB565;
	}
	else {
		return Status::Error;
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

	// Initialize Interface Hardware (DCMIPP)
	status = this->dcmiPPInterface.Init();
	if (status != Status::Ok) {
		return status;
	}

	status = this->csiInterface.ConfigureVirtualChannel(Csi::VirtualChannel::VC0, mipiDataType);
	if (status != Status::Ok) {
		return status;
	}

	// status = this->dcmiPPInterface.LinkCSIVirtualChannel(Dcmipp::PipeID::Main, Csi::VirtualChannel::VC0, mipiDataType);
	// if (status != Status::Ok) {
	// 	return status;
	// }

	// Default configuration: Pipe 1 (Main) at full sensor resolution
	StreamConfig defaultMain;
	defaultMain.width = config.width;
	defaultMain.height = config.height;
	defaultMain.format = config.format;
	defaultMain.frameRate = Dcmipp::FrameRateDecimation::All;

	return this->ConfigurePipe(Dcmipp::PipeID::Main, defaultMain);
}

Status CameraMIPI::ConfigurePipe(Dcmipp::PipeID pipe, const StreamConfig &streamConfig) {
	// Resolve active input and output dimensions
	uint16_t inWidth = (streamConfig.cropWidth > 0U) ? streamConfig.cropWidth : this->sensorConfig.width;
	uint16_t inHeight = (streamConfig.cropHeight > 0U) ? streamConfig.cropHeight : this->sensorConfig.height;
	uint16_t outWidth = (streamConfig.width > 0U) ? streamConfig.width : inWidth;
	uint16_t outHeight = (streamConfig.height > 0U) ? streamConfig.height : inHeight;

	Dcmipp::PipeConfig dcmiPipe = {};
	dcmiPipe.frameRate = streamConfig.frameRate;

	switch(streamConfig.format) {
		case PixelFormat::RGB888:
			dcmiPipe.format = Dcmipp::OutputFormat::RGB888;
			dcmiPipe.swapRBUV = false;
			dcmiPipe.pixelPitch = outWidth * 3U;
			break;
		case PixelFormat::RGB565:
			dcmiPipe.format = Dcmipp::OutputFormat::RGB565;
			dcmiPipe.swapRBUV = false;
			dcmiPipe.pixelPitch = outWidth * 2U;
			break;
		case PixelFormat::YUV422_YUYV:
			dcmiPipe.format = Dcmipp::OutputFormat::YUV422_YUYV;
			dcmiPipe.swapRBUV = false;
			dcmiPipe.pixelPitch = outWidth * 2U;
			break;
		case PixelFormat::YUV422_YVYU:
			dcmiPipe.format = Dcmipp::OutputFormat::YUV422_YUYV;
			dcmiPipe.swapRBUV = true;
			dcmiPipe.pixelPitch = outWidth * 2U;
			break;
		case PixelFormat::YUV422_UYVY:
			dcmiPipe.format = Dcmipp::OutputFormat::YUV422_UYVY;
			dcmiPipe.swapRBUV = false;
			dcmiPipe.pixelPitch = outWidth * 2U;
			break;
		case PixelFormat::YUV420_NV12:
			dcmiPipe.format = Dcmipp::OutputFormat::YUV420_NV21;
			dcmiPipe.swapRBUV = false;
			dcmiPipe.pixelPitch = outWidth;
			break;
		case PixelFormat::Mono:
			dcmiPipe.format = Dcmipp::OutputFormat::Mono;
			dcmiPipe.swapRBUV = false;
			dcmiPipe.pixelPitch = outWidth;
			break;
		default:
			return Status::Error;
	}

	// Pipe 0 (Dump) does not perform color-space conversion, only optional byte swap
	if(pipe == Dcmipp::PipeID::Dump && streamConfig.format != PixelFormat::YUV422_UYVY && streamConfig.format != PixelFormat::YUV422_YUYV && streamConfig.format != PixelFormat::RGB565) {
		return Status::Error;
	}

	Status status = this->dcmiPPInterface.LinkCSIVirtualChannel(pipe, Csi::VirtualChannel::VC0, this->mipiDataType);
	if(status != Status::Ok) {
		return status;
	}

	status = this->dcmiPPInterface.ConfigurePipe(pipe, dcmiPipe);
	if(status != Status::Ok) {
		return status;
	}

	Dcmipp::WindowConfig winCfg = {};
	winCfg.srcWidth = this->sensorConfig.width;
	winCfg.srcHeight = this->sensorConfig.height;
	winCfg.cropX = streamConfig.cropX;
	winCfg.cropY = streamConfig.cropY;
	winCfg.cropWidth = streamConfig.cropWidth;
	winCfg.cropHeight = streamConfig.cropHeight;
	winCfg.outWidth = outWidth;
	winCfg.outHeight = outHeight;

	return this->dcmiPPInterface.ConfigureWindow(pipe, winCfg);
}

Status CameraMIPI::CaptureAsync(Dcmipp::PipeID pipe, VisionFrame &buffer) {
	// Empty pointer check
	if(buffer.startAddress == nullptr) {
		return Status::Error;
	}

	// Alignment check, for DMA cache stuff
	if(((uint32_t)(buffer.startAddress) & 0x1F) != 0) {
		return Status::Error;
	}

	// Save capture context
	PipeContext &ctx = this->pipeCtx[static_cast<uint8_t>(pipe)];
	ctx.mode = Mode::Snapshot;
	ctx.currentFrameIdx = 0;
	ctx.activeFrames[0] = &buffer;
	ctx.activeFrames[1] = nullptr;

	// Ensure line wrapping is disabled for full-frame snapshot
	if(pipe != Dcmipp::PipeID::Dump) {
		this->dcmiPPInterface.DisableLineWrapping(pipe);
	}

	// Handle cache coherency
	System::CleanCache((uint32_t*)buffer.startAddress, buffer.payloadSize);

	// Configure DCMIPP Memory Destination
	Dcmipp::MemoryDestination dest;
	dest.primaryAddress = (uint32_t)buffer.startAddress;
	dest.isDoubleBuffered = false;
	dest.isFullPlanar = false;

	if(buffer.format == PixelFormat::YUV420_NV12) {
		dest.isSemiPlanar = true;
		dest.uAddress = dest.primaryAddress + (buffer.width * buffer.height);
	}
	else {
		dest.isSemiPlanar = false;
		dest.uAddress = 0;
	}

	// Enable interface (Receiver side first)
	Status status = this->dcmiPPInterface.CaptureAsync(pipe, dest, Dcmipp::CaptureMode::Snapshot);
	if(status != Status::Ok) {
		ctx.mode = Mode::Idle;
		return status;
	}

	return this->StartSensor(pipe);
}

Status CameraMIPI::CaptureAsync(Dcmipp::PipeID pipe, VisionFrame &buffer0, VisionFrame &buffer1) {
	// Empty pointer check
	if(buffer0.startAddress == nullptr || buffer1.startAddress == nullptr) {
		return Status::Error;
	}

	// Alignment check, for DMA cache stuff
	if(((uint32_t)(buffer0.startAddress) & 0x1F) != 0 || ((uint32_t)(buffer1.startAddress) & 0x1F) != 0) {
		return Status::Error;
	}

	// Save capture context
	PipeContext &ctx = this->pipeCtx[static_cast<uint8_t>(pipe)];
	ctx.mode = Mode::Continuous;
	ctx.currentFrameIdx = 0;
	ctx.activeFrames[0] = &buffer0;
	ctx.activeFrames[1] = &buffer1;

	// Ensure line wrapping is disabled for full-frame continuous mode
	if(pipe != Dcmipp::PipeID::Dump) {
		this->dcmiPPInterface.DisableLineWrapping(pipe);
	}

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
		dest.vAddress = dest.secondaryAddress + (buffer1.width * buffer1.height);
	}
	else {
		dest.isSemiPlanar = false;
		dest.uAddress = 0;
		dest.vAddress = 0;
	}

	// Enable interface (Receiver side first)
	Status status = this->dcmiPPInterface.CaptureAsync(pipe, dest, Dcmipp::CaptureMode::Continuous);
	if(status != Status::Ok) {
		ctx.mode = Mode::Idle;
		return status;
	}

	return this->StartSensor(pipe);
}

Status CameraMIPI::CaptureAsync(Dcmipp::PipeID pipe, VisionFrame& buffer, Dcmipp::LineCount lineMult, Dcmipp::LineCount wrapAddress) {
	// Pipe check, line mode not supported for Pipe 0
	if(pipe == Dcmipp::PipeID::Dump) {
		return Status::Error;
	}
	
	// Empty pointer check
	if(buffer.startAddress == nullptr) {
		return Status::Error;
	}

	// Alignment check, for DMA cache stuff
	if(((uint32_t)(buffer.startAddress) & 0x1F) != 0) {
		return Status::Error;
	}

	// Save capture context
	PipeContext &ctx = this->pipeCtx[static_cast<uint8_t>(pipe)];
	ctx.mode = Mode::Partial;
	ctx.currentFrameIdx = 0;
	ctx.activeFrames[0] = &buffer;
	ctx.activeFrames[1] = nullptr;

	// Enable hardware line wrapping
	this->dcmiPPInterface.EnableLineWrapping(pipe, wrapAddress, lineMult);

	// Handle cache coherency
	System::CleanCache((uint32_t*)buffer.startAddress, buffer.payloadSize);

	// Configure DCMIPP Memory Destination
	Dcmipp::MemoryDestination dest;
	dest.primaryAddress = (uint32_t)buffer.startAddress;
	dest.isDoubleBuffered = false;
	dest.isFullPlanar = false;

	if(buffer.format == PixelFormat::YUV420_NV12) {
		dest.isSemiPlanar = true;
		uint32_t wrapLines = 1U << static_cast<uint32_t>(wrapAddress);
		dest.uAddress = dest.primaryAddress + (buffer.width * wrapLines);
	}
	else {
		dest.isSemiPlanar = false;
		dest.uAddress = 0;
	}

	Status status = this->dcmiPPInterface.CaptureAsync(pipe, dest, Dcmipp::CaptureMode::Continuous);
	if(status != Status::Ok) {
		ctx.mode = Mode::Idle;
		return status;
	}

	return this->StartSensor(pipe);
}

Status CameraMIPI::CaptureWait(Dcmipp::PipeID pipe, uint32_t timeout) {
	PipeContext &ctx = this->pipeCtx[static_cast<uint8_t>(pipe)];
	if(ctx.mode == Mode::Idle) {
		return Status::Error;
	}

	Status status = this->dcmiPPInterface.CaptureWait(pipe, timeout);
	if(status == Status::Ok) {
		VisionFrame* readyFrame = ctx.activeFrames[ctx.currentFrameIdx];

		// Automatically invalidate the cache for whichever buffer the hardware just completed
		if(ctx.mode != Mode::Partial && readyFrame != nullptr) {
			System::InvalidateCache((uint32_t*)readyFrame->startAddress, readyFrame->payloadSize);
		}

		// Frame timestamp & FPS calculation
		uint32_t nowUs = Time::GetUs();
		if(readyFrame != nullptr) {
			readyFrame->timestampUs = nowUs;
		}
		ctx.stats.frames = ctx.stats.frames + 1;

		if(ctx.timestamp != 0) {
			uint32_t dt = nowUs - ctx.timestamp;
			if(dt > 0) {
				float instantFps = 1000000.0f / static_cast<float>(dt);
				ctx.stats.fps = (ctx.stats.fps == 0.0f) ? instantFps : (0.8f * ctx.stats.fps + 0.2f * instantFps);
			}
		}
		ctx.timestamp = nowUs;

		// State Machine Management
		if(ctx.mode == Mode::Continuous) {
			// Ping-pong the index for the next call
			ctx.currentFrameIdx = 1 - ctx.currentFrameIdx;
		}
		else if(ctx.mode == Mode::Partial) {
			// Line mode runs continuously into the single circular buffer
			ctx.currentFrameIdx = 0;
		}
		else {
			// Snapshots and Partials complete after one wait
			ctx.mode = Mode::Idle;
			ctx.activeFrames[0] = nullptr;
			this->StopSensor(pipe);
		}
	}
	else {
		ctx.stats.errors = ctx.stats.errors + 1;
		
		// Disable Interface
		this->CaptureAbort(pipe);
	}

	return status;
}

Status CameraMIPI::CaptureAbort(Dcmipp::PipeID pipe) {
	// Stop the sensor PHY first
	this->StopSensor(pipe);

	// Stop receivers
	this->dcmiPPInterface.CaptureAbort(Dcmipp::PipeID::Main);
	
	// Clear capture context
	PipeContext &ctx = this->pipeCtx[static_cast<uint8_t>(pipe)];
	ctx.mode = Mode::Idle;
	ctx.activeFrames[0] = nullptr;
	ctx.activeFrames[1] = nullptr;

	return Status::Ok;
}

Status CameraMIPI::StartSensor(Dcmipp::PipeID pipe) {
	bool needStart = (this->activePipeMask == 0U);
	this->activePipeMask |= (1U << static_cast<uint8_t>(pipe));

	if(needStart == true) {
		Status status = this->csiInterface.Start(Csi::VirtualChannel::VC0);
		if(status != Status::Ok) {
			this->StopSensor(pipe);
			return status;
		}

		status = this->sensor.Start();
		if(status != Status::Ok) {
			this->StopSensor(pipe);
			return status;
		}
	}

	return Status::Ok;
}

void CameraMIPI::StopSensor(Dcmipp::PipeID pipe) {
	this->activePipeMask &= ~(1U << static_cast<uint8_t>(pipe));
	bool allIdle = (this->activePipeMask == 0U);
	if(allIdle == true) {
		this->sensor.Stop();
		this->csiInterface.Stop(Csi::VirtualChannel::VC0);
	}
}