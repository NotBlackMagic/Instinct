/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/System/shellUSB.cpp
 */

#include "shellModules.hpp"

static const char* GetStateString(USBDevice::State state) {
	switch(state) {
		case USBDevice::State::Default:
			return "Default (Reset)";
		case USBDevice::State::Addressed:
			return "Addressed";
		case USBDevice::State::Configured:
			return "Configured (Active)";
		case USBDevice::State::Suspended:
			return "Suspended";
		default: 
			return "Unknown";
	}
}

static void PrintHelp() {
	Logger::Instance().Write("--- USB Commands ---\r\n");
	Logger::Instance().Write("  info         - Link state, device status & class config\r\n");
	Logger::Instance().Write("  stats        - PHY, core, UVC & CDC telemetry\r\n");
	Logger::Instance().Write("  reset        - Reset USB telemetry counters\r\n");
	Logger::Instance().Write("--------------------\r\n");
}

static bool CommandUSB(const char* args) {
	if(args == nullptr || args[0] == '\0') {
		PrintHelp();
		return true;
	}

	char action[16] = {0};
	int count = sscanf(args, "%15s", action);
	if(count < 1) {
		PrintHelp();
		return true;
	}

	if(strcmp(action, "info") == 0) {
		Logger::Instance().Write("\r\n--- USB SUBSYSTEM INFORMATION ---\r\n");

		// Hardware & Link State
		Logger::Instance().Write("[HARDWARE & LINK]\r\n");
		const char* speedStr = (usbHardware.GetBusSpeed() == USB::BusSpeed::High) ? "High-Speed (480 Mbps)" : "Full-Speed (12 Mbps)";
		Logger::Instance().Printf("  Speed:       %s\r\n", speedStr);
		Logger::Instance().Printf("  State:       %s\r\n", GetStateString(usbDevice.GetState()));
		Logger::Instance().Printf("  Frame No:    %u\r\n", usbHardware.GetFrameNumber());

		// UVC Class Config
		Logger::Instance().Write("[UVC VIDEO]\r\n");
		const FrameFormat* uvcFmt = usbUVC.GetActiveFormat();
		if(uvcFmt != nullptr) {
			const char* codecStr = "Unknown";
			if(uvcFmt->codec == VisionCodec::H264) {
				codecStr = "H.264 (Frame-Based)";
			}
			else if(uvcFmt->codec == VisionCodec::Jpeg) {
				codecStr = "MJPEG";
			}
			else if(uvcFmt->format == PixelFormat::YUV422_YUYV) {
				codecStr = "YUY2 (Uncompressed)";
			}
			else if(uvcFmt->format == PixelFormat::YUV420_NV12) {
				codecStr = "NV12 (Semi-Planar)";
			}

			uint32_t fps = (uvcFmt->frameInterval > 0) ? (10000000 / uvcFmt->frameInterval) : 0;
			Logger::Instance().Printf("  Negotiated:  %lux%lu @ %lu fps\r\n", uvcFmt->width, uvcFmt->height, fps);
			Logger::Instance().Printf("  Format:      %s\r\n", codecStr);
		}
		else {
			Logger::Instance().Write("  Negotiated:  Idle / Not Negotiated\r\n");
		}

		// CDC Class Config
		Logger::Instance().Write("[CDC VIRTUAL COM]\r\n");
		Logger::Instance().Printf("  TX Buffer:   %u B\r\n", 2048);
		Logger::Instance().Printf("  RX Buffer:   %u B (Available: %lu B)\r\n", 2048, usbCDC.Available());

		Logger::Instance().Write("---------------------------------\r\n");
		return true;
	}

	if(strcmp(action, "stats") == 0) {
		USB::Statistics busStats = usbHardware.GetStatistics();
		USBDevice::Statistics devStats = usbDevice.GetStatistics();
		USBClassUVC::Statistics uvcStats = usbUVC.GetStatistics();
		USBClassCDC::Statistics cdcStats = usbCDC.GetStatistics();

		Logger::Instance().Write("\r\n--- USB SUBSYSTEM STATS ---\r\n");

		// Bus & Core Protocol Stats
		Logger::Instance().Write("[BUS & PROTOCOL]\r\n");
		Logger::Instance().Printf("  SOF Packets: %lu (Resets: %lu, Suspends: %lu)\r\n", busStats.sofCount, busStats.busResets, busStats.suspendEvents);
		Logger::Instance().Printf("  Setup Pkts:  %lu (Stalls: %lu, Configs: %lu)\r\n", devStats.setupPackets, devStats.controlStalls, devStats.configChanges);
		Logger::Instance().Printf("  Bus Errors:  ISO Incomplete: %lu | TX Timeouts: %lu\r\n", busStats.isoIncomplete, busStats.txTimeouts);

		// Video Streaming Stats
		uint32_t uvcMb = static_cast<uint32_t>(uvcStats.transferredBytes >> 20);
		Logger::Instance().Write("[UVC VIDEO]\r\n");
		Logger::Instance().Printf("  Frames:      %lu sub / %lu ack (Dropped: %lu)\r\n", uvcStats.submittedFrames, uvcStats.completedFrames, uvcStats.droppedFrames);
		Logger::Instance().Printf("  Transferred: %lu MB (%lu B/frame)\r\n", uvcMb, uvcStats.lastFrameBytes);
		Logger::Instance().Printf("  Drain Time:  Last: %6lu | Max: %6lu us\r\n", uvcStats.lastDrainTimeUs, uvcStats.maxDrainTimeUs);

		// Virtual COM Port Stats
		Logger::Instance().Write("[CDC VIRTUAL COM]\r\n");
		Logger::Instance().Printf("  TX / RX:     %llu B / %llu B\r\n", cdcStats.txBytes, cdcStats.rxBytes);
		Logger::Instance().Printf("  TX Drops:    %lu B\r\n", cdcStats.txDroppedBytes);
		Logger::Instance().Printf("  Watermarks:  TX: %lu / 2048 B | RX: %lu / 2048 B\r\n", cdcStats.txBufferWatermark, cdcStats.rxBufferWatermark);

		Logger::Instance().Write("---------------------------\r\n");
		return true;
	}

	if(strcmp(action, "reset") == 0) {
		usbHardware.ResetStatistics();
		usbDevice.ResetStatistics();
		usbUVC.ResetStatistics();
		usbCDC.ResetStatistics();
		Logger::Instance().Write("USB statistics reset.\r\n");
		return true;
	}

	Logger::Instance().Printf("Error: Unknown action '%s'.\r\n", action);
	PrintHelp();
	return true;
}

static const Shell::CommandEntry usbCommands[] = {
	{ "usb", CommandUSB, "USB bus, device, UVC & CDC" },
	{ nullptr, nullptr, nullptr }
};

static Shell::CommandList usbShellNode;

void RegisterUSBCommands() {
	Shell::RegisterCommands(&usbShellNode, usbCommands);
}