/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/System/shellFDCAN.cpp
 */

#include "shellModules.hpp"

static void PrintHelp() {
	Logger::Instance().Write("--- FDCAN Commands ---\r\n");
	Logger::Instance().Write("  can write <bus> <id> [b0..b7]       - Transmit frame (ID & bytes in hex)\r\n");
	Logger::Instance().Write("  can read <bus> [timeoutTicks]      - Receive single frame (default 1000 ticks)\r\n");
	Logger::Instance().Write("  can stats <bus>                    - Packet and bus error counters\r\n");
	Logger::Instance().Write("  can reset <bus>                    - Reset FDCAN telemetry counters\r\n");
	Logger::Instance().Write("----------------------\r\n");
}

static FDCAN* GetFDCANBus(uint8_t busID) {
	switch(busID) {
		case 1:
			return &fdcan1;
		case 2:
			return &fdcan2;
		case 3:
			return &fdcan3;
		default:
			return nullptr;
	}
}

static Status Transfer(FDCAN* bus, const FDCAN::Frame &frame, uint32_t timeoutTicks = 50) {
	Status status = bus->TransferAsync(frame);
	if(status != Status::Ok) {
		return status;
	}
	return bus->TransferWait(timeoutTicks);
}

static bool CommandFDCAN(const char* args) {
	if(args == nullptr || args[0] == '\0') {
		PrintHelp();
		return true;
	}

	char action[16] = {0};
	// Parse target action
	int32_t count = sscanf(args, "%15s", action);
	if(count < 1) {
		PrintHelp();
		return true;
	}

	if(strcmp(action, "write") == 0) {
		int32_t busNum = -1;
		uint32_t rawId = 0;
		int offset = 0;

		int32_t parsed = sscanf(args, "%*s %d %x%n", &busNum, &rawId, &offset);
		if(parsed < 2) {
			Logger::Instance().Write("Usage: can write <bus> <id> [b0 b1 b2 ... b7] (hex)\r\n");
			return true;
		}

		FDCAN* bus = GetFDCANBus(static_cast<uint8_t>(busNum));
		if(bus == nullptr) {
			Logger::Instance().Printf("Error: FDCAN%d unavailable.\r\n", busNum);
			return true;
		}

		FDCAN::Frame frame = {};
		frame.id = rawId;
		frame.isExtended = (rawId > 0x7FFU);
		frame.isFD = false;
		frame.bitRateSwitch = false;

		// Parse remaining payload bytes from arguments
		const char* payloadPtr = args + offset;
		char* nextPtr = nullptr;
		while(*payloadPtr != '\0' && frame.length < 64) {
			unsigned long byteVal = strtoul(payloadPtr, &nextPtr, 16);
			if(payloadPtr == nextPtr) {
				// No more hex tokens found
				break;
			}
			frame.data[frame.length] = static_cast<uint8_t>(byteVal);
			frame.length = frame.length + 1;
			payloadPtr = nextPtr;
		}

		if(frame.length > 8) {
			// Auto-promote to FD if payload exceeds 8 bytes
			frame.isFD = true;
		}

		Status status = Transfer(bus, frame);
		Logger::Instance().Printf("FDCAN%d Write [ID: 0x%03X, %uB, %s]: %s\r\n",
									busNum,
									frame.id,
									frame.length,
									frame.isExtended ? "EXT" : "STD",
									(status == Status::Ok) ? "OK" : "FAILED");
		return true;
	}

	if(strcmp(action, "read") == 0) {
		int32_t busNum = -1;
		uint32_t timeoutTicks = 1000;

		int32_t parsed = sscanf(args, "%*s %d %u", &busNum, &timeoutTicks);
		if(parsed < 1) {
			Logger::Instance().Write("Usage: can read <bus> [timeoutTicks (default 1000)]\r\n");
			return true;
		}

		FDCAN* bus = GetFDCANBus(static_cast<uint8_t>(busNum));
		if(bus == nullptr) {
			Logger::Instance().Printf("Error: FDCAN%d unavailable.\r\n", busNum);
			return true;
		}

		FDCAN::Frame frame = {};
		Status status = bus->Receive(frame, timeoutTicks);
		if(status == Status::Timeout) {
			Logger::Instance().Printf("FDCAN%d: Receive timeout (%u ticks).\r\n", busNum, timeoutTicks);
			return true;
		}
		if(status != Status::Ok) {
			Logger::Instance().Printf("Error: FDCAN%d receive failed (%d).\r\n", busNum, static_cast<int>(status));
			return true;
		}

		Logger::Instance().Printf("FDCAN%d Read [ID: 0x%03X, %s%s, %uB]:\r\n",
									busNum,
									frame.id,
									frame.isExtended ? "EXT" : "STD",
									frame.isFD ? " FD" : "",
									frame.length);
		for(uint8_t i = 0; i < frame.length; i++) {
			Logger::Instance().Printf("%02X ", frame.data[i]);
		}
		Logger::Instance().Write("\r\n");
		return true;
	}

	if(strcmp(action, "stats") == 0) {
		int32_t busNum = -1;
		if(sscanf(args, "%*s %d", &busNum) >= 1) {
			FDCAN* bus = GetFDCANBus(static_cast<uint8_t>(busNum));
			if(bus != nullptr) {
				FDCAN::Statistics stats = bus->GetStatistics();
				Logger::Instance().Printf("\r\n--- FDCAN%d Bus Statistics ---\r\n", busNum);
				Logger::Instance().Printf("  TX Success: %lu\r\n", stats.txSuccess);
				Logger::Instance().Printf("  RX Success: %lu\r\n", stats.rxSuccess);
				Logger::Instance().Printf("  TX Errors:  %lu\r\n", stats.txErrors);
				Logger::Instance().Printf("  RX Errors:  %lu\r\n", stats.rxErrors);
				Logger::Instance().Printf("  Bus Errors: %lu\r\n", stats.busErrors);
				Logger::Instance().Printf("  Timeouts:   %lu\r\n", stats.timeouts);
			}
			else {
				Logger::Instance().Printf("Error: FDCAN%d unavailable.\r\n", busNum);
			}
		}
		else {
			Logger::Instance().Write("Usage: can stats <bus>\r\n");
		}
		return true;
	}

	if(strcmp(action, "reset") == 0) {
		int32_t busNum = -1;
		if(sscanf(args, "%*s %d", &busNum) >= 1) {
			FDCAN* bus = GetFDCANBus(static_cast<uint8_t>(busNum));
			if(bus != nullptr) {
				bus->ResetStatistics();
				Logger::Instance().Printf("FDCAN%d statistics reset.\r\n", busNum);
			}
			else {
				Logger::Instance().Printf("Error: FDCAN%d unavailable.\r\n", busNum);
			}
		}
		else {
			Logger::Instance().Write("Usage: can reset <bus>\r\n");
		}
		return true;
	}

	Logger::Instance().Printf("Unknown FDCAN action '%s'.\r\n", action);
	PrintHelp();
	return true;
}

static const Shell::CommandEntry fdcanCommands[] = {
	{ "can", CommandFDCAN, "FDCAN packet transfer, reception test, and diagnostics" },
	{ nullptr, nullptr, nullptr }
};

static Shell::CommandList fdcanShellNode;

void RegisterFDCANCommands() {
	Shell::RegisterCommands(&fdcanShellNode, fdcanCommands);
}