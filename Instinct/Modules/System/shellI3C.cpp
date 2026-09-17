/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/System/shellI3C.cpp
 */

#include "shellModules.hpp"

static void PrintHelp() {
	Logger::Instance().Write("--- I3C Diagnostic Commands ---\r\n");
	Logger::Instance().Write("  i3c scan [bus]                      - Scan specific bus or all available buses (Legacy I2C mode)\r\n");
	Logger::Instance().Write("  i3c read <bus> <addr> <reg> [len]   - Read registers via Legacy I2C (addr/reg in hex, len max 64)\r\n");
	Logger::Instance().Write("  i3c write <bus> <addr> <reg> <b0>.. - Write 1-8 data bytes to register (hex)\r\n");
	Logger::Instance().Write("  i3c dump <bus> <addr> [start] [len] - Hexdump memory map (defaults 0x00 32, max 128)\r\n");
	Logger::Instance().Write("  i3c stats <bus>                     - Bus transfer and error counters\r\n");
	Logger::Instance().Write("  i3c reset <bus>                     - Reset I3C telemetry counters\r\n");
	Logger::Instance().Write("-------------------------------\r\n");
}

static I3C* GetI3CBus(uint8_t busId) {
	switch(busId) {
		case 1:
			return &i3c1;
		case 2:
			return &i3c2;
		default:
			return nullptr;
	}
}

static Status Transfer(I3C* bus, uint8_t addr, I3C::TargetType type, const uint8_t* txBuf, uint16_t txLen, uint8_t* rxBuf, uint16_t rxLen, uint32_t timeoutTicks = 50) {
	Status status = bus->TransferAsync(addr, type, txBuf, txLen, rxBuf, rxLen);
	if(status != Status::Ok) {
		return status;
	}
	return bus->TransferWait(timeoutTicks);
}

static void ScanI3CBus(uint8_t busId, I3C* i3c) {
	Logger::Instance().Printf("\r\n--- Scanning I3C%u (Legacy I2C) ---\r\n", busId);
	Logger::Instance().Write("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");

	for(uint8_t i = 0; i < 128; i += 16) {
		Logger::Instance().Printf("%02x: ", i);

		for(uint8_t j = 0; j < 16; j++) {
			uint8_t addr = static_cast<uint8_t>(i + j);

			// Skip reserved addresses
			if(addr < 0x03 || addr > 0x77) {
				Logger::Instance().Write("   "); 
				continue;
			}

			if(i3c->Probe(addr) == 1) {
				Logger::Instance().Printf("%02x ", addr); 
			}
			else {
				Logger::Instance().Write("-- "); 
			}
		}
		Logger::Instance().Write("\r\n");
	}
}

static bool CommandI3C(const char* args) {
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

	if(strcmp(action, "scan") == 0) {
		int32_t busNum = -1;
		if(sscanf(args, "%*s %d", &busNum) >= 1) {
			I3C* bus = GetI3CBus(static_cast<uint8_t>(busNum));
			if(bus != nullptr) {
				ScanI3CBus(static_cast<uint8_t>(busNum), bus);
			}
			else {
				Logger::Instance().Printf("Error: I3C%d unavailable.\r\n", busNum);
			}
		}
		else {
			// Scan all defined buses
			const uint8_t activeBuses[] = { 1, 2 };
			for(uint8_t busId : activeBuses) {
				I3C* bus = GetI3CBus(busId);
				if(bus != nullptr) {
					ScanI3CBus(busId, bus);
				}
			}
		}
		return true;
	}

	if(strcmp(action, "read") == 0) {
		int32_t busNum = -1;
		uint32_t addr = 0;
		uint32_t reg = 0;
		uint32_t len = 1;

		int32_t parsed = sscanf(args, "%*s %d %x %x %u", &busNum, &addr, &reg, &len);
		if(parsed < 3 || len == 0 || len > 64) {
			Logger::Instance().Write("Usage: i3c read <bus> <addr> <reg> [len (1-64)]\r\n");
			return true;
		}

		I3C* bus = GetI3CBus(static_cast<uint8_t>(busNum));
		if(bus == nullptr) {
			Logger::Instance().Printf("Error: I3C%d unavailable.\r\n", busNum);
			return true;
		}

		uint8_t regAddr = static_cast<uint8_t>(reg);
		uint8_t rxBuf[64] = {0};
		Status status = Transfer(bus, static_cast<uint8_t>(addr), I3C::TargetType::I2C, &regAddr, 1, rxBuf, static_cast<uint16_t>(len));
		if(status != Status::Ok) {
			Logger::Instance().Printf("Error: I3C read failed (%d).\r\n", static_cast<int>(status));
			return true;
		}

		Logger::Instance().Printf("I3C%d [0x%02X] Reg 0x%02X (%u bytes):\r\n  ", busNum, addr, reg, len);
		for(uint32_t i = 0; i < len; i++) {
			Logger::Instance().Printf("%02X ", rxBuf[i]);
		}
		Logger::Instance().Write("\r\n");
		return true;
	}

	if(strcmp(action, "write") == 0) {
		int32_t busNum = -1;
		uint32_t addr = 0;
		uint32_t reg = 0;
		uint32_t bytes[8] = {0};

		int32_t parsed = sscanf(args, "%*s %d %x %x %x %x %x %x %x %x %x %x",
							&busNum, &addr, &reg,
							&bytes[0], &bytes[1], &bytes[2], &bytes[3],
							&bytes[4], &bytes[5], &bytes[6], &bytes[7]);

		if(parsed < 4) {
			Logger::Instance().Write("Usage: i3c write <bus> <addr> <reg> <b0> [b1..b7]\r\n");
			return true;
		}

		I3C* bus = GetI3CBus(static_cast<uint8_t>(busNum));
		if(bus == nullptr) {
			Logger::Instance().Printf("Error: I3C%d unavailable.\r\n", busNum);
			return true;
		}

		uint8_t payloadLen = static_cast<uint8_t>(parsed - 3);
		uint8_t txBuf[9];
		txBuf[0] = static_cast<uint8_t>(reg);
		for(uint8_t i = 0; i < payloadLen; i++) {
			txBuf[i + 1] = static_cast<uint8_t>(bytes[i]);
		}
		Status status = Transfer(bus, static_cast<uint8_t>(addr), I3C::TargetType::I2C, txBuf, payloadLen + 1, nullptr, 0);
		Logger::Instance().Printf("I3C%d Write [0x%02X] Reg 0x%02X: %s\r\n",
									busNum, addr, reg, (status == Status::Ok) ? "OK" : "FAILED");
		return true;
	}

	if(strcmp(action, "dump") == 0) {
		int32_t busNum = -1;
		uint32_t addr = 0;
		uint32_t startReg = 0;
		uint32_t len = 32;

		int32_t parsed = sscanf(args, "%*s %d %x %x %u", &busNum, &addr, &startReg, &len);
		if(parsed < 2 || len == 0 || len > 128) {
			Logger::Instance().Write("Usage: i3c dump <bus> <addr> [start (default 0x00)] [len (1-128, default 32)]\r\n");
			return true;
		}

		I3C* bus = GetI3CBus(static_cast<uint8_t>(busNum));
		if(bus == nullptr) {
			Logger::Instance().Printf("Error: I3C%d unavailable.\r\n", busNum);
			return true;
		}

		uint8_t regAddr = static_cast<uint8_t>(startReg);
		uint8_t rxBuf[128] = {0};

		Status status = Transfer(bus, static_cast<uint8_t>(addr), I3C::TargetType::I2C, &regAddr, 1, rxBuf, static_cast<uint16_t>(len));
		if(status != Status::Ok) {
			Logger::Instance().Printf("Error: I3C dump failed (%d).\r\n", static_cast<int>(status));
			return true;
		}

		Logger::Instance().Printf("\r\n--- Dump I3C%d [0x%02X] Reg 0x%02X (%u bytes) ---\r\n", busNum, addr, startReg, len);
		Logger::Instance().Write("Offset  00 01 02 03 04 05 06 07  08 09 0A 0B 0C 0D 0E 0F   ASCII\r\n");

		for(uint32_t i = 0; i < len; i += 16) {
			Logger::Instance().Printf("0x%02X:   ", startReg + i);
			uint32_t chunk = (len - i < 16) ? (len - i) : 16;

			for(uint32_t j = 0; j < 16; j++) {
				if(j < chunk) {
					Logger::Instance().Printf("%02X ", rxBuf[i + j]);
				}
				else {
					Logger::Instance().Write("   ");
				}
				if(j == 7) {
					Logger::Instance().Write(" ");
				}
			}

			Logger::Instance().Write(" |");
			for(uint32_t j = 0; j < chunk; j++) {
				char c = static_cast<char>(rxBuf[i + j]);
				Logger::Instance().Printf("%c", (c >= 32 && c <= 126) ? c : '.');
			}
			Logger::Instance().Write("|\r\n");
		}
		return true;
	}

	if(strcmp(action, "stats") == 0) {
		int32_t busNum = -1;
		if(sscanf(args, "%*s %d", &busNum) >= 1) {
			I3C* bus = GetI3CBus(static_cast<uint8_t>(busNum));
			if(bus != nullptr) {
				I3C::Statistics stats = bus->GetStatistics();
				Logger::Instance().Printf("\r\n--- I3C%d Bus Statistics ---\r\n", busNum);
				Logger::Instance().Printf("  Completed: %lu\r\n", stats.completedTransfers);
				Logger::Instance().Printf("  NACK:      %lu\r\n", stats.nackErrors);
				Logger::Instance().Printf("  Bus Error: %lu\r\n", stats.busErrors);
				Logger::Instance().Printf("  Timeout:   %lu\r\n", stats.timeouts);
			}
			else {
				Logger::Instance().Printf("Error: I3C%d unavailable.\r\n", busNum);
			}
		}
		else {
			Logger::Instance().Write("Usage: i3c stats <bus>\r\n");
		}
		return true;
	}

	if(strcmp(action, "reset") == 0) {
		int32_t busNum = -1;
		if(sscanf(args, "%*s %d", &busNum) >= 1) {
			I3C* bus = GetI3CBus(static_cast<uint8_t>(busNum));
			if(bus != nullptr) {
				bus->ResetStatistics();
				Logger::Instance().Write("I3C%d statistics reset.\r\n", busNum);
			}
			else {
				Logger::Instance().Printf("Error: I3C%d unavailable.\r\n", busNum);
			}
		}
		else {
			Logger::Instance().Write("Usage: i2c stats <bus>\r\n");
		}
		return true;
	}

	Logger::Instance().Printf("Unknown I3C action '%s'.\r\n", action);
	PrintHelp();
	return true;
}

static const Shell::CommandEntry i3cCommands[] = {
	{ "i3c", CommandI3C, "I3C bus exploration, register peek/poke, and diagnostics" },
	{ nullptr, nullptr, nullptr }
};

static Shell::CommandList i3cShellNode;

void RegisterI3CCommands() {
	Shell::RegisterCommands(&i3cShellNode, i3cCommands);
}