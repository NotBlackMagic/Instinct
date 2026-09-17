/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/System/shellI2C.cpp
 */

#include "shellModules.hpp"

static void PrintHelp() {
	Logger::Instance().Write("--- I2C Diagnostic Commands ---\r\n");
	Logger::Instance().Write("  i2c scan [bus]                      - Scan specific bus or all available buses\r\n");
	Logger::Instance().Write("  i2c read <bus> <addr> <reg> [len]   - Read registers (addr/reg in hex, len max 64)\r\n");
	Logger::Instance().Write("  i2c write <bus> <addr> <reg> <b0>.. - Write 1-8 data bytes to register (hex)\r\n");
	Logger::Instance().Write("  i2c dump <bus> <addr> [start] [len] - Hexdump memory map of device (defaults 0x00 32)\r\n");
	Logger::Instance().Write("  i2c stats <bus>                     - Bus transfer and error counters\r\n");
	Logger::Instance().Write("  i2c reset <bus>                     - Reset I2C telemetry counters\r\n");
	Logger::Instance().Write("-------------------------------\r\n");
}

static I2C* GetI2CBus(uint8_t busID) {
	switch(busID){
		case 1:
			return &i2c1;
		case 2:
			return &i2c2;
		case 3:
			return nullptr;
		case 4:
			return &i2c4;
		default:
			return nullptr;
	}
}

static Status Transfer(I2C* bus, uint16_t addr, const uint8_t* txBuf, uint16_t txLen, uint8_t* rxBuf, uint16_t rxLen, uint32_t timeoutTicks = 50) {
	Status status = bus->TransferAsync(addr, txBuf, txLen, rxBuf, rxLen);
	if(status != Status::Ok) {
		return status;
	}
	return bus->TransferWait(timeoutTicks);
}

static void ScanI2CBus(uint8_t busID, I2C* i2c) {
	Logger::Instance().Printf("\r\n--- Scanning I2C%u ---\r\n", busID);
	Logger::Instance().Write("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");

	for(uint8_t i = 0; i < 128; i += 16) {
		Logger::Instance().Printf("%02x: ", i);

		for(uint8_t j = 0; j < 16; j++) {
			uint16_t addr = i + j;

			// Skip reserved addresses
			if(addr < 0x03 || addr > 0x77) {
					Logger::Instance().Write("   "); 
					continue;
			}

			if(i2c->Probe(addr) == true) {
				Logger::Instance().Printf("%02x ", addr); 
			}
			else {
				Logger::Instance().Write("-- "); 
			}
		}
		Logger::Instance().Write("\r\n");
	}
}

static bool CommandI2C(const char* args) {
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
			I2C* bus = GetI2CBus(static_cast<uint8_t>(busNum));
			if(bus != nullptr) {
				ScanI2CBus(static_cast<uint8_t>(busNum), bus);
			}
			else {
				Logger::Instance().Printf("Error: I2C%d unavailable.\r\n", busNum);
			}
		}
		else {
			// Scan all defined buses
			const uint8_t activeBuses[] = { 1, 2, 4 };
			for(uint8_t busID : activeBuses) {
				I2C* bus = GetI2CBus(busID);
				if(bus != nullptr) {
					ScanI2CBus(busID, bus);
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
			Logger::Instance().Write("Usage: i2c read <bus> <addr> <reg> [len (1-64)]\r\n");
			return true;
		}

		I2C* bus = GetI2CBus(static_cast<uint8_t>(busNum));
		if(bus == nullptr) {
			Logger::Instance().Printf("Error: I2C%d unavailable.\r\n", busNum);
			return true;
		}

		uint8_t regAddr = static_cast<uint8_t>(reg);
		uint8_t rxBuf[64] = {0};
		Status status = Transfer(bus, static_cast<uint16_t>(addr), &regAddr, 1, rxBuf, static_cast<uint16_t>(len));
		if(status != Status::Ok) {
			Logger::Instance().Printf("Error: I2C read failed (%d).\r\n", static_cast<int>(status));
			return true;
		}

		Logger::Instance().Printf("I2C%d [0x%02X] Reg 0x%02X (%u bytes):\r\n  ", busNum, addr, reg, len);
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
			Logger::Instance().Write("Usage: i2c write <bus> <addr> <reg> <b0> [b1..b7]\r\n");
			return true;
		}

		I2C* bus = GetI2CBus(static_cast<uint8_t>(busNum));
		if(bus == nullptr) {
			Logger::Instance().Printf("Error: I2C%d unavailable.\r\n", busNum);
			return true;
		}

		uint8_t payloadLen = static_cast<uint8_t>(parsed - 3);
		uint8_t txBuf[9];
		txBuf[0] = static_cast<uint8_t>(reg);
		for(uint8_t i = 0; i < payloadLen; i++) {
			txBuf[i + 1] = static_cast<uint8_t>(bytes[i]);
		}
		Status status = Transfer(bus, static_cast<uint16_t>(addr), txBuf, payloadLen + 1, nullptr, 0);
		Logger::Instance().Printf("I2C%d Write [0x%02X] Reg 0x%02X: %s\r\n",
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
			Logger::Instance().Write("Usage: i2c dump <bus> <addr> [start (default 0x00)] [len (1-128, default 32)]\r\n");
			return true;
		}

		I2C* bus = GetI2CBus(static_cast<uint8_t>(busNum));
		if(bus == nullptr) {
			Logger::Instance().Printf("Error: I2C%d unavailable.\r\n", busNum);
			return true;
		}

		uint8_t regAddr = static_cast<uint8_t>(startReg);
		uint8_t rxBuf[128] = {0};

		Status status = Transfer(bus, static_cast<uint16_t>(addr), &regAddr, 1, rxBuf, static_cast<uint16_t>(len));
		if(status != Status::Ok) {
			Logger::Instance().Printf("Error: I2C dump failed (%d).\r\n", static_cast<int>(status));
			return true;
		}

		Logger::Instance().Printf("\r\n--- Dump I2C%d [0x%02X] Reg 0x%02X (%u bytes) ---\r\n", busNum, addr, startReg, len);
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
			I2C* bus = GetI2CBus(static_cast<uint8_t>(busNum));
			if(bus != nullptr) {
				I2C::Statistics stats = bus->GetStatistics();
				Logger::Instance().Printf("\r\n--- I2C%d Bus Statistics ---\r\n", busNum);
				Logger::Instance().Printf("  Completed: %lu\r\n", stats.completedTransfers);
				Logger::Instance().Printf("  NACK:      %lu\r\n", stats.nackErrors);
				Logger::Instance().Printf("  Bus Error: %lu\r\n", stats.busErrors);
				Logger::Instance().Printf("  Timeout:   %lu\r\n", stats.timeouts);
			}
			else {
				Logger::Instance().Printf("Error: I2C%d unavailable.\r\n", busNum);
			}
		}
		else {
			Logger::Instance().Write("Usage: i2c stats <bus>\r\n");
		}
		return true;
	}

	if(strcmp(action, "reset") == 0) {
		int32_t busNum = -1;
		if(sscanf(args, "%*s %d", &busNum) >= 1) {
			I2C* bus = GetI2CBus(static_cast<uint8_t>(busNum));
			if(bus != nullptr) {
				bus->ResetStatistics();
				Logger::Instance().Write("I2C%d statistics reset.\r\n", busNum);
			}
			else {
				Logger::Instance().Printf("Error: I2C%d unavailable.\r\n", busNum);
			}
		}
		else {
			Logger::Instance().Write("Usage: i2c stats <bus>\r\n");
		}
		return true;
	}

	Logger::Instance().Printf("Unknown I2C action '%s'.\r\n", action);
	PrintHelp();
	return true;
}

static const Shell::CommandEntry i2cCommands[] = {
	{ "i2c", CommandI2C, "I2C bus exploration, register peek/poke, and diagnostics" },
	{ nullptr, nullptr, nullptr }
};

static Shell::CommandList i2cShellNode;

void RegisterI2CCommands() {
	Shell::RegisterCommands(&i2cShellNode, i2cCommands);
}