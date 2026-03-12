/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/shellI2C.cpp
 */

#include "shellModules.hpp"

#include "i2c.hpp"

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

static void ScanI2CBus(uint8_t busID, I2C* i2c) {
	Logger::Instance().Printf("Scanning I2C%d...\r\n", busID);
	Logger::Instance().Write("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");

	for (int i = 0; i < 128; i += 16) {
		Logger::Instance().Printf("%02x: ", i);

		for (int j = 0; j < 16; j++) {
			uint16_t addr = i + j;

			//Skip reserved addresses
			if(addr < 0x03 || addr > 0x77) {
					Logger::Instance().Write("   "); 
					continue;
			}

			if(i2c->Probe(addr)) {
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
	if (args == nullptr || args[0] == '\0') {
		Logger::Instance().Write("Usage: i2c scan [bus_num]\r\n");
		return true;
	}

	char mode[10];
    int busNum = -1;
	uint8_t count = sscanf(args, "%9s %d", mode, &busNum);
	if (count < 1 || strcmp(mode, "scan") != 0) {
		Logger::Instance().Write("Usage: i2c scan [bus_num]\r\n");
		return true;
	}

	if (busNum != -1) {
        // Case A: User asked for specific bus (e.g., "i2c scan 2")
        I2C* bus = GetI2CBus(busNum);
        if (bus) {
            ScanI2CBus(busNum, bus);
        } else {
            Logger::Instance().Printf("Error: I2C%d not defined.\r\n", busNum);
        }
	}
	else {
		Logger::Instance().Write("Error: Please specify a valid bus ID.\r\n");
	}

	return true;
}

static const Shell::CommandEntry i2cCommands[] = {
	{ "i2c", CommandI2C, "I2C Bus Scan and Debug" },
	{ nullptr, nullptr, nullptr } // Terminator
};

static Shell::CommandList i2cShellNode;

void RegisterI2CCommands() {
	Shell::RegisterCommands(&i2cShellNode, i2cCommands);
}