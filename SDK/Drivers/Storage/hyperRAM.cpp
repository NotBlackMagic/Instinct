/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Storage/hyperRAM.cpp
 */

#include "hyperRAM.hpp"

Status HyperRAM::Init(const Config &config) {
	if(config.frequencyHz == 0 || config.sourceClockHz == 0) {
		return Status::Error;
	}

	this->config = config;

	// Configure the Bus
	HyperBus::Config hyperBusCnfg;
	hyperBusCnfg.sizeBytes = config.sizeBytes;
	hyperBusCnfg.sourceClockHz = config.sourceClockHz;
	hyperBusCnfg.frequencyHz = config.frequencyHz;
	if(config.fixedLatency == true) {
		// For 16-bit mode, "variable latency is not supported (..) LM in XSPI_HLCR must be set."
		hyperBusCnfg.latencyMode = HyperBus::LatencyMode::Fixed;
	}
	else {
		hyperBusCnfg.latencyMode = HyperBus::LatencyMode::Variable;
	}
	hyperBusCnfg.initialLatency = config.initialLatency;
	hyperBusCnfg.rwRecoveryTime = config.rwRecoveryTime;
	if(config.writeZeroLatency == true) {
		hyperBusCnfg.writeZeroLatency = true;
	}
	else {
		hyperBusCnfg.writeZeroLatency = false;
	}
	uint32_t busClockFreq = config.frequencyHz;
	hyperBusCnfg.refreshRate = this->config.refreshRateUs * (busClockFreq / 1000000);	//Convert from ns to clock cycles
	
	// Initialize the Bus Hardware
	if(this->bus.Init(hyperBusCnfg) != Status::Ok) {
		return Status::Error;
	}

	// Verify ID
	uint8_t manID, devType;
	if(this->ReadID(&manID, &devType) != Status::Ok) {
		return Status::Error;
	}

	if(config.expectedID != 0 && manID != config.expectedID) {
		return Status::Error;
	}

	// Set PSRAM Configs
	if(config.configReg0 != 0) {
		// Configuration Register 0: 0xC0 0x00 0x01 0x00 0x00 0x00
		// Default: 0x112F
		// [15]    : Deep power down (Default: 0b)
		// [14:12] : Drive strength (Default: 000b)
		// [11:8]  : Reserved (Default: 1111b)
		// [7:4]   : Initial latency (Default: 0010b)
		// [3]     : Fixed latency enable (Default: 1b)
		// [2]     : Hybrid burst enable (Default: 1b)
		// [1:0]   : Burst length (Default: 11b)
		if(this->WriteRegister(REG_CFG0, config.configReg0) != Status::Ok) {
			return Status::Error;
		}
	}

	if(config.configReg1 != 0) {
		// Configuration Register 1: 0xC0 0x00 0x01 0x00 0x00 0x01
		// Default: 0xFFC1
		// [15:7]  : Reserved (Default: 111111111b)
		// [6]     : Master clock type (Default: 1b)
		// [5]     : Hybrid sleep (Default: 0b)
		// [4:2]   : Partial array refresh (Default: 000b)
		// [1:0]   : Distributed refresh interval, read only (Default: 01b)
		// this->bus.Command(HyperBus::AddressSpace::Register, 0x01000004, HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_16, 4);
		// this->bus.Read(&data);
		if(this->WriteRegister(REG_CFG1, config.configReg1) != Status::Ok) {
			return Status::Error;
		}
	}

	return Status::Ok;
}

Status HyperRAM::ReadID(uint8_t *manID, uint8_t *devType) {
	// NOTE: Addressing is in words of 32-bits, but HyperBus uses byte addressing. Care has to be taken to shift addresses when required i.e. register addresses
	uint16_t regVal;

	// Identification Register 0: 0xC0 0x00 0x00 0x00 0x00 0x00
	// Return: 0x0E76
	// [15:14] : MCP Die address (Default: 00b)
	// [13]    : Reserved (Default: 0b)
	// [12:8]  : Row address bit count (Default: 01110b)
	// [7:4]   : Column address bit count (Default: 0111b)
	// [3:0]   : Manufacturer ID (Default: 0110b)
	if(this->ReadRegister(REG_ID0, &regVal) != Status::Ok) {
		return Status::Error;
	}
	*manID = (uint8_t)(regVal & 0x000F);

	// Identification Register 1: 0xC0 0x00 0x00 0x00 0x00 0x01
	// Return: 0x0009
	// [15:4] : Reserved (Default: 0)
	// [3:0]  : Device type (Default: 1001b)
	if(this->ReadRegister(REG_ID1, &regVal) != Status::Ok) {
		return Status::Error;
	}
	*devType = (uint8_t)(regVal & 0x000F);

	return Status::Ok;
}

Status HyperRAM::Read(uint32_t addr, uint8_t *buf, uint32_t len) {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	Status status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, addr, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_16, buf, len, true);
	if(status == Status::Ok) {
		status = this->bus.TransferWait(1000);
	}

	this->bus.UnlockBus();
	return status;
}

Status HyperRAM::Write(uint32_t addr, const uint8_t *buf, uint32_t len) {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	Status status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, addr, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_16, (uint8_t*)buf, len, false);
	if(status == Status::Ok) {
		status = this->bus.TransferWait(1000);
	}

	this->bus.UnlockBus();
	return status;
}

Status HyperRAM::ReadRegister(uint32_t wordAddr, uint16_t *value) {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	uint32_t regVal;

	Status status = this->bus.TransferAsync(HyperBus::AddressSpace::Register, wordAddr, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_16, (uint8_t*)&regVal, 4, true);
	if(status == Status::Ok) {
		status = this->bus.TransferWait(1000);
		if(status == Status::Ok) {
			*value = (uint16_t)(regVal & 0xFFFF);
		}
	}

	this->bus.UnlockBus();
	return status;
}

Status HyperRAM::WriteRegister(uint32_t wordAddr, uint16_t value) {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	uint32_t regVal = (uint32_t)value;

	Status status = this->bus.TransferAsync(HyperBus::AddressSpace::Register, wordAddr, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_16, (uint8_t*)&regVal, 4, false);
	if(status == Status::Ok) {
		status = this->bus.TransferWait(1000);
	}

	this->bus.UnlockBus();
	return status;
}

Status HyperRAM::EnterMemoryMappedMode() {
	return this->bus.EnterMemoryMappedMode();

	// uint32_t baseAddr = this->bus.GetBaseAddr();
	// SCB_InvalidateDCache_by_Addr((uint32_t*)baseAddr, this->config.sizeBytes);
}

Status HyperRAM::ExitMemoryMappedMode() {
	// uint32_t baseAddr = this->bus.GetBaseAddr();
	// SCB_CleanDCache_by_Addr((uint32_t*)baseAddr, this->config.sizeBytes);

	return this->bus.ExitMemoryMappedMode();
}
