/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Storage/hyperRAM.cpp
 */

#include "hyperRAM.hpp"

Status HyperRAM::Init(const Config &config) {
	if(config.frequencyHz == 0) {
		return Status::Error;
	}

	this->config = config;

	// Configure the Bus
	HyperBus::Config hyperBusCnfg;
	hyperBusCnfg.sizeBytes = config.sizeBytes;
	hyperBusCnfg.frequencyHz = config.frequencyHz;
	if(config.fixedLatency == true) {
		// For 16-bit mode, "variable latency is not supported (..) LM in XSPI_HLCR must be set."
		hyperBusCnfg.latencyMode = HyperBus::LatencyMode::Fixed;
	}
	else {
		hyperBusCnfg.latencyMode = HyperBus::LatencyMode::Variable;
	}
	hyperBusCnfg.initialLatency = config.initalLatency;
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
	this->ReadID(&manID, &devType);

	if(config.expectedID != 0 && manID != config.expectedID) {
		return Status::Error;
	}

	return Status::Ok;
}

Status HyperRAM::ReadID(uint8_t *manufacturerID, uint8_t *devType) {
	// NOTE: Addressing is in words of 32-bits, but HyperBus uses byte addressing. Care has to be taken to shift addresses when required i.e. register addresses
	uint32_t data;

	// Identification Register 0: 0xC0 0x00 0x00 0x00 0x00 0x00
	// Return: 0x0E76
	// [15:14] : MCP Die address (Default: 00b)
	// [13]    : Reserved (Default: 0b)
	// [12:8]  : Row address bit count (Default: 01110b)
	// [7:4]   : Column address bit count (Default: 0111b)
	// [3:0]   : Manufacturer ID (Default: 0110b)
	this->bus.TransferAsync(HyperBus::AddressSpace::Register, REG_ID0, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_16, (uint8_t*)&data, 4, true);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Register, 0x00, HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_16, 4);
	// this->bus.Read(&data);
	*manufacturerID = (uint8_t)(data & 0x000F);

	// Identification Register 1: 0xC0 0x00 0x00 0x00 0x00 0x01
	// Return: 0x0009
	// [15:4] : Reserved (Default: 0)
	// [3:0]  : Device type (Default: 1001b)
	this->bus.TransferAsync(HyperBus::AddressSpace::Register, REG_ID1, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_16, (uint8_t*)&data, 4, true);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Register, 0x04, HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_16, 4);
	// this->bus.Read(&data);
	*devType = (uint8_t)(data & 0x000F);

	// Configuration Register 0: 0xC0 0x00 0x01 0x00 0x00 0x00
	// Default: 0x112F
	// [15]    : Deep power down (Default: 0b)
	// [14:12] : Drive strength (Default: 000b)
	// [11:8]  : Reserved (Default: 1111b)
	// [7:4]   : Initial latency (Default: 0010b)
	// [3]     : Fixed latency enable (Default: 1b)
	// [2]     : Hybrid burst enable (Default: 1b)
	// [1:0]   : Burst length (Default: 11b)
	this->bus.TransferAsync(HyperBus::AddressSpace::Register, REG_CFG0, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_16, (uint8_t*)&data, 4, true);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Register, 0x01000000, HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_16, 4);
	// this->bus.Read(&data);
	// volatile uint16_t cnfgReg0 = data;

	// Configuration Register 1: 0xC0 0x00 0x01 0x00 0x00 0x01
	// Default: 0xFFC1
	// [15:7]  : Reserved (Default: 111111111b)
	// [6]     : Master clock type (Default: 1b)
	// [5]     : Hybrid sleep (Default: 0b)
	// [4:2]   : Partial array refresh (Default: 000b)
	// [1:0]   : Distributed refresh interval, read only (Default: 01b)
	// this->bus.Command(HyperBus::AddressSpace::Register, 0x01000004, HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_16, 4);
	// this->bus.Read(&data);
	this->bus.TransferAsync(HyperBus::AddressSpace::Register, REG_CFG1, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_16, (uint8_t*)&data, 4, true);
	this->bus.TransferWait(1000);
	// volatile uint16_t cnfgReg1 = data;

	return Status::Ok;
}

Status HyperRAM::Read(uint32_t addr, uint8_t *buf, uint32_t len) {
	this->bus.TransferAsync(HyperBus::AddressSpace::Memory, addr, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_16, buf, len, true);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Memory, addr, HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_16, len);
	// this->bus.Read(buf);
	return Status::Ok;
}

Status HyperRAM::Write(uint32_t addr, const uint8_t *buf, uint32_t len) {
	this->bus.TransferAsync(HyperBus::AddressSpace::Memory, addr, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_16, (uint8_t*)buf, len, false);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Memory, addr, HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_16, len);
	// this->bus.Write(buf);
	return Status::Ok;
}

void HyperRAM::EnterMemoryMappedMode() {
	this->bus.EnterMemoryMappedMode();
}

void HyperRAM::ExitMemoryMappedMode() {
	this->bus.ExitMemoryMappedMode();
}
