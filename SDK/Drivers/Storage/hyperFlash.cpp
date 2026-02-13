/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Storage/hyperFlash.cpp
 */

#include "hyperFlash.hpp"
#include "status.hpp"

Status HyperFlash::Init(const Config &config) {
	if(config.frequencyHz == 0) {
		return Status::Error;
	}

	this->config = config;

	// Configure the Bus
	HyperBus::Config hyperBusCnfg;
	hyperBusCnfg.sizeBytes = config.sizeBytes;
	hyperBusCnfg.frequencyHz = config.frequencyHz;
	if(config.fixedLatency == true) {
		hyperBusCnfg.latencyMode = HyperBus::LatencyMode::Fixed;
	}
	else {
		hyperBusCnfg.latencyMode = HyperBus::LatencyMode::Variable;
	}
	hyperBusCnfg.initialLatency = config.initialLatency;
	hyperBusCnfg.rwRecoveryTime = config.rwRecoveryTime;
	hyperBusCnfg.writeZeroLatency = true;
	hyperBusCnfg.refreshRate = 0;

	// Initialize the Bus Hardware
	if(this->bus.Init(hyperBusCnfg) != Status::Ok) {
		return Status::Error;
	}

	// Clear flash status register
	// this->ClearStatus();

	// Reset Flash
	// this->Reset();

	// Verify ID
	uint16_t manID, devID;
	uint64_t uniqueID;
	this->ReadID(&manID, &devID, &uniqueID);

	if(config.expectedID != 0 && manID != config.expectedID) {
		return Status::Error;
	}

	if(config.expectedDeviceID != 0 && devID != config.expectedDeviceID) {
		return Status::Error;
	}

	return Status::Ok;
}

Status HyperFlash::Reset() {
	// Software Reset Command (0xF0)
	this->WriteCommand(0x00, 0x00F0); 
	return Status::Ok;
}

Status HyperFlash::ReadID(uint16_t *manufacturerID, uint16_t *familyID, uint64_t *uniqueID) {
	// NOTE: Addressing is in words of 16-bits, but HyperBus uses byte addressing. Care has to be taken to shift addresses when required i.e. register addresses
	uint8_t flashData[4];

	this->UnlockSequence();
	this->WriteCommand(static_cast<uint32_t>(ADDR_UNLOCK_1), 0x90);

	// this->WriteCommand(static_cast<uint32_t>(ADDR_UNLOCK_1), 0x98);

	// Manufacturer ID (Infineon: 0x0034)
	this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x800 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Memory, (0x800 << 1), HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, 2);
	// this->bus.Read(flashData);
	*manufacturerID = (flashData[1] << 8) | flashData[0];

	// Interface Voltage (HL-T: 0x006A, HS-T: 0x007B)
	// uint16_t interfaceVoltage;
	// this->bus.Command(HyperBus::AddressSpace::Memory, (0x801 << 1), HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, 2);
	// this->bus.Read(flashData);
	// interfaceVoltage = (flashData[1] << 8) | flashData[0];

	// Device Density (256Mb: 0x0019, 512Mb: 0x001A, 1024Mb: 0x001B)
	// uint16_t deviceDensity;
	// this->bus.Command(HyperBus::AddressSpace::Memory, (0x802 << 1), HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, 2);
	// this->bus.Read(flashData);
	// deviceDensity = (flashData[1] << 8) | flashData[0];

	// Device ID Length
	// uint16_t idLength;
	// this->bus.Command(HyperBus::AddressSpace::Memory, (0x803 << 1), HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, 2);
	// this->bus.Read(flashData);
	// idLength = (flashData[1] << 8) | flashData[0];

	// Family ID (HL-T/HS-T Family: 0x0090)
	this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x804 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Memory, (0x804 << 1), HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, 2);
	// this->bus.Read(flashData);
	*familyID = (flashData[1] << 8) | flashData[0];

	// Unique ID
	this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x200 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Memory, (0x200 << 1), HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, 2);
	// this->bus.Read(flashData);
	*uniqueID = (uint64_t)((uint64_t)flashData[1] << 8) | flashData[0];
	this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x201 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Memory, (0x201 << 1), HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, 2);
	// this->bus.Read(flashData);
	*uniqueID |= (uint64_t)((uint64_t)(flashData[1] << 8) | flashData[0]) << 16;
	this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x202 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Memory, (0x202 << 1), HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, 2);
	// this->bus.Read(flashData);
	*uniqueID |= (uint64_t)((uint64_t)(flashData[1] << 8) | flashData[0]) << 32;
	this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x203 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Memory, (0x203 << 1), HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, 2);
	// this->bus.Read(flashData);
	*uniqueID |= (uint64_t)((uint64_t)(flashData[1] << 8) | flashData[0]) << 48;

	this->WriteCommand(0x00, 0xF0);

	return Status::Ok;
}

Status HyperFlash::ReadJEDEC() {
	uint8_t flashData[4];

	this->UnlockSequence();
	this->WriteCommand(static_cast<uint32_t>(ADDR_UNLOCK_1), 0x90);

	//JEDEC SFDP Read
	this->bus.TransferAsync(HyperBus::AddressSpace::Memory, 0x00, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	this->bus.TransferWait(1000);
	this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x01 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, &flashData[2], 2, true);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Memory, 0x00, HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, 2);
	// this->bus.Read(flashData);
	// this->bus.Command(HyperBus::AddressSpace::Memory, (0x01 << 1), HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, 2);
	// this->bus.Read(&flashData[2]);

	this->WriteCommand(0x00, 0xF0);

	return Status::Ok;
}

Status HyperFlash::Read(uint32_t addr, uint8_t *buf, uint32_t len) {
	this->bus.TransferAsync(HyperBus::AddressSpace::Memory, addr, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, buf, len, true);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Memory, addr, HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, len);
	// this->bus.Read((uint8_t*)buf);
	return Status::Ok;
}

Status HyperFlash::Program(uint32_t addr, const uint8_t *buf, uint32_t len) {
	uint32_t curAdr = addr;
	const uint8_t *buffPtr = buf;
	uint32_t transCount = len;

	while(transCount > 0) {
		uint32_t pageOffset = curAdr % this->config.pageSize;
		uint32_t spaceInPage = this->config.pageSize - pageOffset;

		uint32_t chunkLen = (transCount < spaceInPage) ? transCount : spaceInPage;

		if(this->WritePage(curAdr, buffPtr, chunkLen) != Status::Ok) {
			// Something went wrong, clean up
			// this->ClearStatus();
			// this->Reset();
			return Status::Error;
		}

		curAdr += chunkLen;
		buffPtr += chunkLen;
		transCount -= chunkLen;
	}

	return Status::Ok;
}

Status HyperFlash::ChipErase() {
	this->UnlockSequence();
	this->WriteCommand(static_cast<uint32_t>(ADDR_UNLOCK_1), static_cast<uint16_t>(HyperFlash::Operation::Erase));

	this->UnlockSequence();
	this->WriteCommand(static_cast<uint32_t>(ADDR_UNLOCK_1), static_cast<uint16_t>(HyperFlash::Operation::ChipErase));

	//Wait for command complete (device not busy)
	//Maximum chip erase time is 1381 s for 1Gb flash (Typical is 398 s)
	if(this->WaitForReady(1500000) == false) {
		// Something went wrong, clean up
		// this->ClearStatus();
		// this->Reset();
		return Status::Timeout;
	}

	return Status::Ok;
}

Status HyperFlash::SectorErase(uint32_t sectorAddr) {
	this->UnlockSequence();
	this->WriteCommand(static_cast<uint32_t>(ADDR_UNLOCK_1), static_cast<uint16_t>(HyperFlash::Operation::Erase));
	
	this->UnlockSequence();
	this->WriteCommand(sectorAddr, static_cast<uint16_t>(HyperFlash::Operation::SectorErase));

	//Wait for command complete (device not busy)
	//Maximum sector erase time is 5869 ms (Typical is 773)
	if(this->WaitForReady(6000) == false) {
		// Something went wrong, clean up
		// this->ClearStatus();
		// this->Reset();
		return Status::Timeout;
	}

	return Status::Ok;
}

Status HyperFlash::WritePage(uint32_t pageAddr, const uint8_t *buf, uint32_t len) {
	if(len > this->config.pageSize) {
		return Status::Error;
	}

	if (len % 2 != 0) {
		return Status::Error;
	}

	uint32_t sectorAddr = pageAddr & ~(this->config.sectorSize - 1);
	uint16_t wordCnt = (len >> 1) - 1;

	// Write to Buffer Command	
	this->UnlockSequence();
	this->WriteCommand(sectorAddr, static_cast<uint16_t>(HyperFlash::Operation::WriteToBuffer));
	this->WriteCommand(sectorAddr, wordCnt);

	// Load buffer loop, used for flash that do not support burst write sequence i.e. the S26HS512T. 
	// Each transaction is one word with address.
	// Write must be within line (page) boundary
	for(uint32_t i = 0; i < len; i += 2) {		
		this->bus.TransferAsync(HyperBus::AddressSpace::Memory, pageAddr + i, HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, (uint8_t*)&buf[i], 2, false);
		this->bus.TransferWait(1000);
	}

	// Commit data
	this->WriteCommand(sectorAddr, static_cast<uint16_t>(HyperFlash::Operation::ProgBufferToFlash));

	// Wait for command complete (device not busy)
	// Maximum page program time is 2175 / 1700 (4KB vs 256KB sector), typical is 680 / 570 us
	if(this->WaitForReady(5) == false) {
		return Status::Error;
	}

	return Status::Ok;
}

void HyperFlash::EnterMemoryMappedMode() {
	//Ensure we are in read mode (TODO)
	this->bus.EnterMemoryMappedMode();
}

void HyperFlash::ExitMemoryMappedMode() {
	this->bus.ExitMemoryMappedMode();
}

void HyperFlash::WriteCommand(uint32_t addr, uint16_t data) {
	uint16_t cmdData = data;
	this->bus.TransferAsync(HyperBus::AddressSpace::Memory, addr, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, (uint8_t*)&cmdData, 2, false);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Memory, addr, HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, 2);
	// this->bus.Write(&data);
}

void HyperFlash::UnlockSequence() {
	// Flash unlock sequence: an often used initial write patterns
	this->WriteCommand(static_cast<uint32_t>(ADDR_UNLOCK_1), static_cast<uint16_t>(HyperFlash::Operation::Unlock_1));
	this->WriteCommand(static_cast<uint32_t>(ADDR_UNLOCK_2), static_cast<uint16_t>(HyperFlash::Operation::Unlock_2));
}

uint16_t HyperFlash::ReadStatus() {
	uint16_t status;
	this->WriteCommand(static_cast<uint32_t>(ADDR_UNLOCK_1), static_cast<uint16_t>(HyperFlash::Operation::StatusRegRead));
	this->bus.TransferAsync(HyperBus::AddressSpace::Memory, 0, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, (uint8_t*)&status, 2, true);
	this->bus.TransferWait(1000);
	// this->bus.Command(HyperBus::AddressSpace::Memory, 0, HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, 2);
	// this->bus.Read(&status);
	return status;
}

Status HyperFlash::ClearStatus() {
	this->WriteCommand(static_cast<uint32_t>(ADDR_UNLOCK_1), 0x0071);
    return Status::Ok;
}

bool HyperFlash::WaitForReady(uint32_t timeoutMs) {
	uint32_t timestamp = Time::GetMs();
	while(true) {
		uint16_t status = this->ReadStatus();

		// Check for errors: Erase error or program error
		if((status & FLASH_STATUS_ERSERR) == FLASH_STATUS_ERSERR || (status & FLASH_STATUS_PRGERR) == FLASH_STATUS_PRGERR) {
			// Reset to read mode (TODO)
			return false;
		}

		// Check for device ready
		if((status & FLASH_STATUS_RDYBSY) == FLASH_STATUS_RDYBSY) {
			return true;
		}

		if((Time::GetMs() - timestamp) > timeoutMs) {
			return false;
		}
	}
}