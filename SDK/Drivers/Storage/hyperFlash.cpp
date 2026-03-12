/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Storage/hyperFlash.cpp
 */

#include "hyperFlash.hpp"

Status HyperFlash::Init(const Config &config) {
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

	// Set Flash Configs
	if(config.configReg0 != 0) {
		if(this->WriteRegister(REG_CFG0, config.configReg0) != Status::Ok) {
			return Status::Error;
		}
	}

	if(config.configReg1 != 0) {
		if(this->WriteRegister(REG_CFG1, config.configReg1) != Status::Ok) {
			return Status::Error;
		}
	}

	return Status::Ok;
}

Status HyperFlash::Reset() {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	// Software Reset Command (0xF0)
	Status status = this->WriteCommand(0x00, 0x00F0);

	this->bus.UnlockBus();
	return status;
}

Status HyperFlash::ReadID(uint16_t *manID, uint16_t *famID, uint64_t *uniqueID) {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	// NOTE: Addressing is in words of 16-bits, but HyperBus uses byte addressing. Care has to be taken to shift addresses when required i.e. register addresses
	uint8_t flashData[4];

	Status status = this->UnlockSequence();
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->WriteCommand(static_cast<uint32_t>(CMD_ADDR_UNLOCK_1), 0x90);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	// this->WriteCommand(static_cast<uint32_t>(CMD_ADDR_UNLOCK_1), 0x98);

	// Manufacturer ID (Infineon: 0x0034)
	status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x800 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->bus.TransferWait(1000);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}
	*manID = (flashData[1] << 8) | flashData[0];

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
	status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x804 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->bus.TransferWait(1000);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}
	*famID = (flashData[1] << 8) | flashData[0];

	// Unique ID
	status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x200 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->bus.TransferWait(1000);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}
	*uniqueID = (uint64_t)((uint64_t)flashData[1] << 8) | flashData[0];

	status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x201 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->bus.TransferWait(1000);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}
	*uniqueID |= (uint64_t)((uint64_t)(flashData[1] << 8) | flashData[0]) << 16;

	status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x202 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->bus.TransferWait(1000);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}
	*uniqueID |= (uint64_t)((uint64_t)(flashData[1] << 8) | flashData[0]) << 32;

	status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x203 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->bus.TransferWait(1000);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}
	*uniqueID |= (uint64_t)((uint64_t)(flashData[1] << 8) | flashData[0]) << 48;

	status = this->WriteCommand(0x00, 0xF0);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	this->bus.UnlockBus();
	return Status::Ok;
}

Status HyperFlash::ReadJEDEC() {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	uint8_t flashData[4];

	Status status = this->UnlockSequence();
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->WriteCommand(static_cast<uint32_t>(CMD_ADDR_UNLOCK_1), 0x90);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	//JEDEC SFDP Read
	status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, 0x00, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, flashData, 2, true);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->bus.TransferWait(1000);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, (0x01 << 1), HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, &flashData[2], 2, true);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->bus.TransferWait(1000);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->WriteCommand(0x00, 0xF0);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	this->bus.UnlockBus();
	return Status::Ok;
}

Status HyperFlash::Read(uint32_t addr, uint8_t *buf, uint32_t len) {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	Status status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, addr, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, buf, len, true);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->bus.TransferWait(1000);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	this->bus.UnlockBus();
	return Status::Ok;
}

Status HyperFlash::Program(uint32_t addr, const uint8_t *buf, uint32_t len) {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

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
			this->bus.UnlockBus();
			return Status::Error;
		}

		curAdr += chunkLen;
		buffPtr += chunkLen;
		transCount -= chunkLen;
	}

	this->bus.UnlockBus();
	return Status::Ok;
}

Status HyperFlash::ChipErase() {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	Status status = this->UnlockSequence();
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->WriteCommand(static_cast<uint32_t>(CMD_ADDR_UNLOCK_1), static_cast<uint16_t>(CMD_DATA_ERASE));
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->UnlockSequence();
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->WriteCommand(static_cast<uint32_t>(CMD_ADDR_UNLOCK_1), static_cast<uint16_t>(CMD_DATA_CHIP_ERASE));
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	//Wait for command complete (device not busy)
	//Maximum chip erase time is 1381 s for 1Gb flash (Typical is 398 s)
	if(this->WaitForReady(1500000) == false) {
		// Something went wrong, clean up
		// this->ClearStatus();
		// this->Reset();
		this->bus.UnlockBus();
		return Status::Timeout;
	}

	this->bus.UnlockBus();
	return Status::Ok;
}

Status HyperFlash::SectorErase(uint32_t sectorAddr) {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	Status status = this->UnlockSequence();
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status =this->WriteCommand(static_cast<uint32_t>(CMD_ADDR_UNLOCK_1), static_cast<uint16_t>(CMD_DATA_ERASE));
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}
	
	status =this->UnlockSequence();
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status =this->WriteCommand(sectorAddr, static_cast<uint16_t>(CMD_DATA_SECTOR_ERASE));
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	//Wait for command complete (device not busy)
	//Maximum sector erase time is 5869 ms (Typical is 773)
	if(this->WaitForReady(6000) == false) {
		// Something went wrong, clean up
		// this->ClearStatus();
		// this->Reset();
		this->bus.UnlockBus();
		return Status::Timeout;
	}

	this->bus.UnlockBus();

	return Status::Ok;
}

Status HyperFlash::WritePage(uint32_t pageAddr, const uint8_t *buf, uint32_t len) {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	if(len > this->config.pageSize) {
		this->bus.UnlockBus();
		return Status::Error;
	}

	if (len % 2 != 0) {
		this->bus.UnlockBus();
		return Status::Error;
	}

	uint32_t sectorAddr = pageAddr & ~(this->config.sectorSize - 1);
	uint16_t wordCnt = (len >> 1) - 1;

	// Write to Buffer Command	
	Status status = this->UnlockSequence();
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->WriteCommand(sectorAddr, static_cast<uint16_t>(CMD_DATA_WRITE_BUFFER));
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->WriteCommand(sectorAddr, wordCnt);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	// Load buffer loop, used for flash that do not support burst write sequence i.e. the S26HS512T. 
	// Each transaction is one word with address.
	// Write must be within line (page) boundary
	for(uint32_t i = 0; i < len; i += 2) {		
		status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, pageAddr + i, HyperBus::AddrSize::Width_32, HyperBus::BusWidth::Lines_8, (uint8_t*)&buf[i], 2, false);
		if (status != Status::Ok) {
			this->bus.UnlockBus();
			return status;
		}
		this->bus.TransferWait(1000);
		if (status != Status::Ok) {
			this->bus.UnlockBus();
			return status;
		}
	}

	// Commit data
	status = this->WriteCommand(sectorAddr, static_cast<uint16_t>(CMD_DATA_PROG_BUFFER));
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	// Wait for command complete (device not busy)
	// Maximum page program time is 2175 / 1700 (4KB vs 256KB sector), typical is 680 / 570 us
	if(this->WaitForReady(5) == false) {
		this->bus.UnlockBus();
		return Status::Error;
	}

	this->bus.UnlockBus();
	return Status::Ok;
}

Status HyperFlash::ReadRegister(uint32_t wordAddr, uint16_t *value) {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	Status status = this->UnlockSequence();
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->WriteCommand(static_cast<uint32_t>(CMD_ADDR_UNLOCK_1), 0xC7);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, wordAddr, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, (uint8_t*)value, 2, true);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->bus.TransferWait(1000);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	this->bus.UnlockBus();
	return Status::Ok;
}

Status HyperFlash::WriteRegister(uint32_t wordAddr, uint16_t value) {
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	Status status = this->UnlockSequence();
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->WriteCommand(static_cast<uint32_t>(CMD_ADDR_UNLOCK_1), 0x38);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status= this->bus.TransferAsync(HyperBus::AddressSpace::Memory, wordAddr, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, (uint8_t*)&value, 2, false);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	status = this->bus.TransferWait(1000);
	if (status != Status::Ok) {
		this->bus.UnlockBus();
		return status;
	}

	this->bus.UnlockBus();
	return Status::Ok;
}

Status HyperFlash::EnterMemoryMappedMode() {
	//Ensure we are in read mode
	if(this->bus.LockBus(1000) != Status::Ok) {
		return Status::Timeout;
	}

	Status status = this->WriteCommand(0x00, 0x00F0); // Software Reset / Return to Read

	this->bus.UnlockBus();

	if (status != Status::Ok) {
		return status;
	}	

	status = this->bus.EnterMemoryMappedMode();
	if (status != Status::Ok) {
		return status;
	}

	// uint32_t baseAddr = this->bus.GetBaseAddr();
	// SCB_InvalidateDCache_by_Addr((uint32_t*)baseAddr, this->config.sizeBytes);

	return Status::Ok;
}

Status HyperFlash::ExitMemoryMappedMode() {
	// uint32_t baseAddr = this->bus.GetBaseAddr();
	// SCB_CleanDCache_by_Addr((uint32_t*)baseAddr, this->config.sizeBytes);

	return this->bus.ExitMemoryMappedMode();
}

Status HyperFlash::WriteCommand(uint32_t addr, uint16_t data) {
	uint16_t cmdData = data;
	Status status = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, addr, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, (uint8_t*)&cmdData, 2, false);
	if(status == Status::Ok) {
		status = this->bus.TransferWait(1000);
	}
	return status;
}

Status HyperFlash::UnlockSequence() {
	// Flash unlock sequence: an often used initial write patterns
	Status status = this->WriteCommand(static_cast<uint32_t>(CMD_ADDR_UNLOCK_1), static_cast<uint16_t>(CMD_DATA_UNLOCK_1));
	if(status == Status::Ok) {
		status = this->WriteCommand(static_cast<uint32_t>(CMD_ADDR_UNLOCK_2), static_cast<uint16_t>(CMD_DATA_UNLOCK_2));
	}
	return status;
}

Status HyperFlash::ReadStatus(uint16_t &status) {
	Status stat = this->WriteCommand(static_cast<uint32_t>(CMD_ADDR_UNLOCK_1), static_cast<uint16_t>(CMD_DATA_STATUS_REG_READ));
	if(stat != Status::Ok) {
		return stat;
	}
	stat = this->bus.TransferAsync(HyperBus::AddressSpace::Memory, 0, HyperBus::AddrSize::Width_32,  HyperBus::BusWidth::Lines_8, (uint8_t*)&status, 2, true);
	if(stat != Status::Ok) {
		return stat;
	}
	return this->bus.TransferWait(1000);
}

Status HyperFlash::ClearStatus() {
	return this->WriteCommand(static_cast<uint32_t>(CMD_ADDR_UNLOCK_1), 0x0071);
}

bool HyperFlash::WaitForReady(uint32_t timeoutMs) {
	Status status;
	uint16_t stat;
	uint32_t timestamp = Time::GetMs();
	while(true) {
		status = this->ReadStatus(stat);
		if(status != Status::Ok) {
			return false;
		}

		// Check for errors: Erase error or program error
		if((stat & STATUS_ERSERR) == STATUS_ERSERR || (stat & STATUS_PRGERR) == STATUS_PRGERR) {
			// Reset to read mode (TODO)
			return false;
		}

		// Check for device ready
		if((stat & STATUS_RDYBSY) == STATUS_RDYBSY) {
			return true;
		}

		if((Time::GetMs() - timestamp) > timeoutMs) {
			return false;
		}

		tx_thread_sleep(1);
	}
}