/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	SDK/Drivers/Storage/sd.cpp
 */

#include "sd.hpp"

SD::SD(SDMMC &sdmmc) : bus(sdmmc) {
	this->isInitialized = false;
	this->RCA = 0;
	// Defaults
	this->cardInfo.blockSize = 512;
}

Status SD::Init(const Config &config) {
	this->config = config;

	// Configure the Bus
	SDMMC::Config busConfig;
	busConfig.hwFlowControl = true;		//< Mandatory or will fail in write transfers as FIFO is empty at instance of command so underflow error occurs
	if(this->bus.Init(busConfig) == Status::Error) {
		return Status::Error;
	}

	// Send card to idle
	SDMMC::CommandResponse resp;
	resp = bus.Command(static_cast<uint8_t>(Command::GoIdleState), 0, SDMMC::ResponseType::None);
	if(resp.error != SDMMC::Error::None) {
		this->Reset();
		return Status::Error;
	}

	// Send operating conditions (only works on V2.0 cards)
	bool isV2Card = false;
	resp = bus.Command(static_cast<uint8_t>(Command::SendIfCond), 0x1AA, SDMMC::ResponseType::Short_CRC);
	if(resp.error != SDMMC::Error::None) {
		// No response to CMD8 -> Is not V2.0 card
		isV2Card = false;
		// Send card to idle
		resp = bus.Command(static_cast<uint8_t>(Command::GoIdleState), 0, SDMMC::ResponseType::None);
		if(resp.error != SDMMC::Error::None) {
			this->Reset();
			return Status::Error;
		}
	}
	else if((resp.resp[0] & 0xFF) == 0xAA) {
		// Valid response -> Is V2.0 card
		isV2Card = true;
	}
	else {
		// Invalid response, volage miss match??
		this->Reset();
		return Status::Error;
	}

	if(isV2Card == false) {
		this->Reset();
		return Status::Error;
	}

	// Negotiate operating conditions
	bool validVoltage = false;
	uint32_t retries = 1000;
	while(retries > 0) {
		// Prepare arguments for operating conditions App Command:
		// Bit 30 (HCS): 1 if we support High Capacity (SDHC/SDXC), 0 otherwise.
		// Bit 24 (S18R): Request 1.8V Switching (If configured)
		// Bit 20-21: Voltage Window (3.2-3.4V) -> Set bits 20 and 21 to 1? usually 0x00100000 covers 3.2-3.3V
		// Standard Arg: 0x40100000 (HCS=1, 3.2V-3.3V)
		uint32_t arg = 0x00100000U;	// 3.2-3.3V
		if(isV2Card == true) {
			arg |= 0x40000000; // HCS
		}
		if(config.useUHS == true) {
			arg |= 0x01000000; // S18R (Request 1.8V)
		}
		resp = SendAppCommand(AppCommand::SdAppOpCond, arg, SDMMC::ResponseType::Short_NoCRC);
		if(resp.error == SDMMC::Error::None) {
			// Get operating voltage
			if(((resp.resp[0] >> 31) & 0x01) == 0x01) {
				validVoltage = true;
				break;
			}
		}
		retries--;
	}

	if(validVoltage == false) {
		this->Reset();
		return Status::Error;
	}

	// Check/parse capacity type
	if((isV2Card == true) && ((resp.resp[0] & 0x40000000U) == 0x40000000U)) {
		this->cardInfo.highCapacity = true;
	}
	else {
		this->cardInfo.highCapacity = false;
	}

	// Voltage switch, if negotiated
	bool is1v8Mode = false;
	if(config.use1V8Level == true && ((resp.resp[0] & 0x01000000U) == 0x01000000U)) {
		// Voltage switch
		if(bus.SwitchTo1V8(&SD::VoltageSwitchCallback, this->config.vioSelectPin) == Status::Ok) {
			is1v8Mode = true;
		}
		else {
			this->Reset();
			return Status::Error;
		}
	}

	// Get CID
	resp = bus.Command(static_cast<uint8_t>(Command::AllSendCid), 0, SDMMC::ResponseType::Long);
	if(resp.error != SDMMC::Error::None) {
		this->Reset();
		return Status::Error;
	}
	ParseCID(resp.resp);

	// Get RCA
	resp = bus.Command(static_cast<uint8_t>(Command::SendRelativeAddr), 0, SDMMC::ResponseType::Short_CRC);
	if(resp.error != SDMMC::Error::None) {
		this->Reset();
		return Status::Error;
	}
	this->RCA = (uint16_t)(resp.resp[0] >> 16);

	// Get CSD
	resp = bus.Command(static_cast<uint8_t>(Command::SendCsd), (uint32_t)(this->RCA << 16), SDMMC::ResponseType::Long);
	if(resp.error != SDMMC::Error::None) {
		this->Reset();
		return Status::Error;
	}
	ParseCSD(resp.resp);

	// Get the card class
	// this->cardInfo.Class = resp.resp[1] >> 20;

	// Select card
	resp = bus.Command(static_cast<uint8_t>(Command::SelectCard), (uint32_t)(this->RCA << 16), SDMMC::ResponseType::Short_CRC);
	if(resp.error != SDMMC::Error::None) {
		this->Reset();
		return Status::Error;
	}

	// Get capabilites
	if(GetSCR() == false) {
		this->Reset();
		return Status::Error;
	}

	// Get status
	if(GetSDStatus() == false) {
		this->Reset();
		return Status::Error;
	}

	// Set bus width
	if(config.use4BitMode == true && this->cardInfo.support4Bit == true) {
		resp = SendAppCommand(AppCommand::SetBusWidth, 2, SDMMC::ResponseType::Short_CRC);
		if(resp.error == SDMMC::Error::None) {
			bus.SetBusWidth(SDMMC::BusWidth::Lines_4);
			this->cardInfo.currentBusWidth = 0x02;
		}
	}

	// Set speed
	SDMMC::BusSpeed targetSpeed = SDMMC::BusSpeed::Default;
	uint32_t targetFreq = 25000000;

	if(is1v8Mode == true) {
		// UHS (1.8V) speed settings
		// Basic UHS speed mode (SDR25)
		targetSpeed = SDMMC::BusSpeed::UHS_SDR25;
		targetFreq  = 50000000;

		// Higher Speeds (SDR50/104) require Tuning (CMD19)
		// TODO
	}
	else {
		// Legacy (3.3V) speed settings
		// Check for High Speed Support (Spec v1.1+)
		if(config.useHighSpeed && this->cardInfo.specVersion >= 1) {
			targetSpeed = SDMMC::BusSpeed::HighSpeed;
			targetFreq  = 50000000;
		}
	}

	// Apply speed settings
	if(targetSpeed != SDMMC::BusSpeed::Default) {
		if(ChangeSpeedMode(targetSpeed) == true) {
			// Success: Increase Clock
			bus.SetClock(targetFreq);
			this->cardInfo.activeMode = targetSpeed;
		}
		else {
			// Fail: Fallback to Default
			bus.SetClock(25000000);
			this->cardInfo.activeMode = SDMMC::BusSpeed::Default;
		}
	}
	else {
		bus.SetClock(25000000);
		this->cardInfo.activeMode = SDMMC::BusSpeed::Default;
	}

	// Set block length
	if(this->cardInfo.highCapacity == false) {
		bus.Command(static_cast<uint8_t>(Command::SetBlockLen), 512, SDMMC::ResponseType::Short_CRC);
	}

	this->isInitialized = true;
	return Status::Ok;
}

Status SD::Reset(void) {
	// Hard reset the SDMMC bus
	bus.DeInit();

	// Clear card info
	memset(&this->cardInfo, 0x00, sizeof(CardInfo));
	this->RCA = 0;
	this->isInitialized = false;

	// Restore signal levels!
	if(this->config.vioSelectPin != nullptr) {
		this->config.vioSelectPin->Write(0);
	}

	// Some delay
	tx_thread_sleep(20);

	return Status::Ok;
}

Status SD::ReadBlocks(uint32_t lba, uint8_t *buf, uint32_t blockCount) {
	if(this->isInitialized == false) {
		return Status::Error;
	}

	//Alignment check, for SDMMC DMA cache stuff
	if(((uint32_t)(buf) & 0x1F) != 0) {
		return Status::Error;
	}

	// Wait for card to be ready
	if(WaitCardBusy(100) == false) {
		return Status::Timeout;
	}

	// Calculate address
	uint32_t addr = lba;
	if(this->cardInfo.highCapacity == false) {
		// SDSC uses Byte Addressing
		addr = addr * 512;
	}

	// Prepare data transfer
	Status status = this->bus.TransferAsync(buf, blockCount * 512, 512, true);
	if(status != Status::Ok) {
		return status;
	}

	// Send command
	SDMMC::CommandResponse resp;
	if(blockCount == 1) {
		resp = bus.Command(static_cast<uint8_t>(Command::ReadSingleBlock), addr, SDMMC::ResponseType::Short_CRC, SDMMC::TransferMode::Auto);
	}
	else {
		resp = bus.Command(static_cast<uint8_t>(Command::ReadMultiBlock), addr, SDMMC::ResponseType::Short_CRC, SDMMC::TransferMode::Auto);
	}

	if(resp.error != SDMMC::Error::None) {
		return Status::Error;
	}

	// Wait for data
	status = bus.TransferWait(1000 * blockCount);
	if(status != Status::Ok) {
		return status;
	}

	// Stop transmission if was multi-block read
	if(blockCount > 1) {
		resp = bus.Command(static_cast<uint8_t>(Command::StopTransmission), 0, SDMMC::ResponseType::Short_CRC);
		if(resp.error != SDMMC::Error::None) {
			return Status::Error;
		}
	}
	
	return Status::Ok;
}

Status SD::WriteBlocks(uint32_t lba, const uint8_t *buf, uint32_t blockCount) {
	if(this->isInitialized == false) {
		return Status::Error;
	}

	//Alignment check, for SDMMC DMA cache stuff
	if(((uint32_t)(buf) & 0x1F) != 0) {
		return Status::Error;
	}

	// Wait for card to be ready
	if(WaitCardBusy(100) == false) {
		return Status::Timeout;
	}

	// Calculate address
	uint32_t addr = lba;
	if(this->cardInfo.highCapacity == false) {
		// SDSC uses Byte Addressing
		addr = addr * 512;
	}

	// Optional if supported send ACMD23 for faster multi-block write
	// SDMMC::CommandResponse resp;
	// if(blockCount > 1 && this->cardInfo.supportCmd23 == true) {
	// 	resp = SendAppCommand(AppCommand::SetWrBlkEraseCount, blockCount, SDMMC::ResponseType::Short_CRC);
	// 	if(resp.error != 0) {
	// 		return false;
	// 	}
	// }

	// Prepare data transfer´
	Status status = this->bus.TransferAsync((uint8_t*)buf, blockCount * 512, 512, false);
	if(status != Status::Ok) {
		return status;
	}

	// Send command
	SDMMC::CommandResponse resp;
	if(blockCount == 1) {
		resp = bus.Command(static_cast<uint8_t>(Command::WriteSingleBlock), addr, SDMMC::ResponseType::Short_CRC, SDMMC::TransferMode::Auto);
	}
	else {
		resp = bus.Command(static_cast<uint8_t>(Command::WriteMultiBlock), addr, SDMMC::ResponseType::Short_CRC, SDMMC::TransferMode::Auto);
	}

	if(resp.error != SDMMC::Error::None) {
		return Status::Error;
	}

	// Wait for data transmission
	status = bus.TransferWait(1000 * blockCount);
	if(status != Status::Ok) {
		return status;
	}

	// Stop transmission if was multi-block write
	if(blockCount > 1) {
		resp = bus.Command(static_cast<uint8_t>(Command::StopTransmission), 0, SDMMC::ResponseType::Short_CRC);
		if(resp.error != SDMMC::Error::None) {
			return Status::Error;
		}
	}

	// Wait for programming to finish
	status = this->bus.WaitBusyD0(1000);
	if(status != Status::Ok) {
		return status;
	}
	// if(WaitCardBusy(1000) == false) {
	// 	return false;
	// }

	return Status::Ok;
}

Status SD::Erase(uint32_t startAddr, uint32_t endAddr) {
	if(this->isInitialized == false) {
		return Status::Error;
	}

	// Wait for card to be ready
	if(WaitCardBusy(100) == false) {
		return Status::Timeout;
	}

	// Calculate address
	if(this->cardInfo.highCapacity == false) {
		// SDSC uses Byte Addressing
		startAddr = startAddr * 512;
		endAddr = endAddr * 512;
	}

	SDMMC::CommandResponse resp;

	// Set start block
	resp = bus.Command(static_cast<uint8_t>(Command::EraseWrBlkStart), startAddr, SDMMC::ResponseType::Short_CRC);
	if(resp.error != SDMMC::Error::None) {
		return Status::Error;
	}

	// Set end block
	resp = bus.Command(static_cast<uint8_t>(Command::EraseWrBlkEnd), endAddr, SDMMC::ResponseType::Short_CRC);
	if(resp.error != SDMMC::Error::None) {
		return Status::Error;
	}

	// Send command
	resp = bus.Command(static_cast<uint8_t>(Command::Erase), 0, SDMMC::ResponseType::Short_CRC);
	if(resp.error != SDMMC::Error::None) {
		return Status::Error;
	}

	// Wait for erase to finish
	Status status = this->bus.WaitBusyD0(1000);
	if(status != Status::Ok) {
		return status;
	}
	// if(WaitCardBusy(30000) == false) {
	// 	return false;
	// }

	return Status::Ok;
}

SD::CardState SD::GetCardState(void) {
	SDMMC::CommandResponse resp;

	// Send CMD13, get card status register
	resp = bus.Command(static_cast<uint8_t>(Command::SendStatus), (uint32_t)(this->RCA << 16), SDMMC::ResponseType::Short_CRC);
	if(resp.error != SDMMC::Error::None) {
		return CardState::Error;
	}

	// Status Register Response (R1)
    // Bits 12-9 represent CURRENT_STATE
	uint8_t state = (resp.resp[0] >> 9) & 0x0F;

	return static_cast<CardState>(state);
}

bool SD::IsCardBusy(void) {
	CardState state = GetCardState();

	if(state == CardState::Error) {
		return true;
	}

	if(state == CardState::Programming || state == CardState::Receive || state == CardState::Data) {
		return true;
	}

	return false;
}

bool SD::WaitCardBusy(uint32_t timeoutMs) {
	uint32_t timestamp = Time::GetMs();
	while(true) {
		if (!IsCardBusy()) {
			return true;
		}
		if((Time::GetMs() - timestamp) > timeoutMs) {
			return false;
		}
		// Yield to RTOS (this can be limiting with throughput due to added latency of min. 1ms between checks)
		// tx_thread_sleep(1);
	}
}

SDMMC::CommandResponse SD::SendAppCommand(AppCommand acmd, uint32_t arg, SDMMC::ResponseType respType) {
	SDMMC::CommandResponse resp;

	// Send CMD55 with card's RCA
	resp = bus.Command(static_cast<uint8_t>(Command::AppCmd), (uint32_t)(this->RCA << 16), SDMMC::ResponseType::Short_CRC);
	if(resp.error != SDMMC::Error::None) {
		return resp;
	}

	// Send actual app command (ACMD)
	resp = bus.Command(static_cast<uint8_t>(acmd), arg, respType);
	return resp;
}

void SD::ParseCID(uint32_t *cidRaw) {
	// STM32 Response Register Mapping:
	// cidRaw[0] = Bits 127:96
	// cidRaw[1] = Bits 95:64
	// cidRaw[2] = Bits 63:32
	// cidRaw[3] = Bits 31:0

	// Manufacturer ID (MID) - Bits 127:120 -> cidRaw[0] >> 24
	this->cardInfo.manufacturerID = (uint8_t)((cidRaw[0] >> 24) & 0xFF);

	// OEM/Application ID (OID) - Bits 119:104 -> cidRaw[0] >> 8
	this->cardInfo.oemID = (uint16_t)((cidRaw[0] >> 8) & 0xFFFF);

	// Product Name (PNM) - 5 ASCII Characters (Bits 103:64)
	// Bits 103:96 are in cidRaw[0] (Lowest 8 bits)
	// Bits 95:64  are in cidRaw[1] (All 32 bits)
	this->cardInfo.cardName[0] = (char)(cidRaw[0] & 0xFF);
	this->cardInfo.cardName[1] = (char)((cidRaw[1] >> 24) & 0xFF);
	this->cardInfo.cardName[2] = (char)((cidRaw[1] >> 16) & 0xFF);
	this->cardInfo.cardName[3] = (char)((cidRaw[1] >> 8) & 0xFF);
	this->cardInfo.cardName[4] = (char)(cidRaw[1] & 0xFF);
	this->cardInfo.cardName[5] = '\0';

	// Product Revision (PRV) - Bits 63:56 -> cidRaw[2] >> 24
	this->cardInfo.revision = (uint8_t)((cidRaw[2] >> 24) & 0xFF);

	// Product Serial Number (PSN) - Bits 55:24 -> cidRaw[2] & raw[3]
	this->cardInfo.serialNumber = ((cidRaw[2] & 0x00FFFFFF) << 8) | ((cidRaw[3] >> 24) & 0xFF);

	// Manufacturing Date (MDT) - 12 bits [19:8]
	// Format: 8 bits Year (Offset from 2000), 4 bits Month
	uint16_t mdt = (cidRaw[3] >> 8) & 0xFFF;
	this->cardInfo.mfgYear  = 2000 + ((mdt >> 4) & 0xFF);
	this->cardInfo.mfgMonth = (mdt & 0x0F);
}

void SD::ParseCSD(uint32_t *csdRaw) {
	// CSD Structure (Bits 127:126) -> cidRaw[0] >> 30
	uint32_t csdStruct = (csdRaw[0] >> 30) & 0x03;

	// Default Block Size
	this->cardInfo.blockSize = 512;

	if(csdStruct == 1) { 
		// --- CSD Version 2.0 (High Capacity) ---
		// C_SIZE is 22 bits [69:48]
		// Bits 69:64 are in csdRaw[1] (RESP2) [5:0]
		// Bits 63:48 are in csdRaw[2] (RESP3) [31:16]
		uint32_t cSize = ((csdRaw[1] & 0x0000003F) << 16) | ((csdRaw[2] >> 16) & 0xFFFF);
		
		// Memory Capacity = (C_SIZE+1) * 512KByte
		// Block Count (512B blocks) = (C_SIZE+1) * 1024
		this->cardInfo.blockCount = (cSize + 1) * 1024;
		this->cardInfo.sizeBytes = (uint64_t)this->cardInfo.blockCount * 512;
	} 
	else {
		// --- CSD Version 1.0 (Standard Capacity) ---
		// C_SIZE [73:62] (12 bits)
		// C_SIZE_MULT [49:47] (3 bits)
		// READ_BL_LEN [83:80] (4 bits)
		
		// C_SIZE: Bits 73:64 in csdRaw[1][9:0], Bits 63:62 in csdRaw[2][31:30]
		uint32_t cSize = ((csdRaw[1] & 0x3FF) << 2) | ((csdRaw[2] >> 30) & 0x03);
		uint32_t cSizeMult = (csdRaw[2] >> 15) & 0x07;
		uint32_t readBlockLen = (csdRaw[1] >> 16) & 0x0F;

		uint32_t mult = 1 << (cSizeMult + 2);
		uint32_t blockLen = 1 << readBlockLen;
		
		uint64_t capacityBytes = (uint64_t)(cSize + 1) * mult * blockLen;
		this->cardInfo.sizeBytes = capacityBytes;
		this->cardInfo.blockCount = (uint32_t)(capacityBytes / 512);
	}
}

bool SD::GetSCR(void) {
	SDMMC::CommandResponse resp;
	memset(this->statusBuf, 0, sizeof(this->statusBuf));

	// Send CMD55 with card's RCA
	resp = bus.Command(static_cast<uint8_t>(Command::AppCmd), (uint32_t)(this->RCA << 16), SDMMC::ResponseType::Short_CRC);
	if(resp.error != SDMMC::Error::None) {
		return false;
	}

	// Prepare data
	if(bus.TransferAsync((uint8_t*)this->statusBuf, 8, 8, true) != Status::Ok) {
		return false;
	}

	// Send ACMD51
	resp = bus.Command(static_cast<uint8_t>(AppCommand::SendScr), 0, SDMMC::ResponseType::Short_CRC, SDMMC::TransferMode::Manual);
	if(resp.error != SDMMC::Error::None) {
		return false;
	}

	// Wait for data transfer
	if(bus.TransferWait(100) != Status::Ok) {
		return false;
	}

	// Parse information
	uint8_t* pScr = (uint8_t*)this->statusBuf;

	// Byte 0: [63:56] Structure, Spec
	this->cardInfo.specVersion  = pScr[0] & 0x0F;

	// Byte 1: [55:48] ... Bus Widths
	// Bit 0 if support 1-Bit width, Bit 2 if support 4-Bit width
	this->cardInfo.support4Bit = false;
	if((pScr[1] & 0x04) == 0x04) {
		this->cardInfo.support4Bit = true;
	}

	// Byte 3: [39:32] ... Cmd23
	// Cmd23 is bit 33 of SCR, which is bit 1 of Byte 3
	this->cardInfo.supportCmd23 = false;
	if((pScr[3] & 0x02) == 0x02) {
		this->cardInfo.supportCmd23 = true;
	}

	return true;
}

bool SD::GetSDStatus(void) {
	SDMMC::CommandResponse resp;
	memset(this->statusBuf, 0, sizeof(this->statusBuf));

	// Send CMD55 with card's RCA
	resp = bus.Command(static_cast<uint8_t>(Command::AppCmd), (uint32_t)(this->RCA << 16), SDMMC::ResponseType::Short_CRC);
	if(resp.error != SDMMC::Error::None) {
		return false;
	}

	// Prepare data
	if(bus.TransferAsync((uint8_t*)this->statusBuf, 64, 64, true) != Status::Ok) {
		return false;
	}

	// Send ACMD51
	resp = bus.Command(static_cast<uint8_t>(AppCommand::SdStatus), 0, SDMMC::ResponseType::Short_CRC, SDMMC::TransferMode::Manual);
	if(resp.error != SDMMC::Error::None) {
		return false;
	}

	// Wait for data transfer
	if(bus.TransferWait(100) != Status::Ok) {
		return false;
	}

	// Parse information
	uint8_t* pStatus = (uint8_t*)this->statusBuf;
	// Byte 0, Bits 7-6: Data Bus Width (00=1-bit, 10=4-bit)
	this->cardInfo.currentBusWidth = (pStatus[0] >> 6) & 0x03;

	// Byte 8: Speed Class
	uint8_t rawClass = pStatus[8];
	switch(rawClass) {
		case 0: 
			this->cardInfo.speedClass = 0; 
			break;
		case 1:
			this->cardInfo.speedClass = 2;
			break;
		case 2:
			this->cardInfo.speedClass = 4;
			break;
		case 3:
			this->cardInfo.speedClass = 6;
			break;
		case 4:
			this->cardInfo.speedClass = 10;
			break;
		default:
			this->cardInfo.speedClass = 0;
			break;
	}

	// Byte 10, Bits 7-4: Allocation Unit Size
	this->cardInfo.allocationUnitSize = (pStatus[10] >> 4) & 0x0F;

	// Byte 14, Bits 7-4: UHS Speed Grade
	this->cardInfo.uhsSpeedGrade = (pStatus[14] >> 4) & 0x0F;

	// Byte 15: Video Speed Class
	this->cardInfo.videoSpeedClass = pStatus[15];

	// Byte 22: Application Performance Class
	uint8_t appClass = pStatus[22];
	if(appClass == 0x01) {
		this->cardInfo.appPerfClass = 1;
	}
	else if(appClass == 0x02) {
		this->cardInfo.appPerfClass = 2;
	}
	else {
		this->cardInfo.appPerfClass = 0;
	}

	return true;
}

bool SD::ChangeSpeedMode(SDMMC::BusSpeed speed) {
	SDMMC::CommandResponse resp;
	memset(this->statusBuf, 0, sizeof(this->statusBuf));

	// Map SDMMC speeds to SD CMD6 argument
	uint32_t sdCMD6Speed = 0;
	switch(speed) {
		case SDMMC::BusSpeed::HighSpeed: // 3.3V HS
		case SDMMC::BusSpeed::UHS_SDR25: // 1.8V SDR25
			sdCMD6Speed = 1; 
			break;
		case SDMMC::BusSpeed::UHS_SDR50: 
			sdCMD6Speed = 2; 
			break;
		case SDMMC::BusSpeed::UHS_SDR104: 
			sdCMD6Speed = 3; 
			break;
		case SDMMC::BusSpeed::UHS_DDR50: 
			sdCMD6Speed = 4; 
			break;
		default: 
			// Default/SDR12 requires function 0, but usually we just don't switch.
			return true; 
	}

	// Prepare data
	if(bus.TransferAsync((uint8_t*)this->statusBuf, 64, 64, true) != Status::Ok) {
		return false;
	}

	// Send CMD6 (Switch Function)
	// Argument meaning:
	// 31	 : Mode						: 0 > No Switch; 1 > Switch
	// 30:24 : Group 6 (??)				: F No Change
	// 23:20 : Group 5 (??)				: F No Change
	// 19:16 : Group 4 (Power Limit)	: F No Change; 0 > 200mA@3.6V; 1 > 400mA@3.6V; 2 > 600mA@3.6V; 3 > 800mA@3.6V, 
	// 19:16 : Group 3 (Drive Strength)	: F No Change; 0 > 50 Ohm; 1 > 33 Ohm; 2 > 66 Ohm; 3 > 100 Ohm
	// 19:16 : Group 2 (Command System)	: F No Change; 0 > Standard; 1 > eCommerce/Security; 2 > OTP; 3 > advanced security SD
	// 19:16 : Group 1 (Access Mode)	: F No Change; 0 > SDR12; 1 > SDR25; 2 > SDR50; 3 > SDR104; 4 > DDR50
	uint32_t arg = 0x80FFFFF0 | (sdCMD6Speed);
	resp = bus.Command(static_cast<uint8_t>(Command::SwitchFunc), arg, SDMMC::ResponseType::Short_CRC, SDMMC::TransferMode::Manual);
	if(resp.error != SDMMC::Error::None) {
		return false;
	}

	// Wait for data transfer
	if(bus.TransferWait(100) != Status::Ok) {
		return false;
	}

	// Verify switch
	uint8_t* pData = (uint8_t*)this->statusBuf;
	if((pData[16] & 0x0F) != sdCMD6Speed) {
		// Card didn't switch
		return false;
	}

	// Switch host controller (SDMMC bus)
	bus.SetSpeedMode(speed);

	return true;
}

void SD::VoltageSwitchCallback(void* context) {
	GPIO* pin = static_cast<GPIO*>(context);

	//SD VIO Selection: 0 -> 3V3, 1 -> 1V8
	if (pin != nullptr) {
		pin->Write(1); 
	}
}