/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/xspi.cpp
 */

#include "xspi.hpp"

XSPI::XSPI(XSPI_TypeDef *instance) {
	this->instance = instance;
}

void XSPI::Init(const Config &config) {
	//Enable bus clocks
	if(this->instance == XSPI1) {
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_XSPI1);
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_XSPIM);
	}
	else if(this->instance == XSPI2) {
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_XSPI2);
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_XSPIM);
	}
	else if(this->instance == XSPI3) {
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_XSPI3);
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_XSPIM);
	}
	else {
		// return Status::Error;
		return;
	}

	MODIFY_REG(this->instance->CR, XSPI_CR_EN, 0x00);	//Disable XSPI Interface

	//Configure XSPI I/O Manager
	MODIFY_REG(XSPIM->CR, XSPIM_CR_MODE, 0x00);			//XSPI multiplexing mode: Direct Mode i.e. port1 to XSPI1 and port2 to XSPI2
	MODIFY_REG(XSPIM->CR, XSPIM_CR_MUXEN, 0x00);		//Multiplexer mode enable: Disabled
	
	//Configure OCTOSPI System
	MODIFY_REG(this->instance->CR, XSPI_CR_MSEL, 0x00);				//Set flash select: Data exchange over IO[3:0] in quad or [7:0] in octal
	MODIFY_REG(this->instance->CR, XSPI_CR_FMODE, 0x00);			//Set functional mode: Indirect-write mode
	MODIFY_REG(this->instance->CR, XSPI_CR_NOPREF_AXI, 0x00);		//Set automatic prefetch for signaled AXI: Disabled
	MODIFY_REG(this->instance->CR, XSPI_CR_NOPREF, 0x00);			//Set automatic prefetch: Disabled
	MODIFY_REG(this->instance->CR, XSPI_CR_CSSEL, 0x00);			//Set CS selection: NCS1
	MODIFY_REG(this->instance->CR, XSPI_CR_PMM, 0x00);				//Set polling match mode: AND-match mode
	MODIFY_REG(this->instance->CR, XSPI_CR_APMS, 0x00);				//Set auto-polling mode stop: Stopped by abort or disabling OCTOSPI
	MODIFY_REG(this->instance->CR, XSPI_CR_FTHRES, ((0) << XSPI_CR_FTHRES_Pos));	//Set FIFO threshold (1 to 64 bytes): 1 bytes
	MODIFY_REG(this->instance->CR, XSPI_CR_DMM, 0x00);				//Set dual-memory configuration: Disabled
	MODIFY_REG(this->instance->CR, XSPI_CR_TCEN, 0x00);				//Set Timeout counter usage: Disabled
	MODIFY_REG(this->instance->CR, XSPI_CR_DMAEN, 0x00);			//Set DMA usage: Disabled
	// MODIFY_REG(this->instance->CR, XSPI_CR_ABORT, XSPI_CR_ABORT);	//Abort request
	// MODIFY_REG(this->instance->CR, XSPI_CR_EN, XSPI_CR_EN);			//Enable XSPI

	//Enable interrupts
	MODIFY_REG(this->instance->CR, XSPI_CR_TEIE, 0x00);				//Transfer Error Interrupt
	MODIFY_REG(this->instance->CR, XSPI_CR_TCIE, 0x00);				//Transfer Complete Interrupt
	MODIFY_REG(this->instance->CR, XSPI_CR_FTIE, 0x00);				//FIFO Threshold Interrupt
	MODIFY_REG(this->instance->CR, XSPI_CR_SMIE, 0x00);				//Status Match Interrupt
	MODIFY_REG(this->instance->CR, XSPI_CR_TOIE, 0x00);				//TimeOut Interrupt

	//Configure OCTOSPI Device (Settings for W25N512GV NAND Flash)
	MODIFY_REG(this->instance->DCR1, XSPI_DCR1_MTYP, ((2) << XSPI_DCR1_MTYP_Pos));			//Set Memory type: Standard mode
	MODIFY_REG(this->instance->DCR1, XSPI_DCR1_EXTENDMEM, 0x00);							//Set extended memory support: NCS1 and NCS2 depend on CSSEL
	MODIFY_REG(this->instance->DCR1, XSPI_DCR1_DEVSIZE, ((25) << XSPI_DCR1_DEVSIZE_Pos));	//Set Device size (2^[DEVSIZE+1]): 64 MByte
	MODIFY_REG(this->instance->DCR1, XSPI_DCR1_CSHT, ((0) << XSPI_DCR1_CSHT_Pos));			//Set CS high time: High at least 1 cycle
	MODIFY_REG(this->instance->DCR1, XSPI_DCR1_FRCK, 0x00);									//Set free running clock: Disabled
	// MODIFY_REG(this->instance->DCR1, XSPI_DCR1_CKMODE, 0x00);								//Set clock mode: Mode 0 (CLK low while nCS high) READ ONLY

	MODIFY_REG(this->instance->DCR2, XSPI_DCR2_WRAPSIZE, 0x00);									//Set wrap size: Wrapped reads are not supported by the memory
	MODIFY_REG(this->instance->DCR2, XSPI_DCR2_PRESCALER, ((0) << XSPI_DCR2_PRESCALER_Pos));	//Set clock prescaler (Fclk = Fkernel/[value+1]): Fkernel = 100 MHz, Fclk = 100/(0+1) = 100 MHz

	MODIFY_REG(this->instance->DCR3, XSPI_DCR3_CSBOUND, 0x00);		//Set NCS boundary (2^CSBOUND): Disabled
	MODIFY_REG(this->instance->DCR3, XSPI_DCR3_MAXTRAN, 0x00);		//Set maximum transfer (MAXTRAN + 1): Disabled

	MODIFY_REG(this->instance->DCR4, XSPI_DCR4_REFRESH, 0x00);		//Set refresh rate: Disabled

	//Enable x-SPI
	MODIFY_REG(this->instance->CR, XSPI_CR_EN, XSPI_CR_EN);
}

void XSPI::Write(XSPIOperationMode mode, uint8_t inst, uint8_t dmmCyc, uint32_t addr, XSPIOperationSize addrSize, uint8_t *data, uint32_t len) {
	//XSPI Regular command protocol can have up to 5 phases:
	//Phase 1: Instruction phase
	//Phase 2: Address phase
	//Phase 3: Alternate-byte phase
	//Phase 4: Dummy phase
	//Phase 5: Data phase

	//Configure to Indirect-write mode
	MODIFY_REG(this->instance->CR, XSPI_CR_FMODE, ((0) << XSPI_CR_FMODE_Pos));

	//Set frame timing
	MODIFY_REG(this->instance->TCR, XSPI_TCR_DCYC, ((dmmCyc) << XSPI_TCR_DCYC_Pos));		//Number of Dummy Cycles (Phase 4)
//	MODIFY_REG(this->instance->TCR, OCTOSPI_TCR_DHQC, ((0) << OCTOSPI_TCR_DHQC_Pos));		//Delay Hold Quarter Cycle: No delay hold
//	MODIFY_REG(this->instance->TCR, OCTOSPI_TCR_SSHIFT, ((0) << OCTOSPI_TCR_SSHIFT_Pos));	//Sample Shift: No shift

	//Set frame format
	CLEAR_REG(this->instance->CCR);		//Clear format
	//Instruction format
	MODIFY_REG(this->instance->CCR, XSPI_CCR_IMODE, ((1) << XSPI_CCR_IMODE_Pos));			//Instruction Mode: 000: No data; 001: Data on a single line; 010: Data on two lines; 011: Data on four lines; 100: Data on eight lines
//	MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_ISIZE, ((0) << OCTOSPI_CCR_ISIZE_Pos));		//Instruction Size: 00: 8-bit; 01: 16-bit; 10: 24-bit; 11: 32-bit
//	MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_IDTR, 0x00);								//Instruction Double Transfer Rate: Disabled
	
	//Address format
	if(addrSize != XSPI::XSPIOperationSize::OSPI_Size_None) {
		MODIFY_REG(this->instance->CCR, XSPI_CCR_ADMODE, ((1) << XSPI_CCR_ADMODE_Pos));		//Address Mode: 000: No data; 001: Data on a single line; 010: Data on two lines; 011: Data on four lines; 100: Data on eight lines
		MODIFY_REG(this->instance->CCR, XSPI_CCR_ADSIZE, (((uint8_t)addrSize - 1) << XSPI_CCR_ADSIZE_Pos));			//Address Size: 00: 8-bit; 01: 16-bit; 10: 24-bit; 11: 32-bit
		MODIFY_REG(this->instance->CCR, XSPI_CCR_ADDTR, 0x00);								//Address Double Transfer Rate
	}
	//Alternate Byte format
//	MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_ABMODE, ((0) << OCTOSPI_CCR_ABMODE_Pos));	//Alternate Bytes Mode: 000: No data; 001: Data on a single line; 010: Data on two lines; 011: Data on four lines; 100: Data on eight lines
//	MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_ABSIZE, ((0) << OCTOSPI_CCR_ABSIZE_Pos));	//Alternate Bytes Size: 00: 8-bit; 01: 16-bit; 10: 24-bit; 11: 32-bit
//	MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_ABDTR, 0x00);								//Alternate Bytes Double Transfer Rate: Disabled
	//Data format
	if(len > 0) {
		MODIFY_REG(this->instance->CCR, XSPI_CCR_DMODE, (((uint8_t)mode) << XSPI_CCR_DMODE_Pos));	//Data Mode: 000: No data; 001: Data on a single line; 010: Data on two lines; 011: Data on four lines; 100: Data on eight lines
//		MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_DDTR, 0x00);							//Data Double Transfer Rate: Disabled
//		MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_DQSE, 0x00);							//Data Strobe Management: DQS Disabled
//		MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_SIOO, 0x00);							//Send Instruction Only Once Mode
		WRITE_REG(this->instance->DLR, len - 1);											//Data Length
	}

	//Clear transfer complete flag
	MODIFY_REG(this->instance->FCR, XSPI_FCR_CTCF, XSPI_FCR_CTCF);

	//Set Frame content
	//Set Instruction (Phase 1)
	this->instance->IR = inst;

	//Set target address (Phase 2)
	if((uint8_t)addrSize > 0) {
		this->instance->AR = addr;
	}

	//Set optional alternate byte (Phase 3)
//	this->instance->ABR = 0x00;

	//Write data (Phase 5)
	uint16_t dataIndex = 0;
	if(len >= 4) {
		//Transmit data while transfer complete flag is not set
		while((this->instance->SR & XSPI_SR_TCF_Msk) != XSPI_SR_TCF && (len - dataIndex) >= 4) {
			//Check if FIFO has space for more data to be written
			uint8_t fifoLevel = 32 - ((this->instance->SR & XSPI_SR_FLEVEL_Msk) >> XSPI_SR_FLEVEL_Pos);		//Inverse of filled FIFO Level, to get empty bytes
			if(fifoLevel >= 4) {
				//OSPITransmitData32(this->instance, *(uint32_t*)&data[dataIndex]);
				*((__IO uint32_t *)&this->instance->DR) = *(uint32_t*)&data[dataIndex];
				dataIndex += 4;
			}
		}
	}

	uint16_t remainBytes = (len - dataIndex);
	for(; remainBytes > 0; remainBytes--) {
		// OSPITransmitData8(this->instance, data[dataIndex++]);
		*((__IO uint8_t *)&this->instance->DR) = data[dataIndex++];
	}

	//Wait for busy flag to clear
	while((this->instance->SR & XSPI_SR_BUSY_Msk) == XSPI_SR_BUSY);
}

void XSPI::Read(XSPIOperationMode mode, uint8_t inst, uint8_t dmmCyc, uint32_t addr, XSPIOperationSize addrSize, uint8_t *data, uint32_t len) {
	//XSPI Regular command protocol can have up to 5 phases:
	//Phase 1: Instruction phase
	//Phase 2: Address phase
	//Phase 3: Alternate-byte phase
	//Phase 4: Dummy phase
	//Phase 5: Data phase

	//Configure to Indirect-read mode
	MODIFY_REG(this->instance->CR, XSPI_CR_FMODE, ((1) << XSPI_CR_FMODE_Pos));

	//Set frame timing
	MODIFY_REG(this->instance->TCR, XSPI_TCR_DCYC, ((dmmCyc) << XSPI_TCR_DHQC_Pos));		//Number of Dummy Cycles (Phase 4)
//	MODIFY_REG(this->instance->TCR, OCTOSPI_TCR_DHQC, ((0) << OCTOSPI_TCR_DHQC_Pos));		//Delay Hold Quarter Cycle: No delay hold
//	MODIFY_REG(this->instance->TCR, OCTOSPI_TCR_SSHIFT, ((0) << OCTOSPI_TCR_SSHIFT_Pos));	//Sample Shift: No shift

	//Set frame format
	CLEAR_REG(this->instance->CCR);		//Clear format
	//Instruction format
	MODIFY_REG(this->instance->CCR, XSPI_CCR_IMODE, ((1) << XSPI_CCR_IMODE_Pos));			//Instruction Mode: 000: No data; 001: Data on a single line; 010: Data on two lines; 011: Data on four lines; 100: Data on eight lines
//	MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_ISIZE, ((0) << OCTOSPI_CCR_ISIZE_Pos));		//Instruction Size: 00: 8-bit; 01: 16-bit; 10: 24-bit; 11: 32-bit
//	MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_IDTR, 0x00);								//Instruction Double Transfer Rate: Disabled
	//Address format
	if(addrSize != XSPI::XSPIOperationSize::OSPI_Size_None) {
		MODIFY_REG(this->instance->CCR, XSPI_CCR_ADMODE, (((uint8_t)mode) << XSPI_CCR_ADMODE_Pos));			//Address Mode: 000: No data; 001: Data on a single line; 010: Data on two lines; 011: Data on four lines; 100: Data on eight lines
		MODIFY_REG(this->instance->CCR, XSPI_CCR_ADSIZE, (((uint8_t)addrSize - 1) << XSPI_CCR_ADSIZE_Pos));		//Address Size: 00: 8-bit; 01: 16-bit; 10: 24-bit; 11: 32-bit
		MODIFY_REG(this->instance->CCR, XSPI_CCR_ADDTR, 0x00);								//Address Double Transfer Rate
	}
	//Alternate Byte format
//	MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_ABMODE, ((0) << OCTOSPI_CCR_ABMODE_Pos));	//Alternate Bytes Mode: 000: No data; 001: Data on a single line; 010: Data on two lines; 011: Data on four lines; 100: Data on eight lines
//	MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_ABSIZE, ((0) << OCTOSPI_CCR_ABSIZE_Pos));	//Alternate Bytes Size: 00: 8-bit; 01: 16-bit; 10: 24-bit; 11: 32-bit
//	MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_ABDTR, 0x00);								//Alternate Bytes Double Transfer Rate: Disabled
	//Data format
	MODIFY_REG(this->instance->CCR, XSPI_CCR_DMODE, (((uint8_t)mode) << XSPI_CCR_DMODE_Pos));		//Data Mode: 000: No data; 001: Data on a single line; 010: Data on two lines; 011: Data on four lines; 100: Data on eight lines
//	MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_DDTR, 0x00);								//Data Double Transfer Rate: Disabled
//	MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_DQSE, 0x00);								//Data Strobe Management: DQS Disabled
//	MODIFY_REG(this->instance->CCR, OCTOSPI_CCR_SIOO, 0x00);								//Send Instruction Only Once Mode
	WRITE_REG(this->instance->DLR, len - 1);												//Data Length

	//Clear transfer complete flag
	MODIFY_REG(this->instance->FCR, XSPI_FCR_CTCF, XSPI_FCR_CTCF);

	//Set Frame content
	//Set Instruction (Phase 1)
	this->instance->IR = inst;

	//Set target address (Phase 2)
	if((uint8_t)addrSize > 0) {
		this->instance->AR = addr;
	}

	//Set optional alternate byte (Phase 3)
//	this->instance->ABR = 0x00;

	//Read data (Phase 5)
	uint16_t dataIndex = 0;
	//Wait for busy flag to clear
	while((this->instance->SR & XSPI_SR_TCF_Msk) != XSPI_SR_TCF) {
		//Check if FIFO has data to be read
		uint8_t fifoLevel = ((this->instance->SR & XSPI_SR_FLEVEL_Msk) >> XSPI_SR_FLEVEL_Pos);
		if(fifoLevel >= 16) {
			//Read data bytes
			//OSPIReceiveData32(this->instance);	
			*(uint32_t*)&data[dataIndex] = (*((__IO uint32_t *)&this->instance->DR));
			dataIndex += 4;
			*(uint32_t*)&data[dataIndex] = (*((__IO uint32_t *)&this->instance->DR));
			dataIndex += 4;
			*(uint32_t*)&data[dataIndex] = (*((__IO uint32_t *)&this->instance->DR));
			dataIndex += 4;
			*(uint32_t*)&data[dataIndex] = (*((__IO uint32_t *)&this->instance->DR));
			dataIndex += 4;
		}
	}

	uint8_t fifoLevel = ((this->instance->SR & XSPI_SR_FLEVEL_Msk) >> XSPI_SR_FLEVEL_Pos);
	for(; fifoLevel > 0; fifoLevel--) {
		data[dataIndex++] = (*((__IO uint8_t *)&this->instance->DR));
		//OSPIReceiveData8(this->instance);
	}

	//Wait for busy flag to clear
	while((this->instance->SR & XSPI_SR_BUSY_Msk) == XSPI_SR_BUSY);
}