/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/hyperbus.cpp
 */

#include "hyperbus.hpp"

HyperBus::HyperBus(XSPI_TypeDef *instance) {
	this->instance = instance;
	this->irqPriority = 0x0E; // Lowest priority (safe default)
}

Status HyperBus::Init(const Config &config) {
	if(config.frequencyHz == 0 || config.sourceClockHz == 0) {
		return Status::Error;
	}

	// Create RTOS objects
	if(tx_mutex_create(&this->mutex, const_cast<char*>("hyperbus mutex"), TX_INHERIT) != TX_SUCCESS) {
		return Status::Error;
	}
	if(tx_event_flags_create(&this->event, const_cast<char*>("hyperbus event")) != TX_SUCCESS) {
		tx_mutex_delete(&this->mutex);
		return Status::Error;
	}

	// Enable bus clocks
	if(this->instance == XSPI1) {
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_XSPI1);
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_XSPIM);
		this->irqCall = XSPI1_IRQn;
	}
	else if(this->instance == XSPI2) {
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_XSPI2);
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_XSPIM);
		this->irqCall = XSPI2_IRQn;
	}
	else if(this->instance == XSPI3) {
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_XSPI3);
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_XSPIM);
		this->irqCall = XSPI3_IRQn;
	}
	else {
		return Status::Error;
	}

	// Save write latency mode (only for memory access)
	useWriteZeroLatency = config.writeZeroLatency;

	// Get device size in exponent form
	// Check if valid device size i.e. > 0 and power of 2
	if(config.sizeBytes == 0 || (config.sizeBytes & (config.sizeBytes - 1)) != 0x00) {
		return Status::Error;
	}
	uint8_t exponent = 31UL - __CLZ(config.sizeBytes);
	uint32_t devSizeVal = exponent - 1;

	// Disable XSPI Interface
	MODIFY_REG(this->instance->CR, XSPI_CR_EN, 0x00);

	//DMODE[2:0] must be to equal to 101

	// Configure XSPI I/O Manager
	MODIFY_REG(XSPIM->CR, XSPIM_CR_MODE, 0x00);			// XSPI multiplexing mode: Direct Mode i.e. port1 to XSPI1 and port2 to XSPI2
	MODIFY_REG(XSPIM->CR, XSPIM_CR_MUXEN, 0x00);		// Multiplexer mode enable: Disabled
	
	// Configure OCTOSPI System
	MODIFY_REG(this->instance->CR, XSPI_CR_MSEL, 0x00);				// Set flash select: Data exchange over IO[3:0] in quad or [7:0] in octal
	MODIFY_REG(this->instance->CR, XSPI_CR_FMODE, 0x00);			// Set functional mode: Indirect-write mode
	MODIFY_REG(this->instance->CR, XSPI_CR_NOPREF_AXI, 0x00);		// Set automatic prefetch for signaled AXI: Enabled
	MODIFY_REG(this->instance->CR, XSPI_CR_NOPREF, 0x00);			// Set automatic prefetch: Enabled
	MODIFY_REG(this->instance->CR, XSPI_CR_CSSEL, 0x00);			// Set CS selection: NCS1
	MODIFY_REG(this->instance->CR, XSPI_CR_PMM, 0x00);				// Set polling match mode: AND-match mode
	MODIFY_REG(this->instance->CR, XSPI_CR_APMS, 0x00);				// Set auto-polling mode stop: Stopped by abort or disabling OCTOSPI
	MODIFY_REG(this->instance->CR, XSPI_CR_FTHRES, ((31) << XSPI_CR_FTHRES_Pos));	// Set FIFO threshold (1 to 64 bytes): 32 bytes
	MODIFY_REG(this->instance->CR, XSPI_CR_DMM, 0x00);				// Set dual-memory configuration: Disabled
	MODIFY_REG(this->instance->CR, XSPI_CR_TCEN, 0x00);				// Set Timeout counter usage: Disabled
	MODIFY_REG(this->instance->CR, XSPI_CR_DMAEN, 0x00);			// Set DMA usage: Disabled
	// MODIFY_REG(this->instance->CR, XSPI_CR_ABORT, XSPI_CR_ABORT);// Abort request
	// MODIFY_REG(this->instance->CR, XSPI_CR_EN, XSPI_CR_EN);		// Enable XSPI

	// Enable interrupts
	MODIFY_REG(this->instance->CR, XSPI_CR_TEIE, 0x00);				// Transfer Error Interrupt
	MODIFY_REG(this->instance->CR, XSPI_CR_TCIE, 0x00);				// Transfer Complete Interrupt
	MODIFY_REG(this->instance->CR, XSPI_CR_FTIE, 0x00);				// FIFO Threshold Interrupt
	MODIFY_REG(this->instance->CR, XSPI_CR_SMIE, 0x00);				// Status Match Interrupt
	MODIFY_REG(this->instance->CR, XSPI_CR_TOIE, 0x00);				// TimeOut Interrupt

	// Configure OCTOSPI Device (Settings for S26HS512T SEMPER Flash)
	MODIFY_REG(this->instance->DCR1, XSPI_DCR1_MTYP, ((4) << XSPI_DCR1_MTYP_Pos));			// Set Memory type: 4 -> HyperBus memory mode, 5 -> HyperBus register mode
	MODIFY_REG(this->instance->DCR1, XSPI_DCR1_EXTENDMEM, 0x00);							// Set extended memory support: NCS1 and NCS2 depend on CSSEL
	MODIFY_REG(this->instance->DCR1, XSPI_DCR1_DEVSIZE, (devSizeVal << XSPI_DCR1_DEVSIZE_Pos));	// Set Device size (2^[DEVSIZE+1])
	MODIFY_REG(this->instance->DCR1, XSPI_DCR1_CSHT, ((4) << XSPI_DCR1_CSHT_Pos));			// Set CS high time: High at least 5 cycles
	MODIFY_REG(this->instance->DCR1, XSPI_DCR1_FRCK, 0x00);									// Set free running clock: Disabled
	// MODIFY_REG(this->instance->DCR1, XSPI_DCR1_CKMODE, 0x00);							// Set clock mode: Mode 0 (CLK low while nCS high) READ ONLY

	MODIFY_REG(this->instance->DCR2, XSPI_DCR2_WRAPSIZE, 0x00);								// Set wrap size: Wrapped reads are not supported by the memory

	MODIFY_REG(this->instance->DCR3, XSPI_DCR3_CSBOUND, 0x00);		// Set NCS boundary (2^CSBOUND): Disabled
	MODIFY_REG(this->instance->DCR3, XSPI_DCR3_MAXTRAN, 0x00);		// Set maximum transfer (MAXTRAN + 1): Disabled

	MODIFY_REG(this->instance->DCR4, XSPI_DCR4_REFRESH, config.refreshRate);	// Set refresh rate: Disabled, only used/required for HyperRAM devices

	// Calculate best prescaler value, equal or lower then asked frequency
	uint32_t prescaler = 0;				// 0: DIV1, 1: DIV2, 2: DIV3, etc...
	if(config.frequencyHz > 0) {
		prescaler = (config.sourceClockHz + (config.frequencyHz - 1)) / config.frequencyHz;
		prescaler -= 1;
		if(prescaler > 0xFFUL) {
			prescaler = 0xFFUL;
		}
	}
	else {
		prescaler = 255;
	}
	while((this->instance->SR & XSPI_SR_BUSY_Msk) == XSPI_SR_BUSY);	// Wait for not BUSY
	MODIFY_REG(this->instance->DCR2, XSPI_DCR2_PRESCALER, ((prescaler) << XSPI_DCR2_PRESCALER_Pos));	// Set clock prescaler (Fclk = Fkernel/[value+1])
	while((this->instance->SR & XSPI_SR_BUSY_Msk) == XSPI_SR_BUSY);	// Wait for calibrations to complete

	MODIFY_REG(this->instance->TCR, (XSPI_TCR_SSHIFT), 0x00);	// Configure sample shifting

	// HyperBus latency configuration
	uint32_t writeZeroLatencyVal = 0x00;
	if(config.writeZeroLatency == true) {
		writeZeroLatencyVal = XSPI_HLCR_WZL;
	}
	WRITE_REG(this->instance->HLCR, ((static_cast<uint32_t>(config.rwRecoveryTime) << XSPI_HLCR_TRWR_Pos) |
									(static_cast<uint32_t>(config.initialLatency) << XSPI_HLCR_TACC_Pos) |
									static_cast<uint32_t>(writeZeroLatencyVal) | 
									static_cast<uint32_t>(config.latencyMode)));

	// Configure SDMMC Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);

	// Enable x-SPI
	MODIFY_REG(this->instance->CR, XSPI_CR_EN, XSPI_CR_EN);

	this->isInitialized = true;
	return Status::Ok;
}

Status HyperBus::DeInit(void) {
	if(this->isInitialized == false) {
		return Status::Ok;
	}

	// Disable IRQ first
	NVIC_DisableIRQ(this->irqCall);

	// Power down HyperBus
	MODIFY_REG(this->instance->CR, XSPI_CR_EN, 0x00);

	// Force hardware reset through peripheral clock
	if(this->instance == XSPI1) {
		LL_AHB5_GRP1_ForceReset(LL_AHB5_GRP1_PERIPH_XSPI1);
		Time::Delay(2);
		LL_AHB5_GRP1_ReleaseReset(LL_AHB5_GRP1_PERIPH_XSPI1);

		// Disable clocks
		LL_AHB5_GRP1_DisableClock(LL_AHB5_GRP1_PERIPH_XSPI1);
	}
	else if(this->instance == XSPI2) {
		LL_AHB5_GRP1_ForceReset(LL_AHB5_GRP1_PERIPH_XSPI2);
		Time::Delay(2);
		LL_AHB5_GRP1_ReleaseReset(LL_AHB5_GRP1_PERIPH_XSPI2);

		// Disable clocks
		LL_AHB5_GRP1_DisableClock(LL_AHB5_GRP1_PERIPH_XSPI2);
	}
	else if(this->instance == XSPI3) {
		LL_AHB5_GRP1_ForceReset(LL_AHB5_GRP1_PERIPH_XSPI3);
		Time::Delay(2);
		LL_AHB5_GRP1_ReleaseReset(LL_AHB5_GRP1_PERIPH_XSPI3);

		// Disable clocks
		LL_AHB5_GRP1_DisableClock(LL_AHB5_GRP1_PERIPH_XSPI3);
	}

	// Abort RTOS waits/blocks and clean up RTOS Resources
	if (tx_event_flags_delete(&this->event) != TX_SUCCESS) {
		// Something went wrong...
	}

	if (tx_mutex_delete(&this->mutex) != TX_SUCCESS) {
		// Something went wrong...
	}

	this->isInitialized = false;
	return Status::Ok;
}

Status HyperBus::LockBus(uint32_t timeoutTicks) {
	// Try lock HyperBus device
	if(tx_mutex_get(&this->mutex, timeoutTicks) != TX_SUCCESS) {
		return Status::Timeout;
	}
	return Status::Ok;
}

Status HyperBus::UnlockBus() {
	// Release HyperBus device
	if(tx_mutex_put(&this->mutex) != TX_SUCCESS) {
		return Status::Error;
	}
	return Status::Ok;
}

Status HyperBus::TransferAsync(AddressSpace space, uint32_t addr, AddrSize addrSize, BusWidth width, uint8_t* buf, uint32_t len, bool isRead) {
	if(this->isMemoryMapped == true) {
		return Status::Error;
	}
	
	// Enforce 16-bit Alignment and Size
	if ((len % 2 != 0) || ((uintptr_t)buf % 2 != 0)) {
		return Status::Error; // Alignment or Size Error
	}
	
	// Clear event flags
	tx_event_flags_set(&this->event, 0, TX_AND);

	// Prepare internal transfer variables
	this->buffer = (uint16_t*)buf;
	this->length = len;
	this->dirRead = isRead;

	// Automatic write latency handling. Register writes are always Zero write latency!
	if(isRead == false) {
		if(space == HyperBus::AddressSpace::Register || this->useWriteZeroLatency == true) {
			MODIFY_REG(this->instance->HLCR, XSPI_HLCR_WZL, XSPI_HLCR_WZL);
		}
		else {
			MODIFY_REG(this->instance->HLCR, XSPI_HLCR_WZL, 0x00);
		}
	}

	Status status = this->Command(space, addr, addrSize, width, len);
	if(status != Status::Ok) {
		return status;
	}

	if(isRead == false) {
		MODIFY_REG(this->instance->CR, XSPI_CR_FMODE, 0x00);		// Indirect write mode
	}
	else {
		MODIFY_REG(this->instance->CR, XSPI_CR_FMODE, XSPI_CR_FMODE_0);		// Indirect read mode
	}

	// Enable Interrupts: FIFO Threshold (FTIE), Transfer Complete (TCIE), Transfer Error (TEIE)
	SET_BIT(this->instance->CR, XSPI_CR_FTIE | XSPI_CR_TCIE | XSPI_CR_TEIE);

	if(isRead == true) {
		// Trigger the transfer by re-writing address or instruction register
		WRITE_REG(this->instance->AR, addr);
	}

	return Status::Ok;
}

Status HyperBus::TransferWait(uint32_t timeoutTicks) {
	// Wait for event
	ULONG events;
	UINT status = tx_event_flags_get(&this->event, EVT_TRANS_CPLT | EVT_ERR, TX_OR_CLEAR, &events, timeoutTicks);

	if(status != TX_SUCCESS) {
		return Status::Timeout;
	}

	if((events & EVT_ERR) == EVT_ERR) {
		return Status::Error;
	}
	else {
		return Status::Ok;
	}
}

Status HyperBus::Command(AddressSpace space, uint32_t addr, AddrSize addrSize, BusWidth width, uint32_t dataLen) {
	// HyperBus protocol Command/address bit assignment:
	//CA Bit#      | Bit Name          | Bit Function
	//-------------|-------------------|----------------
	//47 	       | R/W#              | Identifies read or write operation: 1=Read, 0=Write
	//46 	       | Target            | Identifies memory or register access: 1=Register, 0=Memory
	//45           | Burst Type        | Identifies burst access type: 0=Wrapped Burst, 1=Linear Burst
	//44-39 (1G)   | Reserved          | Reserved for future address expansion. Set to 0
	//44-38 (512M) | Reserved          | Reserved for future address expansion. Set to 0
	//44-37 (256M) | Reserved          | Reserved for future address expansion. Set to 0
	//38-16 (1G)   | Row & Up Col Addr | Half page component of target address
	//37-16 (512M) | Row & Up Col Addr | Half page component of target address
	//36-16 (256M) | Row & Up Col Addr | Half page component of target address
	//15-3         | Reserved          | Reserved for future column address expansion. Set to 0
	//2-0          | Lower Column Addr | Lower Column component of the target address: System word address bits A2–0 selecting the starting word within a half-page.

	uint32_t dqsMode = ((uint32_t)XSPI_CCR_DQSE);

	// Wait for busy flag to clear
	uint32_t timeoutMs = 5;
	uint32_t timestamp = Time::GetMs();
	while((this->instance->SR & XSPI_SR_BUSY_Msk) == XSPI_SR_BUSY) {
		if((Time::GetMs() - timestamp) > timeoutMs) {
			// Clock stop timeout
			SET_BIT(this->instance->CR, XSPI_CR_ABORT); // Force hardware abort
			return Status::Error;
		}
	}

	// Re-initialize the value of the functional mode
	MODIFY_REG(this->instance->CR, XSPI_CR_FMODE, 0x00);

	// Configure the address space in the DCR1 register: Either Memory space or Register space
	if(space == HyperBus::AddressSpace::Memory) {
		// Memory space
		MODIFY_REG(this->instance->DCR1, XSPI_DCR1_MTYP_0, 0);
	}
	else {
		// Register space
		MODIFY_REG(this->instance->DCR1, XSPI_DCR1_MTYP_0, XSPI_DCR1_MTYP_0);
	}

	// Configure the CCR and WCCR registers with the address size and the following configuration:
    //		- DQS signal enabled (used as RWDS)
    //		- DTR mode enabled on address and data
    //		- address and data on 8 or 16 lines
	WRITE_REG(this->instance->CCR, (dqsMode | XSPI_CCR_DDTR | static_cast<uint32_t>(width) | static_cast<uint32_t>(addrSize) | XSPI_CCR_ADDTR | XSPI_CCR_ADMODE_2));
	WRITE_REG(this->instance->WCCR, (dqsMode | XSPI_WCCR_DDTR | static_cast<uint32_t>(width) | static_cast<uint32_t>(addrSize) | XSPI_WCCR_ADDTR | XSPI_WCCR_ADMODE_2));

	// Configure the DLR register with the number of data
	WRITE_REG(this->instance->DLR, (dataLen - 1U));

	// Configure the AR register with the address value
	WRITE_REG(this->instance->AR, addr);

	return Status::Ok;
}

uint32_t HyperBus::GetBaseAddr() const {
	if(this->instance == XSPI1) {
		return 0x90000000UL;

	}
	if(this->instance == XSPI2) {
		return 0x70000000UL;

	}
	if(this->instance == XSPI3) {
		return 0x80000000UL;
	}
	return 0x00;
}

Status HyperBus::EnterMemoryMappedMode() {
	// Wait for busy flag to clear
	uint32_t timeoutMs = 5;
	uint32_t timestamp = Time::GetMs();
	while((this->instance->SR & XSPI_SR_BUSY_Msk) == XSPI_SR_BUSY) {
		if((Time::GetMs() - timestamp) > timeoutMs) {
			// Clock stop timeout
			SET_BIT(this->instance->CR, XSPI_CR_ABORT); // Force hardware abort
			return Status::Timeout;
		}
	}

	// Configure CR register with functional mode as memory mapped mode
	// Set latency mode for Memory space
	if(this->useWriteZeroLatency == true) {
		MODIFY_REG(this->instance->HLCR, XSPI_HLCR_WZL, XSPI_HLCR_WZL);
	}
	else {
		MODIFY_REG(this->instance->HLCR, XSPI_HLCR_WZL, 0x00);
	}

	// Force memory space
	MODIFY_REG(this->instance->DCR1, XSPI_DCR1_MTYP_0, 0);

	// Set configure CR register
	MODIFY_REG(this->instance->CR, XSPI_CR_FMODE, (XSPI_CR_FMODE_0 | XSPI_CR_FMODE_1));		// Memory mapped mode

	this->isMemoryMapped = true;

	return Status::Ok;
}

Status HyperBus::ExitMemoryMappedMode() {
	Status status = Status::Ok;

	// Wait for busy flag to clear
	uint32_t timeoutMs = 5;
	uint32_t timestamp = Time::GetMs();
	while((this->instance->SR & XSPI_SR_BUSY_Msk) == XSPI_SR_BUSY) {
		if((Time::GetMs() - timestamp) > timeoutMs) {
			// Clock stop timeout
			SET_BIT(this->instance->CR, XSPI_CR_ABORT); // Force hardware abort
			status = Status::Timeout;
		}
	}

	// Force memory space
	MODIFY_REG(this->instance->DCR1, XSPI_DCR1_MTYP_0, 0);

	// Clear configure CR register
	MODIFY_REG(this->instance->CR, XSPI_CR_FMODE, 0x00);		// Indirect write mode

	this->isMemoryMapped = false;

	return status;
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

void HyperBus::InterruptHandler() {
	this->irqStatus = this->instance->SR;
	uint32_t mask = this->instance->CR;

	// Handle Transfer Errors (TEF)
	if(((this->irqStatus & XSPI_SR_TEF) == XSPI_SR_TEF) && ((mask & XSPI_CR_TEIE) == XSPI_CR_TEIE)) {
		// Clear flags
		WRITE_REG(this->instance->FCR, XSPI_FCR_CTEF);
		// Disable interrupts
		CLEAR_BIT(this->instance->CR, XSPI_CR_TCIE | XSPI_CR_TEIE | XSPI_CR_FTIE);
		tx_event_flags_set(&this->event, EVT_ERR, TX_OR);
	}

	// Handle FIFO (FTF)
	if(((this->irqStatus & XSPI_SR_FTF) == XSPI_SR_FTF) && ((mask & XSPI_CR_FTIE) == XSPI_CR_FTIE)) {
		// Calculte chuck bytes to read/write
		uint32_t chunkBytes = (this->length > 32) ? 32 : this->length;

		if(this->dirRead == true) {
			// Burst read
			for(uint32_t i = 0; i < chunkBytes; i += 2) {
				*this->buffer++ = *((__IO uint16_t *)&this->instance->DR);
			}
		}
		else {
			// Burst write
			for(uint32_t i = 0; i < chunkBytes; i += 2) {
				*((__IO uint16_t *)&this->instance->DR) = *this->buffer++;
			}
		}

		this->length -= chunkBytes;

		// Check if all bytes transfered
		if(this->length == 0) {
			// Disable interrupt
			CLEAR_BIT(this->instance->CR, XSPI_CR_FTIE);
		}
	}

	// Handle Transfer Complete (TCF)
	if(((this->irqStatus & XSPI_SR_TCF) == XSPI_SR_TCF) && ((mask & XSPI_CR_TCIE) == XSPI_CR_TCIE)) {
		// For reads, read remaining bytes (less then 32 bytes)
		if(this->dirRead == true && this->length > 0) {
			for(uint32_t i = 0; i < this->length; i += 2) {
				*this->buffer++ = *((__IO uint16_t *)&this->instance->DR);
			}
			this->length = 0;
		}

		// Clear flags
		WRITE_REG(this->instance->FCR, XSPI_FCR_CTCF);
		// Disable interrupts
		CLEAR_BIT(this->instance->CR, XSPI_CR_TCIE | XSPI_CR_TEIE | XSPI_CR_FTIE);

		tx_event_flags_set(&this->event, EVT_TRANS_CPLT, TX_OR);
	}
}