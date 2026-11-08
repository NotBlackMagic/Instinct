/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	SDK/MCU/dmaChannel.cpp
 */

#include "dmaChannel.hpp"

DMAChannel::DMAChannel(DMA_TypeDef *instance, uint8_t channelIndex, const char* channelName) {
	this->instance = instance;
	this->channelIndex = channelIndex & 0x0F; // Limit channel number to 15
	this->channelRegs = reinterpret_cast<DMA_Channel_TypeDef*>(reinterpret_cast<uint32_t>(this->instance) + LL_DMA_CH_OFFSET_TAB[this->channelIndex]);
	this->irqPriority = 0x05; 	// Higher priority
	this->channelName = channelName;

	// Linked list registration
	this->nextChannel = headChannel;
	this->headChannel = this;
}

Status DMAChannel::Init(const Config& config) {
	// Validate burst lengths (hardware supports 1 to 64 beats)
	if(config.srcBurstLength == 0 || config.srcBurstLength > 64 || config.dstBurstLength == 0 || config.dstBurstLength > 64) {
		return Status::Error;
	}

	// DoubleWord (64-bit) width is only valid on HPDMA
	if(this->instance == GPDMA1 && (config.srcWidth == DataWidth::DoubleWord || config.dstWidth == DataWidth::DoubleWord)) {
		return Status::Error;
	}

	this->config = config;

	// Create RTOS objects
	if(this->isInitialized == false) {
		if(tx_event_flags_create(&this->event, const_cast<char*>("dma event")) != TX_SUCCESS) {
			return Status::Error;
		}
		this->isInitialized = true;
	}

	// Enable bus clocks and identify IRQ lines
	if(this->instance == HPDMA1) {
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_HPDMA1);
		this->irqCall = (IRQn_Type)(HPDMA1_Channel0_IRQn + this->channelIndex);
	}
	else if(this->instance == GPDMA1) {
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPDMA1);
		this->irqCall = (IRQn_Type)(GPDMA1_Channel0_IRQn + this->channelIndex);
	}
	else {
		return Status::Error;
	}

	// Ensure channel is stopped and flags are cleared before configuring
	this->SuspendAndReset();

	// Configure RIF
	this->ConfigureRIF(true, true);

	// Configure DMA channel
	// Set transfer direction and data width
	LL_DMA_SetDataTransferDirection(this->instance, this->channelIndex, static_cast<uint32_t>(config.direction));
	uint32_t ctrl1 = (static_cast<uint32_t>(config.srcWidth) << DMA_CTR1_SDW_LOG2_Pos);
	ctrl1 |= (static_cast<uint32_t>(config.dstWidth) << DMA_CTR1_DDW_LOG2_Pos);
	MODIFY_REG(((DMA_Channel_TypeDef *)((uint32_t)this->instance + LL_DMA_CH_OFFSET_TAB[this->channelIndex]))->CTR1, DMA_CTR1_DDW_LOG2_Msk | DMA_CTR1_SDW_LOG2_Msk, ctrl1);
	LL_DMA_SetDataAlignment(this->instance, this->channelIndex, LL_DMA_DATA_PACK_UNPACK);
	// Set addressing mode
	if(config.incSrc == true) {
		LL_DMA_SetSrcIncMode(this->instance, this->channelIndex, LL_DMA_SRC_INCREMENT);
	}
	else {
		LL_DMA_SetSrcIncMode(this->instance, this->channelIndex, LL_DMA_SRC_FIXED);
	}
	if(config.incDst == true) {
		LL_DMA_SetDestIncMode(this->instance, this->channelIndex, LL_DMA_DEST_INCREMENT);
	}
	else {
		LL_DMA_SetDestIncMode(this->instance, this->channelIndex, LL_DMA_DEST_FIXED);
	}
	// Set burst and priority
	LL_DMA_SetBlkHWRequest(this->instance, this->channelIndex, LL_DMA_HWREQUEST_SINGLEBURST);
	LL_DMA_SetSrcBurstLength(this->instance, this->channelIndex, config.srcBurstLength);
	LL_DMA_SetDestBurstLength(this->instance, this->channelIndex, config.dstBurstLength);
	LL_DMA_SetChannelPriorityLevel(this->instance, this->channelIndex, static_cast<uint32_t>(config.priority));
	// Set transfer trigger, event and request
	LL_DMA_SetTransferEventMode(this->instance, this->channelIndex, LL_DMA_TCEM_BLK_TRANSFER);	// Transfer complete trigger mode (when TC flag is set)
	LL_DMA_SetTriggerPolarity(this->instance, this->channelIndex, LL_DMA_TRIG_POLARITY_MASKED);
	LL_DMA_SetPeriphRequest(this->instance, this->channelIndex, config.requestLine);
	LL_DMA_SetTransferMode(this->instance, this->channelIndex, LL_DMA_NORMAL);

	// Reset 2D registers if channel is 2D-capable
	if(this->channelIndex >= LL_DMA_CHANNEL_12) {
		LL_DMA_ConfigBlkRptAddrUpdate(this->instance, this->channelIndex, 0U);
		LL_DMA_ConfigAddrUpdateValue(this->instance, this->channelIndex, 0U, 0U);
		LL_DMA_ConfigBlkRptAddrUpdateValue(this->instance, this->channelIndex, 0U, 0U);
	}

	// Configure Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);
	LL_DMA_EnableIT_DTE(this->instance, this->channelIndex);
	LL_DMA_EnableIT_USE(this->instance, this->channelIndex);
	LL_DMA_EnableIT_TO(this->instance, this->channelIndex);

	return Status::Ok;
}

Status DMAChannel::TransferAsync(uint32_t srcAddr, uint32_t dstAddr, uint32_t len) {
	if(len == 0) {
		return Status::Error;
	}

	// Reject if channel is currently busy with an active transfer
	if(LL_DMA_IsEnabledChannel(this->instance, this->channelIndex) == 1UL || this->remainingBytes > 0) {
		return Status::Busy;
	}

	// Clear stale RTOS event flags and hardware status flags
	tx_event_flags_set(&this->event, 0, TX_AND);
	WRITE_REG(this->channelRegs->CFCR, 0x7F00U);

	// Prepare internal transfer variables
	this->currentSrc = srcAddr;
	this->currentDst = dstAddr;
	this->blockSize = len;
	this->remainingBytes = 0;	// Default: 0 for single-block and hardware 2D transfers
	this->totalTransferLength = len;

	uint32_t hwBlockCount = 0;

	// Check for transfer size limit (standard channel limited to 16-bit block length)
	if(this->blockSize > 0xFFFF) {
		bool hwConfigured = false;

		// Try 2D repeat mode on channels 12-15 if SW intervention is not required
		if(this->channelIndex >= LL_DMA_CHANNEL_12 && this->config.swBlockReload == false) {
			// Channel is capable, find a clean divisor that results in an "burstSize" bytes aligned block size
			constexpr uint32_t maxHwBlk = 65528U;
			constexpr uint32_t burstAlign = 64U;
			uint32_t minBlocks = (len + maxHwBlk - 1U) / maxHwBlk;

			for(uint32_t i = minBlocks; i <= 2048; i++) {
				if((len % i) == 0U) {
					uint32_t candidateBlk = len / i;
					if (candidateBlk <= maxHwBlk && (candidateBlk % burstAlign) == 0U) {
						this->blockSize = candidateBlk;
						hwBlockCount = i - 1U;
						this->remainingBytes = 0;
						hwConfigured = true;
						break;
					}
				}
			}
		}

		if(hwConfigured == false) {
			// Fall back to software chunking if 2D divisor failed or on channels 0-11
			this->blockSize = (len > MAX_SW_BLK_LEN) ? MAX_SW_BLK_LEN : len;
			this->remainingBytes = len - this->blockSize;
			hwBlockCount = 0;
		}
	}

	// Set transfer ports
	uint32_t port = LL_DMA_SRC_ALLOCATED_PORT0;
	if(this->GetPortFromAddress(this->currentSrc) == DMAChannel::Port::Port1) {
		port = LL_DMA_SRC_ALLOCATED_PORT1;
	}
	LL_DMA_SetSrcAllocatedPort(this->instance, this->channelIndex, port);
	
	port = LL_DMA_DEST_ALLOCATED_PORT0;
	if(this->GetPortFromAddress(this->currentDst) == DMAChannel::Port::Port1) {
		port = LL_DMA_DEST_ALLOCATED_PORT1;
	}
	LL_DMA_SetDestAllocatedPort(this->instance, this->channelIndex, port);

	// Configure DMA channel

	// Set address and lengths
	LL_DMA_SetSrcAddress(this->instance, this->channelIndex, (uint32_t)this->currentSrc);
	LL_DMA_SetDestAddress(this->instance, this->channelIndex, (uint32_t)this->currentDst);
	LL_DMA_SetBlkDataLength(this->instance, this->channelIndex, this->blockSize);

	if(this->channelIndex >= LL_DMA_CHANNEL_12) {
		LL_DMA_SetBlkRptCount(this->instance, this->channelIndex, hwBlockCount);
		if(hwBlockCount == 0) {
			// Single block, so normal event mode
			LL_DMA_SetTransferEventMode(this->instance, this->channelIndex, LL_DMA_TCEM_BLK_TRANSFER);
		}
		else {
			// Multi block, so update event mode
			LL_DMA_SetTransferEventMode(this->instance, this->channelIndex, LL_DMA_TCEM_RPT_BLK_TRANSFER);
		}
	}
	else {
		LL_DMA_SetTransferEventMode(this->instance, this->channelIndex, LL_DMA_TCEM_BLK_TRANSFER);
	}

	// Enable interrupts
	LL_DMA_EnableIT_TC(this->instance, this->channelIndex);

	// Enable DMA channel
	LL_DMA_EnableChannel(this->instance, this->channelIndex);

	return Status::Ok;
}

Status DMAChannel::TransferWait(uint32_t timeoutTicks) {
	// Wait for event
	ULONG events;
	UINT status = tx_event_flags_get(&this->event, EVT_TRANS_CPLT | EVT_ERR, TX_OR_CLEAR, &events, timeoutTicks);
	if(status != TX_SUCCESS) {		
		this->TransferStop();
		return Status::Timeout;
	}

	if((events & EVT_ERR) == EVT_ERR) {
		// Transfer error occurred
		this->TransferStop();
		return Status::Error;
	}

	return Status::Ok;
}

Status DMAChannel::TransferStop() {
	bool wasActive = (LL_DMA_IsEnabledChannel(this->instance, this->channelIndex) == 1UL) || (this->remainingBytes > 0);

	// Suspend channel and disable, when enabled
	Status status = this->SuspendAndReset();

	// Reset software multi-block state
	this->remainingBytes = 0;
	this->blockSize = 0;
	if(wasActive == true) {
		this->stats.userAborts = this->stats.userAborts + 1;
		tx_event_flags_set(&this->event, EVT_ERR, TX_OR); // Unblock any thread in TransferWait()
	}

	return status;
}

uint32_t DMAChannel::GetRemaining() {
	uint32_t blkLen = LL_DMA_GetBlkDataLength(this->instance, this->channelIndex);
	uint32_t blkCnt = LL_DMA_GetBlkRptCount(this->instance, this->channelIndex);
	return this->remainingBytes + blkLen + (this->blockSize * blkCnt);
}

Status DMAChannel::SuspendAndReset() {
	if(LL_DMA_IsEnabledChannel(this->instance, this->channelIndex) == 1UL) {
		LL_DMA_SuspendChannel(this->instance, this->channelIndex);

		// Poll CSR.SUSPF (NOT CCR.SUSP) to ensure ongoing AXI/AHB bursts complete before reset
		uint32_t timeout = 10000U;
		while(LL_DMA_IsActiveFlag_SUSP(this->instance, this->channelIndex) == 0UL && timeout > 0U) {
			timeout = timeout - 1;
		}

		if(timeout == 0U) {
			return Status::Timeout;
		}

		// Issue channel reset and wait for hardware to clear EN
		LL_DMA_ResetChannel(this->instance, this->channelIndex);
		timeout = 10000U;
		while(LL_DMA_IsEnabledChannel(this->instance, this->channelIndex) == 1UL && timeout > 0U) {
			timeout = timeout - 1;
		}
	}

	// Clear all channel status flags (TOF, SUSPF, USEF, ULEF, DTEF, HTF, TCF)
	WRITE_REG(this->channelRegs->CFCR, 0x7F00U);
	return Status::Ok;
}

void DMAChannel::ConfigureRIF(bool isSecure, bool isPrivileged) {
	// DMA access configuration
	if(isSecure == true) {
		LL_DMA_EnableChannelSecure(this->instance, this->channelIndex);
		LL_DMA_EnableChannelSrcSecure(this->instance, this->channelIndex);
		LL_DMA_EnableChannelDestSecure(this->instance, this->channelIndex);
	}
	else {
		LL_DMA_DisableChannelSecure(this->instance, this->channelIndex);
		LL_DMA_DisableChannelSrcSecure(this->instance, this->channelIndex);
		LL_DMA_DisableChannelDestSecure(this->instance, this->channelIndex);
	}
	if(isPrivileged == true) {
		LL_DMA_EnableChannelPrivilege(this->instance, this->channelIndex);
	}
	else {
		LL_DMA_DisableChannelPrivilege(this->instance, this->channelIndex);
	}
	// LL_DMA_SetStaticIsolation(this->instance, this->channelIndex, LL_DMA_CHANNEL_STATIC_CID_2);
}

DMAChannel::Port DMAChannel::GetPortFromAddress(uint32_t address) {
	// Check if the address falls in the APB/AHB peripheral memory space
	if(address >= 0x40000000 && address <= 0x5FFFFFFF) {
		return Port::Port1; // AHB Port
	}
	// Check if the address falls in the APB/AHB SRAM memory space
	if((address >= 0x28000000U && address < 0x28008000U) || (address >= 0x38000000U && address < 0x38008000U)) {
		return Port::Port1; // AHB Port
	}
	// Default to AXI / Memory Port for SRAM, FMC, HyperBus, DCMIPP, etc.
	return Port::Port0;
}

void DMAChannel::InterruptHandler() {
	// Snapshot CSR in a single volatile read
	uint32_t csr = READ_REG(this->channelRegs->CSR);

	// Check for hardware faults (DTE, USE, TO)
	if((csr & (DMA_CSR_DTEF | DMA_CSR_USEF | DMA_CSR_TOF)) != 0U) {
		this->remainingBytes = 0;
		LL_DMA_DisableIT_TC(this->instance, this->channelIndex);

		if((csr & DMA_CSR_DTEF) != 0U) {
			// Data Transfer Error (AXI/AHB slave error, unmapped bus, RIF denial)
			this->stats.transferErrors = this->stats.transferErrors + 1;

			// Clear flag
			LL_DMA_ClearFlag_DTE(this->instance, this->channelIndex);

			if(this->config.EventCallback != nullptr) {
				this->config.EventCallback(this->config.callbackContext, Event::TransferError);
			}
		}

		if((csr & DMA_CSR_USEF) != 0U) {
			// User Setting Error (Burst length / FIFO threshold / address alignment mismatch)
			this->stats.settingErrors = this->stats.settingErrors + 1;

			// Clear flag
			LL_DMA_ClearFlag_USE(this->instance, this->channelIndex);

			if(this->config.EventCallback != nullptr) {
				this->config.EventCallback(this->config.callbackContext, Event::SettingError);
			}
		}

		if((csr & DMA_CSR_TOF) != 0U) {
			// Trigger Overrun (Trigger event arrived while channel was already active)
			this->stats.triggerOverruns = this->stats.triggerOverruns + 1;

			// Clear flag
			LL_DMA_ClearFlag_TO(this->instance, this->channelIndex);

			if(this->config.EventCallback != nullptr) {
				this->config.EventCallback(this->config.callbackContext, Event::TriggerOverrun);
			}
		}

		tx_event_flags_set(&this->event, EVT_ERR, TX_OR);

		// Exit early
		return;
	}

	// Handle Transfer Complete
	if((csr & DMA_CSR_TCF) != 0U) {
		// Full transfer completed
		if(this->remainingBytes > 0) {
			// Software chunking continues
			// Disable peripheral DMA request via callback
			if(this->config.swBlockReload && this->config.EventCallback != nullptr) {
				this->config.EventCallback(this->config.callbackContext, Event::RequestDisable);
			}

			// Clear flag here, when DMA request are possibly disabled
			LL_DMA_ClearFlag_TC(this->instance, this->channelIndex);

			// Update pointers
			if(this->config.incSrc == true) {
				this->currentSrc = this->currentSrc + this->blockSize;
			}
			if(this->config.incDst == true) {
				this->currentDst = this->currentDst + this->blockSize;
			}

			// Calculate next block size
			this->blockSize = (this->remainingBytes > MAX_SW_BLK_LEN) ? MAX_SW_BLK_LEN : this->remainingBytes;
			this->remainingBytes -= this->blockSize;
			
			LL_DMA_SetSrcAddress(this->instance, this->channelIndex, this->currentSrc);
			LL_DMA_SetDestAddress(this->instance, this->channelIndex, this->currentDst);
			LL_DMA_SetBlkDataLength(this->instance, this->channelIndex, this->blockSize);
			
			// Re-enable DMA and then the peripheral
			LL_DMA_EnableChannel(this->instance, this->channelIndex);
			if(this->config.swBlockReload && this->config.EventCallback != nullptr) {
				this->config.EventCallback(this->config.callbackContext, Event::RequestEnable);
			}
		}
		else {
			LL_DMA_DisableIT_TC(this->instance, this->channelIndex);
			LL_DMA_ClearFlag_TC(this->instance, this->channelIndex);

			this->stats.transfers = this->stats.transfers + 1;
			this->stats.bytesTransferred = this->stats.bytesTransferred + this->totalTransferLength;

			// Run callback FIRST (if it calls TransferAsync, it clears flags to 0 here)
			if(this->config.EventCallback != nullptr) {
				this->config.EventCallback(this->config.callbackContext, Event::TransferComplete);
			}

			// Set completion flag AFTER callback so CaptureWait() never misses it
			tx_event_flags_set(&this->event, EVT_TRANS_CPLT, TX_OR);
		}
	}
}