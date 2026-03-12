/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/dmaChannel.cpp
 */

#include "dmaChannel.hpp"

DMAChannel::DMAChannel(DMA_TypeDef *instance, uint8_t channelIndex) {
	this->instance = instance;
	this->channelIndex = channelIndex & 0x0F; // Limit channel numer to 15
	this->irqPriority = 0x0E; 	// Lowest priority
}

Status DMAChannel::Init(const Config& config) {
	// Create RTOS objects
	if(tx_event_flags_create(&this->event, const_cast<char*>("dma event")) != TX_SUCCESS) {
		return Status::Error;
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

	// Configure RIF
	this->ConfigureRIF(true, true);

	// Configure DMA channel
	// Set transfer direction and data width
	LL_DMA_SetDataTransferDirection(this->instance, this->channelIndex, static_cast<uint32_t>(config.direction));
	uint32_t ctrl1 = (static_cast<uint32_t>(config.srcWidth) << DMA_CTR1_SDW_LOG2_Pos);
	ctrl1 |= (static_cast<uint32_t>(config.dstWidth) << DMA_CTR1_DDW_LOG2_Pos);
	MODIFY_REG(((DMA_Channel_TypeDef *)((uint32_t)this->instance + LL_DMA_CH_OFFSET_TAB[this->channelIndex]))->CTR1, DMA_CTR1_DDW_LOG2_Msk | DMA_CTR1_SDW_LOG2_Msk, ctrl1);
	LL_DMA_SetDataAlignment(this->instance, this->channelIndex, LL_DMA_DATA_ALIGN_ZEROPADD);
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
	LL_DMA_SetSrcBurstLength(this->instance, this->channelIndex, 1);
	LL_DMA_SetDestBurstLength(this->instance, this->channelIndex, 1);
	LL_DMA_SetChannelPriorityLevel(this->instance, this->channelIndex, static_cast<uint32_t>(config.priority));
	// Set transfer trigger, event and request
	LL_DMA_SetTransferEventMode(this->instance, this->channelIndex, LL_DMA_TCEM_BLK_TRANSFER);
	LL_DMA_SetTriggerPolarity(this->instance, this->channelIndex, LL_DMA_TRIG_POLARITY_MASKED);
	LL_DMA_SetPeriphRequest(this->instance, this->channelIndex, config.requestLine);
	LL_DMA_SetTransferMode(this->instance, this->channelIndex, LL_DMA_NORMAL);

	// Configure Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);
	LL_DMA_EnableIT_DTE(this->instance, this->channelIndex);

	return Status::Ok;
}

Status DMAChannel::Transfer(uint32_t srcAddr, uint32_t dstAddr, uint32_t len) {
	if(len == 0) {
		return Status::Error;
	}

	// Clear event flags
	tx_event_flags_set(&this->event, 0, TX_AND);

	// Prepare internal transfer variables
	this->currentSrc = srcAddr;
	this->currentDst = dstAddr;

	uint32_t blockSize = len;
	uint32_t blockCount  = 0;

	// Check for transfer size limit (standard channel limited to 16-bit block length)
	if(blockSize > 0xFFFF) {
		// Only advanced channels allow reapted blocks i.e. 2D addressing
		if(this->channelIndex < LL_DMA_CHANNEL_12) {
			return Status::Error;
		}

		// Channel is capable, find a clean divisor that results in an 8-byte aligned block size
		for (uint32_t i = 1; i <= 2048; i++) {
			if (len % i == 0 && (len / i) <= 65528 && ((len / i) % 8 == 0)) {
				blockSize = len / i;
				blockCount = i - 1;
				break;
			}
		}

		// If no clean divisor is found, we fall back to an error.
		if (blockSize > 65535) {
			return Status::Error; 
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

	// Ensure channel is disabled
	LL_DMA_DisableChannel(this->instance, this->channelIndex);

	// Set address and lengths
	LL_DMA_SetSrcAddress(this->instance, this->channelIndex, (uint32_t)this->currentSrc);
	LL_DMA_SetDestAddress(this->instance, this->channelIndex, (uint32_t)this->currentDst);
	LL_DMA_SetBlkDataLength(this->instance, this->channelIndex, blockSize);
	LL_DMA_SetBlkRptCount(this->instance, this->channelIndex, blockCount);
	if(blockCount == 0) {
		// Single block, so normal event mode
		LL_DMA_SetTransferEventMode(this->instance, this->channelIndex, LL_DMA_TCEM_BLK_TRANSFER);
	}
	else {
		// Multi block, so update event mode
		LL_DMA_SetTransferEventMode(this->instance, this->channelIndex, LL_DMA_TCEM_RPT_BLK_TRANSFER);
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
	// Suspend channel
	LL_DMA_SuspendChannel(this->instance, this->channelIndex);
	while(LL_DMA_IsSuspendedChannel(this->instance, this->channelIndex) == 0x00);	// Wait for suspend

	// Disable channel
	LL_DMA_DisableChannel(this->instance, this->channelIndex);
	WRITE_REG(((DMA_Channel_TypeDef *)((uint32_t)this->instance + LL_DMA_CH_OFFSET_TAB[this->channelIndex]))->CTR1, 0x7F00);	// Clear all flags

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
		return Port::Port1; // Peripheral Port
	}

	// Default to AXI / Memory Port for SRAM, FMC, HyperBus, DCMIPP, etc.
	return Port::Port0;
}

void DMAChannel::InterruptHandler() {
	// Handle Transfer Error
	if(LL_DMA_IsActiveFlag_DTE(this->instance, this->channelIndex) == 0x01) {
		// Clear flag
		tx_event_flags_set(&this->event, EVT_ERR, TX_OR);
		LL_DMA_ClearFlag_DTE(this->instance, this->channelIndex);
	}

	// Handle Transfer Complete
	if(LL_DMA_IsActiveFlag_TC(this->instance, this->channelIndex) == 0x01) {
		// Full transfer completed
		LL_DMA_DisableIT_TC(this->instance, this->channelIndex);
		tx_event_flags_set(&this->event, EVT_TRANS_CPLT, TX_OR);
		LL_DMA_ClearFlag_TC(this->instance, this->channelIndex);
	}
}