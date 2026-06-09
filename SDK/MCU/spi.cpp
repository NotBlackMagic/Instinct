/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/spi.cpp
 */

#include "spi.hpp"

#define SPI_USE_IRQ

SPI::SPI(SPI_TypeDef *instance) {
	this->instance = instance;
	this->irqPriority = 0x0E; 	//Lowest priority
	this->isInitialized = false;
	this->txBuffer = nullptr;
	this->txLength = 0;
	this->rxBuffer = nullptr;
	this->rxLength = 0;
}

Status SPI::Init(const Config &config) {
	if(config.baudrate == 0 || config.sourceClockHz == 0) {
		return Status::Error;
	}

	if(this->isInitialized == true) {
		return Status::Ok;
	}

	// Create RTOS objects
	if(tx_mutex_create(&this->mutex, const_cast<char*>("spi mutex"), TX_INHERIT) != TX_SUCCESS) {
		return Status::Error;
	}
	if(tx_event_flags_create(&this->event, const_cast<char*>("spi event")) != TX_SUCCESS) {
		tx_mutex_delete(&this->mutex);
		return Status::Error;
	}

	// Enable bus clocks
	uint32_t fifoThreshold = LL_SPI_FIFO_TH_01DATA; // Default safe fallback
	if(this->instance == SPI1) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
		this->irqCall = SPI1_IRQn;
		fifoThreshold = LL_SPI_FIFO_TH_08DATA;	// SPI1, SPI2, SPI3, and SPI6 have 16-byte FIFO
	}
	else if(this->instance == SPI2) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
		this->irqCall = SPI2_IRQn;
		fifoThreshold = LL_SPI_FIFO_TH_08DATA;	// SPI1, SPI2, SPI3, and SPI6 have 16-byte FIFO
	}
	else if(this->instance == SPI3) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
		this->irqCall = SPI3_IRQn;
		fifoThreshold = LL_SPI_FIFO_TH_08DATA;	// SPI1, SPI2, SPI3, and SPI6 have 16-byte FIFO
	}
	else if(this->instance == SPI4) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);
		this->irqCall = SPI4_IRQn;
		fifoThreshold = LL_SPI_FIFO_TH_04DATA;	// SPI4, and SPI5 have 8-byte FIFO
	}
	else if(this->instance == SPI5) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI5);
		this->irqCall = SPI5_IRQn;
		fifoThreshold = LL_SPI_FIFO_TH_04DATA;	// SPI4, and SPI5 have 8-byte FIFO
	}
	else if(this->instance == SPI6) {
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SPI6);
		this->irqCall = SPI6_IRQn;
		fifoThreshold = LL_SPI_FIFO_TH_08DATA;	// SPI1, SPI2, SPI3, and SPI6 have 16-byte FIFO
	}
	this->sourceClockHz = config.sourceClockHz;

	// Configure SPI Interface
	this->SetBaudrate(config.baudrate);
	LL_SPI_SetTransferDirection(this->instance, LL_SPI_FULL_DUPLEX);
	LL_SPI_SetClockPhase(this->instance, static_cast<uint32_t>(config.phase));
	LL_SPI_SetClockPolarity(this->instance, static_cast<uint32_t>(config.polarity));
	LL_SPI_SetTransferBitOrder(this->instance, static_cast<uint32_t>(config.bitOrder));
	LL_SPI_SetDataWidth(this->instance, LL_SPI_DATAWIDTH_8BIT);
	LL_SPI_SetFIFOThreshold(this->instance, fifoThreshold);	// Set FIFO Length, SPI2_FIFO_LENGTH*Datawidth aka SPI2_FIFO_LENGTH*8bits
	LL_SPI_SetNSSMode(this->instance, LL_SPI_NSS_HARD_OUTPUT);
	if(this->instance == SPI1) {
		LL_SPI_EnableIOSwap(this->instance);
	}
//	LL_SPI_SetInternalSSLevel(this->instance, LL_SPI_SS_LEVEL_HIGH);					// VERY IMPORTANT, DOES NOT WORK WITHOUT WITH NSS_SOFT!!
//	LL_SPI_EnableNSSPulseMgt(this->instance);
//	LL_SPI_DisableCRC(this->instance);
//	LL_I2S_Disable(this->instance);
	LL_SPI_SetMode(this->instance, LL_SPI_MODE_MASTER);

	LL_SPI_EnableGPIOControl(this->instance);
	LL_SPI_EnableMasterRxAutoSuspend(this->instance);

#ifdef SPI_USE_IRQ
	// Configure SPI Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);
#endif

	// Enable SPI
	// LL_SPI_Enable(this->instance);

	this->isInitialized = true;
	return Status::Ok;
}

Status SPI::SetBaudrate(uint32_t baudrate) {
	if(baudrate == 0) {
		return Status::Error;
	}

	// Try lock SPI device
	bool useLock = this->isInitialized && (tx_thread_identify() != nullptr);
	if(useLock == true) {
		if(tx_mutex_get(&this->mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
			return Status::Error;
		}
	}

	// Calculate best prescaler value, equal or lower then asked frequency
	uint32_t prescaler = 0;				// As power of 2: 0: DIV2, 1: DIV4, ...
	if(baudrate > 0) {
		while(prescaler < 7) {
			uint32_t currentFreq = this->sourceClockHz / (0x01 << (prescaler + 1));
			if(currentFreq <= baudrate) {
				break;
			}
			prescaler += 1;
		}
	}
	else {
		prescaler = 7;
	}

	// Disable SPI to update prescaler
	LL_SPI_Disable(this->instance);
	LL_SPI_SetBaudRatePrescaler(this->instance, ((prescaler) << SPI_CFG1_MBR_Pos));

	// Release SPI device
	if(useLock == true) {
		tx_mutex_put(&this->mutex);
	}
	return Status::Ok;
}

Status SPI::TransferAsync(const uint8_t *txBuf, uint8_t *rxBuf, uint32_t len) {
	// Try lock SPI device
	if(tx_mutex_get(&this->mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
		return Status::Timeout;
	}

	// Clear event flags
	tx_event_flags_set(&this->event, 0, TX_AND);

	// Prepare internal transfer variables
	this->txBuffer = txBuf;
	this->txLength = len;
	this->rxBuffer = rxBuf;
	this->rxLength = len;

	// Clear flags
	LL_SPI_ClearFlag_EOT(this->instance);
	LL_SPI_ClearFlag_TXTF(this->instance);

	// SPI Setup
	LL_SPI_SetTransferSize(this->instance, this->txLength);
	LL_SPI_Enable(this->instance);

	// Enable interrupts
	LL_SPI_EnableIT_TXP(this->instance);
	LL_SPI_EnableIT_RXP(this->instance);
	LL_SPI_EnableIT_EOT(this->instance);

	LL_SPI_StartMasterTransfer(this->instance);

	return Status::Ok;
}

Status SPI::TransferWait(uint32_t timeoutTicks) {
	// Wait for event
	ULONG events;
	UINT status = tx_event_flags_get(&this->event, EVT_TRANS_CPLT | EVT_ERR, TX_OR_CLEAR, &events, timeoutTicks);

	if(status != TX_SUCCESS) {
		this->TransferAbort();
		return Status::Timeout;
	}

	if((events & EVT_ERR) == EVT_ERR) {
		// Transfer error occurred
		tx_mutex_put(&this->mutex);
		return Status::Error;
	}

	tx_mutex_put(&this->mutex);
	return Status::Ok;
}

Status SPI::TransferAbort() {
	// Disable interrupts
#ifdef SPI_USE_IRQ
	LL_SPI_DisableIT_TXP(this->instance);
	LL_SPI_DisableIT_RXP(this->instance);
	LL_SPI_DisableIT_EOT(this->instance);
#endif

	// Disable peripheral
	LL_SPI_Disable(this->instance);

	// Clear transfer context
	this->txBuffer = nullptr;
	this->txLength = 0;
	this->rxBuffer = nullptr;
	this->rxLength = 0;

	// Signal event flags and release mutex
	tx_event_flags_set(&this->event, EVT_ERR, TX_OR);
	tx_mutex_put(&this->mutex);

	return Status::Ok;
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

void SPI::InterruptHandler() {
	// Read data from RX FIFO until empty: RXP means at least ONE packet (one FIFO Threshold) can be read
	while(LL_SPI_IsActiveFlag_RXP(this->instance) == 0x01 && this->rxLength > 0) {
		uint8_t rxByte = LL_SPI_ReceiveData8(this->instance);
		if(this->rxBuffer != nullptr) {
			*this->rxBuffer = rxByte;
			this->rxBuffer = this->rxBuffer + 1;
		}
		this->rxLength = this->rxLength - 1;
	}

	// Write data to TX FIFO until is full: TXP means at least ONE packet (one FIFO Threshold) can be written
	while(LL_SPI_IsActiveFlag_TXP(this->instance) == 0x01 && this->txLength > 0) {
		uint8_t txByte = (this->txBuffer != nullptr) ? *this->txBuffer : 0xFF;
		LL_SPI_TransmitData8(this->instance, txByte);
		if(this->txBuffer != nullptr) {
			this->txBuffer = this->txBuffer + 1;
		}
		this->txLength = this->txLength - 1;
	}

	// Check if all bytes are sent or in FIFO
	if(this->txLength == 0 && LL_SPI_IsEnabledIT_TXP(this->instance) == 0x01) {
		LL_SPI_DisableIT_TXP(this->instance);
	}

	// Handle End of Transfer (EOT)
	if(LL_SPI_IsEnabledIT_EOT(this->instance) == 0x01 && LL_SPI_IsActiveFlag_EOT(this->instance) == 0x01) {
		// Read all remaining RX bytes in FIFO
		while(this->rxLength > 0) {
			uint8_t rxByte = LL_SPI_ReceiveData8(this->instance);
			if(this->rxBuffer != nullptr) {
				*this->rxBuffer = rxByte;
				this->rxBuffer = this->rxBuffer + 1;
			}
			this->rxLength = this->rxLength - 1;
		}

		// Clear flags
		LL_SPI_ClearFlag_TXTF(this->instance);
		LL_SPI_ClearFlag_EOT(this->instance);

		LL_SPI_DisableIT_RXP(this->instance);
		LL_SPI_DisableIT_EOT(this->instance);

		// Disable SPI (preferred to reset state machine)
		LL_SPI_Disable(this->instance);

		tx_event_flags_set(&this->event, EVT_TRANS_CPLT, TX_OR);
	}
}