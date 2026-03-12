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
	else {
		return Status::Error;
	}

	// Enable bus clocks
	if(this->instance == SPI1) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
		this->irqCall = SPI1_IRQn;
	}
	else if(this->instance == SPI2) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
		this->irqCall = SPI2_IRQn;
	}
	else if(this->instance == SPI3) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
		this->irqCall = SPI3_IRQn;
	}
	else if(this->instance == SPI4) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);
		this->irqCall = SPI4_IRQn;
	}
	else if(this->instance == SPI5) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI5);
		this->irqCall = SPI5_IRQn;
	}
	else if(this->instance == SPI6) {
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SPI6);
		this->irqCall = SPI6_IRQn;
	}
	this->sourceClockHz = config.sourceClockHz;

	// Configure SPI Interface
	this->SetBaudrate(config.baudrate);
	LL_SPI_SetTransferDirection(this->instance, LL_SPI_FULL_DUPLEX);
	LL_SPI_SetClockPhase(this->instance, static_cast<uint32_t>(config.phase));
	LL_SPI_SetClockPolarity(this->instance, static_cast<uint32_t>(config.polarity));
	LL_SPI_SetTransferBitOrder(this->instance, static_cast<uint32_t>(config.bitOrder));
	LL_SPI_SetDataWidth(this->instance, LL_SPI_DATAWIDTH_8BIT);
	LL_SPI_SetFIFOThreshold(this->instance, LL_SPI_FIFO_TH_01DATA);			// Set FIFO Length, SPI2_FIFO_LENGTH*Datawidth aka SPI2_FIFO_LENGTH*8bits
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
//	LL_SPI_EnableIT_TXP(this->instance);
//	LL_SPI_EnableIT_RXP(this->instance);
//	LL_SPI_EnableIT_DXP(this->instance);
//	LL_SPI_EnableIT_EOT(this->instance);
//	LL_SPI_EnableIT_CRCERR(this->instance);
//	LL_SPI_EnableIT_UDR(this->instance);
//	LL_SPI_EnableIT_OVR(this->instance);
#endif

	// Enable SPI
	LL_SPI_Enable(this->instance);

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
	// LL_SPI_Enable(this->instance);

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
	this->rxBuffer = rxBuf;
	this->length = len;

	// SPI Setup
	LL_SPI_SetTransferSize(this->instance, this->length);
	LL_SPI_Enable(this->instance);
	LL_SPI_StartMasterTransfer(this->instance);

	// Fill FIFO (TBD)
	// for(i = 0; i < SPI2_FIFO_LENGTH; i++) {
	LL_SPI_TransmitData8(this->instance, *this->txBuffer);
	this->txBuffer = this->txBuffer + 1;
	this->length = this->length - 1;
	// }

	// Enable interrupts
	LL_SPI_EnableIT_DXP(this->instance);
	LL_SPI_EnableIT_EOT(this->instance);

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
	LL_SPI_DisableIT_DXP(this->instance);
	LL_SPI_DisableIT_EOT(this->instance);
//	LL_SPI_EnableIT_UDR(this->instance);
//	LL_SPI_EnableIT_OVR(this->instance);
#endif

	// Disable peripheral
	LL_SPI_Disable(this->instance);

	// Clear transfer context
	this->txBuffer = nullptr;
	this->rxBuffer = nullptr;
	this->length = 0;

	// Re-enable peripheral
	LL_SPI_Enable(this->instance);

	// Signal event flags and release mutex
	tx_event_flags_set(&this->event, EVT_ERR, TX_OR);
	tx_mutex_put(&this->mutex);

	return Status::Ok;
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

void SPI::InterruptHandler() {
	// Check DXP (duplex data ready, RXP & TXP Flag set) flag value in ISR register
	if(LL_SPI_IsEnabledIT_DXP(this->instance) == 0x01 && LL_SPI_IsActiveFlag_DXP(this->instance) == 0x01) {
		// Write and Read data with number of writes/reads equal to the packet/FIFO threshold size
		// uint8_t i;
		// for(i = 0; i < SPI2_FIFO_LENGTH; i++) {
		// First Write byte to FIFO
		LL_SPI_TransmitData8(this->instance, *this->txBuffer);
		this->txBuffer = this->txBuffer + 1;
		this->length = this->length - 1;

		// Then Read byte from FIFO
		*this->rxBuffer = LL_SPI_ReceiveData8(this->instance);
		this->rxBuffer = this->rxBuffer + 1;
		// }
	}

	// Check EOT flag value in ISR register
	if(LL_SPI_IsEnabledIT_EOT(this->instance) == 0x01 && LL_SPI_IsActiveFlag_EOT(this->instance) == 0x01) {
		// Number of reads equal to the packet/FIFO threshold size
		// uint8_t i;
		// for(i = 0; i < SPI2_FIFO_LENGTH; i++) {
		*this->rxBuffer = LL_SPI_ReceiveData8(this->instance);
		this->rxBuffer = this->rxBuffer + 1;
		// }

		// Set CS High
//		GPIOWrite(GPIO_OUT_NRF_CS, 0x01);

		// Clear TXTF Flag
		LL_SPI_ClearFlag_TXTF(this->instance);

		LL_SPI_DisableIT_DXP(this->instance);
		LL_SPI_DisableIT_EOT(this->instance);

		// Disable SPI (preferred to reset state machine)
		LL_SPI_Disable(this->instance);

		tx_event_flags_set(&this->event, EVT_TRANS_CPLT, TX_OR);
	}
}