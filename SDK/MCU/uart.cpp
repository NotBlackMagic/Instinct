/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/uart.cpp
 */

#include "uart.hpp"

UART::UART(USART_TypeDef *instance) {
	this->instance = instance;
	this->irqPriority = 0x0E; // Lowest priority
}

Status UART::Init(const Config &config) {
	if(config.baudrate == 0 || config.sourceClockHz == 0) {
		return Status::Error;
	}

	// Create RTOS objects
	if(tx_mutex_create(&mutex, const_cast<char*>("uart mutex"), TX_INHERIT) != TX_SUCCESS) {
		return Status::Error;
	}
	
	// Enable bus clocks and identify IRQ lines
	if(this->instance == USART1) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
		this->irqCall = USART1_IRQn;
	}
	else if(this->instance == USART2) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
		this->irqCall = USART2_IRQn;
	}
	else if(this->instance == USART3) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
		this->irqCall = USART3_IRQn;
	}
	else if(this->instance == UART4) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
		this->irqCall = UART4_IRQn;
	}
	else if(this->instance == UART5) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
		this->irqCall = UART5_IRQn;
	}
	else if(this->instance == USART6) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);
		this->irqCall = USART6_IRQn;
	}
	else if(this->instance == UART7) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART7);
		this->irqCall = UART7_IRQn;
	}
	else if(this->instance == UART8) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART8);
		this->irqCall = UART8_IRQn;
	}
	else if(this->instance == UART9) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_UART9);
		this->irqCall = UART9_IRQn;
	}
	else if(this->instance == USART10) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART10);
		this->irqCall = USART10_IRQn;
	}
	else {
		return Status::Error;
	}

	// Configure UART Interface
	LL_USART_SetTransferDirection(this->instance, LL_USART_DIRECTION_TX_RX);
	LL_USART_SetDataWidth(this->instance, static_cast<uint32_t>(config.dataBits));
	LL_USART_SetParity(this->instance, static_cast<uint32_t>(config.parity));
	LL_USART_SetStopBitsLength(this->instance, static_cast<uint32_t>(config.stopBits));
	if(config.hwFlowControl == 0x00) {
		LL_USART_SetHWFlowCtrl(this->instance, LL_USART_HWCONTROL_NONE);
	} 
	else {
		LL_USART_SetHWFlowCtrl(this->instance, LL_USART_HWCONTROL_RTS_CTS);
	}
	LL_USART_SetOverSampling(this->instance, LL_USART_OVERSAMPLING_16);
	LL_USART_SetBaudRate(this->instance, config.sourceClockHz, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, config.baudrate);
	LL_USART_SetPrescaler(this->instance, LL_USART_PRESCALER_DIV1);
	LL_USART_SetTXFIFOThreshold(this->instance, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_SetRXFIFOThreshold(this->instance, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_DisableFIFO(this->instance);
	LL_USART_ConfigAsyncMode(this->instance);

	// Configure UART Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);
	LL_USART_EnableIT_RXNE_RXFNE(this->instance);
	LL_USART_EnableIT_IDLE(this->instance);

	LL_USART_Enable(this->instance);
	// Wait for Init to finish
	while((!(LL_USART_IsActiveFlag_TEACK(this->instance))) || (!(LL_USART_IsActiveFlag_REACK(this->instance))));

	// Flush backlog if any
	//UINT state = tx_interrupt_control(TX_INT_DISABLE);
	if(txBusy == false) {
		StartTX();
	}
	//tx_interrupt_control(state);

	this->isInitialized = true;
	return Status::Ok;
}

Status UART::Transmit(uint8_t *buf, uint16_t len) {
	// Lock UART device
	bool useLock = this->isInitialized && (tx_thread_identify() != nullptr);
	if(useLock == true) {
		if(tx_mutex_get(&mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
			return Status::Timeout;
		}
	}

	// Buffer copy (to internal ring buffer)
	uint16_t tmpHead = txBufHead;

	uint16_t i;
	for(i = 0; i < len; i++) {
		uint16_t nextHead = (tmpHead + 1) % txBufferSize;

		// Check space against tail
		if(nextHead != txBufTail) {
			txBuffer[tmpHead] = buf[i];
			tmpHead = nextHead;
		}
		else {
			// Buffer overflow
			break;
		}
	}

	// Critical section!
	UINT state = tx_interrupt_control(TX_INT_DISABLE);
	txBufHead = tmpHead;

	if(this->isInitialized == true && txBusy == false) {
		txBusy = true;
		StartTX();
	}
	// End of critical section
	tx_interrupt_control(state);

	// Release UART device
	if(useLock == true) {
		tx_mutex_put(&mutex);
	}

	return Status::Ok;
}

uint32_t UART::Receive(uint8_t *buf, uint32_t maxLen) {
	uint32_t tmpHead = rxBufHead;
	uint32_t tmpTail = rxBufTail;

	// Calculate available bytes to read
	uint32_t available;
	if(tmpHead >= tmpTail) {
		available = tmpHead - tmpTail;
	}
	else {
		available = rxBufferSize + tmpHead - tmpTail;
	}

	if(available == 0) {
		return 0;
	}

	uint16_t toRead = (available > maxLen) ? maxLen : available;

	// Copy data
	if(tmpHead >= tmpTail) {
		memcpy(buf, &rxBuffer[tmpTail], toRead);
		rxBufTail = rxBufTail + toRead;
	}
	else {
		uint16_t firstPart = rxBufferSize - tmpTail;

		if(toRead <= firstPart) {
			memcpy(buf, &rxBuffer[tmpTail], toRead);
			rxBufTail = rxBufTail + toRead;
		}
		else {
			uint16_t secondPart = toRead - firstPart;
			memcpy(buf, &rxBuffer[tmpTail], firstPart);
			memcpy(buf + firstPart, &rxBuffer[0], secondPart);
			rxBufTail = secondPart;
		}
	}

	return toRead;
}

uint32_t UART::Available() {
	uint32_t tmpHead = rxBufHead;
	if(tmpHead >= rxBufTail) {
		return (tmpHead - rxBufTail);
	}
	return (rxBufferSize + tmpHead - rxBufTail);
}

void UART::StartTX() {
	if(txBufHead == txBufTail) {
		txBusy = false;
		return;
	}

	if(LL_USART_IsEnabledIT_TXE_TXFNF(instance) == 0x00) {
		// LL_USART_TransmitData8(instance, txBuffer[txBufTail]);
		// txBufTail = (txBufTail + 1) % txBufferSize;
		LL_USART_EnableIT_TXE_TXFNF(instance);
	}
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

void UART::InterruptHandler() {
	// Handle ORE
	if(LL_USART_IsActiveFlag_ORE(instance) == 0x01) {
		// Clear ORE flag
		LL_USART_ClearFlag_ORE(instance);
	}
	// Handle FE
	if(LL_USART_IsActiveFlag_FE(instance) == 0x01) {
		// Clear FE flag
		LL_USART_ClearFlag_FE(instance);
	}
	// Handle NE
	if(LL_USART_IsActiveFlag_NE(instance) == 0x01) {
		// Clear NE flag
		LL_USART_ClearFlag_NE(instance);
	}

	// Handle TXE TXFNF
	if(LL_USART_IsEnabledIT_TXE_TXFNF(instance) == 0x01 && LL_USART_IsActiveFlag_TXE_TXFNF(instance) == 0x01) {
		if(txBufHead != txBufTail) {
			// Have bytes in buffer, write/send
			LL_USART_TransmitData8(instance, txBuffer[txBufTail]);
			txBufTail = (txBufTail + 1) % txBufferSize;
		}
		else {
			// Buffer empty
			LL_USART_DisableIT_TXE_TXFNF(instance);
			txBusy = false;
		}
	}

	// Handle RXNE RXFNE
	if(LL_USART_IsActiveFlag_RXNE_RXFNE(instance) == 0x01) {
		uint8_t byte = LL_USART_ReceiveData8(instance);

		uint16_t nextHead = (rxBufHead + 1) % rxBufferSize;
		if(nextHead != rxBufTail) {
			rxBuffer[rxBufHead] = byte;
			rxBufHead = nextHead;
		}
		
		// if(rxLength != 0x00) {
		// 	// RX Buffer full, has a complete frame in it
		// 	LL_USART_ReceiveData8(instance);
		// }
		// else if(rxIndex >= rxBufferSize) {
		// 	//RX Buffer overflow
		// 	LL_USART_ReceiveData8(instance);

		// 	rxIndex = 0;
		// }
		// else {
		// 	// All good, read received byte to RX buffer
		// 	rxBuffer[rxIndex++] = LL_USART_ReceiveData8(instance);
		// }
	}

	// Handle IDLE
	if(LL_USART_IsActiveFlag_IDLE(instance) == 0x01) {
		// End of frame transmission, detected by receiver timeout
		// rxLength = rxIndex;

		// Call callback function
		// if(OnRXComplete != nullptr) {
		// 	OnRXComplete();
		// }

		// Clear IDLE flag
		LL_USART_ClearFlag_IDLE(instance);
	}
}
