/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/i2c.cpp
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
	if(instance == USART1) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
		irqCall = USART1_IRQn;
	}
	else if(instance == USART2) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
		irqCall = USART2_IRQn;
	}
	else if(instance == USART3) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
		irqCall = USART3_IRQn;
	}
	else if(instance == UART4) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
		irqCall = UART4_IRQn;
	}
	else if(instance == UART5) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
		irqCall = UART5_IRQn;
	}
	else if(instance == USART6) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);
		irqCall = USART6_IRQn;
	}
	else if(instance == UART7) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART7);
		irqCall = UART7_IRQn;
	}
	else if(instance == UART8) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART8);
		irqCall = UART8_IRQn;
	}
	else if(instance == UART9) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_UART9);
		irqCall = UART9_IRQn;
	}
	else if(instance == USART10) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART10);
		irqCall = USART10_IRQn;
	}
	else {
		return Status::Error;
	}

	// Configure UART Interface
	LL_USART_SetTransferDirection(instance, LL_USART_DIRECTION_TX_RX);
	LL_USART_SetDataWidth(instance, static_cast<uint32_t>(config.dataBits));
	LL_USART_SetParity(instance, static_cast<uint32_t>(config.parity));
	LL_USART_SetStopBitsLength(instance, static_cast<uint32_t>(config.stopBits));
	if(config.hwFlowControl == 0x00) {
		LL_USART_SetHWFlowCtrl(instance, LL_USART_HWCONTROL_NONE);
	} 
	else {
		LL_USART_SetHWFlowCtrl(instance, LL_USART_HWCONTROL_RTS_CTS);
	}
	LL_USART_SetOverSampling(instance, LL_USART_OVERSAMPLING_16);
	LL_USART_SetBaudRate(instance, config.sourceClockHz, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, config.baudrate);
	LL_USART_SetPrescaler(instance, LL_USART_PRESCALER_DIV1);
	LL_USART_SetTXFIFOThreshold(instance, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_SetRXFIFOThreshold(instance, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_DisableFIFO(instance);
	LL_USART_ConfigAsyncMode(instance);

	// Configure UART Interrupts
	NVIC_SetPriority(irqCall, irqPriority);
	NVIC_EnableIRQ(irqCall);
	LL_USART_EnableIT_RXNE_RXFNE(instance);
	LL_USART_EnableIT_IDLE(instance);

	LL_USART_Enable(instance);
	// Wait for Init to finish
	while((!(LL_USART_IsActiveFlag_TEACK(instance))) || (!(LL_USART_IsActiveFlag_REACK(instance))));

	// Flush backlog if any
	//UINT state = tx_interrupt_control(TX_INT_DISABLE);
	if(txBusy == false) {
		StartTX();
	}
	//tx_interrupt_control(state);

	this->isInitialized = true;
	return Status::Ok;
}

Status UART::Write(uint8_t *buf, uint16_t len) {
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
	// UINT state = tx_interrupt_control(TX_INT_DISABLE);
	txBufHead = tmpHead;

	if(this->isInitialized == true && txBusy == false) {
		txBusy = true;
		StartTX();
	}
	// End of critical section
	// tx_interrupt_control(state);

	// Release UART device
	if(useLock == true) {
		tx_mutex_put(&mutex);
	}

	return Status::Ok;
}

uint32_t UART::Read(uint8_t *buf, uint32_t maxLen) {
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
	// TX Handling
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

	// RX Handling
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

	//
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
