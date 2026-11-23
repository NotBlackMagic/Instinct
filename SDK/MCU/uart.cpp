/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	SDK/MCU/uart.cpp
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

#if defined (USE_RTOS)
	// Create RTOS objects
	if(tx_mutex_create(&mutex, const_cast<char*>("uart mutex"), TX_INHERIT) != TX_SUCCESS) {
		return Status::Error;
	}
#endif

	this->config = config;
	
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

	// Disable peripheral and clear to reset state (required for when come from FSBL or BootROM)
	LL_USART_Disable(this->instance);
	while(LL_USART_IsEnabled(this->instance) != 0);
	LL_USART_DeInit(this->instance);

	// Configure UART Interface
	LL_USART_SetTransferDirection(this->instance, LL_USART_DIRECTION_TX_RX);
	LL_USART_SetDataWidth(this->instance, static_cast<uint32_t>(config.dataBits));
	LL_USART_SetParity(this->instance, static_cast<uint32_t>(config.parity));
	LL_USART_SetStopBitsLength(this->instance, static_cast<uint32_t>(config.stopBits));
	if(config.hwFlowControl == false) {
		LL_USART_SetHWFlowCtrl(this->instance, LL_USART_HWCONTROL_NONE);
	} 
	else {
		LL_USART_SetHWFlowCtrl(this->instance, LL_USART_HWCONTROL_RTS_CTS);
	}
	if(config.swapTxRx == true) {
		LL_USART_SetTXRXSwap(this->instance, LL_USART_TXRX_SWAPPED);
	}
	else {
		LL_USART_SetTXRXSwap(this->instance, LL_USART_TXRX_STANDARD);
	}
	LL_USART_SetOverSampling(this->instance, LL_USART_OVERSAMPLING_16);
	LL_USART_SetBaudRate(this->instance, config.sourceClockHz, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, config.baudrate);
	LL_USART_SetPrescaler(this->instance, LL_USART_PRESCALER_DIV1);
	LL_USART_DisableAutoBaudRate(this->instance);
	LL_USART_SetTXFIFOThreshold(this->instance, LL_USART_FIFOTHRESHOLD_1_2);
	LL_USART_SetRXFIFOThreshold(this->instance, LL_USART_FIFOTHRESHOLD_1_2);
	LL_USART_EnableFIFO(this->instance);
	LL_USART_ConfigAsyncMode(this->instance);

	// Clear any stale flags
	WRITE_REG(this->instance->ICR, USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_PECF | USART_ICR_NECF | USART_ICR_IDLECF);

	// Configure UART Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);
	// LL_USART_EnableIT_RXNE_RXFNE(this->instance);
	LL_USART_EnableIT_RXFT(this->instance);
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

Status UART::SetBaudrate(uint32_t baudrate) {
	if(this->isInitialized == false || baudrate == 0 || this->config.sourceClockHz == 0) {
		return Status::Error;
	}

	// Lock UART device
#if defined (USE_RTOS)
	bool useLock = this->isInitialized && (tx_thread_identify() != nullptr);
	if(useLock == true) {
		if(tx_mutex_get(&mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
			return Status::Timeout;
		}
	}
#endif

	// Flush TX buffers, write all before baudrate change
	Flush();

	// Disable IRQ while peripheral is disabled
	NVIC_DisableIRQ(this->irqCall);
	__DSB();
	__ISB();

	//Disable peripheral
	LL_USART_Disable(this->instance);
	while(LL_USART_IsEnabled(this->instance) != 0);

	LL_USART_SetBaudRate(this->instance, this->config.sourceClockHz, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, baudrate);
	LL_USART_ClearFlag_ORE(this->instance);
	LL_USART_ClearFlag_FE(this->instance);
	LL_USART_ClearFlag_NE(this->instance);

	// Update config
	this->config.baudrate = baudrate;

	// Re-enable peripheral
	LL_USART_Enable(this->instance);
	// Wait for Init to finish
	while((!(LL_USART_IsActiveFlag_TEACK(this->instance))) || (!(LL_USART_IsActiveFlag_REACK(this->instance))));

	NVIC_EnableIRQ(this->irqCall);

#if defined (USE_RTOS)
	if(useLock == true) {
		tx_mutex_put(&mutex);
	}
#endif

	return Status::Ok;
}

Status UART::Transmit(uint8_t *buf, uint16_t len) {
	if(buf == nullptr || len == 0) {
		return Status::Error;
	}

	// Lock UART device
#if defined (USE_RTOS)
	bool useLock = this->isInitialized && (tx_thread_identify() != nullptr);
	if(useLock == true) {
		if(tx_mutex_get(&mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
			return Status::Timeout;
		}
	}
#endif

	uint16_t tmpHead = txBufHead;
	uint16_t tmpTail = txBufTail;

	// Calculate free space in ring buffer (1 slot reserved to distinguish full vs empty)
	uint16_t freeSpace;
	if(tmpHead >= tmpTail) {
		freeSpace = (txBufferSize - 1) - (tmpHead - tmpTail);
	}
	else {
		freeSpace = (tmpTail - tmpHead) - 1;
	}

	// Reject write if payload cannot fit
	if(len > freeSpace) {
		this->stats.txDrops = this->stats.txDrops + 1;
#if defined (USE_RTOS)
		if(useLock == true) {
			tx_mutex_put(&mutex);
		}
#endif
		return Status::Error;
	}

	// Buffer copy (to internal ring buffer)
	uint16_t firstPart = txBufferSize - tmpHead;
	if(len < firstPart) {
		memcpy(&txBuffer[tmpHead], buf, len);
		tmpHead = tmpHead + len;
	}
	else if(len == firstPart) {
		memcpy(&txBuffer[tmpHead], buf, len);
		tmpHead = 0;
	}
	else {
		uint16_t secondPart = len - firstPart;
		memcpy(&txBuffer[tmpHead], buf, firstPart);
		memcpy(&txBuffer[0], buf + firstPart, secondPart);
		tmpHead = secondPart;
	}

	// Critical section!
#if defined (USE_RTOS)
	UINT state = tx_interrupt_control(TX_INT_DISABLE);
#endif
	txBufHead = tmpHead;

	if(this->isInitialized == true && txBusy == false) {
		StartTX();
	}
	// End of critical section
#if defined (USE_RTOS)
	tx_interrupt_control(state);

	// Release UART device
	if(useLock == true) {
		tx_mutex_put(&mutex);
	}
#endif

	return Status::Ok;
}

uint32_t UART::Receive(uint8_t *buf, uint32_t maxLen) {
	if(buf == nullptr || maxLen == 0) {
		return 0;
	}

	uint32_t tmpHead = rxBufHead;
	uint32_t tmpTail = rxBufTail;

	// Calculate available bytes to read
	uint32_t available;
	if(tmpHead >= tmpTail) {
		available = static_cast<uint32_t>(tmpHead - tmpTail);
	}
	else {
		available = static_cast<uint32_t>(rxBufferSize - tmpTail) + static_cast<uint32_t>(tmpHead);
	}

	if(available == 0) {
		return 0;
	}

	uint32_t toRead = (available > maxLen) ? maxLen : available;
	uint16_t firstPart = rxBufferSize - tmpTail;
	uint16_t newTail;

	// Copy data
	if(toRead < firstPart) {
		memcpy(buf, &rxBuffer[tmpTail], toRead);
		newTail = static_cast<uint16_t>(tmpTail + toRead);
	}
	else if(toRead == firstPart) {
		memcpy(buf, &rxBuffer[tmpTail], toRead);
		newTail = 0;
	}
	else {
		uint32_t secondPart = toRead - firstPart;
		memcpy(buf, &rxBuffer[tmpTail], firstPart);
		memcpy(buf + firstPart, &rxBuffer[0], secondPart);
		newTail = static_cast<uint16_t>(secondPart);
	}

	// Ensure memcpy completes before advancing tail for the ISR
	__asm volatile("" ::: "memory");
	rxBufTail = newTail;

	this->stats.rxBytes = this->stats.rxBytes + toRead;
	return toRead;
}

void UART::Flush(void) {
	// Wait until software ring buffer has drained via ISR
	while(this->txBusy || (this->txBufHead != this->txBufTail)) {
		__NOP();
	}
	// Wait until hardware shift register finishes transmitting (TC = 1)
	while(LL_USART_IsActiveFlag_TC(this->instance) == 0) {
		__NOP();
	}
}

uint32_t UART::Available() {
	uint16_t tmpHead = rxBufHead;
	uint16_t tmpTail = rxBufTail;

	if(tmpHead >= tmpTail) {
		return static_cast<uint32_t>(tmpHead - tmpTail);
	}
	return static_cast<uint32_t>(rxBufferSize - tmpTail) + static_cast<uint32_t>(tmpHead);
}

void UART::StartTX() {
	uint16_t head = txBufHead;
	uint16_t tail = txBufTail;

	if(head == tail) {
		txBusy = false;
		return;
	}

	// Pre-fill TX FIFO
	while((head != tail) && ((instance->ISR & USART_ISR_TXE_TXFNF) != 0U)) {
		instance->TDR = txBuffer[tail];
		tail = tail + 1;
		if(tail >= txBufferSize) {
			tail = 0;
		}
	}

	__asm volatile("" ::: "memory");
	txBufTail = tail;

	// Only arm TXFT interrupt if bytes left to transmit
	if(head != tail) {
		txBusy = true;
		if(LL_USART_IsEnabledIT_TXFT(instance) == 0x00) {
			LL_USART_EnableIT_TXFT(instance);
		}
	}
	else {
		txBusy = false;
	}
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

void UART::InterruptHandler() {
	uint32_t isr = READ_REG(instance->ISR);
	uint32_t cr1 = READ_REG(instance->CR1);
	uint32_t cr3 = READ_REG(instance->CR3);

	// Clear IDLE flag (only used to weak ISR to drain trailing bytes below RXFT threshold)
	if((isr & USART_ISR_IDLE) != 0U) {
		// Clear IDLE flag
		LL_USART_ClearFlag_IDLE(instance);
	}

	// Handle RXNE RXFNE, draining RF FIFO
	if((isr & USART_ISR_RXNE_RXFNE) != 0U) {
		uint16_t head = rxBufHead;
		uint16_t tail = rxBufTail;

		do {
			// Check for errors, as each FIFO entry comes with its OWN error status
			constexpr uint32_t errorMask = USART_ISR_ORE | USART_ISR_FE | USART_ISR_PE | USART_ISR_NE;
			uint32_t errors = isr & errorMask;
			if(errors != 0U) {
				this->stats.hwErrors = this->stats.hwErrors + 1;

				uint32_t icr = 0U;
				if((errors & USART_ISR_ORE) != 0U) {
					icr |= USART_ICR_ORECF;
				}
				if((errors & USART_ISR_FE) != 0U) {
					icr |= USART_ICR_FECF;
				}
				if((errors & USART_ISR_PE) != 0U) {
					icr |= USART_ICR_PECF;
				}
				if((errors & USART_ISR_NE) != 0U) {
					icr |= USART_ICR_NECF;
				}
				// Clear error flags
				WRITE_REG(instance->ICR, icr);
			}

			uint8_t byte = static_cast<uint8_t>(instance->RDR);

			// Discard bytes flagged with FE/PE/NE. ORE byte at top of FIFO is still valid
			if((errors & (USART_ISR_FE | USART_ISR_PE | USART_ISR_NE)) == 0U) {
				uint16_t nextHead = head + 1;
				if(nextHead >= rxBufferSize) {
					nextHead = 0;
				}

				if(nextHead != tail) {
					rxBuffer[head] = byte;
					head = nextHead;
				}
				else {
					this->stats.rxDrops = this->stats.rxDrops + 1;
				}
			}

			// Read error status for next FIFO entry
			isr = READ_REG(instance->ISR);
		} while((isr & USART_ISR_RXNE_RXFNE) != 0U);

		// Ensure buffer writes complete before publishing new head to Receive()
		__asm volatile("" ::: "memory");
		rxBufHead = head;
	}
	else {
		// Safety clear in case an error flag was raised with an empty FIFO
		constexpr uint32_t errorMask = USART_ISR_ORE | USART_ISR_FE | USART_ISR_PE | USART_ISR_NE;
		if((isr & errorMask) != 0U) {
			this->stats.hwErrors = this->stats.hwErrors + 1;
			instance->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_PECF | USART_ICR_NECF;
		}
	}

	// Handle TXFTIE, filling TX FIFO
	if(((cr3 & USART_CR3_TXFTIE) != 0U) && ((isr & USART_ISR_TXFT) != 0U)) {
		uint16_t head = txBufHead;
		uint16_t tail = txBufTail;

		while((head != tail) && ((instance->ISR & USART_ISR_TXE_TXFNF) != 0U)) {
			instance->TDR = txBuffer[tail];
			tail = tail + 1;
			if(tail >= txBufferSize) {
				tail = 0;
			}
		}

		__asm volatile("" ::: "memory");
		txBufTail = tail;

		if(head == tail) {
			LL_USART_DisableIT_TXFT(instance);
			txBusy = false;
		}
	}
}
