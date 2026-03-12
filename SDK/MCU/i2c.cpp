/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/i2c.cpp
 */

//Reserved I2C Addresses List (https://www.ti.com/content/dam/videos/external-videos/en-us/8/3816841626001/6241024036001.mp4/subassets/adcs-introduction-to-i2c-reserved-addresses-presentation.pdf)
//0x00: (R/Wn = 0) General call address
//0x00: (R/Wn = 1) Start byte
//0x01: (R/Wn = X) CBUS address
//0x02: (R/Wn = X) Reserved for different bus format
//0x03: (R/Wn = X) Reserved for future purposes
//0x04-0x07: (R/Wn = X) HS-mode controller code
//0x78-0x7B: (R/Wn = X) 10-bit target addressing
//0x7C-0x7F: (R/Wn = 1) Device ID

#include "i2c.hpp"

I2C::I2C(I2C_TypeDef *instance) {
	this->instance = instance;
	this->irqPriority = 0x0E; // Lowest priority (safe default)
}

Status I2C::Init(const Config &config) {
	if(this->isInitialized == true) {
		return Status::Ok;
	}

	// Create RTOS objects
	if(tx_mutex_create(&this->mutex, const_cast<char*>("i2c mutex"), TX_INHERIT) != TX_SUCCESS) {
		return Status::Error;
	}
	if(tx_event_flags_create(&this->event, const_cast<char*>("i2c event")) != TX_SUCCESS) {
		tx_mutex_delete(&this->mutex);
		return Status::Error;
	}

	// Enable bus clocks and identify IRQ lines
	if(this->instance == I2C1) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
		this->irqCall = I2C1_EV_IRQn;
	}
	else if(this->instance == I2C2) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
		this->irqCall = I2C2_EV_IRQn;
	}
	else if(this->instance == I2C3) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);
		this->irqCall = I2C3_EV_IRQn;
	}
	else if(this->instance == I2C4) {
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_I2C4);
		this->irqCall = I2C4_EV_IRQn;
	}
	else {
		return Status::Error;
	}

	// Configure I2C Interface
	// Configure the SDA setup, hold time and the SCL high, low period
	switch (config.mode) {
		case I2C::Mode::Standard:
			//I2C Standard Mode (100kHz)
			LL_I2C_SetTiming(this->instance, 0x10C0ECFF);		//100MHz Clock
			// LL_I2C_SetTiming(I2C1, 0x30C0EDFF);	//200MHz Clock
			break;
		case I2C::Mode::Fast:
			//I2C Fast Mode (400kHz)
			LL_I2C_SetTiming(this->instance, 0x009034B6);		//100MHz Clock
			// LL_I2C_SetTiming(I2C1, 0x109035B7);	//200MHz Clock
			break;
		case I2C::Mode::FastPlus:
			//I2C Fast Mode Plus (1MHz)
			LL_I2C_SetTiming(this->instance, 0x00401242);		//100MHz Clock
			// LL_I2C_SetTiming(I2C1, 0x00902787);	//200MHz Clock
			break;
	}
//	LL_I2C_SetOwnAddress1(this->instance, 0x00, LL_I2C_OWNADDRESS1_7BIT);	//Reset Values of: OwnAddress1 is 0x00; OwnAddrSize is LL_I2C_OWNADDRESS1_7BIT
//	LL_I2C_EnableOwnAddress1(this->instance);								//Reset Value is Own Address1 is disabled
//	LL_I2C_SetOwnAddress2(this->instance, 0x00, LL_I2C_OWNADDRESS2_NOMASK);	//Reset Values of: OwnAddress2 is 0x00; OwnAddrMask is LL_I2C_OWNADDRESS2_NOMASK
//	LL_I2C_DisableOwnAddress2(this->instance);								//Reset Value is Own Address2 is disabled
//	LL_I2C_EnableClockStretching(this->instance);							//Reset Value is Clock stretching enabled
//	LL_I2C_SetDigitalFilter(this->instance, 0x00);							//Reset Value is 0x00
//	LL_I2C_EnableAnalogFilter(this->instance);								//Reset Value is Analog Filter enabled
//	LL_I2C_EnableGeneralCall(this->instance);								//Reset Value is General Call disabled
//	LL_I2C_SetMasterAddressingMode(this->instance, LL_I2C_ADDRESSING_MODE_7BIT);	//Reset Value is LL_I2C_ADDRESSING_MODE_7BIT
//	LL_I2C_SetMode(this->instance, LL_I2C_MODE_I2C);						//Reset Value is I2C mode

	// Configure Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);
//	NVIC_SetPriority(I2C1_ER_IRQn, 1);
//	NVIC_EnableIRQ(I2C1_ER_IRQn);
	LL_I2C_EnableIT_RX(this->instance);
	LL_I2C_EnableIT_NACK(this->instance);
	LL_I2C_EnableIT_STOP(this->instance);
//	LL_I2C_EnableIT_ERR(this->instance);

	// Enable I2C
	LL_I2C_Enable(this->instance);

	this->isInitialized = true;
	return Status::Ok;
}

uint8_t I2C::Probe(uint16_t addr) {
	// Lock I2C device
	if(tx_mutex_get(&this->mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
		return 0x00;
	}

	uint64_t timestamp = Time::GetUs();
	while(true) {
		if(LL_I2C_IsActiveFlag_BUSY(this->instance) == 0x00) {
			break;
		}

		if((Time::GetUs() - timestamp) > TIMEOUT_BUSY_US) {
			// Wait for Busy timeout
			tx_mutex_put(&this->mutex);
			return 0x00;
		}
	}

	// Clear event flags
	tx_event_flags_set(&this->event, 0, TX_AND);

	this->address = addr << 1;
	this->txLength = 0;
	this->rxLength = 0;

	// Enable Interrupts
	LL_I2C_EnableIT_NACK(this->instance);
	LL_I2C_EnableIT_STOP(this->instance);

	// Start a 0-byte Write to check for ACK
	LL_I2C_HandleTransfer(this->instance, this->address, LL_I2C_ADDRSLAVE_7BIT, this->txLength, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	// Wait for event (STOP or NACK)
	ULONG events;
	tx_event_flags_get(&this->event, EVT_TRANS_CPLT | EVT_ERR, TX_OR_CLEAR, &events, TX_WAIT_FOREVER);

	uint8_t nack = 0x00;
	if((events & EVT_ERR) == EVT_ERR) {
		nack = 0x01;
	}

	// Disable interrupts again
	LL_I2C_DisableIT_NACK(this->instance);
	LL_I2C_DisableIT_STOP(this->instance);

	// Release I2C device
	if(tx_mutex_put(&this->mutex) != TX_SUCCESS) {
		return 0x00;
	}

	return !nack;
}

Status I2C::TransferAsync(uint16_t addr, const uint8_t *txBuf, uint16_t txLen, uint8_t *rxBuf, uint16_t rxLen) {
	// Prevent overflow of I2C internal transfer length register part
	if(txLen > 255 || rxLen > 255) {
		return Status::Error;
	}
	
	// Try lock I2C device
	if(tx_mutex_get(&this->mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
		return Status::Timeout;
	}

	uint64_t timestamp = Time::GetUs();
	while(true) {
		if(LL_I2C_IsActiveFlag_BUSY(this->instance) == 0x00) {
			break;
		}

		if((Time::GetUs() - timestamp) > TIMEOUT_BUSY_US) {
			// Wait for Busy timeout
			tx_mutex_put(&this->mutex);
			return Status::Busy;
		}
	}

	// Clear event flags
	tx_event_flags_set(&this->event, 0, TX_AND);

	// Prepare internal transfer variables
	this->address = addr << 1;
	this->txBuffer = txBuf;
	this->txLength = txLen;
	this->rxBuffer = rxBuf;
	this->rxLength = rxLen;

	uint32_t transferMode;
	if (this->rxLength > 0 && this->txLength > 0) {
		// TX followed by RX, use  Repeated Start (SoftEnd)
		transferMode = LL_I2C_MODE_SOFTEND;
		LL_I2C_EnableIT_TC(this->instance);
	}
	else {
		// Just TX or Just RX, use AutoEnd
		transferMode = LL_I2C_MODE_AUTOEND;
		LL_I2C_DisableIT_TC(this->instance);
	}

	LL_I2C_EnableIT_NACK(this->instance);
	LL_I2C_EnableIT_STOP(this->instance);
	// LL_I2C_EnableIT_ERR(this->instance);

	if(this->txLength > 0) {
		LL_I2C_HandleTransfer(this->instance, this->address, LL_I2C_ADDRSLAVE_7BIT, this->txLength, transferMode, LL_I2C_GENERATE_START_WRITE);
		LL_I2C_EnableIT_TX(this->instance);
	} 
	else if(this->rxLength > 0) {
		LL_I2C_HandleTransfer(this->instance, this->address, LL_I2C_ADDRSLAVE_7BIT, this->rxLength, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
		LL_I2C_EnableIT_RX(this->instance);
	}

	return Status::Ok;
}

Status I2C::TransferWait(uint32_t timeoutTicks) {
	// Wait for event
	ULONG events;
	UINT status = tx_event_flags_get(&this->event, EVT_TRANS_CPLT | EVT_ERR, TX_OR_CLEAR, &events, timeoutTicks);

	if(status != TX_SUCCESS) {
		this->TransferAbort();
		return Status::Timeout;
	}

	if((events & EVT_ERR) == EVT_ERR) {
		// Transfer error occurred
		this->TransferAbort();
		return Status::Error;
	}

	tx_mutex_put(&this->mutex);
	return Status::Ok;
}

Status I2C::TransferAbort() {
	// Disable interrupts
	LL_I2C_DisableIT_TX(this->instance);
	LL_I2C_DisableIT_RX(this->instance);
	LL_I2C_DisableIT_STOP(this->instance);
	LL_I2C_DisableIT_NACK(this->instance);
	// LL_I2C_DisableIT_ERR(this->instance);

	// Stop current transfer if any
	if (LL_I2C_IsActiveFlag_BUSY(this->instance) == 0x01) {
        LL_I2C_GenerateStopCondition(this->instance);
    }

	// Disable peripheral
	LL_I2C_Disable(this->instance);

	// Clear transfer context
	this->address = 0;
	this->txBuffer = nullptr;
	this->txLength = 0;
	this->rxBuffer = nullptr;
	this->rxLength = 0;

	// Re-enable peripheral
	LL_I2C_Enable(this->instance);

	// Signal event flags and release mutex
	tx_event_flags_set(&this->event, EVT_TRANS_CPLT, TX_OR);
	tx_mutex_put(&this->mutex);

	return Status::Ok;
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

void I2C::InterruptHandler() {
	// Handle NACK
	if(LL_I2C_IsActiveFlag_NACK(this->instance) == 0x01) {
		// NACK received, stop transaction and flag error
		LL_I2C_GenerateStopCondition(this->instance);
		tx_event_flags_set(&this->event, EVT_ERR, TX_OR);
		LL_I2C_ClearFlag_NACK(this->instance);
	}

	// Handle Transmit (TXIS)
	if(LL_I2C_IsEnabledIT_TX(this->instance) == 0x01 && LL_I2C_IsActiveFlag_TXIS(this->instance) == 0x01) {
		if(this->txLength > 0) {
			LL_I2C_TransmitData8(this->instance, *this->txBuffer);
			this->txBuffer = this->txBuffer + 1;
			this->txLength = this->txLength - 1;
		}
	}

	// Handle Receive (RXNE)
	if(LL_I2C_IsActiveFlag_RXNE(this->instance) == 0x01) {
		if(this->rxLength > 0) {
			*this->rxBuffer = LL_I2C_ReceiveData8(this->instance);
			this->rxBuffer = this->rxBuffer + 1;
			this->rxLength = this->rxLength - 1;
		}
	}

	// Handle Transfer Complete (TC)
	if(LL_I2C_IsEnabledIT_TC(this->instance) == 0x01 && LL_I2C_IsActiveFlag_TC(this->instance) == 0x01) {
		if (this->rxLength > 0) {
			// Generate REPEATED START for Reading
			LL_I2C_HandleTransfer(this->instance, this->address, LL_I2C_ADDRSLAVE_7BIT, this->rxLength, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
			
			// Switch to RX interrupts
			LL_I2C_DisableIT_TX(this->instance);
			LL_I2C_EnableIT_RX(this->instance);
		}
		else {
			LL_I2C_DisableIT_TC(this->instance);
		}
	}

	// Handle Stop Condition (End of Transaction)
	if(LL_I2C_IsActiveFlag_STOP(this->instance) == 0x01) {
		LL_I2C_DisableIT_TX(this->instance);
		LL_I2C_DisableIT_RX(this->instance);
		tx_event_flags_set(&this->event, EVT_TRANS_CPLT, TX_OR);
		LL_I2C_ClearFlag_STOP(this->instance);
	}
}