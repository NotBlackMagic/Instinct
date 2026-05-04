/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/i3c.cpp
 */

#include "i3c.hpp"

// Enable Interrupt-based Logic
// #define I3C_USE_IRQ

I3C::I3C(I3C_TypeDef *instance) {
	this->instance = instance;
	this->irqPriority = 0x0E;	// Lowest priority (safe default)
}

Status I3C::Init(const Config &config) {
	// Create RTOS objects
	if(tx_mutex_create(&this->mutex, const_cast<char*>("i3c mutex"), TX_INHERIT) != TX_SUCCESS) {
		return Status::Error;
	}
	if(tx_event_flags_create(&this->event, const_cast<char*>("i3c event")) != TX_SUCCESS) {
		tx_mutex_delete(&this->mutex);
		return Status::Error;
	}

	// Enable bus clocks and identify IRQ lines
	if(this->instance == I3C1) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I3C1);
		this->irqCall = I3C1_EV_IRQn;
		//this->irqErrCall = I3C1_ER_IRQn;
	}
	else if(this->instance == I3C2) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I3C2);
		this->irqCall = I3C2_EV_IRQn;
		//this->irqErrCall = I3C2_ER_IRQn;
	}
	else {
		return Status::Error;
	}

	// Configure I3C Interface
	LL_I3C_SetMode(this->instance, LL_I3C_MODE_CONTROLLER);
	// Configure the SDA setup, hold time and the SCL high, low period
	// Timing values for CLOCK = 100MHz
	switch(config.mode) {
		case I3C::Mode::Legacy_Fast:
			//Only legacy I2C Fast Mode (400kHz)
			//I2C DC: 30%, I3C DC: 50%, SCL OD Freq: 559 KHz, SCL PP Freq: 12500 KHz
			LL_I3C_ConfigClockWaveForm(this->instance, 0x4AAE0303);
			//SDA Hold Time (cycles): 0.5, Bus Free (ns): 2115, Bus Idle (ns): 200'000
			LL_I3C_SetCtrlBusCharacteristic(this->instance, 0x00690062);
			break;
		case I3C::Mode::Mixed_Fast:
			//Mixed legacy I2C Fast Mode (400kHz) and I3C SDR at 12.5MHz
			//I2C DC: 30%, I3C DC: 50%, SCL OD Freq: 559 KHz, SCL PP Freq: 12500 KHz
			LL_I3C_ConfigClockWaveForm(this->instance, 0x4AAE0303);
			//SDA Hold Time (cycles): 0.5, Bus Free (ns): 2115, Bus Idle (ns): 200'000
			LL_I3C_SetCtrlBusCharacteristic(this->instance, 0x00690062);
			break;
		case I3C::Mode::Mixed_FastPlus:
			//Mixed legacy I2C Fast Mode Plus (1MHz) and I3C SDR at 12.5MHz
			//I2C DC: 30%, I3C DC: 50%, SCL OD Freq: 1351 KHz, SCL PP Freq: 12500 KHz
			LL_I3C_ConfigClockWaveForm(this->instance, 0x1D450303);
			//SDA Hold Time (cycles): 0.5, Bus Free (ns): 1055, Bus Idle (ns): 200'000
			LL_I3C_SetCtrlBusCharacteristic(this->instance, 0x00340062);
			break;
		case I3C::Mode::Pure_I3C_SDR:
			//No legacy I2C, only I3C SDR at 12.5MHz
			//I3C DC: 50%, SCL OD Freq: 2439 KHz, SCL PP Freq: 12500 KHz
			LL_I3C_ConfigClockWaveForm(this->instance, 0x00240303);
			//SDA Hold Time (cycles): 0.5, Bus Free (ns): 415, Bus Idle (ns): 200'000
			LL_I3C_SetCtrlBusCharacteristic(this->instance, 0x00140062);
			break;
	}
	
	LL_I3C_SetDataHoldTime(this->instance, LL_I3C_SDA_HOLD_TIME_0_5);
	LL_I3C_SetControllerActivityState(this->instance, LL_I3C_OWN_ACTIVITY_STATE_0);
	LL_I3C_DisableHJAck(this->instance);
	//Configure FIFO
	LL_I3C_SetRxFIFOThreshold(this->instance, LL_I3C_RXFIFO_THRESHOLD_1_4);
	LL_I3C_SetTxFIFOThreshold(this->instance, LL_I3C_TXFIFO_THRESHOLD_1_4);
	LL_I3C_DisableControlFIFO(this->instance);
	LL_I3C_DisableStatusFIFO(this->instance);
	LL_I3C_DisableArbitrationHeader(this->instance);
	//Configure controller
	LL_I3C_SetOwnDynamicAddress(this->instance, 0);
	LL_I3C_EnableOwnDynAddress(this->instance);
	LL_I3C_SetStallTime(this->instance, 0x00);
	LL_I3C_DisableStallACK(this->instance);
	LL_I3C_DisableStallParityCCC(this->instance);
	LL_I3C_DisableStallParityData(this->instance);
	LL_I3C_DisableStallTbit(this->instance);
	LL_I3C_DisableHighKeeperSDA(this->instance);

#ifdef I3C_USE_IRQ
	// Configure Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);
	LL_I3C_EnableIT_FC(this->instance);
	LL_I3C_EnableIT_CFNF(this->instance);
	LL_I3C_EnableIT_RXFNE(this->instance);
	LL_I3C_EnableIT_TXFNF(this->instance);
	// LL_I3C_EnableIT_ERR(this->instance);
#endif

	// Enable I3C
	LL_I3C_Enable(this->instance);

	return Status::Ok;
}

uint8_t I3C::AssignDynamicAddress() {
	// Try lock I2C device
	if(tx_mutex_get(&this->mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
		return 0x00;
	}

	// Initiate a Dynamic Address Assignment to the Target connected on the bus
	// Controller Generate Start condition for a write request with a Broadcast ENTDAA:
	// - to the Targets connected on the bus
	// - with an auto stop condition generation when all Targets answer the ENTDAA sequence.

	LL_I3C_ControllerHandleCCC(this->instance, static_cast<uint8_t>(I3C::BroadcastCCC::EnterDAA), 0, LL_I3C_GENERATE_STOP);

	do {
		// If the I3C HW request a TX data, retrieve the Target Payload then send an associated Dynamic Address
		if(LL_I3C_IsActiveFlag_TXFNF(this->instance) == 0x01) {
			// Get Target Payload, then assign an associated Dynamic Address
			// Retrieve Target Payload
			uint64_t targetPayload;

			// Check on the Rx FIFO threshold to know the Dynamic Address Assignment treatment process : byte or word
			if(LL_I3C_GetRxFIFOThreshold(this->instance) == LL_I3C_RXFIFO_THRESHOLD_1_4) {
				// Byte treatment process
				uint8_t i;
				for(i = 0; i < 8; i++) {
					targetPayload |= (uint64_t)((uint64_t)LL_I3C_ReceiveData8(this->instance) << (i * 8));
				}
			}
			else {
				// Word treatment process
				targetPayload = (uint64_t)LL_I3C_ReceiveData32(this->instance);
				targetPayload |= (uint64_t)((uint64_t)LL_I3C_ReceiveData32(this->instance) << 32);
			}

			// Store Payload in aTargetDesc
			// aTargetDesc[uwTargetCount]->TARGET_BCR_DCR_PID = targetPayload;

			// Send associated dynamic address
			// Write device address in the TDR register
			// Increment Target counter
			// LL_I3C_TransmitData8(this->instance, aTargetDesc[uwTargetCount++]->DYNAMIC_ADDR);
		}

	} while((READ_REG(this->instance->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF)) == 0x00);

	// Clear frame complete flag
	if(LL_I3C_IsActiveFlag_FC(this->instance) == 0x01) {
		LL_I3C_ClearFlag_FC(this->instance);
	}

	// Check for errors
	if(LL_I3C_IsActiveFlag_ERR(this->instance) == 0x01) {
		LL_I3C_ClearFlag_ERR(this->instance);
	}

	// Release I2C device
	if(tx_mutex_put(&this->mutex) != TX_SUCCESS) {
		return 0x00;
	}

	return 0x00;
}

Status I3C::SendCommandAsync(BroadcastCCC ccc, uint8_t *txBuf, uint16_t txLen) {
	// Try lock I2C device
	if(tx_mutex_get(&this->mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
		return Status::Timeout;
	}

	// Prepare internal transfer variables
	this->txBuffer = txBuf;
	this->txLength = txLen;
	this->rxLength = 0;

	uint32_t timeoutUs = 25;
	uint64_t timestamp = Time::GetUs();
	while(true) {
		if(LL_I3C_IsActiveFlag_CFNF(this->instance) == 0x00) {
			break;
		}

		if((Time::GetUs() - timestamp) > timeoutUs) {
			// Wait for Busy timeout
			tx_mutex_put(&this->mutex);
			return Status::Busy;
		}
	}

	LL_I3C_ControllerHandleCCC(this->instance, static_cast<uint8_t>(ccc), this->txLength, LL_I3C_GENERATE_STOP);

	do {
		// Transmit Common Command Code Associated data if any
		if((this->txLength > 0) && (LL_I3C_IsActiveFlag_TXFNF(this->instance) == 0x01)) {
			LL_I3C_TransmitData8(this->instance, *this->txBuffer);
			this->txBuffer++;
			this->txLength -= 1;
		}
	} while((READ_REG(this->instance->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF)) == 0x00);

	// Start Transfer CCC
	// LL_I3C_RequestTransfer(this->instance);

	// do {
	// 	// Write message into CR register
	// 	if(LL_I3C_IsActiveFlag_CFNF(this->instance) == 0x01) {
	// 		// WRITE_REG(this->instance->CR, uwCCCMessage[ubNbCCC++]);
	// 	}

	// 	// Receive Common Command Code Associated data if any
	// 	if(LL_I3C_IsActiveFlag_RXFNE(this->instance) == 0x01) {
	// 		//aRxBuffer[ubNbRxData++] = LL_I3C_ReceiveData8(this->instance);
	// 	}

	// 	// Transmit Common Command Code Associated data if any
	// 	if((ubNbTxDataToTransfer > 0) && (LL_I3C_IsActiveFlag_TXFNF(this->instance))) {
	// 		// LL_I3C_TransmitData8(this->instance, aTxBuffer[ubNbTxData++]);
	// 		// ubNbTxDataToTransfer--;
	// 	}
	// } while((READ_REG(this->instance->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF)) == 0x00);

	// Clear frame complete flag
	if(LL_I3C_IsActiveFlag_FC(this->instance) == 0x01) {
		LL_I3C_ClearFlag_FC(this->instance);
	}

	// Check for errors
	if(LL_I3C_IsActiveFlag_ERR(this->instance) == 0x01) {
		LL_I3C_ClearFlag_ERR(this->instance);
	}

	// Release I2C device
	if(tx_mutex_put(&this->mutex) != TX_SUCCESS) {
		return Status::Error;
	}

	return Status::Ok;
}

Status I3C::SendCommandAsync(DirectCCC ccc, uint8_t addr, uint8_t *buf, uint16_t len) {
	// Try lock I2C device
	if(tx_mutex_get(&this->mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
		return Status::Timeout;
	}

	bool isReadCmd = IsReadCommand(ccc);

	// Prepare internal transfer variables
	this->address = addr << 1;
	if(isReadCmd == true) {
		this->rxBuffer = buf;
		this->rxLength = len;
		this->txLength = 0;
	}
	else {
		this->txBuffer = buf;
		this->txLength = len;
		this->rxLength = 0;
	}

	return Status::Ok;
}

Status I3C::TransferAsync(uint8_t addr, TargetType type, uint8_t *txBuf, uint16_t txLen, uint8_t *rxBuf, uint16_t rxLen) {
	// Try lock I2C device
	if(tx_mutex_get(&this->mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
		return Status::Timeout;
	}

	// Clear event flags
	tx_event_flags_set(&this->event, 0, TX_AND);

	// Prepare internal transfer variables
	this->address = addr;
	this->txBuffer = txBuf;
	this->txLength = txLen;
	this->rxBuffer = rxBuf;
	this->rxLength = rxLen;

	//
	uint32_t direction = LL_I3C_DIRECTION_WRITE;
	uint32_t endMode; // Helper to decide between STOP or RESTART
	if(this->txLength > 0) {
		direction = LL_I3C_DIRECTION_WRITE;

		if(this->rxLength > 0) {
			endMode = LL_I3C_GENERATE_RESTART;
		}
		else {
			endMode = LL_I3C_GENERATE_STOP;
		}

		if(type == I3C::TargetType::I2C) {
			LL_I3C_ControllerHandleMessage(this->instance, this->address, this->txLength, direction, LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C, endMode);
		}
		else {
			LL_I3C_ControllerHandleMessage(this->instance, this->address, this->txLength, direction, LL_I3C_CONTROLLER_MTYPE_DIRECT, endMode);
		}

		// Fill FIFO until full or no more data
		while (this->txLength > 0 && LL_I3C_IsActiveFlag_TXFNF(this->instance)) {
			LL_I3C_TransmitData8(this->instance, *this->txBuffer);
			this->txBuffer = this->txBuffer + 1;
			this->txLength = this->txLength - 1;
		}

		LL_I3C_RequestTransfer(this->instance);

		// LL_I3C_EnableIT_TXFNF(this->instance);
		do {
			if(this->txLength > 0 && LL_I3C_IsActiveFlag_TXFNF(this->instance) == 0x01) {
				LL_I3C_TransmitData8(this->instance, *this->txBuffer);
				this->txBuffer = this->txBuffer + 1;
				this->txLength = this->txLength - 1;
			}
		} while((READ_REG(this->instance->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF)) == 0U);

		if(LL_I3C_IsActiveFlag_ERR(this->instance) == 0x01) {
			volatile uint32_t errorStatus = READ_REG(this->instance->SER);
			LL_I3C_ClearFlag_ERR(this->instance);

			// Flush FIFO
			LL_I3C_RequestTxFIFOFlush(this->instance);
			LL_I3C_RequestRxFIFOFlush(this->instance);

			tx_event_flags_set(&this->event, EVT_ERR, TX_OR);
			return Status::Error;
		}

		// Clear flags
		LL_I3C_ClearFlag_FC(this->instance);
	}

	if(this->rxLength > 0) {
		direction = LL_I3C_DIRECTION_READ;

		// Read phase always ends with STOP in this logic
		endMode = LL_I3C_GENERATE_STOP;

		if(type == I3C::TargetType::I2C) {
			LL_I3C_ControllerHandleMessage(this->instance, this->address, this->rxLength, direction, LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C, endMode);
		}
		else {
			LL_I3C_ControllerHandleMessage(this->instance, this->address, this->rxLength, direction, LL_I3C_CONTROLLER_MTYPE_DIRECT, endMode);
		}

		LL_I3C_RequestTransfer(this->instance);

		do {
			if(LL_I3C_IsActiveFlag_RXFNE(this->instance) == 0x01) {
				*this->rxBuffer = LL_I3C_ReceiveData8(this->instance);
				this->rxBuffer = this->rxBuffer + 1;
				this->rxLength = this->rxLength - 1;
			}
		} while((READ_REG(this->instance->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF)) == 0U);

		if(LL_I3C_IsActiveFlag_ERR(this->instance) == 0x01) {
			volatile uint32_t errorStatus = READ_REG(this->instance->SER);
			LL_I3C_ClearFlag_ERR(this->instance);

			// Flush FIFO
			LL_I3C_RequestTxFIFOFlush(this->instance);
			LL_I3C_RequestRxFIFOFlush(this->instance);

			tx_event_flags_set(&this->event, EVT_ERR, TX_OR);
			return Status::Error;
		}

		// Clear flags
		LL_I3C_ClearFlag_FC(this->instance);
	}

	tx_event_flags_set(&this->event, EVT_TRANS_CPLT, TX_OR);

	return Status::Ok;
}

Status I3C::TransferWait(uint32_t timeoutTicks) {
	// Wait for event
	ULONG events;
	UINT status = tx_event_flags_get(&this->event, EVT_TRANS_CPLT | EVT_ERR, TX_OR_CLEAR, &events, timeoutTicks);

	// Release I2C device
	tx_mutex_put(&this->mutex);

	if(status != TX_SUCCESS) {
		return Status::Timeout;
	}
	if((events & EVT_ERR) != 0) {
		return Status::Error;
	}
	return Status::Ok;
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

void I3C::InterruptHandler() {
	// Handle Address NACK
	if(LL_I3C_IsActiveFlag_ANACK(this->instance) == 0x01) {
		// NACK received on address
		// tx_event_flags_set(&this->event, EVT_ERR, TX_OR);
	}

	// Handle Control Transfer
	if(LL_I3C_IsEnabledIT_CFNF(this->instance) == 0x01 && LL_I3C_IsActiveFlag_CFNF(this->instance) == 0x01) {
		// Stuff stuff
		// WRITE_REG(this->instance->CR, uwCCCMessage[ubNbCCC++]);
	}

	// Handle Transmit
	if(LL_I3C_IsEnabledIT_TXFNF(this->instance) == 0x01 && LL_I3C_IsActiveFlag_TXFNF(this->instance) == 0x01) {
		//ptrTXFunc(); //I3C_DynamicAddressTreatment or I3C_TransmitByteTreatment
		//For normal transmit mode (I3C_TransmitByteTreatment)
		if(this->txLength > 0) {
			LL_I3C_TransmitData8(this->instance, *this->txBuffer);
			this->txBuffer = this->txBuffer + 1;
			this->txLength = this->txLength - 1;
		}
		//For dynamic address mode (I3C_DynamicAddressTreatment)
		//Store Payload in aTargetDesc
		//aTargetDesc[uwTargetCount]->TARGET_BCR_DCR_PID = targetPayload;
		//Send associated dynamic address: Write device address in the TDR register, Increment Target counter
		//LL_I3C_TransmitData8(this->instance, aTargetDesc[uwTargetCount++]->DYNAMIC_ADDR);
	}

	// Handle Receive
	if(LL_I3C_IsEnabledIT_RXFNE(this->instance) == 0x01 && LL_I3C_IsActiveFlag_RXFNE(this->instance) == 0x01) {
		*this->rxBuffer = LL_I3C_ReceiveData8(this->instance);
		this->rxBuffer = this->rxBuffer + 1;
		this->rxLength = this->rxLength - 1;
	}

	// Handle Frame Complete
	if(LL_I3C_IsEnabledIT_FC(this->instance) == 0x01 && LL_I3C_IsActiveFlag_FC(this->instance) == 0x01) {
		//ubFrameComplete = 1;
		LL_I3C_ClearFlag_FC(this->instance);
	}
}