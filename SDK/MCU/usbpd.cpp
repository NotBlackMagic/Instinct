/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	SDK/MCU/USBPD.cpp
 */

#include "USBPD.hpp"

USBPD::USBPD(UCPD_TypeDef *instance) {
	this->instance = instance;
	this->irqPriority = 0x0E;
	this->isInitialized = false;
	this->isAttached = false;
	this->activeCCLine = CCLine::None;
	this->profileCount = 0;
	this->activePDOIndex = 1;
	this->rxMsgID = 0;
	this->txMsgID = 0;

	// Default legacy USB 5V profile prior to PD negotiation
	this->activeProfile.voltage = 5.0f;
	this->activeProfile.minVoltage = 5.0f;
	this->activeProfile.current = 0.5f;
	this->activeProfile.type = PowerType::Fixed;
}

Status USBPD::Init(const Config &config) {
	if(this->isInitialized == true) {
		return Status::Ok;
	}

	this->config = config;

	// Create RTOS synchronization objects
	if(tx_mutex_create(&this->mutex, const_cast<char*>("USBPD mutex"), TX_INHERIT) != TX_SUCCESS) {
		return Status::Error;
	}
	if(tx_event_flags_create(&this->event, const_cast<char*>("USBPD event")) != TX_SUCCESS) {
		tx_mutex_delete(&this->mutex);
		return Status::Error;
	}

	// Enable peripheral clock and assign IRQ
	if(this->instance == UCPD1) {
		LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);
		this->irqCall = UCPD1_IRQn;
	}
	else {
		return Status::Error;
	}

	// Ensure peripheral is disabled during configuration
	LL_UCPD_Disable(this->instance);

	// Initialize physical layer timing parameters
	LL_UCPD_InitTypeDef ucpdInit;
	LL_UCPD_StructInit(&ucpdInit);
	LL_UCPD_Init(this->instance, &ucpdInit);

	// Enable hardware SOP packet and Hard Reset detection
	LL_UCPD_SetRxOrderSet(this->instance, LL_UCPD_ORDERSET_SOP | LL_UCPD_ORDERSET_HARDRST);

	// Enable UCPD peripheral
	LL_UCPD_Enable(this->instance);

	// Assert Sink Rd pull-down terminations on both CC lines
	this->AssertRDTerminations();

	// Configure interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);

	LL_UCPD_EnableIT_TypeCEventCC1(this->instance);
	LL_UCPD_EnableIT_TypeCEventCC2(this->instance);
	LL_UCPD_EnableIT_RxNE(this->instance);
	LL_UCPD_EnableIT_RxMsgEnd(this->instance);
	LL_UCPD_EnableIT_RxHRST(this->instance);
	LL_UCPD_EnableIT_RxOvr(this->instance);
	LL_UCPD_EnableIT_TxMSGSENT(this->instance);
	LL_UCPD_EnableIT_TxMSGDISC(this->instance);
	LL_UCPD_EnableIT_TxMSGABT(this->instance);

	// Detect initial attachment state in case cable was connected prior to boot
	this->DetectCCOrientation();

	this->isInitialized = true;
	return Status::Ok;
}

Status USBPD::DeInit() {
	if(this->isInitialized == false) {
		return Status::Ok;
	}

	NVIC_DisableIRQ(this->irqCall);
	LL_UCPD_Disable(this->instance);

	if(this->instance == UCPD1) {
		LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_UCPD1);
		LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_UCPD1);
		LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_UCPD1);
	}

	tx_mutex_delete(&this->mutex);
	tx_event_flags_delete(&this->event);

	this->isInitialized = false;
	this->isAttached = false;
	this->activeCCLine = CCLine::None;
	this->profileCount = 0;

	return Status::Ok;
}

Status USBPD::RequestProfile(uint8_t profileIndex, float current) {
	if(this->isAttached == false || profileIndex == 0 || profileIndex > this->profileCount) {
		return Status::Error;
	}

	if(tx_mutex_get(&this->mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
		return Status::Busy;
	}

	// Clamp current to advertised profile maximum
	float maxCurrent = this->profiles[profileIndex - 1].current;
	if(current > maxCurrent || current == 0.0f) {
		current = maxCurrent;
	}

	// Build Request Data Object (RDO)
	uint32_t rdoPayload = 0;
	if(this->profiles[profileIndex - 1].type == PowerType::PPS) {
		// PPS Request: Output voltage in 20mV units (50 units/V), Operating current in 50mA units (20 units/A)
		uint32_t voltUnits = static_cast<uint32_t>(this->profiles[profileIndex - 1].voltage * 50.0f + 0.5f);
		uint32_t currUnits = static_cast<uint32_t>(current * 20.0f + 0.5f);

		rdoPayload = ((static_cast<uint32_t>(profileIndex) & 0x07) << 28) |
						(1UL << 24) |					// No USB Suspend
						((voltUnits & 0x7FF) << 9) |	// Output Voltage (20mV steps)
						(currUnits & 0x7F);				// Operating Current (50mA steps)
	}
	else {
		// Fixed/Variable Request: Current in 10mA units (100 units/A)
		uint32_t currentUnits = static_cast<uint32_t>(current * 100.0f + 0.5f);
		if(currentUnits > 0x3FF) {
			currentUnits = 0x3FF;
		}

		rdoPayload = ((static_cast<uint32_t>(profileIndex) & 0x07) << 28) |
						(1UL << 24) |					// No USB Suspend
						((currentUnits & 0x3FF) << 10) |// Operating Current (10mA steps)
						(currentUnits & 0x3FF);			// Max Operating Current (10mA steps)
	}

	// Clear negotiation flags
	tx_event_flags_set(&this->event, ~(EVT_READY | EVT_REJECT | EVT_ERR), TX_AND);

	// Transmit Request data message (Type = 0x02, 1 Data Object)
	Status status = this->SendMessage(0x02, &rdoPayload, 1);
	if(status != Status::Ok) {
		tx_mutex_put(&this->mutex);
		return status;
	}

	// Await Source PS_RDY event
	ULONG events;
	UINT waitStatus = tx_event_flags_get(&this->event, EVT_READY | EVT_REJECT | EVT_ERR, TX_OR_CLEAR, &events, TIMEOUT_BUSY_US);
	if(waitStatus != TX_SUCCESS) {
		tx_mutex_put(&this->mutex);
		return Status::Timeout;
	}

	if((events & EVT_READY) == EVT_READY) {
		this->activePDOIndex = profileIndex;
		this->activeProfile.voltage = this->profiles[profileIndex - 1].voltage;
		this->activeProfile.minVoltage = this->profiles[profileIndex - 1].minVoltage;
		this->activeProfile.current = current;
		this->activeProfile.type = this->profiles[profileIndex - 1].type;

		tx_mutex_put(&this->mutex);
		return Status::Ok;
	}

	tx_mutex_put(&this->mutex);
	return Status::Error;
}

Status USBPD::RequestVoltage(float voltage, float current) {
	if(this->profileCount == 0) {
		return Status::Error;
	}

	// Locate matching profile index
	int8_t selectedIndex = -1;
	for(uint8_t i = 0; i < this->profileCount; i++) {
		if(this->profiles[i].type == PowerType::Fixed) {
			float diff = this->profiles[i].voltage - voltage;
			if(diff < 0.0f) {
				diff = -diff;
			}
			// Match within 250mV tolerance
			if(diff < 0.25f) {
				selectedIndex = i;
				break;
			}
		}
		else if(this->profiles[i].type == PowerType::PPS || this->profiles[i].type == PowerType::Variable) {
			if(voltage >= this->profiles[i].minVoltage && voltage <= this->profiles[i].voltage) {
				selectedIndex = i;
				break;
			}
		}
	}

	if(selectedIndex < 0) {
		return Status::Error;
	}

	return this->RequestProfile(static_cast<uint8_t>(selectedIndex + 1), current);
}

Status USBPD::Reset() {
	if(this->isAttached == false) {
		return Status::Error;
	}

	LL_UCPD_SendHardReset(this->instance);
	return Status::Ok;
}

// -----------------------------------------------------------------------------
// Internal Hardware & Protocol Methods
// -----------------------------------------------------------------------------

void USBPD::AssertRDTerminations() {
	// Disable Type-C detectors during configuration
	LL_UCPD_TypeCDetectionCC1Disable(this->instance);
	LL_UCPD_TypeCDetectionCC2Disable(this->instance);

	// Set Sink role (Rd)
	LL_UCPD_SetSNKRole(this->instance);

	// Enable terminations on both CC lines
	LL_UCPD_SetccEnable(this->instance, LL_UCPD_CCENABLE_CC1CC2);

	// Short stabilization delay for analog PHY
	for(uint32_t i = 0; i < 2000; i++) { __NOP(); }

	// Turn on the Type-C voltage comparators
	LL_UCPD_TypeCDetectionCC1Enable(this->instance);
	LL_UCPD_TypeCDetectionCC2Enable(this->instance);
}

void USBPD::DetectCCOrientation() {
	uint32_t stateCC1 = LL_UCPD_GetTypeCVstateCC1(this->instance);
	uint32_t stateCC2 = LL_UCPD_GetTypeCVstateCC2(this->instance);

	uint32_t activeState = 0;

	// CC1 active, CC2 open
	if(stateCC1 != LL_UCPD_SNK_CC1_VOPEN && stateCC2 == LL_UCPD_SNK_CC2_VOPEN) {
		this->activeCCLine = CCLine::CC1;
		this->isAttached = true;
		activeState = stateCC1;

		LL_UCPD_SetCCPin(this->instance, LL_UCPD_CCPIN_CC1);
		LL_UCPD_SetccEnable(this->instance, LL_UCPD_CCENABLE_CC1);
		LL_UCPD_RxEnable(this->instance);

		if(this->config.EventCallback != nullptr) {
			this->config.EventCallback(this->config.callbackContext, Event::Attached);
		}
	}
	// CC2 active, CC1 open
	else if(stateCC2 != LL_UCPD_SNK_CC2_VOPEN && stateCC1 == LL_UCPD_SNK_CC1_VOPEN) {
		this->activeCCLine = CCLine::CC2;
		this->isAttached = true;
		activeState = stateCC2;

		LL_UCPD_SetCCPin(this->instance, LL_UCPD_CCPIN_CC2);
		LL_UCPD_SetccEnable(this->instance, LL_UCPD_CCENABLE_CC2);
		LL_UCPD_RxEnable(this->instance);

		if(this->config.EventCallback != nullptr) {
			this->config.EventCallback(this->config.callbackContext, Event::Attached);
		}
	}
	// Both lines open (Detached)
	else if(stateCC1 == LL_UCPD_SNK_CC1_VOPEN && stateCC2 == LL_UCPD_SNK_CC2_VOPEN) {
		if(this->isAttached == true) {
			this->isAttached = false;
			this->activeCCLine = CCLine::None;
			this->profileCount = 0;

			this->activeProfile.voltage = 5.0f;
			this->activeProfile.minVoltage = 5.0f;
			this->activeProfile.current = 0.5f;
			this->activeProfile.type = PowerType::Fixed;

			LL_UCPD_RxDisable(this->instance);
			LL_UCPD_SetccEnable(this->instance, LL_UCPD_CCENABLE_CC1CC2);

			if(this->config.EventCallback != nullptr) {
				this->config.EventCallback(this->config.callbackContext, Event::Detached);
			}
		}
	}

	// Determine analog advertised current limit if attached
	this->activeProfile.voltage = 5.0f;
	this->activeProfile.minVoltage = 5.0f;
	this->activeProfile.type = PowerType::Fixed;

	if(stateCC1 != LL_UCPD_SNK_CC1_VOPEN && stateCC2 == LL_UCPD_SNK_CC2_VOPEN) {
		if(activeState == LL_UCPD_SNK_CC1_VRP30A) {
			// Type-C 3.0A @ 5V
			this->activeProfile.current = 3.0f;
		}
		else if(activeState == LL_UCPD_SNK_CC1_VRP15A) {
			// Type-C 1.5A @ 5V
			this->activeProfile.current = 1.5f;
		}
		else {
			// Standard USB 2.0 (500mA) / USB 3.0 (900mA)
			this->activeProfile.current = 0.5f;
		}
	}
	else {
		if(activeState == LL_UCPD_SNK_CC2_VRP30A) {
			// Type-C 3.0A @ 5V
			this->activeProfile.current = 3.0f;
		}
		else if(activeState == LL_UCPD_SNK_CC2_VRP15A) {
			// Type-C 1.5A @ 5V
			this->activeProfile.current = 1.5f;
		}
		else {
			// Standard USB 2.0 (500mA) / USB 3.0 (900mA)
			this->activeProfile.current = 0.5f;
		}
	}
}

void USBPD::SendGoodCRC(uint8_t msgID) {
	// Construct GoodCRC Header: Type = 0x01, Sink role, Rev 2.0 (01b), 0 data objects
	uint16_t header = (0x01 & 0x1F) | (0x01 << 6) |	((static_cast<uint16_t>(msgID) & 0x07) << 9);
	this->txBuffer[0] = static_cast<uint8_t>(header & 0xFF);
	this->txBuffer[1] = static_cast<uint8_t>((header >> 8) & 0xFF);

	LL_UCPD_WriteTxOrderSet(this->instance, LL_UCPD_ORDERSET_SOP);
	LL_UCPD_SetTxMode(this->instance, LL_UCPD_TXMODE_NORMAL);
	LL_UCPD_WriteTxPaySize(this->instance, 2);

	// Send frame and feed bytes directly
	LL_UCPD_SendMessage(this->instance);
	LL_UCPD_WriteData(this->instance, this->txBuffer[0]);
	LL_UCPD_WriteData(this->instance, this->txBuffer[1]);
}

Status USBPD::SendMessage(uint8_t msgType, const uint32_t *payload, uint8_t numObjects) {
	uint16_t header = (msgType & 0x1F) | (0x01 << 6) | ((static_cast<uint16_t>(this->txMsgID) & 0x07) << 9) | ((static_cast<uint16_t>(numObjects) & 0x07) << 12);
	this->txMsgID = (this->txMsgID + 1) & 0x07;
	this->txBuffer[0] = static_cast<uint8_t>(header & 0xFF);
	this->txBuffer[1] = static_cast<uint8_t>((header >> 8) & 0xFF);

	uint8_t index = 2;
	for(uint8_t i = 0; i < numObjects; i++) {
		this->txBuffer[index++] = static_cast<uint8_t>(payload[i] & 0xFF);
		this->txBuffer[index++] = static_cast<uint8_t>((payload[i] >> 8) & 0xFF);
		this->txBuffer[index++] = static_cast<uint8_t>((payload[i] >> 16) & 0xFF);
		this->txBuffer[index++] = static_cast<uint8_t>((payload[i] >> 24) & 0xFF);
	}

	uint8_t totalBytes = index;

	LL_UCPD_WriteTxOrderSet(this->instance, LL_UCPD_ORDERSET_SOP);
	LL_UCPD_SetTxMode(this->instance, LL_UCPD_TXMODE_NORMAL);
	LL_UCPD_WriteTxPaySize(this->instance, totalBytes);

	// Trigger transmission and enable TXIS interrupt to feed buffer
	LL_UCPD_EnableIT_TxIS(this->instance);
	LL_UCPD_SendMessage(this->instance);

	return Status::Ok;
}

void USBPD::ParseSourceCapabilities(uint8_t numObjects) {
	if(numObjects > maxProfiles) {
		numObjects = maxProfiles;
	}

	this->profileCount = numObjects;
	for(uint8_t i = 0; i < numObjects; i++) {
		uint8_t offset = 2 + (i * 4);
		uint32_t pdo = static_cast<uint32_t>(this->rxBuffer[offset]);
		pdo = pdo + (static_cast<uint32_t>(this->rxBuffer[offset + 1]) << 8);
		pdo = pdo + (static_cast<uint32_t>(this->rxBuffer[offset + 2]) << 16);
		pdo = pdo + (static_cast<uint32_t>(this->rxBuffer[offset + 3]) << 24);

		uint8_t pdoType = (pdo >> 30) & 0x03;
		if(pdoType == 0x00) {
			// Fixed Supply: Voltage in 50mV units, Current in 10mA units
			this->profiles[i].type = PowerType::Fixed;
			this->profiles[i].voltage = ((pdo >> 10) & 0x3FF) * 0.05f;
			this->profiles[i].minVoltage = this->profiles[i].voltage;
			this->profiles[i].current = (pdo & 0x3FF) * 0.01f;
		}
		else if(pdoType == 0x01) {
			// Battery Supply: Max Voltage in 50mV units, Min Voltage in 50mV units, Max Power in 250mW units
			this->profiles[i].type = PowerType::Battery;
			this->profiles[i].voltage = ((pdo >> 20) & 0x3FF) * 0.05f;
			this->profiles[i].minVoltage = ((pdo >> 10) & 0x3FF) * 0.05f;
			float maxPowerW = (pdo & 0x3FF) * 0.25f;
			this->profiles[i].current = (this->profiles[i].minVoltage > 0.0f) ? (maxPowerW / this->profiles[i].minVoltage) : 0.0f;
		}
		else if(pdoType == 0x02) {
			// Variable Supply: Max Voltage in 50mV units, Min Voltage in 50mV units, Current in 10mA units
			this->profiles[i].type = PowerType::Variable;
			this->profiles[i].voltage = ((pdo >> 20) & 0x3FF) * 0.05f;
			this->profiles[i].minVoltage = ((pdo >> 10) & 0x3FF) * 0.05f;
			this->profiles[i].current = (pdo & 0x3FF) * 0.01f;
		}
		else {
			// Augmented Power Data Object (PPS): Max Voltage in 100mV units, Min Voltage in 100mV units, Current in 50mA units
			this->profiles[i].type = PowerType::PPS;
			this->profiles[i].voltage = ((pdo >> 17) & 0xFF) * 0.1f;
			this->profiles[i].minVoltage = ((pdo >> 8) & 0xFF) * 0.1f;
			this->profiles[i].current = (pdo & 0x7F) * 0.05f;
		}
	}
}

// -----------------------------------------------------------------------------
// IRQ Handler
// -----------------------------------------------------------------------------

void USBPD::InterruptHandler() {
	static uint8_t rxIndex = 0;
	static uint8_t txIndex = 0;

	// Type-C connection status change
	if(LL_UCPD_IsActiveFlag_TypeCEventCC1(this->instance) == 0x01 || LL_UCPD_IsActiveFlag_TypeCEventCC2(this->instance) == 0x01) {
		this->DetectCCOrientation();

		// Clear flags
		LL_UCPD_ClearFlag_TypeCEventCC1(this->instance);
		LL_UCPD_ClearFlag_TypeCEventCC2(this->instance);
	}

	// Receive data register not empty
	if(LL_UCPD_IsActiveFlag_RxNE(this->instance) == 0x01) {
		uint8_t data = LL_UCPD_ReadData(this->instance);
		if(rxIndex < rxBufferSize) {
			this->rxBuffer[rxIndex++] = data;
		}
	}

	// Packet reception complete
	if(LL_UCPD_IsActiveFlag_RxMsgEnd(this->instance) == 0x01) {
		if(LL_UCPD_IsActiveFlag_RxErr(this->instance) == 0 && rxIndex >= 2) {
			uint16_t header = static_cast<uint16_t>(this->rxBuffer[0]) | (static_cast<uint16_t>(this->rxBuffer[1]) << 8);
			uint8_t msgType = header & 0x1F;
			uint8_t numObjects = (header >> 12) & 0x07;
			uint8_t msgId = (header >> 9) & 0x07;

			// Transmit GoodCRC response for all received non-GoodCRC frames
			if(numObjects != 0 || msgType != 0x01) {
				this->SendGoodCRC(msgId);
			}

			// Data messages
			if(numObjects > 0) {
				if(msgType == 0x01) { // Source Capabilities
					this->ParseSourceCapabilities(numObjects);
					tx_event_flags_set(&this->event, EVT_RX_CAPS, TX_OR);

					if(this->config.EventCallback != nullptr) {
						this->config.EventCallback(this->config.callbackContext, Event::ProfileReceived);
					}

					// Automatic target profile negotiation
					if(this->config.autoNegotiate == true) {
						this->RequestVoltage(this->config.targetVoltage, this->config.targetCurrent);
					}
				}
			}
			// Control messages
			else {
				if(msgType == 0x03) {
					// Accept: Power source accepted request, awaiting PS_RDY
				}
				else if(msgType == 0x06) {
					// PS_RDY
					tx_event_flags_set(&this->event, EVT_READY, TX_OR);

					if(this->config.EventCallback != nullptr) {
						this->config.EventCallback(this->config.callbackContext, Event::ProfileReady);
					}
				}
				else if(msgType == 0x04 || msgType == 0x0C) {
					// Reject or Wait
					tx_event_flags_set(&this->event, EVT_REJECT, TX_OR);

					if(this->config.EventCallback != nullptr) {
						this->config.EventCallback(this->config.callbackContext, Event::ProfileRejected);
					}
				}
			}
		}
		rxIndex = 0;

		// Clear flags
		LL_UCPD_ClearFlag_RxMsgEnd(this->instance);
	}

	// Transmit data register empty
	if(LL_UCPD_IsEnableIT_TxIS(this->instance) == 0x01 && LL_UCPD_IsActiveFlag_TxIS(this->instance) == 0x01) {
		uint32_t paySize = LL_UCPD_ReadRxPaySize(this->instance);
		if(txIndex < paySize) {
			LL_UCPD_WriteData(this->instance, this->txBuffer[txIndex++]);
		}
		else {
			txIndex = 0;
			LL_UCPD_DisableIT_TxIS(this->instance);
		}
	}

	// Transmit completed
	if(LL_UCPD_IsActiveFlag_TxMSGSENT(this->instance) == 0x01) {
		txIndex = 0;
		LL_UCPD_DisableIT_TxIS(this->instance);

		//Clear flags
		LL_UCPD_ClearFlag_TxMSGSENT(this->instance);
	}

	// Transmit discarded / aborted
	if(LL_UCPD_IsActiveFlag_TxMSGDISC(this->instance) == 0x01 || LL_UCPD_IsActiveFlag_TxMSGABT(this->instance) == 0x01) {
		txIndex = 0;
		tx_event_flags_set(&this->event, EVT_ERR, TX_OR);

		LL_UCPD_DisableIT_TxIS(this->instance);

		//Clear flags
		LL_UCPD_ClearFlag_TxMSGDISC(this->instance);
		LL_UCPD_ClearFlag_TxMSGABT(this->instance);
	}

	// Hard reset received
	if(LL_UCPD_IsActiveFlag_RxHRST(this->instance) == 0x01) {
		this->activeProfile.voltage = 5.0f;
		this->activeProfile.minVoltage = 5.0f;
		this->activeProfile.current = 0.5f;
		this->activeProfile.type = PowerType::Fixed;

		if(this->config.EventCallback != nullptr) {
			this->config.EventCallback(this->config.callbackContext, Event::Error);
		}

		//Clear flags
		LL_UCPD_ClearFlag_RxHRST(this->instance);
	}

	// Buffer overrun / underrun
	if(LL_UCPD_IsActiveFlag_RxOvr(this->instance) == 0x01) {
		rxIndex = 0;
		//Clear flags
		LL_UCPD_ClearFlag_RxOvr(this->instance);
	}
}