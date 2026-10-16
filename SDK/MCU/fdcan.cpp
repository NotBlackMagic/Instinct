/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	SDK/MCU/fdcan.cpp
 */

#include "fdcan.hpp"

// Internal M_CAN Message RAM Bitmasks
#define FDCAN_ELEMENT_MASK_STDID	((uint32_t)0x1FFC0000U)
#define FDCAN_ELEMENT_MASK_EXTID	((uint32_t)0x1FFFFFFFU)
#define FDCAN_ELEMENT_MASK_XTD		((uint32_t)0x40000000U)
#define FDCAN_ELEMENT_MASK_DLC		((uint32_t)0x000F0000U)
#define FDCAN_ELEMENT_MASK_BRS		((uint32_t)0x00100000U)
#define FDCAN_ELEMENT_MASK_FDF		((uint32_t)0x00200000U)

#define FDCAN_TIMEOUT_INIT_COUNT	100000U

FDCAN::FDCAN(FDCAN_GlobalTypeDef *instance) {
	this->instance = instance;
	this->irqPriority = 0x0E; // Lowest priority safe default
	this->isInitialized = false;
	this->activeTXBufferMask = 0;
	this->ramRXFIFO0Sa = 0;
	this->ramTXFIFOSa = 0;
}

Status FDCAN::Init(const Config &config) {
	if(config.sourceClockHz == 0 || config.nominalBaudrate == 0) {
		return Status::Error;
	}

	if(this->isInitialized == true) {
		return Status::Ok;
	}

	this->config = config;

	// Create RTOS synchronization objects
	if(tx_mutex_create(&this->txMutex, const_cast<char*>("fdcan tx mutex"), TX_INHERIT) != TX_SUCCESS) {
		return Status::Error;
	}
	if(tx_event_flags_create(&this->txEvent, const_cast<char*>("fdcan tx event")) != TX_SUCCESS) {
		tx_mutex_delete(&this->txMutex);
		return Status::Error;
	}
	if(tx_event_flags_create(&this->rxEvent, const_cast<char*>("fdcan rx event")) != TX_SUCCESS) {
		tx_mutex_delete(&this->txMutex);
		tx_event_flags_delete(&this->txEvent);
		return Status::Error;
	}

	// Enable bus clocks and identify IRQ lines
	if(this->instance == FDCAN1) {
		LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_FDCAN);
		this->irqCall = FDCAN1_IT0_IRQn;
		// this->irqCall = FDCAN1_IT1_IRQn;
	}
	else if(this->instance == FDCAN2) {
		LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_FDCAN);
		this->irqCall = FDCAN2_IT0_IRQn;
		// this->irqCall = FDCAN2_IT1_IRQn;
	}
	else if(this->instance == FDCAN3) {
		LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_FDCAN);
		this->irqCall = FDCAN3_IT0_IRQn;
		// this->irqCall = FDCAN3_IT1_IRQn;
	}
	else {
		return Status::Error;
	}

	// Exit Sleep Mode
	CLEAR_BIT(this->instance->CCCR, FDCAN_CCCR_CSR);
	uint32_t timeout = 0;
	while((this->instance->CCCR & FDCAN_CCCR_CSA) == FDCAN_CCCR_CSA) {
		timeout = timeout + 1;
		if(timeout > FDCAN_TIMEOUT_INIT_COUNT) {
			return Status::Timeout;
		}
	}

	// Request Initialization Mode
	SET_BIT(this->instance->CCCR, FDCAN_CCCR_INIT);
	timeout = 0;
	while((this->instance->CCCR & FDCAN_CCCR_INIT) == 0U) {
		timeout = timeout + 1;
		if(timeout > FDCAN_TIMEOUT_INIT_COUNT) {
			return Status::Timeout;
		}
	}

	// Enable Configuration Changes
	SET_BIT(this->instance->CCCR, FDCAN_CCCR_CCE);

	// Set Frame Format (Classic vs CAN FD with optional BRS)
	if(this->config.fdMode == true) {
		if(this->config.dataBaudrate > 0) {
			MODIFY_REG(this->instance->CCCR, FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE, FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);
		}
		else {
			MODIFY_REG(this->instance->CCCR, FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE, FDCAN_CCCR_FDOE);
		}
	}
	else {
		CLEAR_BIT(this->instance->CCCR, FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);
	}

	// Standard Normal Operating Mode
	CLEAR_BIT(this->instance->CCCR, FDCAN_CCCR_TEST | FDCAN_CCCR_MON | FDCAN_CCCR_ASM);

	// Calculate and apply nominal and data bit timings
	uint32_t div = this->config.sourceClockHz / this->config.nominalBaudrate;
	if(div == 0U) {
		return Status::Error;
	}

	// Solve for best integer prescaler
	uint32_t prescaler = 0;
	uint32_t totalTQ = 0;
	for(uint32_t tq = 25; tq >= 10; tq--) {
		if((div % tq) == 0U && (div / tq) <= 512U) {
			totalTQ = tq;
			prescaler = div / tq;
			break;
		}
	}

	if(prescaler == 0U) {
		totalTQ = 20;
		prescaler = div / totalTQ;
		if(prescaler == 0U) {
			prescaler = 1;
		}
		if(prescaler > 512U) {
			prescaler = 512;
		}
	}

	uint32_t tSeg1 = (totalTQ * 80U) / 100U - 1U; // SyncSeg is 1 TQ
	uint32_t tSeg2 = totalTQ - 1U - tSeg1;
	uint32_t sJW = (tSeg2 > 4U) ? 4U : tSeg2;
	WRITE_REG(this->instance->NBTP, ((sJW - 1U) << FDCAN_NBTP_NSJW_Pos) |
									((tSeg1 - 1U) << FDCAN_NBTP_NTSEG1_Pos) |
									((tSeg2 - 1U) << FDCAN_NBTP_NTSEG2_Pos) |
									((prescaler - 1U) << FDCAN_NBTP_NBRP_Pos));

	// Data Bit Timing Setup (CAN FD with BRS)
	if(this->config.fdMode == true && this->config.dataBaudrate > 0) {
		div = this->config.sourceClockHz / this->config.dataBaudrate;
		for(uint32_t tq = 20; tq >= 8; tq--) {
			if((div % tq) == 0U && (div / tq) <= 32U) {
				totalTQ = tq;
				prescaler = div / tq;
				break;
			}
		}

		if(prescaler == 0U) {
			totalTQ = 16;
			prescaler = div / totalTQ;
			if(prescaler == 0U) {
				prescaler = 1;
			}
			if(prescaler > 32U) {
				prescaler = 32;
			}
		}

		tSeg1 = (totalTQ * 75U) / 100U - 1U;
		tSeg2 = totalTQ - 1U - tSeg1;
		sJW = (tSeg2 > 4U) ? 4U : tSeg2;
		WRITE_REG(this->instance->DBTP, ((sJW - 1U) << FDCAN_DBTP_DSJW_Pos) |
										((tSeg1 - 1U) << FDCAN_DBTP_DTSEG1_Pos) |
										((tSeg2 - 1U) << FDCAN_DBTP_DTSEG2_Pos) |
										((prescaler - 1U) << FDCAN_DBTP_DBRP_Pos));

		// Enable Transmitter Delay Compensation (TDC)
		MODIFY_REG(this->instance->DBTP, FDCAN_DBTP_TDC, FDCAN_DBTP_TDC);

		uint32_t tdcOffset = (tSeg1 + 1U) * prescaler;
		if(tdcOffset > 127U) {
			tdcOffset = 127U;
		}
		WRITE_REG(this->instance->TDCR, (tdcOffset << FDCAN_TDCR_TDCO_Pos));
	}

	// Partition internal Message RAM (RX FIFO0 and TX FIFO)
	// FDCAN1 base offset is 0; FDCAN2/3 offset by half-bank to partition the 10KB SRAMCAN cleanly
	uint32_t ramOffsetBytes = 0;
	if(this->instance == FDCAN2) {
		ramOffsetBytes = 0x1400;
	}
	if(this->instance == FDCAN3) {
		ramOffsetBytes = 0x2000;
	}

	// RX FIFO 0 Setup: 32 elements of 64-byte payload (18 words each)
	WRITE_REG(this->instance->RXF0C, ((ramOffsetBytes / 4U) << FDCAN_RXF0C_F0SA_Pos) | (static_cast<uint32_t>(rxFIFO0Size) << FDCAN_RXF0C_F0S_Pos));
	MODIFY_REG(this->instance->RXESC, FDCAN_RXESC_F0DS, (7U << FDCAN_RXESC_F0DS_Pos));	// 7 = 64-byte data field
	this->ramRXFIFO0Sa = SRAMCAN_BASE + ramOffsetBytes;
	ramOffsetBytes += (static_cast<uint32_t>(rxFIFO0Size) * elementSizeWords * 4U);

	// TX FIFO Setup: 16 elements of 64-byte payload (18 words each)
	WRITE_REG(this->instance->TXBC, ((ramOffsetBytes / 4U) << FDCAN_TXBC_TBSA_Pos) | (static_cast<uint32_t>(txFIFOSize) << FDCAN_TXBC_TFQS_Pos));
	MODIFY_REG(this->instance->TXESC, FDCAN_TXESC_TBDS, 7U);	// 7 = 64-byte data field
	this->ramTXFIFOSa = SRAMCAN_BASE + ramOffsetBytes;

	// Clear allocated RAM region
	memset(reinterpret_cast<void*>(this->ramRXFIFO0Sa), 0, (static_cast<uint32_t>(rxFIFO0Size) + txFIFOSize) * elementSizeWords * 4U);

	// Global Filter Configuration: Accept all non-matching standard & extended IDs into RX FIFO 0
	WRITE_REG(this->instance->GFC, (0x01U << FDCAN_GFC_RRFS_Pos) | (0x01U << FDCAN_GFC_RRFE_Pos));

	// Configure Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);
	CLEAR_REG(this->instance->ILS);
	SET_BIT(this->instance->ILE, 0x00000001U);
	WRITE_REG(this->instance->IE, FDCAN_IE_TCE | FDCAN_IE_RF0NE | FDCAN_IE_BOE | FDCAN_IE_EPE | FDCAN_IE_EWE | FDCAN_IE_PEAE | FDCAN_IE_PEDE);	// Enable TC, RF0N, and Bus Fault interrupts
	WRITE_REG(this->instance->TXBTIE, (1UL << txFIFOSize) - 1U);	// Enable Transmission Interrupt for all allocated TX FIFO buffers (0 to txFIFOSize - 1)

	// Leave Initialization Mode to start bus operation
	CLEAR_BIT(this->instance->CCCR, FDCAN_CCCR_INIT);
	timeout = 0;
	while((this->instance->CCCR & FDCAN_CCCR_INIT) != 0U) {
		timeout = timeout + 1;
		if(timeout > FDCAN_TIMEOUT_INIT_COUNT) {
			return Status::Timeout;
		}
	}

	this->isInitialized = true;
	return Status::Ok;
}

Status FDCAN::DeInit() {
	if(this->isInitialized == false) {
		return Status::Ok;
	}

	// Request initialization mode to stop transmission/reception
	SET_BIT(this->instance->CCCR, FDCAN_CCCR_INIT);

	// Disable interrupts
	NVIC_DisableIRQ(this->irqCall);
	CLEAR_REG(this->instance->ILE);
	CLEAR_REG(this->instance->IE);

	// Delete RTOS objects
	tx_mutex_delete(&this->txMutex);
	tx_event_flags_delete(&this->txEvent);
	tx_event_flags_delete(&this->rxEvent);

	this->isInitialized = false;
	this->activeTXBufferMask = 0;
	return Status::Ok;
}

Status FDCAN::TransferAsync(const Frame &frame) {
	if(this->isInitialized == false) {
		return Status::Error;
	}

	// Lock peripheral TX
	if(tx_mutex_get(&this->txMutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
		return Status::Timeout;
	}

	// Check if TX FIFO is full
	if((this->instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U) {
		tx_mutex_put(&this->txMutex);
		return Status::Busy;
	}

	// Clear TX event flags
	tx_event_flags_set(&this->txEvent, 0, TX_AND);

	// Fetch next free buffer index in TX FIFO (Put Index)
	uint32_t pktIx = (this->instance->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;

	// Serialize frame directly into Message RAM
	this->WriteTXBuffer(frame, pktIx);

	// Request transmission for this buffer slot
	this->activeTXBufferMask = (1UL << pktIx);
	this->instance->TXBAR = this->activeTXBufferMask;

	return Status::Ok;
}

Status FDCAN::TransferWait(uint32_t timeoutTicks) {
	ULONG events;
	UINT status = tx_event_flags_get(&this->txEvent, EVT_TX_CPLT | EVT_TX_ERR, TX_OR_CLEAR, &events, timeoutTicks);

	if(status != TX_SUCCESS) {
		this->stats.timeouts = this->stats.timeouts + 1;
		this->TransferAbort();
		return Status::Timeout;
	}

	if((events & EVT_TX_ERR) != 0U) {
		this->TransferAbort();
		return Status::Error;
	}

	this->activeTXBufferMask = 0;
	tx_mutex_put(&this->txMutex);
	return Status::Ok;
}

Status FDCAN::TransferAbort() {
	if(this->activeTXBufferMask != 0U) {
		// Request cancellation of pending frame
		this->instance->TXBCR = this->activeTXBufferMask;
		this->activeTXBufferMask = 0U;
	}

	tx_event_flags_set(&this->txEvent, EVT_TX_ERR, TX_OR);
	tx_mutex_put(&this->txMutex);
	return Status::Ok;
}

Status FDCAN::Receive(Frame &frame, uint32_t timeoutTicks) {
	if(this->isInitialized == false) {
		return Status::Error;
	}

	while(true) {
		if((this->instance->RXF0S & FDCAN_RXF0S_F0FL) > 0U) {
			uint32_t pktIx = (this->instance->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;

			// Deserialize frame from Message RAM
			this->ReadRXFifo(frame, pktIx);

			// Acknowledge read element so hardware frees slot
			this->instance->RXF0A = pktIx;

			this->stats.rxSuccess = this->stats.rxSuccess + 1;
			return Status::Ok;
		}

		// Suspend until ISR alerts a new frame or timeout expires
		ULONG events;
		if(tx_event_flags_get(&this->rxEvent, EVT_RX_NEW, TX_OR_CLEAR, &events, timeoutTicks) != TX_SUCCESS) {
			return Status::Timeout;
		}
	}
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

void FDCAN::InterruptHandler() {
	uint32_t intr = this->instance->IR & this->instance->IE;

	// RX FIFO 0 New Message
	if((intr & FDCAN_IR_RF0N) != 0U) {
		this->instance->IR = FDCAN_IR_RF0N;
		tx_event_flags_set(&this->rxEvent, EVT_RX_NEW, TX_OR);

		if(this->config.EventCallback != nullptr) {
			this->config.EventCallback(this->config.callbackContext, Event::RXComplete);
		}
	}

	// Transmission Completed
	if((intr & FDCAN_IR_TC) != 0U) {
		this->instance->IR = FDCAN_IR_TC;
		this->stats.txSuccess = this->stats.txSuccess + 1;
		tx_event_flags_set(&this->txEvent, EVT_TX_CPLT, TX_OR);

		if(this->config.EventCallback != nullptr) {
			this->config.EventCallback(this->config.callbackContext, Event::TXComplete);
		}
	}

	// Bus Off State
	if((intr & FDCAN_IR_BO) != 0U) {
		this->instance->IR = FDCAN_IR_BO;
		this->stats.busErrors = this->stats.busErrors + 1;

		if(this->config.EventCallback != nullptr) {
			this->config.EventCallback(this->config.callbackContext, Event::BusOff);
		}
	}

	// Protocol / Frame Errors
	uint32_t errorFlags = intr & (FDCAN_IR_PEA | FDCAN_IR_PED | FDCAN_IR_EW | FDCAN_IR_EP | FDCAN_IR_ELO);
	if(errorFlags != 0U) {
		this->instance->IR = errorFlags;
		this->stats.busErrors = this->stats.busErrors + 1;

		if(this->config.EventCallback != nullptr) {
			this->config.EventCallback(this->config.callbackContext, Event::Error);
		}
	}
}

// ---------------------------------------------------------
// Internal Helpers
// ---------------------------------------------------------

void FDCAN::WriteTXBuffer(const Frame &frame, uint32_t bufferIndex) {
	uint32_t *txAddr = reinterpret_cast<uint32_t*>(this->ramTXFIFOSa + (bufferIndex * elementSizeWords * sizeof(uint32_t)));

	// Word T0: ID, XTD
	uint32_t t0 = 0;
	if(frame.isExtended == true) {
		t0 = FDCAN_ELEMENT_MASK_XTD | (frame.id & FDCAN_ELEMENT_MASK_EXTID);
	}
	else {
		t0 = (frame.id & 0x7FFU) << 18U;
	}
	*txAddr++ = t0;

	// Word T1: DLC, BRS, FDF
	uint8_t dlc = BytesToDLC(frame.length);
	uint32_t t1 = (static_cast<uint32_t>(dlc) << 16U);
	if(frame.isFD == true) {
		t1 |= FDCAN_ELEMENT_MASK_FDF;
		if(frame.bitRateSwitch == true) {
			t1 |= FDCAN_ELEMENT_MASK_BRS;
		}
	}
	*txAddr++ = t1;

	// Payload: Words T2..T17
	uint8_t payloadBytes = DLCToBytes(dlc);
	for(uint32_t i = 0; i < payloadBytes; i += 4) {
		uint32_t word = 0;
		for(uint32_t b = 0; b < 4; ++b) {
			if((i + b) < frame.length) {
				word |= (static_cast<uint32_t>(frame.data[i + b]) << (b * 8));
			}
		}
		*txAddr++ = word;
	}
}

void FDCAN::ReadRXFifo(Frame &frame, uint32_t getIndex) {
	const uint32_t *rxAddr = reinterpret_cast<const uint32_t*>(this->ramRXFIFO0Sa + (getIndex * elementSizeWords * sizeof(uint32_t)));

	// Word R0: ID, XTD
	uint32_t r0 = *rxAddr++;
	frame.isExtended = ((r0 & FDCAN_ELEMENT_MASK_XTD) != 0U);
	if(frame.isExtended == true) {
		frame.id = (r0 & FDCAN_ELEMENT_MASK_EXTID);
	}
	else {
		frame.id = (r0 & FDCAN_ELEMENT_MASK_STDID) >> 18U;
	}

	// Word R1: DLC, BRS, FDF
	uint32_t r1 = *rxAddr++;
	frame.isFD = ((r1 & FDCAN_ELEMENT_MASK_FDF) != 0U);
	frame.bitRateSwitch = ((r1 & FDCAN_ELEMENT_MASK_BRS) != 0U);
	uint8_t dlc = (r1 & FDCAN_ELEMENT_MASK_DLC) >> 16U;
	frame.length = DLCToBytes(dlc);

	// Payload: Words R2..R17
	for(uint32_t i = 0; i < frame.length; i += 4) {
		uint32_t word = *rxAddr++;
		for(uint32_t b = 0; b < 4; ++b) {
			if((i + b) < frame.length) {
				frame.data[i + b] = static_cast<uint8_t>((word >> (b * 8)) & 0xFF);
			}
		}
	}
}

uint8_t FDCAN::DLCToBytes(uint8_t dlc) {
	static const uint8_t dlcTable[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
	if(dlc > 15) {
		return 64;
	}
	return dlcTable[dlc];
}

uint8_t FDCAN::BytesToDLC(uint8_t bytes) {
	if(bytes <= 8)  return bytes;
	if(bytes <= 12) return 9;
	if(bytes <= 16) return 10;
	if(bytes <= 20) return 11;
	if(bytes <= 24) return 12;
	if(bytes <= 32) return 13;
	if(bytes <= 48) return 14;
	return 15;
}