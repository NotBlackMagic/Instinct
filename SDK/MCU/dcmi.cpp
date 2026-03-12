/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/dcmi.cpp
 */

#include "dcmi.hpp"

Dcmi::Dcmi(DCMI_TypeDef *instance) {
	this->instance = instance;
	this->irqPriority = 0x0E; 	// Lowest priority
}

Status Dcmi::Init(const Config &config) {
	// Enable bus clocks
	if(this->instance == DCMI) {
		// LL_APB5_GRP1_EnableClock(LL_APB5_GRP1_PERIPH_DCMIPP);
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_PSSI);
		this->irqCall = DCMI_PSSI_IRQn;
	}
	else {
		return Status::Error;
	}

	// Configure DCMI Interface
	MODIFY_REG(this->instance->CR, DCMI_CR_LSM, 0x00);						// Interface captures all received lines
//	MODIFY_REG(this->instance->CR, DCMI_CR_OELS, 0x00);						// Interface captures first line after the frame start, second one being dropped
	MODIFY_REG(this->instance->CR, DCMI_CR_BSM, 0x00);						// Interface captures all received data
//	MODIFY_REG(this->instance->CR, DCMI_CR_OEBS, 0x00);						// Interface captures first data (byte or double byte) from the frame/line start, second one being dropped
	MODIFY_REG(this->instance->CR, DCMI_CR_EDM_0 | DCMI_CR_EDM_1, 0x00);		// Interface captures 8-bit data on every pixel clock
	MODIFY_REG(this->instance->CR, DCMI_CR_FCRC_0 | DCMI_CR_FCRC_1, 0x00);	// All frames are captured, ignored in snapshot mode
	MODIFY_REG(this->instance->CR, DCMI_CR_VSPOL, (static_cast<uint32_t>(config.vSyncPolarity) << DCMI_CR_VSPOL_Pos));	// DCMI_VSYNC active high
	MODIFY_REG(this->instance->CR, DCMI_CR_HSPOL, (static_cast<uint32_t>(config.hSyncPolarity) << DCMI_CR_HSPOL_Pos));	// DCMI_HSYNC active low
	MODIFY_REG(this->instance->CR, DCMI_CR_PCKPOL, (static_cast<uint32_t>(config.pxClkPolarity) << DCMI_CR_PCKPOL_Pos));	// Rising edge active
	if(config.embeddedSync == true) {
		// Embedded synchronization data capture is synchronized with synchronization codes embedded in the data flow 
		MODIFY_REG(this->instance->CR, DCMI_CR_ESS, DCMI_CR_ESS);
	}
	else {
		// Hardware synchronization data capture (frame/line start/stop) is synchronized with the DCMI_HSYNC/DCMI_VSYNC signals.
		MODIFY_REG(this->instance->CR, DCMI_CR_ESS, 0x00);
	}
	MODIFY_REG(this->instance->CR, DCMI_CR_JPEG, 0x00);						// Uncompressed video format (no JPEG)
	MODIFY_REG(this->instance->CR, DCMI_CR_CROP, 0x00);						// The full image is captured. In this case the total number of bytes in an image frame must be a multiple of four
	MODIFY_REG(this->instance->CR, DCMI_CR_CM, DCMI_CR_CM);					// Snapshot mode (single frame)

	// Configure Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);
	// MODIFY_REG(this->instance->IER, DCMI_IER_FRAME_IE, DCMI_IER_FRAME_IE);	// Capture complete interrupt enable
	// MODIFY_REG(this->instance->IER, DCMI_IER_OVR_IE, DCMI_IER_OVR_IE);		// Overrun interrupt enable
	// MODIFY_REG(this->instance->IER, DCMI_IER_ERR_IE, DCMI_IER_ERR_IE);		// Synchronization error interrupt enable
	// MODIFY_REG(this->instance->IER, DCMI_IER_VSYNC_IE, DCMI_IER_VSYNC_IE);	// VSYNC interrupt enable
	// MODIFY_REG(this->instance->IER, DCMI_IER_LINE_IE, DCMI_IER_LINE_IE);	// Line interrupt enable

	// Enable DCMI
	MODIFY_REG(this->instance->CR, DCMI_CR_ENABLE_Msk, 0x00);		// Enable DCMI Interface

	return Status::Ok;
}

Status Dcmi::Start(CaptureMode mode) {
	// Set capture mode
	if(mode == Dcmi::CaptureMode::Snapshot) {
		// Snapshot mode (single frame)
		MODIFY_REG(this->instance->CR, DCMI_CR_CM, DCMI_CR_CM);
	}
	else {
		// Continuous grab mode
		MODIFY_REG(this->instance->CR, DCMI_CR_CM, 0x00);
	}

	// Re-enable the DCMI peripheral (0 to 1 transition resets the FIFO)
	MODIFY_REG(this->instance->CR, DCMI_CR_ENABLE_Msk, DCMI_CR_ENABLE);

	// Capture enabled
	MODIFY_REG(this->instance->CR, DCMI_CR_CAPTURE, DCMI_CR_CAPTURE);

	return Status::Ok;
}

void Dcmi::Stop() {
	// Capture Disable
	MODIFY_REG(this->instance->CR, DCMI_CR_CAPTURE_Msk, 0x00);
	// Disable the entire DCMI peripheral to flush the internal FIFO
	MODIFY_REG(this->instance->CR, DCMI_CR_ENABLE_Msk, 0x00);
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

void Dcmi::InterruptHandler() {
	// Handle Overrun
	if(READ_BIT(this->instance->MISR, DCMI_MIS_OVR_MIS) == DCMI_IER_OVR_IE) {
		// Clear Interrupt
		MODIFY_REG(this->instance->ICR, DCMI_ICR_OVR_ISC_Msk, DCMI_ICR_OVR_ISC);
	}

	//Handle Capture Complete
	if(READ_BIT(this->instance->MISR, DCMI_MIS_FRAME_MIS) == DCMI_IER_FRAME_IE) {
		//Clear Interrupt
		MODIFY_REG(this->instance->ICR, DCMI_ICR_FRAME_ISC_Msk, DCMI_ICR_FRAME_ISC);
	}
}