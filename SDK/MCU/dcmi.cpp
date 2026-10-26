/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	SDK/MCU/dcmi.cpp
 */

#include "dcmi.hpp"

Dcmi::Dcmi(DCMI_TypeDef *instance) {
	this->instance = instance;
	this->irqPriority = 0x05; 	// Match DMA priority
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
	MODIFY_REG(this->instance->CR, DCMI_CR_LSM, 0x00);		// Interface captures all received lines
	MODIFY_REG(this->instance->CR, DCMI_CR_BSM, 0x00);		// Interface captures all received data
	MODIFY_REG(this->instance->CR, DCMI_CR_EDM, 0x00);		// Interface captures 8-bit data on every pixel clock
	MODIFY_REG(this->instance->CR, DCMI_CR_FCRC, 0x00);		// All frames are captured, ignored in snapshot mode
	MODIFY_REG(this->instance->CR, DCMI_CR_VSPOL, (static_cast<uint32_t>(config.vSyncPolarity) << DCMI_CR_VSPOL_Pos));
	MODIFY_REG(this->instance->CR, DCMI_CR_HSPOL, (static_cast<uint32_t>(config.hSyncPolarity) << DCMI_CR_HSPOL_Pos));
	MODIFY_REG(this->instance->CR, DCMI_CR_PCKPOL, (static_cast<uint32_t>(config.pxClkPolarity) << DCMI_CR_PCKPOL_Pos));
	if(config.embeddedSync == true) {
		// Embedded synchronization data capture is synchronized with synchronization codes embedded in the data flow 
		MODIFY_REG(this->instance->CR, DCMI_CR_ESS, DCMI_CR_ESS);
	}
	else {
		// Hardware synchronization data capture (frame/line start/stop) is synchronized with the DCMI_HSYNC/DCMI_VSYNC signals.
		MODIFY_REG(this->instance->CR, DCMI_CR_ESS, 0x00);
	}
	MODIFY_REG(this->instance->CR, DCMI_CR_JPEG, 0x00);		// Uncompressed video format (no JPEG)
	MODIFY_REG(this->instance->CR, DCMI_CR_CROP, 0x00);		// The full image is captured. In this case the total number of bytes in an image frame must be a multiple of four
	MODIFY_REG(this->instance->CR, DCMI_CR_CM, DCMI_CR_CM);	// Snapshot mode (single frame)

	// Configure Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);
	MODIFY_REG(this->instance->IER, DCMI_IER_FRAME_IE, DCMI_IER_FRAME_IE);	// Capture complete interrupt enable
	MODIFY_REG(this->instance->IER, DCMI_IER_OVR_IE, DCMI_IER_OVR_IE);		// Overrun interrupt enable
	MODIFY_REG(this->instance->IER, DCMI_IER_ERR_IE, DCMI_IER_ERR_IE);		// Synchronization error interrupt enable
	// MODIFY_REG(this->instance->IER, DCMI_IER_VSYNC_IE, DCMI_IER_VSYNC_IE);	// VSYNC interrupt enable
	// MODIFY_REG(this->instance->IER, DCMI_IER_LINE_IE, DCMI_IER_LINE_IE);		// Line interrupt enable

	// Enable DCMI
	MODIFY_REG(this->instance->CR, DCMI_CR_ENABLE, 0x00);		// Disable DCMI Interface

	return Status::Ok;
}

Status Dcmi::ConfigureCrop(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
	// Enable crop
	MODIFY_REG(this->instance->CR, DCMI_CR_CROP, DCMI_CR_CROP);
	// Configure crop window start
	WRITE_REG(this->instance->CWSTRTR, (static_cast<uint32_t>(y) << DCMI_CWSTRT_VST_Pos) | (static_cast<uint32_t>(x) << DCMI_CWSTRT_HOFFCNT_Pos));
	// Configure crop window size
	WRITE_REG(this->instance->CWSIZER, (static_cast<uint32_t>(w - 1) << DCMI_CWSIZE_VLINE_Pos) | (static_cast<uint32_t>(h - 1) << DCMI_CWSIZE_CAPCNT_Pos));

	return Status::Ok;
}

Status Dcmi::ConfigureDecimation(bool halfV, uint8_t skipCnt) {
	// Capture line and pixel skip setting
	if(halfV == true) {
		// Interface captures first line after the frame start, second one being dropped
		MODIFY_REG(this->instance->CR, DCMI_CR_OELS, 0x00);
		// Capture one line out of two
		MODIFY_REG(this->instance->CR, DCMI_CR_LSM, DCMI_CR_LSM);
	}
	else {
		// Capture all lines
		MODIFY_REG(this->instance->CR, DCMI_CR_LSM, 0x00);
	}

	// Interface captures first data (byte or double byte) from the frame/line start, second one being dropped
	MODIFY_REG(this->instance->CR, DCMI_CR_OEBS, 0x00);
	switch(skipCnt) {
		case 0:
			// Capture all bytes
			MODIFY_REG(this->instance->CR, DCMI_CR_BSM, (static_cast<uint32_t>(0x00) << DCMI_CR_BSM_Pos));
			break;
		case 1:
			// Capture every other byte
			MODIFY_REG(this->instance->CR, DCMI_CR_BSM, (static_cast<uint32_t>(0x01) << DCMI_CR_BSM_Pos));
			break;
		case 2:
			// Skip 2 bytes, capture 2 bytes
			MODIFY_REG(this->instance->CR, DCMI_CR_BSM, (static_cast<uint32_t>(0x03) << DCMI_CR_BSM_Pos));
			break;
		case 3:
			// Skip 3 bytes, capture 1 bytes
			MODIFY_REG(this->instance->CR, DCMI_CR_BSM, (static_cast<uint32_t>(0x02) << DCMI_CR_BSM_Pos));
			break;
		default:
			return Status::Error;
	}

	return Status::Ok;
}

Status Dcmi::Start(CaptureMode mode) {
	// Capture disable (for safety)
	MODIFY_REG(this->instance->CR, DCMI_CR_CAPTURE | DCMI_CR_ENABLE, 0x00);
	// Set capture mode
	if(mode == Dcmi::CaptureMode::Snapshot) {
		// Snapshot mode (single frame)
		MODIFY_REG(this->instance->CR, DCMI_CR_CM, DCMI_CR_CM);
	}
	else {
		// Continuous grab mode
		MODIFY_REG(this->instance->CR, DCMI_CR_CM, 0x00);
	}

	// Clear all flags
	WRITE_REG(this->instance->ICR, (DCMI_ICR_OVR_ISC | DCMI_ICR_ERR_ISC | DCMI_ICR_FRAME_ISC | DCMI_ICR_VSYNC_ISC | DCMI_ICR_LINE_ISC));

	// Re-enable the DCMI peripheral (0 to 1 transition resets the FIFO)
	MODIFY_REG(this->instance->CR, DCMI_CR_ENABLE, DCMI_CR_ENABLE);

	// Capture enabled
	MODIFY_REG(this->instance->CR, DCMI_CR_CAPTURE, DCMI_CR_CAPTURE);

	return Status::Ok;
}

Status Dcmi::Stop() {
	// Capture Disable
	MODIFY_REG(this->instance->CR, DCMI_CR_CAPTURE, 0x00);
	// Disable the entire DCMI peripheral to flush the internal FIFO
	MODIFY_REG(this->instance->CR, DCMI_CR_ENABLE, 0x00);
	return Status::Ok;
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------
void Dcmi::InterruptHandler() {
	uint32_t misr = this->instance->MISR;

	// Handle Overrun
	if((misr & DCMI_MIS_OVR_MIS) != 0U) {
		this->stats.overruns = this->stats.overruns + 1;
		// Clear Interrupt
		MODIFY_REG(this->instance->ICR, DCMI_ICR_OVR_ISC, DCMI_ICR_OVR_ISC);
	}

	// Handle Synchronization Error
	if((misr & DCMI_MIS_ERR_MIS) != 0U) {
		this->stats.syncErrors = this->stats.syncErrors + 1;
		// Clear Interrupt
		MODIFY_REG(this->instance->ICR, DCMI_ICR_ERR_ISC, DCMI_ICR_ERR_ISC);
	}

	//Handle Capture Complete
	if((misr & DCMI_MIS_FRAME_MIS) != 0U) {
		this->stats.frames = this->stats.frames + 1;
		//Clear Interrupt
		MODIFY_REG(this->instance->ICR, DCMI_ICR_FRAME_ISC, DCMI_ICR_FRAME_ISC);
	}
}