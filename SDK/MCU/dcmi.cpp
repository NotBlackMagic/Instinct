/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/dcmi.cpp
 */

#include "dcmi.hpp"

Dcmi::Dcmi(DCMI_TypeDef *instance) {
	this->instance = instance;
	this->irqPriority = 0x0E; 	//Lowest priority
}

void Dcmi::Init() {
	// Enable bus clocks
	if(this->instance == DCMI) {
		LL_APB5_GRP1_EnableClock(LL_APB5_GRP1_PERIPH_DCMIPP);
	}

	// Configure DCMI Interface
	MODIFY_REG(DCMI->CR, DCMI_CR_LSM, 0x00);						//Interface captures all received lines
//	MODIFY_REG(DCMI->CR, DCMI_CR_OELS, 0x00);						//Interface captures first line after the frame start, second one being dropped
	MODIFY_REG(DCMI->CR, DCMI_CR_BSM, 0x00);						//Interface captures all received data
//	MODIFY_REG(DCMI->CR, DCMI_CR_OEBS, 0x00);						//Interface captures first data (byte or double byte) from the frame/line start, second one being dropped
	MODIFY_REG(DCMI->CR, DCMI_CR_EDM_0 | DCMI_CR_EDM_1, 0x00);		//Interface captures 8-bit data on every pixel clock
	MODIFY_REG(DCMI->CR, DCMI_CR_FCRC_0 | DCMI_CR_FCRC_1, 0x00);	//All frames are captured, ignored in snapshot mode
	MODIFY_REG(DCMI->CR, DCMI_CR_VSPOL, DCMI_CR_VSPOL);				//DCMI_VSYNC active high
	MODIFY_REG(DCMI->CR, DCMI_CR_HSPOL, 0x00);						//DCMI_HSYNC active low
	MODIFY_REG(DCMI->CR, DCMI_CR_PCKPOL, DCMI_CR_PCKPOL);			//Rising edge active
	MODIFY_REG(DCMI->CR, DCMI_CR_ESS, 0x00);						//Hardware synchronization data capture (frame/line start/stop) is synchronized with the DCMI_HSYNC/DCMI_VSYNC signals.
	MODIFY_REG(DCMI->CR, DCMI_CR_JPEG, 0x00);						//Uncompressed video format (no JPEG)
	MODIFY_REG(DCMI->CR, DCMI_CR_CROP, 0x00);						//The full image is captured. In this case the total number of bytes in an image frame must be a multiple of four
	MODIFY_REG(DCMI->CR, DCMI_CR_CM, DCMI_CR_CM);					//Snapshot mode (single frame)


	// Enable DCMI
	MODIFY_REG(DCMI->CR, DCMI_CR_ENABLE_Msk, DCMI_CR_ENABLE);		//Enable DCMI Interface
//	MODIFY_REG(DCMI->CR, DCMI_CR_CAPTURE_Msk, DCMI_CR_CAPTURE);		//Capture enabled
}

void Dcmi::StartContinuous() {
	// Capture enabled
	MODIFY_REG(DCMI->CR, DCMI_CR_CAPTURE_Msk, DCMI_CR_CAPTURE);
}

void Dcmi::Stop() {
	// Capture Disable
	MODIFY_REG(DCMI->CR, DCMI_CR_CAPTURE_Msk, 0x00);
}