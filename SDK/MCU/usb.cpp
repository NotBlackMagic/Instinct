/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/usb.cpp
 */

#include "usb.hpp"

USB::USB(USB_OTG_GlobalTypeDef* instance) {
	this->instance = instance;
	this->device = ((USB_OTG_DeviceTypeDef *)((uint32_t)this->instance + USB_OTG_DEVICE_BASE));
	this->irqPriority = 0x0D; // Lowest priority (safe default)
}

Status USB::Init(const Config &config) {
	if(this->isInitialized == true) {
		return Status::Ok;
	}

	this->config = config;

	// Enable bus clocks
	if(this->instance == USB1_OTG_HS) {
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_OTG1);
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_OTGPHY1);
		this->irqCall = USB1_OTG_HS_IRQn;

		USB1_HS_PHYC->USBPHYC_CR &= ~(0x7 << 0x4);

		USB1_HS_PHYC->USBPHYC_CR |= (0x1 << 16) |
									(0x1 << 4)  |
									(0x1 << 2)  |
										0x1U;
	}
	else if(this->instance == USB2_OTG_HS) {
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_OTG2);
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_OTGPHY2);
		this->irqCall = USB2_OTG_HS_IRQn;
	}
	else {
		return Status::Error;
	}

	// Configure RIF (enable USB DMA secure region access, etc...)
	if(this->config.useDMA == true) {
		this->ConfigureRIF();
	}

	// Wait for bus idle
	uint32_t timeoutMs = 100;
	uint32_t timestamp = Time::GetMs();
	while((this->instance->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0x00) {
		if((Time::GetMs() - timestamp) > timeoutMs) {
			// Wait bus idle timeout
			return Status::Error;
		}
	}

	// Configure PHY interface for Embedded HS PHY
	CLEAR_BIT(this->instance->GUSBCFG, USB_OTG_GUSBCFG_TSDPS);

	// Soft reset
	MODIFY_REG(this->instance->GRSTCTL, USB_OTG_GRSTCTL_CSRST, USB_OTG_GRSTCTL_CSRST);
	timeoutMs = 100;
	timestamp = Time::GetMs();
	while((this->instance->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST) {
		if((Time::GetMs() - timestamp) > timeoutMs) {
			// Soft reset timeout
			return Status::Error;
		}
	}

	// Set function mode: device mode
	MODIFY_REG(this->instance->GUSBCFG, USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD, USB_OTG_GUSBCFG_FDMOD);
	timeoutMs = 200;
	timestamp = Time::GetMs();
	while((this->instance->GINTSTS & 0x01) != USB_DEVICE_MODE) {
		if((Time::GetMs() - timestamp) > timeoutMs) {
			// Set function mode timeout
			return Status::Error;
		}
	}

	// Set turnaround time
	uint32_t turnTime = 0x9U;
	if(this->config.speed == USB::BusSpeed::High) {
		turnTime = 0x9U;	// For High-Speed
	}
	else {
		turnTime = 0x6U;	// For Full-Speed and HCLK Clock Range between 32-200 MHz and 
	}
	MODIFY_REG(this->instance->GUSBCFG, USB_OTG_GUSBCFG_TRDT, ((turnTime) << USB_OTG_GUSBCFG_TRDT_Pos));

	// Set phy mode: internal
	// MODIFY_REG(this->instance->GUSBCFG, USB_OTG_GUSBCFG_PHYSEL, USB_OTG_GUSBCFG_PHYSEL);

	// Configure device mode
	MODIFY_REG(this->instance->GCCFG, USB_OTG_GCCFG_PULLDOWNEN, 0x00);	// Disable USB PHY pulldown resistors
	if(this->config.vbusSensing == true) {
		// B-peripheral session valid override disable
		MODIFY_REG(this->instance->GCCFG, USB_OTG_GCCFG_VBVALEXTOEN, 0x00);
		MODIFY_REG(this->instance->GCCFG, USB_OTG_GCCFG_VBVALOVAL, 0x00);
	}
	else {
		MODIFY_REG(this->device->DCTL, USB_OTG_DCTL_SDIS, USB_OTG_DCTL_SDIS);
		// B-peripheral session valid override enable
		MODIFY_REG(this->instance->GCCFG, USB_OTG_GCCFG_VBVALEXTOEN, USB_OTG_GCCFG_VBVALEXTOEN);
		MODIFY_REG(this->instance->GCCFG, USB_OTG_GCCFG_VBVALOVAL, USB_OTG_GCCFG_VBVALOVAL);
	}
	WRITE_REG(*(__IO uint32_t *)((uint32_t)this->instance + USB_OTG_PCGCCTL_BASE), 0x00);	// Restart the Phy Clock
	if(this->config.speed == USB::BusSpeed::High) {
		MODIFY_REG(this->device->DCFG, USB_OTG_DCFG_DSPD, 0x00);	// Set Core speed to High speed mode
	}
	else {
		MODIFY_REG(this->device->DCFG, USB_OTG_DCFG_DSPD, 0x01);	// Set Core speed to Full speed mode 
	}

	// Setup FIFO (Total RAM: 4KB = 1024 32-bit words)
	// RX FIFO size (Min): (5 * number of control endpoints + 8) + 2*((largest USB packet used / 4) + 1 for status information) + (2 * number of OUT endpoints) + 1 for Global NAK
	uint32_t rxFifoSize = 290;						// Shared RX FIFO for all OUT endpoints
	uint32_t tx0FifoSize = (maxEP0PktSize >> 2);	// Dedicated TX FIFO for EP0 IN (Min: 16 i.e. 64 bytes)
	uint32_t txnFifoSize = (maxPktSize >> 2);		// Dedicated TX FIFO for EP1-EP5 IN (EP6, EP7, and EP8 unused to support 512 byte packets on all endpoints)
	// Set RX FIFO Size (Offset is implicitly 0)
	WRITE_REG(this->instance->GRXFSIZ, rxFifoSize);
	// Set EP0 TX FIFO (Size and Start Address)
	uint32_t offset = rxFifoSize;
	WRITE_REG(this->instance->DIEPTXF0_HNPTXFSIZ, (tx0FifoSize << 16) | offset);
	offset += tx0FifoSize;
	// Set EP1 to EP5 (EP6, EP7, and EP8 unused) TX FIFOs
	for(uint8_t i = 0; i < maxUserInEP; i++) {
		WRITE_REG(this->instance->DIEPTXF[i], (txnFifoSize << 16) | offset);
		offset += txnFifoSize;
	}

	// Flush FIFOs
	this->FlushTxFifo(0x10);
	this->FlushRxFifo();

	// Clear interrupts
	WRITE_REG(this->device->DIEPMSK, 0x00);
	WRITE_REG(this->device->DOEPMSK, 0x00);
	WRITE_REG(this->device->DAINTMSK, 0x00);

	// Basic/essential endpoint configuration

	//
	// MODIFY_REG(this->device->DIEPMSK, USB_OTG_DIEPMSK_TXFURM, USB_OTG_DIEPMSK_TXFURM);

	// Disable all interrupts and clear pending
	WRITE_REG(this->instance->GINTMSK, 0x00);
	WRITE_REG(this->instance->GINTSTS, 0xBFFFFFFFU);

	// Enable common interrupts
	if(this->config.useDMA == false) {
		MODIFY_REG(this->instance->GINTMSK, USB_OTG_GINTMSK_RXFLVLM, USB_OTG_GINTMSK_RXFLVLM);
	}
	else {
		MODIFY_REG(this->instance->GAHBCFG, USB_OTG_GAHBCFG_DMAEN | USB_OTG_GAHBCFG_HBSTLEN_2, USB_OTG_GAHBCFG_DMAEN | USB_OTG_GAHBCFG_HBSTLEN_2);
	}
	MODIFY_REG(this->instance->GINTMSK, 0x00,	USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
												USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
												USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IISOIXFRM |
												USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_WUIM);
	if(this->config.softOutput == true) {
		MODIFY_REG(this->instance->GINTMSK, USB_OTG_GINTMSK_SOFM, USB_OTG_GINTMSK_SOFM);
	}
	if(this->config.vbusSensing == true) {
		MODIFY_REG(this->instance->GINTMSK, USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT, USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT);
	}

	// Configure USB Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);
	// Enable global interrupt
	MODIFY_REG(this->instance->GAHBCFG, USB_OTG_GAHBCFG_GINT, USB_OTG_GAHBCFG_GINT);

	this->isInitialized = true;
	return Status::Ok;
}

Status USB::DeInit() {
	if(this->isInitialized == false) {
		return Status::Ok;
	}

	// Disable IRQ first
	NVIC_DisableIRQ(this->irqCall);

	// Soft reset
	MODIFY_REG(this->instance->GRSTCTL, USB_OTG_GRSTCTL_CSRST, USB_OTG_GRSTCTL_CSRST);

	// Force hardware reset through peripheral clock
	if(this->instance == USB1_OTG_HS) {
		LL_AHB5_GRP1_ForceReset(LL_AHB5_GRP1_PERIPH_OTG1);
		LL_AHB5_GRP1_ForceReset(LL_AHB5_GRP1_PERIPH_OTGPHY1);
		Time::Delay(2);
		LL_AHB5_GRP1_ReleaseReset(LL_AHB5_GRP1_PERIPH_OTG1);
		LL_AHB5_GRP1_ReleaseReset(LL_AHB5_GRP1_PERIPH_OTGPHY1);

		// Disable clocks
		LL_AHB5_GRP1_DisableClock(LL_AHB5_GRP1_PERIPH_OTG1);
		LL_AHB5_GRP1_DisableClock(LL_AHB5_GRP1_PERIPH_OTGPHY1);
	}
	else if(this->instance == USB2_OTG_HS) {
		LL_AHB5_GRP1_ForceReset(LL_AHB5_GRP1_PERIPH_OTG2);
		LL_AHB5_GRP1_ForceReset(LL_AHB5_GRP1_PERIPH_OTGPHY2);
		Time::Delay(2);
		LL_AHB5_GRP1_ReleaseReset(LL_AHB5_GRP1_PERIPH_OTG2);
		LL_AHB5_GRP1_ReleaseReset(LL_AHB5_GRP1_PERIPH_OTGPHY2);

		// Disable clocks
		LL_AHB5_GRP1_DisableClock(LL_AHB5_GRP1_PERIPH_OTG2);
		LL_AHB5_GRP1_DisableClock(LL_AHB5_GRP1_PERIPH_OTGPHY2);
	}

	isInitialized = false;
	return Status::Ok;
}

Status USB::Connect() {
	// Ungate and restore the phy CLK
	volatile uint32_t* pcgcctl = (__IO uint32_t *)((uint32_t)this->instance + USB_OTG_PCGCCTL_BASE);
	*pcgcctl &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
	// Clear the Soft Disconnect bit to connect the internal pull-up	
	MODIFY_REG(this->device->DCTL, USB_OTG_DCTL_SDIS, 0x00);
	return Status::Ok;
}

Status USB::Disconnect() {
	// Ungate and restore the phy CLK
	volatile uint32_t* pcgcctl = (__IO uint32_t *)((uint32_t)this->instance + USB_OTG_PCGCCTL_BASE);
	*pcgcctl &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
	// Set the Soft Disconnect bit to disconnect the internal pull-up
	MODIFY_REG(this->device->DCTL, USB_OTG_DCTL_SDIS, USB_OTG_DCTL_SDIS);
	return Status::Ok;
}

Status USB::SetAddress(uint8_t address) {
	MODIFY_REG(this->device->DCFG, USB_OTG_DCFG_DAD, ((uint32_t)address << USB_OTG_DCFG_DAD_Pos));
	return Status::Ok;
}

Status USB::OpenEndpoint(uint8_t epAddr, EndpointType type, uint16_t maxPacketSize) {
	if(this->IsValidEndpoint(epAddr) == false) {
		return Status::Error;
	}

	uint8_t epNum = epAddr & 0x0F;
	bool isCmdIn = ((epAddr & 0x80) == 0x80);

	// EP0 is initialized internally by the core/driver, lockout for user change!
	if(epNum == 0) {
		return Status::Error;
	}

	if(maxPacketSize > maxPktSize) {
		// Unsuported max packet size
		return Status::Error;
	}

	if(isCmdIn == true) {
		// Setup new IN endpoint
		USB_OTG_INEndpointTypeDef* inEp = ((USB_OTG_INEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_IN_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));
	
		if((inEp->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0x00) {
			// Assign TX FIFO (EP0 -> FIFO0, EP1 -> FIFO1, etc...)
			uint32_t fifoNum = epNum;
			uint32_t epConfig = ((maxPacketSize << USB_OTG_DIEPCTL_MPSIZ_Pos) & USB_OTG_DIEPCTL_MPSIZ_Msk) |
								(((uint32_t)type << USB_OTG_DIEPCTL_EPTYP_Pos) & USB_OTG_DIEPCTL_EPTYP_Msk) |
								((fifoNum << USB_OTG_DIEPCTL_TXFNUM_Pos) & USB_OTG_DIEPCTL_TXFNUM_Msk) |
								USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
								USB_OTG_DIEPCTL_USBAEP;
			WRITE_REG(inEp->DIEPCTL, epConfig);	
		}

		// Clear any pending interrupts
		WRITE_REG(inEp->DIEPINT, 0xFFFFFFFF);

		// Unmask the interrupt for this IN endpoint
		SET_BIT(this->device->DAINTMSK, (uint32_t)(1UL << epNum) & USB_OTG_DAINTMSK_IEPM_Msk);
	}
	else {
		// Setup new OUT endpoint
		USB_OTG_OUTEndpointTypeDef* outEp = ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_OUT_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));
	
		if((outEp->DOEPCTL & USB_OTG_DOEPCTL_USBAEP) == 0x00) {
			uint32_t epConfig = ((maxPacketSize << USB_OTG_DOEPCTL_MPSIZ_Pos) & USB_OTG_DOEPCTL_MPSIZ_Msk) |
								(((uint32_t)type << USB_OTG_DOEPCTL_EPTYP_Pos) & USB_OTG_DOEPCTL_EPTYP_Msk) |
								(epNum << 22) |
								USB_OTG_DOEPCTL_USBAEP;
			WRITE_REG(outEp->DOEPCTL, epConfig);	
		}

		// Clear any pending interrupts
		WRITE_REG(outEp->DOEPINT, 0xFFFFFFFF);

		// Unmask the interrupt for this OUT endpoint
		SET_BIT(this->device->DAINTMSK, (uint32_t)((1UL << epNum) << USB_OTG_DAINTMSK_OEPM_Pos) & USB_OTG_DAINTMSK_OEPM_Msk);
	}

	return Status::Ok;
}

Status USB::CloseEndpoint(uint8_t epAddr) {
	if(this->IsValidEndpoint(epAddr) == false) {
		return Status::Error;
	}

	uint8_t epNum = epAddr & 0x0F;
	bool isCmdIn = ((epAddr & 0x80) == 0x80);

	// EP0 is initialized internally by the core/driver, lockout for user change!
	if(epNum == 0) {
		return Status::Error;
	}

	if(isCmdIn == true) {
		// Disable IN endpoint and clear bits
		USB_OTG_INEndpointTypeDef* inEp = ((USB_OTG_INEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_IN_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));

		if((inEp->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA) {
			MODIFY_REG(inEp->DIEPCTL, USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_EPDIS, USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_EPDIS);

			uint32_t timeoutCount = 0xF0000;
			while (((inEp->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA) && (timeoutCount > 0)) {
				timeoutCount--;
			}
		}

		// Mask interrupts for this IN endpoint
		CLEAR_BIT(this->device->DEACHMSK, (uint32_t)(1UL << epNum) & USB_OTG_DAINTMSK_IEPM_Msk);
		CLEAR_BIT(this->device->DAINTMSK, (uint32_t)(1UL << epNum) & USB_OTG_DAINTMSK_IEPM_Msk);

		inEp->DIEPCTL &= ~(USB_OTG_DIEPCTL_USBAEP |
							USB_OTG_DIEPCTL_MPSIZ |
							USB_OTG_DIEPCTL_TXFNUM |
							USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
							USB_OTG_DIEPCTL_EPTYP);
	}
	else {
		// Disable OUT endpoint and clear bits
		USB_OTG_OUTEndpointTypeDef* outEp = ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_OUT_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));

		if((outEp->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) {
			// Assert Global OUT NAK to pause incoming traffic
			if((this->instance->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) == 0x00) {
				MODIFY_REG(this->device->DCTL, USB_OTG_DCTL_SGONAK, USB_OTG_DCTL_SGONAK);
			}

			uint32_t timeoutCount = 0xF0000;
			while (((this->instance->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) == 0x00) && (timeoutCount > 0)) {
				timeoutCount--;
			}

			MODIFY_REG(outEp->DOEPCTL, USB_OTG_DOEPCTL_SNAK | USB_OTG_DOEPCTL_EPDIS, USB_OTG_DOEPCTL_SNAK | USB_OTG_DOEPCTL_EPDIS);

			timeoutCount = 0xF0000;
			while (((outEp->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) && (timeoutCount > 0)) {
				timeoutCount--;
			}

			// Clear the EPDISD flag
			WRITE_REG(outEp->DOEPINT, USB_OTG_DOEPINT_EPDISD);

			// Clear Global OUT NAK
			MODIFY_REG(this->device->DCTL, USB_OTG_DCTL_CGONAK, USB_OTG_DCTL_CGONAK);
		}

		// Mask interrupts for this OUT endpoint
		CLEAR_BIT(this->device->DEACHMSK, (uint32_t)((1UL << epNum) << USB_OTG_DAINTMSK_OEPM_Pos) & USB_OTG_DAINTMSK_OEPM_Msk);
		CLEAR_BIT(this->device->DAINTMSK, (uint32_t)((1UL << epNum) << USB_OTG_DAINTMSK_OEPM_Pos) & USB_OTG_DAINTMSK_OEPM_Msk);

		outEp->DOEPCTL &= ~(USB_OTG_DOEPCTL_USBAEP |
							USB_OTG_DOEPCTL_MPSIZ |
							USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
							USB_OTG_DOEPCTL_EPTYP);
	}
	
	return Status::Ok;
}

Status USB::StallEndpoint(uint8_t epAddr) {
	if(this->IsValidEndpoint(epAddr) == false) {
		return Status::Error;
	}

	uint8_t epNum = epAddr & 0x0F;
	bool isCmdIn = ((epAddr & 0x80) == 0x80);

	if(isCmdIn == true) {
		USB_OTG_INEndpointTypeDef* inEp = ((USB_OTG_INEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_IN_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));
		// Set Stall bit
		SET_BIT(inEp->DIEPCTL, USB_OTG_DIEPCTL_STALL);
	}
	else {
		USB_OTG_OUTEndpointTypeDef* outEp = ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_OUT_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));
		// Set the STALL bit
		SET_BIT(outEp->DOEPCTL, USB_OTG_DOEPCTL_STALL);
	}

	return Status::Ok;
}

Status USB::ClearStall(uint8_t epAddr) {
	if(this->IsValidEndpoint(epAddr) == false) {
		return Status::Error;
	}

	uint8_t epNum = epAddr & 0x0F;
	bool isCmdIn = ((epAddr & 0x80) == 0x80);

	if(isCmdIn == true) {
		USB_OTG_INEndpointTypeDef* inEp = ((USB_OTG_INEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_IN_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));
		// Clear Stall bit
		CLEAR_BIT(inEp->DIEPCTL, USB_OTG_DIEPCTL_STALL);

		uint8_t epType = (inEp->DIEPCTL & USB_OTG_DIEPCTL_EPTYP_Msk) >> USB_OTG_DIEPCTL_EPTYP_Pos;
		if(epType == 0x02 || epType == 0x03) {
			//EP_TYPE_BULK or EP_TYPE_INTR
			SET_BIT(inEp->DIEPCTL, USB_OTG_DIEPCTL_SD0PID_SEVNFRM);
		}
	}
	else {
		USB_OTG_OUTEndpointTypeDef* outEp = ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_OUT_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));
		// Clear the STALL bit
		CLEAR_BIT(outEp->DOEPCTL, USB_OTG_DOEPCTL_STALL);
		
		uint8_t epType = (outEp->DOEPCTL & USB_OTG_DOEPCTL_EPTYP_Msk) >> USB_OTG_DOEPCTL_EPTYP_Pos;
		if(epType == 0x02 || epType == 0x03) {
			//EP_TYPE_BULK or EP_TYPE_INTR
			SET_BIT(outEp->DOEPCTL, USB_OTG_DOEPCTL_SD0PID_SEVNFRM);
		}
	}

	return Status::Ok;
}

bool USB::IsStalled(uint8_t epAddr) {
	if(this->IsValidEndpoint(epAddr) == false) {
		return true;
	}

	uint8_t epNum = epAddr & 0x0F;
	bool isCmdIn = ((epAddr & 0x80) == 0x80);

	if(isCmdIn == true) {
		USB_OTG_INEndpointTypeDef* inEp = ((USB_OTG_INEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_IN_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));
		if((inEp->DIEPCTL & USB_OTG_DIEPCTL_STALL) == USB_OTG_DIEPCTL_STALL) {
			return true;
		}
		else {
			return false;
		}
	}
	else {
		USB_OTG_OUTEndpointTypeDef* outEp = ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_OUT_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));
		if((outEp->DOEPCTL & USB_OTG_DOEPCTL_STALL) == USB_OTG_DOEPCTL_STALL) {
			return true;
		}
		else {
			return false;
		}
	}
}

Status USB::Transmit(uint8_t epAddr, const uint8_t* buf, uint32_t len) {
	if(this->IsValidEndpoint(epAddr) == false) {
		return Status::Error;
	}

	uint8_t epNum = epAddr & 0x0F;

	USB_OTG_INEndpointTypeDef* inEp = ((USB_OTG_INEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_IN_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));

	// Get maximum packet size
	uint32_t maxPacketSize = (inEp->DIEPCTL & USB_OTG_DIEPCTL_MPSIZ);
	if(epNum == 0) {
		// For endpoint 0, force to 64 bytes packet size
		maxPacketSize = maxEP0PktSize;
	}

	uint32_t pktCnt = 1;
	if(len > 0) {
		pktCnt = (len + maxPacketSize - 1) / maxPacketSize;
	}

	MODIFY_REG(inEp->DIEPTSIZ, USB_OTG_DIEPTSIZ_PKTCNT, (pktCnt << USB_OTG_DIEPTSIZ_PKTCNT_Pos) & USB_OTG_DIEPTSIZ_PKTCNT_Msk);
	
	uint8_t epType = (inEp->DIEPCTL & USB_OTG_DIEPCTL_EPTYP_Msk) >> USB_OTG_DIEPCTL_EPTYP_Pos;
	if(epType == 0x01) {
		//EP_TYPE_ISOC
		uint32_t mulCnt = (pktCnt > 3) ? 3 : pktCnt; // Clamp to max 3
		MODIFY_REG(inEp->DIEPTSIZ, USB_OTG_DIEPTSIZ_MULCNT, (mulCnt << USB_OTG_DIEPTSIZ_MULCNT_Pos) & USB_OTG_DIEPTSIZ_MULCNT_Msk);
	}

	MODIFY_REG(inEp->DIEPTSIZ, USB_OTG_DIEPTSIZ_XFRSIZ, (len) & USB_OTG_DIEPTSIZ_XFRSIZ_Msk);

	// Save endpoint transfer context
	this->inEpState[epNum].buffer = (uint8_t*)buf;
	this->inEpState[epNum].count = len;

	if(this->config.useDMA == true) {
		// Handle cache coherency, attention with Zero Length Packet (ZLP)!!
		if(len > 0 && buf != nullptr) {
			System::CleanCache((uint32_t*)buf, len);
		}

		WRITE_REG(inEp->DIEPDMA, (uint32_t)buf);
		
		if(epType == 0x01) {
			//EP_TYPE_ISOC
			if((this->device->DSTS & (1U << 8)) == 0x00) {
				MODIFY_REG(inEp->DIEPCTL, USB_OTG_DIEPCTL_SODDFRM, USB_OTG_DIEPCTL_SODDFRM);
			}
			else {
				MODIFY_REG(inEp->DIEPCTL, USB_OTG_DIEPCTL_SD0PID_SEVNFRM, USB_OTG_DIEPCTL_SD0PID_SEVNFRM);
			}
		}

		// Enable Endpoint and Clear NAK
		MODIFY_REG(inEp->DIEPCTL, USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA, USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);
	}
	else {
		if(epType == 0x01) {
			//EP_TYPE_ISOC
			if((this->device->DSTS & (1U << 8)) == 0x00) {
				MODIFY_REG(inEp->DIEPCTL, USB_OTG_DIEPCTL_SODDFRM, USB_OTG_DIEPCTL_SODDFRM);
			}
			else {
				MODIFY_REG(inEp->DIEPCTL, USB_OTG_DIEPCTL_SD0PID_SEVNFRM, USB_OTG_DIEPCTL_SD0PID_SEVNFRM);
			}
		}
		
		if(len > 0) {
			SET_BIT(this->device->DIEPEMPMSK, 1UL << (epNum & EP_ADDR_MSK));
		}

		// Enable Endpoint and Clear NAK
		MODIFY_REG(inEp->DIEPCTL, USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA, USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);
	}

	return Status::Ok;
}

Status USB::Receive(uint8_t epAddr, uint8_t* buf, uint32_t len) {
	if(this->IsValidEndpoint(epAddr) == false) {
		return Status::Error;
	}

	uint8_t epNum = epAddr & 0x0F;

	USB_OTG_OUTEndpointTypeDef* outEp = ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_OUT_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));

	// Get maximum packet size
	uint32_t maxPktSize = (outEp->DOEPCTL & USB_OTG_DOEPCTL_MPSIZ_Msk);
	if(epNum == 0) {
		// For endpoint 0, force to 64 bytes packet size
		maxPktSize = maxEP0PktSize;
	}

	uint32_t pktCnt = 1;
	uint32_t tSize = 0;
	if(epNum == 0) {
		tSize = maxPktSize;
		pktCnt = 1;
	}
	else {
		if(len == 0) {
			tSize = maxPktSize;
			pktCnt = 1;
		}
		else {
			pktCnt = (len + maxPktSize - 1) / maxPktSize;
			tSize = maxPktSize * pktCnt;
		}
	}

	uint32_t tSiz = outEp->DOEPTSIZ;
	tSiz &= ~(USB_OTG_DOEPTSIZ_XFRSIZ_Msk | USB_OTG_DOEPTSIZ_PKTCNT_Msk);
	tSiz |= (tSize & USB_OTG_DOEPTSIZ_XFRSIZ_Msk) | ((pktCnt << USB_OTG_DOEPTSIZ_PKTCNT_Pos) & USB_OTG_DOEPTSIZ_PKTCNT_Msk);

	WRITE_REG(outEp->DOEPTSIZ, tSiz);

	// MODIFY_REG(outEp->DOEPTSIZ, USB_OTG_DOEPTSIZ_PKTCNT, (pktCnt << USB_OTG_DOEPTSIZ_PKTCNT_Pos) & USB_OTG_DOEPTSIZ_PKTCNT_Msk);
	// MODIFY_REG(outEp->DOEPTSIZ, USB_OTG_DOEPTSIZ_XFRSIZ, (tSize) & USB_OTG_DOEPTSIZ_XFRSIZ_Msk);

	// Save endpoint transfer context
	this->outEpState[epNum].buffer = (uint8_t*)buf;
	this->outEpState[epNum].count = tSize;

	if(this->config.useDMA == true) {
		// Handle cache coherency, attention with Zero Length Packet (ZLP)!!
		if(len > 0 && buf != nullptr) {
			System::InvalidateCache((uint32_t*)buf, tSize);
		}

		WRITE_REG(outEp->DOEPDMA, (uint32_t)buf);
	}

	uint8_t epType = (outEp->DOEPCTL & USB_OTG_DOEPCTL_EPTYP_Msk) >> USB_OTG_DOEPCTL_EPTYP_Pos;
	if(epType == 0x01) {
		if((this->device->DSTS & (1U << 8)) == 0x00) {
			MODIFY_REG(outEp->DOEPCTL, USB_OTG_DOEPCTL_SODDFRM, USB_OTG_DOEPCTL_SODDFRM);
		}
		else {
			MODIFY_REG(outEp->DOEPCTL, USB_OTG_DOEPCTL_SD0PID_SEVNFRM, USB_OTG_DOEPCTL_SD0PID_SEVNFRM);
		}
	}

	// Enable Endpoint and Clear NAK to start receiving
	MODIFY_REG(outEp->DOEPCTL, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK);

	return Status::Ok;
}

void USB::HandleEpInInterrupt() {
	uint32_t epIntr = (this->device->DAINT & this->device->DAINTMSK) & 0xFFFF;
	uint8_t epNum = 0;
	while(epIntr != 0x00) {
		// Check each endpoint interrupt signal
		if((epIntr & 0x01) == 0x01) {
			USB_OTG_INEndpointTypeDef* inEp = ((USB_OTG_INEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_IN_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));
			uint32_t mask = this->device->DIEPMSK;
			mask |= ((this->device->DIEPEMPMSK >> (epNum & EP_ADDR_MSK)) & 0x1U) << 7;
			uint32_t epInt = (inEp->DIEPINT & mask);

			// Transfer complete
			if((epInt & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC) {
				CLEAR_BIT(this->device->DIEPEMPMSK, (uint32_t)(0x1UL << (epNum & EP_ADDR_MSK)));

				// Clear flag
				WRITE_REG(inEp->DIEPINT, USB_OTG_DIEPINT_XFRC);

				if(this->config.useDMA == true) {
					this->inEpState[epNum].count += 0;
				}

				if(this->config.EventCallback != nullptr) {
					this->config.EventCallback(this->config.callbackContext, Event::TransferComplete, 0x80 | epNum, this->inEpState[epNum].count);
				}
			}

			// Timeout
			if((epInt & USB_OTG_DIEPINT_TOC) == USB_OTG_DIEPINT_TOC) {
				// Clear flag
				WRITE_REG(inEp->DIEPINT, USB_OTG_DIEPINT_TOC);
			}

			// Received IN token when TXFIFO empty
			if((epInt & USB_OTG_DIEPINT_ITTXFE) == USB_OTG_DIEPINT_ITTXFE) {
				// Clear flag
				WRITE_REG(inEp->DIEPINT, USB_OTG_DIEPINT_ITTXFE);
			}

			// NACK effective
			if((epInt & USB_OTG_DIEPINT_INEPNE) == USB_OTG_DIEPINT_INEPNE) {
				// Clear flag
				WRITE_REG(inEp->DIEPINT, USB_OTG_DIEPINT_INEPNE);
			}

			// Endpoint disable
			if((epInt & USB_OTG_DIEPINT_EPDISD) == USB_OTG_DIEPINT_EPDISD) {
				// Clear flag
				WRITE_REG(inEp->DIEPINT, USB_OTG_DIEPINT_EPDISD);

				// --- SELF-HEALING STATE MACHINE ---
				// If this was an Isochronous endpoint, it was forcefully disabled due to a missed 
				// microframe. We MUST tell the UVC class it "completed" so it resets txBusy 
				// and pushes the next packet. Otherwise, the pipeline permanently dies.
				uint8_t epType = (inEp->DIEPCTL & USB_OTG_DIEPCTL_EPTYP_Msk) >> USB_OTG_DIEPCTL_EPTYP_Pos;
				if(epType == 0x01) {
					// Flush current TX FIFO
					this->FlushTxFifo(epNum);
					
					if(this->config.EventCallback != nullptr) {
						this->config.EventCallback(this->config.callbackContext, Event::Error, 0x80 | epNum, 0);
					}
				}
			}

			// Transmit FIFO empty
			if((epInt & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE) {
				this->HandleTxFifoInterrupt(epNum);
			}
		}
		epNum += 1;
		epIntr = epIntr >> 1;
	}
}

void USB::HandleEpOutInterrupt() {
	uint32_t epIntr = (this->device->DAINT & this->device->DAINTMSK) >> 16;
	uint8_t epNum = 0;
	while(epIntr != 0x00) {
		// Check each endpoint interrupt signal
		if((epIntr & 0x01) == 0x01) {
			USB_OTG_OUTEndpointTypeDef* outEp = ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_OUT_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));
			uint32_t epInt = (outEp->DOEPINT & this->device->DOEPMSK);

			// Transfer complete
			if((epInt & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC) {
				// Clear flag
				WRITE_REG(outEp->DOEPINT, USB_OTG_DOEPINT_XFRC);

				// Calculate how many bytes were actually received
				uint32_t transferSize = this->outEpState[epNum].count;
				uint32_t bytesRemaining = (outEp->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ_Msk);
				uint32_t actualCount = transferSize - bytesRemaining;

				if(this->config.useDMA == true) {
					// Handle cache coherency
					System::InvalidateCache((uint32_t*)this->outEpState[epNum].buffer, actualCount);
				}

				if(epNum == 0 && actualCount == 0) {
					this->StartEp0Setup();
				}

				if(this->config.EventCallback != nullptr) {
					this->config.EventCallback(this->config.callbackContext, Event::TransferComplete, epNum, actualCount);
				}
			}

			// Setup phase done
			if((epInt & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP) {
				// Clear flag
				WRITE_REG(outEp->DOEPINT, USB_OTG_DOEPINT_STUP);

				// Clear STPKTRX if set
				if((epInt & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX) {
					WRITE_REG(outEp->DOEPINT, USB_OTG_DOEPINT_STPKTRX);
				}

				// Parse setup packet when in DMA mode
				if(this->config.useDMA == true) {
					// Handle cache coherency
					System::InvalidateCache((uint32_t*)this->setupPacketBytes, sizeof(this->setupPacketBytes));

					uint32_t* setupWords = (uint32_t*)this->setupPacketBytes;
					this->setupPacket.requestType = (setupWords[0] & 0xFF);
					this->setupPacket.request = ((setupWords[0] >> 8) & 0xFF);
					this->setupPacket.value = ((setupWords[0] >> 16) & 0xFFFF);
					this->setupPacket.index = (setupWords[1] & 0xFFFF);
					this->setupPacket.length = ((setupWords[1] >> 16) & 0xFFFF);
				}

				if(this->config.EventCallback != nullptr) {
					this->config.EventCallback(this->config.callbackContext, Event::Setup, epNum, 8);
				}
			}

			// Endpoint disabled token received
			if((epInt & USB_OTG_DOEPINT_OTEPDIS) == USB_OTG_DOEPINT_OTEPDIS) {
				// Clear flag
				WRITE_REG(outEp->DOEPINT, USB_OTG_DOEPINT_OTEPDIS);
			}

			// Endpoint disabled
			if((epInt & USB_OTG_DOEPINT_EPDISD) == USB_OTG_DOEPINT_EPDISD) {
				// Clear flag
				WRITE_REG(outEp->DOEPINT, USB_OTG_DOEPINT_EPDISD);
			}

			// Received status phase
			if((epInt & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR) {
				// Clear flag
				WRITE_REG(outEp->DOEPINT, USB_OTG_DOEPINT_OTEPSPR);
			}

			// NACK packet transmitted
			if((epInt & USB_OTG_DOEPINT_NAK) == USB_OTG_DOEPINT_NAK) {
				// Clear flag
				WRITE_REG(outEp->DOEPINT, USB_OTG_DOEPINT_NAK);
			}
		}
		epNum += 1;
		epIntr = epIntr >> 1;
	}
}

void USB::HandleRxFifoInterrupt() {
	// Reading this register POPS the status from the top of the FIFO
	uint32_t status = this->instance->GRXSTSP;

	uint8_t epNum = status & USB_OTG_GRXSTSP_EPNUM;
	uint32_t byteCnt = (status & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos;
	uint32_t pktStatus = (status & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos;

	// Packet Status Definitions (from RM)
	// 2 = OUT Data Packet received
	// 6 = SETUP Packet received
	// 3, 4 = SETUP/COMP completed (Triggered elsewhere)
	if(pktStatus == STS_DATA_UPDT) {
		// Out data packet received
		if(byteCnt > 0) {
			// Read packet from FIFO
			volatile uint32_t* fifoPtr = (__IO uint32_t *)((uint32_t)this->instance + USB_OTG_FIFO_BASE + ((0) * USB_OTG_FIFO_SIZE));
			uint32_t wordCnt = (byteCnt) >> 2;		// Bytes to 32-bit word count, ceiled to closest larger integer
			uint32_t restCnt = byteCnt % 4;

			uint8_t* destPtr = this->outEpState[epNum].buffer;
			// Read full 32-bit word to buffer
			for(uint32_t i = 0; i < wordCnt; i++) {
				uint32_t data = *fifoPtr;
				if(this->outEpState[epNum].buffer != nullptr) {
					*destPtr++ = (data & 0xFF);
					*destPtr++ = ((data >> 8) & 0xFF);
					*destPtr++ = ((data >> 16) & 0xFF);
					*destPtr++ = ((data >> 24) & 0xFF);
				}
			}

			// Read remaining bytes (less then 4)
			if(restCnt > 0) {
				uint32_t data = *fifoPtr;
				if(this->outEpState[epNum].buffer != nullptr) {
					for(uint8_t i = 0; i < restCnt; i++) {
						*destPtr++ = (uint8_t)((data >> (i * 8)) & 0xFF);
					}
				}
			}

			// this->outEpState[epNum].count = byteCnt;

			// Update the state pointer so the next chunk appends correctly
			if(this->outEpState[epNum].buffer != nullptr) {
				this->outEpState[epNum].buffer = destPtr;
			}

			if(this->config.EventCallback != nullptr) {
				this->config.EventCallback(this->config.callbackContext, Event::TransferComplete, epNum, byteCnt);
			}
		}
	}
	else if(pktStatus == STS_SETUP_UPDT) {
		// Setup packet received

		// Read packet from FIFO
		volatile uint32_t* fifoPtr = (__IO uint32_t *)((uint32_t)this->instance + USB_OTG_FIFO_BASE + ((0) * USB_OTG_FIFO_SIZE));
		uint32_t setupPktWords[2];
		setupPktWords[0] = *fifoPtr;
		setupPktWords[1] = *fifoPtr;

		// Bytes to SetupPacket structure
		setupPacket.requestType = (setupPktWords[0] & 0xFF);
		setupPacket.request = ((setupPktWords[0] >> 8) & 0xFF);
		setupPacket.value = ((setupPktWords[0] >> 16) & 0xFFFF);
		setupPacket.index = (setupPktWords[1] & 0xFFFF);
		setupPacket.length = ((setupPktWords[1] >> 16) & 0xFFFF);

		// if(this->config.EventCallback != nullptr) {
		// 	this->config.EventCallback(this->config.callbackContext, Event::Setup, epNum, 8);
		// }
	}
	else {

	}
}

void USB::HandleTxFifoInterrupt(uint8_t epNum) {
	USB_OTG_INEndpointTypeDef* inEp = ((USB_OTG_INEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_IN_ENDPOINT_BASE + ((epNum) * USB_OTG_EP_REG_SIZE)));

	// Check FIFO free space(in 32-bit words)
	uint32_t wordsAvailable = (inEp->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) >> USB_OTG_DTXFSTS_INEPTFSAV_Pos;

	uint32_t bytesLeft = this->inEpState[epNum].count;
	uint32_t wordsLeft = (bytesLeft + 3) / 4; // Ceil to nearest word

	// Calculate batch write size
	uint32_t wordsToWrite = (wordsLeft > wordsAvailable) ? wordsAvailable : wordsLeft;

	volatile uint32_t* fifoPtr = (__IO uint32_t *)((uint32_t)this->instance + USB_OTG_FIFO_BASE + (epNum * USB_OTG_FIFO_SIZE));
	uint8_t* src = this->inEpState[epNum].buffer;

	// Pack bytes into 32-bit words and push to hardware safely
	for(uint32_t i = 0; i < wordsToWrite; i++) {
		uint32_t data = 0;
		if(bytesLeft >= 4) {
			// Fast copy, 32-bit reads
			data = *((uint32_t*)src);
			src += 4;
			bytesLeft -= 4;
		} 
		else {
			// Handle the trailing 1 to 3 bytes
			for(uint8_t b = 0; b < bytesLeft; b++) {
				data |= ((uint32_t)(src[b]) << (b * 8));
			}
			src += bytesLeft;
			bytesLeft = 0;
		}
		*fifoPtr = data; // Push to hardware
	}

	// Update endpoint buffer tracking
	this->inEpState[epNum].buffer = src;
	this->inEpState[epNum].count = bytesLeft;

	// If all bytes are written, mask this interrupt. 
	if(bytesLeft == 0) {
		CLEAR_BIT(this->device->DIEPEMPMSK, 1UL << (epNum & EP_ADDR_MSK));
	}
}

Status USB::FlushTxFifo(uint8_t epNum) {
	// Wait for bus idle
	uint32_t timeoutCount = 1000000;
	while((this->instance->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0x00) {
		timeoutCount--;
		if(timeoutCount == 0) {
			return Status::Error;
		}
	}

	// Flush TX Fifo
	MODIFY_REG(this->instance->GRSTCTL, USB_OTG_GRSTCTL_TXFNUM, USB_OTG_GRSTCTL_TXFFLSH | ((epNum) << USB_OTG_GRSTCTL_TXFNUM_Pos));
	
	timeoutCount = 1000000;
	while((this->instance->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH) {
		timeoutCount--;
		if(timeoutCount == 0) {
			return Status::Error;
		}
	}

	return Status::Ok;
}

Status USB::FlushRxFifo() {
	// Wait for bus idle
	uint32_t timeoutCount = 1000000;
	while((this->instance->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0x00) {
		timeoutCount--;
		if(timeoutCount == 0) {
			return Status::Error;
		}
	}

	// Flush TX Fifo
	MODIFY_REG(this->instance->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH, USB_OTG_GRSTCTL_RXFFLSH);
	
	timeoutCount = 1000000;
	while((this->instance->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH) {
		timeoutCount--;
		if(timeoutCount == 0) {
			return Status::Error;
		}
	}

	return Status::Ok;
}

void USB::StartEp0Setup() {
	USB_OTG_OUTEndpointTypeDef* outEp0 = ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_OUT_ENDPOINT_BASE));

	// Configure EP0 to receive 1 Setup Packet (8 bytes)
	// PKTCNT = 1, STUPCNT = 3 (Core requires STUPCNT=3 for back-to-back setup packets)
	uint32_t transferSize = 24;
	uint32_t pktCnt = 1;
	uint32_t stupCnt = 3;

	uint32_t tSizConfig = (transferSize & USB_OTG_DOEPTSIZ_XFRSIZ_Msk) |
							((pktCnt << USB_OTG_DOEPTSIZ_PKTCNT_Pos) & USB_OTG_DOEPTSIZ_PKTCNT_Msk) |
							((stupCnt << USB_OTG_DOEPTSIZ_STUPCNT_Pos) & USB_OTG_DOEPTSIZ_STUPCNT_Msk);

	WRITE_REG(outEp0->DOEPTSIZ, tSizConfig);
	// MODIFY_REG(outEp0->DOEPTSIZ, 
	// 			USB_OTG_DOEPTSIZ_PKTCNT | USB_OTG_DOEPTSIZ_XFRSIZ | USB_OTG_DOEPTSIZ_STUPCNT, 
	// 			(pktCnt << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | 
	// 			(stupCnt << USB_OTG_DOEPTSIZ_STUPCNT_Pos) | 
	// 			transferSize);

	// If DMA is enabled, point it to your internal setup buffer
	if(this->config.useDMA == true) {
		// Handle cache coherency
		System::InvalidateCache((uint32_t*)this->setupPacketBytes, sizeof(this->setupPacketBytes));

		WRITE_REG(outEp0->DOEPDMA, (uint32_t)this->setupPacketBytes);
		// MODIFY_REG(outEp0->DOEPCTL, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP);
		MODIFY_REG(outEp0->DOEPCTL, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_USBAEP, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_USBAEP);
	}

	// Enable Endpoint and Clear NAK
	// MODIFY_REG(outEp0->DOEPCTL, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK);
}

void USB::ConfigureRIF(void) {
	// Essential stuff so that USB DMA has access to SRAM regions!!
	// Without this the USB DMA can't access the SRAM, the transfer would not fail but would write 0 and read to nullptr
	// https://github.com/STMicroelectronics/STM32CubeN6/blob/13d4d2e5d89a0b8dacb673b2a4b6d2c9d3f6f688/Projects/STM32N6570-DK/Applications/USBX/Ux_Host_MSC/Core/Src/main.c#L438

	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);

	// Some RIF constants (from HAL). These are independet from the peripehral, are global "defines".
	const uint32_t RIF_PERIPH_REG1 = 0x10000000U;
	const uint32_t RIF_CID_1 = 0x00000002U;
	const uint32_t RIF_PERIPH_REG_SHIFT = 28U;
	const uint32_t RIF_PERIPH_BIT_POSITION = 0x0000001FU;

	// Peripheral specific RIF values
	uint32_t masterID = 0;
	uint32_t periphID = 0;
	if(this->instance == USB1_OTG_HS) {
		masterID = 4U;		// RIMU (Refernce Manual 6.3.4 Table 22 pg. 248)
		periphID = (RIF_PERIPH_REG1 | RIFSC_RISC_SECCFGRx_SEC24_Pos);
	}
	else if(this->instance == USB2_OTG_HS) {
		masterID = 5U;		// RIMU (Refernce Manual 6.3.4 Table 22 pg. 248)
		periphID = (RIF_PERIPH_REG1 | RIFSC_RISC_SECCFGRx_SEC25_Pos);
	}
	else {
		return;
	}

	// RIMC_ATTRx: Controls if USB DMA can read/write Secure/Privileged memory: Set to this master is secure and privilaged
	uint32_t masterCID = POSITION_VAL(RIF_CID_1);
	uint32_t wMask = (RIFSC_RIMC_ATTRx_MCID | RIFSC_RIMC_ATTRx_MPRIV | RIFSC_RIMC_ATTRx_MSEC);
	uint32_t wValue = ((masterCID << RIFSC_RIMC_ATTRx_MCID_Pos) | (0x03 << RIFSC_RIMC_ATTRx_MSEC_Pos));		//Bit 0: Master Secure; Bit 1: Master priviliged
	MODIFY_REG(RIFSC->RIMC_ATTRx[masterID], wMask, wValue);

	// Allows CPU to access USB registers in Secure/Privileged mode.
	// Slave security configuration register: 0: Secure and nonsecure data access are granted to the peripheral; 1: Secure data access only are granted to the peripheral
	wMask = (1UL << (periphID & RIF_PERIPH_BIT_POSITION));
	wValue = ((0x01) << (periphID & RIF_PERIPH_BIT_POSITION));		//0: Secure and nonsecure data access are granted; 1: Only secure access granted
	MODIFY_REG(RIFSC->RISC_SECCFGRx[periphID >> RIF_PERIPH_REG_SHIFT], wMask, wValue);

	// Slave privileged configuration register: 0: Privileged and unprivileged data access are granted to the peripheral; 1: Privileged data access only are granted to the peripheral
	wMask = (1UL << (periphID & RIF_PERIPH_BIT_POSITION));
	wValue = (0x01) << (periphID & RIF_PERIPH_BIT_POSITION);		//0: Secure and nonsecure data access are granted; 1: Only secure access granted
	MODIFY_REG(RIFSC->RISC_PRIVCFGRx[periphID >> RIF_PERIPH_REG_SHIFT], wMask, wValue);
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

void USB::InterruptHandler() {
	// Get interrupts, only enabled ones!
	this->irqStatus = this->instance->GINTSTS & this->instance->GINTMSK;

	// Handle RX FIFO Not Empty (RXFLVL)
	if(((this->irqStatus & USB_OTG_GINTSTS_RXFLVL) == USB_OTG_GINTSTS_RXFLVL)) {
		this->HandleRxFifoInterrupt();
	}

	// Handle OUT Endpoint Interrupts (OEPINT)
	if(((this->irqStatus & USB_OTG_GINTSTS_OEPINT) == USB_OTG_GINTSTS_OEPINT)) {
		this->HandleEpOutInterrupt();
	}

	// Handle IN Endpoint Interrupts (IEPINT)
	if(((this->irqStatus & USB_OTG_GINTSTS_IEPINT) == USB_OTG_GINTSTS_IEPINT)) {
		this->HandleEpInInterrupt();
	}

	// Handle Start of Frame (SOF)
	if(((this->irqStatus & USB_OTG_GINTSTS_SOF) == USB_OTG_GINTSTS_SOF)) {
		// Clear flags
		WRITE_REG(this->instance->GINTSTS, USB_OTG_GINTSTS_SOF);
		
		if(this->config.EventCallback != nullptr) {
			// Broadcast the SOF event to the USB classes
			this->config.EventCallback(this->config.callbackContext, Event::Sof, 0, 0);
		}
	}

	// Handle Incomplete Isochronous IN Transfer (IISOIXFR)
	if(((this->irqStatus & USB_OTG_GINTSTS_IISOIXFR) == USB_OTG_GINTSTS_IISOIXFR)) {
		// Clear flags
		WRITE_REG(this->instance->GINTSTS, USB_OTG_GINTSTS_IISOIXFR);

		// Find any Isochronous IN endpoint that is stuck (EPENA is still high)
		for(uint8_t i = 1; i < maxUserInEP; i++) {
			USB_OTG_INEndpointTypeDef* inEp = ((USB_OTG_INEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_IN_ENDPOINT_BASE + (i * USB_OTG_EP_REG_SIZE)));
			
			uint32_t epCtl = inEp->DIEPCTL;
			uint8_t epType = (epCtl & USB_OTG_DIEPCTL_EPTYP_Msk) >> USB_OTG_DIEPCTL_EPTYP_Pos;
			
			if((epCtl & USB_OTG_DIEPCTL_EPENA) && (epType == 0x01)) { // 0x01 is Isochronous
				// Forcefully disable it and set SNAK to flush the failed packet
				inEp->DIEPCTL |= (USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK);
			}
		}
	}

	// Handle Incomplete Isochronous OUT Transfer (PXFR_INCOMPISOOUT)
	if(((this->irqStatus & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT) == USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)) {
		// Clear flags
		WRITE_REG(this->instance->GINTSTS, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT);
	}

	// Handle Bus Reset (USBRST)
	if(((this->irqStatus & USB_OTG_GINTSTS_USBRST) == USB_OTG_GINTSTS_USBRST)) {
		// Clear flags
		WRITE_REG(this->instance->GINTSTS, USB_OTG_GINTSTS_USBRST);

		// Remote wakeup signaling
		MODIFY_REG(this->device->DCTL, USB_OTG_DCTL_RWUSIG, 0x00);

		// Flush TX FIFO
		this->FlushTxFifo(0x10);
		this->FlushRxFifo();

		// Endpoint scrubbing/clearing
		for(uint8_t i = 0; i < 8; i++) {
			USB_OTG_INEndpointTypeDef* inEp = ((USB_OTG_INEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_IN_ENDPOINT_BASE + (i * USB_OTG_EP_REG_SIZE)));
			USB_OTG_OUTEndpointTypeDef* outEp = ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)this->instance + USB_OTG_OUT_ENDPOINT_BASE + (i * USB_OTG_EP_REG_SIZE)));

			// 0xFB7FU clears all possible endpoint interrupt flags
			WRITE_REG(inEp->DIEPINT, 0xFB7FU);
			WRITE_REG(outEp->DOEPINT, 0xFB7FU);

			// Clear stall conditions
			CLEAR_BIT(inEp->DIEPCTL, USB_OTG_DIEPCTL_STALL);
			CLEAR_BIT(outEp->DOEPCTL, USB_OTG_DOEPCTL_STALL);
			
			// Set NAK on all OUT endpoints to prevent premature data reception
			SET_BIT(outEp->DOEPCTL, USB_OTG_DOEPCTL_SNAK);
		}

		// Unmaske enpoint 0 interrupts: Bit 0 (EP0 IN) | Bit 16 (EP0 OUT)
		MODIFY_REG(this->device->DAINTMSK, 0x00, 0x00010001U);

		// Enable essential OUT endpoint interrupts (Setup Phase, Transfer Complete, Endpoint Disabled)
		WRITE_REG(this->device->DOEPMSK, 	USB_OTG_DOEPMSK_STUPM | 
											USB_OTG_DOEPMSK_XFRCM |
											USB_OTG_DOEPMSK_EPDM | 
											USB_OTG_DOEPMSK_OTEPSPRM | 
											USB_OTG_DOEPMSK_NAKM);
		
		// Enable essential IN endpoint interrupts (Transfer Complete, Timeout, Endpoint Disabled)
		WRITE_REG(this->device->DIEPMSK,	USB_OTG_DIEPMSK_TOM | 
											USB_OTG_DIEPMSK_XFRCM | 
											USB_OTG_DIEPMSK_EPDM);

		// Reset Device Address to 0
		MODIFY_REG(this->device->DCFG, USB_OTG_DCFG_DAD, 0x00);

		// Unmasked some interrupts
		MODIFY_REG(this->instance->GINTMSK, USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IEPINT, USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IEPINT);

		this->StartEp0Setup();

		if(this->config.EventCallback != nullptr) {
			this->config.EventCallback(this->config.callbackContext, Event::Reset, 0, 0);
		}
	}

	// Handle Enumeration Done (ENUMDNE)
	if(((this->irqStatus & USB_OTG_GINTSTS_ENUMDNE) == USB_OTG_GINTSTS_ENUMDNE)) {
		// Clear flags
		WRITE_REG(this->instance->GINTSTS, USB_OTG_GINTSTS_ENUMDNE);

		// Explicitly set EP0 Max Packet Size to 64 bytes (0x00)
		USB_OTG_INEndpointTypeDef* inEp0 = ((USB_OTG_INEndpointTypeDef *)(uint32_t)((uint32_t)this->instance + USB_OTG_IN_ENDPOINT_BASE));
		// MODIFY_REG(inEp0->DIEPCTL, USB_OTG_DIEPCTL_MPSIZ_Msk, 0x00);
		inEp0->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;

		MODIFY_REG(this->device->DCTL, USB_OTG_DCTL_CGINAK, USB_OTG_DCTL_CGINAK);

		// Set turnaround time
		uint32_t turnTime = 0x9U;
		if(this->config.speed == USB::BusSpeed::High) {
			turnTime = 0x9U;	// For High-Speed
		}
		else {
			turnTime = 0x6U;	// For Full-Speed and HCLK Clock Range between 32-200 MHz and 
		}
		MODIFY_REG(this->instance->GUSBCFG, USB_OTG_GUSBCFG_TRDT, ((turnTime) << USB_OTG_GUSBCFG_TRDT_Pos));

		// Ungate the detailed endpoint interrupts
		this->device->DAINTMSK = 0xFFFFFFFF;

		// this->StartEp0Setup();
	}

	// Handle Suspend (USBSUSP) and Resume (WKUINT)
	if(((this->irqStatus & USB_OTG_GINTSTS_WKUINT) == USB_OTG_GINTSTS_WKUINT)) {
		// Clear flags
		WRITE_REG(this->instance->GINTSTS, USB_OTG_GINTSTS_WKUINT);
		if(this->config.EventCallback != nullptr) {
			this->config.EventCallback(this->config.callbackContext, Event::Resume, 0, 0);
		}
	}
	if(((this->irqStatus & USB_OTG_GINTSTS_USBSUSP) == USB_OTG_GINTSTS_USBSUSP)) {
		// Clear flags
		WRITE_REG(this->instance->GINTSTS, USB_OTG_GINTSTS_USBSUSP);
		if(this->config.EventCallback != nullptr) {
			this->config.EventCallback(this->config.callbackContext, Event::Suspend, 0, 0);
		}
	}
}