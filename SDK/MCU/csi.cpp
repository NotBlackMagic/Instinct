/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/csi.cpp
 */

#include "csi.hpp"

// Internal helper structure for Synopsys D-PHY frequency parameters
struct PhyParams {
	uint32_t maxBitrateMbps;
	uint8_t hsFreqRange;
	uint16_t oscFreqTarget;
};

// Synopsys D-PHY frequency table mapped to max bitrates (from STM HAL)
static constexpr uint16_t phyParamsTableSize = 63;
static constexpr PhyParams phyParamsTable[phyParamsTableSize] = {
	{ 80, 0x00U, 460U },
	{ 90, 0x10U, 460U },
	{ 100, 0x20U, 460U },
	{ 110, 0x30U, 460U },
	{ 120, 0x01U, 460U },
	{ 130, 0x11U, 460U },
	{ 140, 0x21U, 460U },
	{ 150, 0x31U, 460U },
	{ 160, 0x02U, 460U },
	{ 170, 0x12U, 460U },
	{ 180, 0x22U, 460U },
	{ 190, 0x32U, 460U },
	{ 205, 0x03U, 460U },
	{ 220, 0x13U, 460U },
	{ 235, 0x23U, 460U },
	{ 250, 0x33U, 460U },
	{ 275, 0x04U, 460U },
	{ 300, 0x14U, 460U },
	{ 325, 0x25U, 460U },
	{ 350, 0x35U, 460U },
	{ 400, 0x05U, 460U },
	{ 450, 0x16U, 460U },
	{ 500, 0x26U, 460U },
	{ 550, 0x37U, 460U },
	{ 600, 0x07U, 460U },
	{ 650, 0x18U, 460U },
	{ 700, 0x28U, 460U },
	{ 750, 0x39U, 460U },
	{ 800, 0x09U, 460U },
	{ 850, 0x19U, 460U },
	{ 900, 0x29U, 460U },
	{ 950, 0x3AU, 460U },
	{ 1000, 0x0AU, 460U },
	{ 1050, 0x1AU, 460U },
	{ 1100, 0x2AU, 460U },
	{ 1150, 0x3BU, 460U },
	{ 1200, 0x0BU, 460U },
	{ 1250, 0x1BU, 460U },
	{ 1300, 0x2BU, 460U },
	{ 1350, 0x3CU, 460U },
	{ 1400, 0x0CU, 460U },
	{ 1450, 0x1CU, 460U },
	{ 1500, 0x2CU, 460U },
	{ 1550, 0x3DU, 285U },
	{ 1600, 0x0DU, 295U },
	{ 1650, 0x1DU, 304U },
	{ 1700, 0x2EU, 313U },
	{ 1750, 0x3EU, 322U },
	{ 1800, 0x0EU, 331U },
	{ 1850, 0x1EU, 341U },
	{ 1900, 0x2FU, 350U },
	{ 1950, 0x3FU, 359U },
	{ 2000, 0x0FU, 368U },
	{ 2050, 0x40U, 377U },
	{ 2100, 0x41U, 387U },
	{ 2150, 0x42U, 396U },
	{ 2200, 0x43U, 405U },
	{ 2250, 0x44U, 414U },
	{ 2300, 0x45U, 423U },
	{ 2350, 0x46U, 432U },
	{ 2400, 0x47U, 442U },
	{ 2450, 0x48U, 451U },
	{ 2500, 0x49U, 460U }
};

Csi::Csi(CSI_TypeDef *instance) : instance(instance) {
	this->isInitialized = false;
	this->irqPriority = 0x0E; // Lowest priority (safe default)
}

Status Csi::Init(const Config &config) {
	if(this->isInitialized == true) {
		return Status::Ok;
	}

	this->config = config;

	// Enable bus clocks
	if(this->instance == CSI) {
		LL_APB5_GRP1_EnableClock(LL_APB5_GRP1_PERIPH_CSI);
		this->irqCall = CSI_IRQn;
	}

	// Ensure CSI is disabled before configuration
	CLEAR_BIT(this->instance->CR, CSI_CR_CSIEN);

	// Configure the Lane Merger (LMCFGR)
	uint32_t laneConfig = (this->config.lanes == LaneCount::Two) ? 0x02 : 0x01;
	if(config.laneMapping == LaneMapping::Direct) {
		WRITE_REG(this->instance->LMCFGR, (0x02 << CSI_LMCFGR_DL1MAP_Pos) | (0x01 << CSI_LMCFGR_DL0MAP_Pos) | (laneConfig << CSI_LMCFGR_LANENB_Pos));
	}
	else {
		WRITE_REG(this->instance->LMCFGR, (0x01 << CSI_LMCFGR_DL1MAP_Pos) | (0x02 << CSI_LMCFGR_DL0MAP_Pos) | (laneConfig << CSI_LMCFGR_LANENB_Pos));
	}

	// Enable the CSI core
	SET_BIT(this->instance->CR, CSI_CR_CSIEN);

	// Find D-PHY timing parameters based on requested bitrate
	uint8_t hsFreq = 0;
	uint16_t oscFreq = 0;
	uint32_t bitrateMbps = config.bitrate / 1000000;
	for (int i = 0; i < phyParamsTableSize; i++) {
		if (bitrateMbps <= phyParamsTable[i].maxBitrateMbps) {
			hsFreq = phyParamsTable[i].hsFreqRange;
			oscFreq = phyParamsTable[i].oscFreqTarget;
			break;
		}
	}
	if(oscFreq == 0) {
		// Bitrate too high or unsupported
		return Status::Error;
	}

	// Start D-PHY Configuration
	CLEAR_BIT(this->instance->PRCR, CSI_PRCR_PEN);	// Stop PHY
	CLEAR_REG(this->instance->PCR);					// Disable all lanes

	// Set testclk (clock enable) on during 15ns
	SET_BIT(this->instance->PTCR0, CSI_PTCR0_TCKEN);
	Time::Delay(1);
	CLEAR_REG(this->instance->PTCR0);

	// Set hsfreqrange
	MODIFY_REG(this->instance->PFCR, CSI_PFCR_HSFR, (0x28U << CSI_PFCR_CCFR_Pos) | (hsFreq << CSI_PFCR_HSFR_Pos));

	// Write PHY internal test registers via macro-like helper
	WritePhyRegister(0x00, 0x08, 0x38);	// deskew_polarity_rw = 1
	WritePhyRegister(0x00, 0xE4, 0x11);	// counter_for_des_en_config_if_rx + DLL prog EN

	// Set DLL target oscillation frequency
	WritePhyRegister(0x00, 0xE3, (oscFreq >> 8) & 0xFF);
	WritePhyRegister(0x00, 0xE3, oscFreq & 0xFF);

	// Set Base direction to RX (Synopsys 1 RX, 0 TX) + freq range
	WRITE_REG(this->instance->PFCR, (0x28U << CSI_PFCR_CCFR_Pos) | (hsFreq << CSI_PFCR_HSFR_Pos) | CSI_PFCR_DLD);

	// Enable D-PHY RX lane(s) and Clock lane
	if(config.lanes == LaneCount::One) {
		// Single lane mode
		WRITE_REG(this->instance->PCR, CSI_PCR_DL0EN | CSI_PCR_CLEN | CSI_PCR_PWRDOWN);
	}
	else {
		// Dual lane mode
		WRITE_REG(this->instance->PCR, CSI_PCR_DL0EN | CSI_PCR_DL1EN | CSI_PCR_CLEN | CSI_PCR_PWRDOWN);		// Start up in Active state (WRONG IN REFERENCE MANUAL?? This is based on the driver code from STM32)
	}

	// Enable PHY (Bring out of reset)
	SET_BIT(this->instance->PRCR, CSI_PRCR_PEN);

	// Remove forces
	CLEAR_REG(this->instance->PMCR);

	// Enable common interrupts, common to all virtual channels
	MODIFY_REG(this->instance->IER0, 0x00, CSI_IER0_CCFIFOFIE | CSI_IER0_SYNCERRIE | CSI_IER0_SPKTERRIE | CSI_IER0_IDERRIE | CSI_IER0_SPKTIE);
	if(config.lanes == LaneCount::One) {
		if(config.laneMapping == LaneMapping::Direct) {
			// Single lane direct mapping
			MODIFY_REG(this->instance->IER1, 0x00, CSI_IER1_ESOTDL0IE | CSI_IER1_ESOTSYNCDL0IE | CSI_IER1_EESCDL0IE | CSI_IER1_ESYNCESCDL0IE | CSI_IER1_ECTRLDL0IE);
		}
		else {
			// Single lane switched/reversed mapping
			MODIFY_REG(this->instance->IER1, 0x00, CSI_IER1_ESOTDL1IE | CSI_IER1_ESOTSYNCDL1IE | CSI_IER1_EESCDL1IE | CSI_IER1_ESYNCESCDL1IE | CSI_IER1_ECTRLDL1IE);
		}
	}
	else {
		// Dual lane
		MODIFY_REG(this->instance->IER1, 0x00,	CSI_IER1_ESOTDL1IE | CSI_IER1_ESOTSYNCDL1IE | CSI_IER1_EESCDL1IE | CSI_IER1_ESYNCESCDL1IE | CSI_IER1_ECTRLDL1IE |
												CSI_IER1_ESOTDL0IE | CSI_IER1_ESOTSYNCDL0IE | CSI_IER1_EESCDL0IE | CSI_IER1_ESYNCESCDL0IE | CSI_IER1_ECTRLDL0IE);
	}

	// Configure CSI Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);

	isInitialized = true;
	return Status::Ok;
}

Status Csi::ConfigureVirtualChannel(VirtualChannel vc, MIPIDataType dataType) {
	// Data type formats:
	// 0x0: (BPP6) 6-bit words (for example RAW6, RGB666)
	// 0x1: (BPP7) 7-bit words (for example RAW7)
	// 0x2: (BPP8) 8-bit words (for example  RAW8, YUV 8 bits, RGB888, RGB444, RGB555, RGB565, JPEG)
	// 0x3: (BPP10) 10-bit words (for example RAW10, YUV 10 bits)
	// 0x0: (BPP12) 12-bit words (for example RAW12)
	// 0x5: (BPP14) 14-bit words (for example RAW14)
	// 0x6: (BPP16) 16-bit words (for example RAW16)

	// For granular data type filtering, with ALLDT=0, set also:
	// SET_BIT(this->instance->VC0CFGR1, CSI_VC0CFGR1_DT0EN):
	// MODIFY_REG(this->instance->VC0CFGR1, CSI_VC0CFGR1_DT0_Msk | CSI_VC0CFGR1_DT0FT_Msk, (static_cast<uint8_t>(dataType) << CSI_VC0CFGR1_DT0_Pos) | (static_cast<uint8_t>(dataTypeFormat) << CSI_VC0CFGR1_DT0FT_Pos));

	uint8_t dataTypeFormat = 0x02;	// 8-bit words (for example RAW8, YUV 8 bits, RGB888, RGB444, RGB555, RGB565, JPEG)
	switch(vc) {
		case VirtualChannel::VC0:
			WRITE_REG(this->instance->VC0CFGR1, (dataTypeFormat << CSI_VC0CFGR1_CDTFT_Pos) | CSI_VC0CFGR1_ALLDT);
			break;
		case VirtualChannel::VC1:
			WRITE_REG(this->instance->VC1CFGR1, (dataTypeFormat << CSI_VC1CFGR1_CDTFT_Pos) | CSI_VC1CFGR1_ALLDT);
			break;
		case VirtualChannel::VC2:
			WRITE_REG(this->instance->VC2CFGR1, (dataTypeFormat << CSI_VC2CFGR1_CDTFT_Pos) | CSI_VC2CFGR1_ALLDT);
			break;
		case VirtualChannel::VC3:
			WRITE_REG(this->instance->VC3CFGR1, (dataTypeFormat << CSI_VC3CFGR1_CDTFT_Pos) | CSI_VC3CFGR1_ALLDT);
			break;
		default:
			return Status::Error;
	}

	return Status::Ok;
}

Status Csi::Start(VirtualChannel vc) {
	switch(vc) {
		case VirtualChannel::VC0: 
			SET_BIT(this->instance->CR, CSI_CR_VC0START);
			break;
		case VirtualChannel::VC1:
			SET_BIT(this->instance->CR, CSI_CR_VC1START);
			break;
		case VirtualChannel::VC2:
			SET_BIT(this->instance->CR, CSI_CR_VC2START);
			break;
		case VirtualChannel::VC3:
			SET_BIT(this->instance->CR, CSI_CR_VC3START);
			break;
		default:
			return Status::Error;
	}

	// // Wait for the VC to enter active state
	// uint64_t timestamp = Time::GetUs();
	// while(true) {
	// 	if((this->instance->SR0 & (CSI_SR0_VC0STATEF << static_cast<uint8_t>(vc))) == (CSI_SR0_VC0STATEF << static_cast<uint8_t>(vc))) {
	// 		break;
	// 	}

	// 	if((Time::GetUs() - timestamp) > 1000) {
	// 		// Wait for Busy timeout
	// 		return Status::Timeout;
	// 	}
	// }

	// Enable the SOF and EOF interrupts for the selected virtual channel
	MODIFY_REG(this->instance->IER0, 0x00, (CSI_IER0_EOF0IE << static_cast<uint8_t>(vc)) | (CSI_IER0_SOF0IE << static_cast<uint8_t>(vc)));

	// Start up in Active state (WRONG IN REFERENCE MANUAL?? This is based on the driver code from STM32)
	MODIFY_REG(this->instance->PCR, CSI_PCR_PWRDOWN, CSI_PCR_PWRDOWN);

	return Status::Ok;
}

Status Csi::Stop(VirtualChannel vc) {
	switch(vc) {
		case VirtualChannel::VC0: 
			SET_BIT(this->instance->CR, CSI_CR_VC0STOP);
			break;
		case VirtualChannel::VC1:
			SET_BIT(this->instance->CR, CSI_CR_VC1STOP);
			break;
		case VirtualChannel::VC2:
			SET_BIT(this->instance->CR, CSI_CR_VC2STOP);
			break;
		case VirtualChannel::VC3:
			SET_BIT(this->instance->CR, CSI_CR_VC3STOP);
			break;
		default:
			return Status::Error;
	}

	// Start up in Power Down state (WRONG IN REFERENCE MANUAL?? This is based on the driver code from STM32)
	MODIFY_REG(this->instance->PCR, CSI_PCR_PWRDOWN, 0x00);

	// // Wait for the VC to enter active state
	// uint64_t timestamp = Time::GetUs();
	// while(true) {
	// 	if((this->instance->SR0 & (CSI_SR0_VC0STATEF << static_cast<uint8_t>(vc))) != (CSI_SR0_VC0STATEF << static_cast<uint8_t>(vc))) {
	// 		break;
	// 	}

	// 	if((Time::GetUs() - timestamp) > 1000) {
	// 		// Wait for Busy timeout
	// 		return Status::Timeout;
	// 	}
	// }

	// Disable the SOF and EOF interrupts for the selected virtual channel
	MODIFY_REG(this->instance->IER0, (CSI_IER0_EOF0IE << static_cast<uint8_t>(vc)) | (CSI_IER0_SOF0IE << static_cast<uint8_t>(vc)), 0x00);

	return Status::Ok;
}

Status Csi::GetLastPacketInfo(VirtualChannel& vc, MIPIDataType& dataType, uint16_t& data) {
	uint32_t spdfr = READ_REG(this->instance->SPDFR);
	vc = static_cast<VirtualChannel>((spdfr & CSI_SPDFR_VCHANNEL_Msk) >> CSI_SPDFR_VCHANNEL_Pos);
	dataType = static_cast<MIPIDataType>((spdfr & CSI_SPDFR_DATATYPE_Msk) >> CSI_SPDFR_DATATYPE_Pos);
	data = (spdfr & CSI_SPDFR_DATAFIELD_Msk) >> CSI_SPDFR_DATAFIELD_Pos;
	return Status::Ok;
}

void Csi::WritePhyRegister(uint32_t regMsb, uint32_t regLsb, uint32_t val) {
	// Based on STM HALL: https://github.com/STMicroelectronics/stm32n6xx-hal-driver/blob/cf84d98ae66419c7bfdc25f6c65954585a4e3811/Src/stm32n6xx_hal_dcmipp.c#L8161

	SET_BIT(this->instance->PTCR1, CSI_PTCR1_TWM);		// Set testen to high
	SET_BIT(this->instance->PTCR0, CSI_PTCR0_TCKEN);	// Set testclk to high
	SET_BIT(this->instance->PTCR1, CSI_PTCR1_TWM);		// Place 0x00 in testdin
	CLEAR_REG(this->instance->PTCR0);					// Set testclk to low (with the falling edge on testclk, the testdin signal content is latched internally)
	CLEAR_REG(this->instance->PTCR1);					// Set testen to low

	// Write MSB
	SET_BIT(this->instance->PTCR1, regMsb & 0xFFU);		// Place the 8-bit word corresponding to the testcode MSBs in testdin
	SET_BIT(this->instance->PTCR0, CSI_PTCR0_TCKEN);	// Set testclk to high
	CLEAR_REG(this->instance->PTCR0);					// Set testclk to low

	// Write LSB
	SET_BIT(this->instance->PTCR1, CSI_PTCR1_TWM);		// Set testen to high
	SET_BIT(this->instance->PTCR0, CSI_PTCR0_TCKEN);	// Set testclk to high
	SET_BIT(this->instance->PTCR1, CSI_PTCR1_TWM | (regLsb & 0xFFU));	// Place the 8-bit word test data in testdin
	CLEAR_REG(this->instance->PTCR0);					// Set testclk to low (with the falling edge on testclk, the testdin signal content is latched internally)
	CLEAR_REG(this->instance->PTCR1);					// Set testen to low

	// Write Value
	SET_BIT(this->instance->PTCR1, val & 0xFFU);		// Place the 8-bit word corresponding to the page offset in testdin
	SET_BIT(this->instance->PTCR0, CSI_PTCR0_TCKEN);	// Set testclk to high (test data is programmed internally)
	CLEAR_REG(this->instance->PTCR0);					// Finish by setting testclk to low
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

void Csi::InterruptHandler() {
	uint32_t sr0 = this->instance->SR0;
	uint32_t sr1 = this->instance->SR1;
	uint32_t ier0 = this->instance->IER0;
	uint32_t ier1 = this->instance->IER1;

	// Handle Clock changer FIFO full
	if(((sr0 & CSI_SR0_CCFIFOFF) == CSI_SR0_CCFIFOFF) && ((ier0 & CSI_IER0_CCFIFOFIE) == CSI_IER0_CCFIFOFIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_CCFIFOFF);
	}

	// Handle Byte/Line Counter
	if(((sr0 & CSI_SR0_LB3F) == CSI_SR0_LB3F) && ((ier0 & CSI_IER0_LB3IE) == CSI_IER0_LB3IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_LB3F);
	}
	if(((sr0 & CSI_SR0_LB2F) == CSI_SR0_LB2F) && ((ier0 & CSI_IER0_LB2IE) == CSI_IER0_LB2IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_LB2F);
	}
	if(((sr0 & CSI_SR0_LB1F) == CSI_SR0_LB1F) && ((ier0 & CSI_IER0_LB1IE) == CSI_IER0_LB1IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_LB1F);
	}
	if(((sr0 & CSI_SR0_LB0F) == CSI_SR0_LB0F) && ((ier0 & CSI_IER0_LB0IE) == CSI_IER0_LB0IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_LB0F);
	}

	// Handle End Of Frame
	if(((sr0 & CSI_SR0_EOF3F) == CSI_SR0_EOF3F) && ((ier0 & CSI_IER0_EOF3IE) == CSI_IER0_EOF3IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_EOF3F);
	}
	if(((sr0 & CSI_SR0_EOF2F) == CSI_SR0_EOF2F) && ((ier0 & CSI_IER0_EOF2IE) == CSI_IER0_EOF2IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_EOF2F);
	}
	if(((sr0 & CSI_SR0_EOF1F) == CSI_SR0_EOF1F) && ((ier0 & CSI_IER0_EOF1IE) == CSI_IER0_EOF1IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_EOF1F);
	}
	if(((sr0 & CSI_SR0_EOF0F) == CSI_SR0_EOF0F) && ((ier0 & CSI_IER0_EOF0IE) == CSI_IER0_EOF0IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_EOF0F);
	}

	// Handle Start Of Frame
	if(((sr0 & CSI_SR0_SOF3F) == CSI_SR0_SOF3F) && ((ier0 & CSI_IER0_SOF3IE) == CSI_IER0_SOF3IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_SOF3F);
	}
	if(((sr0 & CSI_SR0_SOF2F) == CSI_SR0_SOF2F) && ((ier0 & CSI_IER0_SOF2IE) == CSI_IER0_SOF2IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_SOF2F);
	}
	if(((sr0 & CSI_SR0_SOF1F) == CSI_SR0_SOF1F) && ((ier0 & CSI_IER0_SOF1IE) == CSI_IER0_SOF1IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_SOF1F);
	}
	if(((sr0 & CSI_SR0_SOF0F) == CSI_SR0_SOF0F) && ((ier0 & CSI_IER0_SOF0IE) == CSI_IER0_SOF0IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_SOF0F);
	}

	// Handle Timer
	if(((sr0 & CSI_SR0_TIM3F) == CSI_SR0_TIM3F) && ((ier0 & CSI_IER0_TIM3IE) == CSI_IER0_TIM3IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_TIM3F);
	}
	if(((sr0 & CSI_SR0_TIM2F) == CSI_SR0_TIM2F) && ((ier0 & CSI_IER0_TIM2IE) == CSI_IER0_TIM2IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_TIM2F);
	}
	if(((sr0 & CSI_SR0_TIM1F) == CSI_SR0_TIM1F) && ((ier0 & CSI_IER0_TIM1IE) == CSI_IER0_TIM1IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_TIM1F);
	}
	if(((sr0 & CSI_SR0_TIM0F) == CSI_SR0_TIM0F) && ((ier0 & CSI_IER0_TIM0IE) == CSI_IER0_TIM0IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_TIM0F);
	}

	// Handle Synchronization error
	if(((sr0 & CSI_SR0_SYNCERRF) == CSI_SR0_SYNCERRF) && ((ier0 & CSI_IER0_SYNCERRIE) == CSI_IER0_SYNCERRIE)) {
		// Disable the interrupt
		CLEAR_BIT(this->instance->IER0, CSI_IER0_SYNCERRIE);
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_SYNCERRF);
	}

	// Handle Watchdog error
	if(((sr0 & CSI_SR0_WDERRF) == CSI_SR0_WDERRF) && ((ier0 & CSI_IER0_WDERRIE) == CSI_IER0_WDERRIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_WDERRF);
	}

	// Handle Shorter packet than expected
	if(((sr0 & CSI_SR0_SPKTERRF) == CSI_SR0_SPKTERRF) && ((ier0 & CSI_IER0_SPKTERRIE) == CSI_IER0_SPKTERRIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_SPKTERRF);
	}

	// Handle data ID information error
	if(((sr0 & CSI_SR0_IDERRF) == CSI_SR0_IDERRF) && ((ier0 & CSI_IER0_IDERRIE) == CSI_IER0_IDERRIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_IDERRF);
	}

	// Handle corrected ECC error
	if(((sr0 & CSI_SR0_CECCERRF) == CSI_SR0_CECCERRF) && ((ier0 & CSI_IER0_CECCERRIE) == CSI_IER0_CECCERRIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_CECCERRF);
	}

	// Handle ECC error
	if(((sr0 & CSI_SR0_ECCERRF) == CSI_SR0_ECCERRF) && ((ier0 & CSI_IER0_ECCERRIE) == CSI_IER0_ECCERRIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_ECCERRF);
	}

	// Handle CRC error
	if(((sr0 & CSI_SR0_CRCERRF) == CSI_SR0_CRCERRF) && ((ier0 & CSI_IER0_CRCERRIE) == CSI_IER0_CRCERRIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_CRCERRF);
	}

	// Lane 0 Errors
	// Handle Start Of Transmission error
	if(((sr1 & CSI_SR1_ESOTDL0F) == CSI_SR1_ESOTDL0F) && ((ier1 & CSI_IER1_ESOTDL0IE) == CSI_IER1_ESOTDL0IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR1, CSI_SR1_ESOTDL0F);
	}
	// Handle Start Of Transmission Synchronization error
	if(((sr1 & CSI_SR1_ESOTSYNCDL0F) == CSI_SR1_ESOTSYNCDL0F) && ((ier1 & CSI_IER1_ESOTSYNCDL0IE) == CSI_IER1_ESOTSYNCDL0IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR1, CSI_SR1_ESOTSYNCDL0F);
	}
	// Handle Escape entry error
	if(((sr1 & CSI_SR1_EESCDL0F) == CSI_SR1_EESCDL0F) && ((ier1 & CSI_IER1_EESCDL0IE) == CSI_IER1_EESCDL0IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR1, CSI_SR1_EESCDL0F);
	}
	// Handle Low power data transmission synchronization error
	if(((sr1 & CSI_SR1_ESYNCESCDL0F) == CSI_SR1_ESYNCESCDL0F) && ((ier1 & CSI_IER1_ESYNCESCDL0IE) == CSI_IER1_ESYNCESCDL0IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR1, CSI_SR1_ESYNCESCDL0F);
	}
	// Handle error control on data line
	if(((sr1 & CSI_SR1_ECTRLDL0F) == CSI_SR1_ECTRLDL0F) && ((ier1 & CSI_IER1_ECTRLDL0IE) == CSI_IER1_ECTRLDL0IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR1, CSI_SR1_ECTRLDL0F);
	}

	// Lane 1 Errors
	// Handle Start Of Transmission error
	if(((sr1 & CSI_SR1_ESOTDL1F) == CSI_SR1_ESOTDL1F) && ((ier1 & CSI_IER1_ESOTDL1IE) == CSI_IER1_ESOTDL1IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR1, CSI_SR1_ESOTDL1F);
	}
	// Handle Start Of Transmission Synchronization error
	if(((sr1 & CSI_SR1_ESOTSYNCDL1F) == CSI_SR1_ESOTSYNCDL1F) && ((ier1 & CSI_IER1_ESOTSYNCDL1IE) == CSI_IER1_ESOTSYNCDL1IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR1, CSI_SR1_ESOTSYNCDL1F);
	}
	// Handle Escape entry error
	if(((sr1 & CSI_SR1_EESCDL1F) == CSI_SR1_EESCDL1F) && ((ier1 & CSI_IER1_EESCDL1IE) == CSI_IER1_EESCDL1IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR1, CSI_SR1_EESCDL1F);
	}
	// Handle Low power data transmission synchronization error
	if(((sr1 & CSI_SR1_ESYNCESCDL1F) == CSI_SR1_ESYNCESCDL1F) && ((ier1 & CSI_IER1_ESYNCESCDL1IE) == CSI_IER1_ESYNCESCDL1IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR1, CSI_SR1_ESYNCESCDL1F);
	}
	// Handle error control on data line
	if(((sr1 & CSI_SR1_ECTRLDL1F) == CSI_SR1_ECTRLDL1F) && ((ier1 & CSI_IER1_ECTRLDL1IE) == CSI_IER1_ECTRLDL1IE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR1, CSI_SR1_ECTRLDL1F);
	}

	// Handle error control on data line
	if(((sr0 & CSI_SR0_SPKTF) == CSI_SR0_SPKTF) && ((ier0 & CSI_IER0_SPKTIE) == CSI_IER0_SPKTIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->FCR0, CSI_SR0_SPKTF);
	}
}