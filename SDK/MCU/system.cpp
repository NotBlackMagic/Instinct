/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/system.cpp
 */

#include "system.hpp"

// ============================================================================
// System implementations
// ============================================================================

void System::InitClock(void) {
	//Configure the system/core power supply
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_PWR);
	LL_PWR_ConfigSupply(LL_PWR_SMPS_SUPPLY);
	while (LL_PWR_IsActiveFlag_ACTVOSRDY() == 0U);

	//Configure the IO power supply
	LL_PWR_SetVddIOVoltageRange(LL_PWR_VDDIO_VOLTAGE_RANGE_3V3);		//Set the VDD IO voltage range: 3.3V
	LL_PWR_EnableVddIO2();		//Enable VDD IO2 (PO[5:0] and PP[15:0]) voltage supply
	LL_PWR_SetVddIO2VoltageRange(LL_PWR_VDDIO_VOLTAGE_RANGE_1V8);		//Set the VDD IO2 voltage range: 1.8V
	// while (LL_PWR_IsActiveFlag_VDDIO2RDY() == 0U);
	LL_PWR_EnableVddIO3();		//Enable VDD IO3 (PN[12:0]) voltage supply
	LL_PWR_SetVddIO3VoltageRange(LL_PWR_VDDIO_VOLTAGE_RANGE_1V8);		//Set the VDD IO3 voltage range: 1.8V
	// while (LL_PWR_IsActiveFlag_VDDIO3RDY() == 0U);
	LL_PWR_EnableVddIO4();		//Enable VDD IO4 (PC[1], PC[12:6], and PH[9:2]) voltage supply
	LL_PWR_SetVddIO4VoltageRange(LL_PWR_VDDIO_VOLTAGE_RANGE_3V3);		//Set the VDD IO4 voltage range: 3.3V
	// while (LL_PWR_IsActiveFlag_VDDIO4RDY() == 0U);
	LL_PWR_EnableVddIO5();		//Enable VDD IO5 (PC[0], PC[5:2], and PE[4]) voltage supply
	LL_PWR_SetVddIO5VoltageRange(LL_PWR_VDDIO_VOLTAGE_RANGE_3V3);		//Set the VDD IO5 voltage range: 3.3V
	// while (LL_PWR_IsActiveFlag_VDDIO5RDY() == 0U);
	// LL_PWR_EnableVddUSB();	//Enable VDD USB (USB2 HS PHYs and USB Type-C) voltage supply
	// while (LL_PWR_IsActiveFlag_USB33RDY() == 0U);
	// LL_PWR_EnableVddADC();	//Enable VDD ADC voltage supply
	// while (LL_PWR_IsActiveFlag_ARDY() == 0U);

	LL_RCC_HSE_EnableBypass();
	LL_RCC_HSE_Enable();

	//Wait till HSE is ready
	while(LL_RCC_HSE_IsReady() != 1);

	//Wait HSE stabilization time before its selection as PLL source.
	// LL_mDelay(HSE_STARTUP_TIMEOUT);
	
	LL_RCC_HSI_Enable();

	//Wait till HSI is ready
	while(LL_RCC_HSI_IsReady() == 0);

	/** Get current CPU/System buses clocks configuration and
	*if necessary switch to intermediate HSI clock to ensure target clock can be set
	*/
	if ((LL_RCC_GetCpuClkSource() == LL_RCC_CPU_CLKSOURCE_STATUS_IC1) || (LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_IC2_IC6_IC11)) {
		LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
		while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI);
		LL_RCC_SetCpuClkSource(LL_RCC_CPU_CLKSOURCE_HSI);
		while(LL_RCC_GetCpuClkSource() != LL_RCC_CPU_CLKSOURCE_STATUS_HSI);
	}
	//Set PLL1 output frequency to 3200 MHz
	LL_RCC_PLL1_Disable();
	while(LL_RCC_PLL1_IsReady() == 1);
	LL_RCC_PLL1_DisableModulationSpreadSpectrum();
	LL_RCC_PLL1_DisableBypass();
	LL_RCC_PLL1_SetSource(LL_RCC_PLLSOURCE_HSE);
	LL_RCC_PLL1_SetM(4);
	LL_RCC_PLL1_SetN(640);
	LL_RCC_PLL1_SetP1(1);
	LL_RCC_PLL1_SetP2(1);
	LL_RCC_PLL1_SetFRACN(0);
	LL_RCC_PLL1_AssertModulationSpreadSpectrumReset();
	LL_RCC_PLL1_DisableFractionalModulationSpreadSpectrum();
	LL_RCC_PLL1P_Enable();
	LL_RCC_PLL1_Enable();
	while(LL_RCC_PLL1_IsReady() != 1);

	//Set PLL2 output frequency to 2000 MHz
	LL_RCC_PLL2_Disable();
	while(LL_RCC_PLL2_IsReady() == 1);
	LL_RCC_PLL2_DisableModulationSpreadSpectrum();
	LL_RCC_PLL2_DisableBypass();
	LL_RCC_PLL2_SetSource(LL_RCC_PLLSOURCE_HSE);
	LL_RCC_PLL2_SetM(4);
	LL_RCC_PLL2_SetN(400);
	LL_RCC_PLL2_SetP1(1);
	LL_RCC_PLL2_SetP2(1);
	LL_RCC_PLL2_SetFRACN(0);
	LL_RCC_PLL2_AssertModulationSpreadSpectrumReset();
	LL_RCC_PLL2_DisableFractionalModulationSpreadSpectrum();
	LL_RCC_PLL2P_Enable();
	LL_RCC_PLL2_Enable();
	while(LL_RCC_PLL2_IsReady() != 1);

	// Set AHB clock, source is SYSB which source is IC2
	LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
	// Set APB clock, source is AHB Clock
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_1);
	LL_RCC_SetAPB5Prescaler(LL_RCC_APB5_DIV_1);

	//Set IC1 output to PLL1 / 4 = 3200 MHz / 4 = 800 MHz
	LL_RCC_IC1_SetSource(LL_RCC_ICCLKSOURCE_PLL1);
	LL_RCC_IC1_SetDivider(4);
	LL_RCC_IC1_Enable();
	LL_RCC_SetCpuClkSource(LL_RCC_CPU_CLKSOURCE_IC1);
	while(LL_RCC_GetCpuClkSource() != LL_RCC_CPU_CLKSOURCE_STATUS_IC1);

	//Set IC2 output to PLL1 / 8 = 3200 MHz / 8 = 400 MHz
	LL_RCC_IC2_SetSource(LL_RCC_ICCLKSOURCE_PLL1);
	LL_RCC_IC2_SetDivider(8);
	
	//Set IC3 output to PLL2 / 8 = 3200 MHz / 8 = 400 MHz
	LL_RCC_IC3_SetSource(LL_RCC_ICCLKSOURCE_PLL1);
	LL_RCC_IC3_SetDivider(8);
	//Set IC4 output to PLL2 / 20 = 2000 MHz / 20 = 100 MHz
	LL_RCC_IC4_SetSource(LL_RCC_ICCLKSOURCE_PLL2);
	LL_RCC_IC4_SetDivider(20);
	//Set IC6 output to PLL1 / 4 = 3200 MHz / 4 = 800 MHz
	LL_RCC_IC6_SetSource(LL_RCC_ICCLKSOURCE_PLL1);
	LL_RCC_IC6_SetDivider(4);
	//Set IC9 output to PLL2 / 20 = 2000 MHz / 20 = 100 MHz
	LL_RCC_IC9_SetSource(LL_RCC_ICCLKSOURCE_PLL2);
	LL_RCC_IC9_SetDivider(20);
	//Set IC10 output to PLL1 / 32 = 3200 MHz / 32 = 100 MHz
	LL_RCC_IC10_SetSource(LL_RCC_ICCLKSOURCE_PLL1);
	LL_RCC_IC10_SetDivider(32);
	//Set IC11 output to PLL1 / 4 = 3200 MHz / 4 = 800 MHz
	LL_RCC_IC11_SetSource(LL_RCC_ICCLKSOURCE_PLL1);
	LL_RCC_IC11_SetDivider(4);

	LL_RCC_IC2_Enable();
	LL_RCC_IC3_Enable();
	LL_RCC_IC4_Enable();
	LL_RCC_IC6_Enable();
	LL_RCC_IC9_Enable();
	LL_RCC_IC10_Enable();
	LL_RCC_IC11_Enable();

	// AXI Clock -> IC2 (AHB IC2/2), NPU Clock -> IC6, AXISRAM3/4/5/6 -> IC11
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_IC2_IC6_IC11);
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_IC2_IC6_IC11);
	LL_SetSystemCoreClock(800000000);

	//Set peripheral clock sources
	//I2C peripheral
	LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_IC10);
	LL_RCC_SetI2CClockSource(LL_RCC_I2C2_CLKSOURCE_IC10);
	LL_RCC_SetI2CClockSource(LL_RCC_I2C4_CLKSOURCE_IC10);
	//I3C peripheral
	LL_RCC_SetI3CClockSource(LL_RCC_I3C1_CLKSOURCE_IC10);
	LL_RCC_SetI3CClockSource(LL_RCC_I3C2_CLKSOURCE_IC10);
	//SPI peripheral
	LL_RCC_SetSPIClockSource(LL_RCC_SPI1_CLKSOURCE_IC9);
	LL_RCC_SetSPIClockSource(LL_RCC_SPI2_CLKSOURCE_IC9);
	LL_RCC_SetSPIClockSource(LL_RCC_SPI4_CLKSOURCE_IC9);
	//UART peripheral
	LL_RCC_SetUARTClockSource(LL_RCC_UART4_CLKSOURCE_IC9);
	//SDMMC peripheral
	LL_RCC_SetSDMMCClockSource(LL_RCC_SDMMC1_CLKSOURCE_IC4);
	LL_RCC_SetSDMMCClockSource(LL_RCC_SDMMC2_CLKSOURCE_IC4);
	//XPSI/HyperBus peripheral
	LL_RCC_SetXSPIClockSource(LL_RCC_XSPI1_CLKSOURCE_IC3);
	LL_RCC_SetXSPIClockSource(LL_RCC_XSPI2_CLKSOURCE_IC3);
}

void System::InitSysTick(void) {
	LL_Init1msTick(SystemCoreClock);
	LL_SYSTICK_EnableIT();
}

void System::EnableDebug(void) {
	LL_APB4_GRP2_EnableClock(LL_APB4_GRP2_PERIPH_BSEC);
	BSEC->AP_UNLOCK = 0xB4;
	BSEC->DBGCR = 0xB4B4B400;
}

void System::EnableCache(void) {
	SCB_EnableICache();
	SCB_EnableDCache();
}

void System::DisableCache(void) {
	SCB_DisableICache();
	SCB_DisableDCache();
}

void System::CleanCache(void* addr, uint32_t size) {
	SCB_CleanDCache_by_Addr(addr, (int32_t)size);
}

void System::InvalidateCache(void* addr, uint32_t size) {
	SCB_InvalidateDCache_by_Addr(addr, (int32_t)size);
}

void System::Reset() {

}

// ============================================================================
// Time implementations (using TIM7)
// ============================================================================

static volatile uint32_t timeMills = 0;
static volatile uint32_t timeHours = 0;

void Time::Init() {
	// Enable bus clocks 
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

	// Configure Timer
	uint32_t periphClock = 400000000;					// TIM5 is connected to "Timer Group Clocks" i.e. to SYSB through TIMPRE
	uint32_t prescaler = (periphClock / 1000000) - 1;	// Set Clock to 1MHz -> 1us period
	// uint32_t arr = (1000000 / 1000) - 1;				// Set update period to 1kHz -> 1ms period
	uint32_t arr = (3600000000UL - 1);					// Set update period to 1h
	LL_TIM_SetPrescaler(TIM5, prescaler);
	LL_TIM_SetCounterMode(TIM5, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetAutoReload(TIM5, 3600000000);
	LL_TIM_EnableARRPreload(TIM5);

	// This is needed to update the prescaler
	LL_TIM_GenerateEvent_UPDATE(TIM5);

	// Reset timer counters
	timeMills = 0;
	timeHours = 0;

	// Configure Interrupts
	NVIC_SetPriority(TIM5_IRQn, 0);
	NVIC_EnableIRQ(TIM5_IRQn);
	LL_TIM_EnableIT_UPDATE(TIM5);

	// Enable Timer
	LL_TIM_EnableCounter(TIM5);
}

uint32_t Time::GetMs() {
	return (uint32_t)(GetUs() / 1000);
	// return timeMills;
}

uint64_t Time::GetUs() {
	uint32_t cnt = LL_TIM_GetCounter(TIM5);
	return ((uint64_t)timeHours * 3600000000ULL) + cnt;
	// return ((timeMills * 1000) + LL_TIM_GetCounter(TIM5));
}

void Time::Delay(uint32_t ms) {
	uint32_t start = GetUs();
	while ((GetUs() - start) < ((uint64_t)ms * 1000));
	// uint32_t start = timeMills;
	// while ((timeMills - start) < ms);
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

//Handled by ThreadX (??)
/**
  * @brief This function handles System tick timer.
  */
// extern "C" {
// 	void SysTick_Handler(void) {
// 		systickCnt += 1;
// 	}
// }

extern "C" {
	void TIM5_IRQHandler(void) {
		// timeMills += 1;
		timeHours += 1;
		LL_TIM_ClearFlag_UPDATE(TIM5);
	}
}