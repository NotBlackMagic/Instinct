#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_exti.h"
#include "stm32n6xx_ll_gpio.h"

#include "status.hpp"
#include "system.hpp"
#include "hyperbus.hpp"
#include "hyperFlash.hpp"
#include "hyperRAM.hpp"

#include "../../Board/PlumaN6.hpp"

#define FW_HEADER_OFFSET						0x400
#define FW_DESTINATION_ADDRESS					0x34000000
#define FW_SOURCE_ADDRESS_OFFSET				0x00100000
#define FW_IMAGE_OFFSET							0x00100000
#define FW_SIZE									0x00010000

// Default for Rev. A is: Cypress S80KS2564
HyperRAM::Config extRAMConfig = {	
	.deviceName = "Cypress S80KS2564",
	.expectedID = 0x06,
	.sizeBytes = 32768 * 1024,	// 32 MByte
	.pageSize = 32 * 8 * 4,		// 1 kByte
	.sourceClockHz = 0,
	.frequencyHz = 200000000,	// 100 MHz
	.initialLatency = 7,		// 7 Cycles
	.fixedLatency = true,
	.rwRecoveryTime = 7,		// 7 Cycles @ 200MHz
	.refreshRateUs = 4,			// 4us or 4000ns
	.writeZeroLatency = false,
	.configReg0 = 0xBF2F,		// CFG0: Set drive strength to 46 Ohm
	.configReg1 = 0				// CFG1: Do not change
};
HyperBus hyperBus1 = HyperBus(XSPI1);
extern "C" void XSPI1_IRQHandler(void) { hyperBus1.InterruptHandler(); }
HyperRAM externalPSRAM = HyperRAM(hyperBus1);

// Default for Rev. A is: Cypress S26HS512T with Uniform Sector Flash Configuration
HyperFlash::Config extFlashConfig = {	
	.deviceName = "Cypress S26HS512T",
	.expectedID = 0x0034,
	.expectedDeviceID = 0x0090,
	.sizeBytes = 256 * 256 * 1024,	// 64 MByte
	.sectorSize = 256 * 1024,		// 256 kByte
	.pageSize = 256,				// 512 bytes or 256 bytes (default: 256)
	.sourceClockHz = 0,
	.frequencyHz = 200000000,		// 100 MHz
	.initialLatency = 16,			// 16 Cycles
	.fixedLatency = false,
	.rwRecoveryTime = 0,
	.configReg0 = 0x8EBF,			// CFG0: Set drive strength to 46 Ohm
	.configReg1 = 0					// CFG1: Do not change
};
HyperBus hyperBus2 = HyperBus(XSPI2);
extern "C" void XSPI2_IRQHandler(void) { hyperBus2.InterruptHandler(); }
HyperFlash externalFlash = HyperFlash(hyperBus2);

Status CopyApplication(void) {
	Status status = Status::Ok;
	uint8_t *srcAddr;
	uint8_t *dstAddr;
	uint32_t srcBaseAddr;
	uint32_t fwSize;

	// Set destination of Firmware code address
	dstAddr = (uint8_t *)FW_DESTINATION_ADDRESS;

	// Get the address of the source memory
	srcBaseAddr = externalFlash.GetBaseAddr();

	// For memory mapped storage devices:  Manage the copy in mapped mode
	srcAddr = (uint8_t *)(srcBaseAddr + FW_SOURCE_ADDRESS_OFFSET);
	fwSize = FW_SIZE;
	// Copy from source to destination in mapped mode
	for(uint32_t i = 0; i < fwSize; i++) {
		dstAddr[i] = srcAddr[i];
	}

	return status;
}

__attribute__((weak)) uint32_t GetApplicationVectorTable(void) {
	uint32_t vectorTable;
	vectorTable = FW_DESTINATION_ADDRESS + FW_HEADER_OFFSET;
	return vectorTable;
}

Status JumpToApplication(void) {
	uint32_t primaskBit;
	typedef  void (*pFunction)(void);
	static pFunction JumpToApp;
	uint32_t appVector;

	//Suspend SysTick
	System::DisableSysTick();
	Time::Disable();

	// If I-Cache is enabled, disable I-Cache
	if(SCB->CCR & SCB_CCR_IC_Msk) {
		SCB_DisableICache();
	}

	// If D-Cache is enabled, disable I-Cache
	if(SCB->CCR & SCB_CCR_DC_Msk) {
		SCB_DisableDCache();
	}

	// Initialize application's Stack Pointer & Jump to user application
	primaskBit = __get_PRIMASK();
	__disable_irq();

	// appVector = GetApplicationVectorTable();
	appVector = externalFlash.GetBaseAddr();
	appVector += FW_IMAGE_OFFSET + FW_HEADER_OFFSET;

	SCB->VTOR = (uint32_t)appVector;
	JumpToApp = (pFunction)(*(__IO uint32_t *)(appVector + 4u));

	// On ARM v8m, set MSPLIM before setting MSP to avoid unwanted stack overflow faults
	__set_MSPLIM(0x00000000);

	__set_MSP(*(__IO uint32_t *)appVector);

	// Re-enable the interrupts
	__set_PRIMASK(primaskBit);

	LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);

	JumpToApp();
	return Status::Ok;
}

int main(void) {
	// Enable debugger in flash run mode
	System::EnableDebug();
	
	// Initialize GPIOs
	BoardGPIOInit();
	// RED LED
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_13, LL_GPIO_PULL_NO);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
	// GREEN LED
	LL_GPIO_SetPinSpeed(GPIOQ, LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinOutputType(GPIOQ, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(GPIOQ, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);
	LL_GPIO_SetPinMode(GPIOQ, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13);
	LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);

	// MCU Configuration
	NVIC_SetPriorityGrouping(0x03);	// 4 bits for pre-emption priority, 0 bit for subpriority
	// HAL_Init();
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);
	System::EnableCache();

	SystemCoreClockUpdate();
	System::InitSysTick();

	System::InitClock();
	System::InitSysTick();
	System::EnableAXISRAM();
	System::EnableVENCSRAM();
	Time::Init();

	LL_GPIO_ResetOutputPin(GPIOQ, LL_GPIO_PIN_1);
	Time::Delay(100);
	LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);
	Time::Delay(100);
	LL_GPIO_ResetOutputPin(GPIOQ, LL_GPIO_PIN_1);
	Time::Delay(100);
	LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);

	// Initialize HyperBus, HyperFlash, and HyperRAM.
	BoardXSPI1Init();
	extRAMConfig.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC3);	// From IC3
	if(externalPSRAM.Init(extRAMConfig) != Status::Ok) {
		LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13);
		return 0;
	}

	BoardXSPI2Init();
	extFlashConfig.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC3);	// From IC3
	if(externalFlash.Init(extFlashConfig) != Status::Ok) {
		LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13);
		return 0;
	}

	LL_GPIO_ResetOutputPin(GPIOQ, LL_GPIO_PIN_1);
	Time::Delay(100);
	LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);
	Time::Delay(100);
	LL_GPIO_ResetOutputPin(GPIOQ, LL_GPIO_PIN_1);
	Time::Delay(100);
	LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);
	Time::Delay(100);
	LL_GPIO_ResetOutputPin(GPIOQ, LL_GPIO_PIN_1);
	Time::Delay(100);
	LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);
	Time::Delay(100);
	LL_GPIO_ResetOutputPin(GPIOQ, LL_GPIO_PIN_1);

	// Set external RAM (PSRAM) to memory mapped mode
	if(externalPSRAM.EnterMemoryMappedMode() != Status::Ok) {
		LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13);
		return 0;
	}

	// BOOT_Application();
	Status status = Status::Ok;
	// Mount the memory
	if(externalFlash.EnterMemoryMappedMode() == Status::Ok) {
		// If XIP, skip CopyApplication()
		// retr = CopyApplication();
		if(status == Status::Ok) {
			// Jump on the application
			status = JumpToApplication();
		}
		else {
			LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13);
		}
	}
	else {
		LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13);
	}

	// We should never get here as execution is now from user application
	while(1) {
		__NOP();
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
extern "C" {
	void Error_Handler(void) {
		__disable_irq();
		while (1) {
		}
	}
}

/**
  * @brief This function handles System tick timer.
  */
extern "C" {
	void SysTick_Handler(void) {
		__NOP();
	}
}
