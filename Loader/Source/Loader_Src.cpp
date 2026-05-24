/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Loader/Loader_Src.cpp
 */

#include "Loader_Src.h"
#include "system.hpp"

#pragma section=".bss"

#if defined(__ICCARM__)
#pragma section = ".bss"
#elif defined(__GNUC__)
extern int __bss_start__, __bss_end__;
extern int __estack_end__;
#endif /* __ICCARM__ */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
/**
 * @brief not use and only present to avoid compilation issue
 */
 uint32_t __Vectors = 0x0;
#endif /* __CC_ARM || __ARMCC_VERSION */

// !!! Global C++ Constructors are never called due to no startup code before main()!!!
// Classes as pointers
HyperBus* pBus = nullptr;
HyperFlash* pFlash = nullptr;
// Prepare/allocate for the class objects
uint8_t busBuffer[sizeof(HyperBus)];
uint8_t flashBuffer[sizeof(HyperFlash)];

HyperFlash::Config extFlashConfig = {	
	.deviceName = "Cypress S26HS512T",
	.expectedID = 0x0034,
	.expectedDeviceID = 0x0090,
	.sizeBytes = 256 * 256 * 1024,	// 64 MByte
	.sectorSize = 256 * 1024,		// 256 kByte
	.pageSize = 256,				// 512 bytes or 256 bytes (default: 256)
	.sourceClockHz = 0,
	.frequencyHz = 100000000,		// 100 MHz
	.initialLatency = 16,			// 16 Cycles
	.fixedLatency = false,
	.rwRecoveryTime = 0,
	.configReg0 = 0x8EBF,			// CFG0: Set drive strength to 46 Ohm
	.configReg1 = 0					// CFG1: Do not change
};

uint8_t memoryMappedMode;

/**
 * @brief entry point definition for debug
 */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
/*nothing to be done*/
#elif defined(__GNUC__)
uint32_t g_pfnVectors[] __attribute__((section(".isr_vector"))) = { 0
};
void __attribute__((used,optimize("Os"))) Reset_Handler(void)
{
	asm(
		"ldr r0, =_estack\n"
		"nop\n"
		"mov     sp, r0\n");

	asm(
		"ldr r0, =main\n"
		"nop\n"
		"mov     pc, r0\n");
}
#elif defined(__ICCARM__)
extern const uint32_t __ICFEDIT_region_RAM_end__;

void Reset_Handler(void);

uint32_t __vector_table[] __attribute__((section(".vectors"))) = {
	(uint32_t)(&__ICFEDIT_region_RAM_end__), /* Stack pointer */
	(uint32_t)&Reset_Handler                 /* Reset handler */
};

void Reset_Handler(void) {
	main();
}
#endif

/**
 * @brief  main function used for debug purpose
  * @retval the function always returns 0
 */
int main(void) {
	return 0;
}

uint32_t HAL_GetTick(void) {
	return 1;
}

void HAL_Delay(uint32_t Delay) {
	int i=0;
	for (i=0; i<0x1000; i++);
}

/**
  * @brief  System initialization.
  * @param  None
  * @retval  1      : Operation succeeded
  * @retval  0      : Operation failed
  */
KeepInCompilation uint32_t Init() {
	uint32_t retr = 1;
	uint8_t *startadd;
	uint32_t size;

	// get ZI Init variables to zero
#if defined(__ICCARM__)
	startadd = __section_begin(".bss");
	size = __section_size(".bss");
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
	extern uint32_t Image$$PrgData$$ZI$$Base;
	extern uint32_t Image$$PrgData$$ZI$$Limit;

	startadd = (uint8_t *)&Image$$PrgData$$ZI$$Base;
	size = (uint32_t)&Image$$PrgData$$ZI$$Limit - (uint32_t)startadd;
#elif defined ( __GNUC__ )
	startadd = (uint8_t*)& __bss_start__;
	size = (uint8_t*)& __bss_end__ - (uint8_t*)& __bss_start__;
#else
	#error "the compiler is not supported"
#endif /* __ICCARM__ */

	// Init variables to zero
	memset(startadd, 0, size * sizeof(uint8_t));

	// Init system
	SystemInit();

	// disable all the IRQ
	__disable_irq();

	// Enable I-Cache
	// SCB_EnableICache();

	// Enable D-Cache
	// SCB_EnableDCache();

	// Configure the system clock
	System::InitClock();
	Time::Init();

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

	// Initialize HyperBus and HyperFlash
	pBus = new (busBuffer) HyperBus(XSPI2);
	pFlash = new (flashBuffer) HyperFlash(*pBus);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13);
	LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);

	BoardXSPI2Init();

	extFlashConfig.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC3);	// From IC3
	if(pFlash->Init(extFlashConfig) != Status::Ok) {
		LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13);
		return 0;
	}

	// Enable default memory mapped mode
	pFlash->EnterMemoryMappedMode();
	memoryMappedMode = 1;

	LL_GPIO_ResetOutputPin(GPIOQ, LL_GPIO_PIN_1);

	// Enable Interrupts
	// __enable_irq();

	return retr;
}

/**
  * @brief   Program memory.
  * @param   Address: page address
  * @param   Size   : size of data
  * @param   buffer : pointer to data buffer
  * @retval  1      : Operation succeeded
  * @retval  0      : Operation failed
  */
KeepInCompilation uint32_t Write (uint32_t Address, uint32_t Size, uint8_t* buffer) {
	LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);

	// Convert absolute memory address to relative flash offset
	Address &= 0x0FFFFFFF;

	// Disable memory mapped mode if enabled
	if(memoryMappedMode == 1) {
		pFlash->ExitMemoryMappedMode();
		memoryMappedMode = 0;
	}

	// Writes an amount of data to the HyperFlash memory
	if(pFlash->Program(Address, buffer, Size) != Status::Ok) {
		return 0;
	}
	else {
		return 1;
	}	
}

/**
  * @brief 	 Full erase of the device 						
  * @param 	 Parallelism : 0 																		
  * @retval  1           : Operation succeeded
  * @retval  0           : Operation failed											
  */
KeepInCompilation  uint32_t MassErase (uint32_t Parallelism) {
	LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);

	// Disable memory mapped mode if enabled
	if(memoryMappedMode == 1) {
		pFlash->ExitMemoryMappedMode();
		memoryMappedMode = 0;
	}

	// Erases the entire HyperFlash memory
	if(pFlash->ChipErase() != Status::Ok) {
		return 0;
	}
	else {
		return 1;
	}	
}

/**
  * @brief   Sector erase.
  * @param   EraseStartAddress :  erase start address
  * @param   EraseEndAddress   :  erase end address
  * @retval  None
  */
KeepInCompilation uint32_t SectorErase (uint32_t EraseStartAddress ,uint32_t EraseEndAddress) {
	LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);

	// Disable memory mapped mode if enabled
	if(memoryMappedMode == 1) {
		pFlash->ExitMemoryMappedMode();
		memoryMappedMode = 0;
	}

	uint32_t BlockAddr;
	EraseStartAddress = EraseStartAddress & 0x0FFFFFFF;
	EraseEndAddress &= 0x0FFFFFFF;
	EraseStartAddress = EraseStartAddress - EraseStartAddress % 0x00040000;

	while (EraseEndAddress>=EraseStartAddress) {
		BlockAddr = EraseStartAddress;

		// Erases the specified block of the HyperFlash memory
		pFlash->SectorErase(BlockAddr);

		EraseStartAddress+=0x00040000;
	}

	return 1;
}

/**
  * Description :
  * Calculates checksum value of the memory zone
  * Inputs    :
  *      StartAddress  : Flash start address
  *      Size          : Size (in WORD)  
  *      InitVal       : Initial CRC value
  * outputs   :
  *     R0             : Checksum value
  * Note: Optional for all types of device
  */
uint32_t CheckSum(uint32_t StartAddress, uint32_t Size, uint32_t InitVal) {
	uint8_t missalignementAddress = StartAddress%4;
	uint8_t missalignementSize = Size ;
	uint32_t cnt;
	uint32_t Val;
			
	StartAddress-=StartAddress%4;
	Size += (Size%4==0)?0:4-(Size%4);

	for(cnt=0; cnt<Size ; cnt+=4) {
		Val = *(uint32_t*)StartAddress;
		if(missalignementAddress) {
			switch (missalignementAddress) {
				case 1:
					InitVal += (uint8_t) (Val>>8 & 0xff);
					InitVal += (uint8_t) (Val>>16 & 0xff);
					InitVal += (uint8_t) (Val>>24 & 0xff);
					missalignementAddress-=1;
					break;
				case 2:
					InitVal += (uint8_t) (Val>>16 & 0xff);
					InitVal += (uint8_t) (Val>>24 & 0xff);
					missalignementAddress-=2;
					break;
				case 3:
					InitVal += (uint8_t) (Val>>24 & 0xff);
					missalignementAddress-=3;
					break;
			}
		}
		else if((Size-missalignementSize)%4 && (Size-cnt) <=4) {
			switch (Size-missalignementSize) {
				case 1:
					InitVal += (uint8_t) Val;
					InitVal += (uint8_t) (Val>>8 & 0xff);
					InitVal += (uint8_t) (Val>>16 & 0xff);
					missalignementSize-=1;
					break;
				case 2:
					InitVal += (uint8_t) Val;
					InitVal += (uint8_t) (Val>>8 & 0xff);
					missalignementSize-=2;
					break;
				case 3:
					InitVal += (uint8_t) Val;
					missalignementSize-=3;
					break;
			} 
		}
		else{ 
			InitVal += (uint8_t) Val;
			InitVal += (uint8_t) (Val>>8 & 0xff);
			InitVal += (uint8_t) (Val>>16 & 0xff);
			InitVal += (uint8_t) (Val>>24 & 0xff);
		}
		StartAddress+=4;
	}

	return (InitVal);
}

/**
  * Description :
  * Verify flash memory with RAM buffer and calculates checksum value of
  * the programmed memory
  * Inputs    :
  *      FlashAddr     : Flash address
  *      RAMBufferAddr : RAM buffer address
  *      Size          : Size (in WORD)  
  *      InitVal       : Initial CRC value
  * outputs   :
  *     R0             : Operation failed (address of failure)
  *     R1             : Checksum value
  * Note: Optional for all types of device
  */

KeepInCompilation uint64_t Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size, uint32_t missalignement) {
	uint32_t VerifiedData = 0, InitVal = 0;
	uint64_t checksum;
	Size*=4;

	LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);

	// Enable memory mapped mode if disabled
	if(memoryMappedMode == 0) {
		pFlash->EnterMemoryMappedMode();
		memoryMappedMode = 1;
	}

	checksum = CheckSum((uint32_t)MemoryAddr + (missalignement & 0xf), Size - ((missalignement >> 16) & 0xF), InitVal);
	while(Size>VerifiedData) {
		if(*(uint8_t*)MemoryAddr++ != *((uint8_t*)RAMBufferAddr + VerifiedData)) {
			return ((checksum<<32) + (MemoryAddr + VerifiedData));
		}
		VerifiedData++;
	}
			
	return (checksum<<32);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
	LL_GPIO_SetOutputPin(GPIOQ, LL_GPIO_PIN_1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13);
	while(1) {
	}
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void) {
	Error_Handler();
}