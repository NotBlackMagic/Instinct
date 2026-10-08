#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_exti.h"
#include "stm32n6xx_ll_gpio.h"

#include "gpio.hpp"
#include "logger.hpp"
#include "hyperbus.hpp"
#include "hyperFlash.hpp"
#include "hyperRAM.hpp"
#include "status.hpp"
#include "system.hpp"
#include "uart.hpp"

#include "uartLoader.hpp"

#include "../../Board/PlumaN6.hpp"

/*
How to flash using USB DFU:
C:\ST\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe -c port=usb1 -d "Boot_Signed.bin" 0x01 -v
C:\ST\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe -c port=usb1 -detach

& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=usb1 -d "Boot_Signed.bin" 0x01 -v
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=usb1 -detach
*/

// Flash Layout Offsets
#define FLASH_OFFSET_FSBL						0x00000000U		// FSBL flash area
#define FLASH_OFFSET_BOOT_META					0x00040000U		// Boot metadata flash area
#define FLASH_OFFSET_CONFIG						0x00080000U		// General configuration flash area
#define FLASH_OFFSET_APP_SLOT_A					0x00100000U		// Application/firmware Slot A (main firmware area) flash area
#define FLASH_OFFSET_APP_SLOT_B					0x00400000U		// Application/firmware Slot B (backup firmware area) flash area
#define FLASH_OFFSET_NPU_MODELS					0x00700000U		// NPU model area

#define APP_MAX_SIZE							(3U * 1024U * 1024U)	// 3 MB
#define FSBL_FLASH_SIZE							(256U * 1024U)			// 256 KB
#define FSBL_DOWNLOAD_RAM_ADDRESS				0x34180000U

// RAM Execution/Staging Bounds
#define AXI_SRAM_BASE							0x24000000U
#define AXI_SRAM_SIZE							(512U * 1024U)
#define PSRAM_BASE								0x34000000U

#define FW_HEADER_OFFSET						0x400
#define FW_STAGING_RAM_ADDRESS					0x34000000U

UART debugUART(UART4);
extern "C" void UART4_IRQHandler(void) { debugUART.InterruptHandler(); }

GPIO ledRed(GPIOB, LL_GPIO_PIN_13);
GPIO ledBlue(GPIOB, LL_GPIO_PIN_2);
GPIO ledGreen(GPIOQ, LL_GPIO_PIN_1);
GPIO userButton(GPIOO, LL_GPIO_PIN_5);

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

Status FlashApplication(uint32_t flashOffset, const uint8_t *ramSource, uint32_t sizeBytes) {
	if(sizeBytes == 0 || sizeBytes > APP_MAX_SIZE) {
		return Status::Error;
	}

	// Ensure LOGs are sent before disabling ALL interrupts
	debugUART.Flush();

	const uint32_t primask = __get_PRIMASK();
	__disable_irq();

	const uint32_t sectorSize = extFlashConfig.sectorSize;
	const uint32_t flashBase = externalFlash.GetBaseAddr();

	// Exit memory-mapped mode for indirect register/command access
	if(externalFlash.ExitMemoryMappedMode() != Status::Ok) {
		__set_PRIMASK(primask);
		return Status::Error;
	}

	// Erase Sectors covering the payload
	uint32_t eraseEnd = flashOffset + sizeBytes - 1;
	uint32_t blockStart = flashOffset & ~(sectorSize - 1);
	uint32_t blockEnd = eraseEnd & ~(sectorSize - 1);

	for(uint32_t addr = blockStart; addr <= blockEnd; addr += sectorSize) {
		if(externalFlash.SectorErase(addr) != Status::Ok) {
			externalFlash.Reset();
			__set_PRIMASK(primask);
			return Status::Error;
		}
	}

	// Program payload page by page (256 bytes)
	for(uint32_t offset = 0; offset < sizeBytes; offset += extFlashConfig.pageSize) {
		uint32_t chunk = (sizeBytes - offset > extFlashConfig.pageSize) ? extFlashConfig.pageSize : (sizeBytes - offset);

		if(externalFlash.Program(flashOffset + offset, &ramSource[offset], chunk) != Status::Ok) {
			externalFlash.Reset();
			__set_PRIMASK(primask);
			return Status::Error;
		}
	}

	// Return to Memory-Mapped mode for XIP & Verification
	if(externalFlash.EnterMemoryMappedMode() != Status::Ok) {
		__set_PRIMASK(primask);
		return Status::Error;
	}

	// Invalidate D-Cache to prevent stale cache lines from preceding boot checks
	if(SCB->CCR & SCB_CCR_DC_Msk) {
		SCB_CleanInvalidateDCache();
	}

	// Byte-for-byte verification
	volatile uint8_t *mappedFlash = (volatile uint8_t *)(flashBase + flashOffset);
	for(uint32_t i = 0; i < sizeBytes; i++) {
		if(mappedFlash[i] != ramSource[i]) {
			__set_PRIMASK(primask);
			return Status::Error;
		}
	}

	__set_PRIMASK(primask);
	return Status::Ok;
}

bool ValidateApplication(uint32_t vectorTableAddress, uint32_t executionBase, uint32_t executionLimit) {
	uint32_t stackPointer = *(__IO uint32_t *)vectorTableAddress;
	uint32_t resetHandler = *(__IO uint32_t *)(vectorTableAddress + 4u);

	// Guard against blank / erased flash
	if(stackPointer == 0xFFFFFFFF || stackPointer == 0x00000000) {
		return false;
	}

	// Validate Stack Pointer lies inside valid internal SRAM or PSRAM
	bool validAXIRAM = (stackPointer >= AXI_SRAM_BASE && stackPointer <= (AXI_SRAM_BASE + AXI_SRAM_SIZE));
	bool validPSRAM = (stackPointer >= PSRAM_BASE && stackPointer <= (PSRAM_BASE + extRAMConfig.sizeBytes));
	if(validAXIRAM == false && validPSRAM == false) {
		return false;
	}

	// Reset handler must point inside the target execution space and have Thumb bit set
	if(resetHandler < executionBase || resetHandler >= executionLimit || !(resetHandler & 0x1)) {
		return false;
	}

	return true;
}

Status JumpToApplication(uint32_t appVector, uint32_t executionBase, uint32_t executionLimit) {
	typedef void (*AppEntryFunction)(void);
	AppEntryFunction JumpToApp;

	// Validate app header before making any destructive changes
	if(ValidateApplication(appVector, executionBase, executionLimit) == false) {
		return Status::Error;
	}

	// Flush UART
	debugUART.Flush();

	//Suspend SysTick
	System::DisableSysTick();
	Time::Disable();

	// If I-Cache is enabled, disable I-Cache
	if(SCB->CCR & SCB_CCR_IC_Msk) {
		SCB_CleanInvalidateDCache();
		SCB_DisableDCache();
	}

	// If D-Cache is enabled, disable I-Cache
	if(SCB->CCR & SCB_CCR_DC_Msk) {
		SCB_DisableDCache();
	}
	SCB_InvalidateICache();

	// Mask interrupts and keep them disabled across jump, the application load and boot
	__disable_irq();

	// Clear all pending NVIC interrupts
	for(uint32_t i = 0; i < 8; i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

	// Set vector table offset and barriers
	SCB->VTOR = (uint32_t)appVector;
	__DSB();
	__ISB();

	JumpToApp = (AppEntryFunction)(*(__IO uint32_t *)(appVector + 4u));

	// On ARM v8m, set MSPLIM before setting MSP to avoid unwanted stack overflow faults
	__set_MSPLIM(0x00000000U);
	__set_MSP(*(__IO uint32_t *)appVector);
	__DSB();
	__ISB();

	// Interrupts must be re-enabled in the firmware/application so no ISRs are called during the boot
	// __set_PRIMASK(primaskBit);

	// Ready to jump, turn Green LED on
	ledGreen.Write(0);

	JumpToApp();
	
	return Status::Ok;
}

Status SelfFlashFSBL(const uint8_t *ramSource, uint32_t sizeBytes) {
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();

	const uint32_t sectorSize = extFlashConfig.sectorSize;		// 256 KB (0x40000)
	const uint32_t flashBase = externalFlash.GetBaseAddr();		// 0x70000000

	// Exit memory-mapped mode to issue indirect erase/program commands
	if(externalFlash.ExitMemoryMappedMode() != Status::Ok) {
		__set_PRIMASK(primask);
		return Status::Error;
	}

	// Erase required sectors starting from offset 0
	uint32_t eraseEnd = sizeBytes - 1;
	uint32_t blockStart = 0 & ~(sectorSize - 1);
	uint32_t blockEnd = eraseEnd & ~(sectorSize - 1);
	for(uint32_t addr = blockStart; addr <= blockEnd; addr += sectorSize) {
		if(externalFlash.SectorErase(addr) != Status::Ok) {
			__set_PRIMASK(primask);
			return Status::Error;
		}
	}

	// Program payload in page chunks (256 bytes)
	for(uint32_t offset = 0; offset < sizeBytes; offset += extFlashConfig.pageSize) {
		uint32_t chunk = (sizeBytes - offset > extFlashConfig.pageSize) ? extFlashConfig.pageSize : (sizeBytes - offset);
		if(externalFlash.Program(offset, &ramSource[offset], chunk) != Status::Ok) {
			__set_PRIMASK(primask);
			return Status::Error;
		}
	}

	// Re-enter memory-mapped mode and verify byte-for-byte
	if(externalFlash.EnterMemoryMappedMode() != Status::Ok) {
		__set_PRIMASK(primask);
		return Status::Error;
	}

	// Verify programmed bytes against HyperRAM source
	volatile uint8_t *mappedFlash = (volatile uint8_t *)flashBase;
	for(uint32_t i = 0; i < sizeBytes; i++) {
		if(mappedFlash[i] != ramSource[i]) {
			__set_PRIMASK(primask);
			return Status::Error;
		}
	}

	__set_PRIMASK(primask);
	return Status::Ok;
}

void EnterRecoveryMode(void) {
	System::EnableDebug();
	
	// Disable and invalidate L1 caches to prevent DAP incoherency
	if(SCB->CCR & SCB_CCR_DC_Msk) {
		SCB_CleanInvalidateDCache();
		SCB_DisableDCache();
	}
	
	if(SCB->CCR & SCB_CCR_IC_Msk) {
		SCB_DisableICache();
	}
	SCB_InvalidateICache();

	// Wait for button to be released before waiting for user action
	while(userButton.Read() == 0) {
		Time::Delay(10);
	}

	// Blink Blue LED while waiting for ST-Link upload and button trigger
	LOG_INFO("--------------------------------");
	LOG_INFO("Recovery Mode:");
	LOG_INFO("  (1) UART FW flashing");
	LOG_INFO("  (2) Or press [BTN0] to self-flash FSBL");

	uint32_t firmwareSize = 0;
	while(1) {
		ledBlue.Toggle();
		Time::Delay(100);

		// Check for incoming UART firmware upload
		if(debugUART.Available() > 0) {
			LOG_INFO("UART FW flashing: Started...");

			ledBlue.Write(0); // Blue solid ON
			Status status = LoaderFWToPSRAM(&debugUART, FW_STAGING_RAM_ADDRESS, firmwareSize);
			ledBlue.Write(1); // Blue OFF

			if(status == Status::Ok) {
				LOG_INFO("UART FW flashing: Writing to Flash Slot A...");
				debugUART.Flush(); // Ensure LOGs are sent before disabling ALL interrupts

				status = FlashApplication(FLASH_OFFSET_APP_SLOT_A, (const uint8_t *)FW_STAGING_RAM_ADDRESS, firmwareSize);

				ledGreen.Write(0); // Green ON: Download & CRC OK!

				if(status == Status::Ok) {
					LOG_INFO("UART FW flashing: FW flashed to Slot A (%lu bytes)", firmwareSize);

					uint32_t flashBase = externalFlash.GetBaseAddr();
					uint32_t appVector = flashBase + FLASH_OFFSET_APP_SLOT_A + FW_HEADER_OFFSET;
					uint32_t appLimit = flashBase + FLASH_OFFSET_APP_SLOT_A + APP_MAX_SIZE;

					LOG_INFO("UART FW flashing: Try booting application...");
					JumpToApplication(appVector, flashBase + FLASH_OFFSET_APP_SLOT_A, appLimit);

					// If JumpToApplication returns, the image failed validation
					LOG_ERR("UART FW flashing: Booting failed!");
					ledRed.Write(0); // Red ON
				}
				else {
					LOG_ERR("UART FW flashing: FW flashing failed!");
					ledRed.Write(0); // Red ON
				}
			}
			else {
				ledBlue.Write(1);
				ledRed.Write(0); // Red ON
			}
		}

		// Check for button press to trigger programming, with debounce
		bool btnPress = false;
		if(userButton.Read() == 1) {
			btnPress = false;
		}
		else {
			Time::Delay(50);
			if(userButton.Read() == 0) {
				btnPress = true;
			}
		}

		if(btnPress == true) {
			LOG_INFO("FSBL self-flashing: Started...");
			// Flashing in progress: Blue and Green LED solid ON
			ledBlue.Write(0);
			ledGreen.Write(0);
			Status status = SelfFlashFSBL((const uint8_t *)FSBL_DOWNLOAD_RAM_ADDRESS, FSBL_FLASH_SIZE);
			// Turn Blue and Green LED OFF
			ledGreen.Write(1);
			ledBlue.Write(1);

			if(status == Status::Ok) {
				LOG_INFO("FSBL self-flashing: FSBL flashed");
				ledGreen.Write(0);
			}
			else {
				LOG_ERR("FSBL self-flashing: FSBL flashing failed!");
				ledRed.Write(0);
			}
		}
	}
}

int main(void) {
	// Enable debugger in flash run mode
	System::EnableDebug();
	
	// Initialize GPIOs
	BoardGPIOInit();
	ledRed.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	ledBlue.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	ledGreen.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	userButton.Init({.mode = GPIO::Mode::Input, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::PullUp});
	// BOOT1 original
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);

	// All LEDs off
	ledRed.Write(1);
	ledBlue.Write(1);
	ledGreen.Write(1);

	// MCU Configuration
	NVIC_SetPriorityGrouping(0x03);	// 4 bits for pre-emption priority, 0 bit for subpriority
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);
	System::EnableCache();

	// Enable and setup clocks
	SystemCoreClockUpdate();
	System::InitSysTick();
	System::InitClock();
	System::InitSysTick();
	System::EnableAXISRAM();
	System::EnableVENCSRAM();
	Time::Init();

	// Enable console/shell/logger
	BoardUART4Init();
	debugUART.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC9), .baudrate = 115200, .dataBits = UART::DataBits::DataBits_8, .stopBits = UART::StopBits::StopBits_1, .parity = UART::Parity::None, .hwFlowControl = false});	
	Logger::Instance().Init();
	Logger::Instance().RegisterConsole(&debugUART);

	LOG_INFO("--------------------------------");
	LOG_INFO("FSBL Booting...");
	LOG_INFO("BOOTSR: 0x%08lX", (unsigned long)SYSCFG->BOOTSR);
	LOG_INFO("BOOT Pins: BOOT0=%d BOOT1=%d", (SYSCFG->BOOTSR & SYSCFG_BOOTSR_BOOT0), (SYSCFG->BOOTSR & SYSCFG_BOOTSR_BOOT1));
	LOG_INFO("Live Pins: PA6=%d PO5=%d", LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6), userButton.Read());
	LOG_INFO("Reset Status: 0x%08lX", RCC->HWRSR);
	LOG_INFO("BSEC DBGCR: 0x%08lX", (unsigned long)BSEC->DBGCR);
	LOG_INFO("BSEC HDPLSR: 0x%08lX", (unsigned long)BSEC->HDPLSR);
	LOG_INFO("PWR SVMCR3: 0x%08lX", (unsigned long)PWR->SVMCR3);

	// Read relevant OTP bits
	uint32_t tmp = 0;
	LOG_INFO("OTP Configuration:");
	System::ReadOTPHardware(11, tmp);
	LOG_INFO("    OTP11: 0x%08lX", tmp);
	System::ReadOTPHardware(13, tmp);
	LOG_INFO("    OTP13: 0x%08lX", tmp);
	System::ReadOTPHardware(14, tmp);
	LOG_INFO("    OTP14: 0x%08lX", tmp);
	System::ReadOTPHardware(15, tmp);
	LOG_INFO("    OTP15: 0x%08lX", tmp);
	System::ReadOTPHardware(16, tmp);
	LOG_INFO("    OTP16: 0x%08lX", tmp);
	System::ReadOTPHardware(18, tmp);
	LOG_INFO("    OTP18: 0x%08lX", tmp);
	System::ReadOTPHardware(19, tmp);
	LOG_INFO("    OTP19: 0x%08lX", tmp);
	System::ReadOTPHardware(124, tmp);
	LOG_INFO("    OTP124: 0x%08lX", tmp);

	ledBlue.Write(0);
	Time::Delay(100);
	ledBlue.Write(1);
	Time::Delay(100);
	ledBlue.Write(0);
	Time::Delay(100);
	ledBlue.Write(1);

	// Initialize HyperBus, HyperFlash, and HyperRAM.
	BoardXSPI1Init();
	extRAMConfig.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC3);
	if(externalPSRAM.Init(extRAMConfig) != Status::Ok) {
		LOG_ERR("HyperRAM Init Failed!");
		ledRed.Write(0);
		return 0;
	}
	LOG_INFO("HyperRAM Init & Test OK: %s", extRAMConfig.deviceName);

	BoardXSPI2Init();
	extFlashConfig.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC3);
	if(externalFlash.Init(extFlashConfig) != Status::Ok) {
		LOG_ERR("HyperFlash Init Failed!");
		ledRed.Write(0);
		return 0;
	}
	LOG_INFO("HyperFlash Init OK: %s.", extFlashConfig.deviceName);

	// Check Recovery Button early, allow user to use ST-LINK to halt the core during this window
	LOG_INFO("Press [BTN0] to enter recovery mode");
	bool forceRecovery = false;
	for(uint32_t i = 0; i < 25; i++) {
		Logger::Instance().Printf(".");
		if(userButton.Read() == 0) {
			forceRecovery = true;
			break;
		}
		ledBlue.Toggle();
		Time::Delay(200);
	}
	Logger::Instance().Printf("\r\n");

	// If user held button, stay in Recovery Mode with SWD open
	if(forceRecovery == true) {
		LOG_INFO("Entering recovery mode...");
		EnterRecoveryMode();
	}

	// Boot Primary Application from Flash Slot A (XIP mode)
	if(externalFlash.EnterMemoryMappedMode() == Status::Ok) {
		uint32_t flashBase = externalFlash.GetBaseAddr();
		uint32_t appVector = flashBase + FLASH_OFFSET_APP_SLOT_A + FW_HEADER_OFFSET;
		uint32_t appLimit = flashBase + FLASH_OFFSET_APP_SLOT_A + APP_MAX_SIZE;

		LOG_INFO("Try jumping to App Slot A @ 0x%08lX...", (unsigned long)appVector);
		Status status = JumpToApplication(appVector, flashBase + FLASH_OFFSET_APP_SLOT_A, appLimit);
		if(status != Status::Ok) {
			LOG_WARN("FAILED: Entering recovery mode...");
			EnterRecoveryMode();
		}
		else {
			ledRed.Write(0);
		}
	}
	else {
		ledRed.Write(0);
	}

	// Should never get here as execution is now from application/firmware
	while(1) {
		__NOP();
	}
}

/**
 * @brief	This function is executed in case of error occurrence.
 * @retval None
 */
extern "C" {
	void Error_Handler(void) {
		__disable_irq();
		while(1) {
			__NOP();
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
