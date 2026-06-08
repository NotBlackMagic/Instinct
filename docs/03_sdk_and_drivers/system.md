# System (RCC, System Timer, I/D-Cache) Driver
The System driver contains the fundamental system initializations, static and globally accessible. It manages power scaling (internal SMPS, independent VDDIO domains), the complex clock tree (PLLs and IC nodes), specific memory enablement (VENC and NPU SRAM), hardware caching, and a custom microsecond-resolution system timer. It is hardcoded so changes to clocks and/or power domains, and SMPS must be done by changing the driver code.

## Architectural Overview
The driver accounts for two different boot scenarios: debugging mode (code from SRAM) and XIP mode. For debugging mode, the function ```InitClock()``` fully configures the SMPS, VDDIO domains, HSE/HSI and PLLs/ICs to run the MCU core at 800 MHz and the NPU at 1000 MHz. In XIP mode, the function ```InitFWClock()``` assumes that hardware (RCC, power domains, SMPS, etc...) is initialized in the FSBL and it only populates the internal frequency tracking variables.

Because the system clocks are hardcoded, the driver provides ```GetNodeFrequency()``` to allow passing, in the peripherals config structure, the source clock to peripheral drivers (like I2C or UART).

The driver implements an independent timekeeping, using Timer 5, for high resolution microsecond timekeeping without using the SysTick which is typically used by an RTOS (e.g. FreeRTOS or ThreadX).

The driver also implements wrappers for cache coherence, ```CleanCache``` and ```InvalidateCache```, as well as for enabling debug mode in XIP and enabling specific SRAM domains (VENC and NPU).

## Example Code

### System Initialization

```c
#include "system.hpp"

int main(void) {
	// Enable debugger in flash run mode (XIP)
	// System::EnableDebug();

	// System initialization based on boot mode
	System::InitClock();		// Call when running independently from SRAM
	// System::InitFWClock();	// Call when running in XIP (hardware already initialized by FSBL)

	// Enable additional SRAM regions
	System::EnableAXISRAM();
	System::EnableVENCSRAM();

	// Enable I/D-cachefor maximum performance
	System::EnableCache();

	// Init custom system timer (SysTick alternative using TIM5)
	Time::Init();

	// Init SysTick (from RTOS)
	System::InitSysTick();

	while(1) {
		// Blocking (no RTOS) delay for 200ms
		Time::Delay(200);
	}
}
```

### Clock Querying
Active clock frequencies can be queried and passed on to peripheral drivers, typically in their config structure passed to the init functions, required to calculate prescalers and baud rates.

```c
#include "system.hpp"

// Example: Getting the APB1 clock frequency for a timer
uint32_t apb1Freq = System::GetNodeFrequency(System::ClockNode::APB1);

// Example: Getting the I2C clock source frequency
uint32_t i2cFreq = System::GetNodeFrequency(System::ClockNode::IC10);
```

### Cache Handling
When working with bare-metal DMA drivers (like SDMMC, USB, or DCMI), it is required to manually manage the D-Cache to ensure the CPU and DMA are looking at the same data in SRAM.

```c
#include "system.hpp"

// Example: Transmitting data via DMA
// Clean D-Cache before a DMA write
System::CleanCache(txBufferPtr, bufferLength);
// Trigger DMA transmission...

// Example: Receiving data via DMA
// Wait for DMA reception to complete ...
// Invalidate D-Cache after a DMA read
System::InvalidateCache(rxBufferPtr, bufferLength);
```

### Timekeeping

```c
#include "system.hpp"

// Measure the exact execution time of a specific block of code
uint64_t startTimestamp = Time::GetUs();

// Run code/algorithm to be profiled/measured

uint64_t endTimestamp = Time::GetUs();
uint64_t executionTimeUs = endTimestamp - startTimestamp;
```