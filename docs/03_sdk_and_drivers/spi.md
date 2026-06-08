# Serial Peripheral Interface (SPI) Driver
The SPI driver provides a simple object-oriented C++ driver for the SPI peripheral of the STM32N6. It is built on top the STM32 Low-Layer (LL) API. It handles interrupt-driven, asynchronous transfers with native ThreadX RTOS integration for thread-safe bus sharing and thread blocking during transactions. The driver is hardcoded for 8-bit data widths and uses the hardware FIFOs to reduce interrupt calls.

## Architectural Overview
The driver separates peripheral logic from hardware pin mapping. GPIO configurations must be handled outside the driver prior to initialization. In the Instinct firmware, this is managed in the board specific package (e.g., in "Board/PlumaN6.cpp" via ```BoardSPIxInit()```). The driver is configured to use hardware-controlled Chip Select (```LL_SPI_NSS_HARD_OUTPUT```), so that specific pin must be mapped and configured as well. If a standard GPIO is used as a custom CS, it must be controlled manually outside the driver: asserted before "TransferAsync" and released after "TransferWait".

The driver relies on RTOS objects to prevent bus collisions (```tx_mutex```) and CPU polling (```tx_event_flags```). Transactions are split into a non-blocking trigger/start (```TransferAsync```) and a blocking wait (```TransferWait```). This allows the simultaneous start/use of different, independent bus peripherals. Multiple can be started (e.g. ```SPI2``` and ```SPI3```) and then await each. The blocking wait is not polling, using event flags, and therefore yields the CPU time until transfer complete.

Baud rates are calculated dynamically at initialization using the passed clock source frequency. The driver computes the closest available hardware prescaler (power-of-two divider) that gives a frequency equal to or lower than the requested target. Because transfers are always in full-duplex mode, the driver handles dummy-byte generation. Passing nullptr as the transmit buffer automatically sends 0xFF, while passing nullptr as the receive buffer discards all incoming data. 

## Example Code

### Basic Transaction

```c
#include "spi.hpp"

// Instantiate the SPI object (e.g. SPI2)
SPI spi2(SPI2);

// Hardware IRQ Handler (Must exist in the global scope)
extern "C" void SPI2_IRQHandler(void) { spi2.InterruptHandler(); }

// Define the configuration
SPI::Config spiConfig;
spiConfig.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC9);
spiConfig.baudrate = 1000000;
spiConfig.polarity = SPI::ClockPolarity::High;
spiConfig.phase = SPI::ClockPhase::SecondEdge;
spiConfig.bitOrder = SPI::BitOrder::MSBFirst;

// Apply the configuration
spi2.Init(spiConfig);
// Or
spi2.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC9), .baudrate = 1000000, .polarity = SPI::ClockPolarity::High, .phase = SPI::ClockPhase::SecondEdge, .bitOrder = SPI::BitOrder::MSBFirst});

uint8_t txBuffer[32];
uint8_t rxBuffer[32];

// Write to a register: register address in byte 0, value in byte 1
txBuffer[0] = 0x54;
txBuffer[1] = 0xAA;
spi2.TransferAsync(txBuffer, rxBuffer, 2);
spi2.TransferWait(TX_WAIT_FOREVER);

// Read from a register: send register address, then receive data
txBuffer[0] = 0x80;
spi2.TransferAsync(txBuffer, nullptr, 1);
spi2.TransferWait(TX_WAIT_FOREVER);

spi2.TransferAsync(nullptr, rxBuffer, 1);
spi2.TransferWait(TX_WAIT_FOREVER);
```