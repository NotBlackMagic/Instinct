# Inter-Integrated Circuit (I2C) Driver
The I2C driver provides a simple object-oriented C++ driver for the I2C peripheral of the STM32N6. It is built on top the STM32 Low-Layer (LL) API. It handles interrupt-driven, asynchronous transfers with native ThreadX RTOS integration for thread-safe bus sharing and thread blocking during transactions.

## Architectural Overview
The driver expects a 100 MHz clock source for its internal bus timing configurations. If a different clock source frequency is used, the ```LL_I2C_SetTiming``` values must be recalculated and adjusted in the driver code (e.g., using STM32CubeMX). Additionally, the driver separates peripheral logic from hardware pin mapping. GPIO configurations must be handled outside the driver prior to initialization. In the Instinct firmware, this is managed in the board specific package (e.g., in "Board/PlumaN6.cpp" via ```BoardI2CxInit()```).

The driver relies on RTOS objects to prevent bus collisions (tx_mutex) and CPU polling (```tx_event_flags```). Transactions are split into a non-blocking trigger/start (```TransferAsync```) and a blocking wait (```TransferWait```). This allows the simultaneous start/use of different, independent bus peripherals. Multiple can be started (e.g. ```I2C1``` and ```SPI3```) and then await each. The blocking wait is not polling, using event flags, and therefore yields the CPU time until transfer complete.

Transactions follow a write then read sequence, useful for querying a device register, which is handled automatically. The driver issues a repeated start condition between the phases rather than releasing the bus with stop and restart. If a device does NOT support a repeated start, two independent transactions are required, first a transaction for the write with 0 length read and then a transaction for the read with a 0 length write.

Single transaction sequences are limited to a maximum of 255 bytes, a hardware limitation, for both the transmit and receive buffers.

## Example Code

### Basic Transactions

```c
#include "i2c.hpp"

// Instantiate the I2C object (e.g. I2C1)
I2C i2c1(I2C1);

// Hardware IRQ Handler (Must exist in the global scope)
extern "C" void I2C1_EV_IRQHandler(void) { i2c1.InterruptHandler(); }

// Define the configuration
I2C::Config i2cConfig;
i2cConfig.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC10);
i2cConfig.mode = I2C::Mode::Fast;

// Apply the configuration
i2c1.Init(i2cConfig);
// Or
i2c1.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC10), .mode = I2C::Mode::Fast});

// Probe for connected I2C device, on 7-bit address 0x63
uint8_t validAddress = i2c1.Probe(0x63);

uint8_t buffer[32];
if(validAddress == 0x01) {
	// Write to register, register address in byte 0, value in byte 1
	buffer[0] = 0x02;
	buffer[1] = 0xFF;
	i2c1.TransferAsync(0x63, buffer, 2, nullptr, 0);
	i2c1.TransferWait(TX_WAIT_FOREVER);

	// Read from register, register address in byte 0, return value in byte 0
	i2c1.TransferAsync(0x63, buffer, 1, buffer, 1);
	i2c1.TransferWait(TX_WAIT_FOREVER);
}
```

### Non Repeated Start Read

```c
#include "i2c.hpp"

// Instantiate the I2C object (e.g. I2C1)
I2C i2c1(I2C1);

// Hardware IRQ Handler (Must exist in the global scope)
extern "C" void I2C1_EV_IRQHandler(void) { i2c1.InterruptHandler(); }

// Apply the configuration
i2c1.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC10), .mode = I2C::Mode::Fast});

uint8_t buffer[32];
if(validAddress == 0x01) {
	// Write register address to be read
	buffer[0] = 0x02;
	i2c1.TransferAsync(0x63, buffer, 1, nullptr, 0);
	i2c1.TransferWait(TX_WAIT_FOREVER);

	// Read from register
	i2c1.TransferAsync(0x63, nullptr, 0, buffer, 1);
	i2c1.TransferWait(TX_WAIT_FOREVER);
}
```
