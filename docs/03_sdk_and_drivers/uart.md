# Universal Asynchronous Receiver-Transmitter (UART) Driver
The UART driver provides a simple object-oriented C++ driver for the USART and UART peripherals of the STM32N6. It is built on top of the STM32 Low-Layer (LL) API. It handles interrupt-driven, circular ring buffered asynchronous transmission and reception with native ThreadX RTOS integration for thread-safe access across multiple threads.

## Architectural Overview
The driver separates peripheral logic from hardware pin mapping. GPIO configurations must be handled outside the driver prior to initialization. In the Instinct firmware, this is managed in the board specific package (e.g., in "Board/PlumaN6.cpp" via ```BoardUARTxInit()```). Peripheral bus clocks (APB1 or APB2) and NVIC IRQ lines are enabled automatically based on the assigned UART instance.

Data transmission and reception are non-blocking and decoupled through internal circular ring buffers. Transmit copies outbound data into the TX ring buffer protected by a ThreadX mutex and critical section. Reception continuously stores incoming bytes into the RX ring buffer. Available unread bytes can be polled via Available and read via Receive.

## Example Code

### Basic Transmission and Reception

```c
#include "uart.hpp"

// Instantiate the UART object (e.g., UART4)
UART uart4(UART4);

// Hardware IRQ Handler (Must exist in the global scope)
extern "C" void UART4_IRQHandler(void) { uart4.InterruptHandler(); }

// Define the configuration (standard 8-N-1 and 115200 baudrate)
UART::Config uartConfig;
uartConfig.sourceClockHz = System::GetNodeFrequency(System::ClockNode::APB2);
uartConfig.baudrate = 115200;
uartConfig.dataBits = UART::DataBits::Bits8;
uartConfig.parity = UART::Parity::None;
uartConfig.stopBits = UART::StopBits::Bits1;
uartConfig.hwFlowControl = false;
uartConfig.swapTxRx = false;

// Apply the configuration
uart4.Init(uartConfig);
// Or
uart4.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::APB2), .baudrate = 115200, .dataBits = UART::DataBits::Bits8, .parity = UART::Parity::None, .stopBits = UART::StopBits::Bits1, .hwFlowControl = false, .swapTxRx = false});

uint8_t txBuffer[] = "Hello Instinct\r\n";
uint8_t rxBuffer[32];

// Transmit data asynchronously (buffered)
uart4.Transmit(txBuffer, sizeof(txBuffer) - 1);

// Check and read incoming data
if(uart4.Available() > 0) {
	uint32_t bytesRead = uart4.Receive(rxBuffer, sizeof(rxBuffer));
}
```