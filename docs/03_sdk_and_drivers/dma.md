# Direct memory access (DMA) Driver
The DMA driver provides a simple object-oriented C++ driver for the GPDMA and HPDMA peripheral of the STM32N6. It is built on top the STM32 Low-Layer (LL) API. It handles interrupt-driven, asynchronous data transfers with native ThreadX RTOS integration for thread blocking during operations. It automatically manages bus port routing, RIF security attributes, and hardware or software chunking for transfers exceeding standard 16-bit block limits.

## Architectural Overview
The driver supports both GPDMA1 and HPDMA1 instances across all available channels. Peripheral bus clock gating, Resource Isolation Framework (RIF) security/privilege configuration, and NVIC interrupt routing are managed internally during Init. Port allocation (Port 0 for AXI/memory spaces and Port 1 for AHB/APB peripheral ranges) is evaluated and assigned dynamically based on the source and destination addresses.

 Transactions are split into a non-blocking trigger/start (```TransferAsync```) and a blocking wait (```TransferWait```). The driver relies on ThreadX event flags (```tx_event_flags```) to yield the CPU until transfer completion or error. For transfers exceeding the 16-bit hardware block size (65535 bytes), the driver automatically handles chunking: using hardware 2D block repeat counters on advanced channels (channels 12–15) or falling back to interrupt-driven software chunking (channels 0–11 or when peripheral ```StateCallback``` is attached).

## Example Code

### Memory-to-Memory Transfer

```c
#include "dmaChannel.hpp"

// Instantiate the DMA channel object (e.g., HPDMA1 Channel 0)
DMAChannel dma(HPDMA1, LL_DMA_CHANNEL_0);

// Hardware IRQ Handler (Must exist in the global scope)
extern "C" void HPDMA1_Channel0_IRQHandler(void) { dma.InterruptHandler(); }

// Define the configuration
DMAChannel::Config dmaConfig;
dmaConfig.direction = DMAChannel::Direction::MemoryToMemory;
dmaConfig.srcWidth = DMAChannel::DataWidth::Byte;
dmaConfig.dstWidth = DMAChannel::DataWidth::Byte;
dmaConfig.incSrc = true;
dmaConfig.incDst = true;
dmaConfig.srcBurstLength = 1;
dmaConfig.dstBurstLength = 1;
dmaConfig.priority = DMAChannel::Priority::Low;
dmaConfig.requestLine = 0; // Software trigger for memory-to-memory
dmaConfig.StateCallback = nullptr;

// Apply the configuration
dma.Init(dmaConfig);

uint8_t srcBuffer[256];
uint8_t dstBuffer[256];

// Populate sample data
for(size_t i = 0; i < sizeof(srcBuffer); i++) {
	srcBuffer[i] = static_cast<uint8_t>(i);
}

// Start asynchronous transfer
dma.TransferAsync((uint32_t)srcBuffer, (uint32_t)dstBuffer, sizeof(srcBuffer));

// Yield CPU until transfer completes or errors
dma.TransferWait(TX_WAIT_FOREVER);

```