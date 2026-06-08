# General Purpose I/O (GPIO) Driver
The GPIO driver provides a simple object-oriented C++ driver for the digital IO pins of the STM32N6. It is built on top the STM32 Low-Layer (LL) API. It handles basic digital IO operations (read, writes) and also handles the correct mapping of the correct EXTI lines, if interrupts are required. For interrupt operation, the ```EXTIManager``` (exti.cpp) driver is also required to route the user callback to the physical GPIO EXTI lines interrupt handler.

## Architectural Overview
Pin configuration is handled by the ```GPIO::Config``` structure, passed to the Init function. Correct mapping of the EXTI line and NVIC IRQ number is handled internally, callbacks are registered and handled by the ```EXTIManager```, which must be assigned/routed to the EXTIn_IRQHandler with ```extern "C"```, see Interrupt Input example code. The GPIOs are hardcoded the pin speed to ```LL_GPIO_SPEED_FREQ_LOW```, typically enough as the GPIO driver is STRICTLY for independent, none peripheral/alternative functions, GPIOs.

## Example Code

### Basic Output

```c
#include "gpio.hpp"

// Define the configuration
GPIO::Config ledConfig;
ledConfig.mode = GPIO::Mode::Output;
ledConfig.type = GPIO::Output::PushPull;
ledConfig.pull = GPIO::Pull::NoPull;

// Instantiate the GPIO object (e.g., Port E, Pin 3)
GPIO led(GPIOE, LL_GPIO_PIN_3);

// Apply the configuration
led.Init(ledConfig);
// Or
led.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});

// Control the pin
led.Write(1);  // Set high
led.Toggle();  // Toggle state
```

### Interrupt Input

```c
#include "exti.hpp"
#include "gpio.hpp"

// Define the callback function
void ButtonCallback(void* context, EXTIManager::Edge edge) {
	if(edge == EXTIManager::Edge::Rising) {
		// Handle the rising edge event
	}
}

// Hardware IRQ Handler (Must exist in the global scope)
extern "C" void EXTI0_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_0) == 0x01) {
		EXTIManager::Dispatch(0, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_0);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_0) == 0x01) {
		EXTIManager::Dispatch(0, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_0);
	}
}

// Instantiate the GPIO object (e.g., Port A, Pin 0)
GPIO button(GPIOA, LL_GPIO_PIN_0);

// Apply the configuration
button.Init({.mode = GPIO::Mode::Input, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::PullDown});

// Register the callback (context is nullptr since it is not routing to a class instance)
EXTIManager::RegisterCallback(button.GetPinIndex(), ButtonCallback, nullptr);

// Enable the hardware interrupt (Trigger on Rising edge, Priority 15)
button.EnableIRQ(GPIO::Interrupt::Rising, 0x0E);
```