# EXTI (Extended Interrupts and Events) Manager
The EXTI Manager provides a static routing layer between the STM32N6 GPIO interrupts to C++ callback functions. Because bare-metal hardware ISRs must be standard C functions, this manager provides a safe way to map those hardware triggers back to specific application functions.

## Architectural Overview
The STM32 architecture maps all GPIO pins with the same number to a single EXTI line (e.g., PA0, PB0, and PC0 all share ```EXTI_LINE_0```). Because of this, the ```EXTIManager``` maintains a single 16-element callback table. Only one callback can be registered per EXTI line, if the callback has to be changed it must first be unregistered.

The registration and unregistration functions temporarily mask global interrupts using ```__disable_irq()``` and ```__set_PRIMASK()``` to ensure thread safety.

The manager allows passing a ```void* context``` pointer during registration. Simple implementations can pass ```nullptr``` if no context is required, but it can be used to pass a C++ ```this``` pointer to route directly into class member functions.

## Example Code

### Basic Callback Routing

```c
#include "exti.hpp"
#include "gpio.hpp"

// Define the callback function
void ButtonCallback(void* context, EXTIManager::Edge edge) {
	if(edge == EXTIManager::Edge::Falling) {
		// Handle the falling edge event
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
button.Init({.mode = GPIO::Mode::Input, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::PullUp});

// Register the callback (context is nullptr since it is not routing to a class instance)
EXTIManager::RegisterCallback(button.GetPinIndex(), ButtonCallback, nullptr);

// Enable the hardware interrupt (Trigger on Falling edge, Priority 5)
button.EnableIRQ(GPIO::Interrupt::Falling, 0x05);
```