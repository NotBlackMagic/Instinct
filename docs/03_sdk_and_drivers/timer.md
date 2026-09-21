# Timer Driver
The Timer driver provides a simple object-oriented C++ driver for the general-purpose, basic, and advanced-control Timer peripherals of the STM32N6. It is built on top the STM32 Low-Layer (LL) API. It handles basic timebase configuration and counter control. More complex timer-based operations (e.g., PWM) are handled by higher-level drivers built on top of it.

## Architectural Overview
The driver automatically enables the appropriate peripheral bus clock (APB1 or APB2) based on the timer instance passed to the constructor. Pin and alternate function (AF) mapping are not handled by this driver and must be configured externally in the board support package or higher-level drivers.

During Init, the driver calculates the prescaler and auto-reload (ARR) values dynamically to match the requested frequency while maximizing the ARR value (up to 65536) for highest resolution in derivative drivers like PWM. The timer is set to up-counting mode with ARR preload enabled.

## Example Code

### Basic Timer

```c
#include "timer.hpp"

// Define the configuration (50 Hz timer)
Timer::Config timConfig;
timConfig.sourceClockHz = System::GetNodeFrequency(System::ClockNode::AXI);
timConfig.frequencyHz = 50;

// Instantiate the Timer object (e.g., TIM3)
Timer tim(TIM3);

// Apply the configuration
tim.Init(timConfig);
// Or
tim.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::AXI), .frequencyHz = 50});

// Start Timer
tim.Start();

// Reset counter to 0 immediately
tim.Reset();

// Stop Timer
tim.Stop();

```