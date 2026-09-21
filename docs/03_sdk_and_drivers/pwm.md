# Pulse Width Modulation (PWM) Driver
The PWM driver provides a simple object-oriented C++ driver for generating PWM signals on the STM32N6. It is built on top of the STM32 Low-Layer (LL) API and operates as a secondary driver that attaches to an existing Timer instance. It handles channel output configuration, polarity, and duty cycle modulation either via a normalized percentage or a direct pulse width in microseconds.

## Architectural Overview
The driver separates channel modulation from timebase generation and physical pin routing. The underlying Timer peripheral must be initialized separately, and the GPIO configurations must be handled outside the driver prior to initialization. In the Instinct firmware, this is managed in the board specific package (e.g., in "Board/PlumaN6.cpp" via ```BoardPWMxInit()```).

Channels are configured in PWM Mode 1 (```LL_TIM_OCMODE_PWM1```) with Output Compare Preload enabled i.e. duty cycle updates latch synchronously on timer update events. Main output enable (```LL_TIM_EnableAllOutputs```) is asserted during initialization. On initialization, compare values default to 0% duty cycle.

Duty cycle can be controlled via ```SetDutyCycle``` (normalized values between 0 and 1) or ```SetPulseWidth``` in microseconds (us). ```SetPulseWidth``` uses the parent Timer frequency dynamically to calculate the correct tick duration.

## Example Code

### PWM Output

```c
#include "pwm.hpp"

// Define the configuration
PWM::Config pwmConfig;
pwmConfig.polarity = PWM::Polarity::High;

// Initialize the base Timer (e.g., TIM2 at 50 Hz for standard servos)
Timer timer2(TIM2);
timer2.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::AXI), .frequencyHz = 50});
timer2.Start();

// Instantiate the PWM channel (e.g., TIM2 Channel 1)
PWM pwm(timer2, PWM::Channel::Ch1);

// Apply the configuration
pwm.Init(pwmConfig);
// Or
pwm.Init({.polarity = PWM::Polarity::High});

// Enable channel output
pwm.Start();

// Set duty cycle to 50%
pwm.SetDutyCycle(0.5f);

// Set pulse width to 1500us
pwm.SetPulseWidth(1500);

// Disable channel output
pwm.Stop();

```