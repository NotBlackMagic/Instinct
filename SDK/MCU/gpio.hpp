/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/gpio.hpp
 * Author:  NotBlackMagic
 * Brief:   GPIO driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_exti.h"
#include "stm32n6xx_ll_gpio.h"

/// @brief Simple C++ GPIO driver for STM32 using LL (Low-Layer) API.
/// @note  Provides pin configuration, read/write/toggle functions, and optional EXTI interrupt setup without dynamic memory usage.
class GPIO {
	public:
		/// @brief GPIO operating mode.
		enum class Mode : uint32_t {
			Analog = LL_GPIO_MODE_ANALOG,		///< Analog mode (no digital input/output)
			Input = LL_GPIO_MODE_INPUT,			///< Digital input mode
			Output = LL_GPIO_MODE_OUTPUT		///< Digital output mode
		};

		/// @brief Internal pull resistor configuration.
		enum class Pull : uint32_t {
			NoPull = LL_GPIO_PULL_NO,			///< No pull-up or pull-down
			PullUp = LL_GPIO_PULL_UP,			///< Enable internal pull-up
			PullDown = LL_GPIO_PULL_DOWN		///< Enable internal pull-down
		};

		/// @brief Internal pull resistor configuration.
		enum class Output : uint32_t {
			PushPull = LL_GPIO_OUTPUT_PUSHPULL,		///< Push-Pull output drive
			OpenDrain = LL_GPIO_OUTPUT_OPENDRAIN,	///< Open-drain output drive
		};

		/// @brief External interrupt (EXTI) trigger configuration.
		enum class Interrupt : uint32_t {
			None = LL_EXTI_TRIGGER_NONE,			///< No interrupt
			Rising = LL_EXTI_TRIGGER_RISING,		///< Trigger on rising edge
			Falling = LL_EXTI_TRIGGER_FALLING,		///< Trigger on falling edge
			Both = LL_EXTI_TRIGGER_RISING_FALLING	///< Trigger on both edge
		};

		/// @brief GPIO peripheral configuration structure.
		struct Config {
			Mode mode;		///< Pin mode (Input, Output, Analog)
			Output type;	///< Output type (Push-Pull, Open-Drain)
			Pull pull;		///< Internal pull resistor configuration
		};

		/// @brief Construct a GPIO object.
		/// @param port Pointer to GPIO port (e.g., GPIOA, GPIOB, ...)
		/// @param pin  Pin number (LL_GPIO_PIN_x)
		GPIO(GPIO_TypeDef *port, uint32_t pin);

		/// @brief Initialize the GPIO pin.
		/// @param config GPIO configuration.
		void Init(const Config &config);

		/// @brief Write logic level to output pin.
		/// @param level 0 = Low, nonzero = High
		void Write(uint8_t level);

		/// @brief Read logic level from input pin.
		/// @return 0 if low, 1 if high
		uint8_t Read();

		/// @brief Toggle current output state.
		void Toggle();

		/// @brief Enable EXTI interrupt for this GPIO pin.
		/// @param trigger      Trigger condition (Rising/Falling/Both)
		/// @param irqPriority  NVIC priority for the EXTI line
		void EnableIRQ(Interrupt trigger, uint8_t irqPriority);

		/// @brief Disable EXTI interrupt for this GPIO pin.
		void DisableIRQ();
		
	private:
		GPIO_TypeDef *port;			//GPIO port base address
		IRQn_Type irqCall;			//IRQ number for associated EXTI line
		uint8_t irqPriority;		//Configured NVIC priority

		uint32_t pin;				//Pin mask (LL_GPIO_PIN_x)
		uint32_t extiPort;			//EXTI port source
		uint32_t extiLine;			//EXTI line number (used in LL_EXTI_EnableRisingTrig_0_31 and LL_EXTI_EnableIT_0_31)
		uint32_t extiExtiLine;		//EXTI EXTI line number (used in LL_EXTI_SetEXTISource)
};