/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/uart.hpp
 * Author:  NotBlackMagic
 * Brief:   UART driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_usart.h"

#include "status.hpp"

#include "tx_api.h"

/// @brief Driver for the UART peripheral supporting Blocking and Async (Interrupt) modes.
/// @note  This driver integrates with ThreadX mutexes for thread safety.
class UART {
	public:
		/// @brief Defines the UART data width.
		enum class DataBits : uint32_t {
			DataBits_7 = LL_USART_DATAWIDTH_7B,
			DataBits_8 = LL_USART_DATAWIDTH_8B,
			DataBits_9 = LL_USART_DATAWIDTH_9B
		};

		/// @brief Defines the UART stop bits.
		enum class StopBits : uint32_t {
			StopBits_05 = LL_USART_STOPBITS_0_5,
			StopBits_1 = LL_USART_STOPBITS_1,
			StopBits_15 = LL_USART_STOPBITS_1_5,
			StopBits_2 = LL_USART_STOPBITS_2
		};

		/// @brief Defines the UART parity.
		enum class Parity : uint32_t {
			None = LL_USART_PARITY_NONE,
			Odd = LL_USART_PARITY_ODD,
			Even = LL_USART_PARITY_EVEN
		};

		/// @brief UART peripheral configuration structure.
		struct Config {
			uint32_t sourceClockHz;	///< Peripheral source clock frequency in Hz
			uint32_t baudrate;		///< Baudrate.
			DataBits dataBits;		///< Data width mode.
			StopBits stopBits;		///< Stop bits mode.
			Parity parity;			///< Parity mode.
			uint8_t hwFlowControl;	///< Enable hardware flow control (0: Disable, 1: Enable).
		};

		// Delete copy constructors
		UART(const UART&) = delete;
		UART& operator=(const UART&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., USART1, USART2).
		UART(USART_TypeDef *instance);

		/// @brief Initializes the peripheral clock, the peripheral itself, and interrupts.
		/// @param config UART configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		/// @brief Writes data to the internal ring buffer.
		/// @param buf Pointer to data to write (will be copied to ring buffer so does not need to be kept after function call).
		/// @param len Number of bytes to write.
		/// @return Status::Ok if the transfer started, or Status::Busy if the UART is locked by another thread.
		Status Write(uint8_t *buf, uint16_t len);

		/// @brief Read data from the internal ring buffer.
		/// @param buf Pointer to buffer for read data.
		/// @param len Maximum number of bytes to read.
		/// @return Number of bytes read.
		uint32_t Read(uint8_t *buf, uint32_t maxLen);

		/// @brief Get number of bytes available to read from internal ring buffer.
		/// @return Number of bytes read.
		uint32_t Available();

		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler();
		
	private:
		USART_TypeDef *instance;
		IRQn_Type irqCall;
		uint8_t irqPriority;

		bool isInitialized;

		// Transaction Context
		static constexpr uint16_t rxBufferSize = 256;
		static constexpr uint16_t txBufferSize = 2048;

		__attribute__((aligned(32))) uint8_t rxBuffer[rxBufferSize];
		volatile uint16_t rxBufHead;
		volatile uint16_t rxBufTail;

		__attribute__((aligned(32))) uint8_t txBuffer[txBufferSize];
		volatile uint16_t txBufHead;
		volatile uint16_t txBufTail;
		volatile bool txBusy;

		// Synchronization
		TX_MUTEX mutex;

		// Event Flags Definitions
		static constexpr uint32_t EVT_TRANS_CPLT = 0x01;
		static constexpr uint32_t EVT_ERR = 0x02;

		// Timeout defines
		static constexpr uint32_t TIMEOUT_MUTEX = TX_WAIT_FOREVER;

		void StartTX();
};