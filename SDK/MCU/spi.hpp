 /*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/spi.hpp
 * Author:  NotBlackMagic
 * Brief:   SPI driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_exti.h"
#include "stm32n6xx_ll_gpio.h"
#include "stm32n6xx_ll_spi.h"

#include "status.hpp"

#include "tx_api.h"

/// @brief Driver for the SPI peripheral supporting Blocking and Async (Interrupt) modes.
/// @note  This driver integrates with ThreadX mutexes for thread safety.
class SPI {
	public:
		/// @brief SPI clock polarity, clock idle state.
		enum class ClockPolarity : uint32_t {
			Low = LL_SPI_POLARITY_LOW,
			High = LL_SPI_POLARITY_HIGH
		};

		/// @brief SPI clock phase, which clock edge to sample data.
		enum class ClockPhase : uint32_t {
			FirstEdge = LL_SPI_PHASE_1EDGE,
			SecondEdge = LL_SPI_PHASE_2EDGE
		};

		/// @brief SPI data bit order.
		enum class BitOrder : uint32_t {
			MSBFirst = LL_SPI_MSB_FIRST,
			LSBFirst = LL_SPI_LSB_FIRST
		};

		/// @brief SPI peripheral configuration structure.
		struct Config {
			uint32_t sourceClockHz;		///< Peripheral source clock frequency in Hz
			uint32_t baudrate;
			ClockPolarity polarity;
			ClockPhase phase;
			BitOrder bitOrder;
		};

		// Delete copy constructors
		SPI(const SPI&) = delete;
		SPI& operator=(const SPI&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., SPI1, SPI2).
		SPI(SPI_TypeDef *instance);

		/// @brief Initializes the peripheral clock, the peripheral itself, and interrupts.
		/// @param config SPI configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		/// @brief Updates the SPI baudrate at runtime.
		/// @param targetBaudrate Desired frequency in Hz.
		/// @return Status::Ok if the clock was updated, Status::Error if out of range.
		Status SetBaudrate(uint32_t baudrate);

		/// @brief Starts a non-blocking transaction (Write, Read, or Write-then-Read).
		/// @details Returns immediately. Use TransferWait() to synchronize completion.
		/// @param txBuf  Pointer to data to write (need to be kept until end of transfer).
		/// @param rxBuf  Pointer to buffer for read data.
		/// @param len    Number of bytes to write/read.
		/// @return Status::Ok if the transfer started, or Status::Busy if the SPI is locked by another thread.
		Status TransferAsync(const uint8_t *txBuf, uint8_t *rxBuf, uint32_t len);

		/// @brief Blocks the current thread until the Async transfer completes.
		/// @param timeoutTicks Max wait time in OS ticks.
		/// @return Status::Ok if the transfer completed successfully, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status TransferWait(uint32_t timeoutTicks);

		/// @brief Aborts a ongoing transfer.
		/// @return Status::Ok
		Status TransferAbort();

		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler();
		
	private:
		SPI_TypeDef *instance;
		IRQn_Type irqCall;
		uint8_t irqPriority;

		bool isInitialized;
		uint32_t sourceClockHz;

		// Transaction Context
		const uint8_t * volatile txBuffer;
		uint8_t * volatile rxBuffer;
		volatile uint32_t length;

		// Synchronization
		TX_MUTEX mutex;
		TX_EVENT_FLAGS_GROUP event;

		// Event Flags Definitions
		static constexpr uint32_t EVT_TRANS_CPLT = 0x01;
		static constexpr uint32_t EVT_ERR = 0x02;

		// Timeout defines
		static constexpr uint32_t TIMEOUT_MUTEX = TX_WAIT_FOREVER;
};