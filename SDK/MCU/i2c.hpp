/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/i2c.hpp
 * Author:  NotBlackMagic
 * Brief:   I2C driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_exti.h"
#include "stm32n6xx_ll_gpio.h"
#include "stm32n6xx_ll_i2c.h"

#include "status.hpp"
#include "system.hpp"

#include "tx_api.h"

/// @brief Driver for the I2C peripheral supporting Blocking and Async (Interrupt) modes.
/// @note  This driver integrates with ThreadX mutexes for thread safety.
class I2C {
	public:
		/// @brief Defines the I2C bus speed.
		enum class Mode {
			Standard,		///< 100kHz (Standard Mode)
			Fast,			///< 400kHz (Fast Mode)
			FastPlus		///< 1MHz (Fast Mode Plus)
		};

		/// @brief I2C peripheral configuration structure.
		struct Config {
			uint32_t sourceClockHz;	///< Peripheral source clock frequency in Hz
			Mode mode;				///< The target bus speed configuration.
		};

		// Delete copy constructors
		I2C(const I2C&) = delete;
		I2C& operator=(const I2C&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., I2C1, I2C2).
		I2C(I2C_TypeDef *instance);

		/// @brief Initializes the peripheral clock, the peripheral itself, and interrupts.
		/// @param config I2C configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		/// @brief Checks if a device exists on the bus.
		/// @param addr The 7-bit slave address to check.
		/// @return 1 (True) if the device acknowledges (ACK), 0 if NACK.
		uint8_t Probe(uint16_t addr);
		
		/// @brief Starts a non-blocking transaction (Write, Read, or Write-then-Read).
		/// @note  This function blocks momentarily if the BUS is busy, then returns.
		/// @details Returns immediately. Use TransferWait() to synchronize completion.
		/// @param addr   7-bit slave address.
		/// @param txBuf  Pointer to data to write (need to be kept until end of transfer).
		/// @param txLen  Number of bytes to write.
		/// @param rxBuf  Pointer to buffer for read data.
		/// @param rxLen  Number of bytes to read.
		/// @return Status::Ok if the transfer started, or Status::Busy if the I2C is locked by another thread.
		Status TransferAsync(uint16_t addr, const uint8_t *txBuf, uint16_t txLen, uint8_t *rxBuf, uint16_t rxLen);
		
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
		I2C_TypeDef *instance;
		IRQn_Type irqCall;
		uint8_t irqPriority;

		bool isInitialized;

		// Transaction Context
		uint8_t address;
		const uint8_t * volatile txBuffer;
		volatile uint16_t txLength;
		uint8_t * volatile rxBuffer;
		volatile uint16_t rxLength;

		// Synchronization
		TX_MUTEX mutex;
		TX_EVENT_FLAGS_GROUP event;

		// Event Flags Definitions
		static constexpr uint32_t EVT_TRANS_CPLT = 0x01;
		static constexpr uint32_t EVT_ERR = 0x02;

		// Timeout defines
		static constexpr uint32_t TIMEOUT_BUSY_US = 25;
		static constexpr uint32_t TIMEOUT_MUTEX = TX_WAIT_FOREVER;
};