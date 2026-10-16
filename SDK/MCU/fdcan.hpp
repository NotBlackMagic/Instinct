/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	SDK/MCU/fdcan.hpp
 * Author:	NotBlackMagic
 * Brief:	FDCAN peripheral driver for STM32N6 using LL (Low-Layer) API and direct register access (NO HAL).
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"

#include "status.hpp"
#include "system.hpp"

#include "tx_api.h"

class FDCAN {
	public:
		/// @brief Low-level FDCAN hardware bus and packet statistics.
		struct Statistics {
			uint32_t txSuccess = 0;
			uint32_t rxSuccess = 0;
			uint32_t txErrors = 0;
			uint32_t rxErrors = 0;
			uint32_t busErrors = 0;
			uint32_t timeouts = 0;
		};

		/// @brief DCMI events
		enum class Event : uint8_t {
			TXComplete = 0,		///< Transmission acknowledged on the bus
			RXComplete,			///< New frame landed in RX FIFO
			Error,				///< Protocol or frame error encountered
			BusOff				///< Peripheral entered Bus-Off state
		};

		/// @brief CAN / CAN FD Frame representation.
		struct Frame {
			uint32_t id = 0;
			bool isExtended = false;
			bool isFD = false;
			bool bitRateSwitch = false;
			uint8_t length = 0;
			uint8_t data[64] = {0};
		};

		/// @brief FDCAN peripheral configuration structure.
		struct Config {
			uint32_t sourceClockHz;		///< Peripheral kernel source clock in Hz
			uint32_t nominalBaudrate;	///< Nominal bit rate in Hz (e.g. 1000000 for 1 Mbps)
			uint32_t dataBaudrate;		///< Data bit rate in Hz for FD BRS (e.g. 5000000 for 5 Mbps)
			bool fdMode;				///< Enable CAN FD frame formatting

			// Optional event callbacks
			void (*EventCallback)(void* ctx, Event evt);
			void* callbackContext;
		};

		// Delete copy constructors
		FDCAN(const FDCAN&) = delete;
		FDCAN& operator=(const FDCAN&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., FDCAN1, FDCAN2).
		FDCAN(FDCAN_GlobalTypeDef *instance);

		/// @brief Initializes the peripheral clock, the peripheral itself, and interrupts.
		/// @param config FD-CAN configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		Status DeInit();

		/// @brief Queues a frame into the TX FIFO and requests transmission on the bus (non-blocking).
		/// @param frame CAN frame to transmit.
		/// @return Status::Ok if queued, Status::Busy if TX FIFO is full, Status::Timeout if locked.
		Status TransferAsync(const Frame &frame);

		/// @brief Blocks the calling thread until the queued frame is acknowledged on the bus.
		/// @param timeoutTicks Maximum wait time in OS ticks.
		/// @return Status::Ok on bus ACK, Status::Timeout if expired, Status::Error on bus/transmission fault.
		Status TransferWait(uint32_t timeoutTicks);

		/// @brief Aborts a ongoing transfer.
		/// @return Status::Ok
		Status TransferAbort();

		/// @brief Receives a CAN frame (blocks until a message arrives in RX FIFO or timeout).
		Status Receive(Frame &frame, uint32_t timeoutTicks = TX_WAIT_FOREVER);

		/// @brief Returns FD-CAN bus statistics.
		Statistics GetStatistics() const { return this->stats; }

		/// @brief Resets FD-CAN bus statistics counters.
		void ResetStatistics() { this->stats = {}; }

		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler();

	private:
		FDCAN_GlobalTypeDef *instance;
		IRQn_Type irqCall;
		uint8_t irqPriority;
		
		Config config;
		bool isInitialized;
		Statistics stats = {};

		// Transaction Context
		volatile uint32_t activeTXBufferMask;

		// Fixed internal Message RAM allocations
		static constexpr uint8_t rxFIFO0Size = 32;			// 32 frames of 64 bytes
		static constexpr uint8_t txFIFOSize = 16;			// 16 frames of 64 bytes
		static constexpr uint32_t elementSizeWords = 18;	// 2 words header + 16 words data (64 bytes)

		uint32_t ramRXFIFO0Sa;
		uint32_t ramTXFIFOSa;

		// Synchronization
		TX_MUTEX txMutex;
		TX_EVENT_FLAGS_GROUP txEvent;
		TX_EVENT_FLAGS_GROUP rxEvent;

		// Event Flags Definitions
		static constexpr uint32_t EVT_TX_CPLT = 0x01;
		static constexpr uint32_t EVT_TX_ERR = 0x02;
		static constexpr uint32_t EVT_RX_NEW = 0x01;

		// Timeout defines
		static constexpr uint32_t TIMEOUT_MUTEX = TX_WAIT_FOREVER;

		// Internal helpers
		void ConfigureRam();
		Status ConfigureTiming(uint32_t nominalBaud, uint32_t dataBaud);
		void WriteTXBuffer(const Frame &frame, uint32_t bufferIndex);
		void ReadRXFifo(Frame &frame, uint32_t getIndex);

		static uint8_t DLCToBytes(uint8_t dlc);
		static uint8_t BytesToDLC(uint8_t bytes);
};