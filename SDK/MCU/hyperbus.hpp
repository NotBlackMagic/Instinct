/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/hyperbus.hpp
 * Author:  NotBlackMagic
 * Brief:   HyperBus driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_gpio.h"

#include "status.hpp"
#include "system.hpp"

#if defined (USE_RTOS)
#include "tx_api.h"
#endif

/// @brief Driver for the HyperBus peripheral supporting direct write and memory mapped modes
class HyperBus {
	public:
		/// @brief Defines if accessing the memory array or the registers.
		/// @note  HyperBus Protocol Bit 46 (0 = Memory, 1 = Register).
		enum class AddressSpace : uint32_t {
			Memory   = 0,
			Register = 1 
		};

		/// @brief Data lines used for data phase.
		enum class BusWidth : uint32_t {
			Lines_0 = (0x00000000U),										//No data
			Lines_1 = ((uint32_t)XSPI_CCR_DMODE_0),							//Transfer on single line
			Lines_2 = ((uint32_t)XSPI_CCR_DMODE_1),							//Transfer on two lines
			Lines_4 = ((uint32_t)(XSPI_CCR_DMODE_0 | XSPI_CCR_DMODE_1)),	//Transfer on four lines
			Lines_8 = ((uint32_t)XSPI_CCR_DMODE_2),							//Transfer on eight lines
			Lines_16 = ((uint32_t)(XSPI_CCR_DMODE_0 | XSPI_CCR_DMODE_2))	//Transfer on sixten lines
		};

		/// @brief Address size in bits.
		enum class AddrSize : uint32_t {
			Width_8 = (0x00000000U),
			Width_16 = ((uint32_t)XSPI_CCR_ADSIZE_0),
			Width_24 = ((uint32_t)XSPI_CCR_ADSIZE_1),
			Width_32 = ((uint32_t)XSPI_CCR_ADSIZE)
		};

		/// @brief Latency mode.
		enum class LatencyMode : uint8_t {
			Variable = 0x00,	// Wait for RWDS
			Fixed = 0x01		// Fixed Cycle Count
		};

		/// @brief HyperBus peripheral configuration structure.
		struct Config {
			uint32_t sizeBytes;			///< Total size in bytes (e.g. 8*1024*1024)
			uint32_t sourceClockHz;		///< Peripheral source clock frequency in Hz
			uint32_t frequencyHz;		///< Target bus frequency in Hz
			
			// Protocol and Timing
			LatencyMode latencyMode;	///< Variable or Fixed
			uint8_t initialLatency;		///< Clock cycles (latency count)
			uint8_t rwRecoveryTime;		///< Additional latency after read/write
			bool writeZeroLatency;		///< True if memory supports 0-latency writes
			
			uint32_t refreshRate;		///< Max CS low time (for refresh and only for PSRAM)
		};

		// Delete copy constructors.
		HyperBus(const HyperBus&) = delete;
		HyperBus& operator=(const HyperBus&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., XPSI1, XPSI2).
		HyperBus(XSPI_TypeDef *instance);

		/// @brief Initializes the peripheral clock, the peripheral itself, and interrupts.
		/// @param config HyperBus configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		/// @brief Completely resets the HyperBus peripheral and aborts any pending transactions.
		/// @return Status::Ok if reset was successful.
		Status DeInit(void);

		/// @brief Locks the bus using RTOS mutex.
		/// @param timeoutTicks Max wait time in OS ticks to wait for mutex.
		/// @return Status::Ok if lock successful or Status::Timeout if lock timeout.
		Status LockBus(uint32_t timeoutTicks);

		/// @brief Unlocks the bus using RTOS mutex.
		/// @return Status::Ok or Status::Error.
		Status UnlockBus();

		/// @brief Starts a non-blocking transaction (Write, Read, or Write-then-Read).
		/// @param space	Target Space (Memory vs Register).
		/// @param addr		Target address.
		/// @param addrSize Target address width.
		/// @param width	Bus width for this transaction.
		/// @param buf		Pointer to the data buffer.
		/// @param len		Number of bytes to transfer.
		/// @param isRead	If transfer is a read transfer.
		/// @return Status::Ok if the transfer started, or Status::Busy if the HyperBus is locked by another thread.
		Status TransferAsync(AddressSpace space, uint32_t addr, AddrSize addrSize, BusWidth width, uint8_t* buf, uint32_t len, bool isRead);

		/// @brief Blocks the current thread until the Async transfer completes.
		/// @param timeoutTicks Max wait time in OS ticks.
		/// @return Status::Ok if the transfer completed successfully, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status TransferWait(uint32_t timeoutTicks);

		/// @brief Gets the memory mapped base address (e.g. 0x90000000).
		/// @return Memory mapped base address.
		uint32_t GetBaseAddr() const;

		/// @brief Enter into memory mapped mode
		/// @return Status::Ok if memory mapped mode entered, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status EnterMemoryMappedMode();

		/// @brief Exit memory mapped mode
		/// @return Status::Ok if memory mapped mode exit, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status ExitMemoryMappedMode();

		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler();
		
	private:
		XSPI_TypeDef *instance;
		IRQn_Type irqCall;
		uint8_t irqPriority;
		uint32_t irqStatus;

		bool isInitialized;
		bool isMemoryMapped;
		bool useWriteZeroLatency;

		// Transaction Context
		uint16_t *buffer;
		uint32_t length;
		bool dirRead;

		// Event Flags Definitions
		static constexpr uint32_t EVT_TRANS_CPLT = 0x01;
		static constexpr uint32_t EVT_ERR = 0x02;


#if defined (USE_RTOS)
		// Synchronization
		TX_MUTEX mutex;
		TX_EVENT_FLAGS_GROUP event;
		// Timeout defines
		static constexpr uint32_t TIMEOUT_MUTEX = TX_WAIT_FOREVER;
#else
		volatile uint32_t eventFlags;
        static constexpr uint32_t TIMEOUT_MUTEX = 0xFFFFFFFF;
#endif
		
		// Internal helpers
		Status Command(AddressSpace space, uint32_t addr, AddrSize addrSize, BusWidth width, uint32_t dataLen);
};