/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	SDK/MCU/dmaChannel.hpp
 * Author:	NotBlackMagic
 * Brief:	DMA channel driver class (HPDMA and GPDMA) for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_exti.h"
#include "stm32n6xx_ll_dma.h"
#include "stm32n6xx_ll_dma2d.h"

#include "status.hpp"

#include "tx_api.h"

/// @brief Driver for the DMA peripherals (GPDMA and HPDMA).
class DMAChannel {
	public:
		/// @brief DMA low-level hardware bus and operational statistics.
		struct Statistics {
			uint32_t transfers = 0;			// Successfully completed transfers
			uint32_t transferErrors = 0;	// AXI/AHB slave error or RIF violation (DTE Interrupt/Flag)
			uint32_t settingErrors = 0;		// Configuration / burst misalignment error (USE Interrupt/Flag)
			uint32_t triggerOverruns = 0;	// Hardware trigger overrun (TO Interrupt/Flag)
			uint32_t userAborts = 0;		// Transfers aborted via TransferStop()
			uint64_t bytesTransferred = 0;	// Total accumulated bytes transferred
		};
	
		/// @brief DMA transfer direction.
		enum class Direction : uint32_t {
			PeripheralToMemory = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
			MemoryToPeripheral = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
			MemoryToMemory = LL_DMA_DIRECTION_MEMORY_TO_MEMORY
		};

		/// @brief DMA transfer data width.
		enum class DataWidth {
			Byte = 0,
			HalfWord = 1,
			Word = 2,
			DoubleWord = 3
		};

		/// @brief DMA transfer data width.
		enum class Priority : uint32_t {
			Low = LL_DMA_LOW_PRIORITY_LOW_WEIGHT,
			LowMid = LL_DMA_LOW_PRIORITY_MID_WEIGHT,
			LowHigh = LL_DMA_LOW_PRIORITY_HIGH_WEIGHT,
			High = LL_DMA_HIGH_PRIORITY
		};

		/// @brief DMA callback events.
		enum class Event {
			TransferComplete,
			TransferError,		// DTE: AXI/AHB bus fault or RIF access denial
			SettingError,		// USE: Alignment, burst, or 1KB boundary error
			TriggerOverrun,		// TO:  Hardware trigger overrun
			RequestDisable,		// Pre-reload hook: disable peripheral DMA request
			RequestEnable		// Post-reload hook: re-enable peripheral DMA request
		};

		struct Config {
			Direction direction;
			DataWidth srcWidth;
			DataWidth dstWidth;
			uint8_t requestLine;
			Priority priority;
			bool incSrc;
			bool incDst;

			// Advanced settings (Optional, defaults to 1)
			uint32_t srcBurstLength = 1; 
			uint32_t dstBurstLength = 1;

			// Set true if peripheral requires toggling its DMA request between >64KB blocks (e.g., JPEG)
			bool swBlockReload = false;

			// Optional event callback
			void (*EventCallback)(void* ctx, Event evt) = nullptr;
			void* callbackContext = nullptr;

			// RIF/Security Settings
			bool isSecure = true;
			bool isPrivileged = true;
		};

		// Delete copy constructors
		DMAChannel(const DMAChannel&) = delete;
		DMAChannel& operator=(const DMAChannel&) = delete;

		/// @brief Constructor.
		/// @param type			DMA instance type (GPDMA or HPDMA).
		/// @param channelIndex	DMA channel index.
		/// @param channelName	Optional channel name, used to idenitfy/display in shell.
		DMAChannel(DMA_TypeDef *instance, uint8_t channelIndex, const char* channelName = "DMA");

		/// @brief Initializes the DMA clock, the dma channel itself, and interrupts.
		/// @param config DMA channel configuration
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config& config);

		/// @brief Starts a non-blocking transaction (Write, Read, or Write-then-Read).
		/// @param srcAddr	Source address.
		/// @param dstAddr	Destinaion address.
		/// @param len		Number of bytes to transfer.
		/// @return Status::Ok if the transfer started, or Status::Busy if the DMA channel is locked by another thread.
		Status TransferAsync(uint32_t srcAddr, uint32_t dstAddr, uint32_t len);
		
		// Status Transfer2D(uint32_t srcAddr, uint32_t dstAddr, const Config2D& config);
		// Status TransferList(LinkedListNode* headNode);

		/// @brief Blocks the current thread until the transfer completes.
		/// @param timeoutTicks Max wait time in OS ticks.
		/// @return Status::Ok if the transfer completed successfully, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status TransferWait(uint32_t timeoutTicks);

		/// @brief Stops or aborts the current transfer.
		/// @return Status::Ok if transfer stopped successfully, or Status::Error otherwise.
		Status TransferStop();

		/// @brief Returns the number of bytes remaining in the current active transfer.
		uint32_t GetRemaining();

		/// @brief Retrieves the channel statistics.
		Statistics GetStatistics() const { return this->stats; }

		/// @brief Resets all statistics counters.
		void ResetStatistics() { this->stats = {}; }

		// Registry traversal
		static DMAChannel* GetFirstChannel() { return headChannel; }
		DMAChannel* GetNextChannel() const { return this->nextChannel; }

		const char* GetName() const { return (this->channelName) ? this->channelName : "Unknown"; }
		uint8_t GetChannelIndex() const { return this->channelIndex; }
		DMA_TypeDef* GetInstance() const { return this->instance; }

		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler();
	
	private:
		enum class Port {
			Port0 = 0,	// AXI (Memory/High-Speed)
			Port1 = 1	// AHB/APB (Peripherals)
		};

		DMA_TypeDef* instance;
		DMA_Channel_TypeDef* channelRegs;
		uint8_t channelIndex;
		IRQn_Type irqCall;
		uint8_t irqPriority;
		bool isInitialized = false;

		
		Config config;
		Statistics stats = {};

		// Linked list of all used/enabled DMA channels, for easy access in shell
		const char* channelName;
		DMAChannel* nextChannel = nullptr;
		static inline DMAChannel* headChannel = nullptr;

		static constexpr uint32_t MAX_SW_BLK_LEN = 65280;	//64-byte burst and JPEG aligned

		// Transaction Context, used for software-managed multi-block transfers (>65k limit and no 2D support, also for when required to disable DMA request between blocks i.e, JPEG)
		uint32_t currentSrc;
		uint32_t currentDst;
		uint32_t blockSize;
		uint32_t remainingBytes;
		uint32_t totalTransferLength;

		// Synchronization
		TX_EVENT_FLAGS_GROUP event;

		// Event Flags Definitions
		static constexpr uint32_t EVT_TRANS_CPLT = 0x01;
		static constexpr uint32_t EVT_ERR = 0x02;

		// Timeout defines
		static constexpr uint32_t TIMEOUT_MUTEX = TX_WAIT_FOREVER;

		/// @brief Safely suspends an active channel, waits for SUSPF, and issues a channel reset.
		Status SuspendAndReset();

		/// @brief Helper to configure Resource Isolation Framework (RIF)
		void ConfigureRIF(bool isSecure, bool isPrivileged);
		
		// Internal helper to find/get the correct bus port from an address
		Port GetPortFromAddress(uint32_t address);
};
