/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/dmaChannel.hpp
 * Author:  NotBlackMagic
 * Brief:   DMA channel driver class (HPDMA and GPDMA) for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
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

			// Multi-block reload hooks for enable/disable DMA requests
			void (*StateCallback)(void* ctx, bool enable) = nullptr;
			void* callbackContext = nullptr;

			// RIF / Security Settings
			// bool IsSecure;
			// bool IsPrivileged;
		};

		// Delete copy constructors
		DMAChannel(const DMAChannel&) = delete;
		DMAChannel& operator=(const DMAChannel&) = delete;

		/// @brief Constructor.
		/// @param type			DMA instance type (GPDMA or HPDMA).
		/// @param channelIndex	DMA channel index.
		DMAChannel(DMA_TypeDef *instance, uint8_t channelIndex);

		/// @brief Initializes the DMA clock, the dma channel itself, and interrupts.
		/// @param config DMA channel configuration
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config& config);

		/// @brief Starts a non-blocking transaction (Write, Read, or Write-then-Read).
		/// @param srcAddr	Source address.
		/// @param dstAddr	Destinaion address.
		/// @param len		Number of bytes to transfer.
		/// @return Status::Ok if the transfer started, or Status::Busy if the DMA channel is locked by another thread.
		Status Transfer(uint32_t srcAddr, uint32_t dstAddr, uint32_t len);
		
		// Status Transfer2D(uint32_t srcAddr, uint32_t dstAddr, const Config2D& config);
		// Status TransferList(LinkedListNode* headNode);

		/// @brief Blocks the current thread until the transfer completes.
		/// @param timeoutTicks Max wait time in OS ticks.
		/// @return Status::Ok if the transfer completed successfully, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status TransferWait(uint32_t timeoutTicks);

		/// @brief Stops or aborts the current transfer.
		/// @return Status::Ok if transfer stopped successfully, or Status::Error otherwise.
		Status TransferStop();

		uint32_t GetRemaining();

		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler();
	
	private:
		enum class Port {
			Port0 = 0,	// AXI (Memory/High-Speed)
			Port1 = 1	// AHB/APB (Peripherals)
		};

		DMA_TypeDef* instance;
		uint8_t channelIndex;
		IRQn_Type irqCall;
		uint8_t irqPriority;
		Config config;

		// Transaction Context, used for software-managed multi-block transfers (>65k limit and no 2D support, also for when required to disable DMA request between blocks i.e, JPEG)
		uint32_t currentSrc;
		uint32_t currentDst;
		uint32_t blockSize;
		uint32_t remainingBytes;

		// Synchronization
		TX_EVENT_FLAGS_GROUP event;

		// Event Flags Definitions
		static constexpr uint32_t EVT_TRANS_CPLT = 0x01;
		static constexpr uint32_t EVT_ERR = 0x02;

		// Timeout defines
		static constexpr uint32_t TIMEOUT_MUTEX = TX_WAIT_FOREVER;

		/// @brief Helper to configure Resource Isolation Framework (RIF)
		void ConfigureRIF(bool isSecure, bool isPrivileged);
		
		// Internal helper to find/get the correct bus port from an address
		Port GetPortFromAddress(uint32_t address);
};
