/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/sdmmc.hpp
 * Author:  NotBlackMagic
 * Brief:   SDMMC driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

 #pragma once

#include <stdint.h>

#include "stm32n6xx.h"
#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_gpio.h"
#include "stm32n6xx_ll_rcc.h"
#include "stm32n6xx_ll_sdmmc.h"

#include "status.hpp"
#include "system.hpp"

#include "tx_api.h"

class SDMMC {
	public:
		/// @brief Data lines used.
		enum class BusWidth : uint32_t {
			Lines_1 = (0x00000000U),						// 1-bit (Default at boot)
			Lines_4 = (0x1UL << SDMMC_CLKCR_WIDBUS_Pos),	// 4-bit (Standard uSD)
			Lines_8 = (0x2UL << SDMMC_CLKCR_WIDBUS_Pos)		// 8-bit (eMMC only)
		};
		
		/// @brief Response type for commands.
		enum class ResponseType : uint32_t {
			None = (0x00000000U),							// No response
			Short_CRC = (0x1UL << SDMMC_CMD_WAITRESP_Pos),	// 48-bit (R1, R3, R6) or CCRCFAIL
			Short_NoCRC = (0x2UL << SDMMC_CMD_WAITRESP_Pos),// 48-bit (R1, R3, R6) with no CRC
			Long = (0x3UL << SDMMC_CMD_WAITRESP_Pos)		// 136-bit (R2 - CID/CSD)
		};

		/// @brief Bus speed.
		enum class BusSpeed {
			// Standard Modes (3.3V)
			Default = 0,	// SD: DS (25MHz) | eMMC: Legacy (26MHz)
			HighSpeed = 1,	// SD: HS (50MHz) | eMMC: High Speed SDR (52MHz)
			// UHS-I Modes (1.8V - SD Card)
			UHS_SDR12 = 2,	// Up to 25 MHz
			UHS_SDR25 = 3,	// Up to 50 MHz
			UHS_SDR50 = 4,	// Up to 100 MHz
			UHS_DDR50 = 5,	// Up to 50 MHz (Dual Data Rate)
			UHS_SDR104 = 6,	// Up to 208 MHz (Requires Tuning)
			// eMMC Modes (1.8V - eMMC Chip)
			eMMC_DDR52 = 7,	// eMMC High Speed DDR (52MHz, Dual Rate) - Maps to DDR50 logic
			eMMC_HS200 = 8	// eMMC HS200 (200MHz, Single Rate) - Maps to SDR104 logic
		};

		/// @brief Data transfer trigger handling when command sent.
		enum class TransferMode {
			None, 		// No Data (CMD0, CMD2, CMD3, etc.)
			Auto,		// Auto data transfer enable (CMD17/18/24/25). Uses Hardware Auto-Sync (CMDTRANS bit).
			Manual		// Manual data transfer enable (CMD6, ACMD51, SDStatus). Manually enables DPSM.
		};

		/// @brief Error codes.
		enum class Error: uint32_t {
			None = 0x00,
			CmdCrcFail = 0x01,		// Command response received but CRC check failed
			CmdTimeout = 0x02,		// Command sent, but no response received within timeout
			CmdIndexMismatch = 0x03,
			DataCrcFail = 0x04,		// Data block received but CRC check failed
			DataTimeout = 0x05,		// Data timeout (DTIMEOUT)
			TxUnderrun = 0x06,		// FIFO empty while IDMA trying to write to card
			RxOverrun = 0x07,		// FIFO full while IDMA trying to read from card
			StartBitError = 0x08,	// Start bit not detected on all data signals
			Busy = 0x09,			// Hardware or Driver is busy
			Param = 0x0A,			// Invalid parameter passed to function
			Internal = 0x0B,		// RTOS error (Mutex/Event failure)
			DmaError = 0x0C,		// IDMA transfer error
			Hardware = 0x0D,		// Generic hardware failure
			Aborted = 0x0E			// Command/transfer aborted do to driver killed/DeInit
		};

		/// @brief SDMMC peripheral configuration structure.
		struct Config {
			uint32_t sourceClockHz;		///< Peripheral source clock frequency in Hz
			bool hwFlowControl;
		};

		/// @brief Command response structure.
		struct CommandResponse {
			uint32_t resp[4];
			Error error;
		};

		// Delete copy constructors
		SDMMC(const SDMMC&) = delete;
		SDMMC& operator=(const SDMMC&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., SDMMC1, SDMMC2).
		SDMMC(SDMMC_TypeDef *instance);

		/// @brief Initializes the peripheral clock, the peripheral itself, and interrupts.
		/// @param config SDMMC configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		/// @brief Completely resets the SDMMC peripheral and aborts any pending transactions.
		/// @return Status::Ok if reset was successful.
		Status DeInit(void);

		/// @brief Sets the bus clock frequency.
		/// @param freqHz Target frequency (dividers calculated automatically).
		/// @return Status::Ok if change succeeded.
		Status SetClock(uint32_t freqHz);

		/// @brief Sets the data bus width.
		/// @param width Target bus width.
		/// @return Status::Ok if change succeeded.
		Status SetBusWidth(BusWidth width);

		/// @brief Configures protocol timing (Sampling edge, DDR, etc).
		/// @note  Does NOT change frequency. Call SetClock() separately.
		/// @param mode Target bus speed mode.
		/// @return Status::Ok if change succeeded.
		Status SetSpeedMode(BusSpeed mode);

		/// @brief Sends a command transfer, configuring the command path state machine. Blocking until command response.
		/// @param cmd		Command code to send.
		/// @param args		Arguments for the command.
		/// @param respType	Response type for the command.
		/// @param mode		If is followed by a data transfer and how to handle the trigger
		/// @return The command response.
		CommandResponse Command(uint8_t cmd, uint32_t args, ResponseType respType, TransferMode mode = TransferMode::None);

		/// @brief Configures IDMA and starts the Data Path State Machine (Non-blocking).
		/// @note  You MUST send the associated SD Command (e.g., CMD17/CMD25) after calling this.
		/// @param buf		Pointer to the data buffer (must be 32-byte aligned for Cache).
		/// @param len		Number of bytes to transfer.
		/// @param blkSize	Transfer block size in bytes, must be power 2 from 1 to 512.
		/// @param isRead	If transfer is a read transfer.
		/// @return Status::Ok if the transfer started, or Status::Busy if the SDMMC is locked by another thread.
		Status TransferAsync(uint8_t *buf, uint32_t len, uint32_t blkSize, bool isRead);

		/// @brief Blocks the current thread until the Async transfer completes.
		/// @param timeoutTicks Max wait time in OS ticks.
		/// @return Status::Ok if the transfer completed successfully, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status TransferWait(uint32_t timeoutTicks);

		/// @brief Switches the SDMMC interface to 1.8V logic levels.
		/// @return Status::Ok if switch succeeded.
		Status SwitchTo1V8(void (*callback)(void*), void* context);

		/// @brief Helper to get D0 busy status
		/// @param timeoutMs Max wait time in ms.
		/// @return Status::Ok if D0 not busy, Status::Timeout on timeout
		Status WaitBusyD0(uint32_t timeoutMs);

		/// @brief Helper to get last error
		Error GetLastError() const { return this->lastError; }

		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler(void);

	private:
		SDMMC_TypeDef *instance;
		IRQn_Type irqCall;
		uint8_t irqPriority;
		uint32_t irqStatus;

		bool isInitialized;
		uint32_t sourceClockHz;

		// Transaction Context
		volatile Error lastError;
		uint8_t * volatile buffer;
		volatile uint32_t length;
		volatile uint32_t blkSize;
		volatile bool dirRead;

		// Synchronization
		TX_MUTEX mutex;
		TX_EVENT_FLAGS_GROUP event;

		// Event Flag Definitions
		static constexpr uint32_t EVT_DATA_CPLT = 0x01;
		static constexpr uint32_t EVT_DATA_ERR = 0x02;
		static constexpr uint32_t EVT_CMD_CPLT = 0x04;		// Command Complete (CMDREND)
		static constexpr uint32_t EVT_CMD_ERR = 0x08;		// Command Error (Timeout/CRC)

		// Timeout defines
		static constexpr uint32_t TIMEOUT_MUTEX = TX_WAIT_FOREVER;

		/// @brief Helper to configure Resource Isolation Framework (RIF)
		void ConfigureRIF(void);
};