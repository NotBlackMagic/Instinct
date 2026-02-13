/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/i3c.hpp
 * Author:  NotBlackMagic
 * Brief:   I3C driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_exti.h"
#include "stm32n6xx_ll_gpio.h"
#include "stm32n6xx_ll_i3c.h"

#include "status.hpp"
#include "system.hpp"

#include "tx_api.h"

class I3C {
	public:
		/// @brief Defines the I3C bus speed for Mixed Mode operation
		enum class Mode {
			Legacy_Fast,		///< Only 400kHz (I2C Fast Mode)
			Mixed_Fast,			///< Mixed legacy I2C 400kHz (Fast Mode) and I3C SDR at 12.5MHz
			Mixed_FastPlus,		///< Mixed legacy I2C 1MHz (Fast Mode Plus) and I3C SDR at 12.5MHz
			Pure_I3C_SDR		///< No I2C only I3C SDR at 12.5MHz
		};

		/// @brief Transaction target device type (I2C or I3C)
		enum class TargetType {
			I2C,	///< Standard/legacy I2C device with static 7-bit address
			I3C		///< I3C device with assigned dynamic address
		};

		enum class BroadcastCCC : uint8_t {
			EnableEvents = 0x00,		///< ENEC: Enable salve event driven interrupts 
			DisableEvents = 0x01,		///< DISEC: Disable salve event driven interrupts 
			EnterActivityState0 = 0x02,	///< ENTAS0: Set activity mode to 0 (Normal Operation)
			EnterActivityState1 = 0x03,	///< ENTAS1: Set activity mode to 1
			EnterActivityState2 = 0x04,	///< ENTAS2: Set activity mode to 2
			EnterActivityState3 = 0x05,	///< ENTAS3: Set activity mode to 3
			ResetDAA = 0x06,			///< RSTDAA: Reset dynamic address assignment
			EnterDAA = 0x07,			///< ENTDAA: Start dynamic address assignment procedure
			DefineListTargets = 0x08,	///< DEFTGTS: Master defines dynamic address, DCR type, and static address (or 0) per slave
			SetMaxWriteLen = 0x09,		///< SETMWL: Set maximum write length in a single command, for all devices
			SetMaxReadLen = 0x0A,		///< SETMRL: Set maximum read length in a single command, for all devices
			EnterTestMode = 0x0B,		///< ENTTM: Enter test mode
			SetExchangeTime = 0x28,		///< SETXTIME: Synchronize timing information
			SetAuthStaticAddr = 0x29,	///< SETAASA: Assign all slaves static addresses as their dynamic address
			ResetActivity = 0x2A,		///< RSTACT: Reset activity mode
			DefineGroupAddr = 0x2B,		///< DEFGRPA: Define group address
			ResetGroupAddr = 0x2C		///< RSTGRPA: Reset group address
		};

		enum class DirectCCC : uint8_t {
			EnableEvents = 0x80,		///< ENEC: Enable salve event driven interrupts on target
			DisableEvents = 0x81,		///< DISEC: Disable salve event driven interrupts on target
			EnterActivityState0 = 0x82,	///< ENTAS0: Set activity mode to 0 (Normal Operation)
			EnterActivityState1 = 0x83,	///< ENTAS1: Set activity mode to 1
			EnterActivityState2 = 0x84,	///< ENTAS2: Set activity mode to 2
			EnterActivityState3 = 0x85,	///< ENTAS3: Set activity mode to 3
			SetDynamicAddrStatic= 0x87,	///< SETDASA: Assign dynamic address to target with a knows static address
			SetNewDynamicAddr = 0x88,	///< SETNEWDA: Assign a new dynamic address to target
			SetMaxWriteLen = 0x89,		///< SETMWL: Set maximum write length in a single command for target
			SetMaxReadLen = 0x8A,		///< SETMRL: Set maximum read length in a single command for target
			GetMaxWriteLen = 0x8B,		///< GETMWL: Get target's maximum write length
			GetMaxReadLen = 0x8C,		///< GETMRL: Get target's maximum read length
			GetProvId = 0x8D,			///< GETPID: Get targets provisioned ID (48-bit Unique ID)
			GetBusCharReg = 0x8E,		///< GETBCR: Get device bus characteristics register (BCR)
			GetDevCharReg = 0x8F,		///< GETDCR: Get device characteristics register (DCR)
			GetStatus = 0x90,			///< GETSTATUS: Read device operating status
			GetAcceptCtrlCaps = 0x91,	///< GETACCCR: Get secondary master capabilities
			GetMaxDataSpeed = 0x94,		///< GETMXDS: Get target SDR Mode maximum data speeds
			GetCaps = 0x95,				///< GETCAPS: Get optional capabilities
			DevToDevTransfer = 0x97,	///< D2DXFER: Initiate device to device transfer
			SetExchangeTime = 0x98,		///< SETXTIME: Set timing info for target
			GetExchangeTime = 0x99,		///< GETXTIME: Get timing info from target
			ResetActivity = 0x9A,		///< RSTACT: Reset activity mode for target
			SetGroupAddr = 0x9B,		///< SETGRPA: Set group address for target
			ResetGroupAddr = 0x9C		///< RSTGRPA: Reset group address for target
		};

		/// @brief I3C peripheral configuration structure.
		struct Config {
			Mode mode;	///< The target mixed-bus speed configuration.
		};

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., I3C1, I3C2).
		I3C(I3C_TypeDef *instance);

		/// @brief Initializes the peripheral clock, the peripheral itself, and interrupts.
		/// @param config I3C configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		/// @brief Assigns dynamic addresses to all I3C devices on the bus (ENTDAA).
		/// @return Number of I3C devices successfully enumerated.
		uint8_t AssignDynamicAddress();

		/// @brief Starts a non-blocking broadcast common command transaction
		/// @param ccc The broadcast common command code.
		/// @param txBuf Pointer to data to write (optional payload for common command)
		/// @param txLen Number of bytes to write (optional payload length).
		/// @return Status::Ok if the command was started, or Status::Busy if the I3C is locked by another thread.
		Status SendCommandAsync(BroadcastCCC ccc, uint8_t *txBuf, uint16_t txLen);

		/// @brief Starts a non-blocking direct common command transaction
		/// @param ccc The direct common command code.
		/// @param addr The 7-bit address of the target
		/// @param buf Pointer to data to write (for SET commands) or buffer to fill (for GET commands).
		/// @param len Length of the payload in bytes.
		/// @return Status::Ok if the command was started, or Status::Busy if the I3C is locked by another thread.
		Status SendCommandAsync(DirectCCC ccc, uint8_t addr, uint8_t *buf, uint16_t len);

		/// @brief Starts a non-blocking transaction (Write, Read, or Write-then-Read).
		/// @note  Uses the hardware Command Queue to ensure atomic Repeated Starts for legacy sensors.
		/// @param addr The 7-bit address (Static for Legacy, Dynamic for I3C).
		/// @param type Specifies if the target is Legacy or I3C (affects framing).
		/// @param txBuf Pointer to data to write (need to be kept until end of transfer).
		/// @param txLen Number of bytes to write.
		/// @param rxBuf Pointer to buffer for read data.
		/// @param rxLen Number of bytes to read.
		/// @return Status::Ok if the transfer started, or Status::Busy if the I3C is locked by another thread.
		Status TransferAsync(uint8_t addr, TargetType type, uint8_t *txBuf, uint16_t txLen, uint8_t *rxBuf, uint16_t rxLen);

		/// @brief Blocks the current thread until the Async transfer completes.
		/// @param timeoutTicks Max wait time in OS ticks.
		/// @return Status::Ok if the transfer completed successfully, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status TransferWait(uint32_t timeoutTicks);

		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler();

	private:
		I3C_TypeDef *instance;
		IRQn_Type irqCall;
		uint8_t irqPriority;

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
		static constexpr uint32_t TIMEOUT_MUTEX = TX_WAIT_FOREVER;

		/// @brief Checks if a Direct CCC requires a Read phase (GET command).
		/// @note  Based on MIPI I3C Spec v1.1 command ranges.
		static constexpr bool IsReadCommand(DirectCCC ccc) {
			uint8_t code = static_cast<uint8_t>(ccc);
			// GET commands are generally in the range 0x8B - 0x95, plus 0x99 (GETXTIME)
			return (code >= 0x8B && code <= 0x95) || (code == 0x99);
		}
};