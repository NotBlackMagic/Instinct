/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	SDK/MCU/system.hpp
 * Author:	NotBlackMagic
 * Brief:	Basic system drivers, for configuring clock tree, SRAM region, cache handling, and basic time keeping.
 */

#pragma once

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_cortex.h"
#include "stm32n6xx_ll_pwr.h"
#include "stm32n6xx_ll_rcc.h"
#include "stm32n6xx_ll_system.h"
#include "stm32n6xx_ll_tim.h"
#include "stm32n6xx_ll_utils.h"

#include "status.hpp"

struct System {
	// Delete constructor.
	System() = delete;

	/// @brief Cortex-M55 L1 Data Cache size in bytes.
	static constexpr uint32_t dCacheSize = 32768;

	/// @brief Cortex-M55 L1 Instruction Cache size in bytes.
	static constexpr uint32_t iCacheSize = 32768;

	/// @brief Cortex-M55 L1 cache line width in bytes (identical for I-Cache and D-Cache).
	static constexpr uint32_t cacheLineSize = 32;

	/// @brief Defines the primary clock distribution nodes in the STM32N6.
	enum class ClockNode {
		SYS,
		AXI,
		AHB,
		APB1,
		APB2,
		APB3,
		APB4,
		APB5,
		IC1,
		IC2,
		IC3,
		IC4,
		IC5,
		IC6,
		IC7,
		IC8,
		IC9,
		IC10,
		IC11,
		IC12,
		IC13,
		IC14,
		IC15,
		IC16,
		IC17,
		IC18,
		IC19,
		IC20,
		Unknown
	};

	/// @brief Defines allowable memory allocations for Cortex-M55 ITCM from FlexMem (AXISRAM1).
	enum class ITCMSize : uint32_t {
		Size64KB = 0x07,
		Size128KB = 0x08,
		Size256KB = 0x09
	};

	/// @brief Defines allowable memory allocations for Cortex-M55 DTCM from FlexMem (AXISRAM1).
	enum class DTCMSize : uint32_t {
		Size128KB = 0x08,
		Size256KB = 0x09
	};

	/// @brief Initializes the overall system clock tree.
	static void InitClock(void);

	static void InitFWClock(void);

	/// @brief Retrieves the current frequency of a specific clock node.
	/// @param node The clock node to query.
	/// @return Frequency in Hz, or 0 if unknown/disabled.
	static uint32_t GetNodeFrequency(ClockNode node);

	/// @brief Initializes the SysTick timer.
	/// @note Typically used by the RTOS.
	static void InitSysTick(void);

	/// @brief Disables the SysTick timer.
	static void DisableSysTick(void);

	/// @brief Partitions FLEXMEM (AXISRAM1) between ITCM, DTCM, and remaining AXI SRAM.
	/// @note Must be called early in boot before the C runtime initializes or relocates sections to TCM/AXISRAM1.
	/// @param itcmSize Desired ITCM allocation size.
	/// @param dtcmSize Desired DTCM allocation size.
	static void ConfigureFLEXMEM(ITCMSize itcmSize, DTCMSize dtcmSize);

	/// @brief Enables all AXISRAM blocks.
	static void EnableAXISRAM(void);

	/// @brief Enables all VENCSRAM blocks.
	static void EnableVENCSRAM(void);

	/// @brief Configures the Resource Isolation Framework (RIF/RISAF) for all internal SRAM banks.
	/// @note Unlocks all Compartment IDs (CIDs 0-3) for unrestricted Read/Write and clears privilege gating.
	static void ConfigureRIF(void);

	/// @brief Enables the debug interface in flash run mode.
	static void EnableDebug(void);

	/// @brief Reads an OTP word from the BSEC shadow registers.
	/// @param wordIndex OTP word number (0 to 383).
	/// @return 32-bit OTP word value.
	static uint32_t ReadOTP(uint32_t wordIndex);

	/// @brief Forces a direct physical hardware eFuse read via BSEC state machine.
	/// @param wordIndex OTP word number.
	/// @param value Reference to receive the 32-bit fuse value.
	/// @return Status::Ok on success, Status::Timeout or Status::Error on failure.
	static Status ReadOTPHardware(uint32_t wordIndex, uint32_t &value);
	
	/// @brief Enables the I-Cache and D-Cache of the MCU.
	static void EnableCache(void);

	/// @brief Disables the I-Cache and D-Cache of the MCU.
	static void DisableCache(void);

	/// @brief Cleans the D-Cache by address.
	/// @note Call before DMA TX.
	/// @param addr Start address.
	/// @param size Size of memory block in bytes.
	static void CleanCache(void* addr, uint32_t size);

	/// @brief Invalidates the D-Cache by address
	/// @note Call after DMA RX.
	/// @param addr Start address.
	/// @param size Size of memory block in bytes.
	static void InvalidateCache(void* addr, uint32_t size);

	/// @brief Resets the MCU.
	static void Reset();
};

struct Time {
	// Delete constructor.
	Time() = delete;

	/// @brief Initializes the basic system timer (TIM5).
	static void Init();

	/// @brief Disables the basic system timer (TIM5).
	static void Disable(void);

	/// @brief Gets the current time since boot in miliseconds.
	static uint32_t GetMs();

	/// @brief Gets the current time since boot in microseconds.
	static uint64_t GetUs();

	/// @brief Delays/blocks for a set amount of time (milliseconds).
	/// @param ms Milliseconds to wait.
	static void Delay(uint32_t ms);

	/// @brief Delays/blocks for a set amount of NOP instructions.
	/// @param count: Number of NOPs to execute
	static void DelayNOP(uint32_t count);
};