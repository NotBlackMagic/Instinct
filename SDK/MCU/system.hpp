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

struct Pmu {
	// Delete constructor.
	Pmu() = delete;

	/// @brief Maximum number of 32-bit chained counter pairs.
	static constexpr uint8_t maxChainedPairs = 4;

	/// @brief Cortex-M55 PMU event identifiers.
	enum class Event : uint16_t {
		SWIncr = 0x0000,			///< Instruction architecturally executed, condition code check pass, software increment.
		L1ICacheRefill = 0x0001,	///< L1 instruction cache linefill.
		L1DCacheRefill = 0x0003,	///< L1 data cache linefill.
		L1DCache = 0x0004,			///< L1 data cache access.
		LdRetired = 0x0006,			///< Instruction architecturally executed, condition code check pass, load.
		StRetired = 0x0007,			///< Instruction architecturally executed, condition code check pass, store.
		InstRetired = 0x0008,		///< Instruction architecturally executed.
		ExcTaken = 0x0009,			///< Exception taken.
		ExcReturn = 0x000A,			///< Instruction architecturally executed, condition code check pass, exception return.
		PCWriteRetired = 0x000C,	///< Instruction architecturally executed, condition code check pass, software change of the PC.
		BrImmedRetired = 0x000D,	///< Instruction architecturally executed, immediate branch.
		BrReturnRetired = 0x000E,	///< Instruction architecturally executed, condition code check pass, procedure return.
		UnalignedLDSTRetired = 0x000F,	///< Instruction architecturally executed, condition code check pass, unaligned load or store.
		CPUCycles = 0x0011,			///< Cycle.
		MemAccess = 0x0013,			///< Data memory access.
		L1ICache = 0x0014,			///< L1 instruction cache access.
		L1DCacheWB = 0x0015,		///< L1 data cache write-back.
		BusAccess = 0x0019,			///< Any beat access to the M-AXI read interface, M-AXI write interface and any access to P-AHB interface.
		MemError = 0x001A,			///< ECC error for TCMs and caches.
		BusCycles = 0x001D,			///< Counts the number of cycles on which the M-AXI interface is clocked.
		Chain = 0x001E,				///< For an odd-numbered counter, increments when an overflow occurs on the preceding even-numbered counter on the same PE.
		BrRetired = 0x0021,			///< Instruction architecturally executed, branch.
		BrMisPredRetired = 0x022,	///< Instruction architecturally executed, mispredicted branch.
		StallFrontend = 0x0023,		///< If there are no instructions available from the fetch stage of the processor pipeline, the processor considers the front-end of the processor pipeline as being stalled.
		StallBackend = 0x0024,		///< If there is an instruction available from the fetch stage of the pipeline but it cannot be accepted by the decode stage of the processor pipeline, the processor considers the back-end of the processor pipeline as being stalled.
		LLChacheRd = 0x0036,		///< L1 data cache read. For the Cortex®-M55 processor, this event is the same as L1_CACHE_RD.
		LLCacheMissRd = 0x0037,		///< L1 data cache read miss. For the Cortex®-M55 processor, this event is the same as L1D_CACHE_MISS_RD.
		L1dCacheMissRd = 0x0039,	///< L1 data cache read miss. For the Cortex®-M55 processor, this event is the same as LL_CACHE_MISS_RD.
		Stall = 0x003C,				///< No operation sent for execution.
		L1DCacheRd = 0x0040,		///< L1 data cache read. For the Cortex®-M55 processor, this event is the same as LL_CACHE_RD.
		LERetired = 0x0100,			///< Loop end instruction architecturally executed, entry registered in the LO_BRANCH_INFO cache.
		LECancel = 0x0108,			///< LO_BRANCH_INFO cache containing a valid loop entry cleared while not in the last iteration of the loop.
		SECallS = 0x0114,			///< Call to secure function, resulting in security state change.
		SECallNS = 0x0115,			///< Call to Non-secure function, resulting in security state change.
		DWTCmpMatch0 = 0x0118,		///< Data Watchpoint and Trace (DWT) comparator 0 match.
		DWTCmpMatch1 = 0x0119,		///< DWT comparator 1 match.
		DWTCmpMatch2 = 0x011A,		///< DWT comparator 2 match.
		DWTCmpMatch3 = 0x011B,		///< DWT comparator 3 match.
		DWTCmpMatch4 = 0x011C,		///< DWT comparator 4 match.
		DWTCmpMatch5 = 0x011D,		///< DWT comparator 5 match.
		DWTCmpMatch6 = 0x011E,		///< DWT comparator 6 match.
		DWTCmpMatch7 = 0x011F,		///< DWT comparator 7 match.
		MVEInstRetired = 0x0200,	///< M‑profile Vector Extension (MVE) instruction architecturally executed.
		MVEFPRetired = 0x0204,		///< MVE floating-point instruction architecturally executed.
		MVEFPHPRetired = 0x0208,	///< MVE half-precision floating-point instruction architecturally executed.
		MVEFPSPRetired = 0x020C,	///< MVE single-precision floating-point instruction architecturally executed.
		MVEFPMACRetired = 0x0214,	///< MVE floating-point multiply or multiply accumulate instruction architecturally executed.
		MVEIntRetired = 0x0224,		///< MVE integer instruction architecturally executed.
		MVEIntMAXRetired = 0x0228,	///< MVE integer multiply or multiply-accumulate instruction architecturally executed.
		MVELdStRetired = 0x0238,	///< MVE load or store instruction architecturally executed.
		MVELdRetired = 0x023C,		///< MVE load instruction architecturally executed.
		MVEStRetired = 0x0240,		///< MVE store instruction architecturally executed.
		MVELdStContigRetired = 0x0244,	///< MVE contiguous load or store instruction architecturally executed.
		MVELdContigRetired = 0x0248,	///< MVE contiguous load instruction architecturally executed.
		MVEStContigRetired = 0x024C,	///< MVE contiguous store instruction architecturally executed.
		MVELdStNonContigRetired = 0x0250,	///< MVE non-contiguous load or store instruction architecturally executed.
		MVELdNonContigRetired = 0x0254,	///< MVE non-contiguous load instruction architecturally executed.
		MVEStNonContigRetired = 0x0258,	///< MVE non-contiguous store instruction architecturally executed.
		MVELdStMultiRetired = 0x025C,	///< MVE memory instruction targeting multiple registers architecturally executed.
		MVELdMultiRetired = 0x0260,	///< MVE memory load instruction targeting multiple registers architecturally executed.
		MVEStMultiRetired = 0x0264,	///< MVE memory store instruction targeting multiple registers architecturally executed.
		MVELdStUnalignedRetired = 0x028C,	///< MVE unaligned memory load or store instruction architecturally executed.
		MVELdUnalignedRetired = 0x0290,	///< MVE unaligned load instruction architecturally executed.
		MVEStUnalignedRetired = 0x0294,	///< MVE unaligned store instruction architecturally executed.
		MVELdStUnalignedNonContigRetired = 0x0298,	///< MVE unaligned non-contiguous load or store instruction architecturally executed.
		MVEVReduceRetired = 0x02A0,	///< MVE vector reduction instruction architecturally executed.
		MVEVReduceFPRetired = 0x02A4,	///< MVE floating-point vector reduction instruction architecturally executed.
		MVEVReduceIntRetired = 0x02A8,	///< MVE integer vector reduction instruction architecturally executed.
		MVEPred = 0x02B8,			///< Cycles where one or more predicated beats architecturally executed.
		MVEStall = 0x02CC,			///< Stall cycles caused by an MVE instruction.
		MVEStallResource = 0x02CD,	///< Stall cycles caused by an MVE instruction because of resource conflicts.
		MVEStallResourceMem = 0x02CE,	///< Resource conflicts.
		MVEStallResourceFP = 0x02CF,	///< Stall cycles caused by an MVE instruction because of floating-point resource conflicts.
		MVEStallResourceInt = 0x02D0,	///< Stall cycles caused by an MVE instruction because of integer resource conflicts.
		MVEStallBreak = 0x02D3,		///< Stall cycles caused by an MVE chain break.
		MVEStallDependency = 0x02D4, 	///< Stall cycles caused by MVE register dependency.
		ITCMAccess = 0x4007,		///< Instruction Tightly Coupled Memory (ITCM) access.
		DTCMAccess = 0x4008,		///< Data Tightly Coupled Memory (ITCM) access.

		AXIWriteAccess = 0xC302,	///< Any beat access to M-AXI write interface.
		AXIReadAccess = 0xC303,		///< Any beat access to M-AXI read interface.
		DoSTimeoutDouble = 0xC400,	///< Denial of Service timeout has fired twice and caused buffers to drain to allow forward progress.
		DoSTimeoutTriple = 0xC401	///< Denial of Service timeout has fired three times and blocked the LSU to force forward progress.
	};

	/// @brief Initializes DWT and PMU extension registers.
	static void Init();

	/// @brief Starts PMU cycle and event counting.
	static void Start();

	/// @brief Stops PMU counting.
	static void Stop();

	/// @brief Resets all counters and overflow flags.
	static void Reset();

	/// @brief Configures a single 16-bit event counter (0 to 7).
	/// @param index Counter index to configure.
	/// @param event Event to count.
	static void ConfigureCounter(uint8_t index, Event event);

	/// @brief Configures a 32-bit chained counter pair (0 to 3).
	/// @param index Index 0..3 (pairs counter 2*i and 2*i+1 with CHAIN).
	/// @param event Event to count.
	static void ConfigureChained(uint8_t index, Event event);

	/// @brief One-liner configuration for all four 32-bit chained counter pairs.
	/// @param event0 Event 0 to count.
	/// @param event1 Event 1 to count.
	/// @param event2 Event 2 to count.
	/// @param event3 Event 3 to count.
	static void ConfigureChained(Event event0, Event event1, Event event2, Event event3);

	/// @brief Reads the 32-bit cycle counter (PMU_CCNTR / DWT_CYCCNT).
	/// @return Cycle count (DWT_CYCCNT).
	static uint32_t GetCycles();

	/// @brief Reads a single 16-bit counter value.
	/// @param index Counter index to read.
	/// @return Event counter value.
	static uint16_t ReadCounter(uint8_t index);

	/// @brief Reads a 32-bit chained counter pair with carry-safe double sampling.
	/// @param index Counter index to read.
	/// @return Event counter value.
	static uint32_t ReadChained(uint8_t index);
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
	/// @param count: Number of NOPs to execute.
	static void DelayNOP(uint32_t count);
};