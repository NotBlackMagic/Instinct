/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/System/shellCommands.cpp
 */

#include "shellModules.hpp"

// Linker exported symbols
extern "C" {
	// Memory capacities (values mapped directly as symbol addresses)
	extern uint8_t _rom_capacity;
	extern uint8_t _ram_capacity;
	extern uint8_t _itcm_capacity;
	extern uint8_t _dtcm_capacity;
	extern uint8_t _axisram1_capacity;
	extern uint8_t _axisram3_capacity;
	extern uint8_t _axisram4_capacity;
	extern uint8_t _axisram5_capacity;
	extern uint8_t _axisram6_capacity;
	extern uint8_t _ahbsram1_capacity;
	extern uint8_t _ahbsram2_capacity;
	extern uint8_t _vencram_capacity;
	extern uint8_t _bkpsram_capacity;

	// ROM limits
	extern uint8_t _srom, _erom;

	// ITCM bounds
	extern uint8_t _sitcm, _eitcm;

	// DTCM bounds
	extern uint8_t _sdtcm_data, _edtcm_data;
	extern uint8_t _sdtcm_bss, _edtcm_bss;

	// AXISRAM2 bounds (Core RAM)
	extern uint8_t _sdata, _edata;
	extern uint8_t _sbss, _ebss;
	extern uint32_t __snoncacheable, __enoncacheable;
	extern uint8_t _sstack;
	extern uint8_t _estack;

	// Dedicated SRAM bounds
	extern uint8_t _saxisram1, _eaxisram1;
	extern uint8_t _saxisram3, _eaxisram3;
	extern uint8_t _saxisram4, _eaxisram4;
	extern uint8_t _saxisram5, _eaxisram5;
	extern uint8_t _saxisram6, _eaxisram6;
	extern uint8_t _sahbsram1, _eahbsram1;
	extern uint8_t _sahbsram2, _eahbsram2;
	extern uint8_t _svencram, _evencram;
	extern uint8_t _sbkpsram, _ebkpsram;
}

static void FormatBytes(char* out, size_t outSize, uint64_t bytes) {
	if(bytes >= (1024ULL * 1024ULL * 1024ULL)) {
		uint64_t whole = bytes / (1024ULL * 1024ULL * 1024ULL);
		uint64_t tenth = ((bytes % (1024ULL * 1024ULL * 1024ULL)) * 10ULL) / (1024ULL * 1024ULL * 1024ULL);
		snprintf(out, outSize, "%lu.%01lu GB", (unsigned long)whole, (unsigned long)tenth);
	}
	else if(bytes >= (1024ULL * 1024ULL)) {
		uint64_t whole = bytes / (1024ULL * 1024ULL);
		uint64_t tenth = ((bytes % (1024ULL * 1024ULL)) * 10ULL) / (1024ULL * 1024ULL);
		snprintf(out, outSize, "%lu.%01lu MB", (unsigned long)whole, (unsigned long)tenth);
	}
	else if(bytes >= 1024ULL) {
		snprintf(out, outSize, "%lu KB", (unsigned long)(bytes / 1024ULL));
	}
	else {
		snprintf(out, outSize, "%lu B", (unsigned long)bytes);
	}
}

// Basic commands
static bool CommandHelp(const char* args) {
	(void)args;
	Logger::Instance().Write("--- Available Commands ---\r\n");
	// Iterate through the list of commands regsitered
	const Shell::CommandList* currentList = Shell::GetRegistry();
	while(currentList != nullptr) {
		// Iterate through commands in each list (module)
		const Shell::CommandEntry* entry = currentList->commandGroup;

		while(entry->name != nullptr) {
			char buffer[80];
			snprintf(buffer, sizeof(buffer), "  %-14s %s\r\n", entry->name, (entry->helpText) ? entry->helpText : "");

			Logger::Instance().Write(buffer);
			entry++;	// Go to next command in this list/array
		}

		currentList = currentList->next;
	}
	Logger::Instance().Write("--------------------------\r\n");
	return true;
}

static bool CommandClear(const char* args) {
	(void)args;
	// ANSI Code to clear screen and move cursor home
	Logger::Instance().Write("\033[2J\033[H");
	return true;
}

// System status commands
static bool CommandInfo(const char* args) {
	(void)args;
	Logger::Instance().Write("\r\n--- SYSTEM INFORMATION ---\r\n");

	// Hardware information (from Board/BoardInfo.hpp)
	Logger::Instance().Write("[HARDWARE]\r\n");
	Logger::Instance().Printf("  Board:       %s (%s)\r\n", BoardInfo::Name, BoardInfo::Revision);
	Logger::Instance().Printf("  Designed:    %s in %s\r\n", BoardInfo::DesignDate, BoardInfo::DesignLocation);
	Logger::Instance().Printf("  Designer:    %s\r\n", BoardInfo::Designer);

	// Firmware information (From Config/Version.hpp)
	Logger::Instance().Write("[FIRMWARE]\r\n");
	Logger::Instance().Printf("  App Name:    %s\r\n", FW_NAME);
	Logger::Instance().Printf("  Version:     v%s\r\n", FW_VERSION_STR);
	Logger::Instance().Printf("  Build:       %s at %s (%s)\r\n", FW_BUILD_DATE, FW_BUILD_TIME, FW_BUILD_TYPE);
	Logger::Instance().Printf("  Commit:      %s\r\n", FW_GIT_HASH);
	Logger::Instance().Printf("  Compiler:    %s\r\n", FW_COMPILER);

	// Silicon and runtime information
	uint32_t revID = *(uint32_t *)(REVID_BASE);
	uint32_t uid0 = READ_REG(*((uint32_t *)UID_BASE));
	uint32_t uid1 = READ_REG(*((uint32_t *)UID_BASE + 4U));
	uint32_t uid2 = READ_REG(*((uint32_t *)UID_BASE + 8U));

	Logger::Instance().Write("[SILICON]\r\n");
	Logger::Instance().Printf("  Device:      STM32N657 (Rev: 0x%04X)\r\n", revID);
	Logger::Instance().Printf("  UID:         %08lX-%08lX-%08lX\r\n", uid0, uid1, uid2);
	Logger::Instance().Printf("  SysClock:    %lu MHz\r\n", SystemCoreClock / 1000000);

	Logger::Instance().Write("[RTOS]\r\n");
	Logger::Instance().Printf("  Kernel:      %s\r\n", _tx_version_id);

	Logger::Instance().Write("--------------------------\r\n");
	return true;
}

static bool CommandVersion(const char* args) {
	(void)args;
	Logger::Instance().Printf("%s v%s\r\n", FW_NAME, FW_VERSION_STR);
	return true;
}

static bool CommandStatus(const char* args) {
	(void)args;
	// Read temperature using DTS or internal ADC channel
	float coreTempC = 25.0f;	//BoardInfo::ReadJunctionTemperature(); // Implement via DTS / ADC

	Logger::Instance().Printf("CPU     : STM32N657 @ %lu MHz (Tj: %.1f C)\r\n", SystemCoreClock / 1000000, coreTempC);
	Logger::Instance().Printf("Uptime  : %lu s (Tick: %lu ms)\r\n", HAL_GetTick() / 1000, HAL_GetTick());
	
	// Battery & safety critical telemetry
	// Logger::Instance().Printf("Battery : %.2f V | Current: %.2f A\r\n", PowerMonitor::GetVoltage(), PowerMonitor::GetCurrent());
	return true;
}

constexpr uint32_t stackWatermarkPattern = 0xEFEEFEEF;

static void PrintMemoryLine(const char* label, uint32_t used, uint32_t total) {
	char usedStr[16];
	char totalStr[16];
	FormatBytes(usedStr, sizeof(usedStr), used);
	FormatBytes(totalStr, sizeof(totalStr), total);
	int pct = (total > 0) ? static_cast<int>((used * 100ULL) / total) : 0;
	Logger::Instance().Printf(" %-10s: %9s / %9s (%3d%%)\r\n", label, usedStr, totalStr, pct);
}

static bool CommandMemory(const char* args) {
	(void)args;

	// Retrieve absolute capacities from the linker script
	uint32_t romTotal = reinterpret_cast<uintptr_t>(&_rom_capacity);
	uint32_t ramTotal = reinterpret_cast<uintptr_t>(&_ram_capacity);
	uint32_t itcmTotal = reinterpret_cast<uintptr_t>(&_itcm_capacity);
	uint32_t dtcmTotal = reinterpret_cast<uintptr_t>(&_dtcm_capacity);

	// Calculate Usage
	// ROM usage (executable + constants + all TCM/RAM LMA images)
	uint32_t romUsed = reinterpret_cast<uintptr_t>(&_erom) - reinterpret_cast<uintptr_t>(&_srom);

	// ITCM usage (instructions)
	uint32_t itcmUsed = reinterpret_cast<uintptr_t>(&_eitcm) - reinterpret_cast<uintptr_t>(&_sitcm);

	// DTCM usage (.dtcm_data + .dtcm_bss)
	uint32_t dtcmDataUsed = reinterpret_cast<uintptr_t>(&_edtcm_data) - reinterpret_cast<uintptr_t>(&_sdtcm_data);
	uint32_t dtcmBssUsed = reinterpret_cast<uintptr_t>(&_edtcm_bss) - reinterpret_cast<uintptr_t>(&_sdtcm_bss);
	uint32_t dtcmUsed = dtcmDataUsed + dtcmBssUsed;

	// AXISRAM2 usage
	uint32_t dmaUsed = reinterpret_cast<uintptr_t>(&__enoncacheable) - reinterpret_cast<uintptr_t>(&__snoncacheable);
	uint32_t ramUsed = (reinterpret_cast<uintptr_t>(&_edata) - reinterpret_cast<uintptr_t>(&_sdata)) +
						(reinterpret_cast<uintptr_t>(&_ebss) - reinterpret_cast<uintptr_t>(&_sbss)) +
						dmaUsed;

	// Scan MSP for watermark pattern (0xEFEEFEEF) from bottom (_sstack) to top (_estack)
	constexpr uint32_t stackWatermarkPattern = 0xEFEEFEEF;
	uint32_t* stackFloor = reinterpret_cast<uint32_t*>(&_sstack);
	uint32_t* stackCeiling = reinterpret_cast<uint32_t*>(&_estack);
	uint32_t stackTotal = reinterpret_cast<uintptr_t>(stackCeiling) - reinterpret_cast<uintptr_t>(stackFloor);

	uint32_t* scanPtr = stackFloor;
	while(scanPtr < stackCeiling && *scanPtr == stackWatermarkPattern) {
		scanPtr++;
	}
	// High-water mark represents the deepest point MSP has reached
	uint32_t stackPeakUsed = reinterpret_cast<uintptr_t>(stackCeiling) - reinterpret_cast<uintptr_t>(scanPtr);

	// Dedicated AXI SRAMs Usage & Capacities
	uint32_t axisram1Used = reinterpret_cast<uintptr_t>(&_eaxisram1) - reinterpret_cast<uintptr_t>(&_saxisram1);
	uint32_t axisram3Used = reinterpret_cast<uintptr_t>(&_eaxisram3) - reinterpret_cast<uintptr_t>(&_saxisram3);
	uint32_t axisram4Used = reinterpret_cast<uintptr_t>(&_eaxisram4) - reinterpret_cast<uintptr_t>(&_saxisram4);
	uint32_t axisram5Used = reinterpret_cast<uintptr_t>(&_eaxisram5) - reinterpret_cast<uintptr_t>(&_saxisram5);
	uint32_t axisram6Used = reinterpret_cast<uintptr_t>(&_eaxisram6) - reinterpret_cast<uintptr_t>(&_saxisram6);

	uint32_t axisram1Total = reinterpret_cast<uintptr_t>(&_axisram1_capacity);
	uint32_t axisram3Total = reinterpret_cast<uintptr_t>(&_axisram3_capacity);
	uint32_t axisram4Total = reinterpret_cast<uintptr_t>(&_axisram4_capacity);
	uint32_t axisram5Total = reinterpret_cast<uintptr_t>(&_axisram5_capacity);
	uint32_t axisram6Total = reinterpret_cast<uintptr_t>(&_axisram6_capacity);

	// Peripheral SRAMs
	uint32_t ahbsram1Used = reinterpret_cast<uintptr_t>(&_eahbsram1) - reinterpret_cast<uintptr_t>(&_sahbsram1);
	uint32_t ahbsram2Used = reinterpret_cast<uintptr_t>(&_eahbsram2) - reinterpret_cast<uintptr_t>(&_sahbsram2);
	uint32_t vencramUsed = reinterpret_cast<uintptr_t>(&_evencram) - reinterpret_cast<uintptr_t>(&_svencram);
	uint32_t bkpsramUsed = reinterpret_cast<uintptr_t>(&_ebkpsram) - reinterpret_cast<uintptr_t>(&_sbkpsram);

	uint32_t ahbsram1Total = reinterpret_cast<uintptr_t>(&_ahbsram1_capacity);
	uint32_t ahbsram2Total = reinterpret_cast<uintptr_t>(&_ahbsram2_capacity);
	uint32_t vencramTotal = reinterpret_cast<uintptr_t>(&_vencram_capacity);
	uint32_t bkpsramTotal = reinterpret_cast<uintptr_t>(&_bkpsram_capacity);

	// External memory stuff
	uint32_t psramUsed = externalPSRAM.GetUsedSize();
	uint32_t psramTotal = externalPSRAM.GetTotalSize();

	// Print
	Logger::Instance().Write("\r\n--- SYSTEM MEMORY ---\r\n");
	PrintMemoryLine("ROM", romUsed, romTotal);
	PrintMemoryLine("ITCM", itcmUsed, itcmTotal);
	PrintMemoryLine("DTCM", dtcmUsed, dtcmTotal);
	PrintMemoryLine("RAM (AXI2)", ramUsed, ramTotal);
	PrintMemoryLine("MSP STACK", stackPeakUsed, stackTotal);

	Logger::Instance().Write("\r\n--- AXI SRAM POOLS ---\r\n");
	PrintMemoryLine("AXISRAM1", axisram1Used, axisram1Total);
	PrintMemoryLine("AXISRAM3", axisram3Used, axisram3Total);
	PrintMemoryLine("AXISRAM4", axisram4Used, axisram4Total);
	PrintMemoryLine("AXISRAM5", axisram5Used, axisram5Total);
	PrintMemoryLine("AXISRAM6", axisram6Used, axisram6Total);

	Logger::Instance().Write("\r\n--- PERIPHERAL / EXT ---\r\n");
	PrintMemoryLine("AHBSRAM1", ahbsram1Used, ahbsram1Total);
	PrintMemoryLine("AHBSRAM2", ahbsram2Used, ahbsram2Total);
	PrintMemoryLine("VENCRAM", vencramUsed, vencramTotal);
	PrintMemoryLine("BKPSRAM", bkpsramUsed, bkpsramTotal);
	PrintMemoryLine("H-FLASH", 0UL, BoardInfo::SizeExtFlash);
	PrintMemoryLine("PSRAM", psramUsed, psramTotal);
	Logger::Instance().Write("-----------------------\r\n");

	return true;
}

static bool CommandFaults(const char* args) {
	if(args != nullptr && strcmp(args, "clear") == 0) {
		// Clear sticky fault bits by writing 1s to CFSR
		SCB->CFSR = 0xFFFFFFFF;
		SCB->HFSR = 0xFFFFFFFF;
		Logger::Instance().Write("Fault status registers cleared.\r\n");
		return true;
	}

	uint32_t cfsr = SCB->CFSR;
	uint32_t hfsr = SCB->HFSR;
	uint32_t mmfar = SCB->MMFAR;
	uint32_t bfar = SCB->BFAR;
	uint32_t abfsr = SCB->AFSR; // Cortex-M55 specific AXI/AHB fault status

	Logger::Instance().Write("\r\n--- FAULT STATUS REGISTERS ---\r\n");
	Logger::Instance().Printf(" CFSR  : 0x%08lX\r\n", (unsigned long)cfsr);
	Logger::Instance().Printf(" HFSR  : 0x%08lX\r\n", (unsigned long)hfsr);
	Logger::Instance().Printf(" ABFSR : 0x%08lX (Cortex-M55 Bus Fault Source)\r\n", (unsigned long)abfsr);

	// MemManage Faults (CFSR[7:0])
	uint8_t mmsr = static_cast<uint8_t>(cfsr & 0xFF);
	if(mmsr != 0) {
		Logger::Instance().Printf(" [!] MemManage Fault (0x%02X):", mmsr);
		if(mmsr & (1 << 0)) {
			Logger::Instance().Write(" IACCVIOL");
		}
		if(mmsr & (1 << 1)) {
			Logger::Instance().Write(" DACCVIOL");
		}
		if(mmsr & (1 << 3)) {
			Logger::Instance().Write(" MUNSTKERR");
		}
		if(mmsr & (1 << 4)) {
			Logger::Instance().Write(" MSTKERR");
		}
		if(mmsr & (1 << 7)) {
			Logger::Instance().Printf(" | Fault Address: 0x%08lX", (unsigned long)mmfar);
		}
		Logger::Instance().Write("\r\n");
	}

	// Bus Faults (CFSR[15:8])
	uint8_t bfsr = static_cast<uint8_t>((cfsr >> 8) & 0xFF);
	if(bfsr != 0) {
		Logger::Instance().Printf(" [!] Bus Fault (0x%02X):", bfsr);
		if(bfsr & (1 << 0)) {
			Logger::Instance().Write(" IBUSERR");
		}
		if(bfsr & (1 << 1)) {
			Logger::Instance().Write(" PRECISERR");
		}
		if(bfsr & (1 << 2)) {
			Logger::Instance().Write(" IMPRECISERR");
		}
		if(bfsr & (1 << 3)) {
			Logger::Instance().Write(" UNSTKERR");
		}
		if(bfsr & (1 << 4)) {
			Logger::Instance().Write(" STKERR");
		}
		if(bfsr & (1 << 7)) {
			Logger::Instance().Printf(" | Fault Address: 0x%08lX", (unsigned long)bfar);
		}
		Logger::Instance().Write("\r\n");
	}

	// Usage Faults (CFSR[31:16])
	uint16_t ufsr = static_cast<uint16_t>((cfsr >> 16) & 0xFFFF);
	if(ufsr != 0) {
		Logger::Instance().Printf(" [!] Usage Fault (0x%04X):", ufsr);
		if(ufsr & (1 << 0)) {
			Logger::Instance().Write(" UNDEFINSTR");
		}
		if(ufsr & (1 << 1)) {
			Logger::Instance().Write(" INVSTATE");
		}
		if(ufsr & (1 << 2)) {
			Logger::Instance().Write(" INVPC");
		}
		if(ufsr & (1 << 3)) {
			Logger::Instance().Write(" NOCP");
		}
		if(ufsr & (1 << 8)) {
			Logger::Instance().Write(" UNALIGNED");
		}
		if(ufsr & (1 << 9)) {
			Logger::Instance().Write(" DIVBYZERO");
		}
		Logger::Instance().Write("\r\n");
	}

	// Cortex-M55 Bus Interfaces (ABFSR)
	if(abfsr != 0) {
		if(abfsr & (1 << 0)) {
			Logger::Instance().Write(" [!] AXI Master (AXIM) asynchronous bus error detected\r\n");
		}
		if(abfsr & (1 << 1)) {
			Logger::Instance().Write(" [!] Peripheral Bus (AHBP) asynchronous bus error detected\r\n");
		}
		if(abfsr & (1 << 2)) {
			Logger::Instance().Write(" [!] External Private Bus (EPB) error detected\r\n");
		}
	}

	if(cfsr == 0 && hfsr == 0 && abfsr == 0) {
		Logger::Instance().Write(" No active or sticky hardware faults detected.\r\n");
	}

	Logger::Instance().Write("------------------------------\r\n");
	return true;
}

// Helper to decode numeric states into human-readable strings
static const char* GetThreadState(UINT state) {
	switch (state) {
		case TX_READY:
			return "READY";
		case TX_COMPLETED:
			return "DONE";
		case TX_TERMINATED:
			return "DEAD";
		case TX_SUSPENDED:
			return "SUSP";
		case TX_SLEEP:
			return "SLEEP";
		case TX_QUEUE_SUSP:
			return "QUEUE";
		case TX_SEMAPHORE_SUSP:
			return "SEM";
		case TX_EVENT_FLAG:
			return "FLAG";
		case TX_BLOCK_MEMORY:
			return "MEM";
		case TX_BYTE_MEMORY:
			return "BYTE";
		case TX_IO_DRIVER:
			return "IO";
		case TX_FILE:
			return "FILE";
		case TX_TCP_IP:
			return "NET";
		case TX_MUTEX_SUSP:
			return "MUTEX";
		default:
			return "????";
	}
}

#ifdef TX_EXECUTION_PROFILE_ENABLE
// ThreadX global variables for system profiling
extern ULONG64 _tx_execution_idle_time_total;
extern ULONG64 _tx_execution_isr_time_total;
#endif

static bool CommandRTOS(const char* args) {
	uint32_t sampleWindowMs = 100;
	if(args != nullptr && strlen(args) > 0) {
		uint32_t parsed = static_cast<uint32_t>(atoi(args));
		if(parsed >= 20 && parsed <= 5000) {
			sampleWindowMs = parsed;
		}
	}

	Logger::Instance().Write("--------------------------------------------------------------------\r\n");
	Logger::Instance().Write("NAME         STATE   PRIO     STACK (Used/Max)   RUN COUNT   CPU %\r\n");
	Logger::Instance().Write("--------------------------------------------------------------------\r\n");

#ifdef TX_EXECUTION_PROFILE_ENABLE
	struct ThreadSnapshot {
		TX_THREAD* thread;
		ULONG64 execTime;
	};

	constexpr size_t maxSnapshots = 32;
	ThreadSnapshot snapshots[maxSnapshots];
	size_t threadCount = 0;

	// Snapshot start values
	UINT intState = tx_interrupt_control(TX_INT_DISABLE);
	ULONG64 idleStart = _tx_execution_idle_time_total;
	ULONG64 isrStart = _tx_execution_isr_time_total;

	TX_THREAD* thread = _tx_thread_created_ptr;
	if(thread != nullptr) {
		TX_THREAD* curr = thread;
		do {
			if(threadCount < maxSnapshots) {
				snapshots[threadCount].thread = curr;
				snapshots[threadCount].execTime = curr->tx_thread_execution_time_total;
				threadCount++;
			}
			curr = curr->tx_thread_created_next;
		} while (curr != thread);
	}
	tx_interrupt_control(intState);

	// Window delay (converts ms to ThreadX ticks)
	ULONG sleepTicks = (sampleWindowMs * TX_TIMER_TICKS_PER_SECOND) / 1000;
	if(sleepTicks == 0) {
		sleepTicks = 1;
	}
	tx_thread_sleep(sleepTicks);

	// Snapshot end values
	intState = tx_interrupt_control(TX_INT_DISABLE);
	ULONG64 idleEnd = _tx_execution_idle_time_total;
	ULONG64 isrEnd = _tx_execution_isr_time_total;
	thread = _tx_thread_created_ptr;
	tx_interrupt_control(intState);

	if(thread == nullptr) {
		Logger::Instance().Write("No threads found!\r\n");
		return true;
	}

	// Delta total execution time over the window
	ULONG64 deltaIdle = (idleEnd >= idleStart) ? (idleEnd - idleStart) : 0;
	ULONG64 deltaIsr = (isrEnd >= isrStart) ? (isrEnd - isrStart) : 0;
	ULONG64 totalDelta = deltaIdle + deltaIsr;

	for(size_t i = 0; i < threadCount; ++i) {
		ULONG64 currentExec = snapshots[i].thread->tx_thread_execution_time_total;
		ULONG64 dExec = (currentExec >= snapshots[i].execTime) ? (currentExec - snapshots[i].execTime) : 0;
		totalDelta += dExec;
	}
#else
	UINT intState = tx_interrupt_control(TX_INT_DISABLE);
	TX_THREAD* thread = _tx_thread_created_ptr;
	tx_interrupt_control(intState);

	if(thread == nullptr) {
		Logger::Instance().Write("No threads found!\r\n");
		return true;
	}
#endif

	//Iterate through thread list
	TX_THREAD* startThread = thread;
	char buffer[128];

	do {
		uint32_t stackSize = thread->tx_thread_stack_size;
		uint8_t* deepPtr = (uint8_t*)thread->tx_thread_stack_highest_ptr;	//'highest_ptr' is the lowest address the stack pointer has ever reached (High Watermark).
		uint8_t* endPtr = (uint8_t*)thread->tx_thread_stack_end;			//'end' is the XXX address (limit).
		
		// Bytes used from watermark to stack end
		uint32_t used = 0;
		if(deepPtr != nullptr && endPtr >= deepPtr) {
			used = (uint32_t)(endPtr - deepPtr);
			if(used > stackSize) {
				used = stackSize;
			}
		}
		
		uint32_t usagePercent = 0;
		if(stackSize > 0) {
			usagePercent = (used * 100) / stackSize;
		}

		// Calculate CPU Percentage for this specific thread
		uint32_t cpuPercent = 0;
#ifdef TX_EXECUTION_PROFILE_ENABLE
		if(totalDelta > 0) {
			ULONG64 threadDelta = 0;
			for(size_t i = 0; i < threadCount; ++i) {
				if(snapshots[i].thread == thread) {
					ULONG64 currentExec = thread->tx_thread_execution_time_total;
					threadDelta = (currentExec >= snapshots[i].execTime) ? (currentExec - snapshots[i].execTime) : 0;
					break;
				}
			}
			cpuPercent = (uint32_t)((threadDelta * 1000) / totalDelta);
		}
#endif

		// Format the line
		snprintf(buffer, sizeof(buffer), "%-12.12s %-7s %2d/%2d     %5lu / %5lu (%2lu%%)   %-9lu   %2lu.%01lu%%\r\n",
			thread->tx_thread_name ? thread->tx_thread_name : "???",
			GetThreadState(thread->tx_thread_state),
			thread->tx_thread_priority,
			thread->tx_thread_preempt_threshold,
			(unsigned long)used,
			(unsigned long)stackSize,
			(unsigned long)usagePercent,
			(unsigned long)thread->tx_thread_run_count,
			(unsigned long)(cpuPercent / 10), 
			(unsigned long)(cpuPercent % 10)
		);

		Logger::Instance().Write(buffer);

		//Move to next thread
		thread = thread->tx_thread_created_next;

	} while(thread != startThread);

#ifdef TX_EXECUTION_PROFILE_ENABLE
	uint32_t idlePercent = (totalDelta > 0) ? (uint32_t)((deltaIdle * 1000) / totalDelta) : 0;
	uint32_t isrPercent = (totalDelta > 0) ? (uint32_t)((deltaIsr * 1000) / totalDelta) : 0;

	Logger::Instance().Write("--------------------------------------------------------------------\r\n");
	snprintf(buffer, sizeof(buffer), "System Idle: %2lu.%01lu%%  |  Hardware ISRs: %2lu.%01lu%%\r\n",
			(unsigned long)(idlePercent / 10), (unsigned long)(idlePercent % 10),
			(unsigned long)(isrPercent / 10), (unsigned long)(isrPercent % 10));
	Logger::Instance().Write(buffer);
#endif

	Logger::Instance().Write("--------------------------------------------------------------------\r\n");
	return true;
}

// Control commands
static bool CommandReboot(const char* args) {
	(void)args;
	Logger::Instance().Write("Rebooting...\r\n");
	// NVIC_SystemReset();
	return true;
}

static bool CommandLog(const char* args) {
	if(args == nullptr) {
		Logger::Instance().Write("Usage: log <0-6>\r\n");
		return true;
	}
	int level = atoi(args);
	if(level >= 0 && level <= 6) {
		Logger::Instance().SetConsoleLevel((Logger::LogLevel)level);
		Logger::Instance().Printf("Log Level set to %d\r\n", level);
	}
	else {
		Logger::Instance().Write("Usage: log <0-6>\r\n");
	}
	return true;
}

static bool CommandRamTest(const char* args) {
	(void)args;
	Logger::Instance().Write("Starting PSRAM Benchmark...\r\n");
	MemoryTest::RunBenchmark();
	return true;
}

static bool CommandPerf(const char* args) {
	uint32_t totalWindowMs = 500;
	if(args != nullptr && strlen(args) > 0) {
		uint32_t val = (uint32_t)atoi(args);
		if(val >= 20 && val <= 10000) {
			totalWindowMs = val;
		}
	}

	uint32_t phaseMs = totalWindowMs / 2;

	Pmu::Init();
	Logger::Instance().Printf("Profiling AXI & CPU over %lu ms (2x %lu ms phases)...\r\n", totalWindowMs, phaseMs);

	// PHASE 1: Core Pipeline & Caches (Inst, Stalls, D-Cache, I-Cache)
	Pmu::Stop();
	Pmu::Reset();
	Pmu::ConfigureChained(
		Pmu::Event::InstRetired,
		Pmu::Event::StallBackend,
		Pmu::Event::L1DCacheRefill,
		Pmu::Event::L1ICacheRefill
	);

	Pmu::Start();
	uint32_t p1StartCycles = Pmu::GetCycles();
	uint32_t p1StartInst = Pmu::ReadChained(0);
	uint32_t p1StartStall = Pmu::ReadChained(1);
	uint32_t p1StartDCache = Pmu::ReadChained(2);
	uint32_t p1StartICache = Pmu::ReadChained(3);

	Time::Delay(phaseMs);

	Pmu::Stop();
	uint32_t p1Cycles = Pmu::GetCycles() - p1StartCycles;
	uint32_t deltaInst = Pmu::ReadChained(0) - p1StartInst;
	uint32_t deltaStalls = Pmu::ReadChained(1) - p1StartStall;
	uint32_t deltaDCache = Pmu::ReadChained(2) - p1StartDCache;
	uint32_t deltaICache = Pmu::ReadChained(3) - p1StartICache;

	// PHASE 2: Interconnect & Bus Contention (AXI Read, Write, Bus, DoS)
	Pmu::Reset();
	Pmu::ConfigureChained(
		Pmu::Event::AXIReadAccess,
		Pmu::Event::AXIWriteAccess,
		Pmu::Event::BusAccess,
		Pmu::Event::DoSTimeoutDouble
	);

	Pmu::Start();
	uint32_t p2StartRd = Pmu::ReadChained(0);
	uint32_t p2StartWr = Pmu::ReadChained(1);
	uint32_t p2StartBus = Pmu::ReadChained(2);
	uint32_t p2StartDos = Pmu::ReadChained(3);

	Time::Delay(phaseMs);

	Pmu::Stop();
	uint32_t deltaRd = Pmu::ReadChained(0) - p2StartRd;
	uint32_t deltaWr = Pmu::ReadChained(1) - p2StartWr;
	uint32_t deltaBus = Pmu::ReadChained(2) - p2StartBus;
	uint32_t deltaDos = Pmu::ReadChained(3) - p2StartDos;

	if(p1Cycles == 0) {
		Logger::Instance().Write("PMU cycle counter failed to run.\r\n");
		return true;
	}

	// Calculations
	uint32_t ipcx100 = (uint32_t)(((uint64_t)deltaInst * 100) / p1Cycles);
	uint32_t stallPercent = (uint32_t)(((uint64_t)deltaStalls * 100) / p1Cycles);
	uint32_t stallPerMiss = (deltaDCache > 0) ? (deltaStalls / deltaDCache) : 0;

	// Compact terminal output
	Logger::Instance().Printf("\r\n--- PERF REPORT (%lu ms) ---\r\n", totalWindowMs);
	Logger::Instance().Printf("IPC           : %lu.%02lu  (%lu%% stall)\r\n", ipcx100 / 100, ipcx100 % 100, stallPercent);
	Logger::Instance().Printf("I-Cache Miss  : %lu  [XIP HyperFlash]\r\n", deltaICache);
	Logger::Instance().Printf("D-Cache Miss  : %lu  (Avg: %lu cyc/miss)\r\n", deltaDCache, stallPerMiss);
	Logger::Instance().Printf("AXI Beats     : Rd: %lu | Wr: %lu | Total: %lu\r\n", deltaRd, deltaWr, deltaBus);
	Logger::Instance().Printf("DoS Timeouts  : %lu\r\n", deltaDos);

	// Simple single-line warnings
	if(deltaDos > 0) {
		Logger::Instance().Printf(" [!] AXI Starvation: %lu DoS timeouts (bus arbiter blocking core)\r\n", deltaDos);
	}
	if(stallPerMiss > 40) {
		Logger::Instance().Printf(" [!] High Miss Latency: %lu cyc (nominal: 12-18 cyc)\r\n", stallPerMiss);
	}
	if(deltaICache > 100000) {
		Logger::Instance().Printf(" [!] XIP Contention: High I-cache refills over 8-bit bus\r\n");
	}
	Logger::Instance().Write("----------------------------\r\n");

	return true;
}

static bool CommandDMA(const char* args) {
	if(args != nullptr && strcmp(args, "reset") == 0) {
		DMAChannel* channel = DMAChannel::GetFirstChannel();
		while(channel != nullptr) {
			channel->ResetStatistics();
			channel = channel->GetNextChannel();
		}
		Logger::Instance().Write("All DMA channel statistics reset.\r\n");
		return true;
	}

	Logger::Instance().Write("\r\n------------------------------------------------------------------------------------------\r\n");
	Logger::Instance().Write("NAME         TYPE    CH   TRANSFERS    DTE (BUS)  USE (CFG)  OVERRUN   ABORTS  DATA           \r\n");
	Logger::Instance().Write("------------------------------------------------------------------------------------------\r\n");

	DMAChannel* channel = DMAChannel::GetFirstChannel();
	if(channel == nullptr) {
		Logger::Instance().Write("No registered DMA channels found.\r\n");
		return true;
	}

	uint32_t totalTransfers = 0;
	uint32_t totalBusErrors = 0;
	uint32_t totalSettingErrors = 0;

	char buffer[128];
	char bytesStr[16];

	while(channel != nullptr) {
		DMAChannel::Statistics stats = channel->GetStatistics();
		totalTransfers += stats.transfers;
		totalBusErrors += stats.transferErrors;
		totalSettingErrors += stats.settingErrors;

		FormatBytes(bytesStr, sizeof(bytesStr), stats.bytesTransferred);

		const char* instName = (channel->GetInstance() == HPDMA1) ? "HPDMA1" : "GPDMA1";

		snprintf(buffer, sizeof(buffer), "%-12.12s %-7s %-4u %-12lu %-10lu %-10lu %-9lu %-7lu %-12s\r\n",
			channel->GetName(),
			instName,
			channel->GetChannelIndex(),
			(unsigned long)stats.transfers,
			(unsigned long)stats.transferErrors,
			(unsigned long)stats.settingErrors,
			(unsigned long)stats.triggerOverruns,
			(unsigned long)stats.userAborts,
			bytesStr
		);
		Logger::Instance().Write(buffer);

		channel = channel->GetNextChannel();
	}

	Logger::Instance().Write("------------------------------------------------------------------------------------------\r\n");

	// Diagnostic warnings for AXI bus faults
	if(totalBusErrors > 0) {
		Logger::Instance().Printf(" [!] DTE Errors Detected: %lu transfers failed due to AXI/AHB slave error or RIF lockout.\r\n", 
			(unsigned long)totalBusErrors);
	}
	if(totalSettingErrors > 0) {
		Logger::Instance().Printf(" [!] USE Errors Detected: %lu configuration errors (check burst 1KB boundaries).\r\n", 
			(unsigned long)totalSettingErrors);
	}

	return true;
}

// SYSTEM COMMANDS
static const Shell::CommandEntry systemCommands[] {
	// Basic commands
	{ "help",	CommandHelp,	"Lists commands" },
	{ "?",	CommandHelp,	"Lists commands" },
	{ "clear",CommandClear,	"Clear terminal" },

	// System status commands
	{ "info",		CommandInfo,	"Board & FW info" },
	{ "version",	CommandVersion,"Firmware info" },
	{ "status",	CommandStatus,	"System stats" },
	{ "ps",		CommandRTOS,	"Thread status [window_ms]" },
	{ "top",		CommandRTOS,	"Thread status [window_ms]" },
	{ "mem",		CommandMemory,	"Memory usage" },
	{ "perf",		CommandPerf,	"Profile AXI/CPU stalls [dos|read|write|bus|mve]" },
	{ "dma",		CommandDMA,	"DMA channel statistics [reset]" },
	{ "faults",	CommandFaults,	"Dump Cortex-M55/AXI fault registers [clear]" },

	// Control commands
	{ "reboot",	CommandReboot,		"Reboots system" },
	{ "log",		CommandLog,	"Set Log Level (0-6)" },

	// Tests
	{ "ramtest", CommandRamTest, "Benchmark PSRAM (Direct, MM, DMA)" },
	
	{ nullptr,	nullptr,		nullptr } // Terminator
};

// Static memory for the node
static Shell::CommandList systemShellNode;

void RegisterSystemCommands() {
	Shell::RegisterCommands(&systemShellNode, systemCommands); //Register the table defined below
}