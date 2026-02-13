#include "BoardInfo.hpp"
#include "hardware.hpp"
#include "logger.hpp"
#include "shell.hpp"
#include "version.hpp"

#include "system_stm32n6xx.h"
#include "tx_thread.h"

extern char _stext[], _etext[];
extern char _sdata[], _edata[];
extern char _sbss[],  _ebss[];
extern char _estack[];

//Basic commands

static bool CommandHelp(const char* args) {
	(void)args;
	Logger::Instance().Write("--- Available Commands ---\r\n");
	//Iterate through the list of commands regsitered
	const Shell::CommandList* currentList = Shell::GetRegistry();
	while(currentList != nullptr) {
		//Iterate through commands in each list (module)
		const Shell::CommandEntry* entry = currentList->commandGroup;

		while(entry->name != nullptr) {
			char buffer[80];
			snprintf(buffer, sizeof(buffer), "  %-12s - %s\r\n", entry->name, (entry->helpText) ? entry->helpText : "");

			Logger::Instance().Write(buffer);
			entry++;	//Go to next command in this list/array
		}

		currentList = currentList->next;
	}
	Logger::Instance().Write("--------------------------\r\n");
	return true;
	/*
	(void)args;
	Logger::Instance().Write("Available Commands:\r\n");
	//Iterate table to auto-generate help!
	//(Note: We can't access private _commands easily unless we expose it or use a getter. 
	//For simplicity here, we hardcode the knowns or make _commands public/friend).
	Logger::Instance().Write("  help     - Show list\r\n");
	Logger::Instance().Write("  reboot   - Soft Reset\r\n");
	Logger::Instance().Write("  status   - System Stats\r\n");
	Logger::Instance().Write("  clear    - Clear Screen\r\n");
	return true;
	*/
}

static bool CommandClear(const char* args) {
	(void)args;
	//ANSI Code to clear screen and move cursor home
	Logger::Instance().Write("\033[2J\033[H");
	return true;
}

//System status commands
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
	Logger::Instance().Printf("CPU: STM32N6 @ 800MHz\r\n");
	Logger::Instance().Printf("Tick: %lu ms\r\n", HAL_GetTick());
	//TBD: add Battery Voltage, Stack usage, etc.
	return true;
}

static bool CommandMemory(const char* args) {
	(void)args;

	// Calculate Usage
	uint32_t romUsed = (uint32_t)(_etext - _stext) + (uint32_t)(_edata - _sdata);
	uint32_t ramUsed = (uint32_t)(_edata - _sdata) + (uint32_t)(_ebss - _sbss);
	uint32_t dmaUsed = (uint32_t)(__enoncacheable - __snoncacheable);

	// Dynamic memory stuff
	uint32_t stackTotal = (uint32_t)_estack - (uint32_t)_ebss;
	uint32_t stackUsed  = (uint32_t)_estack - __get_MSP();

	// Calculate Percentages
	int romPct = (romUsed * 100) / BoardInfo::SizeROM;
	int ramPct = (ramUsed * 100) / BoardInfo::SizeRAM;
	int stackPct = (stackTotal > 0) ? (stackUsed * 100) / stackTotal : 0;

	// Print
	Logger::Instance().Write("\r\n--- MEMORY USAGE ---\r\n");
	// Internal Memory
	Logger::Instance().Printf("ROM    : %7lu / %8lu B (%d%%)\r\n", romUsed, BoardInfo::SizeROM, romPct);
	Logger::Instance().Printf("RAM    : %7lu / %8lu B (%d%%)\r\n", ramUsed, BoardInfo::SizeRAM, ramPct);
	// External Memory
	Logger::Instance().Printf("H-FLASH: %8lu / %8lu B (%d%%)\r\n", 0UL, BoardInfo::SizeExtFlash, 0);
	Logger::Instance().Printf("PSRAM  : %8lu / %8lu B (%d%%)\r\n", 0UL, BoardInfo::SizeExtRAM, 0);
	// Dynamic Memory
	Logger::Instance().Printf("STACK  : %8lu / %8lu B (%d%%)\r\n", stackUsed, stackTotal, stackPct);
	Logger::Instance().Write("--------------------\r\n");

	return true;
}

//Helper to decode numeric states into human-readable strings
static const char* GetThreadState(UINT state) {
	switch (state) {
		case TX_READY:             return "READY";
		case TX_COMPLETED:         return "DONE";
		case TX_TERMINATED:        return "DEAD";
		case TX_SUSPENDED:         return "SUSP";
		case TX_SLEEP:             return "SLEEP";
		case TX_QUEUE_SUSP:        return "QUEUE";
		case TX_SEMAPHORE_SUSP:    return "SEM";
		case TX_EVENT_FLAG:        return "FLAG";
		case TX_BLOCK_MEMORY:      return "MEM";
		case TX_BYTE_MEMORY:       return "BYTE";
		case TX_IO_DRIVER:         return "IO";
		case TX_FILE:              return "FILE";
		case TX_TCP_IP:            return "NET";
		case TX_MUTEX_SUSP:        return "MUTEX";
		default:                   return "????";
	}
}

static bool CommandRTOS(const char* args) {
	(void)args;
	Logger::Instance().Write("------------------------------------------------------------\r\n");
	Logger::Instance().Write("NAME         STATE   PRIO   STACK (Used/Max)   RUN COUNT\r\n");
	Logger::Instance().Write("------------------------------------------------------------\r\n");

	UINT state = tx_interrupt_control(TX_INT_DISABLE);
	TX_THREAD* thread = _tx_thread_created_ptr;
	tx_interrupt_control(state);

	if (thread == nullptr) {
		Logger::Instance().Write("No threads found!\r\n");
		return true;
	}

	//Iterate through thread list
	TX_THREAD* startThread = thread;
    char buffer[128];

	do {		
		uint32_t stackSize = thread->tx_thread_stack_size;
		uint8_t* deepPtr = (uint8_t*)thread->tx_thread_stack_highest_ptr;	//'highest_ptr' is the lowest address the stack pointer has ever reached (High Watermark).
		uint8_t* startPtr = (uint8_t*)thread->tx_thread_stack_start;		//'start' is the lowest address (limit). 
		
		//Bytes used is the distance from the bottom of the stack to the deepest point
		uint32_t used = (uint32_t)(deepPtr - startPtr);
		
		uint32_t usagePercent = 0;
		if (stackSize > 0) {
			usagePercent = (used * 100) / stackSize;
		}

		// Format the line
		snprintf(buffer, sizeof(buffer), "%-12.12s %-7s %2d     %4lu / %4lu (%2lu%%)   %lu\r\n",
			thread->tx_thread_name ? thread->tx_thread_name : "???",
			GetThreadState(thread->tx_thread_state),
			thread->tx_thread_priority,
			(unsigned long)used,
			(unsigned long)stackSize,
			(unsigned long)usagePercent,
			(unsigned long)thread->tx_thread_run_count
		);

		Logger::Instance().Write(buffer);

		//Move to next thread
		thread = thread->tx_thread_created_next;

	} while (thread != startThread);

	Logger::Instance().Write("------------------------------------------------------------\r\n");
	return true;
}

//Control commands
static bool CommandReboot(const char* args) {
	(void)args;
	Logger::Instance().Write("Rebooting...\r\n");
	// NVIC_SystemReset();
	return true;
}

static bool CommandLog(const char* args) {
	int level = atoi(args);
	if (level >= 0 && level <= 6) {
		Logger::Instance().SetConsoleLevel((Logger::LogLevel)level);
		Logger::Instance().Printf("Log Level set to %d\r\n", level);
	}
	else {
		Logger::Instance().Write("Usage: log <0-6>\r\n");
	}
	return true;
}

//Hardware debug commands
static bool CommandGPIO(const char* args) {
	if (args == nullptr || args[0] == '\0') {
		Logger::Instance().Write("Usage: gpio <read|write|toggle> <port> <pin> [val]\r\n");
		return true;
	}
	return true;
}

static I2C* GetI2CBus(uint8_t busID) {
	switch(busID){
		case 1:
			return &i2c1;
		case 2:
			return &i2c2;
		case 3:
			return nullptr;
		case 4:
			return &i2c4;
		default:
			return nullptr;
	}
}

static void ScanI2CBus(uint8_t busID, I2C* i2c) {
	Logger::Instance().Printf("Scanning I2C%d...\r\n", busID);
	Logger::Instance().Write("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");

	for (int i = 0; i < 128; i += 16) {
		Logger::Instance().Printf("%02x: ", i);

		for (int j = 0; j < 16; j++) {
			uint16_t addr = i + j;

			//Skip reserved addresses
			if(addr < 0x03 || addr > 0x77) {
					Logger::Instance().Write("   "); 
					continue;
			}

			if(i2c->Probe(addr)) {
				Logger::Instance().Printf("%02x ", addr); 
			}
			else {
				Logger::Instance().Write("-- "); 
			}
		}
		Logger::Instance().Write("\r\n");
	}
	// dataLen = sprintf((char*)data, "I2C4 | 0 1 2 3 4 5 6 7 8 9 A B C D E F \n\r");
	// while(uart4.Write(data, dataLen) != 0x00);

	// uint8_t addr;
	// for(addr = 0; addr < 128; addr++) {
	// 	if((addr % 16) == 0) {
	// 		dataLen = sprintf((char*)data, "0x%02X | ", addr);
	// 		while(uart4.Write(data, dataLen) != 0x00);
	// 	}
	// 	if(i2c4.Probe(addr) == 0x01) {
	// 		dataLen = sprintf((char*)data, "Y ");
	// 		while(uart4.Write(data, dataLen) != 0x00);
	// 	}
	// 	else {
	// 		dataLen = sprintf((char*)data, "- ");
	// 		while(uart4.Write(data, dataLen) != 0x00);
	// 	}
	// 	if((addr % 16) == 15) {
	// 		dataLen = sprintf((char*)data, "\n\r");
	// 		while(uart4.Write(data, dataLen) != 0x00);
	// 	}
	// }
}

static bool CommandI2C(const char* args) {
	if (args == nullptr || args[0] == '\0') {
		Logger::Instance().Write("Usage: i2c scan [bus_num]\r\n");
		return true;
	}

	char mode[10];
    int busNum = -1;
	uint8_t count = sscanf(args, "%9s %d", mode, &busNum);
	if (count < 1 || strcmp(mode, "scan") != 0) {
		Logger::Instance().Write("Usage: i2c scan [bus_num]\r\n");
		return true;
	}

	if (busNum != -1) {
        // Case A: User asked for specific bus (e.g., "i2c scan 2")
        I2C* bus = GetI2CBus(busNum);
        if (bus) {
            ScanI2CBus(busNum, bus);
        } else {
            Logger::Instance().Printf("Error: I2C%d not defined.\r\n", busNum);
        }
    }

	return true;
}

//SYSTEM COMMANDS
static const Shell::CommandEntry systemCommands[] {
	//Basic commands
	{ "help",	CommandHelp,	"Lists commands" },
	{ "?",	CommandHelp,	"Lists commands" },
	{ "clear",CommandClear,	"Clear terminal" },

	//System status commands
	{"info",		CommandInfo, "Board & FW info" },
	{ "version",	CommandVersion,"Firmware info" },
	{ "status",	CommandStatus,	"System stats" },
	{ "ps",		CommandRTOS,	"Thread status" },
	{ "mem",		CommandMemory,	"Memory usage" },

	//Control commands
	{ "reboot",	CommandReboot,		"Reboots system" },
	{ "log",		CommandLog,	"Set Log Level (0-6)" },

	//Hardware debug commands
	{ "gpio",	CommandGPIO,	"GPIO Read/Write" },
	{ "i2c",	CommandI2C,	"I2C Bus Scan" },
	
	{ nullptr,	nullptr,		nullptr } // Terminator
};

//Static memory for the node
static Shell::CommandList systemShellNode;

void RegisterSystemCommands() {
    Shell::RegisterCommands(&systemShellNode, systemCommands); //Register the table defined below
}