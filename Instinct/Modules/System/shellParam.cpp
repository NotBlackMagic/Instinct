/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/System/shellParam.cpp
 * Author:	NotBlackMagic
 * Brief:	Shell commands for parameter inspection, runtime tuning, and storage.
 */

#include <stdlib.h>
#include <string.h>

#include "paramService.hpp"
#include "shellModules.hpp"

static void PrintHelp() {
	Logger::Instance().Write("--- Parameter Commands ---\r\n");
	Logger::Instance().Write("  param list [filter]        - List parameters (optional substring/prefix filter)\r\n");
	Logger::Instance().Write("  param get <name>           - Display detailed info and value of a parameter\r\n");
	Logger::Instance().Write("  param set <name> <val>     - Set parameter value (auto-converts float or int)\r\n");
	Logger::Instance().Write("  param reset                - Reset all parameters to factory defaults\r\n");
	Logger::Instance().Write("  param lock [0|1]           - Query or set Protected parameter lock\r\n");
	Logger::Instance().Write("  param info                 - Show summary statistics and registry status\r\n");
	Logger::Instance().Write("--------------------------\r\n");
}

static void FormatFlags(uint8_t flags, char* outStr, size_t maxLen) {
	if(maxLen < 5) {
		return;
	}
	outStr[0] = (flags & ParamFlags::ReadOnly) ? 'R' : '-';
	outStr[1] = (flags & ParamFlags::Protected) ? 'P' : '-';
	outStr[2] = (flags & ParamFlags::Volatile) ? 'V' : '-';
	outStr[3] = '\0';
}

static void PrintParamRow(uint16_t id, const ParamService::ParamEntry* entry) {
	if(entry == nullptr || entry->valPtr == nullptr) {
		return;
	}

	char flagBuf[8];
	FormatFlags(entry->flags, flagBuf, sizeof(flagBuf));

	char valBuf[16];
	char defBuf[16];
	char rangeBuf[16];

	if(entry->valType == ParamType::Float) {
		float currentVal = *static_cast<const float*>(entry->valPtr);
		snprintf(valBuf, sizeof(valBuf), "%.4f", currentVal);
		snprintf(defBuf, sizeof(defBuf), "%.3f", entry->defaultVal.fVal);
		snprintf(rangeBuf, sizeof(rangeBuf), "%.2f..%.2f", entry->minVal.fVal, entry->maxVal.fVal);
	}
	else {
		int32_t currentVal = *static_cast<const int32_t*>(entry->valPtr);
		snprintf(valBuf, sizeof(valBuf), "%ld", currentVal);
		snprintf(defBuf, sizeof(defBuf), "%ld", entry->defaultVal.iVal);
		snprintf(rangeBuf, sizeof(rangeBuf), "%ld..%ld", entry->minVal.iVal, entry->maxVal.iVal);
	}

	char buffer[128];
	snprintf(buffer, sizeof(buffer), " %-4u %-20.20s %-12.12s %-12.12s %-13.13s %-5s\r\n",
		id,
		entry->name,
		valBuf,
		defBuf,
		rangeBuf,
		flagBuf
	);

	Logger::Instance().Write(buffer);
}

static bool CommandParam(const char* args) {
	if(args == nullptr || args[0] == '\0') {
		PrintHelp();
		return true;
	}

	char action[16] = {0};
	// Parse target action
	int32_t count = sscanf(args, "%15s", action);
	if(count < 1) {
		PrintHelp();
		return true;
	}

	if(strcmp(action, "list") == 0) {
		char filter[17] = {0};
		bool hasFilter = (sscanf(args, "%*s %16s", filter) >= 1);

		Logger::Instance().Write("------------------------------------------------------------------------\r\n");
		Logger::Instance().Write(" ID   NAME                 VALUE        DEFAULT      RANGE         FLAGS\r\n");
		Logger::Instance().Write("------------------------------------------------------------------------\r\n");

		const ParamService::ParamList* currentList = ParamService::GetParamList();
		if(currentList == nullptr) {
			Logger::Instance().Write(" No parameters registered.\r\n");
			Logger::Instance().Write("------------------------------------------------------------------------\r\n");
			return true;
		}

		uint16_t matchedCount = 0;
		while(currentList != nullptr) {
			const ParamService::ParamEntry* entry = currentList->paramGroup;
			uint16_t localIdx = 0;

			while(entry->name != nullptr) {
				if(!hasFilter || strstr(entry->name, filter) != nullptr) {
					uint16_t globalIdx = currentList->baseParamID + localIdx;
					PrintParamRow(globalIdx, entry);
					matchedCount++;
				}
				entry++;
				localIdx++;
			}
			currentList = currentList->next;
		}

		if(matchedCount == 0) {
			Logger::Instance().Write(" No matching parameters found.\r\n");
		}

		Logger::Instance().Write("------------------------------------------------------------------------\r\n");
		return true;
	}

	if(strcmp(action, "get") == 0) {
		char paramName[17] = {0};
		if(sscanf(args, "%*s %16s", paramName) < 1) {
			Logger::Instance().Write("Usage: param get <name>\r\n");
			return true;
		}

		ParamService::ParamList* ownerList = nullptr;
		const ParamService::ParamEntry* entry = ParamService::FindByName(paramName, &ownerList);
		if(entry == nullptr || ownerList == nullptr) {
			Logger::Instance().Printf("Error: Parameter '%s' not found.\r\n", paramName);
			return true;
		}

		uint16_t globalIdx = ownerList->baseParamID + static_cast<uint16_t>(entry - ownerList->paramGroup);

		Logger::Instance().Write("\r\n--- Parameter Details ---\r\n");
		Logger::Instance().Printf("  Name    : %s\r\n", entry->name);
		Logger::Instance().Printf("  Index   : %u\r\n", globalIdx);
		Logger::Instance().Printf("  Hash    : 0x%08lX\r\n", entry->hash);
		Logger::Instance().Printf("  Type    : %s\r\n", (entry->valType == ParamType::Float) ? "Float" : "Int32");

		char flagBuf[8];
		FormatFlags(entry->flags, flagBuf, sizeof(flagBuf));
		Logger::Instance().Printf("  Flags   : [%s]\r\n", flagBuf);

		if(entry->valType == ParamType::Float) {
			Logger::Instance().Printf("  Value   : %f\r\n", *static_cast<const float*>(entry->valPtr));
			Logger::Instance().Printf("  Default : %f\r\n", entry->defaultVal.fVal);
			Logger::Instance().Printf("  Range   : [%.3f, %.3f]\r\n", entry->minVal.fVal, entry->maxVal.fVal);
		}
		else {
			Logger::Instance().Printf("  Value   : %ld\r\n", *static_cast<const int32_t*>(entry->valPtr));
			Logger::Instance().Printf("  Default : %ld\r\n", entry->defaultVal.iVal);
			Logger::Instance().Printf("  Range   : [%ld, %ld]\r\n", entry->minVal.iVal, entry->maxVal.iVal);
		}
		Logger::Instance().Write("-------------------------\r\n");
		return true;
	}

	if(strcmp(action, "set") == 0) {
		char paramName[17] = {0};
		char valStr[32] = {0};

		if(sscanf(args, "%*s %16s %31s", paramName, valStr) < 2) {
			Logger::Instance().Write("Usage: param set <name> <val>\r\n");
			return true;
		}

		ParamService::ParamList* ownerList = nullptr;
		const ParamService::ParamEntry* entry = ParamService::FindByName(paramName, &ownerList);
		if(entry == nullptr || ownerList == nullptr) {
			Logger::Instance().Printf("Error: Parameter '%s' not found.\r\n", paramName);
			return true;
		}

		char* endPtr = nullptr;
		bool success = false;

		if(entry->valType == ParamType::Float) {
			float fVal = strtof(valStr, &endPtr);
			if(endPtr == valStr) {
				Logger::Instance().Printf("Error: '%s' is not a valid float.\r\n", valStr);
				return true;
			}
			float oldVal = *static_cast<const float*>(entry->valPtr);
			success = ParamService::Set(ownerList, entry, fVal, true);
			if(success) {
				Logger::Instance().Printf("Param %s: %.4f -> %.4f [OK]\r\n", entry->name, oldVal, fVal);
			}
		}
		else {
			int32_t iVal = static_cast<int32_t>(strtol(valStr, &endPtr, 0));
			if(endPtr == valStr) {
				Logger::Instance().Printf("Error: '%s' is not a valid integer.\r\n", valStr);
				return true;
			}
			int32_t oldVal = *static_cast<const int32_t*>(entry->valPtr);
			success = ParamService::Set(ownerList, entry, iVal, true);
			if(success) {
				Logger::Instance().Printf("Param %s: %ld -> %ld [OK]\r\n", entry->name, oldVal, iVal);
			}
		}

		if(!success) {
			Logger::Instance().Printf("Failed to set '%s' (check bounds or locks).\r\n", entry->name);
		}
		return true;
	}

	if(strcmp(action, "reset") == 0) {
		ParamService::Reset();
		Logger::Instance().Write("All parameters reset to factory defaults.\r\n");
		return true;
	}

	if(strcmp(action, "lock") == 0) {
		int32_t lockVal = -1;
		if(sscanf(args, "%*s %d", &lockVal) >= 1) {
			ParamService::SetProtectedLock(lockVal > 0);
			Logger::Instance().Printf("Protected parameters: %s\r\n", (lockVal > 0) ? "LOCKED" : "UNLOCKED");
		}
		else {
			Logger::Instance().Write("Usage: param lock <0|1>\r\n");
		}
		return true;
	}

	if(strcmp(action, "info") == 0) {
		uint16_t groupCount = 0;
		const ParamService::ParamList* curr = ParamService::GetParamList();
		while(curr != nullptr) {
			groupCount++;
			curr = curr->next;
		}

		Logger::Instance().Write("\r\n--- PARAMETER REGISTRY INFO ---\r\n");
		Logger::Instance().Printf("  Total Parameters : %u\r\n", ParamService::GetTotalParamCount());
		Logger::Instance().Printf("  Registered Groups: %u\r\n", groupCount);
		Logger::Instance().Write("-------------------------------\r\n");
		return true;
	}

	Logger::Instance().Printf("Unknown parameter action '%s'.\r\n", action);
	PrintHelp();
	return true;
}

static const Shell::CommandEntry paramCommands[] = {
	{ "param", CommandParam, "Inspect, tune, and reset system parameters" },
	{ nullptr, nullptr, nullptr }
};

static Shell::CommandList paramShellNode;

void RegisterParamCommands() {
	Shell::RegisterCommands(&paramShellNode, paramCommands);
}