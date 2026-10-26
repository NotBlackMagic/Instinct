/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/System/Parameters/paramService.hpp
 * Author:	NotBlackMagic
 * Brief:	Decentralized parameter registry and PubSub notifier.
 */

#pragma once

#include <atomic>
#include <stdint.h>
#include <string.h>

#include "logger.hpp"
#include "parameters.hpp"
#include "pubSub.hpp"
#include "system.hpp"

enum class ParamType : uint8_t {
	Float = 0,
	Int32 = 1
};

enum ParamFlags : uint8_t {
	None = 0x00,
	ReadOnly = 0x01,	// Cannot be modified at runtime
	Protected = 0x02,	// Locked when system is active/operating (e.g., in-flight or motors active)
	Volatile = 0x04 	// Tuned dynamically (e.g. ML/AutoTune)
};

union ParamValue {
	float fVal;
	int32_t iVal;

	constexpr ParamValue() : fVal(0.0f) {}
	constexpr ParamValue(float f) : fVal(f) {}
	constexpr ParamValue(int32_t i) : iVal(i) {}
};

class ParamService {
	public:
		/// @brief Compile-time 32-bit FNV-1a string hash for O(1) comparisons and flash keys.
		static constexpr uint32_t HashName(const char* str) {
			if(str == nullptr) {
				return 0;
			}
			uint32_t hash = 2166136261u;
			while(*str != '\0') {
				hash ^= static_cast<uint8_t>(*str);
				hash *= 16777619u;
				str++;
			}
			return hash;
		}

		struct ParamEntry {
			const char* name;		// Parameter name (max 16 chars)
			uint32_t hash;			// Compile-time FNV-1a hash
			void* valPtr;			// Direct pointer to module's variable
			ParamValue defaultVal;	// Factory default
			ParamValue minVal;		// Minimum allowed value
			ParamValue maxVal;		// Maximum allowed value
			ParamType valType;		// Float or Int32
			uint8_t flags;			// Bitmask of ParamFlags

			// Sentinel constructor for null-terminated array end: { nullptr }
			constexpr ParamEntry() : name(nullptr), hash(0), valPtr(nullptr), defaultVal(0.0f), minVal(0.0f), maxVal(0.0f), valType(ParamType::Float), flags((ParamFlags::None)) {}

			// Float parameter constructor
			constexpr ParamEntry(const char* name, float* ptr, float defVal, float minVal, float maxVal, uint8_t flags = ParamFlags::None) : name(name), hash(HashName(name)), valPtr(ptr), defaultVal(defVal), minVal(minVal), maxVal(maxVal), valType(ParamType::Float), flags(flags) {}

			// Int32 parameter constructor
			constexpr ParamEntry(const char* name, int32_t* ptr, int32_t defVal, int32_t minVal, int32_t maxVal, uint8_t flags = ParamFlags::None) : name(name), hash(HashName(name)), valPtr(ptr), defaultVal(defVal), minVal(minVal), maxVal(maxVal), valType(ParamType::Int32), flags(flags) {}
		};

		struct ParamList {
			const ParamEntry* paramGroup;
			std::atomic<bool> updated;
			uint16_t baseParamID;			// Auto-assigned base index on registration
			uint16_t count;					// Number of parameters in this group
			ParamList* next;
		};

		/// @brief Registers the global parameter update topic with the Broker.
		static void Init();

		/// @brief Links a module's parameter array into the registry and initializes defaults.
		static void RegisterParams(ParamList* list, const ParamEntry* array);

		/// @brief Lock-free check for a module to see if any of its own parameters changed.
		static bool HasUpdate(ParamList* list) {
			if(list == nullptr || list->updated.load(std::memory_order_relaxed) == false) {
				return false;
			}
			// Atomically consume the flag and acquire memory barrier so all new param writes are visible
			return list->updated.exchange(false, std::memory_order_acquire);
		}

		/// @brief Type-safe setters by name. Rejects type mismatches at runtime.
		static bool Set(const char* name, float fVal, bool notify = true);
		static bool Set(const char* name, int32_t iVal, bool notify = true);

		/// @brief Fast O(1) type-safe setters when holding direct pointers.
		static bool Set(ParamList* list, const ParamEntry* entry, float fVal, bool notify = false);
		static bool Set(ParamList* list, const ParamEntry* entry, int32_t iVal, bool notify = false);

		/// @brief Finds a parameter entry by string name.
		static const ParamEntry* FindByName(const char* name, ParamList** outOwnerList = nullptr);

		/// @brief Finds a parameter entry by sequential index (0 .. TotalCount - 1) for MAVLink/iteration.
		static const ParamEntry* FindByIndex(uint16_t index, ParamList** outOwnerList = nullptr);

		/// @brief Resets all registered parameters across all modules to defaults.
		static void Reset();

		/// @brief Locks or unlocks Protected parameters (call when system transitions to active/operating).
		static void SetProtectedLock(bool locked) { isLocked.store(locked, std::memory_order_relaxed); }

		static uint16_t GetTotalParamCount() { return totalParams; }
		static ParamList* GetParamList() { return head; }

	private:
		static ParamList* head;
		static uint16_t totalParams;
		static std::atomic<bool> isLocked;
		static Topic<ParamUpdateMsg> topicParamUpdate;
};