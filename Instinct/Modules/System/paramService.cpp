/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/System/Parameters/paramService.cpp
 */

#include "paramService.hpp"

ParamService::ParamList* ParamService::head = nullptr;
uint16_t ParamService::totalParams = 0;
std::atomic<bool> ParamService::isLocked{false};

// Single topic for SD blackbox logging and telemetry listeners
Topic<ParamUpdateMsg> ParamService::topicParamUpdate("param_update", static_cast<uint8_t>(TopicID::ParamUpdate), 0);

void ParamService::Init() {
	Broker::RegisterTopic(&topicParamUpdate);
}

void ParamService::RegisterParams(ParamList* list, const ParamEntry* array) {
	if(list == nullptr || array == nullptr) {
		return;
	}

	// Initialize all module variables to factory defaults
	uint16_t groupCount = 0;
	const ParamEntry* entry = array;
	while(entry->name != nullptr) {
		if(entry->valPtr != nullptr) {
			if(entry->valType == ParamType::Float) {
				*static_cast<float*>(entry->valPtr) = entry->defaultVal.fVal;
			}
			else {
				*static_cast<int32_t*>(entry->valPtr) = entry->defaultVal.iVal;
			}
		}
		groupCount = groupCount + 1;
		entry = entry + 1;
	}

	list->paramGroup = array;
	list->count = groupCount;
	list->updated.store(false, std::memory_order_relaxed);

	UINT state = tx_interrupt_control(TX_INT_DISABLE);
	list->baseParamID = totalParams;
	totalParams = totalParams + groupCount;
	list->next = head;
	head = list;
	tx_interrupt_control(state);
}

const ParamService::ParamEntry* ParamService::FindByName(const char* name, ParamList** outOwnerList) {
	if(name == nullptr) {
		return nullptr;
	}

	const uint32_t targetHash = HashName(name);
	ParamList* currentList = head;
	while(currentList != nullptr) {
		const ParamEntry* entry = currentList->paramGroup;
		while(entry->name != nullptr) {
			// Fast hash check first, and only strcmp on hash match
			if(entry->hash == targetHash && strcmp(entry->name, name) == 0) {
				if(outOwnerList != nullptr) {
					*outOwnerList = currentList;
				}
				return entry;
			}
			entry = entry + 1;
		}
		currentList = currentList->next;
	}

	return nullptr;
}

const ParamService::ParamEntry* ParamService::FindByIndex(uint16_t id, ParamList** outOwnerList) {
	ParamList* currentList = head;
	while(currentList != nullptr) {
		// Fast range check using auto-assigned sequential IDs
		if(id >= currentList->baseParamID && id < (currentList->baseParamID + currentList->count)) {
			if(outOwnerList != nullptr) {
				*outOwnerList = currentList;
			}
			uint16_t offset = id - currentList->baseParamID;
			return &currentList->paramGroup[offset];
		}
		currentList = currentList->next;
	}

	return nullptr;
}

bool ParamService::Set(ParamList* list, const ParamEntry* entry, float fVal, bool notify) {
	if(list == nullptr || entry == nullptr || entry->valPtr == nullptr) {
		return false;
	}

	if(entry->valType != ParamType::Float) {
		LOG_WARN("Param %s type mismatch (expected Int32, got Float)", entry->name);
		return false;
	}

	if((entry->flags & ParamFlags::ReadOnly) != 0) {
		LOG_WARN("Param %s is read-only", entry->name);
		return false;
	}

	if((entry->flags & ParamFlags::Protected) != 0 && isLocked.load(std::memory_order_relaxed) == true) {
		LOG_WARN("Param %s is protected while system is active", entry->name);
		return false;
	}

	if(fVal < entry->minVal.fVal || fVal > entry->maxVal.fVal) {
		LOG_WARN("Param %s out of bounds [%.3f, %.3f]", entry->name, entry->minVal.fVal, entry->maxVal.fVal);
		return false;
	}

	*static_cast<float*>(entry->valPtr) = fVal;
	list->updated.store(true, std::memory_order_release);

	if(notify == true) {
		ParamUpdateMsg msg{};
		msg.timestamp = Time::GetUs();
		strncpy(msg.name, entry->name, sizeof(msg.name) - 1);
		msg.hash = entry->hash;
		msg.paramIndex = list->baseParamID + static_cast<uint16_t>(entry - list->paramGroup);
		msg.type = static_cast<uint8_t>(ParamType::Float);
		msg.value.fVal = fVal;
		topicParamUpdate.Publish(msg);
	}

	return true;
}

bool ParamService::Set(ParamList* list, const ParamEntry* entry, int32_t iVal, bool notify) {
	if(list == nullptr || entry == nullptr || entry->valPtr == nullptr) {
		return false;
	}

	if(entry->valType != ParamType::Int32) {
		LOG_WARN("Param %s type mismatch (expected Float, got Int32)", entry->name);
		return false;
	}

	if((entry->flags & ParamFlags::ReadOnly) != 0) {
		LOG_WARN("Param %s is read-only", entry->name);
		return false;
	}

	if((entry->flags & ParamFlags::Protected) != 0 && isLocked.load(std::memory_order_relaxed) == true) {
		LOG_WARN("Param %s is protected while system is active", entry->name);
		return false;
	}

	if(iVal < entry->minVal.iVal || iVal > entry->maxVal.iVal) {
		LOG_WARN("Param %s out of bounds [%ld, %ld]", entry->name, entry->minVal.iVal, entry->maxVal.iVal);
		return false;
	}

	*static_cast<int32_t*>(entry->valPtr) = iVal;
	list->updated.store(true, std::memory_order_release);

	if(notify == true) {
		ParamUpdateMsg msg{};
		msg.timestamp = Time::GetUs();
		strncpy(msg.name, entry->name, sizeof(msg.name) - 1);
		msg.hash = entry->hash;
		msg.paramIndex = list->baseParamID + static_cast<uint16_t>(entry - list->paramGroup);
		msg.type = static_cast<uint8_t>(ParamType::Int32);
		msg.value.iVal = iVal;
		topicParamUpdate.Publish(msg);
	}

	return true;
}

bool ParamService::Set(const char* name, float fVal, bool notify) {
	ParamList* ownerList = nullptr;
	const ParamEntry* entry = FindByName(name, &ownerList);
	return Set(ownerList, entry, fVal, notify);
}

bool ParamService::Set(const char* name, int32_t iVal, bool notify) {
	ParamList* ownerList = nullptr;
	const ParamEntry* entry = FindByName(name, &ownerList);
	return Set(ownerList, entry, iVal, notify);
}
void ParamService::Reset() {
	if(isLocked.load(std::memory_order_relaxed) == true) {
		LOG_WARN("Cannot reset parameters while armed");
		return;
	}

	ParamList* currentList = head;
	while(currentList != nullptr) {
		const ParamEntry* entry = currentList->paramGroup;
		while(entry->name != nullptr) {
			if(entry->valPtr != nullptr) {
				if(entry->valType == ParamType::Float) {
					*static_cast<float*>(entry->valPtr) = entry->defaultVal.fVal;
				}
				else {
					*static_cast<int32_t*>(entry->valPtr) = entry->defaultVal.iVal;
				}
			}
			entry = entry + 1;
		}
		currentList->updated.store(true, std::memory_order_release);
		currentList = currentList->next;
	}

	// Publish bulk update notification (0xFFFF)
	ParamUpdateMsg msg{};
	msg.timestamp = Time::GetUs();
	strncpy(msg.name, "RESET_ALL", sizeof(msg.name) - 1);
	msg.hash = 0xFFFFFFFF;
	msg.paramIndex = 0xFFFF;
	topicParamUpdate.Publish(msg);
}