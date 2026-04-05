/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/exti.cpp
 */


#include "exti.hpp"

EXTIManager::CallbackEntry EXTIManager::extiCallbackTable[16];

Status EXTIManager::RegisterCallback(uint8_t line, EXTICallback callback, void* context) {
	if(line >= 16 || callback == nullptr) {
		return Status::Error;
	}

	uint32_t primaskState = __get_PRIMASK();
	__disable_irq();

	if(extiCallbackTable[line].callback != nullptr) {
		__set_PRIMASK(primaskState);
        return Status::Error;
	}
	
	extiCallbackTable[line].callback = callback;
	extiCallbackTable[line].context = context;

	__set_PRIMASK(primaskState);

	return Status::Ok;
}

Status EXTIManager::UnregisterCallback(uint8_t line) {
	if(line >= 16) {
		return Status::Error;
	}

	uint32_t primaskState = __get_PRIMASK();
	__disable_irq();

	extiCallbackTable[line].callback = nullptr;
	extiCallbackTable[line].context = nullptr;
	
	__set_PRIMASK(primaskState);

	return Status::Ok;
}

void EXTIManager::Dispatch(uint8_t line, Edge edge) {
	if(line < 16 && extiCallbackTable[line].callback != nullptr) {
		extiCallbackTable[line].callback(extiCallbackTable[line].context, edge);
	}
}