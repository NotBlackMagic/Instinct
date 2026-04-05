 /*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/exti.hpp
 * Author:  NotBlackMagic
 * Brief:   EXTI driver class (EXTI interrupt manager) for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_exti.h"

#include "status.hpp"

class EXTIManager {
	public:
		/// @brief EXTI interrupt triggered edge.
		enum class Edge {
			Rising,
			Falling,
			Unknown
		};

		/// @brief EXTI event callback function template.
		typedef void (*EXTICallback)(void* context, Edge edge);

		// Delete constructor, and copy constructors.
		EXTIManager() = delete;
		EXTIManager(const EXTIManager&) = delete;
		EXTIManager& operator=(const EXTIManager&) = delete;

		/// @brief Register/add a callback to a EXTI trigger line, only one callback per EXTI line allowed.
		/// @param line		EXTI line to assign callback function to.
		/// @param callback	Pointer to callback function.
		/// @param context	Pointer to callback context.
		/// @return Status::Ok if was successfully regsitered, Status::Error for invalid arguments or already has callback on EXTI line.
		static Status RegisterCallback(uint8_t line, EXTICallback callback, void* context);
		
		/// @brief Unregister/remove a callback from a EXTI trigger line.
		/// @param line		EXTI line to unregister/remove callback function from.
		/// @return Status::Ok if was successfully unregistered, Status::Error for invalid argument.
		static Status UnregisterCallback(uint8_t line);
		
		/// @brief Dispatcher function for EXTI trigger handling i.e. Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		static void Dispatch(uint8_t line, Edge edge);

	private:
		struct CallbackEntry {
			EXTICallback callback = nullptr;
			void* context = nullptr;
		};

		static CallbackEntry extiCallbackTable[16];
};