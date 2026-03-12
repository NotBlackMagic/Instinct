/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/xspi.hpp
 * Author:  NotBlackMagic
 * Brief:   XSPI driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

#pragma once

#include <stdint.h>

#include "stm32n6xx.h"
#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_gpio.h"

class XSPI {
	public:
		enum class XSPIOperationMode : uint8_t {
			None,
			Single,
			Double,
			Quad,
			Octo,
			Hexadeca
		};

		enum class XSPIOperationSize : uint8_t {
			OSPI_Size_None,
			OSPI_Size_8Bit,
			OSPI_Size_16Bit,
			OSPI_Size_24Bit,
			OSPI_Size_32Bit,
		};

		/// @brief SPI peripheral configuration structure.
		struct Config {
		};

		// Delete copy constructors
		XSPI(const XSPI&) = delete;
		XSPI& operator=(const XSPI&) = delete;

		XSPI(XSPI_TypeDef *instance);

		void Init(const Config &config);
		void Write(XSPIOperationMode mode, uint8_t inst, uint8_t dmmCyc, uint32_t addr, XSPIOperationSize addrSize, uint8_t *data, uint32_t len);
		void Read(XSPIOperationMode mode, uint8_t inst, uint8_t dmmCyc, uint32_t addr, XSPIOperationSize addrSize, uint8_t *data, uint32_t len);
	private:
		XSPI_TypeDef *instance;
};