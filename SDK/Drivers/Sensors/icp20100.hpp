/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Sensors/icp20100.hpp
 * Author:  NotBlackMagic
 * Brief:   ICP-20100 driver class for STM32N6.
 */

 #pragma once

#include <stdint.h>

#include "i2c.hpp"
#include "status.hpp"

class ICP20100 {
	public:
		enum class Register : uint8_t {

		};

		struct Config {

		};

		ICP20100(I2C& i2c, uint8_t addr) : bus(i2c), addr(addr) {};

		Status Init(const Config& config);
		Status Reset();
		Status ReadID(uint8_t& id);

		Status RequestData();
		Status GetData();

	private:
		I2C& bus;
		const uint8_t addr;
		Config config;

		Status WriteRegister(Register reg, uint8_t value);
		Status ReadRegister(Register reg, uint8_t& value);
		Status ModifyRegister(Register reg, uint8_t mask, uint8_t value);
};