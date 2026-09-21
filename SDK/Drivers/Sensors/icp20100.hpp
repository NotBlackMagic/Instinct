/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	SDK/Drivers/Sensors/icp20100.hpp
 * Author:	NotBlackMagic
 * Brief:	ICP-20100 driver class for STM32N6.
 */

#pragma once

#include <stdint.h>

#include "i2c.hpp"
#include "status.hpp"

class ICP20100 {
	public:
		// Standard Chip identifications
		static constexpr uint8_t i2cAddrPrimary = 0x63;
		static constexpr uint8_t i2cAddrSecondary = 0x64;
		static constexpr uint8_t chipID = 0x63;

		enum class Register : uint8_t {
			TRIM1_MSB = 0x05,
			TRIM2_LSB = 0x06,
			TRIM2_MSB = 0x07,
			DEVICE_ID = 0x0C,
			IO_DRIVE_STRENGTH = 0x0D,
			OTP_CONFIG1 = 0xAC,
			OTP_MR_LSB = 0xAD,
			OTP_MR_MSB = 0xAE,
			OTP_MRA_LSB = 0xAF,
			OTP_MRA_MSB = 0xB0,
			OTP_MRB_LSB = 0xB1,
			OTP_MRB_MSB = 0xB2,
			OTP_ADDRESS_REG = 0xB5,
			OTP_COMMAND_REG = 0xB6,
			OTP_RDATA = 0xB8,
			OTP_STATUS = 0xB9,
			OTP_DBG2 = 0xBC,
			MASTER_LOCK = 0xBE,
			OTP_STATUS2 = 0xBF,
			MODE_SELECT = 0xC0,
			INTERRUPT_STATUS = 0xC1,
			INTERRUPT_MASK = 0xC2,
			FIFO_CONFIG = 0xC3,
			FIFO_FILL = 0xC4,
			SPI_MODE = 0xC5,
			PRESS_ABS_LSB = 0xC7,
			PRESS_ABS_MSB = 0xC8,
			PRESS_DELTA_LSB = 0xC9,
			PRESS_DELTA_MSB = 0xCA,
			DEVICE_STATUS = 0xCD,
			I3C_INFO = 0xCE,
			VERSION = 0xD3,
			PRESS_DATA_0 = 0xFA,
			PRESS_DATA_1 = 0xFB,
			PRESS_DATA_2 = 0xFC,
			TEMP_DATA_0 = 0xFD,
			TEMP_DATA_1 = 0xFE,
			TEMP_DATA_2 = 0xFF
		};

		enum class OutputDataRate : uint8_t {
			Hz2 = 0x03,
			Hz25 = 0x00,
			Hz40 = 0x02,
			Hz120 = 0x01
		};

		struct Config {
			OutputDataRate odr;
		};

		/// @brief Constructor.
		/// @param i2c	Reference to the low-level bus driver.
		/// @param addr	Device I2C address.
		ICP20100(I2C& i2c, uint8_t addr) : bus(i2c), addr(addr) {};

		/// @brief Initializes the ICP-20100 sensor.
		/// @param config ICP-20100 sensor configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid or failed.
		Status Init(const Config& config);

		/// @brief Resets the sensor device, using software reset.
		/// @return Status::Ok if reset succeeded cleanly.
		Status Reset();

		/// @brief Reads the Manufacturer and Device IDs.
		/// @param id Device ID, or manufacturer ID.
		/// @return Status::Ok if read succeeded, or Status::Error if failed.
		Status ReadID(uint8_t& id);

		/// @brief Sets the barometer/pressure sensor offset value, to be added to the read value.
		/// @param offsetPressure Offset for pressure value.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetOffset(float offsetPressure);

		/// @brief Start a non-blocking data transfer request (calls TransferAsync of the underlying bus).
		/// @details Returns immediately. Use GetData() to synchronize completion.
		/// @return Status::Ok if the transfer started, or Status::Busy if the bus is locked by another thread.
		Status RequestData();

		/// @brief Blocks the current thread until data transfer completes.
		/// @param pressure	Pointer to be filled with received pressure data.
		/// @param temp		Pointer to be filled with received temperature data.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status GetData(float* pressure, float* temp);

	private:
		I2C& bus;
		const uint8_t addr;
		Config config;

		static constexpr uint16_t transferSize = 32;
		__attribute__((aligned(32))) uint8_t buffer[transferSize];

		static constexpr float pressureSens = (40000.0f/131072);
		static constexpr float tempSens = (65.0f/262144);		
		
		float pressureOffset = 70000.0f;
		static constexpr float tempOffset = 25.0f;

		Status WriteRegister(Register reg, uint8_t value);
		Status ReadRegister(Register reg, uint8_t& value);
		Status ModifyRegister(Register reg, uint8_t mask, uint8_t value);
};