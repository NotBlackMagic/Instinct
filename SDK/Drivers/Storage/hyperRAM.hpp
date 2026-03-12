/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Storage/hyperRAM.hpp
 * Author:  NotBlackMagic
 * Brief:   HyperRAM driver class for STM32N6.
 */

#pragma once

#include <stdint.h>

#include "hyperbus.hpp"
#include "status.hpp"
#include "system.hpp"

#include "tx_api.h"

class HyperRAM {
	public:
		// Internal Register Addresses (Register Space)
		// Based on verification of x4 alignment due to DDR and 16-bit bus width
		static constexpr uint32_t REG_ID0 = 0x0000;		// Identification Register 0
		static constexpr uint32_t REG_ID1 = 0x0004;		// Identification Register 1
		static constexpr uint32_t REG_CFG0 = 0x2000;	// Configuration Register 0
		static constexpr uint32_t REG_CFG1 = 0x2004;	// Configuration Register 1

		/// @brief HyperRAM device configuration structure.
		struct Config {
			const char* deviceName;		///< Free text name e.g. "Cypress S27KL0641"
			uint16_t expectedID;		///< Manufacturer ID to check against (0 = Ignore check)

			// Memory array geometry
			uint32_t sizeBytes;			///< Total size in bytes (e.g. 8*1024*1024)
			uint32_t pageSize;			///< Page boundary (e.g. 1024 Bytes)

			// Timing Presets
			uint32_t sourceClockHz;		///< Peripheral source clock frequency in Hz
			uint32_t frequencyHz;       ///< Target Bus Frequency
			uint8_t initialLatency;		///< Clock cycles (latency count)
			bool fixedLatency;			///< True = Force Fixed, False = Variable (Default)
			uint8_t rwRecoveryTime;		///< Additional latency after read/write
			uint32_t refreshRateUs;		///< Max chip select low time (for refresh and only for PSRAM)
			bool writeZeroLatency;		///< True if Chip supports 0-latency writes.

			// HyperRAM device configurations
			uint16_t configReg0;		//< Value to write to CFG0
			uint16_t configReg1;		//< Value to write to CFG1
		};

		// Delete copy constructors
		HyperRAM(const HyperRAM&) = delete;
		HyperRAM& operator=(const HyperRAM&) = delete;

		HyperRAM(HyperBus &hyperBus) : bus(hyperBus) {}

		/// @brief Initializes the RAM and the underlying bus.
		/// @param config HyperRAM configuration
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		/// @brief Reads the Manufacturer and Device IDs.
		/// @param manID   Pointer to be filled with manufacturer ID.
		/// @param devType Pointer to be filled with device type.
		/// @return Status::Ok if read succeeded, or Status::Error if failed.
		Status ReadID(uint8_t *manID, uint8_t *devType);

		/// @brief Reads data from RAM (Memory Mapped or Indirect).
		/// @param addr	Target address.
		/// @param data	Pointer to buffer for read data.
		/// @param len	Number of bytes to read.
		/// @return Status::Ok if the read was successful, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status Read(uint32_t addr, uint8_t *buf, uint32_t len);

		/// @brief Writes data to RAM.
		/// @param addr	Target address.
		/// @param data	Pointer to buffer to write.
		/// @param len	Number of bytes to write.
		/// @return Status::Ok if the write was successful, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status Write(uint32_t addr, const uint8_t *buf, uint32_t len);

		/// @brief Reads a device register.
		/// @param wordAddr Word address of the register to be read.
		/// @param value    Pointer to be filled with the register value.
		/// @return Status::Ok if the read was successful, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status ReadRegister(uint32_t wordAddr, uint16_t *value);

		/// @brief Writes to a device register.
		/// @param wordAddr Word address of the register to be written.
		/// @param value    Value to write to the register.
		/// @return Status::Ok if the write was successful, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status WriteRegister(uint32_t wordAddr, uint16_t value);

		/// @brief Enter into memory mapped mode
		/// @return Status::Ok if memory mapped mode entered, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status EnterMemoryMappedMode();

		/// @brief Exit memory mapped mode
		/// @return Status::Ok if memory mapped mode exit, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status ExitMemoryMappedMode();

		/// @brief Gets the physical base address (e.g., 0x90000000).
		/// @return The physical address from the underlying bus.
		uint32_t GetBaseAddr() const { return bus.GetBaseAddr(); }

	private:
		HyperBus& bus;
		Config config;
};