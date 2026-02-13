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

class HyperRAM {
	public:
		struct Config {
			const char* deviceName;		///< Free text name e.g. "Cypress S27KL0641"
			uint16_t expectedID;		///< Manufacturer ID to check against (0 = Ignore check)

			// Memory array geometry
			uint32_t sizeBytes;			///< Total size in bytes (e.g. 8*1024*1024)
			uint32_t pageSize;			///< Page boundary (e.g. 1024 Bytes)

			// Timing Presets
			uint32_t frequencyHz;       ///< Target Bus Frequency
			uint8_t initalLatency;		///< Clock cycles (latency count)
			bool fixedLatency;			///< True = Force Fixed, False = Variable (Default)
			uint8_t rwRecoveryTime;		///< Additional latency after read/write
			uint32_t refreshRateUs;		///< Max chip select low time (for refresh and only for PSRAM)
			bool writeZeroLatency;		///< True if Chip supports 0-latency writes.
		};

		// Delete copy constructors
		HyperRAM(const HyperRAM&) = delete;
		HyperRAM& operator=(const HyperRAM&) = delete;

		HyperRAM(HyperBus &hyperBus) : bus(hyperBus) {}

		/// @brief Initializes the RAM and the underlying bus.
		/// @param config HyperRAM configuration
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		Status ReadID(uint8_t *manufacturerID, uint8_t *devType);

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

		/// @brief Enter into memory mapped mode
		void EnterMemoryMappedMode();

		/// @brief Exit memory mapped mode
		void ExitMemoryMappedMode();

		/// @brief Gets the physical base address (e.g., 0x90000000).
		/// @return The physical address from the underlying bus.
		uint32_t GetBaseAddr() const { return bus.GetBaseAddr(); }

	private:
		HyperBus& bus;
		Config config;

		// Internal Register Addresses (Register Space)
        static constexpr uint32_t REG_ID0 = 0x0000; // Identification Register 0
        static constexpr uint32_t REG_ID1 = 0x0004; // Identification Register 1
        static constexpr uint32_t REG_CFG0 = 0x01000000; // Configuration Register 0
        static constexpr uint32_t REG_CFG1 = 0x01000004; // Configuration Register 1
};