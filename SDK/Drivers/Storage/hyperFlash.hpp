/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Storage/hyperFlash.hpp
 * Author:  NotBlackMagic
 * Brief:   HyperFlash driver class for STM32N6.
 */

#pragma once

#include <stdint.h>

#include "hyperbus.hpp"
#include "status.hpp"
#include "system.hpp"

#include "tx_api.h"

class HyperFlash {
	public:
		// Internal Register Addresses (Register Space)
		// Based on verification of x2 alignment due to DDR and 8-bit bus width
		static constexpr uint32_t REG_CFG0 = 0x1000;	// Configuration Register 0
		static constexpr uint32_t REG_CFG1 = 0x1002;	// Configuration Register 1

		// Status Register: Provides status on operation
		static constexpr uint32_t STATUS_DICRCS = 0x0100;		// Memory Array Data Integrity Check CRC Suspend Status Flag (0: Not supend mode, 1: Supend mode)
		static constexpr uint32_t STATUS_RDYBSY = 0x0080;		// Device Ready/Busy Status Flag (0: Device busy, 1: Device standby)
		static constexpr uint32_t STATUS_ERASES = 0x0040;		// Erase operation Suspend Status Flag (0: Not supend mode, 1: Supend mode)
		static constexpr uint32_t STATUS_ERSERR = 0x0020;		// Erasing Error Status Flag (0: Operation was successfully, 1: Operation was UNsuccessfully)
		static constexpr uint32_t STATUS_PRGERR = 0x0010;		// Programming Error Status Flag (0: Operation was successfully, 1: Operation was UNsuccessfully)
		static constexpr uint32_t STATUS_WRBFAB = 0x0008;		// Write Buffer Abort Status Flag (0: Program not aborted, 1: Program aborted)
		static constexpr uint32_t STATUS_PROGMS = 0x0004;		// Program operation Suspend Status Flag (0: Not supend mode, 1: Supend mode)
		static constexpr uint32_t STATUS_SPROTE = 0x0002;		// Sector Protection (Lock) Error Flag (0: Not Error, 1: Error)
		static constexpr uint32_t STATUS_SESTAT = 0x0001;		// Sector Erase Success/Failure Status Flag (0: Not successfully, 1: Successfully)

		/// @brief HyperFlash device configuration structure.
		struct Config {
			const char* deviceName;		///< e.g. "Cypress S26KS512S"
			uint16_t expectedID;		///< Manufacturer ID to check against (0 = Ignore check)
			uint16_t expectedDeviceID;	///< Device ID check (0 = Skip)

			// Memory array geometry
			uint32_t sizeBytes;			///< Total size in bytes (e.g. 8*1024*1024)
			uint32_t sectorSize;		///< Erase Block (e.g. 256KB)
			uint32_t pageSize;			///< Write Page Buffer (e.g. 512B)

			// Timing Presets
			uint32_t sourceClockHz;		///< Peripheral source clock frequency in Hz
			uint32_t frequencyHz;       ///< Target Bus Frequency
			uint8_t initialLatency;		///< Clock cycles (latency count)
			bool fixedLatency;			///< True = Force Fixed, False = Variable (Default)
			uint8_t rwRecoveryTime;		///< Additional latency after read/write

			// HyperFlash device configurations
			uint16_t configReg0;		//< Value to write to CFG0
			uint16_t configReg1;		//< Value to write to CFG1
		};

		// Delete copy constructors
		HyperFlash(const HyperFlash&) = delete;
		HyperFlash& operator=(const HyperFlash&) = delete;

		HyperFlash(HyperBus &hyperBus) : bus(hyperBus) {}

		/// @brief Initializes the flash and the underlying bus.
		/// @param config HyperFlash configuration
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		/// @brief Reads the Manufacturer, Family IDs and Unique ID
		/// @param manID    Pointer to be filled with manufacturer ID.
		/// @param devType  Pointer to be filled with family ID.
		/// @param uniqueID Pointer to be filled with unique ID.
		/// @return Status::Ok if read succeeded, or Status::Error if failed.
		Status ReadID(uint16_t *manID, uint16_t *famID, uint64_t *uniqueID);
		Status ReadJEDEC();

		/// @brief Resets the Flash state machine to Read Array mode.
        /// @note  Must be called if Program/Erase returns an Error status.
        Status Reset();

		/// @brief Reads data from Flash (Memory Mapped or Indirect).
		/// @param addr	Target address.
		/// @param data	Pointer to buffer for read data.
		/// @param len	Number of bytes to read.
		/// @return Status::Ok if the read was successful, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status Read(uint32_t addr, uint8_t *buf, uint32_t len);

		/// @brief Programs data to Flash.
		/// @param addr	Target address.
		/// @param data	Pointer to buffer to write.
		/// @param len	Number of bytes to write.
		/// @return Status::Ok if the write was successful, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status Program(uint32_t addr, const uint8_t *buf, uint32_t len);

		/// @brief Erases the entire chip.
		Status ChipErase();

		/// @brief Erases a specific sector.
        /// @param sectorAddr Byte address within the sector to erase.
		Status SectorErase(uint32_t sectorAddr);

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

		// Command Address Patterns (Byte-Addressing shifted: Word Addr << 1)
		static constexpr uint32_t CMD_ADDR_UNLOCK_1 = (0x555UL << 1);
        static constexpr uint32_t CMD_ADDR_UNLOCK_2 = (0x2AAUL << 1);
        
        static constexpr uint16_t CMD_DATA_UNLOCK_1 = 0xAA;
        static constexpr uint16_t CMD_DATA_UNLOCK_2 = 0x55;
        static constexpr uint16_t CMD_DATA_ERASE = 0x80;
        static constexpr uint16_t CMD_DATA_RESET = 0xF0;
        static constexpr uint16_t CMD_DATA_AUTOSELECT = 0x90;
        static constexpr uint16_t CMD_DATA_PROGRAM = 0xA0;
        static constexpr uint16_t CMD_DATA_CHIP_ERASE = 0x10;
        static constexpr uint16_t CMD_DATA_SECTOR_ERASE = 0x30;
        static constexpr uint16_t CMD_DATA_WRITE_BUFFER = 0x25;
        static constexpr uint16_t CMD_DATA_PROG_BUFFER  = 0x29;
		static constexpr uint16_t CMD_DATA_STATUS_REG_READ = 0x70;

		Status WritePage(uint32_t pageAddr, const uint8_t *buf, uint32_t len);
		Status UnlockSequence();
		Status WriteCommand(uint32_t addr, uint16_t data);
		Status ReadStatus(uint16_t &status);
		Status ClearStatus();

		bool WaitForReady(uint32_t timeoutMs);
};