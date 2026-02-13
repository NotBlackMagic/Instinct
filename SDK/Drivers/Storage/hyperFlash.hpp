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

//Status Register: Provides status on operation
#define FLASH_STATUS_DICRCS					0x0100			//Memory Array Data Integrity Check CRC Suspend Status Flag (0: Not supend mode, 1: Supend mode)
#define FLASH_STATUS_RDYBSY					0x0080			//Device Ready/Busy Status Flag (0: Device busy, 1: Device standby)
#define FLASH_STATUS_ERASES					0x0040			//Erase operation Suspend Status Flag (0: Not supend mode, 1: Supend mode)
#define FLASH_STATUS_ERSERR					0x0020			//Erasing Error Status Flag (0: Operation was successfully, 1: Operation was UNsuccessfully)
#define FLASH_STATUS_PRGERR					0x0010			//Programming Error Status Flag (0: Operation was successfully, 1: Operation was UNsuccessfully)
#define FLASH_STATUS_WRBFAB					0x0008			//Write Buffer Abort Status Flag (0: Program not aborted, 1: Program aborted)
#define FLASH_STATUS_PROGMS					0x0004			//Program operation Suspend Status Flag (0: Not supend mode, 1: Supend mode)
#define FLASH_STATUS_SPROTE					0x0002			//Sector Protection (Lock) Error Flag (0: Not Error, 1: Error)
#define FLASH_STATUS_SESTAT					0x0001			//Sector Erase Success/Failure Status Flag (0: Not successfully, 1: Successfully)

class HyperFlash {
	public:
		struct Config {
			const char* deviceName;		///< e.g. "Cypress S26KS512S"
			uint16_t expectedID;		///< Manufacturer ID to check against (0 = Ignore check)
			uint16_t expectedDeviceID;	///< Device ID check (0 = Skip)

			// Memory array geometry
			uint32_t sizeBytes;			///< Total size in bytes (e.g. 8*1024*1024)
			uint32_t sectorSize;		///< Erase Block (e.g. 256KB)
			uint32_t pageSize;			///< Write Page Buffer (e.g. 512B)

			// Timing Presets
			uint32_t frequencyHz;       ///< Target Bus Frequency
			uint8_t initialLatency;		///< Clock cycles (latency count)
			bool fixedLatency;			///< True = Force Fixed, False = Variable (Default)
			uint8_t rwRecoveryTime;		///< Additional latency after read/write
		};

		// Delete copy constructors
		HyperFlash(const HyperFlash&) = delete;
		HyperFlash& operator=(const HyperFlash&) = delete;

		HyperFlash(HyperBus &hyperBus) : bus(hyperBus) {}

		/// @brief Initializes the flash and the underlying bus.
		/// @param config HyperFlash configuration
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		Status ReadID(uint16_t *manufacturerID, uint16_t *familyID, uint64_t *uniqueID);
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

		// Internal flash operation codes
		enum class Operation : uint8_t {
			StatusRegRead = 0x70,
			WriteToBuffer = 0x25,
			ProgBufferToFlash = 0x29,
			Erase = 0x80,
			ChipErase = 0x10,
			SectorErase = 0x30,
			Reset = 0xF0,
			Unlock_1 = 0xAA,
			Unlock_2 = 0x55
		};

		// Command Address Patterns (Byte-Addressing shifted: Word Addr << 1)
		static constexpr uint32_t ADDR_UNLOCK_1 = (0x555UL << 1);
		static constexpr uint32_t ADDR_UNLOCK_2 = (0x2AAUL << 1);

		Status WritePage(uint32_t pageAddr, const uint8_t *buf, uint32_t len);
		void UnlockSequence();
		void WriteCommand(uint32_t addr, uint16_t data);
		uint16_t ReadStatus();
		Status ClearStatus();

		bool WaitForReady(uint32_t timeoutMs);
};