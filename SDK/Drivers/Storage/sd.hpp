/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Storage/sd.hpp
 * Author:  NotBlackMagic
 * Brief:   SD Card driver class for STM32N6.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "gpio.hpp"
#include "sdmmc.hpp"
#include "system.hpp"

#include "status.hpp"

#include "tx_api.h"

class SD {
	public:
		/// @brief SD card command codes.
		enum class Command : uint8_t {
			GoIdleState			= 0,	// CMD0:  Resets the SD memory card. 
			SendOpCondMMC		= 1,	// CMD1:  Asks card to send operating conditions (MMC Only).
			AllSendCid			= 2,	// CMD2:  Asks all cards to send their CID numbers (Unique ID).
			SendRelativeAddr	= 3,	// CMD3:  Asks card to publish its new relative address (RCA).
			SetDsr				= 4,	// CMD4:  Programs the DSR (Driver Stage Register) of all cards.
			SendOpCondSD		= 5,	// CMD5:  Sends host capacity support information (HCS) and asks the accessed card to send its operating condition register (OCR) content in the response on the CMD line.
			SwitchFunc			= 6,	// CMD6:  Checks switchable function (mode 0) and switch card function (mode 1).
			SelectCard			= 7,	// CMD7:  Selects a card (enters transfer State) or deselects (enters standby).
			SendIfCond			= 8,	// CMD8:  Sends interface condition (voltage check).
			SendCsd				= 9,	// CMD9:  Asks specific card to send card specific data (CSD).
			SendCid				= 10,	// CMD10: Asks specific card to send card identifier (CID).
			VoltageSwitch		= 11,	// CMD11: SD card switch to 1.8V signaling.
			StopTransmission	= 12,	// CMD12: Forces card to stop transmission.
			SendStatus			= 13,	// CMD13: Asks specific card to send its status register.
										// CMD14: Reserved
			GoInactiveState		= 15,	// CMD15: Sends an addressed card into the Inactive State.
			SetBlockLen			= 16,	// CMD16: Sets block length (in bytes) for all following block commands (read, write, lock). Default block length is fixed to 512 Bytes. Not effective for SDHS and SDXC.
			ReadSingleBlock		= 17,	// CMD17: Reads a single block of size "SetBlockLen in case of SDSC and a fixed 512 bytes for SDHC and SDXC.
			ReadMultiBlock		= 18,	// CMD18: Reads multiple blocks continuously until "StopTransmission" command.
			SendTuningBlock		= 19,	// CMD19: Send 64-byte tuning pattern, for UHS-I (SDR50 and SDR104).
			SpeedClassControl	= 20,	// CMD20: Speed class control.
			SetBlockCount		= 23,	// CMD23: Defines number of blocks to transfer for "ReadMultiBlock" and "WriteMultiBlock"
			WriteSingleBlock	= 24,	// CMD24: Writes a single block of size "SetBlockLen in case of SDSC and a fixed 512 bytes for SDHC and SDXC.
			WriteMultiBlock		= 25,	// CMD25: Writes multiple blocks continuously until "StopTransmission" command.
										// CMD26: Reserved for manufacturers.
			ProgramCsd			= 27,	// CMD27: Programming of programmable CSD bits.
			SetWriteProt		= 28,	// CMD28: Sets write protection bit of the addressed group.
			ClrWriteProt		= 29,	// CMD29: Clears write protection bit of the addressed group.
			SendWriteProt		= 30,	// CMD30: Asks card to send status of write protection bits.
			EraseWrBlkStart		= 32,	// CMD32: Sets the address of the first block to be erased.
			EraseWrBlkEnd		= 33,	// CMD33: Sets the address of the last block to be erased.
										// CMD35: ??
										// CMD36: ??
			Erase				= 38,	// CMD38: Reserved for SD security applications.
										// CMD39: SD card doesn't support it (Reserved).
										// CMD40: SD card doesn't support it (Reserved).
			LockUnlock			= 42,	// CMD42: Sets/Resets password or Locks/Unlocks card.
			AppCmd				= 55,	// CMD55: Indicates next command is an Application Command (ACMD).
			GenCmd				= 56,	// CMD56: General Purpose Command (Vendor Specific In/Out).
			NoCmd				= 64	// No command
		};

		/// @brief SD card application command codes.
		enum class AppCommand : uint8_t {
			SetBusWidth			= 6,	// ACMD6:  Defines the data bus width to be used for data transfer. The allowed data bus widths are given in SCR register. 
			SdStatus			= 13,	// ACMD13: Asks card to send SD Status (512 bits of proprietary status).
			SendNumWrBlocks		= 22,	// ACMD22: Sends the number of the written (without errors) write blocks. Responds with 32bit+CRC data block. 
			SetWrBlkEraseCount	= 23,	// ACMD23: ??
			SdAppOpCond			= 41,	// ACMD41: Sends host capacity support information (HCS) and asks the accessed card to send its operating condition register (OCR) content in the response.
			GetClrCardDetect	= 42,	// ACMD42: Connect/Disconnect the 50KOhm pull-up on CD/DAT3 of the card.
			SendScr				= 51	// ACMD51: Reads the SD Configuration Register (SCR).
										// ACMD52: For SD I/O card only, reserved for security specification.
										// ACMD53: For SD I/O card only, reserved for security specification.
		};

		/// @brief SD card states.
		enum class CardState : uint8_t {
			Idle = 0,
			Ready = 1,
			Ident = 2,
			Standby = 3,
			Transfer = 4,
			Data = 5,
			Receive = 6,
			Programming = 7,
			Disconnected= 8,
			Error = 0xFF
		};

		/// @brief SD bus configuration structure.
		struct Config {
			bool use4BitMode;       ///< True: Switch to 4-bit bus (ACMD6) after init.
			bool use1V8Level;		///< True: Switch to 1.8 V if supported by card.
			bool useHighSpeed;      ///< True: Switch to High Speed (50MHz) if supported by card.
			bool useUHS;			///< True: Switch to Ultra-High Speed if supported by card.
			GPIO* vioSelectPin;		///< GPIO that controls the Voltage selection switch, if exist.
		};

		/// @brief SD card information structure.
		struct CardInfo {
			uint64_t sizeBytes;		///< Total card capacity in bytes (e.g. 32000000000 for 32GB).
			uint32_t blockCount;	///< Total number of addressable blocks (LBA count).
			uint32_t blockSize;		///< Block size in bytes (Standard is 512 bytes).
			bool highCapacity;		///< True if SDHC/SDXC (>2GB), False if SDSC (<2GB).

			char cardName[6];		///< Product Name (PNM) as a null-terminated string (e.g., "SU32G").
			uint8_t manufacturerID;	///< Manufacturer ID (MID). Assiged by SD-3C, LLC.
			uint16_t oemID;			///< OEM/Application ID (OID).
			uint32_t serialNumber;	///< Product Serial Number (PSN). Unique 32-bit ID.
			uint8_t revision;		///< Product Revision (PRV). 0x12 means v1.2.
			uint16_t mfgYear;		///< Manufacturing Year (e.g., 2025). Parsed from MDT.
			uint8_t mfgMonth;		///< Manufacturing Month (1-12). Parsed from MDT.

			uint8_t speedClass;		///< Speed Class: 0, 2, 4, 6, 10. (10 = 10 MB/s min write).
			uint8_t uhsSpeedGrade;	///< UHS Grade: 0, 1 (U1), 3 (U3). (U3 = 30 MB/s min write).
			uint8_t videoSpeedClass;	///< Video Class: 0, 10 (V10), 30 (V30), 60 (V60), 90 (V90). (V30 = 30 MB/s min write).
			uint8_t appPerfClass;	///< Application Performance Class: 0, 1 (A1), 2 (A2).

			bool support4Bit;		///< True if card supports 4-bit bus width.
			bool supportCmd23;		///< True if card supports CMD23 (Faster Multi-Block Writes).
			uint8_t specVersion;	///< Physical Spec: 0=v1.0, 1=v1.1, 2=v2.0+ (SDHC/SDXC).
			uint8_t allocationUnitSize;	///< AU Size code (Shift value). Used for UHS tuning.
			
			SDMMC::BusSpeed activeMode;	///< Current Speed Mode (Default, HighSpeed, SDR25...)
			uint8_t currentBusWidth;	///< Active Bus Width: 0=1-bit, 2=4-bit.
		};

		/// @brief Constructor.
		/// @param sdmmc Reference to the low-level bus driver.
		SD(SDMMC& sdmmc);

		/// @brief Initializes the card.
		/// @details Performs the full Power-On -> Idle -> Ready -> Transfer sequence.
		///          Negotiates voltage, bus width, and speed.
		/// @param config SD card configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		/// @brief Resets the internal driver state and prepares for a new card.
		/// @return Status::Ok if reset succeeded cleanly.
		Status Reset(void);

		/// @brief Checks if a card is physically present via GPIO.
		/// @return True if card detected.
		bool IsDetected(void);

		/// @brief Gets the parsed card information (Capacity, ID, etc).
		/// @return Parsed card information.
		CardInfo GetCardInfo(void) const { return this->cardInfo; }

		/// @brief Reads data blocks from the card.
		/// @param lba Logical block address (Sector Index).
		/// @param buf Pointer to buffer for read data (Must be 4-byte aligned for DMA!).
		/// @param blockCount Number of blocks to read.
		/// @return Status::Ok if read succeeded, Status::Timeout on timeout, Status::Error on error.
		Status ReadBlocks(uint32_t lba, uint8_t *buf, uint32_t blockCount);

		/// @brief Writes data blocks to the card.
		/// @param lba Logical Block Address (Sector Index).
		/// @param buf Pointer to data to write (Must be 4-byte aligned for DMA!).
		/// @param blockCount Number of blocks to write.
		/// @return Status::Ok if write succeeded, Status::Timeout on timeout, Status::Error on error.
		Status WriteBlocks(uint32_t lba, const uint8_t *buf, uint32_t blockCount);

		/// @brief Erases a range of blocks.
		/// @param startAddr Start logical block address (LBA)
		/// @param endAddr End logical block address (LBA)
		/// @return Status::Ok if erase succeeded, Status::Timeout on timeout, Status::Error on error.
		Status Erase(uint32_t startAddr, uint32_t endAddr);

		/// @brief Gets the current card state (Transfer, Programming, etc.) via CMD13.
		/// @return Current card state.
        CardState GetCardState(void);

		/// @brief Checks if the card is currently busy (Programming/Receiving).
        /// @return True if busy, False if ready for commands.
        bool IsCardBusy(void);

		/// @brief Helper to wait for card not busy, with timeout and yielding to RTOS´
		/// @param timeoutMs Max wait time in ms.
		bool WaitCardBusy(uint32_t timeoutMs);

	private:
		SDMMC& bus;
		Config config;
		CardInfo cardInfo;

		uint16_t RCA;           ///< Relative Card Address (Assigned by card during init)
		bool isInitialized;
		__attribute__((aligned(32))) uint32_t statusBuf[16];

		/// @brief Sends an Application Command (ACMD).
		/// @details Handles the requirement of sending CMD55 before the specific command.
		SDMMC::CommandResponse SendAppCommand(AppCommand acmd, uint32_t arg, SDMMC::ResponseType respType);

		/// @brief Parses the CID (Card Identification) register (128 bits).
		void ParseCID(uint32_t *cidRaw);

		/// @brief Parses the CSD (Card Specific Data) register (128 bits).
		void ParseCSD(uint32_t *csdRaw);

		/// @brief Reads the SCR (SD Configuration Register) to determine bus width support.
		bool GetSCR(void);

		/// @brief Reads the card status register (SSR).
		bool GetSDStatus(void);

		/// @brief Switches the card speed mode via CMD6.
		bool ChangeSpeedMode(SDMMC::BusSpeed speed);

		/// @brief Wrapper function for the GPIO voltage switch call, required for the SDMMC SwitchTo1V8() function call
		static void VoltageSwitchCallback(void* context);
};