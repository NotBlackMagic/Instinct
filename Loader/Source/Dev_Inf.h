/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Loader/Dev_Inf.h
 * Author:  NotBlackMagic
 * Brief:   This file defines the structure containing informations about the external flash memory used by STM32CubeProgramer in programming/erasing the device.
 */

#ifdef __cplusplus
 extern "C" {
#endif

#define MCU_FLASH		1
#define NAND_FLASH		2
#define NOR_FLASH		3
#define SRAM			4
#define PSRAM			5
#define PC_CARD			6
#define SPI_FLASH		7
#define I2C_FLASH		8
#define SDRAM			9
#define I2C_EEPROM		10

#define SECTOR_NUM		10				// Max Number of Sector types

typedef struct {
	unsigned long SectorNum;					// Number of Sectors
	unsigned long SectorSize;					// Sector Size in Bytes
} DeviceSectors;

typedef struct {
	char DeviceName[100];		// Device Name and Description
	unsigned short DeviceType;				// Device Type: ONCHIP, EXT8BIT, EXT16BIT, ...
	unsigned long DeviceStartAddress;		// Default Device Start Address
	unsigned long DeviceSize;				// Total Size of Device
	unsigned long PageSize;				// Programming Page Size
	unsigned char EraseValue;				// Content of Erased Memory
	DeviceSectors sectors[SECTOR_NUM];
} sStorageInfo;

#ifdef __cplusplus
}
#endif