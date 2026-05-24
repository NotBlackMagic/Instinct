/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Loader/Dev_Inf.c
 */

#include "Dev_Inf.h"

/* This structure containes information used by ST-LINK Utility to program and erase the device */
#if defined(__ICCARM__)
__root sStorageInfo const StorageInfo =
#else
__attribute__((used)) sStorageInfo const StorageInfo =
#endif /* __ICCARM__*/
{
	"26HS512TA100_PlumaN6HD",		// Device Name + Board name
	NOR_FLASH,						// Device Type
	0x70000000UL,					// Device Start Address (For XSPI2)
	0x4000000,						// Device Size in 64 MBytes
	0x0100,							// Programming Page Size 512 Bytes
	0xFF,							// Initial Content of Erased Memory
	{
		{0x00000100, 0x00040000},	// Sector Num: 256, Sector Size: 256 KBytes
		{0x00000000, 0x00000000}
	},
};