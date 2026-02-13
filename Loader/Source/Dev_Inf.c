/**
  ******************************************************************************
  * @file    Dev_Inf.c
  * @author  MCD Application Team
  * @brief   This file defines the structure containing informations about the 
  *          external flash memory MX25LM51245G used by STM32CubeProgramer in 
  *          programming/erasing the device.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "Dev_Inf.h"

/* This structure containes information used by ST-LINK Utility to program and erase the device */
#if defined(__ICCARM__)
__root sStorageInfo const StorageInfo =
#else
__attribute__((used)) sStorageInfo const StorageInfo =
#endif                                     /* __ICCARM__*/
{
	"26HS512TA100_PlumaN6HD",		// Device Name + Board name
	NOR_FLASH,						// Device Type
	0x70000000UL,					// Device Start Address (For XSPI2)
	0x4000000,						// Device Size in 64 MBytes
	0x0100,							// Programming Page Size 512 Bytes
	0xFF,							// Initial Content of Erased Memory
	{
		{0x00000100, 0x00040000},			// Sector Num: 256, Sector Size: 256 KBytes
		{0x00000000, 0x00000000}
	},
};