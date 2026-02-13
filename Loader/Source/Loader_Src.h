/**
  ******************************************************************************
  * @file    Loader_Src.h
  * @author  MCD Application Team
  * @brief   Header file of Loader_Src.c
  *           
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOADER_SRC_H
#define __LOADER_SRC_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_cortex.h"
#include "stm32n6xx_ll_pwr.h"
#include "stm32n6xx_ll_rcc.h"
#include "stm32n6xx_ll_system.h"
#include "stm32n6xx_ll_utils.h"

#include "system.hpp"
#include "hyperbus.hpp"
#include "hyperFlash.hpp"

#include "../../Board/PlumaN6.hpp"

#ifdef __cplusplus
 extern "C" {
#endif

#define TIMEOUT 5000U

/**
 * @brief macro to force the compiler to keep the code
 */
#if defined(__ICCARM__)
#define KeepInCompilation __root
#else
#define KeepInCompilation __attribute__((used))
#endif /* __ICCARM__ */

/* Private function prototypes -----------------------------------------------*/
extern "C" {
	KeepInCompilation uint32_t Init ();
	KeepInCompilation uint32_t Write (uint32_t Address, uint32_t Size, uint8_t* buffer);
	KeepInCompilation uint32_t SectorErase (uint32_t EraseStartAddress, uint32_t EraseEndAddress);
	KeepInCompilation uint64_t Verify (uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size, uint32_t missalignement);
	KeepInCompilation uint32_t MassErase (uint32_t Parallelism );
}

int main(void);

#ifdef __cplusplus
}
#endif

#endif /* __LOADER_SRC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/