/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	tx_pluman6_sd_driver.cpp
 */

#include "tx_pluman6_sd_driver.h"

extern SD sdCard;
extern SD::Config sdConfig;

// Internal Bounce Buffer for Boot Sector (MBR) reads
// FileX sometimes reads Sector 0 into an unaligned stack variable.
// We need this fallback to prevent IDMA errors.
static __attribute__((aligned(32))) uint8_t bootSectorBuffer[512];

extern "C" void PlumaSDDriver(FX_MEDIA *media_ptr) {
	// Process the driver request
	switch(media_ptr->fx_media_driver_request) {
		case FX_DRIVER_INIT: {
			// Initialize the SD Card
			if(sdCard.Init(sdConfig) == Status::Ok) {
				media_ptr->fx_media_driver_status = FX_SUCCESS;
				
				// Pass SD card geometry to FIleX
				SD::CardInfo cardInfo = sdCard.GetCardInfo();
				media_ptr->fx_media_driver_sectors = cardInfo.blockCount;
			} 
			else {
				media_ptr->fx_media_driver_status = FX_IO_ERROR;
			}
			break;
		}
		case FX_DRIVER_UNINIT: {
			sdCard.Reset();
			media_ptr->fx_media_driver_status = FX_SUCCESS;
			break;
		}
		case FX_DRIVER_READ: {
			// Calculate Physical Address (LBA)
			// LBA = Partition Start (Hidden Sectors) + Requested Logical Sector
			uint32_t lba = media_ptr->fx_media_driver_logical_sector + media_ptr->fx_media_hidden_sectors;
			
			uint32_t count = media_ptr->fx_media_driver_sectors;
			uint8_t* buffer = (uint8_t*)media_ptr->fx_media_driver_buffer;

			// Perform Read
			if(sdCard.ReadBlocks(lba, buffer, count) == Status::Ok) {
				media_ptr->fx_media_driver_status = FX_SUCCESS;
			}
			else {
				media_ptr->fx_media_driver_status = FX_IO_ERROR;
			}
			break;
		}
		case FX_DRIVER_WRITE: {
			uint32_t lba = media_ptr->fx_media_driver_logical_sector + media_ptr->fx_media_hidden_sectors;
			uint32_t count = media_ptr->fx_media_driver_sectors;
			uint8_t* buffer = (uint8_t*)media_ptr->fx_media_driver_buffer;

			if(sdCard.WriteBlocks(lba, buffer, count) == Status::Ok) {
				media_ptr->fx_media_driver_status = FX_SUCCESS;
			}
			else {
				media_ptr->fx_media_driver_status = FX_IO_ERROR;
			}
			break;
		}
		case FX_DRIVER_FLUSH: {
			// Cache flushing is handled in WriteBlocks already (with WaitForBusyD0End)
			media_ptr->fx_media_driver_status = FX_SUCCESS;
			break;
		}
		case FX_DRIVER_ABORT: {
			// FileX wants to cancel. Our driver handles "DeInit" as a cancel,
			// but for a simple abort, we usually just return success.
			media_ptr->fx_media_driver_status = FX_SUCCESS;
			break;
		}
		case FX_DRIVER_BOOT_READ: {
			// Read Physical Sector 0 (Master Boot Record)
			// WARN: buffer passed here might be on the stack (Unaligned)
			uint8_t* targetBuf = (uint8_t*)media_ptr->fx_media_driver_buffer;
			
			Status status;
			if(((uint32_t)targetBuf & 0x1F) == 0) {
				// Good alignment, read directly
				status = sdCard.ReadBlocks(0, targetBuf, 1);
			}
			else {
				// Bad alignment! Use internal bounce buffer.
				status = sdCard.ReadBlocks(0, bootSectorBuffer, 1);
				if(status == Status::Ok) {
					memcpy(targetBuf, bootSectorBuffer, 512);
				}
			}
			if(status == Status::Ok) {
				// MBR handling (why??)
				if(targetBuf[0] != 0xEB && targetBuf[0] != 0xE9) {
				// It's an MBR. Parse the Partition Table (starts at offset 446).
				// We want Partition 1's starting LBA (offset 8 within the 16-byte entry, so 454).
				uint32_t hiddenSectors = targetBuf[454] | 
										(targetBuf[455] << 8) | 
										(targetBuf[456] << 16) | 
										(targetBuf[457] << 24);

					// Tell FileX where the real partition starts
					media_ptr->fx_media_hidden_sectors = hiddenSectors;

					// Read the ACTUAL FAT Boot Sector
					status = sdCard.ReadBlocks(hiddenSectors, bootSectorBuffer, 1);
					if(status == Status::Ok) {
						memcpy(targetBuf, bootSectorBuffer, 512);
					}
				}
				else {
					// It's already a raw FAT volume (No MBR)
					media_ptr->fx_media_hidden_sectors = 0;
				}
				media_ptr->fx_media_driver_status = FX_SUCCESS;
			}
			else {
				media_ptr->fx_media_driver_status = FX_IO_ERROR;
			}
			break;
		}
		case FX_DRIVER_BOOT_WRITE: {
			// Write Physical Sector 0
			uint8_t* sourceBuf = (uint8_t*)media_ptr->fx_media_driver_buffer;

			Status status;
			if(((uint32_t)sourceBuf & 0x1F) == 0) {
				status = sdCard.WriteBlocks(0, sourceBuf, 1);
			}
			else {
				memcpy(bootSectorBuffer, sourceBuf, 512);
				status = sdCard.WriteBlocks(0, bootSectorBuffer, 1);
			}
			if(status == Status::Ok) {
				media_ptr->fx_media_driver_status = FX_SUCCESS;
			}
			else {
				media_ptr->fx_media_driver_status = FX_IO_ERROR;
			}
			break;
		}
		default:
			media_ptr->fx_media_driver_status = FX_IO_ERROR;
			break;
	}
}