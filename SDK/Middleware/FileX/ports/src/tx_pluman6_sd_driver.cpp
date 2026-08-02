/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	tx_pluman6_sd_driver.cpp
 */

#include "tx_pluman6_sd_driver.h"

extern SD sdCard;
extern SD::Config sdConfig;

// Internal Bounce Buffer for cache unaligned reads and writes. Used to create an aligned buffer to pass to the SDMMC driver, required for the IDMA
static __attribute__((aligned(32))) uint8_t bounceBuffer[512];

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

			// Check for cache alignment
			if(((uint32_t)buffer & 0x1F) == 0) {
				// Good alignment, read directly
				if (sdCard.ReadBlocks(lba, buffer, count) == Status::Ok) {
					media_ptr->fx_media_driver_status = FX_SUCCESS;
					break;
				}
			}
			else {
				// Bad alignment! Use internal bounce buffer.
				bool success = true;
				for(uint32_t i = 0; i < count; i++) {
					if(sdCard.ReadBlocks(lba + i, bounceBuffer, 1) != Status::Ok) {
						success = false;
						break;
					}
					memcpy(buffer + (i * 512), bounceBuffer, 512);
				}
				if(success == true) {
					media_ptr->fx_media_driver_status = FX_SUCCESS;
					break;
				}
			}
			media_ptr->fx_media_driver_status = FX_IO_ERROR;
			break;
		}
		case FX_DRIVER_WRITE: {
			uint32_t lba = media_ptr->fx_media_driver_logical_sector + media_ptr->fx_media_hidden_sectors;
			uint32_t count = media_ptr->fx_media_driver_sectors;
			uint8_t* buffer = (uint8_t*)media_ptr->fx_media_driver_buffer;

			if (((uint32_t)buffer & 0x1F) == 0) {
				// Good alignment, write directly
				if (sdCard.WriteBlocks(lba, buffer, count) == Status::Ok) {
					media_ptr->fx_media_driver_status = FX_SUCCESS;
					break;
				}
			}
			else {
				// Bad alignment! Use internal bounce buffer.
				bool success = true;
				for (uint32_t i = 0; i < count; i++) {
					memcpy(bounceBuffer, buffer + (i * 512), 512);
					if (sdCard.WriteBlocks(lba + i, bounceBuffer, 1) != Status::Ok) {
						success = false;
						break;
					}
				}
				
				if (success) {
					media_ptr->fx_media_driver_status = FX_SUCCESS;
					break;
				}
			}
			media_ptr->fx_media_driver_status = FX_IO_ERROR;
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
				status = sdCard.ReadBlocks(0, bounceBuffer, 1);
				if(status == Status::Ok) {
					memcpy(targetBuf, bounceBuffer, 512);
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
					if(((uint32_t)targetBuf & 0x1F) == 0) {
						// Good alignment, read directly
						status = sdCard.ReadBlocks(hiddenSectors, targetBuf, 1);
					}
					else {
						// Bad alignment! Use internal bounce buffer.
						status = sdCard.ReadBlocks(hiddenSectors, bounceBuffer, 1);
						if(status == Status::Ok) {
							memcpy(targetBuf, bounceBuffer, 512);
						}
					}
				}
				else {
					// It's already a raw FAT volume (No MBR)
					media_ptr->fx_media_hidden_sectors = 0;
				}

				if(status == Status::Ok) {
                    media_ptr->fx_media_driver_status = FX_SUCCESS;
                }
                else {
                    media_ptr->fx_media_driver_status = FX_IO_ERROR;
                }
				break;
			}
			else {
				media_ptr->fx_media_driver_status = FX_IO_ERROR;
			}
			break;
		}
		case FX_DRIVER_BOOT_WRITE: {
			// Write Physical Sector 0
			uint8_t* sourceBuf = (uint8_t*)media_ptr->fx_media_driver_buffer;
			uint32_t bootLba = 0;	//media_ptr->fx_media_hidden_sectors;

			Status status;
			if(((uint32_t)sourceBuf & 0x1F) == 0) {
				// Good alignment, write directly
				status = sdCard.WriteBlocks(bootLba, sourceBuf, 1);
			}
			else {
				// Bad alignment! Use internal bounce buffer.
				memcpy(bounceBuffer, sourceBuf, 512);
				status = sdCard.WriteBlocks(bootLba, bounceBuffer, 1);
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