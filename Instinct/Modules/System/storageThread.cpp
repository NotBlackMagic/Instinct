/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/System/storageThread.cpp
 */

#include "storageThread.hpp"

TX_THREAD StorageThread::threadPtr;
__attribute__((aligned(32))) uint8_t StorageThread::threadStack[StorageThread::threadStackSize];

FX_MEDIA StorageThread::sdMedia;
__attribute__((aligned(32))) uint8_t StorageThread::sdMediaPool[StorageThread::threadStackSize];
std::atomic<bool> StorageThread::storageReady{false};

// ATTENTION: PlumaN6 HW Rev. 1 does NOT support 1V8 voltage switch for uSD card due to routing error!
// const SD::Config sdConfig = {.use4BitMode = true, .use1V8Level = false, .useHighSpeed = true, .useUHS = false, .vioSelectPin = &sdVioSel};
// SD sdCard = SD(sdmmc1);

void StorageThread::Init() {
	fx_system_initialize();
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("Storage"),
											StorageThread::Run, 0,
											threadStack, sizeof(threadStack),
											1, 1,
											TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Storage Thread Create Failed.");
	}
}

void StorageThread::Run(ULONG input) {
	uint32_t status;

	sdVioSel.Write(0);		// SD VIO Selection: 0 -> 3V3, 1 -> 1V8

	LOG_INFO("Storage Thread Initialized.");

	uint8_t sdCardStatus = 0;
	while(1) {
		switch(sdCardStatus) {
			case 0: {
				// Wait for card
				if(sdDet.Read() == 0) {
					LOG_INFO("FileX Card Detected.");
					_tx_thread_sleep(20);
					sdCardStatus = 1;
				}
				break;
			}
			case 1: {
				// Card just inserted
				LOG_INFO("FileX Mount volume.");

				// Mount SD Card
				status = fx_media_open(	&sdMedia,				// Media Ptr
										const_cast<char*>("PLUMAN6"),		// Name
										PlumaSDDriver,	// Driver Entry
										0,					// Driver Info Ptr (Unused)
										sdMediaPool,			// Memory Pool
										sizeof(sdMediaPool));	// Pool Size
				
				if(status == FX_SUCCESS) {
					sdCardStatus = 2;

					ULONG64 available_bytes;
					fx_media_extended_space_available(&sdMedia, &available_bytes);
					LOG_INFO("FileX uSD Free Space: %lu MB\n", (uint32_t)(available_bytes / (1024 * 1024)));

					storageReady.store(true, std::memory_order_release);
				}
				else {
					LOG_INFO("FileX Mount Failed!");
					
					if(status == FX_BOOT_ERROR || status == FX_MEDIA_INVALID) {
						LOG_INFO("FileX Card needs formatting.");
						sdCardStatus = 3;

						// Formatting logic
						uint32_t total_sectors = 31116288;	//sdCard.GetCardInfo().blockCount;

						status = fx_media_format(	&sdMedia,		// Media Ptr
													PlumaSDDriver,	// Driver Entry
													0,			// Driver Info
													sdMediaPool,	// Memory for Init
													sizeof(sdMediaPool),
													const_cast<char*>("STM32_VOL"),	// Volume Name
													1,			// Number of FATs
													32,		// Directory Entries
													0,			// Hidden Sectors
													total_sectors,				// Total Sectors
													512,		// Sector Size
													16,	// Sectors per Cluster (32KB = Fast!)
													12345,				// Volume ID
													1 );		// Boundary Unit (Alignment)
						
						if(status == FX_SUCCESS) {
							LOG_INFO("FileX Format Complete. Retry Mount...");
						}
						else {
							LOG_INFO("FileX Format Failed.");
							sdCardStatus = 3;
						}
					}
					else {
						sdCardStatus = 3;
					}
				}
				break;
			}
			case 2: {
				if(sdDet.Read() == 1) {
					LOG_INFO("FileX Card Removed.");

					storageReady.store(false, std::memory_order_release);

					fx_media_close(&sdMedia);

					sdCardStatus = 0;
				}
				else {
					tx_thread_sleep(50);
				}
				break;
			}
			case 3: {
				if(sdDet.Read() == 1) {
					LOG_INFO("FileX Card Removed.");

					sdCardStatus = 0;
				}
				else {
					tx_thread_sleep(50);
				}
			}
		}
		tx_thread_sleep(10);
	}
}

bool StorageThread::IsReady() {
	return storageReady.load(std::memory_order_acquire);	
}

FX_MEDIA* StorageThread::GetMedia() {
	return &sdMedia;
}