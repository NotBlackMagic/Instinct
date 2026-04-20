/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/storageThread.cpp
 */

#include "storageThread.hpp"

TX_THREAD StorageThread::threadPtr;
uint8_t StorageThread::threadStack[4096];

FX_MEDIA StorageThread::sdMedia;
__attribute__((aligned(32))) uint8_t StorageThread::sdMediaPool[4096];
std::atomic<bool> StorageThread::storageReady{false};

// ATTENTION: PlumaN6 HW Rev. 1 does NOT support 1V8 voltage switch for uSD card due to routing error!
// const SD::Config sdConfig = {.use4BitMode = true, .use1V8Level = false, .useHighSpeed = true, .useUHS = false, .vioSelectPin = &sdVioSel};
// SD sdCard = SD(sdmmc1);

void StorageThread::Init() {
	fx_system_initialize();
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("Storage"),
											StorageThread::Run,
											0,
											threadStack,
											sizeof(threadStack),
											0,
											0,
											TX_NO_TIME_SLICE,
											TX_AUTO_START);
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

// void SDMMCThread(ULONG thread_input) {
// 	sdVioSel.Write(0);		// SD VIO Selection: 0 -> 3V3, 1 -> 1V8
	
// 	bool hasCard = false;
// 	while(1) {
// 		if(sdDet.Read() == 0x00 && hasCard == false) {
// 			// SD Card inserted/detected
// 			hasCard = true;

// 			ledGreen.Write(0);
// 			LOG_INFO("uSD Card Inserted");

// 			// Delay a bit before try to access card
// 			tx_thread_sleep(100);

// 			if(sdCard.Init(sdConfig) != Status::Ok) {
// 				LOG_WARN("uSD Card Init FAILED");
// 			}
// 			else {
// 				SD::CardInfo cardInfo = sdCard.GetCardInfo();
// 				LOG_INFO("uSD Card Initialized");

// 				// Calculate SD card size in Gbytes
// 				float sizeGB = (float)cardInfo.sizeBytes / (1024.0f * 1024.0f * 1024.0f);

// 				// Get card type/class as a string
// 				uint8_t len = 0;
// 				char speedStr[16] = "\0";
// 				if(cardInfo.videoSpeedClass > 0) {
// 					// Video Speed Classes [Min. Continuouse Write Speeds]: 
// 					// V6  [6 MB/s]
// 					// V10 [10 MB/s]
// 					// V30 [30 MB/s]
// 					// V60 [60 MB/s]
// 					// V90 [90 MB/s]
// 					len += snprintf(speedStr, sizeof(speedStr), "V%d", cardInfo.videoSpeedClass);
// 				}
// 				else if(cardInfo.uhsSpeedGrade > 0) {
// 					// UHS Speed Classes [Min. Continuouse Write Speeds]:
// 					// U1 [10 MB/s]
// 					// U3 [30 MB/s]
// 					len += snprintf(speedStr, sizeof(speedStr), "U%d", cardInfo.uhsSpeedGrade);
// 				}
// 				else {
// 					// Speed Classes [Min. Continuouse Write Speeds]:
// 					// C2  [2 MB/s]
// 					// C4  [4 MB/s]
// 					// C6  [6 MB/s]
// 					// C10 [10 MB/s]
// 					len += snprintf(speedStr, sizeof(speedStr), "C%d", cardInfo.speedClass);
// 				}

// 				if(cardInfo.appPerfClass > 0) {
// 					// Application Performance Classes [Min. Random Read; Min Random Write; Min. Sustained Sequetial Write]
// 					// IOPS: Number of 4 KByte read or write commands per second
// 					// A1 [1500 IOPS;  500 IOPS; 10 MB/s]
// 					// A2 [4000 IOPS; 2000 IOPS; 10 MB/s]
// 					if(len > 0) {
// 						len += snprintf(speedStr + len, sizeof(speedStr) - len, ", ");
// 					}
// 					len += snprintf(speedStr + len, sizeof(speedStr) - len, "A%d", cardInfo.appPerfClass);
// 				}

// 				if(len == 0) {
// 					snprintf(speedStr, sizeof(speedStr), "N/A");
// 				}

// 				// Get currently use speed mode
// 				const char* modeStr = "Unknown";
// 				switch(cardInfo.activeMode) {
// 					case SDMMC::BusSpeed::Default:
// 						modeStr = "DS (25MHz)";
// 						break;
// 					case SDMMC::BusSpeed::HighSpeed:
// 						modeStr = "HS (50MHz)";
// 						break;
// 					case SDMMC::BusSpeed::UHS_SDR12:
// 						modeStr = "SDR12 (25MHz)";
// 						break;
// 					case SDMMC::BusSpeed::UHS_SDR25:
// 						modeStr = "SDR25 (50MHz)";
// 						break;
// 					case SDMMC::BusSpeed::UHS_SDR50:
// 						modeStr = "SDR50 (100MHz)";
// 						break;
// 					case SDMMC::BusSpeed::UHS_SDR104:
// 						modeStr = "SDR104 (208MHz)";
// 						break;
// 					case SDMMC::BusSpeed::UHS_DDR50:
// 						modeStr = "DDR50 (50MHz)";
// 						break;
// 					default:
// 						modeStr = "Unknown";
// 						break;
// 				}

// 				// Get currently used bus width
// 				uint8_t busWidth = 1;
// 				if(cardInfo.currentBusWidth == 2) {
// 					busWidth = 4;
// 				}
				
// 				LOG_INFO("uSD Card Specs: %s %.2f GB (%s) | %d-bit %s", cardInfo.cardName, sizeGB, speedStr, busWidth, modeStr);

// 				LOG_INFO("uSD Start R/W Test");

// 				// uSD Card R/W Test
// 				uint32_t i;
// 				for(i = 0; i < buffLen; i++) {
// 					dataW[i] = (uint8_t)(i);
// 				}
// 				memset(dataR, 0x55, sizeof(dataR));

// 				uint32_t testSector = 2000; 
// 				uint32_t numBlocks = buffLen / 512;
// 				uint16_t repeats = 100;

// 				// uSD write speed test
// 				uint32_t timestamp = Time::GetUs();
// 				for(i = 0; i < repeats; i++) {
// 					sdCard.WriteBlocks(testSector, dataW, numBlocks);
// 				}
// 				uint32_t deltaTime = Time::GetUs() - timestamp;
// 				float speed = (repeats * buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
// 				LOG_INFO("uSD Write: %d Bytes in %d us (%.2f MByte/s)", (repeats * buffLen), deltaTime, speed);

// 				// uSD read speed test
// 				timestamp = Time::GetUs();
// 				for(i = 0; i < repeats; i++) {
// 					sdCard.ReadBlocks(testSector, dataR, numBlocks);
// 				}
// 				deltaTime = Time::GetUs() - timestamp;
// 				speed = (repeats * buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
// 				LOG_INFO("uSD Read: %d Bytes in %d us (%.2f MByte/s)", (repeats * buffLen), deltaTime, speed);

// 				// // Verify
// 				// bool verifyPass = true;
// 				// for(i = 0; i < buffLen; i++) {
// 				// 	if(uSDDataR[i] != uSDDataW[i]) {
// 				// 		verifyPass = false;
// 				// 		break;
// 				// 	}
// 				// }
// 				// if(verifyPass == true) {
// 				// 	LOG_INFO("uSD Verify OK: Data Matches!");
// 				// }
// 				// else {
// 				// 	LOG_WARN("uSD Verify FAILED: Data Mismatch at %d", i);
// 				// }

// 				// // Erase test
// 				// if(sdCard.Erase(testSector, testSector + numBlocks - 1)) {
// 				// 	LOG_INFO("uSD Erase Command Sent OK");
// 				// 	// Read back to confirm erase (Should be 0x00 or 0xFF depending on card)
// 				// 	memset(uSDDataR, 0x55, sizeof(uSDDataR)); // Fill with dummy
// 				// 	sdCard.ReadBlocks(testSector, uSDDataR, numBlocks);

// 				// 	// Check if all bytes are 0x00 or 0xFF
// 				// 	bool isZero = true;
// 				// 	bool isFF = true;
// 				// 	for(int i = 0; i < sizeof(uSDDataR); i++) {
// 				// 		if(uSDDataR[i] != 0x00) {
// 				// 			isZero = false;
// 				// 		}
// 				// 		if(uSDDataR[i] != 0xFF){
// 				// 			isFF = false;
// 				// 		}
// 				// 	}

// 				// 	if(isZero || isFF) {
// 				// 		LOG_INFO("uSD Erase Verify OK (Value: 0x%02X)", isZero ? 0x00 : 0xFF);
// 				// 	} 
// 				// 	else {
// 				// 		LOG_WARN("uSD Erase Verify: Data is not uniform 0x00/0xFF");
// 				// 	}
// 				// }
// 				// else {
// 				// 	LOG_WARN("uSD Erase Command FAILED");
// 				// }
// 			}
// 		}
// 		else if(sdDet.Read() == 0x01 && hasCard == true) {
// 			// SD Card removed
// 			hasCard = false;			

// 			ledGreen.Write(1);
// 			LOG_INFO("uSD Card Removed");

// 			sdCard.Reset();
// 		}
// 		tx_thread_sleep(100);
// 	}
// }