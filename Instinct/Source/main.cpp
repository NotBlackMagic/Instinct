#include "main.h"

#include <cstring>
#include <stdio.h>

#include "auxiliaryThread.hpp"
#include "i3c.hpp"
#include "inertialThread.hpp"
#include "status.hpp"
#include "stm32n6xx_ll_dma.h"
#include "tx_pluman6_sd_driver.h"

#include "sdmmc.hpp"
#include "sd.hpp"

#include "tx_api.h"
#include "fx_api.h"

#include "console.hpp"
#include "system.hpp"
#include "hardware.hpp"

#include "logger.hpp"
#include "topics.hpp"

Subscriber<ImuMsg> subIMU;
Subscriber<BaroMsg> subBaro;
Subscriber<MagMsg> subMag;

#define THREADX_BUFFER_POOL_SIZE				10240
alignas(32) static UCHAR tx_byte_pool_buffer[THREADX_BUFFER_POOL_SIZE];
static TX_BYTE_POOL threadBytePool;
static TX_THREAD testThread;
static TX_THREAD uSDThread;
static TX_THREAD i2cThread;
static TX_THREAD fileXThread;

static FX_MEDIA sdMedia;
alignas(32) uint8_t sdMediaPool[32 * 1024];

const uint32_t buffLen = 32 * 1024;
alignas(32) uint8_t dataW[buffLen];
alignas(32) uint8_t dataR[buffLen];

void TestThread(ULONG thread_input) {
	uint32_t i;
	volatile uint32_t errCnt = 0;
	volatile uint64_t timestamp = Time::GetUs();
	volatile uint64_t deltaTime = Time::GetUs() - timestamp;
	float speed = 0;
	uint8_t repeats = 1;

	// Init External PSRAM
	if(externalPSRAM.Init(extRAMConfig) == Status::Ok) {
		LOG_INFO("HyperRAM Init OK: %s.", extRAMConfig.deviceName);
	}
	else {
		LOG_ERR("HyperRAM Init Failed!");
	}

	// Init External Flash
	if(externalFlash.Init(extFlashConfig) == Status::Ok) {
		LOG_INFO("HyperFlash Init OK: %s.", extFlashConfig.deviceName);
	}
	else {
		LOG_ERR("HyperFlash Init Failed!");
	}

	//RAM Test
	for(i = 0; i < buffLen; i++) {
		dataW[i] = (uint8_t)i;
	}
	System::CleanCache((uint32_t*)dataW, buffLen);

	timestamp = Time::GetUs();
	externalPSRAM.Write(0, dataW, buffLen);
	deltaTime = Time::GetUs() - timestamp;
	speed = (buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM Write: %d Bytes in %d us (%.2f MByte/s)", buffLen, deltaTime, speed);

	memset(dataR, 0x55, buffLen);
	timestamp = Time::GetUs();
	externalFlash.Read(0, dataR, buffLen);
	deltaTime = Time::GetUs() - timestamp;

	errCnt = 0;
	for(i = 0; i < buffLen; i++) {
		if(dataR[i] != dataW[i]) {
			errCnt += 1;
		}
	}

	speed = (buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM Read: %d Bytes in %d us (%.2f MByte/s), Err %d", buffLen, deltaTime, speed, errCnt);

	//RAM Memory Mapped Test
	externalPSRAM.EnterMemoryMappedMode();
	void *extRAMPtr = (void*)hyperBus1.GetBaseAddr();

	timestamp = Time::GetUs();
	for(i = 0; i < repeats; i++) {
		memcpy(extRAMPtr, dataW, buffLen);
	}
	deltaTime = Time::GetUs() - timestamp;

	speed = (repeats * buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM MM Write: %d Bytes in %d us (%.2f MByte/s), Err %d", (repeats * buffLen), deltaTime, speed, errCnt);
	
	memset(dataR, 0x55, buffLen);
	timestamp = Time::GetUs();
	for(i = 0; i < repeats; i++) {
		memcpy(dataR, extRAMPtr, buffLen);
	}
	deltaTime = Time::GetUs() - timestamp;

	errCnt = 0;
	for(i = 0; i < buffLen; i++) {
		if(dataR[i] != dataW[i]) {
			errCnt += 1;
		}
	}

	speed = (repeats * buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM MM Read: %d Bytes in %d us (%.2f MByte/s), Err %d", (repeats * buffLen), deltaTime, speed, errCnt);

	// HPDMA Test (PSRAM to SRAM)
	memset(dataR, 0x55, buffLen);

	LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_HPDMA1);
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RISAF);
	// NVIC_SetPriority(HPDMA1_Channel0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	// NVIC_EnableIRQ(HPDMA1_Channel0_IRQn);

	// RIF configuration (AXISRAM2)
	const uint32_t RIF_CID_2 = 0x00000004U | 0x00000002U | 0x00000001U;
	const uint32_t RIF_ATTRIBUTE_SEC = 0x00000001U;
	const uint32_t RIF_CID_NONE = 0x00000000U;
	RISAF3->REG[0].STARTR = 0x0;
	RISAF3->REG[0].ENDR = 0x000FFFFFU;		// 1 MByte area 
	RISAF3->REG[0].CIDCFGR = (RIF_CID_2 | (RIF_CID_2 << RISAF_REGx_CIDCFGR_WRENC0_Pos));
	RISAF3->REG[0].CFGR = (RISAF_REGx_CFGR_BREN | (RIF_ATTRIBUTE_SEC << RISAF_REGx_CFGR_SEC_Pos)
							| (RIF_CID_NONE << RISAF_REGx_CFGR_PRIVC0_Pos));

	// RIF configuration (XSPI1)
	RISAF11->REG[0].STARTR = 0x0;
	RISAF11->REG[0].ENDR = 0x00FFFFFFU;		// 256 MByte area
	RISAF11->REG[0].CIDCFGR = (RIF_CID_2 | (RIF_CID_2 << RISAF_REGx_CIDCFGR_WRENC0_Pos));
	RISAF11->REG[0].CFGR = (RISAF_REGx_CFGR_BREN | (RIF_ATTRIBUTE_SEC << RISAF_REGx_CFGR_SEC_Pos)
							| (RIF_CID_NONE << RISAF_REGx_CFGR_PRIVC0_Pos));

	// Access configuration
	LL_DMA_EnableChannelSecure(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_EnableChannelPrivilege(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_EnableChannelSrcSecure(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_EnableChannelDestSecure(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_SetStaticIsolation(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_CHANNEL_STATIC_CID_2);

	LL_DMA_InitTypeDef hpdmaCnfg;
	LL_DMA_StructInit(&hpdmaCnfg);
	hpdmaCnfg.SrcAddress = (uint32_t)hyperBus1.GetBaseAddr();
	hpdmaCnfg.DestAddress = (uint32_t)dataR;

	hpdmaCnfg.Direction = LL_DMA_DIRECTION_MEMORY_TO_MEMORY;
	hpdmaCnfg.BlkHWRequest = LL_DMA_HWREQUEST_SINGLEBURST;

	hpdmaCnfg.SrcDataWidth = LL_DMA_SRC_DATAWIDTH_WORD;
	hpdmaCnfg.DestDataWidth = LL_DMA_DEST_DATAWIDTH_WORD;
	hpdmaCnfg.DataAlignment = LL_DMA_DATA_ALIGN_ZEROPADD;

	hpdmaCnfg.SrcIncMode = LL_DMA_SRC_INCREMENT;
	hpdmaCnfg.DestIncMode = LL_DMA_DEST_INCREMENT;

	hpdmaCnfg.SrcBurstLength = 2;
	hpdmaCnfg.DestBurstLength = 2;
	hpdmaCnfg.Priority = LL_DMA_HIGH_PRIORITY;

	hpdmaCnfg.BlkDataLength = buffLen;

	hpdmaCnfg.SrcAllocatedPort = LL_DMA_SRC_ALLOCATED_PORT0;
	hpdmaCnfg.DestAllocatedPort = LL_DMA_DEST_ALLOCATED_PORT0;

	hpdmaCnfg.TransferEventMode = LL_DMA_TCEM_BLK_TRANSFER;
	hpdmaCnfg.Mode = LL_DMA_NORMAL;

	LL_DMA_Init(HPDMA1, LL_DMA_CHANNEL_12, &hpdmaCnfg);

	LL_DMA_ClearFlag_TC(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_ClearFlag_HT(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_ClearFlag_DTE(HPDMA1, LL_DMA_CHANNEL_12);

	timestamp = Time::GetUs();
	LL_DMA_EnableChannel(HPDMA1, LL_DMA_CHANNEL_12);
	DMA_Channel_TypeDef* dmaChannel = ((DMA_Channel_TypeDef *)((uint32_t)HPDMA1 + LL_DMA_CH_OFFSET_TAB[LL_DMA_CHANNEL_12]));
	uint32_t dmaStatus = dmaChannel->CSR;
	do {
		dmaStatus = dmaChannel->CSR;
	}
	while((dmaStatus & DMA_CSR_TCF) != DMA_CSR_TCF);
	deltaTime = Time::GetUs() - timestamp;

	System::InvalidateCache((uint32_t*)dataR, buffLen);

	errCnt = 0;
	for(i = 0; i < buffLen; i++) {
		if(dataR[i] != dataW[i]) {
			errCnt += 1;
		}
	}

	speed = (buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM DMA Read: %d Bytes in %d us (%.2f MByte/s), Err %d", (buffLen), deltaTime, speed, errCnt);
	
	//Flash Test
	uint32_t flashAddr = 0;
	for(i = 0; i < buffLen; i++) {
		dataW[i] = (uint8_t)i;
	}
	externalFlash.SectorErase(flashAddr);

	timestamp = Time::GetUs();
	externalFlash.Program(flashAddr, dataW, buffLen);
	deltaTime = Time::GetUs() - timestamp;
	speed = (buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("Flash Write: %d Bytes in %d us (%.2f MByte/s)", buffLen, deltaTime, speed);

	memset(dataR, 0x55, buffLen);
	timestamp = Time::GetUs();
	externalFlash.Read(flashAddr, dataR, buffLen);
	deltaTime = Time::GetUs() - timestamp;
	speed = (buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("Flash Read: %d Bytes in %d us (%.2f MByte/s), Err %d", buffLen, deltaTime, speed, errCnt);

	errCnt = 0;
	for(i = 0; i < buffLen; i++) {
		if(dataR[i] != dataW[i]) {
			errCnt += 1;
		}
	}

	//Flash Memory Mapped Test
	externalFlash.EnterMemoryMappedMode();
	void *extFlashPtr = (void*)hyperBus2.GetBaseAddr();

	memset(dataR, 0x55, buffLen);
	timestamp = Time::GetUs();
	for(i = 0; i < repeats; i++) {
		memcpy(dataR, extFlashPtr, buffLen);
	}
	deltaTime = Time::GetUs() - timestamp;

	errCnt = 0;
	for(i = 0; i < buffLen; i++) {
		if(dataR[i] != dataW[i]) {
			errCnt += 1;
		}
	}

	speed = (repeats * buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("Flash MM Read: %d Bytes in %d us (%.2f MByte/s), Err %d", (repeats * buffLen), deltaTime, speed, errCnt);

	// HPDMA Test (HyperFlash to SRAM)
	memset(dataR, 0x55, buffLen);

	// RIF configuration (XSPI2)
	RISAF12->REG[0].STARTR = 0x0;
	RISAF12->REG[0].ENDR = 0x00FFFFFFU;		// 256 MByte area
	RISAF12->REG[0].CIDCFGR = (RIF_CID_2 | (RIF_CID_2 << RISAF_REGx_CIDCFGR_WRENC0_Pos));
	RISAF12->REG[0].CFGR = (RISAF_REGx_CFGR_BREN | (RIF_ATTRIBUTE_SEC << RISAF_REGx_CFGR_SEC_Pos)
							| (RIF_CID_NONE << RISAF_REGx_CFGR_PRIVC0_Pos));

	// Re-setup HPDMA
	LL_DMA_SetDestAddress(HPDMA1, LL_DMA_CHANNEL_12, (uint32_t)dataR);
	LL_DMA_SetSrcAddress(HPDMA1, LL_DMA_CHANNEL_12, (uint32_t)hyperBus2.GetBaseAddr());
	LL_DMA_SetBlkDataLength(HPDMA1, LL_DMA_CHANNEL_12, buffLen);

	LL_DMA_ClearFlag_TC(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_ClearFlag_HT(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_ClearFlag_DTE(HPDMA1, LL_DMA_CHANNEL_12);

	timestamp = Time::GetUs();
	LL_DMA_EnableChannel(HPDMA1, LL_DMA_CHANNEL_12);
	dmaChannel = ((DMA_Channel_TypeDef *)((uint32_t)HPDMA1 + LL_DMA_CH_OFFSET_TAB[LL_DMA_CHANNEL_12]));
	dmaStatus = dmaChannel->CSR;
	do {
		dmaStatus = dmaChannel->CSR;
	}
	while((dmaStatus & DMA_CSR_TCF) != DMA_CSR_TCF);
	deltaTime = Time::GetUs() - timestamp;

	System::InvalidateCache((uint32_t*)dataR, buffLen);

	errCnt = 0;
	for(i = 0; i < buffLen; i++) {
		if(dataR[i] != dataW[i]) {
			errCnt += 1;
		}
	}

	speed = (buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("Flash DMA Read: %d Bytes in %d us (%.2f MByte/s), Err %d", (buffLen), deltaTime, speed, errCnt);

	while(1) {
		ledBlue.Toggle();
		tx_thread_sleep(100);
	}
}

// ATTENTION: PlumaN6 HW Rev. 1 does NOT support 1V8 voltage switch for uSD card due to routing error!
const SD::Config sdConfig = {.use4BitMode = true, .use1V8Level = false, .useHighSpeed = true, .useUHS = false, .vioSelectPin = &sdVioSel};
SD sdCard = SD(sdmmc1);

void SDMMCThread(ULONG thread_input) {
	sdVioSel.Write(0);		//SD VIO Selection: 0 -> 3V3, 1 -> 1V8
	
	bool hasCard = false;
	while(1) {
		if(sdDet.Read() == 0x00 && hasCard == false) {
			// SD Card inserted/detected
			hasCard = true;

			ledGreen.Write(0);
			LOG_INFO("uSD Card Inserted");

			// Delay a bit before try to access card
			tx_thread_sleep(100);

			if(sdCard.Init(sdConfig) != Status::Ok) {
				LOG_WARN("uSD Card Init FAILED");
			}
			else {
				SD::CardInfo cardInfo = sdCard.GetCardInfo();
				LOG_INFO("uSD Card Initialized");

				// Calculate SD card size in Gbytes
				float sizeGB = (float)cardInfo.sizeBytes / (1024.0f * 1024.0f * 1024.0f);

				// Get card type/class as a string
				uint8_t len = 0;
				char speedStr[16] = "\0";
				if(cardInfo.videoSpeedClass > 0) {
					// Video Speed Classes [Min. Continuouse Write Speeds]: 
					// V6  [6 MB/s]
					// V10 [10 MB/s]
					// V30 [30 MB/s]
					// V60 [60 MB/s]
					// V90 [90 MB/s]
					len += snprintf(speedStr, sizeof(speedStr), "V%d", cardInfo.videoSpeedClass);
				}
				else if(cardInfo.uhsSpeedGrade > 0) {
					// UHS Speed Classes [Min. Continuouse Write Speeds]:
					// U1 [10 MB/s]
					// U3 [30 MB/s]
					len += snprintf(speedStr, sizeof(speedStr), "U%d", cardInfo.uhsSpeedGrade);
				}
				else {
					// Speed Classes [Min. Continuouse Write Speeds]:
					// C2  [2 MB/s]
					// C4  [4 MB/s]
					// C6  [6 MB/s]
					// C10 [10 MB/s]
					len += snprintf(speedStr, sizeof(speedStr), "C%d", cardInfo.speedClass);
				}

				if(cardInfo.appPerfClass > 0) {
					// Application Performance Classes [Min. Random Read; Min Random Write; Min. Sustained Sequetial Write]
					// IOPS: Number of 4 KByte read or write commands per second
					// A1 [1500 IOPS;  500 IOPS; 10 MB/s]
					// A2 [4000 IOPS; 2000 IOPS; 10 MB/s]
					if(len > 0) {
						len += snprintf(speedStr + len, sizeof(speedStr) - len, ", ");
					}
					len += snprintf(speedStr + len, sizeof(speedStr) - len, "A%d", cardInfo.appPerfClass);
				}

				if(len == 0) {
					snprintf(speedStr, sizeof(speedStr), "N/A");
				}

				// Get currently use speed mode
				const char* modeStr = "Unknown";
				switch(cardInfo.activeMode) {
					case SDMMC::BusSpeed::Default:
						modeStr = "DS (25MHz)";
						break;
					case SDMMC::BusSpeed::HighSpeed:
						modeStr = "HS (50MHz)";
						break;
					case SDMMC::BusSpeed::UHS_SDR12:
						modeStr = "SDR12 (25MHz)";
						break;
					case SDMMC::BusSpeed::UHS_SDR25:
						modeStr = "SDR25 (50MHz)";
						break;
					case SDMMC::BusSpeed::UHS_SDR50:
						modeStr = "SDR50 (100MHz)";
						break;
					case SDMMC::BusSpeed::UHS_SDR104:
						modeStr = "SDR104 (208MHz)";
						break;
					case SDMMC::BusSpeed::UHS_DDR50:
						modeStr = "DDR50 (50MHz)";
						break;
					default:
						modeStr = "Unknown";
						break;
				}

				// Get currently used bus width
				uint8_t busWidth = 1;
				if(cardInfo.currentBusWidth == 2) {
					busWidth = 4;
				}
				
				LOG_INFO("uSD Card Specs: %s %.2f GB (%s) | %d-bit %s", cardInfo.cardName, sizeGB, speedStr, busWidth, modeStr);

				LOG_INFO("uSD Start R/W Test");

				// uSD Card R/W Test
				uint32_t i;
				for(i = 0; i < buffLen; i++) {
					dataW[i] = (uint8_t)(i);
				}
				memset(dataR, 0x55, sizeof(dataR));

				uint32_t testSector = 2000; 
				uint32_t numBlocks = buffLen / 512;
				uint16_t repeats = 100;

				// uSD write speed test
				uint32_t timestamp = Time::GetUs();
				for(i = 0; i < repeats; i++) {
					sdCard.WriteBlocks(testSector, dataW, numBlocks);
				}
				uint32_t deltaTime = Time::GetUs() - timestamp;
				float speed = (repeats * buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
				LOG_INFO("uSD Write: %d Bytes in %d us (%.2f MByte/s)", (repeats * buffLen), deltaTime, speed);

				// uSD read speed test
				timestamp = Time::GetUs();
				for(i = 0; i < repeats; i++) {
					sdCard.ReadBlocks(testSector, dataW, numBlocks);
				}
				deltaTime = Time::GetUs() - timestamp;
				speed = (repeats * buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
				LOG_INFO("uSD Read: %d Bytes in %d us (%.2f MByte/s)", (repeats * buffLen), deltaTime, speed);

				// // Verify
				// bool verifyPass = true;
				// for(i = 0; i < buffLen; i++) {
				// 	if(uSDDataR[i] != uSDDataW[i]) {
				// 		verifyPass = false;
				// 		break;
				// 	}
				// }
				// if(verifyPass == true) {
				// 	LOG_INFO("uSD Verify OK: Data Matches!");
				// }
				// else {
				// 	LOG_WARN("uSD Verify FAILED: Data Mismatch at %d", i);
				// }

				// // Erase test
				// if(sdCard.Erase(testSector, testSector + numBlocks - 1)) {
				// 	LOG_INFO("uSD Erase Command Sent OK");
				// 	// Read back to confirm erase (Should be 0x00 or 0xFF depending on card)
				// 	memset(uSDDataR, 0x55, sizeof(uSDDataR)); // Fill with dummy
				// 	sdCard.ReadBlocks(testSector, uSDDataR, numBlocks);

				// 	// Check if all bytes are 0x00 or 0xFF
				// 	bool isZero = true;
				// 	bool isFF = true;
				// 	for(int i = 0; i < sizeof(uSDDataR); i++) {
				// 		if(uSDDataR[i] != 0x00) {
				// 			isZero = false;
				// 		}
				// 		if(uSDDataR[i] != 0xFF){
				// 			isFF = false;
				// 		}
				// 	}

				// 	if(isZero || isFF) {
				// 		LOG_INFO("uSD Erase Verify OK (Value: 0x%02X)", isZero ? 0x00 : 0xFF);
				// 	} 
				// 	else {
				// 		LOG_WARN("uSD Erase Verify: Data is not uniform 0x00/0xFF");
				// 	}
				// }
				// else {
				// 	LOG_WARN("uSD Erase Command FAILED");
				// }
			}
		}
		else if(sdDet.Read() == 0x01 && hasCard == true) {
			// SD Card removed
			hasCard = false;			

			ledGreen.Write(1);
			LOG_INFO("uSD Card Removed");

			sdCard.Reset();
		}
		tx_thread_sleep(100);
	}
}

void I2CThread(ULONG thread_input) {
	ImuMsg imuLocal;
	topicImu.Subscribe(&subIMU);

	BaroMsg baroLocal;
	topicBaro.Subscribe(&subBaro);

	MagMsg magLocal;
	topicMag.Subscribe(&subMag);

	//Initialize
	if(i2c2.Probe(0x44) == 0x01) {
		ina700.Init();
	}

	uint8_t i3cTxBuf[8];
	uint8_t i3cRxBuf[8];
	i3cTxBuf[0] = 0x0F;
	i3c2.TransferAsync(0x0C, I3C::TargetType::I2C, i3cTxBuf, 1, i3cRxBuf, 1);
	i3c2.TransferWait(1000);

	uint16_t ina700ID;
	ina700.ReadID(ina700ID);
	LOG_INFO("INA ID: 0x%04X", ina700ID);

	float voltage, current, temperature;
	while(1) {
		ina700.ReadCurrent(current);
		ina700.ReadVoltage(voltage);
		ina700.ReadTemperature(temperature);

		// LOG_INFO("PW: %dmV %dmA %dC", (int)(voltage * 1000), (int)(current * 1000), (int)temperature);

		if(topicMag.Take(&subMag, magLocal, TX_NO_WAIT) == true) {
			// LOG_INFO("MAG: x: %d y: %d z: %d %dC", (int)(magLocal.values[0]), (int)(magLocal.values[1]), (int)(magLocal.values[2]), (int)(magLocal.temperature));
		}

		if(topicImu.Take(&subIMU, imuLocal, TX_NO_WAIT) == true) {
			// LOG_INFO("ACCEL: x: %d y: %d z: %d %dC", (int)(imuLocal.accel[0]), (int)(imuLocal.accel[1]), (int)(imuLocal.accel[2]), (int)(imuLocal.temperature));
		}

		if(topicBaro.Take(&subBaro, baroLocal, TX_NO_WAIT) == true) {
			// LOG_INFO("BARO: %d Pa %dC", (int)(baroLocal.pressure), (int)(baroLocal.temperature));
		}

		tx_thread_sleep(1000);
	}
}

void FileXThread(ULONG thread_input) {
	uint32_t status;

	sdVioSel.Write(0);		//SD VIO Selection: 0 -> 3V3, 1 -> 1V8

	LOG_INFO("FileX Started");

	uint8_t sdCardStatus = 0;
	while(1) {
		switch(sdCardStatus) {
			case 0: {
				//Wait for card
				if(sdDet.Read() == 0) {
					LOG_INFO("FileX Card Detected.");
					_tx_thread_sleep(20);
					sdCardStatus = 1;
				}
				break;
			}
			case 1: {
				//Card just inserted
				LOG_INFO("FileX Mount volume.");

				//Mount SD Card
				status = fx_media_open(	&sdMedia,				// Media Ptr
										const_cast<char*>("SD CARD"),		// Name
										fx_stm32_sd_driver,	// Driver Entry
										0,					// Driver Info Ptr (Unused)
										sdMediaPool,			// Memory Pool
										sizeof(sdMediaPool));	// Pool Size
				
				if(status == FX_SUCCESS) {
					ledGreen.Write(0);
					sdCardStatus = 2;

					ULONG available_bytes;
					fx_media_space_available(&sdMedia, &available_bytes);
					LOG_INFO("FileX uSD Free Space: %lu MB\n", available_bytes / (1024 * 1024));
				}
				else {
					LOG_INFO("FileX Mount Failed!");
					
					if(status == FX_BOOT_ERROR || status == FX_MEDIA_INVALID) {
						LOG_INFO("FileX Card needs formatting.");

						// Formatting logic
						uint32_t total_sectors = 7829504;	//sdCard.GetCardInfo().blockCount;

						status = fx_media_format(	&sdMedia,		// Media Ptr
													fx_stm32_sd_driver,	// Driver Entry
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

					fx_media_close(&sdMedia);

					ledGreen.Write(1);
					sdCardStatus = 0;
				}
				else {
					tx_thread_sleep(50);
				}
				break;
			}
			case 3: {
				tx_thread_sleep(500);
			}
		}
		tx_thread_sleep(10);
	}
}

void tx_application_define(void *first_unused_memory) {
	uint32_t status = TX_SUCCESS;
	char *pointer;

	Logger::Instance().Init();
	Logger::Instance().RegisterConsole(&uart4);

	LOG_INFO("--------------------------------");
	LOG_INFO("System Booting...");
	LOG_INFO("Logger Initialized.");

	//Start Hardware stuff here, uses RTOS objects
	HardwareInit();
	LOG_INFO("Peripherals Initialized.");

	//Start system threads
	Console::Init(&uart4);

	//Start application threads
	InertialThread::Init();
	AuxiliaryThread::Init();

	//Create a byte memory pool from which to allocate the thread stacks
	status = tx_byte_pool_create(&threadBytePool, const_cast<char*>("Static Thread Byte Pool"), tx_byte_pool_buffer, THREADX_BUFFER_POOL_SIZE);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Create Byte Pool Failed.");
	}

	//Create the TestThread
	//Allocate the stack
	status = tx_byte_allocate(&threadBytePool, (VOID**) &pointer, 2048, TX_NO_WAIT);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Stack 0 Allocate Failed.");
	}
	//Create thread
	status = tx_thread_create(&testThread, const_cast<char*>("Test Thread"),
											TestThread, 0,
											pointer, 2048,
											0, 0,
											TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Test Thread Create Failed.");
	}
	
	//Allocate the stack
	status = tx_byte_allocate(&threadBytePool, (VOID**) &pointer, 2048, TX_NO_WAIT);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Stack 1 Allocate Failed.");
	}
	//Create thread
	status = tx_thread_create(&uSDThread, const_cast<char*>("uSD Thread"),
											SDMMCThread, 0,
											pointer, 2048,
											2, 0,
											TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX uSD Thread Create Failed.");
	}
	
	//Allocate the stack
	status = tx_byte_allocate(&threadBytePool, (VOID**) &pointer, 2048, TX_NO_WAIT);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Stack 2 Allocate Failed.");
	}
	//Create thread
	status = tx_thread_create(&i2cThread, const_cast<char*>("I2C Thread"),
											I2CThread, 0,
											pointer, 2048,
											3, 0,
											TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX I2C Thread Create Failed.");
	}

	// //Allocate the stack
	// status = tx_byte_allocate(&threadBytePool, (VOID**) &pointer, 2048, TX_NO_WAIT);
	// if(status != TX_SUCCESS) {
	// 	LOG_ERR("ThreadX Stack 3 Allocate Failed.");
	// }
	// //FileX stuff
	// fx_system_initialize();
	// status = tx_thread_create(&fileXThread, const_cast<char*>("FileX Thread"),
	// 										FileXThread, 0,
	// 										pointer, 2048,
	// 										0, 0,
	// 										TX_NO_TIME_SLICE, TX_AUTO_START);
	// if(status != TX_SUCCESS) {
	// 	LOG_ERR("ThreadX FileX Thread Create Failed.");
	// }
}

int main(void) {
	//MCU Configuration
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	// HAL_Init();
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);
	System::EnableCache();

	SystemCoreClockUpdate();
	System::InitSysTick();

	System::InitClock();
	System::InitSysTick();
	Time::Init();

	//Enable debugger in flash run mode
	System::EnableDebug();

	//Launch the application
	/*
	if (BOOT_OK != BOOT_Application()) {
		Error_Handler();
	}
	*/

	tx_kernel_enter();

	//We should never get here as control is now taken by the scheduler
	while (1) {
		ledRed.Toggle();
		Time::Delay(200);
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}
