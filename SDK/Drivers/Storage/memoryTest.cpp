/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	SDK/Drivers/Storage/memoryTest.cpp
 */

#include "memoryTest.hpp"

const uint32_t benchBuffLen = 16 * 1024;
alignas(32) static uint8_t dataW[benchBuffLen];
alignas(32) static uint8_t dataR[benchBuffLen];

Status MemoryTest::RunQuickTest(uint32_t sizeBytes) {
	volatile uint32_t* extRam = reinterpret_cast<volatile uint32_t*>(hyperBus1.GetBaseAddr());
	uint32_t wordCount = sizeBytes / sizeof(uint32_t);

	for(uint32_t i = 0; i < wordCount; i++) {
		extRam[i] = 0xAA55AA55 ^ i;
	}

	System::CleanCache((uint32_t*)extRam, sizeBytes);
	System::InvalidateCache((uint32_t*)extRam, sizeBytes);

	for(uint32_t i = 0; i < wordCount; i++) {
		if(extRam[i] != (0xAA55AA55 ^ i)) {
			return Status::Error;
		}
	}
	return Status::Ok;
}

void MemoryTest::RunBenchmark(uint32_t sizeBytes) {
	if(sizeBytes == 0 || sizeBytes > benchBuffLen) {
		sizeBytes = benchBuffLen;
	}

	uint32_t i;
	volatile uint32_t errCnt = 0;
	volatile uint64_t timestamp = Time::GetUs();
	volatile uint64_t deltaTime = Time::GetUs() - timestamp;
	float speed = 0;
	uint8_t repeats = 1;

	// Exit Memory Mapped mode for direct indirect Read/Write tests
	externalPSRAM.ExitMemoryMappedMode();

	// RAM Test
	for(i = 0; i < sizeBytes; i++) {
		dataW[i] = (uint8_t)i;
	}
	System::CleanCache((uint32_t*)dataW, sizeBytes);

	timestamp = Time::GetUs();
	externalPSRAM.Write(0, dataW, sizeBytes);
	deltaTime = Time::GetUs() - timestamp;
	speed = (sizeBytes) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM Write: %d Bytes in %d us (%.2f MByte/s)", sizeBytes, deltaTime, speed);

	memset(dataR, 0x55, sizeBytes);
	timestamp = Time::GetUs();
	externalPSRAM.Read(0, dataR, sizeBytes);
	deltaTime = Time::GetUs() - timestamp;

	errCnt = 0;
	for(i = 0; i < sizeBytes; i++) {
		if(dataR[i] != dataW[i]) {
			volatile uint8_t readError = dataR[i];
			errCnt += 1;
		}
	}

	speed = (sizeBytes) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM Read: %d Bytes in %d us (%.2f MByte/s), Err %d", sizeBytes, deltaTime, speed, errCnt);

	// Enter Memory Mapped Mode for memcpy & DMA tests
	externalPSRAM.EnterMemoryMappedMode();
	void *extRAMPtr = (void*)hyperBus1.GetBaseAddr();

	timestamp = Time::GetUs();
	for(i = 0; i < repeats; i++) {
		memcpy(extRAMPtr, dataW, sizeBytes);
	}
	deltaTime = Time::GetUs() - timestamp;

	speed = (repeats * sizeBytes) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM MM Write: %d Bytes in %d us (%.2f MByte/s)", (repeats * sizeBytes), deltaTime, speed);
	
	System::CleanCache((uint32_t*)extRAMPtr, sizeBytes);
	System::InvalidateCache((uint32_t*)extRAMPtr, sizeBytes);
	memset(dataR, 0x55, sizeBytes);
	timestamp = Time::GetUs();
	for(i = 0; i < repeats; i++) {
		memcpy(dataR, extRAMPtr, sizeBytes);
	}
	deltaTime = Time::GetUs() - timestamp;

	errCnt = 0;
	for(i = 0; i < sizeBytes; i++) {
		if(dataR[i] != dataW[i]) {
			volatile uint8_t readError = dataR[i];
			errCnt += 1;
		}
	}

	speed = (repeats * sizeBytes) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM MM Read: %d Bytes in %d us (%.2f MByte/s), Err %d", (repeats * sizeBytes), deltaTime, speed, errCnt);

	// HPDMA Test (PSRAM to SRAM)
	memset(dataR, 0x55, sizeBytes);

	System::CleanCache((uint32_t*)hyperBus1.GetBaseAddr(), sizeBytes);
	System::CleanCache((uint32_t*)dataR, sizeBytes);

	LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_HPDMA1);
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RISAF);

	// RIF configuration
	const uint32_t RIF_CID = 0x0F;	// Allow ALL i.e RW for everyone
	const uint32_t RIF_ATTRIBUTE_SEC = 0x00000001U;
	const uint32_t RIF_CID_NONE = 0x00000000U;

	// RIF configuration (CPUAXI RAM0: AXISRAM1 without FLEXMEM part)
	RISAF2->REG[0].STARTR = 0x0;
	RISAF2->REG[0].ENDR = 0xFFFFFFFFU;		// Full region
	RISAF2->REG[0].CIDCFGR = (RIF_CID | (RIF_CID << RISAF_REGx_CIDCFGR_WRENC0_Pos));
	RISAF2->REG[0].CFGR = (RISAF_REGx_CFGR_BREN | (RIF_ATTRIBUTE_SEC << RISAF_REGx_CFGR_SEC_Pos)
							| (RIF_CID_NONE << RISAF_REGx_CFGR_PRIVC0_Pos));

	// RIF configuration (CPUAXI RAM1: AXISRAM2)
	RISAF3->REG[0].STARTR = 0x0;
	RISAF3->REG[0].ENDR = 0xFFFFFFFFU;		// Full region
	RISAF3->REG[0].CIDCFGR = (RIF_CID | (RIF_CID << RISAF_REGx_CIDCFGR_WRENC0_Pos));
	RISAF3->REG[0].CFGR = (RISAF_REGx_CFGR_BREN | (RIF_ATTRIBUTE_SEC << RISAF_REGx_CFGR_SEC_Pos)
							| (RIF_CID_NONE << RISAF_REGx_CFGR_PRIVC0_Pos));

	// Access configuration
	LL_DMA_EnableChannelSecure(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_EnableChannelPrivilege(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_EnableChannelSrcSecure(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_EnableChannelDestSecure(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_SetStaticIsolation(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_CHANNEL_STATIC_CID_0);

	LL_DMA_SetSrcAddress(HPDMA1, LL_DMA_CHANNEL_12, (uint32_t)hyperBus1.GetBaseAddr());
	LL_DMA_SetDestAddress(HPDMA1, LL_DMA_CHANNEL_12, (uint32_t)dataR);

	LL_DMA_SetDataTransferDirection(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_DIRECTION_MEMORY_TO_MEMORY);
	LL_DMA_SetBlkHWRequest(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_HWREQUEST_SINGLEBURST);

	LL_DMA_SetSrcDataWidth(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_SRC_DATAWIDTH_WORD);
	LL_DMA_SetDestDataWidth(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_DEST_DATAWIDTH_WORD);
	LL_DMA_SetDataAlignment(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_DATA_ALIGN_ZEROPADD);

	LL_DMA_SetSrcIncMode(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_SRC_INCREMENT);
	LL_DMA_SetDestIncMode(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_DEST_INCREMENT);

	LL_DMA_SetSrcBurstLength(HPDMA1, LL_DMA_CHANNEL_12, 2);
	LL_DMA_SetDestBurstLength(HPDMA1, LL_DMA_CHANNEL_12, 2);
	LL_DMA_SetChannelPriorityLevel(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_HIGH_PRIORITY);

	LL_DMA_SetBlkDataLength(HPDMA1, LL_DMA_CHANNEL_12, sizeBytes);

	LL_DMA_SetSrcAllocatedPort(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_SRC_ALLOCATED_PORT0);
	LL_DMA_SetDestAllocatedPort(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_DEST_ALLOCATED_PORT0);
	
	LL_DMA_SetTransferEventMode(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_TCEM_BLK_TRANSFER);
	LL_DMA_SetTransferMode(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_NORMAL);

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

	System::InvalidateCache((uint32_t*)dataR, sizeBytes);

	errCnt = 0;
	for(i = 0; i < sizeBytes; i++) {
		if(dataR[i] != dataW[i]) {
			volatile uint8_t readError = dataR[i];
			errCnt += 1;
		}
	}

	speed = (sizeBytes) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM DMA Read: %d Bytes in %d us (%.2f MByte/s), Err %d", (sizeBytes), deltaTime, speed, errCnt);
	
	// // Flash Test
	// uint32_t flashAddr = 0;
	// for(i = 0; i < sizeBytes; i++) {
	// 	dataW[i] = (uint8_t)i;
	// }
	// status = externalFlash.SectorErase(flashAddr);

	// timestamp = Time::GetUs();
	// status = externalFlash.Program(flashAddr, dataW, sizeBytes);
	// deltaTime = Time::GetUs() - timestamp;
	// speed = (sizeBytes) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	// LOG_INFO("Flash Write: %d Bytes in %d us (%.2f MByte/s)", sizeBytes, deltaTime, speed);

	// memset(dataR, 0x55, sizeBytes);
	// timestamp = Time::GetUs();
	// status = externalFlash.Read(flashAddr, dataR, sizeBytes);
	// deltaTime = Time::GetUs() - timestamp;

	// errCnt = 0;
	// for(i = 0; i < sizeBytes; i++) {
	// 	if(dataR[i] != dataW[i]) {
	// 		errCnt += 1;
	// 	}
	// }

	// speed = (sizeBytes) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	// LOG_INFO("Flash Read: %d Bytes in %d us (%.2f MByte/s), Err %d", sizeBytes, deltaTime, speed, errCnt);

	// // Flash Memory Mapped Test
	// status = externalFlash.EnterMemoryMappedMode();
	// void *extFlashPtr = (void*)hyperBus2.GetBaseAddr();

	// memset(dataR, 0x55, sizeBytes);
	// timestamp = Time::GetUs();
	// for(i = 0; i < repeats; i++) {
	// 	memcpy(dataR, extFlashPtr, sizeBytes);
	// }
	// deltaTime = Time::GetUs() - timestamp;

	// errCnt = 0;
	// for(i = 0; i < sizeBytes; i++) {
	// 	if(dataR[i] != dataW[i]) {
	// 		errCnt += 1;
	// 	}
	// }

	// speed = (repeats * sizeBytes) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	// LOG_INFO("Flash MM Read: %d Bytes in %d us (%.2f MByte/s), Err %d", (repeats * sizeBytes), deltaTime, speed, errCnt);

	// // HPDMA Test (HyperFlash to SRAM)
	// memset(dataR, 0x55, sizeBytes);
	// System::CleanCache((uint32_t*)hyperBus2.GetBaseAddr(), sizeBytes);
	// System::CleanCache((uint32_t*)dataR, sizeBytes);

	// // Re-setup HPDMA
	// LL_DMA_SetDestAddress(HPDMA1, LL_DMA_CHANNEL_12, (uint32_t)dataR);
	// LL_DMA_SetSrcAddress(HPDMA1, LL_DMA_CHANNEL_12, (uint32_t)hyperBus2.GetBaseAddr());
	// LL_DMA_SetBlkDataLength(HPDMA1, LL_DMA_CHANNEL_12, sizeBytes);

	// LL_DMA_ClearFlag_TC(HPDMA1, LL_DMA_CHANNEL_12);
	// LL_DMA_ClearFlag_HT(HPDMA1, LL_DMA_CHANNEL_12);
	// LL_DMA_ClearFlag_DTE(HPDMA1, LL_DMA_CHANNEL_12);

	// timestamp = Time::GetUs();
	// LL_DMA_EnableChannel(HPDMA1, LL_DMA_CHANNEL_12);
	// dmaChannel = ((DMA_Channel_TypeDef *)((uint32_t)HPDMA1 + LL_DMA_CH_OFFSET_TAB[LL_DMA_CHANNEL_12]));
	// dmaStatus = dmaChannel->CSR;
	// do {
	// 	dmaStatus = dmaChannel->CSR;
	// }
	// while((dmaStatus & DMA_CSR_TCF) != DMA_CSR_TCF);
	// deltaTime = Time::GetUs() - timestamp;

	// System::InvalidateCache((uint32_t*)dataR, sizeBytes);

	// errCnt = 0;
	// for(i = 0; i < sizeBytes; i++) {
	// 	if(dataR[i] != dataW[i]) {
	// 		errCnt += 1;
	// 	}
	// }

	// speed = (sizeBytes) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	// LOG_INFO("Flash DMA Read: %d Bytes in %d us (%.2f MByte/s), Err %d", (sizeBytes), deltaTime, speed, errCnt);
}