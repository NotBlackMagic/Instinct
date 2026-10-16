/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/System/shellStorage.cpp
 */

#include "shellModules.hpp"

#include "storageThread.hpp"

static const char* GetBusSpeedString(SDMMC::BusSpeed speed) {
	switch(speed) {
		case SDMMC::BusSpeed::Default:
			return "Default (25 MHz)";
		case SDMMC::BusSpeed::HighSpeed:
			return "High Speed (50 MHz)";
		case SDMMC::BusSpeed::UHS_SDR12:
			return "UHS-I SDR12 (25 MHz)";
		case SDMMC::BusSpeed::UHS_SDR25:
			return "UHS-I SDR25 (50 MHz)";
		case SDMMC::BusSpeed::UHS_SDR50:	
			return "UHS-I SDR50 (100 MHz)";
		case SDMMC::BusSpeed::UHS_DDR50:
			return "UHS-I DDR50 (50 MHz DDR)";
		case SDMMC::BusSpeed::UHS_SDR104:
			return "UHS-I SDR104 (208 MHz)";
		case SDMMC::BusSpeed::eMMC_DDR52:
			return "eMMC DDR52 (52 MHz DDR)";
		case SDMMC::BusSpeed::eMMC_HS200:
			return "eMMC HS200 (200 MHz)";
		default:
			return "Unknown";
	}
}

static void PrintHelp() {
	Logger::Instance().Write("--- Storage Commands ---\r\n");
	Logger::Instance().Write("  info         - Media status & filesystem capacity\r\n");
	Logger::Instance().Write("  stats        - I/O throughput, latencies & HW faults\r\n");
	Logger::Instance().Write("  reset        - Reset driver telemetry counters\r\n");
	Logger::Instance().Write("------------------------\r\n");
}

static bool CommandStorage(const char* args) {
	if(args == nullptr || args[0] == '\0') {
		PrintHelp();
		return true;
	}

	char action[16] = {0};

	// Parse target, action, and an optional parameter value(s)
	int count = sscanf(args, "%15s", action);
	if(count < 1) {
		PrintHelp();
		return true;
	}

	if(strcmp(action, "info") == 0) {
		Logger::Instance().Write("\r\n--- STORAGE INFORMATION ---\r\n");

		// Filesystem Layer (FileX)
		Logger::Instance().Write("[FILESYSTEM]\r\n");
		if(StorageThread::IsReady() && StorageThread::GetMedia() != nullptr) {
			FX_MEDIA* media = StorageThread::GetMedia();
			ULONG64 freeBytes = 0;
			fx_media_extended_space_available(media, &freeBytes);

			uint64_t totalBytes = static_cast<uint64_t>(media->fx_media_total_sectors) * media->fx_media_bytes_per_sector;
			uint32_t usedPct = (totalBytes > 0) ? static_cast<uint32_t>(((totalBytes - freeBytes) * 100) / totalBytes) : 0;

			Logger::Instance().Printf("  Volume:      %s\r\n", media->fx_media_name);
			Logger::Instance().Printf("  State:       Mounted (Ready)\r\n");
			Logger::Instance().Printf("  Capacity:    %lu / %lu MB (%u%% used)\r\n", static_cast<uint32_t>((totalBytes - freeBytes) >> 20), static_cast<uint32_t>(totalBytes >> 20), usedPct);
			Logger::Instance().Printf("  Sector Size: %u B (Cluster: %u B)\r\n", media->fx_media_bytes_per_sector, media->fx_media_bytes_per_sector * media->fx_media_sectors_per_cluster);
		}
		else {
			Logger::Instance().Write("  State:       Not Mounted / No Media\r\n");
		}

		// Physical Card Layer (SD Driver)
		SD::CardInfo card = sdCard.GetCardInfo();
		Logger::Instance().Write("[SD CARD]\r\n");
		if(card.sizeBytes > 0) {
			const char* specStr = (card.specVersion == 0) ? "1.0" : (card.specVersion == 1) ? "1.1" : "2.0+";

			// Video Speed Classes [Min. Continuouse Write Speeds]: 
			// V6  [6 MB/s]
			// V10 [10 MB/s]
			// V30 [30 MB/s]
			// V60 [60 MB/s]
			// V90 [90 MB/s]
			// UHS Speed Classes [Min. Continuouse Write Speeds]:
			// U1 [10 MB/s]
			// U3 [30 MB/s]
			// Speed Classes [Min. Continuouse Write Speeds]:
			// C2  [2 MB/s]
			// C4  [4 MB/s]
			// C6  [6 MB/s]
			// C10 [10 MB/s]
			// Application Performance Classes [Min. Random Read; Min Random Write; Min. Sustained Sequetial Write]
			// IOPS: Number of 4 KByte read or write commands per second
			// A1 [1500 IOPS;  500 IOPS; 10 MB/s]
			// A2 [4000 IOPS; 2000 IOPS; 10 MB/s]

			Logger::Instance().Printf("  Card Name:   %s (Rev %u.%u)\r\n", card.cardName, (card.revision >> 4) & 0x0F, card.revision & 0x0F);
			Logger::Instance().Printf("  Identity:    MID: 0x%02X, OID: 0x%04X, PSN: 0x%08lX\r\n", card.manufacturerID, card.oemID, card.serialNumber);
			Logger::Instance().Printf("  Mfg Date:    %02u/%04u\r\n", card.mfgMonth, card.mfgYear);
			Logger::Instance().Printf("  Capacity:    %lu MB (%s, %lu blocks)\r\n", static_cast<uint32_t>(card.sizeBytes >> 20), card.highCapacity ? "SDHC/SDXC" : "SDSC", card.blockCount);
			Logger::Instance().Printf("  Bus Mode:    %u-bit | %s\r\n", (card.currentBusWidth == 0x02) ? 4 : 1, GetBusSpeedString(card.activeMode));
			Logger::Instance().Printf("  Classes:     Class %u | U%u | V%u%s\r\n", card.speedClass, card.uhsSpeedGrade, card.videoSpeedClass, (card.appPerfClass == 1) ? " | A1" : (card.appPerfClass == 2) ? " | A2" : "");
			Logger::Instance().Printf("  Features:    Spec v%s | CMD23: %s\r\n", specStr, card.supportCmd23 ? "YES" : "NO");
		}
		else {
			Logger::Instance().Write("  Card:        Not Detected\r\n");
		}

		Logger::Instance().Write("---------------------------\r\n");
		return true;
	}

	if(strcmp(action, "stats") == 0) {
		SD::Statistics sdStats = sdCard.GetStatistics();
		SDMMC::Statistics busStats = sdmmc1.GetStatistics();

		uint32_t readMb = static_cast<uint32_t>((sdStats.sectorsRead * 512) >> 20);
		uint32_t writtenMb = static_cast<uint32_t>((sdStats.sectorsWritten * 512) >> 20);

		Logger::Instance().Write("\r\n--- STORAGE I/O STATS ---\r\n");

		Logger::Instance().Write("[THROUGHPUT]\r\n");
		Logger::Instance().Printf("  Read:        %lu MB (%llu sectors)\r\n", readMb, sdStats.sectorsRead);
		Logger::Instance().Printf("  Written:     %lu MB (%llu sectors)\r\n", writtenMb, sdStats.sectorsWritten);
		Logger::Instance().Printf("  Errors:      Read: %lu | Write: %lu\r\n", sdStats.readErrors, sdStats.writeErrors);

		Logger::Instance().Write("[LATENCY (us)]\r\n");
		Logger::Instance().Printf("  Block Read:  Last: %6lu | Max: %6lu\r\n", sdStats.lastReadTimeUs, sdStats.maxReadTimeUs);
		Logger::Instance().Printf("  Block Write: Last: %6lu | Max: %6lu (Worst GC Stall)\r\n", sdStats.lastWriteTimeUs, sdStats.maxWriteTimeUs);

		Logger::Instance().Write("[BUS FAULTS (SDMMC)]\r\n");
		Logger::Instance().Printf("  CRC Errors:  CMD: %lu | Data: %lu\r\n", busStats.cmdCRCErrors, busStats.dataCRCErrors);
		Logger::Instance().Printf("  Timeouts:    CMD: %lu | Data: %lu\r\n", busStats.cmdTimeouts, busStats.dataTimeouts);
		Logger::Instance().Printf("  IDMA Faults: %lu\r\n", busStats.dmaErrors);

		Logger::Instance().Write("-------------------------\r\n");
		return true;
	}

	if(strcmp(action, "reset") == 0) {
		sdCard.ResetStatistics();
		sdmmc1.ResetStatistics();
		Logger::Instance().Write("Storage statistics reset.\r\n");
		return true;
	}

	Logger::Instance().Printf("Error: Unknown action '%s'.\r\n", action);
	PrintHelp();
	
	return true;
}

static const Shell::CommandEntry storageCommands[] = {
	{ "storage", CommandStorage, "Storage subsystem info, stats and control" },
	{ nullptr, nullptr, nullptr }
};

static Shell::CommandList storageShellNode;

void RegisterStorageCommands() {
	Shell::RegisterCommands(&storageShellNode, storageCommands);
}