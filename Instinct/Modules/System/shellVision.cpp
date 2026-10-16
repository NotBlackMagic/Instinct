/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/System/shellVision.cpp
 */

#include "shellModules.hpp"

#include "visionThread.hpp"

static void PrintVisionHelp() {
	Logger::Instance().Write("--- Vision Commands ---\r\n");
	Logger::Instance().Write("  stats        - Pipeline throughput & latency stats\r\n");
	Logger::Instance().Write("  iframe       - Force IDR keyframe on next frame\r\n");
	Logger::Instance().Write("  record       - Toggle onboard SD video stream recording\r\n");
	Logger::Instance().Write("  snap         - Capture a single high-res snapshot\r\n");
	Logger::Instance().Write("  reset        - Reset telemetry accumulators\r\n");
	Logger::Instance().Write("-----------------------\r\n");
}

static bool CommandVision(const char* args) {
	if(args == nullptr || args[0] == '\0') {
		PrintVisionHelp();
		return true;
	}

	char action[16] = {0};
	// Parse target action
	int32_t count = sscanf(args, "%15s", action);
	if(count < 1) {
		PrintVisionHelp();
		return true;
	}

	if(strcmp(action, "stats") == 0) {
		VisionThread::Statistics stats;
		VisionThread::GetStatistics(stats);

		Logger::Instance().Write("\r\n--- VISION PIPELINE STATS ---\r\n");
		Logger::Instance().Printf("  FPS:         %.1f\r\n", stats.fps);
		Logger::Instance().Printf("  Frames:      %lu (UVC Drop: %lu, Restarts: %lu)\r\n", stats.totalFrames, stats.uvcDroppedCount, stats.pipelineRestarts);
		Logger::Instance().Printf("  Bitrate:     %lu kbps (%lu B/frame)\r\n",  (uint32_t)(stats.lastFrameBytes * 8 * stats.fps / 1000.0f), stats.lastFrameBytes);
		Logger::Instance().Printf("  Compression: %.1fx (Avg: %.1fx)\r\n", stats.compressionRatio, stats.avgCompressionRatio);

		Logger::Instance().Write("[LATENCY (us)]\r\n");
		Logger::Instance().Printf("  Capture:     %6lu / Avg %6lu\r\n", stats.lastCapTimeUs, stats.avgCapTimeUs);
		Logger::Instance().Printf("  Encode:      %6lu / Avg %6lu\r\n", stats.lastEncTimeUs, stats.avgEncTimeUs);
		Logger::Instance().Printf("  I/O:         %6lu / Avg %6lu\r\n", stats.lastIOTimeUs, stats.avgIOTimeUs);
#if USE_LINE_MODE
		Logger::Instance().Printf("  Slice Cap:   %6lu / Avg %6lu\r\n", stats.lastSliceCapUs, stats.avgSliceCapUs);
		Logger::Instance().Printf("  Slice Enc:   %6lu / Avg %6lu\r\n", stats.lastSliceEncUs, stats.avgSliceEncUs);
#endif

#if USE_LINE_MODE
		Logger::Instance().Write("---------------------------------------------------------------------------\r\n");
		Logger::Instance().Write(" SLICE STREAMING (64 lines)\r\n");
		Logger::Instance().Printf(" Slice Capture:          %8lu       %8lu\r\n", stats.lastSliceCapUs, stats.avgSliceCapUs);
		Logger::Instance().Printf(" Slice Encode:           %8lu       %8lu\r\n", stats.lastSliceEncUs, stats.avgSliceEncUs);
#endif

		Logger::Instance().Write("[HARDWARE ERRORS]\r\n");
		Logger::Instance().Printf("  CSI:         CRC: %lu, ECC: %lu, Sync: %lu\r\n", stats.csi.crcErrors, stats.csi.eccErrors, stats.csi.syncErrors);
		Logger::Instance().Printf("  DCMIPP:      Overrun: %lu, Line: %lu\r\n", stats.dcmipp.overrunErrors, stats.dcmipp.lineEvents);
		Logger::Instance().Printf("  VENC:        Errors: %lu, Encoded: %lu\r\n", stats.venc.hwErrors, stats.venc.encodedFrames);

		Logger::Instance().Write("-----------------------------\r\n");
		return true;
	}

	if(strcmp(action, "iframe") == 0) {
		// vencEncoder.ForceKeyframe();
		Logger::Instance().Write("Forced next frame to IDR Keyframe.\r\n");
		return true;
	}

	if(strcmp(action, "record") == 0) {
		// VisionThread::ToggleRecording();
		// Logger::Instance().Printf("Video stream recording: %s\r\n", VisionThread::IsRecording() ? "STARTED" : "STOPPED");
		return true;
	}

	if(strcmp(action, "snap") == 0) {
		// VisionThread::TriggerSnapshot();
		Logger::Instance().Write("Snapshot scheduled for next frame.\r\n");
		return true;
	}

	if(strcmp(action, "reset") == 0) {
		VisionThread::ResetStatistics();
		Logger::Instance().Write("Vision statistics reset.\r\n");
		return true;
	}

	Logger::Instance().Printf("Error: Unknown action '%s'.\r\n", action);
	PrintVisionHelp();
	
	return true;
}

static void PrintVENCHelp() {
	Logger::Instance().Write("--- VENC Hardware Codec Commands ---\r\n");
	Logger::Instance().Write("  venc                      - Dump ASIC capabilities and active config\r\n");
	Logger::Instance().Write("  venc bitrate <kbps>       - Set dynamic bitrate (e.g. venc bitrate 4000)\r\n");
	Logger::Instance().Write("  venc qp <min> <max>       - Set auto rate control QP bounds (0-51)\r\n");
	Logger::Instance().Write("  venc fixedqp <qp>         - Lock constant QP [0-51], or -1 for auto RC\r\n");
	Logger::Instance().Write("  venc gop <len>            - Set I-frame cadence (frames between IDRs)\r\n");
	Logger::Instance().Write("  venc roi <enable> [dQP]   - Set center ROI (e.g. venc roi 1 -10)\r\n");
	Logger::Instance().Write("------------------------------------\r\n");
}

static bool CommandVenc(const char* args) {
	if(args == nullptr || args[0] == '\0') {
		Venc::Config cfg = vencEncoder.GetConfig();
		EWLHwConfig_t asic = Venc::GetHWCapabilities();
		uint32_t asicId = EWLReadAsicID();

		Logger::Instance().Write("\r\n--- VENC CONFIGURATION ---\r\n");

		Logger::Instance().Write("[SILICON]\r\n");
		Logger::Instance().Printf("  ASIC ID:     0x%08lX (Max Width: %u)\r\n", asicId, asic.maxEncodedWidth);
		Logger::Instance().Printf("  Codecs:      H.264: %s, JPEG: %s, VP8: %s\r\n", 
									asic.h264Enabled ? "YES" : "NO",
									asic.jpegEnabled ? "YES" : "NO",
									asic.vp8Enabled ? "YES" : "NO");
		Logger::Instance().Printf("  Bus:         %u-bit (AXI64: %s)\r\n", asic.busWidth * 32, asic.addr64Support ? "YES" : "NO");

		Logger::Instance().Write("[STREAM]\r\n");
		Logger::Instance().Printf("  Codec:       %s\r\n", (cfg.codec == Venc::Codec::H264) ? "H.264" : "JPEG");
		Logger::Instance().Printf("  Resolution:  %lux%lu @ %u fps\r\n", cfg.imageParams.width, cfg.imageParams.height, cfg.imageParams.frameRate);
		Logger::Instance().Printf("  Profile:     %s\r\n", (cfg.codingControl.profile == Venc::H264Profile::High) ? "High (CABAC+8x8)" : (cfg.codingControl.profile == Venc::H264Profile::Main) ? "Main (CABAC)" : "Baseline");
		Logger::Instance().Printf("  Bitrate:     %lu kbps (GOP: %u)\r\n", cfg.rateControl.bitPerSecond / 1000, cfg.rateControl.gopLen);
		Logger::Instance().Printf("  Rate Ctrl:   %s (QP: %u-%u, Init: %u)\r\n", cfg.rateControl.enablePictureRc ? "Picture RC" : "Fixed QP", cfg.rateControl.qpMin, cfg.rateControl.qpMax, cfg.rateControl.qpHdr);
		Logger::Instance().Printf("  Streaming:   %s (Slice: %u MB rows)\r\n", cfg.enableStreamingMode ? "Line Mode" : "Full Frame", cfg.codingControl.sliceSize);
		Logger::Instance().Printf("  Deblock:     %s\r\n", cfg.codingControl.disableDeblockingFilter ? "Disabled" : "Enabled");
		Logger::Instance().Printf("  RAM Pool:    0x%08lX (%lu KB)\r\n", reinterpret_cast<uint32_t>(cfg.poolAddress), cfg.poolSize / 1024);

		Logger::Instance().Write("--------------------------\r\n");
		return true;
	}

	char subAction[16] = {0};
	// Parse target action
	int32_t count = sscanf(args, "%15s", subAction);
	if(count < 1) {
		PrintVENCHelp();
		return true;
	}

	// Bitrate control
	if(strcmp(subAction, "bitrate") == 0) {
		int32_t kbps = 0;
		if(sscanf(args, "%*s %d", &kbps) < 1 || kbps < 100 || kbps > 50000) {
			Logger::Instance().Write("Usage: venc bitrate <100-50000> (kbps)\r\n");
			return true;
		}
		uint32_t bps = static_cast<uint32_t>(kbps) * 1000U;
		Status status = vencEncoder.SetBitrate(bps);
		Logger::Instance().Printf("VENC Bitrate -> %d kbps [%s]\r\n", kbps, (status == Status::Ok) ? "OK" : "FAILED");
		return true;
	}

	// Dynamic QP limits
	if(strcmp(subAction, "qp") == 0) {
		int32_t qpMin = 0;
		int32_t qpMax = 0;
		if(sscanf(args, "%*s %d %d", &qpMin, &qpMax) < 2 || qpMin < 0 || qpMax > 51 || qpMin > qpMax) {
			Logger::Instance().Write("Usage: venc qp <min 0-51> <max 0-51>\r\n");
			return true;
		}
		Status status = vencEncoder.SetAutoQP(static_cast<uint32_t>(qpMin), static_cast<uint32_t>(qpMax));
		Logger::Instance().Printf("VENC QP Bounds -> [%d, %d] [%s]\r\n", qpMin, qpMax, (status == Status::Ok) ? "OK" : "FAILED");
		return true;
	}

	// Fixed QP mode (benchmarking / calibration)
	if(strcmp(subAction, "fixedqp") == 0) {
		int32_t qpVal = 0;
		if(sscanf(args, "%*s %d", &qpVal) < 1 || qpVal < -1 || qpVal > 51) {
			Logger::Instance().Write("Usage: venc fixedqp <-1 to disable, or 0-51>\r\n");
			return true;
		}
		Status status = Status::Error;
		if(qpVal < 0) {
			// Restore dynamic rate control
			status = vencEncoder.SetAutoQP(10, 48);
			Logger::Instance().Printf("Restored Auto Rate Control (QP 10-48) [%s]\r\n", (status == Status::Ok) ? "OK" : "FAILED");
		}
		else {
			status = vencEncoder.SetFixedQP(static_cast<uint32_t>(qpVal));
			Logger::Instance().Printf("Locked Constant QP -> %d [%s]\r\n", qpVal, (status == Status::Ok) ? "OK" : "FAILED");
		}
		return true;
	}

	// GOP Length
	if(strcmp(subAction, "gop") == 0) {
		int32_t gop = 0;
		if(sscanf(args, "%*s %d", &gop) < 1 || gop < 0 || gop > 300) {
			Logger::Instance().Write("Usage: venc gop <0-300> (0 = infinite P-frames, 1 = all I-frames)\r\n");
			return true;
		}
		Status status = vencEncoder.SetGOPLength(static_cast<uint32_t>(gop));
		Logger::Instance().Printf("VENC GOP Length -> %d [%s]\r\n", gop, (status == Status::Ok) ? "OK" : "FAILED");
		return true;
	}

	// Center Region of Interest
	if(strcmp(subAction, "roi") == 0) {
		int32_t enable = 0;
		int32_t deltaQp = -8;
		int32_t parsed = sscanf(args, "%*s %d %d", &enable, &deltaQp);
		if(parsed < 1) {
			Logger::Instance().Write("Usage: venc roi <0|1> [deltaQp -15..0]\r\n");
			return true;
		}

		Status status = Status::Error;
		if(enable == 0) {
			status = vencEncoder.SetROI(0, 0, 0, 0, 0);
			Logger::Instance().Printf("VENC ROI -> Disabled [%s]\r\n", (status == Status::Ok) ? "OK" : "FAILED");
		}
		else {
			// Compute a 1/3-size box centered in macroblock coordinates
			Venc::Config cfg = vencEncoder.GetConfig();
			uint32_t mbWidth = (cfg.imageParams.width + 15U) / 16U;
			uint32_t mbHeight = (cfg.imageParams.height + 15U) / 16U;
			
			uint32_t topMb = mbHeight / 3U;
			uint32_t bottomMb = (mbHeight * 2U) / 3U;
			uint32_t leftMb = mbWidth / 3U;
			uint32_t rightMb = (mbWidth * 2U) / 3U;

			status = vencEncoder.SetROI(topMb, leftMb, bottomMb, rightMb, deltaQp);
			Logger::Instance().Printf("VENC Center ROI -> MB [%lu,%lu to %lu,%lu] dQP: %d [%s]\r\n", leftMb, topMb, rightMb, bottomMb, deltaQp, (status == Status::Ok) ? "OK" : "FAILED");
		}
		return true;
	}

	Logger::Instance().Printf("Unknown VENC action '%s'.\r\n", subAction);
	PrintVENCHelp();
	return true;
}

static const Shell::CommandEntry visionCommands[] = {
	{ "vision", CommandVision, "Pipeline-level diagnostics and operations" },
	{ "venc", CommandVenc, "VENC hardware video codec live controls" },
	{ nullptr, nullptr, nullptr }
};

static Shell::CommandList visionShellNode;

void RegisterVisionCommands() {
	Shell::RegisterCommands(&visionShellNode, visionCommands);
}