/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:    Instinct/Modules/System/shellOpticalFlow.cpp
 */

#include "shellModules.hpp"

#include "opticalFlowThread.hpp"

static void PrintFlowHelp() {
	Logger::Instance().Write("--- Optical Flow & ToF Commands ---\r\n");
	Logger::Instance().Write("  stats  - Telemetry rates, latency, and tracking quality\r\n");
	Logger::Instance().Write("  reset  - Reset telemetry accumulators\r\n");
	Logger::Instance().Write("-----------------------------------\r\n");
}

static bool CommandFlow(const char* args) {
	if(args == nullptr || args[0] == '\0') {
		PrintFlowHelp();
		return true;
	}

	char action[16] = {0};
	int32_t count = sscanf(args, "%15s", action);
	if(count < 1) {
		PrintFlowHelp();
		return true;
	}

	if(strcmp(action, "stats") == 0) {
		OpticalFlowThread::Statistics stats =  OpticalFlowThread::GetStatistics();

		Logger::Instance().Write("\r\n--- OPTICAL FLOW & TOF STATS ---\r\n");
		Logger::Instance().Printf("  Camera:      %.1f FPS (Total: %lu, Drops: %lu)\r\n", 
									stats.camFPS, stats.frameCount, stats.frameDrops);
		Logger::Instance().Printf("  Flow Timing: %lu us (Max: %lu us)\r\n", 
									stats.procTime, stats.maxProcTime);
		Logger::Instance().Printf("  Quality:     %.1f%% (Flow Drops: %lu)\r\n", 
									stats.lastQuality * 100.0f, stats.flowDrops);
		Logger::Instance().Printf("  ToF Sensor:  %.1f Hz (Total: %lu, Drops: %lu)\r\n", 
									stats.tofFPS, stats.tofSamples, stats.tofDrops);
		Logger::Instance().Write("--------------------------------\r\n");
		return true;
	}

	if(strcmp(action, "reset") == 0) {
		OpticalFlowThread::ResetStatistics();
		Logger::Instance().Write("Optical flow statistics reset.\r\n");
		return true;
	}

	Logger::Instance().Printf("Error: Unknown action '%s'.\r\n", action);
	PrintFlowHelp();
	return true;
}

static const Shell::CommandEntry flowCommands[] = {
	{ "flow", CommandFlow, "Optical flow & downward ToF diagnostics" },
	{ nullptr, nullptr, nullptr }
};

static Shell::CommandList flowShellNode;

void RegisterOpticalFlowCommands() {
	Shell::RegisterCommands(&flowShellNode, flowCommands);
}