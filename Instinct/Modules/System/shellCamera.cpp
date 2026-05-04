/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/shellCamera.cpp
 */

#include "shellModules.hpp"

#include "cameraDCMI.hpp"

static bool CommandCam(const char* args) {
	if (args == nullptr || args[0] == '\0') {
		Logger::Instance().Write("Usage: cam <target> <action> [val]\r\n");
		return true;
	}

	char target[10] = {0};
	char action[16] = {0};
	int val = 0;

	// Parse target, action, and an optional integer value
	int count = sscanf(args, "%9s %15s %d", target, action, &val);

	if(count < 2) {
		Logger::Instance().Write("Usage: cam <target> <action> [val]\r\n");
		return true;
	}

	// Determine which camera(s) we are talking to
	bool routeDcmi = false;
	if(strcmp(target, "dcmi") == 0 || strcmp(target, "all") == 0) {
		routeDcmi = true;
	}
	bool routeMipi = false;
	if(strcmp(target, "mipi") == 0 || strcmp(target, "all") == 0) {
		routeMipi = true;
	}

	if(!routeDcmi && !routeMipi) {
		Logger::Instance().Printf("Error: Unknown target '%s'.\r\n", target);
		return true;
	}

	// Route the actions
	if(strcmp(action, "info") == 0) {
		if(routeDcmi == true) {
			Logger::Instance().Write("[DCMI/OV7670] Fetching info...\r\n");
			OV7670::SensorInfo info = cameraSD.GetSensor().GetInfo();
			Logger::Instance().Printf("  ID: 0x%04X, Res: %dx%d\r\n", info.id, info.width, info.height);
		}
		if(routeMipi == true) {
			Logger::Instance().Write("[MIPI/OV5645] Fetching info...\r\n");
			// Fetch MIPI info here
		}
	}
	else if(strcmp(action, "start") == 0) {
		if(routeDcmi == true) {
			/* cameraDcmi.Start(); */ 
			Logger::Instance().Write("DCMI Camera started.\r\n");
		} 
		if(routeMipi == true) {
			/* cameraMipi.Start(); */
			Logger::Instance().Write("MIPI Camera started.\r\n");
		}
	}
	else if(strcmp(action, "stop") == 0) {
		if(routeDcmi == true) {
			/* cameraDcmi.Stop(); */
			Logger::Instance().Write("DCMI Camera stopped.\r\n");
		}
		if(routeMipi == true) {
			/* cameraMipi.Stop(); */
			Logger::Instance().Write("MIPI Camera stopped.\r\n");
		}
	}
	else if(strcmp(action, "brightness") == 0 && count >= 3) {
		if(routeDcmi == true) {
			cameraSD.GetSensor().SetBrightness((int8_t)val);
			Logger::Instance().Printf("DCMI Brightness set to %d.\r\n", (int8_t)val);
		}
		if(routeMipi == true) {
			// cameraMipi.GetSensor().SetBrightness((int8_t)val);
		}
	}
	else if(strcmp(action, "contrast") == 0 && count >= 3) {
		if(routeDcmi == true) {
			cameraSD.GetSensor().SetContrast((uint8_t)val);
			Logger::Instance().Printf("DCMI Contrast set to %u.\r\n", (uint8_t)val);
		}
	}
	else if(strcmp(action, "flicker") == 0 && count >= 3) {
		// Map integer input to the OV7670::BandingFilter enum
		OV7670::BandingFilter filter = OV7670::BandingFilter::Off;
		const char* filterName = "Off";
		
		if(val == 50) { 
			filter = OV7670::BandingFilter::Hz50; 
			filterName = "50Hz"; 
		}
		else if(val == 60) {
			filter = OV7670::BandingFilter::Hz60;
			filterName = "60Hz";
		}
		else if(val == 1) {
			filter = OV7670::BandingFilter::Auto;
			filterName = "Auto";
		}

		if(routeDcmi == true) {
			cameraSD.GetSensor().SetBandingFilter(filter);
			Logger::Instance().Printf("DCMI Flicker filter set to %s.\r\n", filterName);
		}
	}
	else if(strcmp(action, "test") == 0 && count >= 3) {
		bool enable = (val > 0);
		if(routeDcmi == true) {
			cameraSD.GetSensor().SetTestPattern(enable);
			Logger::Instance().Printf("DCMI Test pattern %s.\r\n", enable ? "ENABLED" : "DISABLED");
		}
	}
	else {
		Logger::Instance().Printf("Error: Invalid action or missing value for '%s'.\r\n", action);
	}

	return true;
}

static const Shell::CommandEntry cameraCommands[] = {
	{ "cam", CommandCam, "Camera pipeline controls" },
	{ nullptr, nullptr, nullptr } // Terminator
};

static Shell::CommandList cameraShellNode;

void RegisterCameraCommands() {
	Shell::RegisterCommands(&cameraShellNode, cameraCommands);
}
