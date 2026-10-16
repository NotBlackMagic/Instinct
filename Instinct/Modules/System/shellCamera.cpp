/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/System/shellCamera.cpp
 */

#include "shellModules.hpp"

static void PrintHelp() {
	Logger::Instance().Write("--- Camera Commands ---\r\n");
	Logger::Instance().Write("  <target> info              - Return connected camera sensor info\r\n");
	Logger::Instance().Write("  <target> start             - Start capture\r\n");
	Logger::Instance().Write("  <target> stop              - Stop capture\r\n");
	Logger::Instance().Write("  <target> fps [val]         - Set camera sensor FPS\r\n");
	Logger::Instance().Write("  <target> gain [val]        - Set camera sensor AGC/analog gain\r\n");
	Logger::Instance().Write("  <target> exposure [val]    - Set camera sensor exposure\r\n");
	Logger::Instance().Write("  <target> brightness [val]  - Set camera sensor brightness\r\n");
	Logger::Instance().Write("  <target> contrast [val]    - Set camera sensor contrast\r\n");
	Logger::Instance().Write("  <target> flicker [val]     - Set camera sensor flicker filter\r\n");
	Logger::Instance().Write("  <target> test [val]        - Enable/disable test pattern output\r\n");
	Logger::Instance().Write("-----------------------\r\n");
}

static bool CommandCam(const char* args) {
	if(args == nullptr || args[0] == '\0') {
		PrintHelp();
		return true;
	}

	char target[10] = {0};
	char action[16] = {0};
	int val = 0;

	// Parse target, action, and an optional parameter value(s)
	int count = sscanf(args, "%9s %15s %d", target, action, &val);
	if(count < 2) {
		PrintHelp();
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
			// Logger::Instance().Write("[DCMI/OV7670] Fetching info...\r\n");
			// OV7670::SensorInfo info = cameraSD.GetSensor().GetInfo();
			Logger::Instance().Write("[DCMI/OV9281] Fetching info...\r\n");
			OV9281::SensorInfo info = cameraSD.GetSensor().GetInfo();
			Logger::Instance().Printf("  ID: 0x%04X, Res: %dx%d\r\n", info.id, info.width, info.height);
		}
		if(routeMipi == true) {
			Logger::Instance().Write("[MIPI/OV5645] Fetching info...\r\n");
			// Fetch MIPI info here
		}
		return true;
	}
	
	if(strcmp(action, "start") == 0) {
		if(routeDcmi == true) {
			/* cameraDcmi.Start(); */ 
			Logger::Instance().Write("DCMI Camera started.\r\n");
		} 
		if(routeMipi == true) {
			/* cameraMipi.Start(); */
			Logger::Instance().Write("MIPI Camera started.\r\n");
		}
		return true;
	}
	
	if(strcmp(action, "stop") == 0) {
		if(routeDcmi == true) {
			/* cameraDcmi.Stop(); */
			Logger::Instance().Write("DCMI Camera stopped.\r\n");
		}
		if(routeMipi == true) {
			/* cameraMipi.Stop(); */
			Logger::Instance().Write("MIPI Camera stopped.\r\n");
		}
		return true;
	}

	if(strcmp(action, "fps") == 0 && count >= 3) {
		if(routeDcmi == true) {
			cameraSD.GetSensor().SetFPS((uint32_t)val);
			Logger::Instance().Printf("DCMI Gain set to %d.\r\n", (uint32_t)val);
		}
		if(routeMipi == true) {
			// cameraMipi.GetSensor().SetBrightness((int8_t)val);
		}
		return true;
	}

	if(strcmp(action, "gain") == 0 && count >= 3) {
		if(routeDcmi == true) {
			cameraSD.GetSensor().SetAnalogGain((uint16_t)val);
			Logger::Instance().Printf("DCMI Gain set to %d.\r\n", (uint16_t)val);
		}
		if(routeMipi == true) {
			// cameraMipi.GetSensor().SetBrightness((int8_t)val);
		}
		return true;
	}

	if(strcmp(action, "exposure") == 0 && count >= 3) {
		if(routeDcmi == true) {
			cameraSD.GetSensor().SetExposure((uint32_t)val);
			Logger::Instance().Printf("DCMI Exposure set to %d.\r\n", (uint32_t)val);
		}
		if(routeMipi == true) {
			// cameraMipi.GetSensor().SetBrightness((int8_t)val);
		}
		return true;
	}
	
	if(strcmp(action, "brightness") == 0 && count >= 3) {
		if(routeDcmi == true) {
			// cameraSD.GetSensor().SetBrightness((int8_t)val);
			Logger::Instance().Printf("DCMI Brightness set to %d.\r\n", (int8_t)val);
		}
		if(routeMipi == true) {
			// cameraMipi.GetSensor().SetBrightness((int8_t)val);
		}
		return true;
	}
	
	if(strcmp(action, "contrast") == 0 && count >= 3) {
		if(routeDcmi == true) {
			// cameraSD.GetSensor().SetContrast((uint8_t)val);
			Logger::Instance().Printf("DCMI Contrast set to %u.\r\n", (uint8_t)val);
		}
		return true;
	}
	
	if(strcmp(action, "flicker") == 0 && count >= 3) {
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
			// cameraSD.GetSensor().SetBandingFilter(filter);
			Logger::Instance().Printf("DCMI Flicker filter set to %s.\r\n", filterName);
		}
		return true;
	}

	if(strcmp(action, "test") == 0 && count >= 3) {
		bool enable = (val > 0);
		if(routeDcmi == true) {
			cameraSD.GetSensor().SetTestPattern(enable);
			Logger::Instance().Printf("DCMI Test pattern %s.\r\n", enable ? "ENABLED" : "DISABLED");
		}
		return true;
	}
	
	Logger::Instance().Printf("Error: Unknown action '%s'.\r\n", action);
	PrintHelp();

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
