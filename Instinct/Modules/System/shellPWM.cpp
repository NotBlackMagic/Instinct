/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/System/shellPWM.cpp
 */

#include "shellModules.hpp"

static void PrintHelp() {
	Logger::Instance().Write("--- PWM Commands ---\r\n");
	Logger::Instance().Write("  pwm pulse <ch> <us>      - Set pulse width in us (1-8, 0 for all)\r\n");
	Logger::Instance().Write("  pwm duty <ch> <percent>  - Set duty cycle 0-100% (1-8, 0 for all)\r\n");
	Logger::Instance().Write("  pwm servo <ch> <percent> - Set servo position 0-100% (1-8, 0 for all)\r\n");
	Logger::Instance().Write("  pwm start [ch]           - Enable PWM channel (1-8, default all)\r\n");
	Logger::Instance().Write("  pwm stop [ch]            - Disable PWM channel (1-8, default all)\r\n");
	Logger::Instance().Write("--------------------\r\n");
}

static PWM* GetPWM(uint8_t ch) {
	switch(ch) {
		case 1:
			return &pwm1Ch1;
		case 2:
			return &pwm1Ch2;
		case 3:
			return &pwm1Ch3;
		case 4:
			return &pwm1Ch4;
		case 5:
			return &pwm2Ch1;
		case 6:
			return &pwm2Ch2;
		case 7:
			return &pwm2Ch3;
		case 8:
			return &pwm2Ch4;
		default:
			return nullptr;
	}
}

static bool CommandPWM(const char* args) {
	if(args == nullptr || args[0] == '\0') {
		PrintHelp();
		return true;
	}

	char action[16] = {0};
	// Parse target action
	int32_t count = sscanf(args, "%15s", action);
	if(count < 1) {
		PrintHelp();
		return true;
	}

	if(strcmp(action, "pulse") == 0) {
		int32_t ch = -1;
		uint32_t pulse = 0;

		int32_t parsed = sscanf(args, "%*s %d %lu", &ch, &pulse);
		if(parsed < 2 || ch < 0 || ch > 8) {
			Logger::Instance().Write("Usage: pwm pulse <ch (1-8, 0 for all)> <us>\r\n");
			return true;
		}

		if(ch == 0) {
			for(uint8_t i = 1; i <= 8; i++) {
				GetPWM(i)->SetPulseWidth(pulse);
			}
			Logger::Instance().Printf("PWM ALL set to %lu us.\r\n", pulse);
		}
		else {
			GetPWM(static_cast<uint8_t>(ch))->SetPulseWidth(pulse);
			Logger::Instance().Printf("PWM CH%d set to %lu us.\r\n", ch, pulse);
		}
		return true;
	}

	if(strcmp(action, "duty") == 0) {
		int32_t ch = -1;
		float dcPer = 0.0f;

		int32_t parsed = sscanf(args, "%*s %d %f", &ch, &dcPer);
		if(parsed < 2 || ch < 0 || ch > 8 || dcPer < 0.0f || dcPer > 100.0f) {
			Logger::Instance().Write("Usage: pwm duty <ch (1-8, 0 for all)> <percent (0.0-100.0)>\r\n");
			return true;
		}

		float dcRatio = dcPer / 100.0f;
		if(ch == 0) {
			for(uint8_t i = 1; i <= 8; i++) {
				GetPWM(i)->SetDutyCycle(dcRatio);
			}
			Logger::Instance().Printf("PWM ALL duty set to %.1f%%.\r\n", static_cast<double>(dcPer));
		}
		else {
			GetPWM(static_cast<uint8_t>(ch))->SetDutyCycle(dcRatio);
			Logger::Instance().Printf("PWM CH%d duty set to %.1f%%.\r\n", ch, static_cast<double>(dcPer));
		}
		return true;
	}

	if(strcmp(action, "servo") == 0) {
		int32_t ch = -1;
		float dcPer = 0.0f;

		int32_t parsed = sscanf(args, "%*s %d %f", &ch, &dcPer);
		if(parsed < 2 || ch < 0 || ch > 8 || dcPer < 0.0f || dcPer > 100.0f) {
			Logger::Instance().Write("Usage: pwm servo <ch (1-8, 0 for all)> <percent (0.0-100.0)>\r\n");
			return true;
		}

		uint32_t pulse = 1000.0f + (dcPer * 10.f);	// Servo pulse width typically is in range 1ms to 2ms
		if(ch == 0) {
			for(uint8_t i = 1; i <= 8; i++) {
				GetPWM(i)->SetPulseWidth(pulse);
			}
			Logger::Instance().Printf("PWM ALL servo set to %.1f%%.\r\n", static_cast<double>(dcPer));
		}
		else {
			GetPWM(static_cast<uint8_t>(ch))->SetPulseWidth(pulse);
			Logger::Instance().Printf("PWM CH%d servo set to %.1f%%.\r\n", ch, static_cast<double>(dcPer));
		}
		return true;
	}

	if(strcmp(action, "start") == 0) {
		int32_t ch = 0;

		int32_t parsed = sscanf(args, "%*s %d", &ch);
		if(parsed < 1 || ch == 0) {
			for(uint8_t i = 1; i <= 8; i++) {
				GetPWM(i)->Start();
			}
			Logger::Instance().Write("PWM ALL channels started.\r\n");
		}
		else if(ch >= 1 && ch <= 8) {
			GetPWM(static_cast<uint8_t>(ch))->Start();
			Logger::Instance().Printf("PWM CH%d started.\r\n", ch);
		}
		else {
			Logger::Instance().Write("Usage: pwm start [ch (1-8, default all)]\r\n");
		}
		return true;
	}

	if(strcmp(action, "stop") == 0) {
		int32_t ch = 0;

		int32_t parsed = sscanf(args, "%*s %d", &ch);
		if(parsed < 1 || ch == 0) {
			for(uint8_t i = 1; i <= 8; i++) {
				GetPWM(i)->Stop();
			}
			Logger::Instance().Write("PWM ALL channels stopped.\r\n");
		}
		else if(ch >= 1 && ch <= 8) {
			GetPWM(static_cast<uint8_t>(ch))->Stop();
			Logger::Instance().Printf("PWM CH%d stopped.\r\n", ch);
		}
		else {
			Logger::Instance().Write("Usage: pwm stop [ch (1-8, default all)]\r\n");
		}
		return true;
	}

	Logger::Instance().Printf("Unknown PWM action '%s'.\r\n", action);
	PrintHelp();
	return true;
}

static const Shell::CommandEntry pwmCommands[] = {
	{ "pwm", CommandPWM, "PWM output controls (pulse, duty, start, stop)" },
	{ nullptr, nullptr, nullptr }
};

static Shell::CommandList pwmShellNode;

void RegisterPWMCommands() {
	Shell::RegisterCommands(&pwmShellNode, pwmCommands);
}