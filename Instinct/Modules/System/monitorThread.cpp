/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/System/monitorThread.cpp
 */

#include "monitorThread.hpp"

TX_THREAD MonitorThread::threadPtr;
__attribute__((aligned(32))) uint8_t MonitorThread::threadStack[MonitorThread::threadStackSize];

static Topic<PowerMsg> topicPower("power", static_cast<uint8_t>(TopicID::Power));

void MonitorThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("Monitor"),
											MonitorThread::Run, 0,
											threadStack, sizeof(threadStack),
											0, 0,
											TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Monitor Thread Create Failed.");
	}
}

void MonitorThread::Run(ULONG input) {
	(void)input;
	LOG_INFO("Monitor Thread Initialized.");

	PowerMsg pw = {};
	float temperature = 0.0f;
	bool wasAttached = false;
	bool pdNegotiated = false;
	uint32_t attachWaitTicks = 0;
	uint32_t telemetryTickCounter = 0;

	// Configure USB-PD peripheral
	USBPD::Config pdConfig = {
		.targetVoltage = 12000,	// Request 12V
		.targetCurrent = 3000,	// Request 3.0A budget
		.autoNegotiate = false,		// Handled in a task or callback to prevent ISR blocking
		.EventCallback = nullptr,
		.callbackContext = nullptr
	};

	// Initialize
	if(usbPD.Init(pdConfig) == Status::Ok) {
		LOG_INFO("USB-PD Init OK.");
	}
	else {
		LOG_ERR("USB-PD Init Failed!");
	}

	volatile uint8_t probeI2C2 = i2c2.Probe(0x44);

	// Initialize
	if(i2c2.Probe(0x44) == 0x01) {
		INA700::Config config = {};
		if(ina700.Init(config) == Status::Ok) {
			LOG_INFO("PW Monitor (INA700) Init OK");
		}
		else {
			LOG_WARN("PW Monitor (INA700) Init Failed!");
		}
	}
	else {
		LOG_WARN("PW Monitor (INA700) not found on I2C2!");
	}

	FDCAN::Frame frame = {};
	while(1) {
		// USB Power Delivery Handling
		bool isAttached = usbPD.IsAttached();

		if(isAttached && !wasAttached) {
			wasAttached = true;
			pdNegotiated = false;
			attachWaitTicks = 0;
			LOG_INFO("USB-PD Attached on CC%u.", static_cast<uint8_t>(usbPD.GetActiveCCLine()));
		}
		else if(!isAttached && wasAttached) {
			wasAttached = false;
			pdNegotiated = false;
			attachWaitTicks = 0;
			LOG_INFO("USB-PD Detached.");
		}

		// Once attached, inspect caps and negotiate desired profile
		if(isAttached && !pdNegotiated) {
			uint8_t profileCnt = usbPD.GetProfileCount();

			if(profileCnt > 0) {
				const USBPD::Profile *profiles = usbPD.GetProfiles();

				LOG_INFO("USB-PD SRC Capabilities (%u profiles):", profileCnt);
				for(uint8_t i = 0; i < profileCnt; i++) {
					const char *typeStr = (profiles[i].type == USBPD::PowerType::Fixed) ? "Fixed" : (profiles[i].type == USBPD::PowerType::PPS)   ? "PPS"   : "Variable";
					LOG_INFO("  [%u] %s: %.2f V @ %.2f A", i + 1, typeStr, profiles[i].voltage, profiles[i].current);
				}

				// Request target power rail (e.g. 9V @ 3A)
				Status reqStatus = usbPD.RequestVoltage(9000, 3000);
				if(reqStatus == Status::Ok) {
					const USBPD::Profile &active = usbPD.GetActiveProfile();
					LOG_INFO("USB-PD Negotiated: %.2f V @ %.2f A.", active.voltage, active.current);
				}
				else {
					LOG_WARN("USB-PD: Failed to negotiate profile (%.2f V @ %.2f A), fallback to 5V default.", pdConfig.targetVoltage, pdConfig.targetCurrent);
				}

				pdNegotiated = true;
			}
			else {
				attachWaitTicks++;
				// Wait 10 iterations * 50 ms = 500 ms for PD message arrival
				if(attachWaitTicks >= 10) {
					const USBPD::Profile &active = usbPD.GetActiveProfile();
					LOG_INFO("USB-PD Non-PD SRC (Legacy).");
					LOG_INFO("USB-PD Active profile: %.2f V @ %.2f A.", active.voltage, active.current);
					pdNegotiated = true;
				}
			}
		}

		// Telemetry Sampling (every ~500ms or 100ms)
		telemetryTickCounter++;
		if(telemetryTickCounter >= 10) {
			telemetryTickCounter = 0;

			pw.timestamp = Time::GetUs();
			ina700.ReadVoltage(pw.voltage);
			ina700.ReadCurrent(pw.current);
			ina700.ReadTemperature(temperature);

			topicPower.Publish(pw);
		}

		ledGreen.Toggle();
		tx_thread_sleep(50);
	}
}