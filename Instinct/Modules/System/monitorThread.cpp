/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/monitorThread.cpp
 */

#include "monitorThread.hpp"

TX_THREAD MonitorThread::threadPtr;
uint8_t MonitorThread::threadStack[4096];

static Topic<PowerMsg> topicBaroInt("power", static_cast<uint8_t>(TopicID::Power));

void MonitorThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("Monitor"),
											MonitorThread::Run,
											0,
											threadStack,
											sizeof(threadStack),
											0,
											0,
											TX_NO_TIME_SLICE,
											TX_AUTO_START);
}

void MonitorThread::Run(ULONG input) {
	PowerMsg pw;
	float temperature;

	LOG_INFO("Monitor Thread Initialized.");

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

	while(1) {
		pw.timestamp = Time::GetUs();
		ina700.ReadVoltage(pw.voltage);
		ina700.ReadCurrent(pw.current);
		ina700.ReadTemperature(temperature);

		// LOG_INFO("PW: %dmV %dmA %dC", (int)(pw.voltage * 1000), (int)(pw.current * 1000), (int)temperature);

		tx_thread_sleep(5000);
	}
}