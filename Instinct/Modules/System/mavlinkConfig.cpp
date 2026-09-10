/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/mavlinkConfig.cpp
 */

#include "mavlinkConfig.hpp"

void MavlinkConfig::ApplyStandardProfile() {
	// 433MHz / 900MHz Radio Safe Rates (57600 baud limit)
	MavlinkThread::SetRate(MavlinkThread::Stream::Heartbeat, 1.0f);
	MavlinkThread::SetRate(MavlinkThread::Stream::Attitude, 20.0f);
	MavlinkThread::SetRate(MavlinkThread::Stream::HighresIMU, 10.0f);

	LOG_INFO("Mavlink: Standard Telemetry Profile Applied");
}

void MavlinkConfig::ApplyHighSpeedProfile() {
	// USB-CDC or ESP32 Wi-Fi Rates (High Bandwidth)
	MavlinkThread::SetRate(MavlinkThread::Stream::Heartbeat, 1.0f);
	MavlinkThread::SetRate(MavlinkThread::Stream::Attitude, 50.0f);
	MavlinkThread::SetRate(MavlinkThread::Stream::HighresIMU, 50.0f);

	LOG_INFO("Mavlink: High-Speed Telemetry Profile Applied");
}

void MavlinkConfig::ApplyQuietProfile() {
	// Shuts up the radio to clear bandwidth, maintains connection heartbeat
	MavlinkThread::SetRate(MavlinkThread::Stream::Heartbeat, 1.0f);
	MavlinkThread::SetRate(MavlinkThread::Stream::Attitude, 0.0f);
	MavlinkThread::SetRate(MavlinkThread::Stream::HighresIMU, 0.0f);

	LOG_INFO("Mavlink: Quiet Telemetry Profile Applied");
}

void MavlinkConfig::DisableAll() {
	for(uint8_t i = 0; i < static_cast<uint8_t>(MavlinkThread::Stream::Count); i++) {
		MavlinkThread::SetRate(static_cast<MavlinkThread::Stream>(i), 0.0f);
	}
	LOG_INFO("Mavlink: All Telemetry Streams Disabled");
}