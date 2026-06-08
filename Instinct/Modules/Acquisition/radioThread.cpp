/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Acquisitions/radioThread.cpp
 */

#include "radioThread.hpp"

TX_THREAD RadioThread::threadPtr;
uint8_t RadioThread::threadStack[4096];

static Topic<RCMsg> topicRC("rcMain", static_cast<uint8_t>(TopicID::RC), 0);
static Topic<ManualControlMsg> topicManualControl("manualControl", static_cast<uint8_t>(TopicID::ManualControl), 0);

void RadioThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("ACQ_Radio"),
										RadioThread::Run, 0,
										threadStack, sizeof(threadStack),
										2, 0,
										TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX ACQ Radio Thread Create Failed.");
	}

	Broker::RegisterTopic(&topicRC);
}

void RadioThread::Run(ULONG input) {
	RCMsg rcMsg;
	ManualControlMsg controlMsg;
	RCReceiver::ChannelData data;
	uint64_t lastLogTimeUs = 0; // Timer for rate-limiting the console output

	// Setup Telemetry Subscribers (for outbound telemetry)
	// Subscriber<BatteryMsg> subBattery(TopicID::Battery);

	LOG_INFO("Radio Thread Initialized.");

	// Fetch Dynamic Configuration
	// RadioThread::Config threadConfig = SystemSettings::GetRadioConfig();

	// Testing Hardcode
    RadioThread::Config threadConfig = {
        .protocol = RCReceiver::Protocol::IBus,
        .rssiChannelIndex = 13,
        .channelMap = ChannelMap::AETR, // FlySky default
        .deadband = 4                   // 4us deadband for jitter rejection
    };

	// Initialize with a default protocol
	RCReceiver::Config config = {
		.protocol = threadConfig.protocol,
		.rssiChannelIndex = threadConfig.rssiChannelIndex
	};

	if(mainRC.Init(config) == Status::Ok) {
		LOG_INFO("Radio RX Init OK");
	}
	else {
		LOG_WARN("Radio RX Init Failed!");
	}

	while(1) {
		// Drain UART buffer and update internal state machine
		if(mainRC.ProcessStream() == Status::Ok) {
			if(mainRC.GetChannelData(data) == Status::Ok) {
				rcMsg.timestamp = data.timestamp;
				rcMsg.channelCount = data.channelCount;
				rcMsg.linkState = static_cast<uint8_t>(data.linkState);
				rcMsg.linkQuality = data.linkQuality;
				rcMsg.rssi = data.rssi;

				for(uint8_t i = 0; i < RCReceiver::MAX_CHANNELS; i++) {
					rcMsg.channels[i] = data.channels[i];
				}

				// Publish raw RC channel states
				topicRC.Publish(rcMsg);

				// Map and filter
				controlMsg.timestamp = data.timestamp;
				controlMsg.linkState = static_cast<uint8_t>(data.linkState);

				// Failsafe override
				if(data.linkState != RCReceiver::LinkState::Connected) {
					//ZERO out the control vectors
					controlMsg.roll = 0.0f;
					controlMsg.pitch = 0.0f;
					controlMsg.yaw = 0.0f;
					controlMsg.throttle = 0.0f;
					
					// Force Aux channels to a safe disarmed state (-1.0f)
					controlMsg.aux1 = -1.0f; 
					controlMsg.aux2 = -1.0f;
					controlMsg.aux3 = -1.0f;
					controlMsg.aux4 = -1.0f;
				}
				else {
					// Apply Channel Mapping
					if(threadConfig.channelMap == ChannelMap::Surface) {
						// Non flying vehicle mappings: CH1 is Steering (Roll), CH2 is Forward/Reverse (Throttle)
						controlMsg.roll = NormalizeBidirectional(data.channels[0], threadConfig.deadband);
						controlMsg.pitch = 0.0f;
						controlMsg.yaw = 0.0f;
						controlMsg.throttle = NormalizeBidirectional(data.channels[1], threadConfig.deadband);
					}
					else {
						// Flying vehicle mappings
						// AETR Default
						uint8_t idxRoll = 0, idxPitch = 1, idxYaw = 3, idxThrottle = 2;
						if (threadConfig.channelMap == ChannelMap::TAER) {
							idxThrottle = 0; idxRoll = 1; idxPitch = 2; idxYaw = 3;
						}
						
						controlMsg.roll = NormalizeBidirectional(data.channels[idxRoll], threadConfig.deadband);
						controlMsg.pitch = NormalizeBidirectional(data.channels[idxPitch], threadConfig.deadband);
						controlMsg.yaw = NormalizeBidirectional(data.channels[idxYaw], threadConfig.deadband);
						controlMsg.throttle = NormalizeUnidirectional(data.channels[idxThrottle]);
					}

					// Map Aux channels
					controlMsg.aux1 = NormalizeBidirectional(data.channels[4], 0);
					controlMsg.aux2 = NormalizeBidirectional(data.channels[5], 0);
					controlMsg.aux3 = NormalizeBidirectional(data.channels[6], 0);
					controlMsg.aux4 = NormalizeBidirectional(data.channels[7], 0);
				}				

				topicManualControl.Publish(controlMsg);

				// if((rcMsg.timestamp - lastLogTimeUs) > 500000) {
				// 	lastLogTimeUs = rcMsg.timestamp;
					
				// 	LOG_INFO("RC [St:%d] CH1:%4u CH2:%4u CH3:%4u CH4:%4u | LQ:%3u%% RSSI:%4ddBm",	rcMsg.linkState,
				// 																						rcMsg.channels[0],
				// 																						rcMsg.channels[1], 
				// 																						rcMsg.channels[2],
				// 																						rcMsg.channels[3],
				// 																						rcMsg.linkQuality,
				// 																						rcMsg.rssi);
				// }
			}
		}

		// Outbound telemetry, if protocol allows it (e.g. CRSF/ELRS)
		if(threadConfig.protocol == RCReceiver::Protocol::CRSF) {
			// BatteryMsg batData;
			// if(subBattery.Update(&batData)) {
			// 	// Construct CRSF Telemetry Frame
			// 	mainRC.SendTelemetry(payload, length);
			// }
		}

		// Sleep for 2 ticks (~2ms) to hit >500Hz loop times
		tx_thread_sleep(2);
	}
}

float RadioThread::NormalizeBidirectional(uint16_t raw, uint16_t deadband) {
	// Maps 1000-2000us to -1.0f to +1.0f (with center deadband)
	if(raw > (1500 - deadband) && raw < (1500 + deadband)) {
		return 0.0f; 
	}

	float scaled = (static_cast<float>(raw) - 1500.0f) / 500.0f;
	if(scaled > 1.0f) {
		return 1.0f;
	}
	if(scaled < -1.0f) {
		return -1.0f;
	}
	return scaled;
}

float RadioThread::NormalizeUnidirectional(uint16_t raw) {
	// Maps 1000-2000us to 0.0f to +1.0f (No deadband, used for drone lift)
	float scaled = (static_cast<float>(raw) - 1000.0f) / 1000.0f;
	if(scaled > 1.0f) {
		return 1.0f;
	}
	if(scaled < 0.0f) {
		return 0.0f;
	}
	return scaled;
}