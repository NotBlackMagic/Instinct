/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Messaging/instinctLink.cpp
 */

#include "instinctLink.hpp"

TX_THREAD InstinctLink::threadPtr;
__attribute__((aligned(32))) uint8_t InstinctLink::threadStack[InstinctLink::threadStackSize];

UART* InstinctLink::linkPort = nullptr;
InstinctLink::StreamSlot InstinctLink::streams[InstinctLink::maxStreams]{};

Subscriber<ParamUpdateMsg> InstinctLink::subParamUpdate;
Topic<ParamUpdateMsg>* InstinctLink::topicParamUpdate = nullptr;

bool InstinctLink::isDumpingParams = false;
uint16_t InstinctLink::dumpParamIdx = 0;

void InstinctLink::Init(UART* uart) {
	linkPort = uart;

	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("InstinctLink"),
										InstinctLink::Run, 0,
										threadStack, sizeof(threadStack),
										5, 5,
										TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX InstinctLink Thread Create Failed.");
	}

	// Zero initialize topic stream slots
	for(uint8_t i = 0; i < maxStreams; i++) {
		streams[i].topic = nullptr;
		streams[i].interval = 0;
		streams[i].timestamp = 0;
		streams[i].version = 0;
	}
}

void InstinctLink::Run(ULONG input) {
	(void)input;
	LOG_INFO("InstinctLink Thread Initialized.");

	// Subscribe to parameter updates for immediate tuning ACKs (non-blocking poll)
	topicParamUpdate = Broker::GetTopic<ParamUpdateMsg>(static_cast<uint8_t>(TopicID::ParamUpdate), 0);
	if(topicParamUpdate != nullptr) {
		topicParamUpdate->Subscribe(&subParamUpdate, false);
	}

	// Framing buffer: 2 (Sync) + 2 (ID) + 1 (Len) + maxPayload + 1 (CRC)
	uint8_t rxFrame[6 + maxPayloadSize];
	uint16_t rxIndex = 0;

	uint8_t uartRXBuf[128];
	uint8_t payloadBuf[maxPayloadSize];
	ParamUpdateMsg paramUpdateMsg{};

	while(1) {
		if(linkPort == nullptr) {
			tx_thread_sleep(10);
			continue;
		}

		// Handle received UART bytes
		uint32_t bytesAvailable = linkPort->Available();
		while(bytesAvailable > 0) {
			uint32_t toRead = (bytesAvailable > sizeof( uartRXBuf)) ? sizeof( uartRXBuf) : bytesAvailable;
			uint32_t bytesRead = linkPort->Receive( uartRXBuf, toRead);
			bytesAvailable -= bytesRead;

			for(uint32_t i = 0; i < bytesRead; i++) {
				uint8_t b = uartRXBuf[i];

				if(rxIndex == 0) {
					if(b == static_cast<uint8_t>(syncWord & 0xFF)) {
						rxFrame[rxIndex++] = b;
					}
				}
				else if(rxIndex == 1) {
					if(b == static_cast<uint8_t>(syncWord >> 8)) {
						rxFrame[rxIndex++] = b;
					}
					else if(b == static_cast<uint8_t>(syncWord & 0xFF)) {
						rxIndex = 1; // Handled repeated 0xAA preamble byte
					}
					else {
						rxIndex = 0;
					}
				}
				else {
					rxFrame[rxIndex++] = b;

					// Header received: Sync[2] + ID[2] + Len[1] = 5 bytes
					if(rxIndex >= 5) {
						uint8_t payloadLen = rxFrame[4];
						uint16_t packetLen = 6 + payloadLen;

						if(rxIndex >= packetLen) {
							// Validate XOR checksum over ID, Len, and Payload
							uint8_t crc = rxFrame[2] ^ rxFrame[3] ^ rxFrame[4];
							for(uint8_t j = 0; j < payloadLen; j++) {
								crc ^= rxFrame[5 + j];
							}

							if(rxFrame[5 + payloadLen] == crc) {
								uint16_t packetID = static_cast<uint16_t>(rxFrame[2]) | (static_cast<uint16_t>(rxFrame[3]) << 8);
								ProcessPacket(packetID, &rxFrame[5], payloadLen);
							}

							rxIndex = 0;
						}
					}
				}
			}
		}

		// Transmit parameter update ACKs
		if(topicParamUpdate != nullptr) {
			if(topicParamUpdate->Take(&subParamUpdate, paramUpdateMsg, TX_NO_WAIT) == true) {
				uint16_t fullID = static_cast<uint16_t>(TopicID::ParamUpdate);
				SendPacket(fullID, reinterpret_cast<const uint8_t*>(&paramUpdateMsg), sizeof(ParamUpdateMsg));
			}
		}

		// Non-blocking parameter table dump (throttled to 2 params per tick)
		if(isDumpingParams == true) {
			uint8_t sentCount = 0;
			uint16_t paramCount = ParamService::GetTotalParamCount();

			while(dumpParamIdx < paramCount && sentCount < 2) {
				if(SendParamInfo(dumpParamIdx) == true) {
					dumpParamIdx += 1;
					sentCount += 1;
				}
				else {
					// UART TX buffer full, retry on next tick
					break;
				}
			}

			if(dumpParamIdx >= paramCount) {
				isDumpingParams = false;
			}
		}

		// Periodic Topic Stream Scheduler
		uint64_t now = Time::GetUs();
		for(uint8_t i = 0; i < maxStreams; i++) {
			if(streams[i].topic != nullptr && streams[i].interval > 0) {
				if((now - streams[i].timestamp) >= streams[i].interval) {
					streams[i].timestamp = now;

					uint32_t currentVer = 0;
					uint8_t payloadSize = streams[i].topic->GetPayloadSize();

					if(streams[i].topic->CopyPayload(payloadBuf, sizeof(payloadBuf), &currentVer) == true) {
						// Only transmit if the topic was updated since last transmit
						if(currentVer != streams[i].version) {
							uint16_t fulltopicID = (static_cast<uint16_t>(streams[i].topic->GetInstance()) << 8) | streams[i].topic->GetTopicID();
							if(SendPacket(fulltopicID, payloadBuf, payloadSize) == true) {
								streams[i].version = currentVer;
							}
						}
					}
				}
			}
		}

		// Run link loop at 500Hz (2ms) for low-latency tuning and HITL
		tx_thread_sleep(2);
	}
}

bool InstinctLink::ConfigureStream(uint8_t topicID, uint8_t instance, float rate) {
	TopicBase* targetTopic = Broker::GetTopicBase(topicID, instance);
	if(targetTopic == nullptr) {
		return false;
	}

	uint32_t newinterval = 0;
	if(rate > 0.0f) {
		newinterval = static_cast<uint32_t>(1000000.0f / rate);
	}

	// Check if topic already has a slot
	for(uint8_t i = 0; i < maxStreams; i++) {
		if(streams[i].topic == targetTopic) {
			streams[i].interval = newinterval;
			if(newinterval == 0) {
				// Disable and free slot
				streams[i].topic = nullptr;
				streams[i].timestamp = 0;
				streams[i].version = 0;
			}
			return true;
		}
	}

	// If disabling a stream that wasn't active, nothing more to do exit early
	if(newinterval == 0) {
		return true;
	}

	// Allocate first empty slot
	for(uint8_t i = 0; i < maxStreams; i++) {
		if(streams[i].topic == nullptr) {
			streams[i].topic = targetTopic;
			streams[i].interval = newinterval;
			streams[i].timestamp = 0;
			streams[i].version = 0;
			return true;
		}
	}

	LOG_WARN("InstinctLink: No free stream slots.");
	return false;
}

void InstinctLink::ProcessPacket(uint16_t packetID, const uint8_t* payload, uint8_t len) {
	// Packet IDs < 0xFF00 are direct Topic Injections (HITL / SITL / Setpoint step inputs)
	if(packetID < 0xFF00) {
		uint8_t topicID = static_cast<uint8_t>(packetID & 0xFF);
		uint8_t instance = static_cast<uint8_t>((packetID >> 8) & 0xFF);

		TopicBase* topic = Broker::GetTopicBase(topicID, instance);
		if(topic != nullptr) {
			topic->PublishRaw(payload, len);
		}
		return;
	}

	// Packet IDs >= 0xFF00 are Control & Parameter RPC Commands
	CommandID cmd = static_cast<CommandID>(packetID);
	switch(cmd) {
		case CommandID::StreamConfig: {
			if(len == sizeof(StreamConfigPayload)) {
				StreamConfigPayload cfg;
				memcpy(&cfg, payload, sizeof(StreamConfigPayload));
				ConfigureStream(cfg.topicID, cfg.instance, static_cast<float>(cfg.rate));
			}
			break;
		}
		case CommandID::ParamRequestList: {
			dumpParamIdx = 0;
			isDumpingParams = true;
			break;
		}
		case CommandID::ParamRequestRead: {
			if(len == sizeof(ParamReadPayload)) {
				ParamReadPayload req;
				memcpy(&req, payload, sizeof(ParamReadPayload));
				SendParamInfo(req.index);
			}
			break;
		}
		case CommandID::ParamSet: {
			if(len == sizeof(ParamSetPayload)) {
				ParamSetPayload req;
				memcpy(&req, payload, sizeof(ParamSetPayload));

				ParamService::ParamList* ownerList = nullptr;
				const ParamService::ParamEntry* entry = ParamService::FindByIndex(req.index, &ownerList);
				if(entry != nullptr && ownerList != nullptr) {
					if(req.type == static_cast<uint8_t>(ParamType::Float)) {
						ParamService::Set(ownerList, entry, req.value.fVal, true);
					}
					else if(req.type == static_cast<uint8_t>(ParamType::Int32)) {
						ParamService::Set(ownerList, entry, req.value.iVal, true);
					}
				}
			}
			break;
		}
		case CommandID::ParamReset: {
			ParamService::Reset();
			break;
		}
		case CommandID::TopicRequestList: {
			SendTopicList();
			break;
		}
		default: {
			break;
		}
	}
}

bool InstinctLink::SendPacket(uint16_t packetID, const uint8_t* payload, uint8_t len) {
	if(linkPort == nullptr) {
		return false;
	}

	uint8_t frame[6 + maxPayloadSize];
	frame[0] = static_cast<uint8_t>(syncWord & 0xFF);
	frame[1] = static_cast<uint8_t>(syncWord >> 8);
	frame[2] = static_cast<uint8_t>(packetID & 0xFF);
	frame[3] = static_cast<uint8_t>(packetID >> 8);
	frame[4] = len;

	uint8_t crc = frame[2] ^ frame[3] ^ frame[4];

	for(uint8_t i = 0; i < len; i++) {
		frame[5 + i] = payload[i];
		crc ^= payload[i];
	}

	frame[5 + len] = crc;

	if(linkPort->Transmit(frame, static_cast<uint16_t>(6 + len)) == Status::Ok) {
		return true;
	}
	return false;
}

bool InstinctLink::SendParamInfo(uint16_t index) {
	ParamService::ParamList* ownerList = nullptr;
	const ParamService::ParamEntry* entry = ParamService::FindByIndex(index, &ownerList);
	if(entry == nullptr || entry->valPtr == nullptr) {
		return false;
	}

	ParamInfoPayload info{};
	info.index = index;
	info.hash = entry->hash;
	info.type = static_cast<uint8_t>(entry->valType);
	info.flags = entry->flags;
	info.defaultVal = entry->defaultVal;
	info.minVal = entry->minVal;
	info.maxVal = entry->maxVal;

	if(entry->valType == ParamType::Float) {
		info.currentVal.fVal = *static_cast<float*>(entry->valPtr);
	}
	else {
		info.currentVal.iVal = *static_cast<int32_t*>(entry->valPtr);
	}

	strncpy(info.name, entry->name, sizeof(info.name) - 1);

	return SendPacket(static_cast<uint16_t>(CommandID::ParamRequestList), reinterpret_cast<const uint8_t*>(&info), sizeof(ParamInfoPayload));
}

void InstinctLink::SendTopicList() {
	TopicBase* curr = Broker::GetTopicList();
	while(curr != nullptr) {
		TopicInfoPayload info{};
		info.topicID = curr->GetTopicID();
		info.instance = curr->GetInstance();
		info.payloadSize = curr->GetPayloadSize();
		if(curr->GetName() != nullptr) {
			strncpy(info.name, curr->GetName(), sizeof(info.name) - 1);
		}

		SendPacket(static_cast<uint16_t>(CommandID::TopicRequestList), reinterpret_cast<const uint8_t*>(&info), sizeof(TopicInfoPayload));

		curr = curr->nextTopic;
	}
}