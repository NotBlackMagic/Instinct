/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	SDK/Drivers/Radio/rcReceiver.cpp
 */

 #include "rcReceiver.hpp"

// CRSF specific constants
static constexpr uint8_t CRSF_SYNC_BYTE = 0xC8;
static constexpr uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
static constexpr uint8_t CRSF_FRAMETYPE_LINK_STATISTICS = 0x14;

RCReceiver::RCReceiver(UART& uart) : bus(uart) {
	this->config.protocol = Protocol::None;
	this->frameIndex = 0;
	this->timestamp = 0;
	ApplyDefaultChannels();
}

Status RCReceiver::Init(const Config& config) {
	if(config.protocol == Protocol::None) {
		return Status::Error;
	}

	this->config = config;
	ApplyDefaultChannels();
	this->frameIndex = 0;

	return Status::Ok;
}

Status RCReceiver::ProcessStream() {
	if(this->config.protocol == Protocol::None) {
		return Status::Error;
	}

	// Check Failsafe triggering, reception timeout
	LinkState previousState = this->channelData.linkState;
	if ((Time::GetUs() - this->channelData.timestamp) > FAILSAFE_TIMEOUT) {
		this->channelData.linkState = LinkState::LinkLost;
	}
	bool linkStateChanged = (this->channelData.linkState != previousState);

	uint32_t available = this->bus.Available();
	bool newFrame = false;

	if (available > 0) {
		uint8_t byte;
		while(this->bus.Receive(&byte, 1) > 0) {
			uint64_t now = Time::GetUs();
			
			// Reset frame buffer if large gap between bytes
			if((now - this->timestamp) > BYTE_TIMEOUT) {
				this->frameIndex = 0;
			}
			this->timestamp = now;

			switch(this->config.protocol) {
				case Protocol::IBus:
					newFrame = ParseIBus(byte);
					break;
				case Protocol::SBus:
					newFrame = ParseSBus(byte);
					break;
				case Protocol::CRSF:
					newFrame = ParseCRSF(byte);
					break;
				default:
					break;
			}
		}
	}

	// If either a new frame arrived OR the link dropped, tell the thread it's time to publish
	if (newFrame == true || linkStateChanged == true) {
		return Status::Ok;
	}

	// Otherwise, we are just waiting for more bytes
	return Status::Busy;
}

Status RCReceiver::GetChannelData(ChannelData& data) {
	data = this->channelData;
	return Status::Ok;
}

Status RCReceiver::SendTelemetry(uint8_t* payload, uint16_t length) {
	if(this->config.protocol != Protocol::CRSF) {
		// Telemetry push is primarily an ELRS/CRSF feature
		return Status::Error; 
	}

	return this->bus.Transmit(payload, length);
}

void RCReceiver::ApplyDefaultChannels() {
	for(uint8_t i = 0; i < MAX_CHANNELS; i++) {
		// Set default channel value, midpoint from [1000, 2000]
		this->channelData.channels[i] = 1500; 
	}
	this->channelData.channelCount = 0;
	this->channelData.linkState = LinkState::Disconnected;
	this->channelData.linkQuality = 0;
	this->channelData.rssi = -130;
	this->channelData.timestamp = Time::GetUs();
}

// ---------------------------------------------------------
// Protocol Parsers
// ---------------------------------------------------------
bool RCReceiver::ParseIBus(uint8_t byte) {
	this->buffer[this->frameIndex++] = byte;

	// Validate Sync Bytes
	if (this->frameIndex == 1 && this->buffer[0] != 0x20) {
		this->frameIndex = 0;
		return false;
	}
	if (this->frameIndex == 2 && this->buffer[1] != 0x40) {
		this->frameIndex = 0;
		return false;
	}

	// Wait/get full frame (32 bytes)
	if(this->frameIndex == 32) {
		// Calculate checksum
		uint16_t crc = 0xFFFF;
		for(uint8_t i = 0; i < 30; i++) {
			crc -= this->buffer[i];
		}
		uint16_t crcPkt = (this->buffer[31] << 8) | this->buffer[30];

		if(crc == crcPkt) {
			// Unpack 14 channels (Little Endian)
			for(uint8_t i = 0; i < 14; i++) {
				uint16_t ch = (this->buffer[3 + (i * 2)] << 8) | this->buffer[2 + (i * 2)];
				this->channelData.channels[i] = ch;
			}

			this->channelData.channelCount = 14;
			this->channelData.linkState = LinkState::Connected;
			this->channelData.timestamp = Time::GetUs();

			// Custom Firmware RSSI Injection Handling (https://github.com/povlhp/FlySkyRxFirmware)
			if (this->config.rssiChannelIndex >= 0 && this->config.rssiChannelIndex < 14) {
				uint16_t rssi = this->channelData.channels[this->config.rssiChannelIndex];
				
				// Clamp limits just in case
				if(rssi < 1000) {
					rssi = 1000;
				}
				if (rssi > 2000) {
					rssi = 2000;
				}

				// Map 1000-2000us to 0-100% Link Quality (LQ)
				this->channelData.linkQuality = (uint8_t)((rssi - 1000) / 10);
				
				// Approximate a dBm value from the percentage (rough scale for i-Bus)
				// -130dBm (0%) to -30dBm (100%)
				this->channelData.rssi = (int8_t)(-130 + this->channelData.linkQuality); 

				// Zero out the channel so it doesn't accidentally trigger a flight mode/switch
				this->channelData.channels[this->config.rssiChannelIndex] = 1500;
			}

			this->frameIndex = 0;
			return true;
		}
		this->frameIndex = 0;
	}
	return false;
}

bool RCReceiver::ParseSBus(uint8_t byte) {
	this->buffer[this->frameIndex++] = byte;

	// Validate Sync Byte
	if(this->frameIndex == 1 && this->buffer[0] != 0x0F) {
		this->frameIndex = 0;
		return false;
	}

	// Wait/get full frame (25 bytes)
	if(this->frameIndex == 25) {
		// Unpack 16 channels (11 bits each)
		this->channelData.channels[0] = ((this->buffer[1] | this->buffer[2] << 8) & 0x07FF);
		this->channelData.channels[1] = ((this->buffer[2] >> 3 | this->buffer[3] << 5) & 0x07FF);
		this->channelData.channels[2] = ((this->buffer[3] >> 6 | this->buffer[4] << 2 | this->buffer[5] << 10) & 0x07FF);
		this->channelData.channels[3] = ((this->buffer[5] >> 1 | this->buffer[6] << 7) & 0x07FF);
		this->channelData.channels[4] = ((this->buffer[6] >> 4 | this->buffer[7] << 4) & 0x07FF);
		this->channelData.channels[5] = ((this->buffer[7] >> 7 | this->buffer[8] << 1 | this->buffer[9] << 9) & 0x07FF);
		this->channelData.channels[6] = ((this->buffer[9] >> 2 | this->buffer[10] << 6) & 0x07FF);
		this->channelData.channels[7] = ((this->buffer[10] >> 5 | this->buffer[11] << 3) & 0x07FF);
		this->channelData.channels[8] = ((this->buffer[12] | this->buffer[13] << 8) & 0x07FF);
		this->channelData.channels[9] = ((this->buffer[13] >> 3 | this->buffer[14] << 5) & 0x07FF);
		this->channelData.channels[10] = ((this->buffer[14] >> 6 | this->buffer[15] << 2 | this->buffer[16] << 10) & 0x07FF);
		this->channelData.channels[11] = ((this->buffer[16] >> 1 | this->buffer[17] << 7) & 0x07FF);
		this->channelData.channels[12] = ((this->buffer[17] >> 4 | this->buffer[18] << 4) & 0x07FF);
		this->channelData.channels[13] = ((this->buffer[18] >> 7 | this->buffer[19] << 1 | this->buffer[20] << 9) & 0x07FF);
		this->channelData.channels[14] = ((this->buffer[20] >> 2 | this->buffer[21] << 6) & 0x07FF);
		this->channelData.channels[15] = ((this->buffer[21] >> 5 | this->buffer[22] << 3) & 0x07FF);

		// Map 11-bit S.Bus range (172-1811) to standard 1000-2000us
		for(uint8_t i = 0; i < 16; i++) {
			this->channelData.channels[i] = (uint16_t)((this->channelData.channels[i] * 0.625f) + 880);
		}
		this->channelData.channelCount = 16;

		// Check Flags (Byte 23)
		// bit 3: Failsafe active, bit 2: Frame lost
		bool frameLost = (this->buffer[23] & (1 << 2));
		bool failsafe  = (this->buffer[23] & (1 << 3));

		if(failsafe == true) {
			this->channelData.linkState = LinkState::Failsafe;
		} 
		else {
			this->channelData.linkState = LinkState::Connected;
			this->channelData.timestamp = Time::GetUs();
		}

		this->frameIndex = 0;
		return true;
	}
	return false;
}

bool RCReceiver::ParseCRSF(uint8_t byte) {
	this->buffer[this->frameIndex++] = byte;

	// Validate Sync Byte
	if(this->frameIndex == 1 && this->buffer[0] != CRSF_SYNC_BYTE) {
		this->frameIndex = 0;
		return false;
	}

	// Wait/get length byte
	if(this->frameIndex == 2) {
		uint8_t len = this->buffer[1];
		if(len > bufferSize - 2 || len < 2) {
			// Invalid length
			this->frameIndex = 0;
			return false;
		}
	}

	// 3. Wait for full frame (Length byte + 2 offset bytes)
	uint8_t expectedLength = this->buffer[1] + 2;
	if(this->frameIndex >= expectedLength) {
		// Validate CRC (calculated over Type + Payload)
		uint8_t crc = CalculateCRC(&this->buffer[2], expectedLength - 3);
		uint8_t crcPkt = this->buffer[expectedLength - 1];

		if(crc == crcPkt) {
			uint8_t frameType = this->buffer[2];
			if(frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
				// Unpack 11-bit channels (similar structure to S.Bus, but different mapping)
				// Payload starts at buffer[3]
				this->channelData.channels[0] = ((this->buffer[3] | this->buffer[4] << 8) & 0x07FF);
				this->channelData.channels[1] = ((this->buffer[4] >> 3 | this->buffer[5] << 5) & 0x07FF);
				this->channelData.channels[2] = ((this->buffer[5] >> 6 | this->buffer[6] << 2 | this->buffer[7] << 10) & 0x07FF);
				this->channelData.channels[3] = ((this->buffer[7] >> 1 | this->buffer[8] << 7) & 0x07FF);
				this->channelData.channels[4] = ((this->buffer[8] >> 4 | this->buffer[9] << 4) & 0x07FF);
				this->channelData.channels[5] = ((this->buffer[9] >> 7 | this->buffer[10] << 1 | this->buffer[11] << 9) & 0x07FF);
				this->channelData.channels[6] = ((this->buffer[11] >> 2 | this->buffer[12] << 6) & 0x07FF);
				this->channelData.channels[7] = ((this->buffer[12] >> 5 | this->buffer[13] << 3) & 0x07FF);
				this->channelData.channels[8] = ((this->buffer[14] | this->buffer[15] << 8) & 0x07FF);
				this->channelData.channels[9] = ((this->buffer[15] >> 3 | this->buffer[16] << 5) & 0x07FF);
				this->channelData.channels[10] = ((this->buffer[16] >> 6 | this->buffer[17] << 2 | this->buffer[18] << 10) & 0x07FF);
				this->channelData.channels[11] = ((this->buffer[18] >> 1 | this->buffer[19] << 7) & 0x07FF);
				this->channelData.channels[12] = ((this->buffer[19] >> 4 | this->buffer[20] << 4) & 0x07FF);
				this->channelData.channels[13] = ((this->buffer[20] >> 7 | this->buffer[21] << 1 | this->buffer[22] << 9) & 0x07FF);
				this->channelData.channels[14] = ((this->buffer[22] >> 2 | this->buffer[23] << 6) & 0x07FF);
				this->channelData.channels[15] = ((this->buffer[23] >> 5 | this->buffer[24] << 3) & 0x07FF);

				// Map 11-bit CRSF range (191-1792) to standard 1000-2000us
				for (uint8_t i = 0; i < 16; i++) {
					this->channelData.channels[i] = (uint16_t)((this->channelData.channels[i] * 0.625f) + 881);
				}

				this->channelData.channelCount = 16;
				this->channelData.linkState = LinkState::Connected;
				this->channelData.timestamp = Time::GetUs();
			} 
			else if(frameType == CRSF_FRAMETYPE_LINK_STATISTICS) {
				// Link Statistics parsing (Payload length 10 bytes)
				// buffer[3] = Uplink RSSI 1, buffer[4] = Uplink RSSI 2, buffer[5] = Uplink Link Quality
				this->channelData.rssi = -(int8_t)this->buffer[3]; 
				this->channelData.linkQuality = this->buffer[5];
			}

			this->frameIndex = 0;
			return true;
		}

		this->frameIndex = 0;
		return false;
	}
	return false;
}

// ---------------------------------------------------------
// Helpers
// ---------------------------------------------------------

uint8_t RCReceiver::CalculateCRC(const uint8_t* data, uint16_t length) {
	// CRSF uses CRC8 with polynomial 0xD5
	uint8_t crc = 0;
	for(uint16_t i = 0; i < length; i++) {
		crc ^= data[i];
		for(uint8_t j = 0; j < 8; j++) {
			if((crc & 0x80) == 0x80) {
				crc = (crc << 1) ^ 0xD5;
			}
			else {
				crc <<= 1;
			}
		}
	}
	return crc;
}