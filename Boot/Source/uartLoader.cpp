/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Boot/Source/uartLoader.cpp
 */

#include "uartLoader.hpp"

static uint8_t payloadBuffer[MAX_PAYLOAD_SIZE];

uint16_t CalculateCRC16(const uint8_t *data, uint16_t len) {
	uint16_t crc = 0xFFFF;
	for(uint16_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for(uint8_t bit = 0; bit < 8; bit++) {
			if(crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021; // CRC-16-CCITT
			}
			else {
				crc = crc << 1;
			}
		}
	}
	return crc;
}

uint32_t CalculateCRC32(const uint8_t *data, uint32_t len) {
	uint32_t crc = 0xFFFFFFFF;
	for(uint32_t i = 0; i < len; i++) {
		crc ^= data[i];
		for(uint8_t bit = 0; bit < 8; bit++) {
			if(crc & 1) {
				crc = (crc >> 1) ^ 0xEDB88320U;
			}
			else {
				crc = (crc >> 1);
			}
		}
	}
	return ~crc;
}

Status LoaderWrite(UART *uart, BootResponse respType, uint8_t sequence, BootError errorCode) {
	uint8_t txFrame[7];
	txFrame[0] = SYNC_0;
	txFrame[1] = SYNC_1;
	txFrame[2] = respType;
	txFrame[3] = sequence;
	txFrame[4] = errorCode;

	uint16_t crc = CalculateCRC16(&txFrame[2], 3);
	txFrame[5] = (uint8_t)(crc & 0xFF);
	txFrame[6] = (uint8_t)((crc >> 8) & 0xFF);

	return uart->Transmit(txFrame, 7);
}

Status LoaderRead(UART *uart, uint8_t *dest, uint32_t count, uint32_t timeout) {
	uint32_t readCount = 0;
	uint32_t timestamp = Time::GetMs();

	while(readCount < count) {
		if(uart->Available() > 0) {
			uint32_t chunk = uart->Receive(&dest[readCount], count - readCount);
			readCount += chunk;
		}

		if((Time::GetMs() - timestamp) > timeout) {
			return Status::Timeout;
		}
	}
	return Status::Ok;
}

Status LoaderFWToPSRAM(UART *uart, uint32_t ramAddr, uint32_t &fwSize) {
	DownloadStatus context = {
		.size = 0,
		.count = 0,
		.crc32 = 0,
		.nextSequence = 0,
		.active = false
	};

	uint8_t *stagingRAM = (uint8_t *)ramAddr;

	// Flush any pending RX bytes (like stray terminal input)
	uint8_t buf[64];
	while(uart->Available() > 0) {
		uart->Receive(buf, sizeof(buf));
	}

	while(1) {
		// Wait for Sync word pattern of: 0x55 0xAA
		uint8_t byte = 0;
		if(LoaderRead(uart, &byte, 1, 100) != Status::Ok || byte != SYNC_0) {
			continue;
		}
		if(LoaderRead(uart, &byte, 1, 50) != Status::Ok || byte != SYNC_1) {
			continue;
		}

		// Read Frame Header (Command: 1B, Sequence: 1B, Length: 2B)
		uint8_t header[4];
		if(LoaderRead(uart, header, 4, 100) != Status::Ok) {
			continue;
		}

		uint8_t command = header[0];
		uint8_t sequence = header[1];
		uint16_t length = (uint16_t)(header[2] | (header[3] << 8));
		if(length > MAX_PAYLOAD_SIZE) {
			LoaderWrite(uart, BootResponse::NACK, sequence, BootError::PayloadTooLarge);
			continue;
		}

		// Read Payload and CRC16
		if(length > 0) {
			if(LoaderRead(uart, payloadBuffer, length, 500) != Status::Ok) {
				continue;
			}
		}

		if(LoaderRead(uart, buf, 2, 100) != Status::Ok) {
			continue;
		}

		uint16_t rxCRC = (uint16_t)(buf[0] | (buf[1] << 8));

		// Calculate and validate Packet CRC16
		uint16_t calcCRC = 0xFFFF;
		// Feed header
		for(uint8_t i = 0; i < 4; i++) {
			calcCRC ^= (uint16_t)header[i] << 8;
			for(uint8_t bit = 0; bit < 8; bit++) {
				calcCRC = (calcCRC & 0x8000) ? ((calcCRC << 1) ^ 0x1021) : (calcCRC << 1);
			}
		}
		// Feed payload
		for(uint16_t i = 0; i < length; i++) {
			calcCRC ^= (uint16_t)payloadBuffer[i] << 8;
			for(uint8_t bit = 0; bit < 8; bit++) {
				calcCRC = (calcCRC & 0x8000) ? ((calcCRC << 1) ^ 0x1021) : (calcCRC << 1);
			}
		}

		if(calcCRC != rxCRC) {
			LoaderWrite(uart, BootResponse::NACK, sequence, BootError::BadCRC);
			continue;
		}

		// Handle Commands
		switch(command) {
			case BootCommand::Ping: {
				LoaderWrite(uart, BootResponse::ACK, sequence, BootError::None);
				break;
			}

			case BootCommand::Start: {
				if(length != 8) {
					LoaderWrite(uart, BootResponse::NACK, sequence, BootError::UnexpectedState);
					break;
				}

				context.size =	((uint32_t)payloadBuffer[0]) | 
								((uint32_t)payloadBuffer[1] << 8) | 
								((uint32_t)payloadBuffer[2] << 16) | 
								((uint32_t)payloadBuffer[3] << 24);

				context.crc32 = ((uint32_t)payloadBuffer[4]) | 
								((uint32_t)payloadBuffer[5] << 8) | 
								((uint32_t)payloadBuffer[6] << 16) | 
								((uint32_t)payloadBuffer[7] << 24);

				if(context.size == 0 || context.size > APP_MAX_SIZE) {
					LoaderWrite(uart, BootResponse::NACK, sequence, BootError::SizeExceeded);
					break;
				}

				context.count = 0;
				context.nextSequence = (uint8_t)(sequence + 1);
				context.active = true;

				LOG_INFO("UART FW flashing: Download started. Size: %lu bytes", context.size);
				LoaderWrite(uart, BootResponse::ACK, sequence, BootError::None);
				break;
			}

			case BootCommand::Data: {
				if(context.active == false) {
					LoaderWrite(uart, BootResponse::NACK, sequence, BootError::UnexpectedState);
					break;
				}

				if(sequence != context.nextSequence) {
					LoaderWrite(uart, BootResponse::NACK, sequence, BootError::BadSequence);
					break;
				}

				if((context.count + length) > context.size) {
					LoaderWrite(uart, BootResponse::NACK, sequence, BootError::SizeExceeded);
					break;
				}

				// Copy chunk to HyperRAM (PSRAM)
				memcpy(&stagingRAM[context.count], payloadBuffer, length);
				context.count = context.count + length;
				context.nextSequence = context.nextSequence + 1;

				LoaderWrite(uart, BootResponse::ACK, sequence, BootError::None);
				break;
			}

			case BootCommand::Finish: {
				if(context.active == false || (context.count != context.size)) {
					LoaderWrite(uart, BootResponse::NACK, sequence, BootError::UnexpectedState);
					break;
				}

				// Push dirty D-Cache lines to physical HyperRAM before verifying
				if(SCB->CCR & SCB_CCR_DC_Msk) {
					SCB_CleanInvalidateDCache();
				}

				uint32_t calcCRC = CalculateCRC32(stagingRAM, context.count);
				if(calcCRC != context.crc32) {
					LoaderWrite(uart, BootResponse::NACK, sequence, BootError::ImageCRCMismatch);
					LOG_ERR("UART FW flashing: CRC32 mismatch! Exp: 0x%08lX Calc: 0x%08lX", (unsigned long)context.crc32, (unsigned long)calcCRC);
					context.active = false;
					return Status::Error;
				}

				LoaderWrite(uart, BootResponse::ACK, sequence, BootError::None);
				LOG_INFO("UART FW flashing: FW ready in PSRAM (%lu bytes, CRC32: 0x%08lX)", context.count, (unsigned long)calcCRC);
				// Flush UART
				uart->Flush();
				fwSize = context.count;
				return Status::Ok;
			}

			default: {
				LoaderWrite(uart, BootResponse::NACK, sequence, BootError::UnexpectedState);
				break;
			}
		}
	}
}