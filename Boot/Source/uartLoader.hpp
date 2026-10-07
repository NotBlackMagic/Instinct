/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Boot/Source/uartLoader.hpp
 * Author:	NotBlackMagic
 * Brief:	Utility to manage reception of a firmware over UART and save/stage it to RAM. Use in combination with "flashUART.py" python script.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "logger.hpp"
#include "status.hpp"
#include "system.hpp"
#include "uart.hpp"

#define APP_MAX_SIZE							(3U * 1024U * 1024U)	// 3 MB
#define SYNC_0									0x55
#define SYNC_1									0xAA
#define MAX_PAYLOAD_SIZE						1024

/// @brief UART loader command defines.
enum BootCommand : uint8_t {
	Ping = 0x01,
	Start = 0x02,
	Data = 0x03,
	Finish = 0x04
};

/// @brief UART loader command responses defines.
enum BootResponse : uint8_t {
	ACK = 0x06,
	NACK = 0x15
};

/// @brief UART loader error codes.
enum BootError : uint8_t {
	None = 0x00,
	BadCRC = 0x01,
	BadSequence = 0x02,
	SizeExceeded = 0x03,
	UnexpectedState = 0x04,
	PayloadTooLarge = 0x05,
	ImageCRCMismatch = 0x06
};

/// @brief UART loader download status tracking structure.
struct DownloadStatus {
	uint32_t size;			///< Expected Firmware size, parsed from header/first packet
	uint32_t count;			///< Keeps track of received bytes so far
	uint32_t crc32;			///< Expected Firmware CRC32, parsed from header/first packet
	uint8_t nextSequence;	///< Keeps track of next sequence
	bool active;			///< Current Firmware download session status
};

/// @brief Calculates CRC16, using CRC-16-CCITT (0x1021) polynomial, for passed data array.
/// @param data Pointer to data array
/// @param len	Length of data array
/// @return Calculated CR16 value
uint16_t CalculateCRC16(const uint8_t *data, uint16_t len);

/// @brief Calculates CRC32, using 0xEDB88320U polynomial, for passed data array.
/// @param data Pointer to data array
/// @param len	Length of data array
/// @return Calculated CRC32 value
uint32_t CalculateCRC32(const uint8_t *data, uint32_t len);

/// @brief Helper function to write UART loader packets.
/// @param uart			Pointer to used UART instance
/// @param respType		Response type
/// @param sequence		Packet sequence number
/// @param errorCode	Packet error code
/// @return Status::Ok
Status LoaderWrite(UART *uart, BootResponse respType, uint8_t sequence, BootError errorCode);

/// @brief Helper function to read a complete UART loader packet.
/// @param uart		Pointer to used UART instance
/// @param dest		Pointer to buffer for read data
/// @param count	To receive byte count
/// @param timeout	Read timeout, in ms
/// @return Status::Ok
Status LoaderRead(UART *uart, uint8_t *dest, uint32_t count, uint32_t timeout);

/// @brief Function to handle full firmware reception, over UART, and staging in RAM.
/// @param uart		Pointer to used UART instance
/// @param ramAddr	RAM address to staging area (where the received firmware will be written)
/// @param fwSize	Returns size of the received firmware
/// @return Status::Ok if full firmware received and CRC check passes, Status::Error otherwise.
Status LoaderFWToPSRAM(UART *uart, uint32_t ramAddr, uint32_t &fwSize);