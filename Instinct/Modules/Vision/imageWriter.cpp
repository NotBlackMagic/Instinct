/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Vision/imageWriter.cpp
 */

#include "imageWriter.hpp"

// Allocate enough for a 54-byte header + 1024-byte color palette, aligned to 32 bytes
alignas(32) uint8_t bmpHeader[1088] = {0};
Status ImageWriter::SaveBMP(const VisionFrame& frame, FX_MEDIA& media, const char* fileName) {
	FX_FILE myFile;
	UINT status;

	if(frame.startAddress == nullptr || frame.payloadSize == 0) {
		return Status::Error;
	}

	if(media.fx_media_id != FX_MEDIA_ID) {
		LOG_WARN("Cannot save: SD Card not ready or not mounted.");
		return Status::Error;
	}

	// Generate BMP pixel format values
	uint32_t bytesPerPixel = 0;
	uint8_t bitsPerPixel = 0;
	uint32_t compressionMode = 0; // 0 = BI_RGB (Uncompressed), 3 = BI_BITFIELDS
	uint32_t headerSize = 0;
	switch (frame.format) {
		case PixelFormat::RGB565:
			bytesPerPixel = 2;
			bitsPerPixel = 16;
			compressionMode = 3; 
			headerSize = 512; // 54 byte standard + 12 byte masks + padding
			break;
		case PixelFormat::Grayscale:
		case PixelFormat::BayerRaw8:
			bytesPerPixel = 1;
			bitsPerPixel = 8;
			compressionMode = 0; 
			// 8-bit BMPs require a 256-color palette (1024 bytes) after the 54-byte header
			headerSize = 54 + 1024; 
			break;
		case PixelFormat::YUV422_YUYV:
		case PixelFormat::BayerRaw10:
		// case PixelFormat::Jpeg:
		// 	LOG_ERR("Unsupported PixelFormat for BMP export.");
		// 	return Status::Error;
		default:
			LOG_ERR("Unsupported PixelFormat for BMP export.");
			return Status::Error;
	}

	// Generate BMP header
	uint32_t dataSize = frame.width * frame.height * bytesPerPixel;
	uint32_t fileSize = headerSize + dataSize;

	// Construct Header securely as a byte array to avoid struct padding issues
	memset(bmpHeader, 0, sizeof(bmpHeader));

	// File Header (14 Bytes)
	bmpHeader[0] = 'B';
	bmpHeader[1] = 'M';
	bmpHeader[2] = (uint8_t)(fileSize);
	bmpHeader[3] = (uint8_t)(fileSize >> 8);
	bmpHeader[4] = (uint8_t)(fileSize >> 16);
	bmpHeader[5] = (uint8_t)(fileSize >> 24);

	// Clear reserved fields
	bmpHeader[6] = 0;
	bmpHeader[7] = 0;
	bmpHeader[8] = 0;
	bmpHeader[9] = 0;

	bmpHeader[10] = (uint8_t)(headerSize);
	bmpHeader[11] = (uint8_t)(headerSize >> 8);

	// DIB Header (BITMAPINFOHEADER - 40 Bytes)
	bmpHeader[14] = 40; // DIB Header size
	bmpHeader[15] = 0;
	bmpHeader[16] = 0;
	bmpHeader[17] = 0;

	// Width
	bmpHeader[18] = (uint8_t)(frame.width);
	bmpHeader[19] = (uint8_t)(frame.width >> 8);
	bmpHeader[20] = 0;
	bmpHeader[21] = 0;

	// Height (Negative to indicate Top-Down rendering)
	int32_t height = -frame.height;
	bmpHeader[22] = (uint8_t)(height);
	bmpHeader[23] = (uint8_t)(height >> 8);
	bmpHeader[24] = (uint8_t)(height >> 16);
	bmpHeader[25] = (uint8_t)(height >> 24);

	bmpHeader[26] = 1;	// Color Planes
	bmpHeader[27] = 0;
	bmpHeader[28] = bitsPerPixel;
	bmpHeader[29] = 0;
	bmpHeader[30] = compressionMode;
	bmpHeader[31] = 0;
	bmpHeader[32] = 0;
	bmpHeader[33] = 0;

	bmpHeader[34] = (uint8_t)(dataSize);
	bmpHeader[35] = (uint8_t)(dataSize >> 8);
	bmpHeader[36] = (uint8_t)(dataSize >> 16);
	bmpHeader[37] = (uint8_t)(dataSize >> 24);

	// --- Color Masks (12 Bytes) ---
	if(frame.format == PixelFormat::RGB565) {
		// Tells the viewer exactly how the 16 bits are split (5-6-5)
		// Red Mask: 0x0000F800
		bmpHeader[54] = 0x00;
		bmpHeader[55] = 0xF8;
		bmpHeader[56] = 0x00;
		bmpHeader[57] = 0x00;

		// Green Mask: 0x000007E0
		bmpHeader[58] = 0xE0;
		bmpHeader[59] = 0x07;
		bmpHeader[60] = 0x00;
		bmpHeader[61] = 0x00;

		// Blue Mask: 0x0000001F
		bmpHeader[62] = 0x1F;
		bmpHeader[63] = 0x00;
		bmpHeader[64] = 0x00;
		bmpHeader[65] = 0x00;
	}
	else if(frame.format == PixelFormat::Grayscale || frame.format == PixelFormat::BayerRaw8) {
		// Generate a standard 256-color grayscale palette
		for (int i = 0; i < 256; i++) {
			bmpHeader[54 + (i * 4)] = i; // Blue
			bmpHeader[54 + (i * 4) + 1] = i; // Green
			bmpHeader[54 + (i * 4) + 2] = i; // Red
			bmpHeader[54 + (i * 4) + 3] = 0; // Reserved
		}
	}

	// Force delete file if it already exists
	fx_file_delete(&media, const_cast<char*>(fileName));

	// Create and open a file
	status = fx_file_create(&media, const_cast<char*>(fileName));
	if(status != FX_SUCCESS) {
		LOG_WARN("Cannot save: SD Card not ready or not mounted.");
		return Status::Error;
	}

	status = fx_file_open(&media, &myFile, const_cast<char*>(fileName), FX_OPEN_FOR_WRITE);
	if(status != FX_SUCCESS) {
		LOG_ERR("Failed to open file on SD Card.");
		return Status::Error;
	}

	// Write Header
	status = fx_file_write(&myFile, bmpHeader, headerSize);
	if(status != FX_SUCCESS) {
		fx_file_close(&myFile);
		return Status::Error;
	}

	// Write the raw buffer directly to the file
	status = fx_file_write(&myFile, frame.startAddress, frame.payloadSize);
	if(status != FX_SUCCESS) {
		LOG_ERR("Failed to write frame data.");
		fx_file_close(&myFile);
		return Status::Error;
	}

	fx_file_close(&myFile);
	fx_media_flush(&media);

	LOG_INFO("Snapshot saved successfully!");

	return Status::Ok;
}

Status ImageWriter::SaveBinary(const VisionFrame& frame, FX_MEDIA& media, const char* fileName) {
	FX_FILE myFile;
	UINT status;

	if(frame.startAddress == nullptr || frame.payloadSize == 0) {
		return Status::Error;
	}

	if(media.fx_media_id != FX_MEDIA_ID) {
		LOG_WARN("Cannot save: SD Card not ready or not mounted.");
		return Status::Error;
	}

	// Force delete file if it already exists
	fx_file_delete(&media, const_cast<char*>(fileName));

	// Create the file
	status = fx_file_create(&media, const_cast<char*>(fileName));
	if(status != FX_SUCCESS) {
		LOG_ERR("Failed to create binary file on SD Card.");
		return Status::Error;
	}

	// Open the file
	status = fx_file_open(&media, &myFile, const_cast<char*>(fileName), FX_OPEN_FOR_WRITE);
	if(status != FX_SUCCESS) {
		LOG_ERR("Failed to open file on SD Card.");
		return Status::Error;
	}

	// Write the raw buffer directly to the file
	status = fx_file_write(&myFile, frame.startAddress, frame.payloadSize);
	if(status != FX_SUCCESS) {
		LOG_ERR("Failed to write frame data.");
		fx_file_close(&myFile);
		return Status::Error;
	}

	fx_file_close(&myFile);
	fx_media_flush(&media);

	LOG_INFO("Snapshot saved successfully!");

	return Status::Ok;
}