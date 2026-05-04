/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/jpeg.hpp
 * Author:  NotBlackMagic
 * Brief:   JPEG hardware codec driver for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"

#include "dmaChannel.hpp"
#include "status.hpp"

#include "tx_api.h"

/// @brief Driver for the STM32 Hardware JPEG Codec.
class Jpeg {
	public:
		/// @brief Input color space format.
		enum class ColorSpace : uint32_t {
			Grayscale = 0,
			YCbCr = 1,
			RGB = 2,
			CMYK = 3
		};

		/// @brief Chroma subsampling.
		enum class Subsampling : uint32_t {
			YUV444 = 0,		// MCU: One 8x8 Y block + one 8x8 Cb block + one 8x8 Cr block = Total 192 bytes
			YUV422 = 1,		// MCU: Two 8x8 Y block + one 8x8 Cb block + one 8x8 Cr block = Total 256 bytes
			YUV420 = 2,		// MCU: Four 8x8 Y block + one 8x8 Cb block + one 8x8 Cr block = Total 384 bytes
			None = 3 		// Used for Grayscale or RGB bypass
		};

		enum class Event {
			EncodeComplete,
			HeaderParsingComplete,
			Error
		};

		/// @brief Parameters for the specific image being encoded.
		struct ImageParams {
			uint32_t width;
			uint32_t height;
			ColorSpace colorSpace;
			Subsampling subsampling;
			uint32_t quality;       ///< Compression quality (1-100)
		};

		/// @brief JPEG peripheral hardware configuration.
		struct Config {
			void (*EventCallback)(void* ctx, Event evt);
			void* callbackContext;
		};

		// Delete copy constructors.
		Jpeg(const Jpeg&) = delete;
		Jpeg& operator=(const Jpeg&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., JPEG).
		Jpeg(JPEG_TypeDef* instance);

		/// @brief Initializes the JPEG clock, underlying DMA channels, and interrupts.
		/// @param config JPEG hardware and DMA configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config& config);

		/// @brief Starts an asynchronous JPEG encoding process.
		/// @param params	Information about the image dimensions and format.
		/// @return Status::Ok if encoding started, Status::Busy if already encoding.
		Status Start(const ImageParams& params);

		/// @brief Aborts the current encoding process.
		/// @return Status::Ok.
		Status Stop();

		/// @brief Dynamically enables or disables the RX DMA request 
		void EnableRxDMA(bool enable);

		/// @brief Dynamically enables or disables the TX DMA request
		void EnableTxDMA(bool enable);

		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler();

	private:
		JPEG_TypeDef* instance;
		IRQn_Type irqCall;
		uint8_t irqPriority;
		Config config;

		uint32_t finalJpegSize;

		struct HuffmanCodeTable {
			uint8_t codeLength[256];
			uint32_t huffmanCode[256];
		};

		/// @brief Computes the Huffman sizes and codes from the raw BITS array
		Status ComputeHuffmanCodes(const uint8_t* bits, uint8_t* huffSize, uint32_t* huffCode, uint32_t* lastK);
		
		/// @brief Formats and loads a DC table into the HUFFENC_DCx registers
		Status LoadDCTable(const uint8_t* bits, const uint8_t* vals, volatile uint32_t* tableAddress);
		
		/// @brief Formats and loads an AC table into the HUFFENC_ACx registers
		Status LoadACTable(const uint8_t* bits, const uint8_t* vals, volatile uint32_t* tableAddress);
		
		/// @brief Writes the raw DHT payload into the DHTMEM registers for the JPEG header
		Status LoadDHTMem();

		/// @brief Generates and loads the quantization and Huffman tables based on the requested quality.
		Status LoadTables(uint32_t quality);
		
		/// @brief Configures the hardware registers for image dimensions and format.
		Status ConfigureEncoding(const ImageParams& params);
};