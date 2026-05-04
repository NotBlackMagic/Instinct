/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/jpeg.cpp
 */

#include "jpeg.hpp"

// The following Huffman tables, are based on the JPEG standard as defined by the ITU-T Recommendation T.81.
// DC Luminance
static const uint8_t stdHuffDCLumBits[16] = { 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
static const uint8_t stdHuffDCLumVal[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xa, 0xb };

// DC Chrominance
static const uint8_t stdHuffDCChromBits[16] = { 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };
static const uint8_t stdHuffDCChromVal[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xa, 0xb };

// AC Luminance
static const uint8_t huffACTableSize = 162;
static const uint8_t stdHuffACLumBits[16] = { 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d };
static const uint8_t stdHuffACLumVal[huffACTableSize] = {
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
	0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
	0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
	0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
	0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa
};

// AC Chrominance
static const uint8_t stdHuffACChromBits[16] = { 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77 };
static const uint8_t stdHuffACChromVal[huffACTableSize] = {
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
	0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
	0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
	0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
	0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
	0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
	0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
	0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
	0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa
};

// These are the sample quantization tables given in JPEG spec ISO/IEC 10918-1 standard , section K.1.
static constexpr uint8_t quantTableSize = 64;
static const uint8_t lumQuantTable[quantTableSize] = {
	16, 11, 10, 16, 24, 40, 51, 61,
	12, 12, 14, 19, 26, 58, 60, 55,
	14, 13, 16, 24, 40, 57, 69, 56,
	14, 17, 22, 29, 51, 87, 80, 62,
	18, 22, 37, 56, 68, 109, 103, 77,
	24, 35, 55, 64, 81, 104, 113, 92,
	49, 64, 78, 87, 103, 121, 120, 101,
	72, 92, 95, 98, 112, 100, 103, 99
};

static const uint8_t chromaQuantTable[quantTableSize] = {
	17, 18, 24, 47, 99, 99, 99, 99,
	18, 21, 26, 66, 99, 99, 99, 99,
	24, 26, 56, 99, 99, 99, 99, 99,
	47, 66, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99
};

static const uint8_t zigzagOrder[quantTableSize] = {
	0, 1, 8, 16, 9, 2, 3, 10,
	17, 24, 32, 25, 18, 11, 4, 5,
	12, 19, 26, 33, 40, 48, 41, 34,
	27, 20, 13, 6, 7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36,
	29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46,
	53, 60, 61, 54, 47, 55, 62, 63
};

Jpeg::Jpeg(JPEG_TypeDef* instance) {
	this->instance = instance;
	this->irqPriority = 0x0E; 	//Lowest priority
}

Status Jpeg::Init(const Config& config) {
	this->config = config;

	// Enable bus clocks
	if(this->instance == JPEG) {
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_JPEG);
		this->irqCall = JPEG_IRQn;
	}
	else {
		return Status::Error;
	}

	// Configure JPEG Interface


	// Configure Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);

	// Enable JPEG (register access ONLY possible when enabled!) initially and clear flags
	MODIFY_REG(this->instance->CR, JPEG_CR_JCEN, JPEG_CR_JCEN);
	WRITE_REG(this->instance->CFR, JPEG_CFR_CEOCF | JPEG_CFR_CHPDF);

	return Status::Ok;
}

Status Jpeg::Start(const ImageParams& params) {
	// Set Image Dimensions and Subsampling
	Status status = this->ConfigureEncoding(params);
	if(status != Status::Ok) {
		return status;
	}

	// Load Quantization Tables based on Quality (1-100)
	status = this->LoadTables(params.quality);
	if(status != Status::Ok) {
		return status;
	}

	// Flush FIFOs
	MODIFY_REG(this->instance->CR, 0, JPEG_CR_IFF | JPEG_CR_OFF);

	// Clear flags and Enable interrupts/DMA
	WRITE_REG(this->instance->CFR, JPEG_CFR_CEOCF | JPEG_CFR_CHPDF);
	MODIFY_REG(this->instance->CR, 0, JPEG_CR_EOCIE | JPEG_CR_IDMAEN | JPEG_CR_ODMAEN);

	// Start Hardware Codec
	MODIFY_REG(this->instance->CONFR0, 0, JPEG_CONFR0_START);

	return Status::Ok;
}

Status Jpeg::Stop() {
	MODIFY_REG(this->instance->CONFR0, JPEG_CONFR0_START, 0x00);
	MODIFY_REG(this->instance->CR, JPEG_CR_IDMAEN | JPEG_CR_ODMAEN | JPEG_CR_EOCIE, 0x00);
	return Status::Ok;
}

void Jpeg::EnableRxDMA(bool enable) {
	if(enable == true) {
		MODIFY_REG(this->instance->CR, 0, JPEG_CR_IDMAEN);
	}
	else {
		MODIFY_REG(this->instance->CR, JPEG_CR_IDMAEN, 0x00);
	}
}

void Jpeg::EnableTxDMA(bool enable) {
	if(enable == true) {
		MODIFY_REG(this->instance->CR, 0, JPEG_CR_ODMAEN);
	}
	else {
		MODIFY_REG(this->instance->CR, JPEG_CR_ODMAEN, 0x00);
	}
}

Status Jpeg::ConfigureEncoding(const ImageParams& params) {
	// Configure JPEG Codec
	MODIFY_REG(this->instance->CONFR1, JPEG_CONFR1_DE, 0x00);	// Set to Encoding mode
	MODIFY_REG(this->instance->CONFR1, 0x00, JPEG_CONFR1_HDR);	// Enable Header processing/generation

	// Set Dimensions
	MODIFY_REG(this->instance->CONFR1, JPEG_CONFR1_YSIZE, (params.height << 16) & JPEG_CONFR1_YSIZE_Msk);
	MODIFY_REG(this->instance->CONFR3, JPEG_CONFR3_XSIZE, (params.width << 16) & JPEG_CONFR3_XSIZE_Msk);

	uint32_t hFactor = 8;
	uint32_t vFactor = 8;

	// Configure Color Space & Subsampling
	if(params.colorSpace == ColorSpace::Grayscale) {
		MODIFY_REG(this->instance->CONFR1, JPEG_CONFR1_NF_Msk, (0x00 << JPEG_CONFR1_NF_Pos));					// Number of color components: 0 -> Grayscale 
		MODIFY_REG(this->instance->CONFR1, JPEG_CONFR1_COLORSPACE_Msk, (0x00 << JPEG_CONFR1_COLORSPACE_Pos));	// Color space: 0 -> Grayscale (1 quantization table)
		MODIFY_REG(this->instance->CONFR1, JPEG_CONFR1_NS_Msk, (0x00 << JPEG_CONFR1_NS_Pos));					// Number of components for scan (minus 1): 1
		// Component 0 (grayscale only has one component/one dimension)
		MODIFY_REG(this->instance->CONFR4, JPEG_CONFR4_VSF_Msk, (0x01 << JPEG_CONFR4_VSF_Pos));		// Vertical sampling factor
		MODIFY_REG(this->instance->CONFR4, JPEG_CONFR4_HSF_Msk, (0x01 << JPEG_CONFR4_HSF_Pos));		// Horizontal sampling factor
	}
	else if(params.colorSpace == ColorSpace::YCbCr) {
		MODIFY_REG(this->instance->CONFR1, JPEG_CONFR1_NF_Msk, (0x02 << JPEG_CONFR1_NF_Pos));					// Number of color components: 0 -> YUV or RGB (3 components) 
		MODIFY_REG(this->instance->CONFR1, JPEG_CONFR1_COLORSPACE_Msk, (0x01 << JPEG_CONFR1_COLORSPACE_Pos));	// Color space: 0 -> YUV (2 quantization tables)
		MODIFY_REG(this->instance->CONFR1, JPEG_CONFR1_NS_Msk, (0x02 << JPEG_CONFR1_NS_Pos));					// Number of components for scan (minus 1): 3
	
		uint32_t hsf = 0;			// Horizontal sampling factor
		uint32_t vsf = 0;			// Vertical sampling factor
		uint32_t yBlockNb = 0x00;	// Number of blocks
		if(params.subsampling == Subsampling::YUV420) {
			hFactor = 16; 
			vFactor = 16;
			hsf = 2;
			vsf = 2;
			yBlockNb = 3; // 4 blocks of 8x8
		}
		else if(params.subsampling == Subsampling::YUV422) {
			hFactor = 16; 
			vFactor = 8;
			hsf = 2;
			vsf = 1;
			yBlockNb = 1; // 2 blocks of 8x8
		}
		else {
			// Default is YUV444
			hFactor = 8;
			vFactor = 8;
			hsf = 1;
			vsf = 1;
			yBlockNb = 0;
		}

		// Component 0 (Y)
		WRITE_REG(this->instance->CONFR4,	(hsf << JPEG_CONFR4_HSF_Pos) |
											(vsf << JPEG_CONFR4_VSF_Pos) |
											(yBlockNb << JPEG_CONFR4_NB_Pos));
		// Component 1 (Cb)
		WRITE_REG(this->instance->CONFR5,	(0x01 << JPEG_CONFR5_HSF_Pos) |	// Horizontal sampling factor
											(0x01 << JPEG_CONFR5_VSF_Pos) |	// Vertical sampling factor
											(0x00 << JPEG_CONFR5_NB_Pos) |	// Number of blocks
											(0x01 << JPEG_CONFR5_QT_Pos) |	// Quantization table
											JPEG_CONFR5_HA |				// Huffman AC table
											JPEG_CONFR5_HD);				// Huffman DC table
		// Component 2 (Cr)
		WRITE_REG(this->instance->CONFR6,	(0x01 << JPEG_CONFR6_HSF_Pos) |	// Horizontal sampling factor
											(0x01 << JPEG_CONFR6_VSF_Pos) |	// Vertical sampling factor
											(0x00 << JPEG_CONFR6_NB_Pos) |	// Number of blocks
											(0x01 << JPEG_CONFR6_QT_Pos) |	// Quantization table
											JPEG_CONFR6_HA |				// Huffman AC table
											JPEG_CONFR6_HD);				// Huffman DC table
		// // Component 3 (???)
		// WRITE_REG(this->instance->CONFR7,	(0x01 << JPEG_CONFR7_HSF_Pos) |	// Horizontal sampling factor
		// 									(0x01 << JPEG_CONFR7_VSF_Pos) |	// Vertical sampling factor
		// 									(0x00 << JPEG_CONFR7_NB_Pos) |	// Number of blocks
		// 									(0x01 << JPEG_CONFR7_QT_Pos) |	// Quantization table
		// 									JPEG_CONFR7_HA |				// Huffman AC table
		// 									JPEG_CONFR7_HD);				// Huffman DC table
	}

	// Calculate Total MCUs
	uint32_t hMcu = (params.width + hFactor - 1) / hFactor;
	uint32_t vMcu = (params.height + vFactor - 1) / vFactor;
	uint32_t totalMcu = (hMcu * vMcu) - 1;
	WRITE_REG(this->instance->CONFR2, totalMcu & JPEG_CONFR2_NMCU_Msk);

	return Status::Ok;
}

Status Jpeg::LoadTables(uint32_t quality) {
	uint32_t scaleFactor;

	if(quality >= 50 && quality <= 100) {
		scaleFactor = 200 - (quality * 2);
	}
	else if(quality > 0) {
		scaleFactor = 5000 / quality;
	}
	else {
		scaleFactor = 50; // Default
	}

	// Luma Quantization Table (QMEM0)
	uint32_t quantRow = 0;
	uint32_t quantVal = 0;
	volatile uint32_t* tableAddress = this->instance->QMEM0;
	for(uint32_t i = 0; i < quantTableSize; i += 4) {
		quantRow = 0;
		for(uint32_t j = 0; j < 4; j++) {
			quantVal = ((lumQuantTable[zigzagOrder[i + j]] * scaleFactor) + 50) / 100;
			if(quantVal == 0) {
				quantVal = 1;
			}
			if(quantVal > 255) {
				quantVal = 255;
			}
			quantRow |= ((quantVal & 0xFF) << (8 * j));
		}
		*tableAddress = quantRow;
		tableAddress = tableAddress + 1;
	}

	// Chroma Quantization Table (QMEM1)
	tableAddress = this->instance->QMEM1;
	for (uint32_t i = 0; i < quantTableSize; i += 4) {
		uint32_t quantRow = 0;
		for (uint32_t j = 0; j < 4; j++) {
			uint32_t quantVal = ((chromaQuantTable[zigzagOrder[i + j]] * scaleFactor) + 50) / 100;
			if(quantVal == 0) {
				quantVal = 1;
			}
			else if(quantVal > 255) {
				quantVal = 255;
			}
			quantRow |= ((quantVal & 0xFF) << (8 * j));
		}
		*tableAddress++ = quantRow;
	}

	// Load baseline Huffman tables
	// Load DC0 (Luma) and DC1 (Chroma)
	Status status = this->LoadDCTable(stdHuffDCLumBits, stdHuffDCLumVal, this->instance->HUFFENC_DC0);
	if(status != Status::Ok) {
		return status;
	}
	status = this->LoadDCTable(stdHuffDCChromBits, stdHuffDCChromVal, this->instance->HUFFENC_DC1);
	if(status != Status::Ok) {
		return status;
	}

	// Load AC0 (Luma) and AC1 (Chroma)
	status = this->LoadACTable(stdHuffACLumBits, stdHuffACLumVal, this->instance->HUFFENC_AC0);
	if(status != Status::Ok) {
		return status;
	}
	status = this->LoadACTable(stdHuffACChromBits, stdHuffACChromVal, this->instance->HUFFENC_AC1);
	if(status != Status::Ok) {
		return status;
	}

	// Load the raw DHT markers into memory so the hardware can inject them into the JPEG header
	status = this->LoadDHTMem();
	if(status != Status::Ok) {
		return status;
	}

	return Status::Ok;
}

Status Jpeg::ComputeHuffmanCodes(const uint8_t* bits, uint8_t* huffSize, uint32_t* huffCode, uint32_t* lastK) {
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t code = 0;

	// Generate sizes
	for(uint32_t k = 0; k < 16; k++) {
		i = bits[k];
		if((j + i) > 256) {
			// Table overflow!
			return Status::Error;
		}
		while(i != 0) {
			huffSize[j] = k + 1;
			j = j + 1;
			i = i - 1;
		}
	}
	huffSize[j] = 0;
	*lastK = j;

	// Generate codes
	uint32_t si = huffSize[0];
	j = 0;
	while(huffSize[j] != 0) {
		while(huffSize[j] == si) {
			huffCode[j] = code;
			j = j + 1;
			code = code + 1;
		}
		//code must fit in "size" bits (si), no code is allowed to be all ones
		if(si > 31 || ((uint32_t) code) >= (((uint32_t) 1) << si)) {
			return Status::Error;
		}
		code = code << 1;
		si = si + 1;
	}

	return Status::Ok;
}

Status Jpeg::LoadDCTable(const uint8_t* bits, const uint8_t* vals, volatile uint32_t* tableAddress) {
	uint8_t huffSize[257];
	uint32_t huffCode[257];
	uint32_t lastK;
	HuffmanCodeTable sizeCodeTable;

	if(ComputeHuffmanCodes(bits, huffSize, huffCode, &lastK) != Status::Ok) {
		return Status::Error;
	}

	// Map to HW table layout (Figure C.3)
	for(uint32_t k = 0; k < lastK; k++) {
		uint32_t i = vals[k];
		if(i >= 12) {
			return Status::Error;
		}
		sizeCodeTable.huffmanCode[i] = huffCode[k];
		sizeCodeTable.codeLength[i] = huffSize[k] - 1;
	}

	// Write to HW Registers (12 values packed into 32-bit registers)
	volatile uint32_t* addressDef = tableAddress + (12 / 2); 
	*addressDef++ = 0x0FFF0FFF;
	*addressDef++ = 0x0FFF0FFF;

	volatile uint32_t* address = tableAddress + (12 / 2);
	uint32_t i = 12;
	while(i > 1) {
		i = i - 1;
		address = address - 1;
		// CRITICAL: Always mask Huffman codes with 0xFF (`code & 0xFF`) before packing into HUFFENC RAM.
		// If unmasked, 16-bit AC codes will overwrite the length bitfield and fatally corrupt the JPEG bitstream!
		uint32_t msb = (sizeCodeTable.codeLength[i] << 8) | (sizeCodeTable.huffmanCode[i] & 0xFF);
		i = i - 1;
		uint32_t lsb = (sizeCodeTable.codeLength[i] << 8) | (sizeCodeTable.huffmanCode[i] & 0xFF);
		*address = lsb | (msb << 16);
	}
	return Status::Ok;
}

Status Jpeg::LoadACTable(const uint8_t* bits, const uint8_t* vals, volatile uint32_t* tableAddress) {
	uint8_t huffSize[257];
	uint32_t huffCode[257];
	uint32_t lastK;
	HuffmanCodeTable sizeCodeTable;

	if(ComputeHuffmanCodes(bits, huffSize, huffCode, &lastK) != Status::Ok) {
		return Status::Error;
	}

	// Map to HW table layout (Figure C.3)
	for(uint32_t k = 0; k < lastK; k++) {
		uint32_t i = vals[k];
		
		if(i == 0x00) {
			i = huffACTableSize - 2;	// EOB Code
		}
		else if(i == 0xF0) {
			i = huffACTableSize - 1;	// ZRL Code
		}
		else {
			i = (((i & 0xF0) >> 4) * 10) + (i & 0x0F) - 1;
		}
		
		if(i >= huffACTableSize) {
			return Status::Error;
		}
		
		sizeCodeTable.huffmanCode[i] = huffCode[k];
		sizeCodeTable.codeLength[i] = huffSize[k] - 1;
	}

	// Write HW default internal codes for AC
	// Default values settings: 162:167 FFFh , 168:175 FD0h_FD7h
	// Locations 162:175 of each AC table contain information used internally by the core
	volatile uint32_t* addressDef = tableAddress + (huffACTableSize / 2);
	*addressDef++ = 0x0FFF0FFF;
	*addressDef++ = 0x0FFF0FFF;
	*addressDef++ = 0x0FFF0FFF;
	*addressDef++ = 0x0FD10FD0;
	*addressDef++ = 0x0FD30FD2;
	*addressDef++ = 0x0FD50FD4;
	*addressDef++ = 0x0FD70FD6;
	// End of Locations 162:175 

	// Write 162 values packed into 32-bit registers
	volatile uint32_t* address = tableAddress + (huffACTableSize / 2);
	uint32_t i = huffACTableSize;
	while(i > 1) {
		i = i - 1;
		address = address - 1;
		// CRITICAL: Always mask Huffman codes with 0xFF (`code & 0xFF`) before packing into HUFFENC RAM.
		// If unmasked, 16-bit AC codes will overwrite the length bitfield and fatally corrupt the JPEG bitstream!
		uint32_t msb = (sizeCodeTable.codeLength[i] << 8) | (sizeCodeTable.huffmanCode[i] & 0xFF);
		i = i - 1;
		uint32_t lsb = (sizeCodeTable.codeLength[i] << 8) | (sizeCodeTable.huffmanCode[i] & 0xFF);
		*address = lsb | (msb << 16);
	}
	return Status::Ok;
}

Status Jpeg::LoadDHTMem() {
	// JPEG_Set_Huff_DHTMem from stm32n6xx_hal_jpeg.c
	uint32_t value;
	uint32_t index;
	__IO uint32_t *address;

	/* DC0 Huffman Table : BITS*/
	/* DC0 BITS is a 16 Bytes table i.e 4x32bits words from DHTMEM base address to DHTMEM + 3*/
	address = (this->instance->DHTMEM + 3);
	index = 16;
	while (index > 3UL) {
		*address = (((uint32_t)stdHuffDCLumBits[index - 1UL] & 0xFFUL) << 24) |
					(((uint32_t)stdHuffDCLumBits[index - 2UL] & 0xFFUL) << 16) |
					(((uint32_t)stdHuffDCLumBits[index - 3UL] & 0xFFUL) << 8) |
					((uint32_t)stdHuffDCLumBits[index - 4UL] & 0xFFUL);
		address--;
		index -= 4UL;
	}
	/* DC0 Huffman Table : Val*/
	/* DC0 VALS is a 12 Bytes table i.e 3x32bits words from DHTMEM base address +4 to DHTMEM + 6 */
	address = (this->instance->DHTMEM + 6);
	index = 12;
	while (index > 3UL){
		*address = (((uint32_t)stdHuffDCLumVal[index - 1UL] & 0xFFUL) << 24) |
					(((uint32_t)stdHuffDCLumVal[index - 2UL] & 0xFFUL) << 16) |
					(((uint32_t)stdHuffDCLumVal[index - 3UL] & 0xFFUL) << 8) |
					((uint32_t)stdHuffDCLumVal[index - 4UL] & 0xFFUL);
		address--;
		index -= 4UL;
	}

	/* AC0 Huffman Table : BITS*/
	/* AC0 BITS is a 16 Bytes table i.e 4x32bits words from DHTMEM base address + 7 to DHTMEM + 10*/
	address = (this->instance->DHTMEM + 10UL);
	index = 16;
	while (index > 3UL){
		*address = (((uint32_t)stdHuffACLumBits[index - 1UL] & 0xFFUL) << 24) |
					(((uint32_t)stdHuffACLumBits[index - 2UL] & 0xFFUL) << 16) |
					(((uint32_t)stdHuffACLumBits[index - 3UL] & 0xFFUL) << 8) |
					((uint32_t)stdHuffACLumBits[index - 4UL] & 0xFFUL);
		address--;
		index -= 4UL;
	}
	/* AC0 Huffman Table : Val*/
	/* AC0 VALS is a 162 Bytes table i.e 41x32bits words from DHTMEM base address + 11 to DHTMEM + 51 */
	/* only Byte 0 and Byte 1 of the last word (@ DHTMEM + 51) belong to AC0 VALS table */
	address = (this->instance->DHTMEM + 51);
	value = *address & 0xFFFF0000U;
	value = value | (((uint32_t)stdHuffACLumVal[161] & 0xFFUL) << 8) |
			((uint32_t)stdHuffACLumVal[160] & 0xFFUL);
	*address = value;

	/*continue setting 160 AC0 huffman values */
	address--; /* address = this->instance->DHTMEM + 50*/
	index = huffACTableSize - 2UL;
	while (index > 3UL) {
		*address = (((uint32_t)stdHuffACLumVal[index - 1UL] & 0xFFUL) << 24) |
					(((uint32_t)stdHuffACLumVal[index - 2UL] & 0xFFUL) << 16) |
					(((uint32_t)stdHuffACLumVal[index - 3UL] & 0xFFUL) << 8) |
					((uint32_t)stdHuffACLumVal[index - 4UL] & 0xFFUL);
		address--;
		index -= 4UL;
	}

	/* DC1 Huffman Table : BITS*/
	/* DC1 BITS is a 16 Bytes table i.e 4x32bits words from DHTMEM + 51 base address to DHTMEM + 55*/
	/* only Byte 2 and Byte 3 of the first word (@ DHTMEM + 51) belong to DC1 Bits table */
	address = (this->instance->DHTMEM + 51);
	value = *address & 0x0000FFFFU;
	value = value | (((uint32_t)stdHuffDCChromBits[1] & 0xFFUL) << 24) |
			(((uint32_t)stdHuffDCChromBits[0] & 0xFFUL) << 16);
	*address = value;

	/* only Byte 0 and Byte 1 of the last word (@ DHTMEM + 55) belong to DC1 Bits table */
	address = (this->instance->DHTMEM + 55);
	value = *address & 0xFFFF0000U;
	value = value | (((uint32_t)stdHuffDCChromBits[15] & 0xFFUL) << 8) |
			((uint32_t)stdHuffDCChromBits[14] & 0xFFUL);
	*address = value;

	/*continue setting 12 DC1 huffman Bits from DHTMEM + 54 down to DHTMEM + 52*/
	address--;
	index = 12;
	while (index > 3UL) {
		*address = (((uint32_t)stdHuffDCChromBits[index + 1UL] & 0xFFUL) << 24) |
					(((uint32_t)stdHuffDCChromBits[index] & 0xFFUL) << 16) |
					(((uint32_t)stdHuffDCChromBits[index - 1UL] & 0xFFUL) << 8) |
					((uint32_t)stdHuffDCChromBits[index - 2UL] & 0xFFUL);
		address--;
		index -= 4UL;
	}
	/* DC1 Huffman Table : Val*/
	/* DC1 VALS is a 12 Bytes table i.e 3x32bits words from DHTMEM base address +55 to DHTMEM + 58 */
	/* only Byte 2 and Byte 3 of the first word (@ DHTMEM + 55) belong to DC1 Val table */
	address = (this->instance->DHTMEM + 55);
	value = *address & 0x0000FFFFUL;
	value = value | (((uint32_t)stdHuffDCChromVal[1] & 0xFFUL) << 24) |
			(((uint32_t)stdHuffDCChromVal[0] & 0xFFUL) << 16);
	*address = value;

	/* only Byte 0 and Byte 1 of the last word (@ DHTMEM + 58) belong to DC1 Val table */
	address = (this->instance->DHTMEM + 58);
	value = *address & 0xFFFF0000UL;
	value = value | (((uint32_t)stdHuffDCChromVal[11] & 0xFFUL) << 8) |
			((uint32_t)stdHuffDCChromVal[10] & 0xFFUL);
	*address = value;

	/*continue setting 8 DC1 huffman val from DHTMEM + 57 down to DHTMEM + 56*/
	address--;
	index = 8;
	while (index > 3UL) {
		*address = (((uint32_t)stdHuffDCChromVal[index + 1UL] & 0xFFUL) << 24) |
					(((uint32_t)stdHuffDCChromVal[index] & 0xFFUL) << 16) |
					(((uint32_t)stdHuffDCChromVal[index - 1UL] & 0xFFUL) << 8) |
					((uint32_t)stdHuffDCChromVal[index - 2UL] & 0xFFUL);
		address--;
		index -= 4UL;
	}

	/* AC1 Huffman Table : BITS*/
	/* AC1 BITS is a 16 Bytes table i.e 4x32bits words from DHTMEM base address + 58 to DHTMEM + 62*/
	/* only Byte 2 and Byte 3 of the first word (@ DHTMEM + 58) belong to AC1 Bits table */
	address = (this->instance->DHTMEM + 58);
	value = *address & 0x0000FFFFU;
	value = value | (((uint32_t)stdHuffACChromBits[1] & 0xFFUL) << 24) |
			(((uint32_t)stdHuffACChromBits[0] & 0xFFUL) << 16);
	*address = value;

	/* only Byte 0 and Byte 1 of the last word (@ DHTMEM + 62) belong to Bits Val table */
	address = (this->instance->DHTMEM + 62);
	value = *address & 0xFFFF0000U;
	value = value | (((uint32_t)stdHuffACChromBits[15] & 0xFFUL) << 8) | ((uint32_t)stdHuffACChromBits[14] & 0xFFUL);
	*address = value;

	/*continue setting 12 AC1 huffman Bits from DHTMEM + 61 down to DHTMEM + 59*/
	address--;
	index = 12;
	while (index > 3UL) {
		*address = (((uint32_t)stdHuffACChromBits[index + 1UL] & 0xFFUL) << 24) |
					(((uint32_t)stdHuffACChromBits[index] & 0xFFUL) << 16) |
					(((uint32_t)stdHuffACChromBits[index - 1UL] & 0xFFUL) << 8) |
					((uint32_t)stdHuffACChromBits[index - 2UL] & 0xFFUL);
		address--;
		index -= 4UL;
	}
	/* AC1 Huffman Table : Val*/
	/* AC1 VALS is a 162 Bytes table i.e 41x32bits words from DHTMEM base address + 62 to DHTMEM + 102 */
	/* only Byte 2 and Byte 3 of the first word (@ DHTMEM + 62) belong to AC1 VALS table */
	address = (this->instance->DHTMEM + 62);
	value = *address & 0x0000FFFFUL;
	value = value | (((uint32_t)stdHuffACChromVal[1] & 0xFFUL) << 24) |
			(((uint32_t)stdHuffACChromVal[0] & 0xFFUL) << 16);
	*address = value;

	/*continue setting 160 AC1 huffman values from DHTMEM + 63 to DHTMEM+102 */
	address = (this->instance->DHTMEM + 102);
	index = huffACTableSize - 2UL;
	while (index > 3UL) {
		*address = (((uint32_t)stdHuffACChromVal[index + 1UL] & 0xFFUL) << 24) |
					(((uint32_t)stdHuffACChromVal[index] & 0xFFUL) << 16) |
					(((uint32_t)stdHuffACChromVal[index - 1UL] & 0xFFUL) << 8) |
					((uint32_t)stdHuffACChromVal[index - 2UL] & 0xFFUL);
		address--;
		index -= 4UL;
	}

	return Status::Ok;
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------
void Jpeg::InterruptHandler() {
	// Handle End of Conversion
	if(READ_BIT(this->instance->SR, JPEG_SR_HPDF) == JPEG_SR_HPDF) {
		// Clear flag
		MODIFY_REG(this->instance->CFR, JPEG_CFR_CHPDF, JPEG_CFR_CHPDF);
	}

	// Handle End of Conversion
	if(READ_BIT(this->instance->SR, JPEG_SR_EOCF) == JPEG_SR_EOCF) {
		// Required to not ingest more and start next convertion (e.g. would add new data header if enabled)
		this->Stop();

		if(this->config.EventCallback != nullptr) {
			this->config.EventCallback(this->config.callbackContext, Event::EncodeComplete);
		}
		
		// Clear flag
		MODIFY_REG(this->instance->CFR, JPEG_CFR_CEOCF, JPEG_CFR_CEOCF);
	}
}