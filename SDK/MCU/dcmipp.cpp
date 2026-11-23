/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	SDK/MCU/dcmipp.cpp
 */

#include "dcmipp.hpp"

Dcmipp::Dcmipp(DCMIPP_TypeDef *instance) : instance(instance) {
	this->isInitialized = false;
	this->irqPriority = 0x0E; // Lowest priority (safe default)
}

Status Dcmipp::Init() {
	if(this->isInitialized == true) {
		return Status::Ok;
	}

	// Create RTOS objects
	if(tx_event_flags_create(&this->event, const_cast<char*>("dcmipp event")) != TX_SUCCESS) {
		return Status::Error;
	}

	// Enable bus clocks
	if(this->instance == DCMIPP) {
		LL_APB5_GRP1_EnableClock(LL_APB5_GRP1_PERIPH_DCMIPP);
		this->irqCall = DCMIPP_IRQn;
	}

	// Configure RIF (enable IDMA secure region access, etc...)
	this->ConfigureRIF();

	// Configure the AXI IP-Plug to allow the DCMIPP to write to memory
	// Set IP-Plug to IDLE, to allow reconfiguration
	SET_BIT(this->instance->IPGR2, DCMIPP_IPGR2_PSTART);
	uint32_t timeout = 10000;
	while ((this->instance->IPGR3 & DCMIPP_IPGR3_IDLE) != DCMIPP_IPGR3_IDLE) {
		timeout = timeout - 1;
		if(timeout == 0) {
			return Status::Timeout;
		}
	}

	// Set Memory Page Size (Default 2: 256 bytes page boundary): 2^VAL * 64-bytes
	WRITE_REG(this->instance->IPGR1, 0x02);

	// Bandwidth allocation is set by:
	// Max outstanding as shared of combined 16 (IPCxR1.OTR)
	// Bandwith ratio as shared of combined SUM of all IPCxR2.WLRU values (IPCxR2.WLRU)

	// Available total FIFO: 640x64 bit (5120 bytes), i.e. 640 lines of 64-bit words
	// Split evenly across all 5 clients: 5 * 128 = 640 words
	uint32_t fifoSize = 128;
	uint32_t fifoAddStart = 0;
	uint32_t fifoAddrEnd = (fifoSize - 1);
	// INFO:
	// With otrValue = 2, trafficValue = 2 DOES NOT WORK! (frame dropped like crazy)
	// With otrValue = 2, trafficValue = 3 Slower i.e. FPS drops from ~18fps to ~16fps
	uint32_t trafficValue = 4;	// Burst size (2^x of 8-byte units i.e. 0->8, 1->16, 2->32, etc...): 4 i.e 128 bytes/burst (recommended in Reference Manual)
	uint32_t otrValue[5] = {2, 4, 3, 3, 4};	// Maximum outstanding transaction per Pipe. OTR budget must sum to <= 16 across all 5 clients: {2, 4, 3, 3, 4} = 16

	// IP-PLUG Client 1 (Pipe 0, Dump/Snapshot) Configuration
	WRITE_REG(this->instance->IPC1R1, (otrValue[0] << DCMIPP_IPC1R1_OTR_Pos) | (trafficValue << DCMIPP_IPC1R1_TRAFFIC_Pos));
	WRITE_REG(this->instance->IPC1R2, (0U << DCMIPP_IPC1R2_WLRU_Pos));
	WRITE_REG(this->instance->IPC1R3, (fifoAddStart << DCMIPP_IPC1R3_DPREGSTART_Pos) | (fifoAddrEnd << DCMIPP_IPC1R3_DPREGEND_Pos));
	fifoAddStart = fifoAddrEnd + 1;
	fifoAddrEnd += fifoSize;

	// IP-PLUG Client 2 (Pipe 1, Y/RGB component) Configuration
	WRITE_REG(this->instance->IPC2R1, (otrValue[1] << DCMIPP_IPC1R1_OTR_Pos) | (trafficValue << DCMIPP_IPC2R1_TRAFFIC_Pos));
	// Set ratio of total bandwith, arbitration between clients (default: 1 / (SUM of parts))
	WRITE_REG(this->instance->IPC2R2, (0U << DCMIPP_IPC1R2_WLRU_Pos));
	// Set FIFO End/Start address (64-bit word address)
	WRITE_REG(this->instance->IPC2R3, (fifoAddStart << DCMIPP_IPC1R3_DPREGSTART_Pos) | (fifoAddrEnd << DCMIPP_IPC1R3_DPREGEND_Pos));
	fifoAddStart = fifoAddrEnd + 1;
	fifoAddrEnd += fifoSize;

	// IP-PLUG Client 3 (Pipe 1, U component) Configuration
	WRITE_REG(this->instance->IPC3R1, (otrValue[2] << DCMIPP_IPC3R1_OTR_Pos) | (trafficValue << DCMIPP_IPC3R1_TRAFFIC_Pos));
	WRITE_REG(this->instance->IPC3R2, (0U << DCMIPP_IPC1R2_WLRU_Pos));
	WRITE_REG(this->instance->IPC3R3, (fifoAddStart << DCMIPP_IPC1R3_DPREGSTART_Pos) | (fifoAddrEnd << DCMIPP_IPC1R3_DPREGEND_Pos));
	fifoAddStart = fifoAddrEnd + 1;
	fifoAddrEnd += fifoSize;

	// IP-PLUG Client 4 (Pipe 1, V component) Configuration
	WRITE_REG(this->instance->IPC4R1, (otrValue[3] << DCMIPP_IPC4R1_OTR_Pos) | (trafficValue << DCMIPP_IPC4R1_TRAFFIC_Pos));
	WRITE_REG(this->instance->IPC4R2, (0U << DCMIPP_IPC1R2_WLRU_Pos));
	WRITE_REG(this->instance->IPC4R3, (fifoAddStart << DCMIPP_IPC1R3_DPREGSTART_Pos) | (fifoAddrEnd << DCMIPP_IPC1R3_DPREGEND_Pos));
	fifoAddStart = fifoAddrEnd + 1;
	fifoAddrEnd += fifoSize;

	// IP-PLUG Client 5 (Pipe 2, Auxiliary/NPU)
	WRITE_REG(this->instance->IPC5R1, (otrValue[4] << DCMIPP_IPC5R1_OTR_Pos) | (trafficValue << DCMIPP_IPC5R1_TRAFFIC_Pos));
	WRITE_REG(this->instance->IPC5R2, (0U << DCMIPP_IPC5R2_WLRU_Pos));
	WRITE_REG(this->instance->IPC5R3, (fifoAddStart << DCMIPP_IPC5R3_DPREGSTART_Pos) | (fifoAddrEnd << DCMIPP_IPC5R3_DPREGEND_Pos));

	// Pipe 2 filtering: Set independent of pipe 1
	SET_BIT(this->instance->P1FSCR, DCMIPP_P1FSCR_PIPEDIFF);

	// Disable Parallel interface
	CLEAR_BIT(this->instance->PRCR, DCMIPP_PRCR_ENABLE);

	// Set DCMIPP Input Selection: Input from CSI
	SET_BIT(this->instance->CMCR, DCMIPP_CMCR_INSEL);

	// Set DCMIPP Frame Counter: Pipe 1 (main)
	MODIFY_REG(this->instance->CMCR, DCMIPP_CMCR_PSFC_Msk, (0x01 << DCMIPP_CMCR_PSFC_Pos));
	
	// Clear frame counter
	SET_BIT(this->instance->CMCR, DCMIPP_CMCR_CFC);

	// Release lock, IP-Plug runs on demand by background HW
	CLEAR_BIT(this->instance->IPGR2, DCMIPP_IPGR2_PSTART);

	// Configure Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);
	MODIFY_REG(this->instance->CMIER, 0x00, DCMIPP_CMIER_ATXERRIE);		// Enable IPPLUG AXI Transfer error interrupt

	isInitialized = true;
	return Status::Ok;
}

Status Dcmipp::LinkCSIVirtualChannel(PipeID pipe, Csi::VirtualChannel vc, Csi::MIPIDataType dataType) {
	uint32_t dtVal = static_cast<uint32_t>(dataType);
	uint32_t vcVal = static_cast<uint32_t>(vc);

	if(pipe == PipeID::Dump) {
		// Set Virtual Channel
		MODIFY_REG(this->instance->P0FSCR, DCMIPP_P0FSCR_VC_Msk, (vcVal << DCMIPP_P0FSCR_VC_Pos));

		// Filter specific Data Type Mode: Only data matching DTIDA
		// 0x0: Only flow DTIDA from the selected virtual channel is forwarded in the pipe
		// 0x1: Flows DTIDA and/or DTIDB from the selected virtual channel are forwarded in the pipe
		// 0x2: All data types from the selected virtual channel (except the DTIDA or DTIDB) are forwarded in the pipe
		// 0x3: All data types of the selected virtual channel VC are forwarded in the pipe
		MODIFY_REG(this->instance->P0FSCR, DCMIPP_P0FSCR_DTMODE_Msk, (0x00 << DCMIPP_P0FSCR_DTMODE_Pos));

		// Data Type Selection IDA
		MODIFY_REG(this->instance->P0FSCR, DCMIPP_P0FSCR_DTIDA_Msk, (dtVal << DCMIPP_P0FSCR_DTIDA_Pos));
	}
	else if(pipe == PipeID::Main) {
		// Set Virtual Channel
		MODIFY_REG(this->instance->P1FSCR, DCMIPP_P1FSCR_VC_Msk, (vcVal << DCMIPP_P1FSCR_VC_Pos));

		// Filter specific Data Type Mode: Only data matching DTIDA
		// 0x0: Only flow DTIDA from the selected virtual channel is forwarded in the pipe
		// 0x1: Flows DTIDA and/or DTIDB from the selected virtual channel are forwarded in the pipe
		MODIFY_REG(this->instance->P1FSCR, DCMIPP_P1FSCR_DTMODE_Msk, (0x00 << DCMIPP_P1FSCR_DTMODE_Pos));

		// Pipe 2 filtering: Set independent of pipe 1
		// MODIFY_REG(this->instance->P1FSCR, DCMIPP_P1FSCR_PIPEDIFF, DCMIPP_P1FSCR_PIPEDIFF);
		
		// Data Type Selection IDA
		MODIFY_REG(this->instance->P1FSCR, DCMIPP_P1FSCR_DTIDA_Msk, (dtVal << DCMIPP_P1FSCR_DTIDA_Pos));		
	} 
	else if(pipe == PipeID::Auxiliary) {
		// Pipe 2 filtering: Set independent of pipe 1
		// MODIFY_REG(this->instance->P1FSCR, DCMIPP_P1FSCR_PIPEDIFF, DCMIPP_P1FSCR_PIPEDIFF);

		// Those bit fields are meaningful when PIPEDIFF = 1: Pipe1, Pipe2 is fully independent
		if((this->instance->P1FSCR & DCMIPP_P1FSCR_PIPEDIFF) == DCMIPP_P1FSCR_PIPEDIFF) {
			// Set Virtual Channel
			MODIFY_REG(this->instance->P2FSCR, DCMIPP_P2FSCR_VC, (vcVal << DCMIPP_P2FSCR_VC_Pos));

			// Data Type Selection IDA
			MODIFY_REG(this->instance->P2FSCR, DCMIPP_P2FSCR_DTIDA_Msk, (dtVal << DCMIPP_P2FSCR_DTIDA_Pos));
		}
	}
	else {
		return Status::Error;
	}
	return Status::Ok;
}

Status Dcmipp::ConfigurePipe(PipeID pipe, const PipeConfig &config) {
	if(pipe == PipeID::Dump) {
		// Configure Frame Rate Decimation
		uint32_t fRate = static_cast<uint32_t>(config.frameRate);
		MODIFY_REG(this->instance->P0FCTCR, DCMIPP_P0FCTCR_FRATE_Msk, (fRate << DCMIPP_P0FCTCR_FRATE_Pos));

		// Configure Pixel Packer Format. Pipe 0 only supports swapping
		if(config.swapRBUV == true) {
			// Swaps the bytes from provided words, byte 0-vs.-1 and 2-vs.-3: This is, swap R-vs-B or U-vs-V
			MODIFY_REG(this->instance->P0PPCR, DCMIPP_P0PPCR_SWAPYUV_Msk, DCMIPP_P0PPCR_SWAPYUV);
		}
		else {
			MODIFY_REG(this->instance->P0PPCR, DCMIPP_P0PPCR_SWAPYUV_Msk, 0x00);
		}
	}
	else if(pipe == PipeID::Main) {
		// Configure Frame Rate Decimation
		uint32_t fRate = static_cast<uint32_t>(config.frameRate);
		MODIFY_REG(this->instance->P1FCTCR, DCMIPP_P1FCTCR_FRATE_Msk, (fRate << DCMIPP_P1FCTCR_FRATE_Pos));

		// Configure Pixel Packer Format
		MODIFY_REG(this->instance->P1PPCR, DCMIPP_P1PPCR_FORMAT_Msk, (static_cast<uint32_t>(config.format) << DCMIPP_P1PPCR_FORMAT_Pos));
		if(config.swapRBUV == true) {
			// Swap R-vs-B or U-vs-V
			MODIFY_REG(this->instance->P1PPCR, DCMIPP_P1PPCR_SWAPRB_Msk, DCMIPP_P1PPCR_SWAPRB);
		}
		else {
			MODIFY_REG(this->instance->P1PPCR, DCMIPP_P1PPCR_SWAPRB_Msk, 0x00);
		}

		// Configure Pixel Pipe Pitch (Memory Stride)
		MODIFY_REG(this->instance->P1PPM0PR, DCMIPP_P1PPM0PR_PITCH_Msk, (config.pixelPitch << DCMIPP_P1PPM0PR_PITCH_Pos));

		// If using YUV modes that require UV pitch configuration
		if(config.format == OutputFormat::YUV422 || config.format == OutputFormat::YUV420_NV21) {
			MODIFY_REG(this->instance->P1PPM1PR, DCMIPP_P1PPM1PR_PITCH_Msk, (config.pixelPitch << DCMIPP_P1PPM1PR_PITCH_Pos));
		}
		else if(config.format == OutputFormat::YUV420_YV12) {
			MODIFY_REG(this->instance->P1PPM1PR, DCMIPP_P1PPM1PR_PITCH_Msk, ((config.pixelPitch / 2) << DCMIPP_P1PPM1PR_PITCH_Pos));
		}
	} 
	else if(pipe == PipeID::Auxiliary) {
		// Pipe 2 only supports 1-buffer packed formats (<= YUV422_YUYV)
		if(static_cast<uint8_t>(config.format) > static_cast<uint8_t>(OutputFormat::YUV422_YUYV)) {
			return Status::Error;
		}
		
		// Configure Frame Rate Decimation
		uint32_t fRate = static_cast<uint32_t>(config.frameRate);
		MODIFY_REG(this->instance->P2FCTCR, DCMIPP_P2FCTCR_FRATE_Msk, (fRate << DCMIPP_P2FCTCR_FRATE_Pos));

		// Configure Pixel Packer Format
		MODIFY_REG(this->instance->P2PPCR, DCMIPP_P2PPCR_FORMAT_Msk, (static_cast<uint32_t>(config.format) << DCMIPP_P2PPCR_FORMAT_Pos));
		if(config.swapRBUV == true) {
			// Swap R-vs-B or U-vs-V
			MODIFY_REG(this->instance->P2PPCR, DCMIPP_P2PPCR_SWAPRB_Msk, DCMIPP_P2PPCR_SWAPRB);
		}
		else {
			MODIFY_REG(this->instance->P2PPCR, DCMIPP_P2PPCR_SWAPRB_Msk, 0x00);
		}

		// Configure Pixel Pipe Pitch (Memory Stride)
		MODIFY_REG(this->instance->P2PPM0PR, DCMIPP_P2PPM0PR_PITCH_Msk, (config.pixelPitch << DCMIPP_P2PPM0PR_PITCH_Pos));
	}
	else {
		return Status::Error;
	}

	return Status::Ok;
}

Status Dcmipp::ConfigureWindow(PipeID pipe, const WindowConfig &config) {
	if(config.srcWidth < 2U || config.srcHeight < 2U) {
		return Status::Error;
	}

	// Configure or disable Crop
	bool enableCrop = (config.cropWidth > 0U && config.cropHeight > 0U);
	if(enableCrop == true && ((config.cropX + config.cropWidth) > config.srcWidth || (config.cropY + config.cropHeight) > config.srcHeight)) {
		return Status::Error;
	}

	uint16_t inWidth = enableCrop ? config.cropWidth : config.srcWidth;
	uint16_t inHeight = enableCrop ? config.cropHeight : config.srcHeight;
	uint16_t outWidth = (config.outWidth > 0U) ? config.outWidth : inWidth;
	uint16_t outHeight = (config.outHeight > 0U) ? config.outHeight : inHeight;
	if(outWidth < 2U || outHeight < 2U || outWidth > inWidth || outHeight > inHeight) {
		return Status::Error;
	}

	if(pipe == PipeID::Dump) {
		// Pipe 0 only supports cropping, not arbitrary downscaling
		if(outWidth != inWidth || outHeight != inHeight) {
			return Status::Error;
		}
		if(enableCrop == true) {
			MODIFY_REG(this->instance->P0SCSTR, DCMIPP_P0SCSTR_HSTART_Msk | DCMIPP_P0SCSTR_VSTART_Msk, (static_cast<uint32_t>(config.cropX) << DCMIPP_P0SCSTR_HSTART_Pos) | (static_cast<uint32_t>(config.cropY) << DCMIPP_P0SCSTR_VSTART_Pos));
			MODIFY_REG(this->instance->P0SCSZR, DCMIPP_P0SCSZR_HSIZE_Msk | DCMIPP_P0SCSZR_VSIZE_Msk | DCMIPP_P0SCSZR_POSNEG_Msk, (static_cast<uint32_t>(config.cropWidth) << DCMIPP_P0SCSZR_HSIZE_Pos) | (static_cast<uint32_t>(config.cropHeight) << DCMIPP_P0SCSZR_VSIZE_Pos));
			SET_BIT(this->instance->P0SCSZR, DCMIPP_P0SCSZR_ENABLE);
		}
		else {
			CLEAR_BIT(this->instance->P0SCSZR, DCMIPP_P0SCSZR_ENABLE);
		}
		return Status::Ok;
	}

	// Apply Crop for Pipe 1 or Pipe 2
	volatile uint32_t *crstr = (pipe == PipeID::Main) ? &this->instance->P1CRSTR : &this->instance->P2CRSTR;
	volatile uint32_t *crszr = (pipe == PipeID::Main) ? &this->instance->P1CRSZR : &this->instance->P2CRSZR;
	volatile uint32_t *dccr = (pipe == PipeID::Main) ? &this->instance->P1DCCR : &this->instance->P2DCCR;
	volatile uint32_t *dscr = (pipe == PipeID::Main) ? &this->instance->P1DSCR : &this->instance->P2DSCR;
	volatile uint32_t *dsrt = (pipe == PipeID::Main) ? &this->instance->P1DSRTIOR : &this->instance->P2DSRTIOR;
	volatile uint32_t *dsszr = (pipe == PipeID::Main) ? &this->instance->P1DSSZR : &this->instance->P2DSSZR;

	if(enableCrop == true) {
		MODIFY_REG(*crstr, DCMIPP_P1CRSTR_HSTART_Msk | DCMIPP_P1CRSTR_VSTART_Msk, (static_cast<uint32_t>(config.cropX) << DCMIPP_P1CRSTR_HSTART_Pos) | (static_cast<uint32_t>(config.cropY) << DCMIPP_P1CRSTR_VSTART_Pos));
		MODIFY_REG(*crszr, DCMIPP_P1CRSZR_HSIZE_Msk | DCMIPP_P1CRSZR_VSIZE_Msk, (static_cast<uint32_t>(config.cropWidth) << DCMIPP_P1CRSZR_HSIZE_Pos) | (static_cast<uint32_t>(config.cropHeight) << DCMIPP_P1CRSZR_VSIZE_Pos));
		SET_BIT(*crszr, DCMIPP_P1CRSZR_ENABLE);
	}
	else {
		CLEAR_BIT(*crszr, DCMIPP_P1CRSZR_ENABLE);
	}

	 // Auto-select power-of-2 Decimation (1x, 2x, 4x, 8x) so Downsize block stays within its 8x limit
	uint32_t hDec = 0U;
	uint16_t postDecW = inWidth;
	while((((static_cast<uint32_t>(postDecW) - 1U) * 8192U) / (outWidth - 1U)) > 65535U && hDec < 3U) {
		postDecW /= 2U;
		hDec++;
	}

	uint32_t vDec = 0U;
	uint16_t postDecH = inHeight;
	while((((static_cast<uint32_t>(postDecH) - 1U) * 8192U) / (outHeight - 1U)) > 65535U && vDec < 3U) {
		postDecH /= 2U;
		vDec++;
	}

	if(hDec > 0U || vDec > 0U) {
		MODIFY_REG(*dccr, DCMIPP_P1DCCR_HDEC_Msk | DCMIPP_P1DCCR_VDEC_Msk, (hDec << DCMIPP_P1DCCR_HDEC_Pos) | (vDec << DCMIPP_P1DCCR_VDEC_Pos));
		SET_BIT(*dccr, DCMIPP_P1DCCR_ENABLE);
	}
	else {
		CLEAR_BIT(*dccr, DCMIPP_P1DCCR_ENABLE);
	}

	// Auto-configure Downsize block for the remaining scale factor
	if(postDecW == outWidth && postDecH == outHeight) {
		CLEAR_BIT(*dscr, DCMIPP_P1DSCR_ENABLE);
		return Status::Ok;
	}

	uint32_t hRatio = ((static_cast<uint32_t>(postDecW) - 1U) * 8192U) / (outWidth - 1U);
	uint32_t vRatio = ((static_cast<uint32_t>(postDecH) - 1U) * 8192U) / (outHeight - 1U);
	if(hRatio > 65535U || vRatio > 65535U || hRatio < 8192U || vRatio < 8192U) {
		// Exceeds 64x combined max hardware downscale
		return Status::Error;
	}

	uint32_t hDiv = (1024U * 8192U) / hRatio;
	uint32_t vDiv = (1024U * 8192U) / vRatio;
	if(hDiv > 1023U){
		hDiv = 1023U;
	}
	if(vDiv > 1023U) {
		vDiv = 1023U;
	}

	MODIFY_REG(*dscr, DCMIPP_P1DSCR_HDIV_Msk | DCMIPP_P1DSCR_VDIV_Msk, (hDiv << DCMIPP_P1DSCR_HDIV_Pos) | (vDiv << DCMIPP_P1DSCR_VDIV_Pos));
	WRITE_REG(*dsrt, (hRatio << DCMIPP_P1DSRTIOR_HRATIO_Pos) | (vRatio << DCMIPP_P1DSRTIOR_VRATIO_Pos));
	MODIFY_REG(*dsszr, DCMIPP_P1DSSZR_HSIZE_Msk | DCMIPP_P1DSSZR_VSIZE_Msk, (static_cast<uint32_t>(outWidth) << DCMIPP_P1DSSZR_HSIZE_Pos) | (static_cast<uint32_t>(outHeight) << DCMIPP_P1DSSZR_VSIZE_Pos));
	SET_BIT(*dscr, DCMIPP_P1DSCR_ENABLE);

	return Status::Ok;
}

Status Dcmipp::EnableLineWrapping(PipeID pipe, LineCount addressWrap, LineCount lineMult) {
	if(pipe == PipeID::Main || pipe == PipeID::Auxiliary) {
		// Set Line Mult Address Wrapping Modulo (LMAWM) and Line Multiplier (LINEMULT)
		
		// #define DCMIPP_MULTILINE_1_LINE		0UL										// Event after every 1 line
		// #define DCMIPP_MULTILINE_2_LINES		(1UL << DCMIPP_P0PPCR_LINEMULT_Pos)		// Event after every 2 lines
		// #define DCMIPP_MULTILINE_4_LINES		(2UL << DCMIPP_P0PPCR_LINEMULT_Pos)		// Event after every 4 lines
		// #define DCMIPP_MULTILINE_8_LINES		(3UL << DCMIPP_P0PPCR_LINEMULT_Pos)		// Event after every 8 lines
		// #define DCMIPP_MULTILINE_16_LINES	(4UL << DCMIPP_P0PPCR_LINEMULT_Pos)		// Event after every 16 lines
		// #define DCMIPP_MULTILINE_32_LINES	(5UL << DCMIPP_P0PPCR_LINEMULT_Pos)		// Event after every 32 lines
		// #define DCMIPP_MULTILINE_64_LINES	(6UL << DCMIPP_P0PPCR_LINEMULT_Pos)		// Event after every 64 lines
		// #define DCMIPP_MULTILINE_128_LINES	(7UL << DCMIPP_P0PPCR_LINEMULT_Pos)		// Event after every 128 lines

		// #define DCMIPP_WRAP_ADDRESS_1_LINE		(0UL << DCMIPP_P1PPCR_LMAWM_Pos)	// Wraps address after 1 line
		// #define DCMIPP_WRAP_ADDRESS_2_LINES		(1UL << DCMIPP_P1PPCR_LMAWM_Pos)	// Wraps address after 2 lines
		// #define DCMIPP_WRAP_ADDRESS_4_LINES		(2UL << DCMIPP_P1PPCR_LMAWM_Pos)	// Wraps address after 4 lines
		// #define DCMIPP_WRAP_ADDRESS_8_LINES		(3UL << DCMIPP_P1PPCR_LMAWM_Pos)	// Wraps address after 8 lines
		// #define DCMIPP_WRAP_ADDRESS_16_LINES		(4UL << DCMIPP_P1PPCR_LMAWM_Pos)	// Wraps address after 16 lines
		// #define DCMIPP_WRAP_ADDRESS_32_LINES		(5UL << DCMIPP_P1PPCR_LMAWM_Pos)	// Wraps address after 32 lines
		// #define DCMIPP_WRAP_ADDRESS_64_LINES		(6UL << DCMIPP_P1PPCR_LMAWM_Pos)	// Wraps address after 64 lines
		// #define DCMIPP_WRAP_ADDRESS_128_LINES	(7UL << DCMIPP_P1PPCR_LMAWM_Pos)	// Wraps address after 128 lines

		volatile uint32_t *ppcr = (pipe == PipeID::Main) ? &this->instance->P1PPCR : &this->instance->P2PPCR;
		MODIFY_REG(*ppcr, DCMIPP_P1PPCR_LMAWM_Msk, (static_cast<uint32_t>(addressWrap) << DCMIPP_P1PPCR_LMAWM_Pos));
		MODIFY_REG(*ppcr, DCMIPP_P1PPCR_LINEMULT_Msk, (static_cast<uint32_t>(lineMult) << DCMIPP_P1PPCR_LINEMULT_Pos));
		
		// Enable Line Mult Address Wrapping (LMAWE)
		SET_BIT(*ppcr, DCMIPP_P1PPCR_LMAWE);
	}
	else {
		return Status::Error;
	}
	return Status::Ok;
}

Status Dcmipp::DisableLineWrapping(PipeID pipe) {
	if(pipe == PipeID::Main || pipe == PipeID::Auxiliary) {
		volatile uint32_t *ppcr = (pipe == PipeID::Main) ? &this->instance->P1PPCR : &this->instance->P2PPCR;
		CLEAR_BIT(*ppcr, DCMIPP_P1PPCR_LMAWE);
		// Reset line multiplier to 128 lines to prevent spurious interrupts
		MODIFY_REG(*ppcr, DCMIPP_P1PPCR_LINEMULT_Msk, (static_cast<uint32_t>(LineCount::Lines128) << DCMIPP_P1PPCR_LINEMULT_Pos));
		return Status::Ok;
	}
	return Status::Error;
}

Status Dcmipp::CaptureAsync(PipeID pipe, const MemoryDestination &dest, CaptureMode mode) {
	// Validate 16-byte AXI IDMA alignment
	if(dest.primaryAddress == 0U || (dest.primaryAddress & 0x0FU) != 0U) {
		return Status::Error;
	}
	if(dest.isDoubleBuffered && (dest.secondaryAddress == 0U || (dest.secondaryAddress & 0x0FU) != 0U)) {
		return Status::Error;
	}
	if(dest.isSemiPlanar == true || dest.isFullPlanar == true) {
		if(pipe != PipeID::Main) {
			// Only Pipe 1 supports planar/semi-planar buffers
			return Status::Error; 
		}
		if(dest.uAddress == 0U || (dest.uAddress & 0x0FU) != 0U) {
			return Status::Error;
		}
	}
	if(dest.isFullPlanar == true) {
		if(dest.vAddress == 0U || (dest.vAddress & 0x0FU) != 0U) {
			return Status::Error;
		}
	}

	if(pipe == PipeID::Dump) {
		// Ensure pipe is not active
		if(this->instance->P0FSCR & DCMIPP_P0FSCR_PIPEN) {
			return Status::Error;
		}

		// Primary Address (Y Plane or Interleaved)
		WRITE_REG(this->instance->P0PPM0AR1, dest.primaryAddress);

		// Handle Double Buffering
		if(dest.isDoubleBuffered) {
			WRITE_REG(this->instance->P0PPM0AR2, dest.secondaryAddress);
			SET_BIT(this->instance->P0PPCR, DCMIPP_P0PPCR_DBM);
		}
		else {
			CLEAR_BIT(this->instance->P0PPCR, DCMIPP_P0PPCR_DBM);
		}

		// Clear ThreadX event flags on this pipe only
		ULONG dummy;
		uint32_t pipeMask = GetPipeEventMask(pipe, EVT_TRANS_CPLT | EVT_VSYNC | EVT_ERR);
		tx_event_flags_get(&this->event, pipeMask, TX_OR_CLEAR, &dummy, TX_NO_WAIT);

		// Set Capture Mode (Snapshot / Continuous)
		if(mode == Dcmipp::CaptureMode::Continuous) {
			MODIFY_REG(this->instance->P0FCTCR, DCMIPP_P0FCTCR_CPTMODE_Msk, 0x00);
		}
		else {
			MODIFY_REG(this->instance->P0FCTCR, DCMIPP_P0FCTCR_CPTMODE_Msk, DCMIPP_P0FCTCR_CPTMODE);
		}

		// Enable Interrupts
		MODIFY_REG(this->instance->CMIER, 0x00, DCMIPP_CMIER_P0FRAMEIE | DCMIPP_CMIER_P0VSYNCIE | DCMIPP_CMIER_P0OVRIE | DCMIPP_CMIER_ATXERRIE);

		// Enable the Pipe
		SET_BIT(this->instance->P0FSCR, DCMIPP_P0FSCR_PIPEN);

		// Request Capture Start
		SET_BIT(this->instance->P0FCTCR, DCMIPP_P0FCTCR_CPTREQ);
	}
	else if(pipe == PipeID::Main) {
		// Ensure pipe is not active
		if((this->instance->P1FSCR & DCMIPP_P1FSCR_PIPEN) == DCMIPP_P1FSCR_PIPEN) {
			return Status::Error;
		}

		// Primary Address (Y Plane or Interleaved)
		WRITE_REG(this->instance->P1PPM0AR1, dest.primaryAddress);

		// Handle Double Buffering
		if(dest.isDoubleBuffered == true) {
			WRITE_REG(this->instance->P1PPM0AR2, dest.secondaryAddress);
			SET_BIT(this->instance->P1PPCR, DCMIPP_P1PPCR_DBM);
		}
		else {
			CLEAR_BIT(this->instance->P1PPCR, DCMIPP_P1PPCR_DBM);
		}

		// Handle Semi/Full Planar UV Addresses
		if(dest.isSemiPlanar == true || dest.isFullPlanar == true) {
			WRITE_REG(this->instance->P1PPM1AR1, dest.uAddress);
			if(dest.isDoubleBuffered == true) {
				WRITE_REG(this->instance->P1PPM1AR2, dest.vAddress);
			}
		}
		
		if(dest.isFullPlanar == true) {
			WRITE_REG(this->instance->P1PPM2AR1, dest.vAddress);
			// if(dest.isDoubleBuffered == true) {
			// 	WRITE_REG(this->instance->P1PPM2AR2, dest.secondaryVAddress);
			// }
		}

		// Clear ThreadX event flags on this pipe only
		ULONG dummy;
		uint32_t pipeMask = GetPipeEventMask(pipe, EVT_TRANS_CPLT | EVT_VSYNC | EVT_ERR);
		tx_event_flags_get(&this->event, pipeMask, TX_OR_CLEAR, &dummy, TX_NO_WAIT);

		// Set Capture Mode (Snapshot / Continuous)
		if(mode == Dcmipp::CaptureMode::Continuous) {
			MODIFY_REG(this->instance->P1FCTCR, DCMIPP_P1FCTCR_CPTMODE_Msk, 0x00);
		}
		else {
			MODIFY_REG(this->instance->P1FCTCR, DCMIPP_P1FCTCR_CPTMODE_Msk, DCMIPP_P1FCTCR_CPTMODE);
		}

		// Enable Interrupts
		MODIFY_REG(this->instance->CMIER, 0x00, DCMIPP_CMIER_P1FRAMEIE | DCMIPP_CMIER_P1VSYNCIE | DCMIPP_CMIER_P1OVRIE | DCMIPP_CMIER_ATXERRIE);

		// Enable the Pipe
		SET_BIT(this->instance->P1FSCR, DCMIPP_P1FSCR_PIPEN);

		// Request Capture Start
		SET_BIT(this->instance->P1FCTCR, DCMIPP_P1FCTCR_CPTREQ);
	} 
	else if(pipe == PipeID::Auxiliary) {
		// Ensure pipe is not active
		if(this->instance->P2FSCR & DCMIPP_P2FSCR_PIPEN) {
			return Status::Error;
		}

		// Primary Address (Y Plane or Interleaved)
		WRITE_REG(this->instance->P2PPM0AR1, dest.primaryAddress);

		// Handle Double Buffering
		if(dest.isDoubleBuffered) {
			WRITE_REG(this->instance->P2PPM0AR2, dest.secondaryAddress);
			SET_BIT(this->instance->P2PPCR, DCMIPP_P2PPCR_DBM);
		}
		else {
			CLEAR_BIT(this->instance->P2PPCR, DCMIPP_P2PPCR_DBM);
		}

		// Clear ThreadX event flags on this pipe only
		ULONG dummy;
		uint32_t pipeMask = GetPipeEventMask(pipe, EVT_TRANS_CPLT | EVT_VSYNC | EVT_ERR);
		tx_event_flags_get(&this->event, pipeMask, TX_OR_CLEAR, &dummy, TX_NO_WAIT);

		// Set Capture Mode (Snapshot / Continuous)
		if(mode == Dcmipp::CaptureMode::Continuous) {
			MODIFY_REG(this->instance->P2FCTCR, DCMIPP_P2FCTCR_CPTMODE_Msk, 0x00);
		}
		else {
			MODIFY_REG(this->instance->P2FCTCR, DCMIPP_P2FCTCR_CPTMODE_Msk, DCMIPP_P2FCTCR_CPTMODE);
		}

		// Enable Interrupts
		MODIFY_REG(this->instance->CMIER, 0x00, DCMIPP_CMIER_P2FRAMEIE | DCMIPP_CMIER_P2VSYNCIE | DCMIPP_CMIER_P2OVRIE | DCMIPP_CMIER_ATXERRIE);

		// Enable the Pipe
		SET_BIT(this->instance->P2FSCR, DCMIPP_P2FSCR_PIPEN);

		// Request Capture Start
		SET_BIT(this->instance->P2FCTCR, DCMIPP_P2FCTCR_CPTREQ);
	}
	else {
		return Status::Error;
	}

	return Status::Ok;
}

Status Dcmipp::CaptureAbort(PipeID pipe) {
	uint32_t timeout = 10000;

	if(pipe == PipeID::Dump) {
		// Disable Interrupts first to stop any overrun/error IRQ storm
		MODIFY_REG(this->instance->CMIER, DCMIPP_CMIER_P0LINEIE | DCMIPP_CMIER_P0FRAMEIE | DCMIPP_CMIER_P0VSYNCIE | DCMIPP_CMIER_P0LIMITIE | DCMIPP_CMIER_P0OVRIE, 0x00);

		// Stop the capture
		CLEAR_BIT(this->instance->P0FCTCR, DCMIPP_P0FCTCR_CPTREQ);

		// Wait until capture is no longer active
		while((this->instance->CMSR1 & DCMIPP_CMSR1_P0CPTACT) != 0U) {
			timeout = timeout - 1;
			if(timeout == 0) {
				// Proceed to force-disable PIPEN below
				break;
			}
		}

		// Disable Double Buffering to reset state
		CLEAR_BIT(this->instance->P0PPCR, DCMIPP_P0PPCR_DBM);

		// Disable Pipe Processing
		CLEAR_BIT(this->instance->P0FSCR, DCMIPP_P0FSCR_PIPEN);

		// Clear any pending flags
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP0LINEF | DCMIPP_CMFCR_CP0FRAMEF | DCMIPP_CMFCR_CP0VSYNCF | DCMIPP_CMFCR_CP0LIMITF | DCMIPP_CMFCR_CP0OVRF);
	}
	else if(pipe == PipeID::Main) {
		// Disable Interrupts first to stop any overrun/error IRQ storm
		MODIFY_REG(this->instance->CMIER, DCMIPP_CMIER_P1LINEIE | DCMIPP_CMIER_P1FRAMEIE | DCMIPP_CMIER_P1VSYNCIE | DCMIPP_CMIER_P1OVRIE, 0x00);
		
		// Stop the capture
		CLEAR_BIT(this->instance->P1FCTCR, DCMIPP_P1FCTCR_CPTREQ);

		// Wait until capture is no longer active
		while((this->instance->CMSR1 & DCMIPP_CMSR1_P1CPTACT) != 0U) {
			timeout = timeout - 1;
			if(timeout == 0) {
				// Proceed to force-disable PIPEN below
				break;
			}
		}

		// Disable Double Buffering to reset state and clear line wrapping
		CLEAR_BIT(this->instance->P1PPCR, DCMIPP_P1PPCR_DBM | DCMIPP_P1PPCR_LMAWE);
		MODIFY_REG(this->instance->P1PPCR, DCMIPP_P1PPCR_LINEMULT_Msk, (static_cast<uint32_t>(LineCount::Lines128) << DCMIPP_P1PPCR_LINEMULT_Pos)); // 128 lines

		// Disable Pipe Processing
		CLEAR_BIT(this->instance->P1FSCR, DCMIPP_P1FSCR_PIPEN);

		// Clear any pending flags
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP1LINEF | DCMIPP_CMFCR_CP1FRAMEF | DCMIPP_CMFCR_CP1VSYNCF | DCMIPP_CMFCR_CP1OVRF | DCMIPP_CMFCR_CATXERRF);
	}
	else if(pipe == PipeID::Auxiliary) {
		// Disable Interrupts first to stop any overrun/error IRQ storm
		MODIFY_REG(this->instance->CMIER, DCMIPP_CMIER_P2LINEIE | DCMIPP_CMIER_P2FRAMEIE | DCMIPP_CMIER_P2VSYNCIE | DCMIPP_CMIER_P2OVRIE, 0x00);

		// Stop the capture
		CLEAR_BIT(this->instance->P2FCTCR, DCMIPP_P2FCTCR_CPTREQ);

		// Wait until capture is no longer active
		while((this->instance->CMSR1 & DCMIPP_CMSR1_P2CPTACT) != 0U) {
			timeout = timeout - 1;
			if(timeout == 0) {
				// Proceed to force-disable PIPEN below
				break;
			}
		}

		// Disable Double Buffering to reset state and clear line wrapping
		CLEAR_BIT(this->instance->P2PPCR, DCMIPP_P2PPCR_DBM);
		MODIFY_REG(this->instance->P2PPCR, DCMIPP_P2PPCR_LINEMULT_Msk, (static_cast<uint32_t>(LineCount::Lines128) << DCMIPP_P2PPCR_LINEMULT_Pos));

		// Disable Pipe Processing
		CLEAR_BIT(this->instance->P2FSCR, DCMIPP_P2FSCR_PIPEN);

		// Clear any pending flags
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP2LINEF | DCMIPP_CMFCR_CP2FRAMEF | DCMIPP_CMFCR_CP2VSYNCF | DCMIPP_CMFCR_CP2OVRF);
	}
	else {
		return Status::Error;
	}

	return Status::Ok;
}

Status Dcmipp::CaptureWait(PipeID pipe, uint32_t timeoutTicks) {
	// Wait for event
	ULONG events;
	uint32_t errFlag = GetPipeEventMask(pipe, EVT_ERR);
	uint32_t vsyncFlag = GetPipeEventMask(pipe, EVT_VSYNC);
	uint32_t cpltFlag = GetPipeEventMask(pipe, EVT_TRANS_CPLT);
	uint32_t waitMask = cpltFlag | errFlag;

	bool isLineWrapActive = (pipe == PipeID::Main && (this->instance->P1PPCR & DCMIPP_P1PPCR_LMAWE) != 0U) || (pipe == PipeID::Auxiliary && (this->instance->P2PPCR & DCMIPP_P2PPCR_LMAWE) != 0U);

	if(isLineWrapActive == true) {
		// In line-wrapping mode, synchronize on a fresh VSYNC so VENC is armed at line 0
		tx_event_flags_set(&this->event, ~vsyncFlag, TX_AND);
		waitMask = vsyncFlag | errFlag;
	}

	UINT status = tx_event_flags_get(&this->event, waitMask, TX_OR_CLEAR, &events, timeoutTicks);
	if(status != TX_SUCCESS) {
		this->CaptureAbort(pipe);
		return Status::Timeout;
	}

	if((events & errFlag) != 0x00) {
		// Transfer error occurred
		this->CaptureAbort(pipe);
		return Status::Error;
	}

	return Status::Ok;
}

void Dcmipp::ConfigureRIF(void) {
	// Essential stuff so that IDMA has access to SRAM regions!!
	// Without this the IDMA can't access the SRAM, the transfer would not fail but would write 0 and read to nullptr
	// https://github.com/STMicroelectronics/STM32CubeN6/blob/main/Projects/STM32N6570-DK/Examples/SD/SD_ReadWrite_DMA/FSBL/Src/main.c

	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);

	// Some RIF constants (from HAL)
	const uint32_t RIF_PERIPH_REG2 = 0x20000000U;
	const uint32_t RIF_CID_1 = 0x00000002U;
	const uint32_t RIF_PERIPH_REG_SHIFT = 28U;
	const uint32_t RIF_PERIPH_BIT_POSITION = 0x0000001FU;

	uint32_t masterID = 0;
	uint32_t periphID = 0;
	if(this->instance == DCMIPP) {
		masterID = 9U;		// RIMU (Refernce Manual 6.3.4 Table 22 pg. 248)
		periphID = (RIF_PERIPH_REG2 | RIFSC_RISC_SECCFGRx_SEC29_Pos);
	}
	else {
		return;
	}

	// RIMC_ATTRx: Controls if IDMA can read/write Secure/Privileged memory: Set to this master is secure and privilaged
	uint32_t masterCID = POSITION_VAL(RIF_CID_1);
	uint32_t wMask = (RIFSC_RIMC_ATTRx_MCID | RIFSC_RIMC_ATTRx_MPRIV | RIFSC_RIMC_ATTRx_MSEC);
	uint32_t wValue = ((masterCID << RIFSC_RIMC_ATTRx_MCID_Pos) | (0x03 << RIFSC_RIMC_ATTRx_MSEC_Pos));		//Bit 0: Master Secure; Bit 1: Master priviliged
	MODIFY_REG(RIFSC->RIMC_ATTRx[masterID], wMask, wValue);

	// Allows CPU to access DCMIPP registers in Secure/Privileged mode.
	// Slave security configuration register: 0: Secure and nonsecure data access are granted to the peripheral; 1: Secure data access only are granted to the peripheral
	wMask = (1UL << (periphID & RIF_PERIPH_BIT_POSITION));
	wValue = ((0x01) << (periphID & RIF_PERIPH_BIT_POSITION));		//0: Secure and nonsecure data access are granted; 1: Only secure access granted
	MODIFY_REG(RIFSC->RISC_SECCFGRx[periphID >> RIF_PERIPH_REG_SHIFT], wMask, wValue);

	// Slave privileged configuration register: 0: Privileged and unprivileged data access are granted to the peripheral; 1: Privileged data access only are granted to the peripheral
	wMask = (1UL << (periphID & RIF_PERIPH_BIT_POSITION));
	wValue = (0x01) << (periphID & RIF_PERIPH_BIT_POSITION);		//0: Secure and nonsecure data access are granted; 1: Only secure access granted
	MODIFY_REG(RIFSC->RISC_PRIVCFGRx[periphID >> RIF_PERIPH_REG_SHIFT], wMask, wValue);
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------
void Dcmipp::InterruptHandler() {
	uint32_t cmsr2 = this->instance->CMSR2;
	uint32_t cmier = this->instance->CMIER;

	// Pipe 0 Interrupts
	// Handle Pipe 0 LIMIT Error Event
	if(((cmsr2 & DCMIPP_CMSR2_P0LIMITF) == DCMIPP_CMSR2_P0LIMITF) && ((cmier & DCMIPP_CMIER_P0LIMITIE) == DCMIPP_CMIER_P0LIMITIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP0LIMITF);
	}

	// Handle Pipe 0 VSYNC Event
	if(((cmsr2 & DCMIPP_CMSR2_P0VSYNCF) == DCMIPP_CMSR2_P0VSYNCF) && ((cmier & DCMIPP_CMIER_P0VSYNCIE) == DCMIPP_CMIER_P0VSYNCIE)) {
		this->stats[0].lastVsyncTimestamp = Time::GetUs();
		// Reset line timestamp so the first slice duration is measured from VSYNC
		this->stats[0].lastLineTimestamp = this->stats[0].lastVsyncTimestamp;
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP0VSYNCF);

		tx_event_flags_set(&this->event, GetPipeEventMask(PipeID::Dump, EVT_VSYNC), TX_OR);
	}

	// Handle Pipe 0 FRAME Event
	if(((cmsr2 & DCMIPP_CMSR2_P0FRAMEF) == DCMIPP_CMSR2_P0FRAMEF) && ((cmier & DCMIPP_CMIER_P0FRAMEIE) == DCMIPP_CMIER_P0FRAMEIE)) {
		this->stats[0].lastFrameTimestamp = Time::GetUs();
		this->stats[0].frameCount++;
		// If snapshot mode, automatically disable interrupts and reset states
		if((this->instance->P0FCTCR & DCMIPP_P0FCTCR_CPTMODE) == DCMIPP_P0FCTCR_CPTMODE) {
			// Disable Interrupts
			MODIFY_REG(this->instance->CMIER, DCMIPP_CMIER_P0FRAMEIE | DCMIPP_CMIER_P0VSYNCIE | DCMIPP_CMIER_P0OVRIE, 0x00);

			// Disable Pipe Processing
			CLEAR_BIT(this->instance->P0FSCR, DCMIPP_P0FSCR_PIPEN);
		}

		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP0FRAMEF);

		tx_event_flags_set(&this->event, GetPipeEventMask(PipeID::Dump, EVT_TRANS_CPLT), TX_OR);
	}

	// Handle Pipe 0 LINE Event
	if(((cmsr2 & DCMIPP_CMSR2_P0LINEF) == DCMIPP_CMSR2_P0LINEF) && ((cmier & DCMIPP_CMIER_P0LINEIE) == DCMIPP_CMIER_P0LINEIE)) {
		uint64_t now = Time::GetUs();
		this->stats[0].sliceDurationUs = static_cast<uint32_t>(now - this->stats[0].lastLineTimestamp);
		this->stats[0].lastLineTimestamp = now;
		this->stats[0].lineEvents++;
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP0LINEF);
	}

	// Handle Pipe 0 Overrun error Event
	if(((cmsr2 & DCMIPP_CMSR2_P0OVRF) == DCMIPP_CMSR2_P0OVRF) && ((cmier & DCMIPP_CMIER_P0OVRIE) == DCMIPP_CMIER_P0OVRIE)) {
		this->stats[0].overrunErrors++;
		// Disable interupt, prevent interrupt storm
		CLEAR_BIT(this->instance->CMIER, DCMIPP_CMIER_P0OVRIE);

		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP0OVRF);

		tx_event_flags_set(&this->event, GetPipeEventMask(PipeID::Dump, EVT_ERR), TX_OR);
	}

	// Pipe 1 Interrupts
	// Handle Pipe 1 LINE Event
	if(((cmsr2 & DCMIPP_CMSR2_P1LINEF) == DCMIPP_CMSR2_P1LINEF) && ((cmier & DCMIPP_CMIER_P1LINEIE) == DCMIPP_CMIER_P1LINEIE)) {
		uint64_t now = Time::GetUs();
		this->stats[1].sliceDurationUs = static_cast<uint32_t>(now - this->stats[1].lastLineTimestamp);
		this->stats[1].lastLineTimestamp = now;
		this->stats[1].lineEvents++;
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP1LINEF);
	}

	// Handle Pipe 1 VSYNC Event
	if(((cmsr2 & DCMIPP_CMSR2_P1VSYNCF) == DCMIPP_CMSR2_P1VSYNCF) && ((cmier & DCMIPP_CMIER_P1VSYNCIE) == DCMIPP_CMIER_P1VSYNCIE)) {
		this->stats[1].lastVsyncTimestamp = Time::GetUs();
		// Reset line timestamp so the first slice duration is measured from VSYNC
		this->stats[1].lastLineTimestamp = this->stats[1].lastVsyncTimestamp;
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP1VSYNCF);

		tx_event_flags_set(&this->event, GetPipeEventMask(PipeID::Main, EVT_VSYNC), TX_OR);
	}

	// Handle Pipe 1 Frame End Event
	if(((cmsr2 & DCMIPP_CMSR2_P1FRAMEF) == DCMIPP_CMSR2_P1FRAMEF) && ((cmier & DCMIPP_CMIER_P1FRAMEIE) == DCMIPP_CMIER_P1FRAMEIE)) {
		this->stats[1].lastFrameTimestamp = Time::GetUs();
		this->stats[1].frameCount++;
		// If snapshot mode, automatically disable interrupts and reset states
		if((this->instance->P1FCTCR & DCMIPP_P1FCTCR_CPTMODE) == DCMIPP_P1FCTCR_CPTMODE) {
			// Disable Interrupts
			MODIFY_REG(this->instance->CMIER, DCMIPP_CMIER_P1FRAMEIE | DCMIPP_CMIER_P1VSYNCIE | DCMIPP_CMIER_P1OVRIE, 0x00);

			// Disable Pipe Processing
			CLEAR_BIT(this->instance->P1FSCR, DCMIPP_P1FSCR_PIPEN);
		}

		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP1FRAMEF);

		tx_event_flags_set(&this->event, GetPipeEventMask(PipeID::Main, EVT_TRANS_CPLT), TX_OR);
	}

	// Handle Overrun and Synchronization Errors as needed
	if(((cmsr2 & DCMIPP_CMSR2_P1OVRF) == DCMIPP_CMSR2_P1OVRF) && ((cmier & DCMIPP_CMIER_P1OVRIE) == DCMIPP_CMIER_P1OVRIE)) {
		this->stats[1].overrunErrors++;
		// Disable interupt, prevent interrupt storm
		CLEAR_BIT(this->instance->CMIER, DCMIPP_CMIER_P1OVRIE);

		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP1OVRF);

		tx_event_flags_set(&this->event, GetPipeEventMask(PipeID::Main, EVT_ERR), TX_OR);
	}
	
	// Pipe 2 Interrupts
	// Handle Pipe 2 LINE Event
	if(((cmsr2 & DCMIPP_CMSR2_P2LINEF) == DCMIPP_CMSR2_P2LINEF) && ((cmier & DCMIPP_CMIER_P2LINEIE) == DCMIPP_CMIER_P2LINEIE)) {
		uint64_t now = Time::GetUs();
		this->stats[2].sliceDurationUs = static_cast<uint32_t>(now - this->stats[2].lastLineTimestamp);
		this->stats[2].lastLineTimestamp = now;
		this->stats[2].lineEvents++;
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP2LINEF);
	}

	// Handle Pipe 2 VSYNC Event
	if(((cmsr2 & DCMIPP_CMSR2_P2VSYNCF) == DCMIPP_CMSR2_P2VSYNCF) && ((cmier & DCMIPP_CMIER_P2VSYNCIE) == DCMIPP_CMIER_P2VSYNCIE)) {
		this->stats[2].lastVsyncTimestamp = Time::GetUs();
		// Reset line timestamp so the first slice duration is measured from VSYNC
		this->stats[2].lastLineTimestamp = this->stats[2].lastVsyncTimestamp;
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP2VSYNCF);

		tx_event_flags_set(&this->event, GetPipeEventMask(PipeID::Auxiliary, EVT_VSYNC), TX_OR);
	}

	// Handle Pipe 2 Frame End Event
	if(((cmsr2 & DCMIPP_CMSR2_P2FRAMEF) == DCMIPP_CMSR2_P2FRAMEF) && ((cmier & DCMIPP_CMIER_P2FRAMEIE) == DCMIPP_CMIER_P2FRAMEIE)) {
		this->stats[2].lastFrameTimestamp = Time::GetUs();
		this->stats[2].frameCount++;
		// If snapshot mode, automatically disable interrupts and reset states
		if((this->instance->P2FCTCR & DCMIPP_P2FCTCR_CPTMODE) == DCMIPP_P2FCTCR_CPTMODE) {
			// Disable Interrupts
			MODIFY_REG(this->instance->CMIER, DCMIPP_CMIER_P2FRAMEIE | DCMIPP_CMIER_P2VSYNCIE | DCMIPP_CMIER_P2OVRIE, 0x00);

			// Disable Pipe Processing
			CLEAR_BIT(this->instance->P2FSCR, DCMIPP_P2FSCR_PIPEN);
		}

		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP2FRAMEF);

		tx_event_flags_set(&this->event, GetPipeEventMask(PipeID::Auxiliary, EVT_TRANS_CPLT), TX_OR);
	}

	// Handle Pipe 2 Overrun and Synchronization Errors as needed
	if(((cmsr2 & DCMIPP_CMSR2_P2OVRF) == DCMIPP_CMSR2_P2OVRF) && ((cmier & DCMIPP_CMIER_P2OVRIE) == DCMIPP_CMIER_P2OVRIE)) {
		this->stats[2].overrunErrors++;
		// Disable interupt, prevent interrupt storm
		CLEAR_BIT(this->instance->CMIER, DCMIPP_CMIER_P2OVRIE);

		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP2OVRF);

		tx_event_flags_set(&this->event, GetPipeEventMask(PipeID::Auxiliary, EVT_ERR), TX_OR);
	}

	// Synchronization Error Interrupt on the parallel interface
	if(((cmsr2 & DCMIPP_CMSR2_PRERRF) == DCMIPP_CMSR2_PRERRF) && ((cmier & DCMIPP_CMIER_PRERRIE) == DCMIPP_CMIER_PRERRIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CPRERRF);
	}

	// IPPLUG AXI transfer Error Interrupt
	if(((cmsr2 & DCMIPP_CMSR2_ATXERRF) == DCMIPP_CMSR2_ATXERRF) && ((cmier & DCMIPP_CMIER_ATXERRIE) == DCMIPP_CMIER_ATXERRIE)) {
		// Disable interupt, prevent interrupt storm
		CLEAR_BIT(this->instance->CMIER, DCMIPP_CMIER_ATXERRIE);

		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CATXERRF);
		
		uint32_t allErrs = GetPipeEventMask(PipeID::Dump, EVT_ERR) |
							GetPipeEventMask(PipeID::Main, EVT_ERR) |
							GetPipeEventMask(PipeID::Auxiliary, EVT_ERR);
		tx_event_flags_set(&this->event, allErrs, TX_OR);
	}
}