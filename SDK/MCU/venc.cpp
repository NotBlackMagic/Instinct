/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/venc.cpp
 */

#include "venc.hpp"
Venc* Venc::instance = nullptr;

Venc::Venc(void* instance) {
	this->isInitialized = false;
	this->irqPriority = 0x0E; // Lowest priority (safe default)
	this->h264Encoder = nullptr;
	this->jpegEncoder = nullptr;
	this->frameCount = 0;
}

Status Venc::Init(const Config& config) {
	if(this->isInitialized == true) {
		return Status::Ok;
	}

	// Create RTOS objects
	if(tx_mutex_create(&this->mutex, const_cast<char*>("venc mutex"), TX_INHERIT) != TX_SUCCESS) {
		return Status::Error;
	}
	if(tx_event_flags_create(&this->event, const_cast<char*>("venc event")) != TX_SUCCESS) {
		tx_mutex_delete(&this->mutex);
		return Status::Error;
	}

	// Create ThreadX Byte Pool
	if(tx_byte_pool_create(&this->bytePool, const_cast<char*>("venc pool"), config.poolAddress, config.poolSize) != TX_SUCCESS) {
		tx_event_flags_delete(&this->event);
		tx_mutex_delete(&this->mutex);
		return Status::Error;
	}

	this->config = config;
	Venc::instance = this;

	// Enable bus clocks
	WRITE_REG(RCC->BUSENSR, RCC_BUSENSR_APB5ENS);	// From errata
	LL_MEM_EnableClock(LL_MEM_VENCRAM);	// Enable VENCRAM
	LL_APB5_GRP1_EnableClock(LL_APB5_GRP1_PERIPH_VENC);
	this->irqCall = VENC_IRQn;

	// Configure RIF (enable IDMA secure region access, etc...)
	this->ConfigureRIF();

	Status status = Status::Error;
	if(config.codec == Codec::H264) {
		status = this->InitH264();
	}
	else {
		status = this->InitJPEG();
	}

	// Configure VENC Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);

	if(status == Status::Ok) {
		this->isInitialized = true;
	}
	return status;
}

Status Venc::Start(uint32_t* outBuffer, uint32_t outBufSize, uint32_t& generatedBytes) {
	if(config.codec == Codec::Jpeg) {
		generatedBytes = 0;
		return Status::Ok;
	}

	// Lock VENC device
	if(tx_mutex_get(&this->mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
		return Status::Timeout;
	}

	// Assign buffers to input structure
	h264EncIn.pOutBuf = outBuffer;
	h264EncIn.busOutBuf = reinterpret_cast<uint32_t>(outBuffer);
	h264EncIn.outBufSize = outBufSize;
	// h264EncIn.sendAUD = 1; // Access Unit Delimiter generation enabled

	H264EncRet ret = H264EncStrmStart(this->h264Encoder, &h264EncIn, &h264EncOut);
	
	tx_mutex_put(&this->mutex);
	
	if(ret != H264ENC_OK) {
		return Status::Error;
	}
	generatedBytes = h264EncOut.streamSize;
	return Status::Ok;
}

Status Venc::EncodeFrame(const FrameBuffer& params, uint32_t& generatedBytes, FrameType& outFrameType) {
	// Lock VENC device
	if(tx_mutex_get(&this->mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
		return Status::Timeout;
	}
	
	Status status = Status::Error;
	if(this->config.codec == Codec::H264) {
		// Dynamically compute Frame Coding Type sequence rules
		if(params.frameType == FrameType::Intra || (config.rateControl.gopLen > 0 && (this->frameCount % config.rateControl.gopLen) == 0x00)) {
			h264EncIn.timeIncrement = 0;
			h264EncIn.codingType = H264ENC_INTRA_FRAME;
		}
		else {
			h264EncIn.timeIncrement = 1;
			h264EncIn.codingType = H264ENC_PREDICTED_FRAME;
		}

		h264EncIn.ipf = H264ENC_REFERENCE_AND_REFRESH;
		h264EncIn.ltrf = H264ENC_REFERENCE;

		// Set input buffers to structures
		// Map input frame buffers
		h264EncIn.busLuma = reinterpret_cast<uint32_t>(params.busLuma);
		h264EncIn.busChromaU = reinterpret_cast<uint32_t>(params.busChromaU);
		h264EncIn.busChromaV = reinterpret_cast<uint32_t>(params.busChromaV);
		// Map output bitstream destinations
		h264EncIn.pOutBuf = params.outBuffer;
		h264EncIn.busOutBuf = reinterpret_cast<uint32_t>(params.outBuffer);
		h264EncIn.outBufSize = params.outBufSize;

		// Fire off hardware block encode execution operation
		H264EncRet ret = H264EncStrmEncode(this->h264Encoder, &this->h264EncIn, &this->h264EncOut, nullptr, nullptr, nullptr);
		switch(ret) {
			case H264ENC_FRAME_READY:
				if(h264EncOut.streamSize > 0) {
					generatedBytes = h264EncOut.streamSize;
					outFrameType = (h264EncIn.codingType == H264ENC_INTRA_FRAME) ? FrameType::Intra : FrameType::Predicted;
					this->frameCount++;
					status = Status::Ok;
				}
				break;
			case H264ENC_SYSTEM_ERROR:
				if(config.EventCallback != nullptr) {
					config.EventCallback(config.callbackContext, Event::SystemError);
				}
				status = Status::Error;
				break;
			case H264ENC_FUSE_ERROR:
				if(config.EventCallback != nullptr) {
					config.EventCallback(config.callbackContext, Event::DesyncError);
				}
				status = Status::Error;
				break;
			case H264ENC_HW_TIMEOUT:
				if(config.EventCallback != nullptr) {
					config.EventCallback(config.callbackContext, Event::TimeoutError);
				}
				status = Status::Error;
				break;
			default:
				status = Status::Error;
				break;
		}
	}
	else {
		outFrameType = FrameType::Intra;
		
		this->jpegEncIn.busLum = reinterpret_cast<uint32_t>(params.busLuma);
		if(this->config.imageParams.inputFormat == InputFormat::YUV420Planar || this->config.imageParams.inputFormat == InputFormat::YUV420SemiPlanar) {
			this->jpegEncIn.busCb = this->jpegEncIn.busLum + this->config.imageParams.width * this->config.imageParams.height;
			this->jpegEncIn.busCr = this->jpegEncIn.busCb + (this->config.imageParams.width / 2) * (this->config.imageParams.height / 2);
		}
		else {
			this->jpegEncIn.busCb = this->jpegEncIn.busLum + this->config.imageParams.width * this->config.imageParams.height;
			this->jpegEncIn.busCr = this->jpegEncIn.busCb + (this->config.imageParams.width / 2) * (this->config.imageParams.height / 2);
		}
		this->jpegEncIn.frameHeader = 1;
		this->jpegEncIn.pOutBuf = reinterpret_cast<uint8_t*>(params.outBuffer);
		this->jpegEncIn.busOutBuf = reinterpret_cast<uint32_t>(params.outBuffer);
		this->jpegEncIn.outBufSize = params.outBufSize;

		JpegEncRet ret = JPEGENC_OK;
		do {
			ret = JpegEncEncode(this->jpegEncoder, &this->jpegEncIn, &this->jpegEncOut, nullptr, nullptr);
			switch(ret) {
				case JPEGENC_RESTART_INTERVAL:
					break;
				case JPEGENC_FRAME_READY:
					generatedBytes = this->jpegEncOut.jfifSize;
					status = Status::Ok;
					break;
				case JPEGENC_SYSTEM_ERROR:
				default:
					if(config.EventCallback != nullptr) {
						config.EventCallback(config.callbackContext, Event::SystemError);
					}
					LOG_ERR("JPEG Error: %d\n", ret);
					status = Status::Error;
					break;
			}
		} while(ret == JPEGENC_RESTART_INTERVAL);
	}

	tx_mutex_put(&this->mutex);
	return status;
}

Status Venc::Stop(uint32_t* outBuffer, uint32_t outBufSize, uint32_t& generatedBytes) {
	// Lock VENC device
	if(tx_mutex_get(&this->mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
		return Status::Timeout;
	}

	if(config.codec == Codec::Jpeg) {
		if(this->jpegEncoder != nullptr) {
			JpegEncRelease(this->jpegEncoder);
			this->jpegEncoder = nullptr;
		}
		generatedBytes = 0;
		tx_mutex_put(&this->mutex);
		return Status::Ok;
	}

	H264EncRet ret = H264EncStrmEnd(this->h264Encoder, &this->h264EncIn, &this->h264EncOut);
	if(ret == H264ENC_OK) {
		generatedBytes = this->h264EncOut.streamSize;
	}
	else {
		generatedBytes = 0;
	}

	H264EncRelease(this->h264Encoder);
	this->h264Encoder = nullptr;

	tx_mutex_put(&this->mutex);
	return (ret == H264ENC_OK) ? Status::Ok : Status::Error;
}

void Venc::Reset() {

}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

void Venc::InterruptHandler() {
	uint32_t irqStatus = LL_VENC_ReadRegister(1UL);
	uint32_t handshakeStatus = READ_BIT(LL_VENC_ReadRegister(BASE_HEncInstantInput >> 2U), (1U << 29U));

	if(handshakeStatus == 0x00 && ((irqStatus & ASIC_STATUS_FUSE) == ASIC_STATUS_FUSE)) {
		LL_VENC_WriteRegister(1UL, ASIC_STATUS_FUSE | ASIC_IRQ_LINE);
		irqStatus = LL_VENC_ReadRegister(1UL);
	}

	if(irqStatus != 0U) {
		// Clear slice ready and IRQ line flags in hardware
		LL_VENC_WriteRegister(1UL, ASIC_STATUS_SLICE_READY | ASIC_IRQ_LINE);
		
		// Signal waiting thread via RTOS event flag
		tx_event_flags_set(&this->event, EVT_SLICE_RDY, TX_OR);
	}
}

Status Venc::InitH264() {
	// Configure VENC Interface
	H264EncConfig h264Config = {};
	h264Config.frameRateDenom = 1;
	h264Config.frameRateNum = this->config.imageParams.frameRate;
	h264Config.width = this->config.imageParams.width;
	h264Config.height = this->config.imageParams.height;
	// Baseline profile??
	h264Config.streamType = H264ENC_BYTE_STREAM;
	h264Config.level = H264ENC_LEVEL_2_2;	//H264ENC_LEVEL_4_1
	h264Config.refFrameAmount = 1;
	h264Config.svctLevel = 0;

	// Initialize encoder instance
	H264EncRet ret = H264EncInit(&h264Config, &this->h264Encoder);
	if(ret != H264ENC_OK) {
		return Status::Error;
	}

	// Set processing, coding and rate controls
	if(this->SetPreProcessing() != Status::Ok) {
		H264EncRelease(&this->h264Encoder);
		return Status::Error;
	}
	if(this->SetCodingControl() != Status::Ok) {
		H264EncRelease(&this->h264Encoder);
		return Status::Error;
	}
	if(this->SetRateControl() != Status::Ok) {
		H264EncRelease(&this->h264Encoder);
		return Status::Error;
	}

	// Configure Interrupts

	return Status::Ok;
}

Status Venc::InitJPEG() {
	JpegEncCfg jpegCfg = {};
	jpegCfg.qLevel = this->config.jpegQuality;
	jpegCfg.inputWidth = this->config.imageParams.width;
	jpegCfg.inputHeight = this->config.imageParams.height;
	jpegCfg.codingWidth = this->config.imageParams.width;
	jpegCfg.codingHeight = this->config.imageParams.height;

	switch(this->config.imageParams.inputFormat) {
		case InputFormat::YUV420Planar:
			jpegCfg.frameType = JPEGENC_YUV420_PLANAR;
			break;
		case InputFormat::YUV422InterleavedYUYV:
			jpegCfg.frameType = JPEGENC_YUV422_INTERLEAVED_YUYV;
			break;
		case InputFormat::RGB565:
			jpegCfg.frameType = JPEGENC_RGB565;
			break;
		case InputFormat::RGB888:
			jpegCfg.frameType = JPEGENC_RGB888;
			break;
		default:
			return Status::Error;
	}

	jpegCfg.xOffset = 0;
	jpegCfg.yOffset = 0;
	jpegCfg.rotation = JPEGENC_ROTATE_0;
	jpegCfg.codingType = JPEGENC_WHOLE_FRAME;
	jpegCfg.codingMode = JPEGENC_422_MODE;
	jpegCfg.unitsType = JPEGENC_DOTS_PER_INCH;
	jpegCfg.markerType = JPEGENC_SINGLE_MARKER;
	jpegCfg.xDensity = 72;
	jpegCfg.yDensity = 72;

	JpegEncRet ret = JpegEncInit(&jpegCfg, &this->jpegEncoder);
	if(ret != JPEGENC_OK) {
		return Status::Error;
	}

	ret = JpegEncSetPictureSize(this->jpegEncoder, &jpegCfg);
	if(ret != JPEGENC_OK) {
		JpegEncRelease(this->jpegEncoder);
		this->jpegEncoder = nullptr;
		return Status::Error;
	}

	memset(&jpegEncIn, 0, sizeof(jpegEncIn));
	System::CleanCache((uint32_t*)this->config.poolAddress, this->config.poolSize);
	return Status::Ok;
}

Status Venc::SetPreProcessing() {
	H264EncPreProcessingCfg preProcConfig;
	H264EncGetPreProcessing(this->h264Encoder, &preProcConfig);

	// Color space selection mapping
	switch(config.imageParams.inputFormat) {
		case InputFormat::YUV420Planar:
			preProcConfig.inputType = H264ENC_YUV420_PLANAR;
			break;
		case InputFormat::YUV420SemiPlanar:
			preProcConfig.inputType = H264ENC_YUV420_SEMIPLANAR;
			break;
		case InputFormat::YUV422InterleavedYUYV:
			preProcConfig.inputType = H264ENC_YUV422_INTERLEAVED_YUYV;
			break;
		case InputFormat::RGB565:
			preProcConfig.inputType = H264ENC_RGB565;
			break;
		case InputFormat::RGB888:
			preProcConfig.inputType = H264ENC_RGB888;
			break;
		default:
			return Status::Error;
	}

	if(H264EncSetPreProcessing(this->h264Encoder, &preProcConfig) != H264ENC_OK) {
		return Status::Error;
	}

	return Status::Ok;
}

Status Venc::SetCodingControl() {
	H264EncCodingCtrl codingCtrl;
	if(H264EncGetCodingCtrl(this->h264Encoder, &codingCtrl) != H264ENC_OK) {
		return Status::Error;
	}

	codingCtrl.sliceSize = this->config.codingControl.sliceSize;
	codingCtrl.enableCabac = this->config.codingControl.enableCabac ? 1 : 0;
	codingCtrl.transform8x8Mode = this->config.codingControl.enableTransform8x8 ? 1 : 0;
	codingCtrl.disableDeblockingFilter = this->config.codingControl.disableDeblockingFilter ? 1 : 0;

	if(H264EncSetCodingCtrl(this->h264Encoder, &codingCtrl) != H264ENC_OK) {
		return Status::Error;
	}
	return Status::Ok;
}

Status Venc::SetRateControl() {
	H264EncRateCtrl rateCtrl;
	if(H264EncGetRateCtrl(this->h264Encoder, &rateCtrl) != H264ENC_OK) {
		return Status::Error;
	}

	// Apply your dynamic configuration
	rateCtrl.pictureRc = this->config.rateControl.enablePictureRc ? 1 : 0;
	rateCtrl.bitPerSecond = this->config.rateControl.bitPerSecond;
	rateCtrl.qpMin = this->config.rateControl.qpMin;
	rateCtrl.qpMax = this->config.rateControl.qpMax;
	rateCtrl.qpHdr = this->config.rateControl.qpHdr;
	rateCtrl.gopLen = this->config.rateControl.gopLen;
	// Set standard/safe H.264 Rate Control defaults (matching STM's tested config)
	rateCtrl.pictureSkip = 0;			// Don't let the encoder drop frames randomly
	rateCtrl.mbRc = 1;					// Enable Macroblock-level rate control for better quality
	rateCtrl.hrd = 1;					// Enable Hypothetical Reference Decoder (strict bitrate constraint)
	rateCtrl.hrdCpbSize = 0;			// 0 = Auto-calculate Max CPB size for the active H.264 Level
	rateCtrl.intraQpDelta = -3;			// Slightly higher quality for I-Frames
	rateCtrl.fixedIntraQp = 0;			// Let rate controller dynamically pick the best I-Frame QP
	rateCtrl.mbQpAdjustment = 0;		// Disable manual adjustment
	rateCtrl.longTermPicRate = 15;		// Standard refresh cadence
	rateCtrl.mbQpAutoBoost = 0;			// Disable auto boost to save encode time

	if(H264EncSetRateCtrl(this->h264Encoder, &rateCtrl) != H264ENC_OK) {
		return Status::Error;
	}
	return Status::Ok;
}

void Venc::ConfigureRIF(void) {
	// Essential stuff so that IDMA has access to SRAM regions!!
	// Without this the IDMA can't access the SRAM, the transfer would not fail but would write 0 and read to nullptr
	// https://github.com/STMicroelectronics/STM32CubeN6/blob/main/Projects/STM32N6570-DK/Examples/SD/SD_ReadWrite_DMA/FSBL/Src/main.c

	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);

	// Some RIF constants (from HAL)
	const uint32_t RIF_PERIPH_REG3 = 0x30000000U;
	const uint32_t RIF_CID_1 = 0x00000002U;
	const uint32_t RIF_PERIPH_REG_SHIFT = 28U;
	const uint32_t RIF_PERIPH_BIT_POSITION = 0x0000001FU;

	uint32_t masterID = 0;
	uint32_t periphID = 0;
	// if(this->instance == VENC) {
		masterID = 12U;		// RIMU (Refernce Manual 6.3.4 Table 22 pg. 248)
		periphID = (RIF_PERIPH_REG3 | RIFSC_RISC_SECCFGRx_SEC1_Pos);	// stm32n6xx_hal_rif.h
	// }
	// else {
	// 	return;
	// }

	// RIMC_ATTRx: Controls if IDMA can read/write Secure/Privileged memory: Set to this master is secure and privilaged
	uint32_t masterCID = POSITION_VAL(RIF_CID_1);
	uint32_t wMask = (RIFSC_RIMC_ATTRx_MCID | RIFSC_RIMC_ATTRx_MPRIV | RIFSC_RIMC_ATTRx_MSEC);
	uint32_t wValue = ((masterCID << RIFSC_RIMC_ATTRx_MCID_Pos) | (0x03 << RIFSC_RIMC_ATTRx_MSEC_Pos));		//Bit 0: Master Secure; Bit 1: Master priviliged
	MODIFY_REG(RIFSC->RIMC_ATTRx[masterID], wMask, wValue);

	// Allows CPU to access VENC registers in Secure/Privileged mode.
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
// Integrated VeriSilicon Embedded Wrapper Layer (EWL) Bridge
// ---------------------------------------------------------

int32_t Venc::AllocateLinearMemory(uint32_t size, EWLLinearMem_t* info) {
	// Force 8-byte alignment for AXI bus transaction efficiency
	uint32_t alignedSize = (size + 7U) & ~7U;
	void* ptr = nullptr;

	// Allocate directly from our dedicated VENC SRAM pool using ThreadX
	if(tx_byte_allocate(&this->bytePool, &ptr, alignedSize, TX_NO_WAIT) != TX_SUCCESS) {
		LOG_ERR("VENC EWL: Out of memory in ThreadX pool! Req: %lu\n", alignedSize);
		return EWL_ERROR;
	}

	info->virtualAddress = reinterpret_cast<u32*>(ptr);
	info->busAddress = reinterpret_cast<ptr_t>(ptr);
	info->size = alignedSize;
	return EWL_OK;
}

void Venc::FreeLinearMemory(EWLLinearMem_t* info) {
	if(info->virtualAddress != nullptr) {
		tx_byte_release(info->virtualAddress);
		info->virtualAddress = nullptr;
		info->busAddress = 0;
		info->size = 0;
	}
}

void* Venc::AllocateMemory(uint32_t size) {
	void* ptr = nullptr;
	// Align to 8 bytes to prevent hardfaults on unaligned struct access
	uint32_t alignedSize = (size + 7U) & ~7U;

	if(tx_byte_allocate(&this->bytePool, &ptr, alignedSize, TX_NO_WAIT) != TX_SUCCESS) {
		LOG_ERR("VENC EWL: EWLmalloc failed size: %lu\n", alignedSize);
		return nullptr;
	}
	return ptr;
}

void Venc::FreeMemory(void* ptr) {
	if(ptr != nullptr) {
		tx_byte_release(ptr);
	}
}

int32_t Venc::WaitHardwareReady(uint32_t* slicesReady) {
	ULONG events = 0;
	// Block thread execution until IRQ fires or timeout occurs
	UINT status = tx_event_flags_get(&this->event, EVT_SLICE_RDY | EVT_ERR, TX_OR_CLEAR, &events, TIMEOUT_HW);

	if(status != TX_SUCCESS) {
		return EWL_HW_WAIT_TIMEOUT;
	}
	if((events & EVT_ERR) == EVT_ERR) {
		return EWL_HW_WAIT_ERROR;
	}

	if(slicesReady != nullptr) {
		*slicesReady = (LL_VENC_ReadRegister(21UL) >> 16) & 0xFFUL;
	}
	return EWL_OK;
}

extern "C" {
	u32 EWLReadAsicID(void) {
		return LL_VENC_ReadRegister(0UL);
	}

	EWLHwConfig_t EWLReadAsicConfig(void) {
		// Read first part of configuration (Register 63)
		u32 cfgval = LL_VENC_ReadRegister(63UL);
		EWLHwConfig_t cfgInfo;
		cfgInfo.maxEncodedWidth = cfgval & ((1U << 12U) - 1U);
		cfgInfo.h264Enabled = (cfgval >> 27U) & 1U;
		cfgInfo.vp8Enabled = (cfgval >> 26U) & 1U;
		cfgInfo.jpegEnabled = (cfgval >> 25U) & 1U;
		cfgInfo.vsEnabled = (cfgval >> 24U) & 1U;
		cfgInfo.rgbEnabled = (cfgval >> 28U) & 1U;
		cfgInfo.searchAreaSmall = (cfgval >> 29U) & 1U;
		cfgInfo.scalingEnabled = (cfgval >> 30U) & 1U;

		cfgInfo.busType = (cfgval >> 20U) & 15U;
		cfgInfo.synthesisLanguage = (cfgval >> 16U) & 15U;
		cfgInfo.busWidth = (cfgval >> 12U) & 15U;

		// Read second part of configuration (Register 296)
		cfgval = LL_VENC_ReadRegister(296UL);
		cfgInfo.addr64Support = (cfgval >> 31U) & 1U;
		cfgInfo.dnfSupport = (cfgval >> 30U) & 1U;
		cfgInfo.rfcSupport = (cfgval >> 28U) & 3U;
		cfgInfo.enhanceSupport = (cfgval >> 27U) & 1U;
		cfgInfo.instantSupport = (cfgval >> 26U) & 1U;
		cfgInfo.svctSupport = (cfgval >> 25U) & 1U;
		cfgInfo.inAxiIdSupport = (cfgval >> 24U) & 1U;
		cfgInfo.inLoopbackSupport = (cfgval >> 23U) & 1U;
		cfgInfo.irqEnhanceSupport = (cfgval >> 22U) & 1U;

		// PTRACE("EWLReadAsicConfig:\n"
		// 	"    maxEncodedWidth   = %d\n"
		// 	"    h264Enabled       = %s\n"
		// 	"    jpegEnabled       = %s\n"
		// 	"    vp8Enabled        = %s\n"
		// 	"    vsEnabled         = %s\n"
		// 	"    rgbEnabled        = %s\n"
		// 	"    searchAreaSmall   = %s\n"
		// 	"    scalingEnabled    = %s\n"
		// 	"    address64bits     = %s\n"
		// 	"    denoiseEnabled    = %s\n"
		// 	"    rfcEnabled        = %s\n"
		// 	"    instanctEnabled   = %s\n"
		// 	"    busType           = %s\n"
		// 	"    synthesisLanguage = %s\n"
		// 	"    busWidth          = %d\n",
		// 	cfgInfo.maxEncodedWidth,
		// 	cfgInfo.h264Enabled == 1 ? "YES" : "NO",
		// 	cfgInfo.jpegEnabled == 1 ? "YES" : "NO",
		// 	cfgInfo.vp8Enabled == 1 ? "YES" : "NO",
		// 	cfgInfo.vsEnabled == 1 ? "YES" : "NO",
		// 	cfgInfo.rgbEnabled == 1 ? "YES" : "NO",
		// 	cfgInfo.searchAreaSmall == 1 ? "YES" : "NO",
		// 	cfgInfo.scalingEnabled == 1 ? "YES" : "NO",
		// 	cfgInfo.addr64Support == 1 ? "YES" : "NO",
		// 	cfgInfo.dnfSupport == 1 ? "YES" : "NO",
		// 	cfgInfo.rfcSupport == 0 ? "NO" :
		// 	(cfgInfo.rfcSupport == 2 ? "LUMA" : (cfgInfo.rfcSupport == 1 ? "CHROMA" : "FULL")),
		// 	cfgInfo.instantSupport == 1 ? "YES" : "NO",
		// 	cfgInfo.busType < 7 ? busTypeName[cfgInfo.busType] : "UNKNOWN",
		// 	cfgInfo.synthesisLanguage < 3
		// 	? synthLangName[cfgInfo.synthesisLanguage]
		// 	: "ERROR",
		// 	cfgInfo.busWidth * 32);

		return cfgInfo;
	}

	const void* EWLInit(EWLInitParam_t* param) {
		if(param == nullptr) {
			return nullptr;
		}
		return static_cast<const void*>(Venc::GetInstance());
	}

	i32 EWLRelease(const void* inst) {
		return EWL_OK;
	}

	void EWLWriteReg(const void* inst, u32 offset, u32 val) {
		LL_VENC_WriteRegister(offset >> 2, val);
	}

	void EWLEnableHW(const void* inst, u32 offset, u32 val) {
		EWLWriteReg(inst, offset, val);
	}

	void EWLDisableHW(const void* inst, u32 offset, u32 val) {
		EWLWriteReg(inst, offset, val);
	}

	u32 EWLReadReg(const void* inst, u32 offset) {
		return LL_VENC_ReadRegister(offset >> 2);
	}

	i32 EWLMallocRefFrm(const void* instance, u32 size, EWLLinearMem_t* info) {
		return EWLMallocLinear(instance, size, info);
	}

	void EWLFreeRefFrm(const void* instance, EWLLinearMem_t* info) {
		EWLFreeLinear(instance, info);
	}

	i32 EWLMallocLinear(const void* instance, u32 size, EWLLinearMem_t* info) {
		if(instance == nullptr || info == nullptr) {
			return EWL_ERROR;
		}
		return const_cast<Venc*>(static_cast<const Venc*>(instance))->AllocateLinearMemory(size, info);
	}

	void EWLFreeLinear(const void* instance, EWLLinearMem_t* info) {
		if(instance != nullptr && info != nullptr) {
			const_cast<Venc*>(static_cast<const Venc*>(instance))->FreeLinearMemory(info);
		}
	}

	i32 EWLReserveHw(const void* inst) {
		return EWL_OK;
	}

	void EWLReleaseHw(const void* inst) {
		(void)inst;
	}

	void* EWLmalloc(u32 n) {
		Venc* instance = Venc::GetInstance();
		return instance ? instance->AllocateMemory(n) : nullptr;
	}

	void EWLfree(void* p) {
		Venc* instance = Venc::GetInstance();
		if(instance != nullptr) {
			instance->FreeMemory(p);
		}
	}

	void* EWLcalloc(u32 n, u32 s) {
		void *p = EWLmalloc(n * s);
		EWLmemset(p, 0, n * s);
		return p;
	}

	void* EWLmemcpy(void* d, const void* s, u32 n) {
		return memcpy(d, s, static_cast<size_t>(n));
	}

	void* EWLmemset(void* d, i32 c, u32 n) {
		return memset(d, c, static_cast<size_t>(n));
	}

	int EWLmemcmp(const void* s1, const void* s2, u32 n) {
		return memcmp(s1, s2, static_cast<size_t>(n));
	}

	i32 EWLWaitHwRdy(const void* inst, u32* slicesReady) {
		if(inst == nullptr) {
			return EWL_HW_WAIT_ERROR;
		}
		return const_cast<Venc*>(static_cast<const Venc*>(inst))->WaitHardwareReady(slicesReady);
	}
}