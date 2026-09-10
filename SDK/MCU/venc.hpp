/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	SDK/MCU/venc.hpp
 * Author:	NotBlackMagic
 * Brief:	VENC (H.264) hardware codec driver for STM32N6 encapsulating the core encoding API.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_venc.h"
#include "stm32n6xx_ll_bus.h"

#include "h264encapi.h"
#include "jpegencapi.h"
#include "ewl.h"
#include "reg_offset_v7.h"

#include "logger.hpp"
#include "status.hpp"
#include "system.hpp"

#include "tx_api.h"

/// @brief Driver for the STM32 Hardware VENC (H.264) Codec.
class Venc {
	public:
		/// @brief Used VENC encoder/decoder mode.
		enum class Codec : uint32_t {
			H264 = 0,
			Jpeg = 1
		};

		/// @brief Input color space and format parameters.
		enum class InputFormat : uint32_t {
			YUV420Planar = 0,
			YUV420SemiPlanar = 1,
			YUV422InterleavedYUYV = 2,
			RGB565 = 3,
			RGB888 = 4
		};

		/// @brief H.264 Frame coding type.
		enum class FrameType : uint32_t {
			Intra = 0,			// IDR I-Frame
			Predicted = 1,		// P-Frame
			NonIdrIntra = 2		// Non-IDR I-Frame
		};

		/// @brief H.264 Profiles defining compression feature sets.
		enum class H264Profile : uint32_t {
			Baseline = 0,		// Low latency/power, no CABAC, no 8x8 transform
			Main = 1,			// Good balance, uses CABAC
			High = 2			// Best compression, uses CABAC and 8x8 transform
		};

		/// @brief Total line capacity of the circular wrap buffer (2^k lines).
		/// @brief Line interval between hardware trigger pulses (2^k lines).
		enum class LineCount : uint32_t {
			Lines1 = 0,
			Lines2 = 1,
			Lines4 = 2,
			Lines8 = 3,
			Lines16 = 4,
			Lines32 = 5,
			Lines64 = 6,
			Lines128 = 7
		};

		/// @brief Hardware and Encoding Events.
		enum class Event {
			FrameReady,
			SliceReady,
			DesyncError,
			TimeoutError,
			SystemError
		};

		/// @brief Parameters for the source image being encoded.
		struct ImageParams {
			uint32_t width;
			uint32_t height;
			uint32_t frameRate;
			InputFormat inputFormat;
		};

		/// @brief H.264 Rate Control configuration.
		struct RateControl {
			bool enablePictureRc = true;
			uint32_t bitPerSecond = 0;	// 0 = Auto-calculate based on resolution
			uint32_t gopLen = 30;
			uint32_t qpMin = 10;
			uint32_t qpMax = 51;
			uint32_t qpHdr = 26;
		};

		/// @brief H.264 Coding and Slice control.
		struct CodingControl {
			H264Profile profile = H264Profile::High;
			uint32_t sliceSize = 0;
			bool insertIdrHeader = true;
			bool disableDeblockingFilter = false;
		};

		/// @brief VENC peripheral hardware configuration.
		struct Config {
			Codec codec;
			ImageParams imageParams;
			RateControl rateControl;		// For H264
			CodingControl codingControl;	// For H264
			uint32_t jpegQuality;			// For JPEG
			bool enableStreamingMode;		// For streaming partial frames, no complete frames mode
			LineCount lineWrap = LineCount::Lines64;	// Total lines in buffer (e.g., 64 or 128)
			LineCount lineTrigger = LineCount::Lines32;	// Handshake trigger step (e.g., 16 or 32)

			// Memory pool for encoder internal states
			uint8_t* poolAddress;
			uint32_t poolSize;

			void (*EventCallback)(void* ctx, Event evt);
			void* callbackContext;
		};

		/// @brief Buffer addresses for a single frame encoding operation.
		struct FrameBuffer {
			const uint32_t* busLuma;
			const uint32_t* busChromaU;
			const uint32_t* busChromaV;
			uint32_t* outBuffer;
			uint32_t outBufSize;
			FrameType frameType;
		};

		// Delete copy constructors.
		Venc(const Venc&) = delete;
		Venc& operator=(const Venc&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance.
		Venc(void* instance);

		/// @brief Initializes the VENC peripheral, clocks, and internal encoder state.
		/// @param config VENC hardware configuration and memory pool pointers.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config& config);

		/// @brief Generates the initial Sequence/Picture Parameter Sets (SPS/PPS).
		/// @param outBuffer		Output destination for the bitstream headers.
		/// @param outBufSize		Maximum size of the output buffer.
		/// @param generatedBytes	Returns the number of bytes written.
		/// @return Status::Ok if encoding started, Status::Busy if already encoding.
		Status Start(uint32_t* outBuffer, uint32_t outBufSize, uint32_t& generatedBytes);

		/// @brief Encodes a single frame.
		/// @param params 			Frame buffer pointers, for input and output.
		/// @param generatedBytes	Returns the number of bytes written.
		/// @param outFrameType 	Type of generated output frame.
		/// @return Status::Ok if the encoding was successful, or Status::Error if failed.
		Status EncodeFrame(const FrameBuffer& params, uint32_t& generatedBytes, FrameType& outFrameType);

		/// @brief Terminates the stream and flushes resources (writes EOS NALU).
		/// @param outBuffer		Output destination for the bitstream headers.
		/// @param outBufSize		Maximum size of the output buffer.
		/// @param generatedBytes	Returns the number of bytes written.
		/// @return Status::Ok.
		Status Stop(uint32_t* outBuffer, uint32_t outBufSize, uint32_t& generatedBytes);

		/// @brief Triggers a peripheral reset to recover from fuse/desync errors.
		void Reset();

		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler();

		/// @brief Global singleton accessor used by C EWL bridge functions.
		static Venc* GetInstance() { return instance; }

		// Internal EWL Bridge Methods (Called by extern "C" EWL functions)
		int32_t AllocateLinearMemory(uint32_t size, EWLLinearMem_t* info);
		void FreeLinearMemory(EWLLinearMem_t* info);
		void* AllocateMemory(uint32_t size);
		void FreeMemory(void* ptr);
		int32_t WaitHardwareReady(uint32_t* slicesReady);

	private:
		// VENC_TypeDef* instance;
		static Venc* instance;

		IRQn_Type irqCall;
		uint8_t irqPriority;
		Config config;
		bool isInitialized;

		// ThreadX Primitives
		TX_MUTEX mutex;
		TX_EVENT_FLAGS_GROUP event;
		TX_BYTE_POOL bytePool;

		// Variables to hold the latest slice info for the application to read
		uint32_t lastSliceSize = 0;
		uint32_t lastSliceOffset = 0;

		// Event Flags Definitions
		static constexpr ULONG EVT_SLICE_RDY = 0x01;
		static constexpr ULONG EVT_ERR = 0x02;

		// Timeout defines
		static constexpr ULONG TIMEOUT_MUTEX = 100;
		static constexpr ULONG TIMEOUT_HW = 200;

		H264EncInst h264Encoder;
		H264EncIn h264EncIn;
		H264EncOut h264EncOut;
		JpegEncInst jpegEncoder;
		JpegEncIn jpegEncIn;
		JpegEncOut jpegEncOut;
		uint32_t frameCount;

		// Internal setup helpers
		Status InitH264();
		Status InitJPEG();
		Status SetPreProcessing();
		Status SetCodingControl();
		Status SetRateControl();

		// Internal VENC hardware callback handlers
		static void SliceReadyCallback(H264EncSliceReady* sliceReady);

		/// @brief Helper to configure Resource Isolation Framework (RIF)
		void ConfigureRIF(void);
};