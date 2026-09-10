/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	SDK/MCU/dcmipp.hpp
 * Author:	NotBlackMagic
 * Brief:	DCMIPP (Digital Camera Interface Pixel Pipeline) driver class for STM32N6.
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"

#include "csi.hpp"
#include "status.hpp"

#include "tx_api.h"

/// @brief Driver for the DCMIPP peripheral.
class Dcmipp {
	public:
		/// @brief Available processing pipes in the DCMIPP.
		enum class PipeID : uint8_t {
			Dump = 0,		// Main dump pipe
			Main = 1,		// ISP processing pipe
			Auxiliary = 2	// Auxiliary pipe
		};

		/// @brief DCMIPP frame capture mode.
		enum class CaptureMode {
			Continuous,
			Snapshot
		};

		/// @brief Output format from the Pixel Packer.
		enum class OutputFormat : uint8_t {
			RGB888 = 0x00,		// RGB888 (packed) or YUV444 1-buffer
			RGB565 = 0x01,		// RGB565 1-buffer
			ARGB8888 = 0x02,	// ARGB8888 (with A = 0xFF)
			RGBA8888 = 0x03,	// RGBA8888 (with A = 0xFF)
			Mono = 0x04,		// Monochrome Y8 or G8 1-buffer
			YUV444 = 0x05,		// YUV444 1-buffer (32 bpp, FOURCC = AYUV, with A = 0xFF)
			YUV422_YUYV = 0x06,	// YUV422 1-buffer (16 bpp, FOURCC = YUYV)
			YUV422 = 0x07,		// YUV422 2-buffer (16 bpp, FOURCC = none)
			YUV420_NV21 = 0x08,	// YUV420 2-buffer (12 bpp, FOURCC = NV21), NV12 available with SWAPRB = 1
			YUV420_YV12 = 0x09,	// YUV420 3-buffer (12 bpp, FOURCC = YV12)
			YUV422_UYVY = 0x0A	// YUV422 1-buffer (16 bpp, FOURCC = UYVY)
		};

		/// @brief Number of lines before memory address wrapping (LMAWM).
		/// @brief Line interval between line event trigger pulses (LINEMULT).
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

		/// @brief Configuration for a specific pipeline.
		struct PipeConfig {
			uint32_t frameRate;		// Frame rate control/decimation
			OutputFormat format;	// Output format
			bool swapRBUV;			// If should swap R-vs-B or U-vs-V
			uint32_t pixelPitch;	// Memory stride/pitch for DMA
		};

		/// @brief Unified memory destination config. Replaces multiple Start() functions.
		struct MemoryDestination {
			uint32_t primaryAddress = 0;	///< Main buffer (or Y buffer for planar)
			uint32_t secondaryAddress = 0;	///< Used for Double Buffering (Buffer 1)
			uint32_t uAddress = 0;			///< U plane (for Semi/Full Planar modes)
			uint32_t vAddress = 0;			///< V plane (for Full Planar modes)
			
			bool isDoubleBuffered = false;	///< True to enable hardware double buffering
			bool isSemiPlanar = false;		///< True if routing Y and UV to separate buffers
			bool isFullPlanar = false;		///< True if routing Y, U, and V to separate buffers
		};

		// Delete copy constructors
		Dcmipp(const Dcmipp&) = delete;
		Dcmipp& operator=(const Dcmipp&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., DCMIPP).
		Dcmipp(DCMIPP_TypeDef *instance);

		/// @brief Initializes the DCMIPP block and AXI IP-Plug.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init();

		/// @brief Binds a specific CSI Virtual Channel and MIPI Data Type to a Pipe.
		/// @param pipe		Target pipe to link.
		/// @param vc 		Source Virtual Channel from the CSI.
		/// @param dataType	MIPI Data Type to filter for this pipe.
		/// @return Status::Ok if the VC link was successful, or Status::Error if failed.
		Status LinkCSIVirtualChannel(PipeID pipe, Csi::VirtualChannel vc, Csi::MIPIDataType dataType);

		/// @brief Configures the pixel packer and formatting for a specific pipe.
		/// @param pipe		Pipe to configure.
		/// @param config	Aaa
		/// @return Status::Ok if the pipe configuration was successful, or Status::Error if failed.
		Status ConfigurePipe(PipeID pipe, const PipeConfig &config);
		
		/// @brief Aaa
		/// @param pipe			Pipe to enable line wrapping mode for.
		/// @param addressWrap	Aaa
		/// @param lineMult		Aaa
		/// @return Status::Ok.
		Status EnableLineWrapping(PipeID pipe, LineCount addressWrap, LineCount lineMult);

		/// @brief Aaa
		/// @param pipe			Pipe to disable line wrapping mode for.
		/// @return Status::Ok.
		Status DisableLineWrapping(PipeID pipe);

		/// @brief Starts capturing frames on the specified pipe to the given memory address.
		/// @param pipe	Pipe from which to capture.
		/// @param dst	Destination memory configuration for the capture.
		/// @param mode	Capturing mode (Snapshot or Continuous).
		/// @return Status::Ok
		Status CaptureAsync(PipeID pipe, const MemoryDestination &dest, CaptureMode mode);

		/// @brief Aborts the current capture on the specified pipe.
		/// @param pipe	Pipe to abort capture.
		/// @return Status::Ok
		Status CaptureAbort(PipeID pipe);

		/// @brief Blocks until the hardware completes the current frame capture.
		/// @param pipe			Pipe to wait capture on.
		/// @param timeoutTicks	Max wait time in OS ticks.
		/// @return Status::Ok if the capture completed successfully, Status::Timeout if it expired, or Status::Error on hardware faults.
		Status CaptureWait(PipeID pipe, uint32_t timeoutTicks);

		/// @brief Interrupt Service Routine handler for DCMIPP (Frame End, Limit, Overrun).
		void InterruptHandler();

	private:
		DCMIPP_TypeDef *instance;
		IRQn_Type irqCall;
		uint8_t irqPriority;
		
		bool isInitialized;

		// Synchronization
		TX_MUTEX mutex;
		TX_EVENT_FLAGS_GROUP event;

		// Event Flags Definitions
		static constexpr uint32_t EVT_TRANS_CPLT = 0x01;
		static constexpr uint32_t EVT_ERR = 0x02;

		/// @brief Helper to configure Resource Isolation Framework (RIF)
		void ConfigureRIF(void);
};