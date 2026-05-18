/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/dcmipp.hpp
 * Author:  NotBlackMagic
 * Brief:   DCMIPP (Digital Camera Interface Pixel Pipeline) driver class for STM32N6.
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

		/// @brief Configuration for a specific pipeline.
		struct PipeConfig {
			uint32_t frameRate;		// Frame rate control/decimation
			OutputFormat format;	// Output format
			bool swapRBUV;			// If should swap R-vs-B or U-vs-V
			uint32_t pixelPitch;	// Memory stride/pitch for DMA
		};

		/// @brief Unified memory destination config. Replaces multiple Start() functions.
		struct MemoryDestination {
			uint32_t primaryAddress = 0;   ///< Main buffer (or Y buffer for planar)
			uint32_t secondaryAddress = 0; ///< Used for Double Buffering (Buffer 1)
			uint32_t uAddress = 0;         ///< U plane (for Semi/Full Planar modes)
			uint32_t vAddress = 0;         ///< V plane (for Full Planar modes)
			
			bool isDoubleBuffered = false; ///< True to enable hardware double buffering
			bool isSemiPlanar = false;     ///< True if routing Y and UV to separate buffers
			bool isFullPlanar = false;     ///< True if routing Y, U, and V to separate buffers
		};

		// Delete copy constructors
		Dcmipp(const Dcmipp&) = delete;
		Dcmipp& operator=(const Dcmipp&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., DCMIPP).
		Dcmipp(DCMIPP_TypeDef *instance);

		/// @brief Initializes the DCMIPP block and AXI IP-Plug.
		Status Init();

		/// @brief Binds a specific CSI Virtual Channel and MIPI Data Type to a Pipe.
		/// @param pipe		Target pipe to link.
		/// @param vc 		Source Virtual Channel from the CSI.
		/// @param dataType	MIPI Data Type to filter for this pipe.
		Status LinkCSIVirtualChannel(PipeID pipe, Csi::VirtualChannel vc, Csi::MIPIDataType dataType);

		/// @brief Configures the pixel packer and formatting for a specific pipe.
		Status ConfigurePipe(PipeID pipe, const PipeConfig &config);

		/// @brief Starts capturing frames on the specified pipe to the given memory address.
		/// @param pipe	Target pipe.
		/// @param dst	
		/// @param mode	Capturing mode (Snapshot or Continuous).
		Status CaptureAsync(PipeID pipe, const MemoryDestination &dest, CaptureMode mode);

		/// @brief Aborts the current capture on the specified pipe.
		Status CaptureAbort(PipeID pipe);

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