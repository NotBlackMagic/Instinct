/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/csi.hpp
 * Author:  NotBlackMagic
 * Brief:   MIPI CSI-2 receiver driver class for STM32N6 using direct register access.
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"

#include "status.hpp"
#include "system.hpp"

class Csi {
	public:
		/// @brief Number of active data lanes.
		enum class LaneCount : uint8_t {
			One = 1,
			Two = 2
		};

		/// @brief Physical mapping of the data lanes.
		enum class LaneMapping : uint8_t {
			Direct = 0,		// DL0 -> Lane 0, DL1 -> Lane 1
			Inverted = 1	// DL0 -> Lane 1, DL1 -> Lane 0
		};

		/// @brief Standard MIPI CSI-2 Data Type (DT) IDs.
		enum class MIPIDataType : uint8_t {
			// YUV Formats
			YUV420_8bit = 0x18,
			YUV420_10bit = 0x19,
			YUV422_8bit = 0x1E,
			YUV422_10bit = 0x1F,
			// RGB Formats
			RGB444 = 0x20,
			RGB555 = 0x21,
			RGB565 = 0x22,
			RGB666 = 0x23,
			RGB888 = 0x24,
			// RAW Formats
			Raw6 = 0x28,
			Raw7 = 0x29,
			Raw8 = 0x2A,
			Raw10 = 0x2B,
			Raw12 = 0x2C,
			Raw14 = 0x2D,
			// User Defined
			User1 = 0x30,
			User2 = 0x31
		};

		/// @brief MIPI Virtual Channel ID.
		enum class VirtualChannel : uint8_t {
			VC0 = 0,
			VC1 = 1,
			VC2 = 2,
			VC3 = 3
		};

		/// @brief CSI peripheral configuration structure.
		struct Config {
			LaneCount lanes;
			LaneMapping laneMapping;
			uint32_t bitrate;			///< Target bitrate in bit/s
		};

		// Delete copy constructors
		Csi(const Csi&) = delete;
		Csi& operator=(const Csi&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., CSI).
		Csi(CSI_TypeDef *instance);

		/// @brief Initializes the CSI peripheral, configures the D-PHY, and brings it out of reset.
		/// @param config CSI configuration.
		/// @return Status::Ok if initialization succeeded.
		Status Init(const Config &config);

		/// @brief Configures a Virtual Channel to accept a specific MIPI Data Type (e.g., RAW8, RGB565).
		/// @param vc		Virtual Channel to configure.
		/// @param dataType	MIPI Data Type.
		/// @return Status::Ok.
		Status ConfigureVirtualChannel(VirtualChannel vc, MIPIDataType dataType);

		/// @brief Starts the CSI reception.
		/// @param vc Virtual Channel to enable/start.
		/// @return Status::Ok.
		Status Start(VirtualChannel vc);

		/// @brief Stops the CSI reception and powers down the PHY.
		/// @param vc Virtual Channel to disable/stop.
		/// @return Status::Ok.
		Status Stop(VirtualChannel vc);

		/// @brief Gets/reads last decoded short package i.e. last MIPI-CSI information packet.
		/// @param vc		Read/decoded virtual channel number.
		/// @param dataType	Read/decoded MIPI Data Type.
		/// @param data		Read/decoded data field.
		/// @return Status::Ok.
		Status GetLastPacketInfo(VirtualChannel& vc, MIPIDataType& dataType, uint16_t& data);

		/// @brief Interrupt Service Routine handler for CSI (Line errors, ECC, CRC, etc.).
		void InterruptHandler();

	private:
		CSI_TypeDef *instance;
		IRQn_Type irqCall;
		uint8_t irqPriority;

		Config config;
		bool isInitialized;

		// Internal helper to write to the Synopsys PHY test registers
		void WritePhyRegister(uint32_t regMsb, uint32_t regLsb, uint32_t val);
};