/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/USB/usbClassCDC.hpp
 * Author:  NotBlackMagic
 * Brief:   CDC ACM (Virtual Serial Port) Class Driver
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "usbClass.hpp"
#include "status.hpp"

class USBClassCDC : public USBClass {
	public:
		/// @brief Defines the USB CDC (Virtual COM) data width.
		enum class DataBits : uint8_t {
			DataBits_5 = 0,
			DataBits_6 = 1,
			DataBits_7 = 2,
			DataBits_8 = 3,
			DataBits_16 = 4
		};

		/// @brief Defines the USB CDC (Virtual COM) stop bits.
		enum class StopBits : uint8_t {
			StopBits_1 = 0,
			StopBits_15 = 1,
			StopBits_2 = 2
		};

		/// @brief Defines the USB CDC (Virtual COM) parity.
		enum class Parity : uint8_t {
			None = 0,
			Odd = 1,
			Even = 2,
			Mark = 3,
			Space = 4
		};

		/// @brief Standard CDC Line Coding Structure.
		struct LineCoding {
			uint32_t baudRate;		///< Baudrate.
			StopBits stopBits;		///< Stop bits mode.
			Parity parity;			///< Parity mode.
			DataBits dataBits;		///< Data width mode.
		};

		/// @brief CDC ACM Class-Specific Request Codes.
		enum CDCRequest {
			SetLineCoding = 0x20,
			GetLineCoding = 0x21,
			SetControlLineState = 0x22,
			SendBreak = 0x23
		};
		
		USBClassCDC();
		~USBClassCDC() override = default;

		/// @brief Initializes the USB CDC Class. Called when the device is configured.
		/// @param bus Passed reference of the low-level bus driver.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(USB& usb) override;

		/// @brief Removes the USB CDC Class endpoints.
		/// @param bus Passed reference of the low-level bus driver.
		/// @return Status::Ok if was successful.
		Status DeInit(USB& usb) override;

		/// @brief Called by the core device during registration to allocate interfaces and endpoints.
		/// @param nextInterfaceID	Reference to the device's running interface counter, value updated in this function.
		/// @param nextInEp			Reference to the device's running IN endpoint counter, value updated in this function.
		/// @param nextOutEp		Reference to the device's running OUT endpoint counter, value updated in this function.
		/// @return Status::Ok if was successful.
		Status AssignResource(uint8_t& nextInterfaceID, uint8_t& nextInEp, uint8_t& nextOutEp) override;
		
		/// @brief Checks if this class instance manages the given endpoint address.
		/// @param epAddr Endpoint address/number.
		/// @return True if has endpoint address, False otherwise.
		bool HasEndpoint(uint8_t epAddr) const override;

		/// @brief Checks if this interface ID corresponds to this class instance.
		/// @param interfaceID Interface ID.
		/// @return True if is this interface ID address, False otherwise.
		bool HasInterface(uint8_t interfaceID) const override;

		/// @brief Returns the USB Class configuration descriptor, used in the USB Device driver when asked by Host.
		/// @param speed	Passed USB Bus speed by the USB Device class.
		/// @param len		Pointer to be filled with the configuration size, number of bytes.
		/// @return Pointer to the configuration descriptor.
		const uint8_t* GetConfigDescriptor(USB::BusSpeed speed, uint16_t* len) override;

		/// @brief Event handler for USB setup.
		/// @param bus		Passed reference of the low-level bus driver.
		/// @param setup	Passed setup package, received from the host.
		/// @return Status::Ok if was successful.
		Status OnSetup(USB& usb, const USB::SetupPacket& setup) override;

		/// @brief Event handler for USB Data In (Bulk write/transfer finished callback).
		/// @param bus		Passed reference of the low-level bus driver.
		/// @param epNum	Endpoint number.
		/// @return Status::Ok if was successful.
		Status OnDataIn(USB& usb, uint8_t epNum) override;

		/// @brief Event handler for USB Data Out (bulk receive callback).
		/// @param bus		Passed reference of the low-level bus driver.
		/// @param epNum	Endpoint number.
		/// @param len		Number of bytes received.
		/// @return Status::Ok if was successful.
		Status OnDataOut(USB& usb, uint8_t epNum, uint32_t len) override;

		/// @brief Writes data to the internal ring buffer.
		/// @param data	Pointer to data to write (will be copied to ring buffer so does not need to be kept after function call).
		/// @param len	Number of bytes to write.
		/// @return Status::Ok if the transfer started, Status::Error if no USB device.
		Status Write(const uint8_t* buf, uint32_t len);

		/// @brief Read data from the internal ring buffer.
		/// @param data	Pointer to buffer for read data.
		/// @param len	Maximum number of bytes to read.
		/// @return Number of bytes read.
		uint32_t Read(uint8_t* buf, uint32_t maxLen);

		/// @brief Get number of bytes available to read from internal ring buffer.
		/// @return Number of bytes read.
		uint32_t Available();
		
	private:
		// Dynamically assigned resources
		uint8_t commInterface;
		uint8_t dataInterface;
		uint8_t intInEp;
		uint8_t bulkInEp;
		uint8_t bulkOutEp;

		// Device state
		// Minimum EP0 size that can be used is 64 bytes, ensure enough buffer size to no overwrite next variables in memory (CRITICAl in DMA mode)
		__attribute__((aligned(32))) uint8_t lineCodingBytes[64];
		LineCoding lineCoding;

		// Store the negotiated packet size for runtime use
		uint16_t bulkMaxPacketSize;

		// Transaction Context
		static constexpr uint32_t maxPacketSize = 512;
		static constexpr uint32_t rxBufferSize = 2048; 
		static constexpr uint32_t txBufferSize = 2048;

		__attribute__((aligned(32))) uint8_t epOutBuffer[maxPacketSize];
		__attribute__((aligned(32))) uint8_t rxBuffer[rxBufferSize];
		volatile uint32_t rxBufHead; 
		volatile uint32_t rxBufTail;

		__attribute__((aligned(32))) uint8_t epInBuffer[maxPacketSize];
		__attribute__((aligned(32))) uint8_t txBuffer[txBufferSize];
		volatile uint32_t txBufHead; 
		volatile uint32_t txBufTail; 
		volatile bool txBusy;

		// Internal helper to push data to hardware
		void StartTX();
		void PackLineCoding();
		void UnpackLineCoding();
};