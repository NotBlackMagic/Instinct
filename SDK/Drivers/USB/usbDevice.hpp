/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/USB/usbDevice.hpp
 * Author:  NotBlackMagic
 * Brief:   USB Core Protocol driver.
 */

#pragma once

#include <stdint.h>

#include "usb.hpp"
#include "usbClass.hpp"
#include "status.hpp"

class USBDevice {
	public:
		/// @brief USB Standard Request Codes (Chapter 9).
		enum class Code : uint8_t {
			GetStatus = 0,
			ClearFeature = 1,
			SetFeature = 3,
			SetAddress = 5,
			GetDescriptor = 6,
			SetDescriptor = 7,
			GetConfiguration = 8,
			SetConfiguration = 9,
			GetInterface = 10,
			SetInterface = 11
		};

		/// @brief USB Descriptor Types.
		enum class DescriptorType : uint8_t {
			Device = 1,
			Config = 2,
			String = 3,
			Qualifier = 6
		};

		/// @brief USB Device States.
		enum class State : uint8_t {
			Default = 0,
			Addressed = 1,
			Configured = 2,
			Suspended = 3
		};

		/// @brief USB Device configuration structure.
		struct Config {
			uint16_t vid;
			uint16_t pid;
			uint16_t version;			// bcdDevice (e.g., 0x0100 for 1.00)
			const char* manufacturer;	// String Index 1
			const char* product;		// String Index 2
			const char* serialNumber;	// String Index 3
			uint8_t maxPower;			// Max power in mA
			bool selfPowered;
		};

		// Delete copy constructors.
		USBDevice(const USBDevice&) = delete;
		USBDevice& operator=(const USBDevice&) = delete;

		/// @brief Constructor.
		/// @param usb Reference to the low-level bus driver.
		USBDevice(USB& usb);

		/// @brief Initializes the USB device, and the underlying bus.
		/// @param config USB device configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config& config);

		/// @brief Starts the USB device by calling the USB peripheral Connect to enabling the pull up on D+.
		/// @return Status::Ok if was successful.
		Status Start();

		/// @brief Stops the USB device by calling the USB peripheral Disconnect to disable the pull up on D+.
		/// @return Status::Ok if was successful.
		Status Stop();

		/// @brief Register a new USB device class (e.g. USB CDC Class), use virtual/template class "usbClass.hpp".
		/// @param usbClass Pointer to USB class to register.
		/// @return Status::Ok if was successful.
		Status RegisterClass(USBClass* usbClass);

		/// @brief Returns the current USB device state.
		/// @return USB device state.
		State GetState() const { return this->state; }

	private:
		USB& bus;
		Config config;
		State state;

		bool isInitialized;
		uint8_t deviceAddress;
		
		// Memory for EP0 IN transmissions (Descriptors)
		__attribute__((aligned(32))) uint8_t ep0TxBuffer[512];
		uint8_t* ep0TxData;
		uint16_t ep0TxRemaining;

		// Support for multiple/composite device classes
		static const uint8_t maxClasses = 4;
		USBClass* classes[maxClasses];
		uint8_t classCount = 0;
		// Endpoint resource tracking
		uint8_t nextInterfaceID = 0;
		uint8_t nextInEp = 0x81;		// IN endpoints have bit 7 set
		uint8_t nextOutEp = 0x01;
		
		// Static bridge for the hardware callback
		static void EventCallbackWrapper(void* ctx, USB::Event event, uint8_t epAddr, uint32_t len);
		void EventHandler(USB::Event event, uint8_t epAddr, uint32_t len);

		// State Machine Parsers
		void HandleSetup(uint8_t epNum);
		void HandleTransferComplete(uint8_t epAddr, uint32_t len);
		
		// Chapter 9 Standard Requests
		void StandardDeviceRequest(const USB::SetupPacket& setup);
		void StandardInterfaceRequest(const USB::SetupPacket& setup);
		void StandardEndpointRequest(const USB::SetupPacket& setup);

		// Core Enumeration Methods
		void GetDescriptor(const USB::SetupPacket& setup);
		void SetAddress(const USB::SetupPacket& setup);
		void SetConfiguration(const USB::SetupPacket& setup);
		
		// Helper to format and send string descriptors
		void SendStringDescriptor(uint8_t index, uint16_t reqLength);

		// Control Endpoint 0 transfer helpers
		void ControlTransmit(const uint8_t* data, uint16_t len, uint16_t reqLength);
		void ControlACK();
		void ControlStall();
};