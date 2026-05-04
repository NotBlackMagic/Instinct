/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/USB/usbClass.hpp
 * Author:  NotBlackMagic
 * Brief:   Abstract USB Class interface.
 */

#pragma once

#include <stdint.h>

#include "usb.hpp"
#include "status.hpp"

class USBClass {
	public:
		virtual ~USBClass() = default;

		/// @brief Called when the device is configured (Set Configuration request).
		/// @param bus Passed reference of the low-level bus driver.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		virtual Status Init(USB& usb) = 0;
		
		/// @brief Called when the device is reset or unconfigured.
		/// @param bus Passed reference of the low-level bus driver.
		/// @return Status::Ok if was successful.
		virtual Status DeInit(USB& usb) = 0;

		/// @brief Called by the core device during registration to allocate interfaces and endpoints.
		/// @param nextInterfaceID	Reference to the device's running interface counter, value updated in this function.
		/// @param nextInEp			Reference to the device's running IN endpoint counter, value updated in this function.
		/// @param nextOutEp		Reference to the device's running OUT endpoint counter, value updated in this function.
		/// @return Status::Ok if was successful.
		virtual Status AssignResource(uint8_t& nextInterfaceID, uint8_t& nextInEp, uint8_t& nextOutEp) = 0;

		/// @brief Checks if this class instance manages the given endpoint address.
		/// @param epAddr Endpoint address/number.
		/// @return True if has endpoint address, False otherwise.
		virtual bool HasEndpoint(uint8_t epAddr) const = 0;

		/// @brief Checks if this class instance manages the given interface number.
		/// @param interfaceID Interface ID.
		/// @return True if is this interface ID address, False otherwise.
		virtual bool HasInterface(uint8_t interfaceID) const = 0;

		/// @brief Returns the Class-specific configuration descriptor appended to the standard config.
		/// @param speed	Passed USB Bus speed by the USB Device class.
		/// @param len		Pointer to be filled with the configuration size, number of bytes.
		/// @return Pointer to the configuration descriptor.
		virtual const uint8_t* GetConfigDescriptor(USB::BusSpeed speed, uint16_t* len) = 0;

		/// @brief Optional: Handles requests for custom string descriptors (e.g. MS OS 2.0).
		/// @param bus			Passed reference of the low-level bus driver.
		/// @param index		Device descriptor index.
		/// @param reqLength	Requested device descriptor length.
		/// @return True if has handle and custom string descriptor, False otherwise.
		virtual bool HandleStringDescriptor(USB& usb, uint8_t index, uint16_t reqLength) {
			(void)usb;
			(void)index;
			(void)reqLength;
			return false;
		}

		/// @brief Event handler for USB setup.
		/// @param bus		Passed reference of the low-level bus driver.
		/// @param setup	Passed setup package, received from the host.
		/// @return Status::Ok if was successful.
		virtual Status OnSetup(USB& usb, const USB::SetupPacket& setup) = 0;

		/// @brief Event handler for USB On Start of Frame.
		/// @param bus		Passed reference of the low-level bus driver.
		/// @return Status::Ok if was successful.
		virtual Status OnSoF(USB& usb) = 0;

		/// @brief Event handler for USB Data In (Bulk write/transfer finished callback).
		/// @param bus		Passed reference of the low-level bus driver.
		/// @param epNum	Endpoint number.
		/// @return Status::Ok if was successful.
		virtual Status OnDataIn(USB& usb, uint8_t epNum) = 0;

		/// @brief Event handler for USB Data Out (bulk receive callback).
		/// @param bus		Passed reference of the low-level bus driver.
		/// @param epNum	Endpoint number.
		/// @param len		Number of bytes received.
		/// @return Status::Ok if was successful.
		virtual Status OnDataOut(USB& usb, uint8_t epNum, uint32_t len) = 0;

		/// @brief Event handler for USB Error.
		/// @param bus		Passed reference of the low-level bus driver.
		/// @param epNum	Endpoint number.
		/// @return Status::Ok if was successful.
		virtual Status OnError(USB& usb, uint8_t epNum) = 0;

		/// @brief Optional: Called when the host clears an endpoint halt/stall condition.
		/// @param bus		Passed reference of the low-level bus driver.
		/// @param epAddr	Endpoint address/number.
		/// @return Status::Ok if was successful.
		virtual Status OnEndpointClear(USB& usb, uint8_t epAddr) {
			// Default empty implementation
			(void)usb;
			(void)epAddr;
			return Status::Ok;
		}

	protected:
		// Pointer to the active USB peripheral, accessible by all derived classes
		USB* bus = nullptr;
};
