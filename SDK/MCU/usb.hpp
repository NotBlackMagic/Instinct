/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/usb.hpp
 * Author:  NotBlackMagic
 * Brief:   USB peripheral driver for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */

#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_gpio.h"
#include "stm32n6xx_ll_usb.h"

#include "status.hpp"
#include "system.hpp"

class USB {
	public:
		/// @brief Bus speed.
		enum class BusSpeed {
			Full = 0,	// USB-FS/USB 1.0 (12 Mbps)
			High = 1	// USB-HS/USB 2.0 (480 Mbps)
		};

		/// @brief USB endpoint type.
		enum class EndpointType {
			Control = 0,
			Isochronous = 1,
			Bulk = 2,
			Interrupt = 3
		};

		/// @brief USB event.
		enum class Event {
			Reset,
			Suspend,
			Resume,
			Sof,
			Setup,
			TransferComplete,
			Error
		};

		/// @brief Standard USB Setup Packet (8 bytes).
		struct SetupPacket {
			uint8_t requestType;
			uint8_t request;
			uint16_t value;
			uint16_t index;
			uint16_t length;
		};
		
		/// @brief USB peripheral configuration structure.
		struct Config {
			BusSpeed speed;
			bool vbusSensing;
			bool softOutput;
			bool useDMA;

			// Callbacks
			void (*eventCallback)(void* ctx, Event evt, uint8_t epAddr, uint32_t len);
			void* callbackContext;
		};

		// Delete copy constructors.
		USB(const USB&) = delete;
		USB& operator=(const USB&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., USB1_OTG_HS, USB2_OTG_HS).
		USB(USB_OTG_GlobalTypeDef* instance);

		/// @brief Initializes the peripheral clock, the peripheral itself, and interrupts.
		/// @param config USB configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config& config);

		/// @brief Completely resets the USB peripheral and aborts any pending transactions.
		/// @return Status::Ok if was successful.
		Status DeInit();

		/// @brief Connect the USB peripheral by enabling/adding the pull up on D+.
		/// @return Status::Ok if was successful.
		Status Connect();

		/// @brief Disconnects the USB peripheral by removing the pull up on D+.
		/// @return Status::Ok if was successful.
		Status Disconnect();
		
		/// @brief Sets the USB address.
		/// @return Status::Ok if was successful.
		Status SetAddress(uint8_t address);

		/// @brief Opens/adds a new USB endpoint.
		/// @param epAddr			Endpoint address/number.
		/// @param type				Endpoint type (Control, Bulk, etc...).
		/// @param maxPacketSize	Maximum packet size supported by this endpoint.
		/// @return Status::Ok if was successful, Status::Error if unsupported endpoint address/number.
		Status OpenEndpoint(uint8_t epAddr, EndpointType type, uint16_t maxPacketSize);

		/// @brief Closes a USB endpoint.
		/// @param epAddr Endpoint address/number.
		/// @return Status::Ok if was successful.
		Status CloseEndpoint(uint8_t epAddr);
		
		/// @brief Stalls the USB endpoint.
		/// @param epAddr Endpoint address/number.
		/// @return Status::Ok if was successful.
		Status StallEndpoint(uint8_t epAddr);

		/// @brief Clear a stall on the USB endpoint.
		/// @param epAddr Endpoint address/number.
		/// @return Status::Ok if was successful.
		Status ClearStall(uint8_t epAddr);

		/// @brief Checks if USB endpoint is stalled.
		/// @param epAddr Endpoint address/number.
		/// @return True if stalled, false otherwise.
		bool IsStalled(uint8_t epAddr);

		/// @brief Checks if USB endpoint is open.
		/// @param epAddr Endpoint address/number.
		/// @return True if open, false otherwise.
		bool IsEndpointOpen(uint8_t epAddr) const;

		/// @brief Starts a data write on a endpoint.
		/// @param epAddr	Endpoint address/number.
		/// @param buf		Pointer to the data to be transferred.
		/// @param len		Number of bytes to transfer.
		/// @return Status::Ok if was successful.
		Status Transmit(uint8_t epAddr, const uint8_t* buf, uint32_t len);

		/// @brief Setup a data read on a endpoint (Write).
		/// @param epAddr	Endpoint address/number.
		/// @param buf		Pointer to the receiver data buffer.
		/// @param len		Number of bytes to be received.
		/// @return Status::Ok if was successful.
		Status Receive(uint8_t epAddr, uint8_t* buf, uint32_t len);

		/// @brief Reads the 8-byte setup packet popped from the RX FIFO
		/// @return Setup packet struct.
		const SetupPacket* GetSetupPacket() const { return &this->setupPacket; }

		/// @brief Gets the configured USB bus speed.
		/// @return USB Bus Speed.
		USB::BusSpeed GetBusSpeed() const { 
			return this->config.speed;
		}

		/// @brief Start/prepare the EP0 for a new transfer.
		void StartEp0Setup();
		
		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler();

	private:
		USB_OTG_GlobalTypeDef* instance;
		USB_OTG_DeviceTypeDef* device;
		IRQn_Type irqCall;
		uint8_t irqPriority;
		uint32_t irqStatus;

		Config config;
		bool isInitialized;

		__attribute__((aligned(32))) uint8_t setupPacketBytes[32];
		SetupPacket setupPacket;

		// Support for multiple endpoints
		static const uint8_t maxUserInEP = 5;
		static const uint8_t maxUserOutEP = 8;
		static const uint8_t maxEP0PktSize = 64;
		static const uint16_t maxPktSize = 512;
		// OUT Endpoint data tracking
		struct EndpointState {
			uint8_t* buffer;
			uint32_t count;
		};
		EndpointState inEpState[6];
		EndpointState outEpState[9];

		/// @brief Helper to configure Resource Isolation Framework (RIF)
		void ConfigureRIF(void);

		void HandleEpInInterrupt();
		void HandleEpOutInterrupt();
		void HandleRxFifoInterrupt();
		void HandleTxFifoInterrupt(uint8_t epNum);
		
		Status FlushTxFifo(uint8_t epNum);
		Status FlushRxFifo();

		/// @brief Validates if an endpoint address is legal for user operations.
		inline bool IsValidEndpoint(uint8_t epAddr) const {
			uint8_t epNum = epAddr & 0x0F;
			bool isCmdIn = ((epAddr & 0x80) == 0x80);

			if(isCmdIn == true && epNum > maxUserInEP) {
				return false;
			}
			if(isCmdIn == false && epNum > maxUserOutEP) {
				return false;
			}
			return true;
		}
};