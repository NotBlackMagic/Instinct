/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	SDK/MCU/usbpd.hpp
 * Author:	NotBlackMagic
 * Brief:	USB-PD driver class for STM32N6 using LL (Low-Layer) API and direct register access (NO HALL).
 */
 
#pragma once

#include <stdint.h>

#include "stm32n657xx.h"
#include "stm32n6xx_ll_ucpd.h"

#include "logger.hpp"
#include "status.hpp"
#include "system.hpp"

#include "tx_api.h"

class USBPD {
	public:
		/// @brief USB-PD CC line definitions.
		enum class CCLine : uint8_t {
			None = 0x00,
			CC1 = 0x01,
			CC2 = 0x02
		};

		/// @brief USB-PD power profile types.
		enum class PowerType : uint8_t {
			Fixed = 0,
			Variable,
			Battery,
			PPS
		};

		/// @brief USB-PD events.
		enum class Event : uint8_t {
			Attached = 0,
			Detached,
			ProfileReceived,
			ProfileReady,
			ProfileRejected,
			Error
		};

		/// @brief USB-PD profiles structure.
		struct Profile {
			float voltage = 0.0f;				///< Profile nominal or maximum voltage, in V
			float minVoltage = 0.0f;			///< Profile minimum voltage (for PPS/Variable; equals voltage for Fixed), in V
			float current = 0.0f;				///< Profile maximum current, in A
			PowerType type = PowerType::Fixed;	///< Profile power type
		};

		/// @brief USB-PD peripheral configuration structure.
		struct Config {
			float targetVoltage = 5.0f;	///< Target voltage to auto-negotiate, in V
			float targetCurrent = 0.5f;	///< Current budget needed, in A
			bool autoNegotiate = false;	///< Request best profile immediately when caps arrive
			
			// Callbacks
			void (*EventCallback)(void *ctx, Event evt) = nullptr;
			void *callbackContext = nullptr;
		};
		
		// Delete copy constructors.
		USBPD(const USBPD&) = delete;
		USBPD& operator=(const USBPD&) = delete;

		/// @brief Constructor.
		/// @param instance Pointer to the hardware instance (e.g., UCPD1).
		USBPD(UCPD_TypeDef* instance);

		/// @brief Initializes the peripheral clock, the peripheral itself, and interrupts.
		/// @param config USB-PD configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(const Config &config);

		/// @brief Completely resets and disables the USB-PD peripheral.
		/// @return Status::Ok if was successful.
		Status DeInit();

		/// @brief Checks whether a USB Type-C port partner is currently attached.
		/// @return True if a partner is connected on either CC line, false if disconnected.
		bool IsAttached() const { return this->isAttached; };

		/// @brief Gets the active CC pin detected during cable orientation resolution.
		/// @return CCLine::CC1, CCLine::CC2, or CCLine::None if detached.
		CCLine GetActiveCCLine() const { return this->activeCCLine; };

		/// @brief Gets the number of valid Power Data Objects (PDOs) received from the source.
		/// @return Total number of advertised profiles (0 if detached or connected to a non-PD source).
		uint8_t GetProfileCount() const { return this->profileCount; };

		/// @brief Returns a pointer to the array of advertised source profiles.
		/// @return Pointer to an array of up to 7 Profile structures.
		const Profile* GetProfiles() const { return this->profiles; };

		/// @brief Gets the currently active negotiated or legacy power contract.
		/// @return Reference to active Profile (defaults to 5.0 V @ 0.5 A when non-PD or unnegotiated).
		const Profile& GetActiveProfile() const { return this->activeProfile; };

		/// @brief Requests a specific advertised power profile by index.
		/// @param profileIndex	1-based index corresponding to advertised PDOs (1 to GetProfileCount()).
		/// @param current		Operating current requested, in A (clamped to profile maximum).
		/// @return Status::Ok if accepted
		Status RequestProfile(uint8_t profileIndex, float current);

		/// @brief Finds the best matching advertised profile for a target voltage and requests it.
		/// @param voltage	Desired operating voltage, in V.
		/// @param current	Desired operating current, in A.
		/// @return Status::Ok if negotiated correctly, Status::Error if no matching profile exists.
		Status RequestVoltage(float voltage, float current);

		/// @brief Triggers a USB-PD Hard Reset sequence on the active CC communication line.
		/// @return Status::Ok if signaled successfully, Status::Error if detached.
		Status Reset();

		/// @brief Interrupt Service Routine handler.
		/// @warning This function is called by the NVIC. Do not call manually.
		void InterruptHandler();

	private:
		UCPD_TypeDef *instance;
		IRQn_Type irqCall;
		uint32_t irqPriority;

		Config config;
		bool isInitialized;
		bool isAttached;
		CCLine activeCCLine;

		// Received Capabilities & Active Contract
		static constexpr uint8_t maxProfiles = 7;	// Maximum PDOs per USB-PD spec
		Profile profiles[maxProfiles];
		uint8_t profileCount;
		Profile activeProfile;
		uint8_t activePDOIndex;

		// Transaction Context
		static constexpr uint16_t rxBufferSize = 64;	// Max PD 3.0 frame: 2-byte header + 7 * 4-byte PDOs
		static constexpr uint16_t txBufferSize = 64;
		uint8_t rxMsgID;
		uint8_t txMsgID;
		__attribute__((aligned(32))) uint8_t rxBuffer[rxBufferSize];
		__attribute__((aligned(32))) uint8_t txBuffer[txBufferSize];

		// Synchronization
		TX_MUTEX mutex;
		TX_EVENT_FLAGS_GROUP event;

		// Event Flags Definitions
		static constexpr ULONG EVT_RX_CAPS = 0x01;
		static constexpr ULONG EVT_READY = 0x02;
		static constexpr ULONG EVT_REJECT = 0x04;
		static constexpr ULONG EVT_ERR = 0x08;

		// Timeout defines
		static constexpr uint32_t TIMEOUT_BUSY_US = 100;
		static constexpr uint32_t TIMEOUT_MUTEX = 50;

		// Applies 5.1 kOhm pull-down terminations and activates Type-C level comparators.
		void AssertRDTerminations();
		// Reads analog CC line comparators to resolve cable presence, orientation, and current limits.
		void DetectCCOrientation();
		// Immediately constructs and transmits a GoodCRC control frame in response to received packets.
		void SendGoodCRC(uint8_t msgID);
		// Packs header and data objects into txBuffer and initiates transmission via UCPD.
		Status SendMessage(uint8_t msgType, const uint32_t *payload, uint8_t numObjects);
		// Decodes raw 32-bit Source Capability PDOs into float-based Profile entries.
		void ParseSourceCapabilities(uint8_t numObjects);
};