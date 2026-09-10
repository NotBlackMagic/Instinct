/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Estimation/sensorHubThread.hpp
 * Author:	NotBlackMagic
 * Brief:	Pre-fusion DSP layer. Aggregates raw sensors, handles hardware failovers, phase-aligns timestamps, and publishes Virtual Sensors.
 */

#pragma once

#include "hardware.hpp"

#include "logger.hpp"
#include "pubSub.hpp"

#include "tx_api.h"

class SensorHubThread {
	public:
		static void Init();

	private:
		static TX_THREAD threadPtr;
		static uint8_t threadStack[8192];

		// Hardware Redundancy Configuration
		static constexpr uint8_t maxIMUs = 3;	// 1 Onboard, 2 Offboard
		static constexpr uint8_t maxMags = 2;	// 1 Onboard, 1 Offboard
		static constexpr uint8_t maxBaros = 2;	// 1 Onboard, 1 Offboard

		// Virtual instance ID, to be used by estimator etc...
		static constexpr uint8_t virtualInstanceID = 255;

		// Sensor Subscribers
		static Subscriber<IMUMsg> subIMU[maxIMUs];
		static Subscriber<MagMsg> subMag[maxMags];
		static Subscriber<BaroMsg> subBaro[maxBaros];

		// Subscriber Topic Pointers
		static Topic<IMUMsg>* topicsIMU[maxIMUs];
		static Topic<MagMsg>* topicsMag[maxMags];
		static Topic<BaroMsg>* topicsBaro[maxBaros];

		// Virtual Sensor Publishers
		static Topic<IMUMsg> topicVirtualIMU;
		static Topic<MagMsg> topicVirtualMag;
		static Topic<BaroMsg> topicVirtualBaro;

		// Active sensor tracking
		static uint8_t activeIMUCount;
		static uint8_t activeMagCount;
		static uint8_t activeBaroCount;

		// Health monitoring flags (true = healthy, false = timeout/fault)
		static bool imuHealth[maxIMUs];
		static bool magHealth[maxMags];
		static bool baroHealth[maxBaros];

		// Define the physical locations of IMUs relative to the CoG
		static constexpr Vector3f imuOffsetCoG[maxIMUs] = {
			{0.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 0.0f}
		};

		// IMU state variables for the dynamic variance filter
		static constexpr float alpha = 0.05f; // Tuning parameter for how fast it reacts to new noise
		static uint64_t lastIMUTime[maxIMUs];
		static float imuVariance[maxIMUs][3];
		static float imuMean[maxIMUs][3];

		// Barometer state variables and selection parameters
		static constexpr float maxPressRate = 500.0f;	// Roughly 40 m/s vertical climb rate at sea level.
		static constexpr float baroAgreeThres = 50.0f;	// Max allowed difference between two barometers before we stop averaging them (~4 meters)
		static float lastPressure[maxBaros];
		static uint64_t lastBaroTimeUs[maxBaros];

		// Magnetometer state variables and selection parameters
		static constexpr float referenceMagNorm = 0.5f;	//Earth's magnetic field. In Gauss it is ~0.45 to 0.65 depending on latitude.

		static void Run(ULONG input);

		static IMUMsg AggregateIMUs(IMUMsg* rawMsgs, uint8_t count);
		static BaroMsg AggregateBaros(BaroMsg* rawMsgs, uint8_t count);
		static MagMsg SelectMag(MagMsg* rawMsgs, uint8_t count);
		static bool CheckHealth(uint64_t lastTime, uint64_t refTime, uint32_t timeout);

		static void CrossProduct(const float a[3], const float b[3], float out[3]);
};