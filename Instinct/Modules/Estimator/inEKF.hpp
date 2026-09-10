/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Estimation/inEKF.hpp
 * Author:  NotBlackMagic
 * Brief:   Lie Group Invariant Extended Kalman Filter on SE_2(3) x R^6.
 */

#pragma once

#include "common.hpp"
#include "matrix.hpp"
#include "lieGroups.hpp"
#include "status.hpp"

class InEKF {
	public:
		// Delete copy constructors
		InEKF(const InEKF&) = delete;
		InEKF& operator=(const InEKF&) = delete;

		/// @brief Constructor.
		InEKF();

		/// @brief Initializes the Magnetometer reference. Should be called after EKF first stabiles with level drone. Updates the magRefNed with passed and oriented magnetometer readings.
		/// @param magInit The stable and level magnetometer readings.
		/// @return Status::Ok if initialization succeeded.
		Status InitializeMagReference(const float magInit[3]);

		/// @brief EKF prediction.
		/// @param accel		Accelerometer data used for prediction (m/s^2).
		/// @param gyro			Gyroscope data used for prediction (rad/s).
		/// @param timestampUs	Current iteration timestamp (us).
		/// @return Status::Ok if update succeeded.
		Status Predict(const float accel[3], const float gyro[3], uint64_t timestampUs);

		/// @brief EKF update using accelerometer data (gravity vector).
		/// @param accel Accelerometer data used for update (m/s^2).
		/// @return Status::Ok if update succeeded.
		Status UpdateGravity(const float accel[3]);

		/// @brief EKF update using magnetometer data (earth magnetic field vector).
		/// @param mag Magnetometer data used for update (Gauss).
		/// @return Status::Ok if update succeeded.
		Status UpdateMag(const float mag[3]);

		/// @brief EKF update using barometer data (atmospheric pressure).
		/// @param pressure Barometer data used for update (Pascal).
		/// @return Status::Ok if update succeeded.
		Status UpdateBaro(float pressure);

		/// @brief Gets the quaternion orientation estimate.
		/// @return Quaternion orientation estimate.
		Quaternion GetQuaternion() const;

		/// @brief Gets the velocity estimate.
		/// @return Velocity estimate.
		Vector3f GetVelocity() const;

		/// @brief Gets the position estimate.
		/// @return Position estimate.
		Vector3f GetPosition() const;

		/// @brief Gets the gyroscope bias estimate.
		/// @return Gyroscope bias estimate.
		Vector3f GetGyroBias() const;

		/// @brief Gets the accelerometer bias estimate.
		/// @return Accelerometer bias estimate.
		Vector3f GetAccelBias() const;
		
		/// @brief Gets the if EKF estimate has converged.
		/// @return Converged status.
		bool IsConverged() const { return isConverged; }

	private:
		// Manifold State Space
		Matrix3f R;		// Rotation Matrix (Orientation)
		Vector3fMat v;	// Velocity Column Vector
		Vector3fMat p;	// Position Column Vector
		
		// Sensor Biases
		Vector3fMat gyroBias;	// Gyroscope Biases
		Vector3fMat accelBias;	// Accelerometer Biases

		// Covariance Matrix
		Matrix3f P[5][5];

		// Filter Constants & Configurations
		uint64_t lastTimestampUs;
		bool isInitialized;
		bool isConverged;
		uint32_t sampleCount;

		// Fixed Local Earth Parameters (NED Coordinate Frame)
		const Vector3fMat gravityNed;
		Vector3fMat magRefNed;

		// Tuned Noise Covariances
		float gyroNoiseVar;
		float accelNoiseVar;
		float gyroBiasWalkVar;
		float accelBiasWalkVar;

		// Private Helper Functions
		void InitializeState(uint64_t timestampUs);
		void NormalizeRotation();
};