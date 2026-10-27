/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Estimation/inEKF.hpp
 * Author:	NotBlackMagic
 * Brief:	Lie Group Invariant Extended Kalman Filter on SE_2(3) x R^6.
 */

#pragma once

#include "common.hpp"
#include "matrix.hpp"
#include "lieGroups.hpp"
#include "status.hpp"

class InEKF {
	public:
		/// @brief Invariant EKF configuration with conservative defaults.
		struct Config {
			// Initial state covariance diagonals (P0)
			float initAttitudeVar = 0.01f;	 	// rad^2
			float initVelocityVar = 0.1f;		// (m/s)^2
			float initPositionVar = 0.5f;		// m^2
			float initGyroBiasVar = 0.001f;		// (rad/s)^2
			float initAccelBiasVar = 0.01f;		// (m/s^2)^2

			// Process noise variances (Q)
			float gyroNoiseVar = 0.0001f;		// (rad/s)^2 / Hz
			float accelNoiseVar = 0.01f;		// (m/s^2)^2 / Hz
			float posNoiseVar = 1e-5f;			// m^2 / s (numerical floor)
			float gyroBiasWalkVar = 1e-6f;		// (rad/s^2)^2 / Hz
			float accelBiasWalkVar = 1e-5f;		// (m/s^3)^2 / Hz

			// Measurement noise variances (R)
			float gravityMeasVar = 0.5f;		// (m/s^2)^2
			float magMeasVar = 0.04f;			// Gauss^2
			float baroMeasVar = 0.25f;			// m^2
			float rangeMeasVar = 0.0009f;		// m^2
			float flowMeasVar = 0.02f;			// (m/s)^2 base variance
			float gpsHorizPosVar = 1.5f;		// m^2 (North/East)
			float gpsVertPosVar = 4.0f;			// m^2 (Down)
			float gpsVelVar = 0.05f;			// (m/s)^2

			// Operational gates and limits
			float maxDt = 0.1f;					// Max valid prediction step (s)
			float minRange = 0.05f;				// Min valid ToF range (m)
			float maxRange = 4.0f;				// Max valid ToF range (m)
			float minTiltCos = 0.5f;			// Max tilt (~60 deg) for range/flow
			uint32_t convergenceSamples = 8000; // Samples required for convergence
		};

		// Delete copy constructors
		InEKF(const InEKF&) = delete;
		InEKF& operator=(const InEKF&) = delete;

		/// @brief Default constructor using default Config values.
		InEKF();

		/// @brief Constructor with custom configuration.
		explicit InEKF(const Config& cfg);

		/// @brief Resets filter state and covariance using current Config.
		void Reset();

		void InitializeState(uint64_t timestamp);

		/// @brief Initializes the Magnetometer reference. Should be called after EKF first stabiles with level drone. Updates the magRefNed with passed and oriented magnetometer readings.
		/// @param magInit The stable and level magnetometer readings.
		/// @return Status::Ok if initialization succeeded.
		Status InitializeMagReference(const float magInit[3]);

		/// @brief Initializes the Barometer zero-height reference pressure. Used as the "0" for barometer based hight calculation/estimation
		/// @param baroInit Zero-height pressure (Pa)
		/// @return Status::Ok if initialization succeeded.
		Status InitializeBaroReference(float baroInit);

		/// @brief EKF prediction.
		/// @param accel		Accelerometer data used for prediction (m/s^2).
		/// @param gyro			Gyroscope data used for prediction (rad/s).
		/// @param timestamp	Current iteration timestamp (us).
		/// @return Status::Ok if update succeeded.
		Status Predict(const float accel[3], const float gyro[3], uint64_t timestamp);

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

		/// @brief EKF update using ranging data (distance from ground using down facing sensor).
		/// @param range Ranging data used for update (meters).
		/// @return Status::Ok if update succeeded.
		Status UpdateRange(float range);

		/// @brief EKF update using optical flow data (motion vector estimation using down facing camera).
		/// @param flowRate	Motion vector used for update ().
		/// @param gyro		Gyroscope data used for update, fused with flowRate (rad/s).
		/// @param range	Ranging data used for update, fused with flowRate (meters).
		/// @param quality	Quality/confidence value of the flowRate.
		/// @return Status::Ok if update succeeded.
		Status UpdateOpticalFlow(const float flowRate[2], const float gyro[3], float range, float quality);

		/// @brief EKF update using horizontal NED position from GPS [North, East] (meters).
		Status UpdateGPSPosition(const float position[3]);

		/// @brief EKF update using 3D NED velocity from GPS [vNorth, vEast, vDown] (m/s).
		Status UpdateGPSVelocity(const float velocity[3]);

		/// @brief Gets the quaternion orientation estimate.
		/// @return Quaternion orientation estimate.
		Quaternion GetQuaternion() const;

		/// @brief Gets the velocity estimate.
		/// @return Velocity estimate.
		Vector3f GetVelocity() const { return {v.data[0], v.data[1], v.data[2]}; }

		/// @brief Gets the position estimate.
		/// @return Position estimate.
		Vector3f GetPosition() const { return {p.data[0], p.data[1], p.data[2]}; }

		/// @brief Gets the gyroscope bias estimate.
		/// @return Gyroscope bias estimate.
		Vector3f GetGyroBias() const { return {gyroBias.data[0], gyroBias.data[1], gyroBias.data[2]}; }

		/// @brief Gets the accelerometer bias estimate.
		/// @return Accelerometer bias estimate.
		Vector3f GetAccelBias() const { return {accelBias.data[0], accelBias.data[1], accelBias.data[2]}; }

		/// @brief Gets the if EKF is initialized.
		/// @return Initialization status.
		bool IsInitialized() const { return isInitialized; }
		
		/// @brief Gets the if EKF estimate has converged.
		/// @return Converged status.
		bool IsConverged() const { return isConverged; }

	private:
		// Fixed physical & statistical constants (not tunable)
		static constexpr float gVal = 9.80665f;
		static constexpr float gSq = gVal * gVal;
		static constexpr float minCovDiag = 1e-6f;
		static constexpr float maxCovDiag = 100.0f;
		static constexpr float gate3Sigma1D = 9.0f;		// Chi-square 1 DOF (99.7%)
		static constexpr float gate3Sigma2D = 9.21f;	// Chi-square 2 DOF (99%)
		static constexpr float gate3Sigma3D = 11.3f;	// Chi-square 3 DOF (99%)

		Config config;

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
		uint64_t lastTimestamp;
		bool isInitialized;
		bool isConverged;
		uint32_t sampleCount;

		// Fixed Local Earth Parameters (NED Coordinate Frame)
		Vector3fMat magRefNed;
		float invBaroRef;

		// Tuned Noise Covariances
		float gyroNoiseCov;
		float accelNoiseCov;
		float gyroBiasWalkCov;
		float accelBiasWalkCov;

		// Private Helper Function
		void NormalizeRotation();
		void ApplyErrorState(const float delta[15]);
		Status UpdateVerticalPosition(float range, float rNoise);
		void MirrorCovarianceLower();
};