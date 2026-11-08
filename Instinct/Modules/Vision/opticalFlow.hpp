/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Vision/opticalFlow.hpp
 * Author:	NotBlackMagic
 * Brief:	High-performance Gyro-Aided Pyramidal IC-LK Optical Flow.
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "common.hpp"
#include "status.hpp"
#include "visionTypes.hpp"

#include "arm_math.h"
#include "arm_mve.h"

class OpticalFlow {
	public:
		/// @brief Frame processing and tracking health statistics.
		struct Statistics {
			uint32_t processedFrames = 0;
			uint32_t validFrames = 0;
			uint32_t rejectedFrames = 0;
			uint32_t lastTrackPatches = 0;
		};

		/// @brief Optical flow solver and camera intrinsic configuration.
		struct Config {
			float fx = 143.7f;		///< Horizontal focal length in Level-0 (128x128) pixels
			float fy = 143.7f;		///< Vertical focal length in Level-0 (128x128) pixels
			float minEig = 25.0f;	///< Min Shi-Tomasi eigenvalue to accept a patch
			float maxRes = 25.0f;	///< Max mean patch residual after convergence
			float epsSq = 0.0004f;	///< Convergence step threshold squared (px^2)
			uint8_t minValid = 5;	///< Min inlier patches required for valid output
		};

		/// @brief Instantaneous optical flow measurement output.
		struct Output {
			Vector3f flowRate;		///< Total optical angular rate [wx, wy, 0] (rad/s)
			Vector3f bodyRate;		///< Derotated translational rate (flowRate - gyro) (rad/s)
			Vector3f pixelShift;	///< Inlier mean shift [dx, dy, 0] in L0 pixels
			float quality;			///< Normalized confidence [0.0f to 1.0f]
			uint8_t validCount;		///< Number of inlier patches
			bool isValid;			///< True if validCount >= config.minValid
		};

		// Delete copy constructors
		OpticalFlow(const OpticalFlow&) = delete;
		OpticalFlow& operator=(const OpticalFlow&) = delete;

		/// @brief Constructor.
		OpticalFlow();

		/// @brief Initializes the optical flow solver with camera intrinsics and thresholds.
		/// @param config Solver configuration structure.
		/// @return Status::Ok if initialization succeeded, or Status::Error if invalid.
		Status Init(const Config& config);

		/// @brief Resets pyramid history and output state.
		void Reset();

		/// @brief Processes a new camera frame and computes optical flow rates.
		/// @param frame	Input VisionFrame (expects 256x256 Grayscale in internal SRAM).
		/// @param gyro		Unbiased body angular velocity [wx, wy, wz] (rad/s).
		/// @return Status::Ok if processed, or Status::Error if frame format/size is invalid.
		Status Process(const VisionFrame& frame, const float gyro[3]);

		/// @brief Returns the latest optical flow measurement.
		const Output& GetOutput() const { return this->output; }

		/// @brief Returns true if the last processed frame produced a valid flow vector.
		bool IsValid() const { return this->output.isValid; }

		/// @brief Returns optical flow processing statistics.
		Statistics GetStatistics() const { return this->stats; }

		/// @brief Resets optical flow statistics counters.
		void ResetStatistics() { this->stats = {}; }

	private:
		// Resolution and grid constants
		static constexpr uint16_t rawWidth = 256;
		static constexpr uint16_t rawHeight = 256;
		static constexpr uint16_t l0Width = 128;
		static constexpr uint16_t l0Height = 128;
		static constexpr uint16_t l1Width = 64;
		static constexpr uint16_t l1Height = 64;
		static constexpr uint16_t l2Width = 32;
		static constexpr uint16_t l2Height = 32;

		static constexpr uint8_t patchSize = 8;
		static constexpr uint8_t gridRows = 5;
		static constexpr uint8_t gridCols = 5;
		static constexpr uint8_t numPatches = gridRows * gridCols;
		static constexpr uint8_t maxIter = 4;

		// Pyramid buffer: 21 KB per buffer (Fits completely inside the 32 KB STM32N6 L1 D-Cache)
		struct Pyramid {
			__attribute__((aligned(32))) uint8_t l0[l0Width * l0Height];	// 16'384 bytes
			__attribute__((aligned(32))) uint8_t l1[l1Width * l1Height];	// 4'096 bytes
			__attribute__((aligned(32))) uint8_t l2[l2Width * l2Height];	// 1'024 bytes
		};

		struct Point2u8 {
			uint8_t x;
			uint8_t y;
		};

		// Ping-pong pyramid buffers
		Pyramid pyramids[2];
		uint8_t pyrIdx;
		bool isInitialized;
		bool hasPrev;
		uint64_t timestamp;

		// Precomputed L0 patch centers
		Point2u8 grid[numPatches];

		// Configuration & cached reciprocals
		Config config;
		float invFocalX;
		float invFocalY;

		// State & diagnostics
		Output output = {};
		Statistics stats = {};

		// Internal helpers
		void BuildPyramid(const uint8_t* src, Pyramid& dst);
		static void Bin2x2(const uint8_t* src, uint8_t* dst, uint16_t srcW, uint16_t srcH);
		bool TrackPatch(const uint8_t* prev, const uint8_t* curr, uint16_t stride, float cx, float cy, float& u, float& v) const;
		void SolveVelocity(const float* uArr, const float* vArr, uint8_t count, float dt, const float gyro[3]);
		static float Median(float* buf, uint8_t count);
};