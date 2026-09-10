/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Math/rotations.hpp
 * Author:	NotBlackMagic
 * Brief:	Centralized 3D rotation and trigonometry utilities.
 */

#pragma once

#include <math.h>

#include "common.hpp"

#include "arm_math.h"

class Rotations {
	public:
		/// @brief Euler angles to quaternion conversion.
		/// @param roll		Roll/x-axis angle (rad).
		/// @param pitch	Pitch/y-axis angle (rad).
		/// @param yaw		Yaw/z-axis angle (rad).
		/// @return Quaternion
		static Quaternion EulerToQuaternion(float roll, float pitch, float yaw);

		/// @brief Quaternion to euler angles conversion.
		/// @param q Quaternion.
		/// @return Vector3 euler angles (rad).
		static Vector3f QuaternionToEuler(const Quaternion& q);
		
		/// @brief Single-axis Yaw (z-axis) conversion from quaternion.
		/// @param q Quaternion.
		/// @return Yaw/z-axis euler angle (rad).
		static float ExtractYawFromQuaternion(const Quaternion& q);
		
		/// @brief Wrap euler angles, keep in range -PI to PI.
		/// @param angle Euler angles (rad).
		/// @return Wrapped euler angle (rad).
		static float WrapAngle(float angle);

		/// @brief Apply a deadband, value forced to zero in this band.
		/// @param value	Value to apply deadband.
		/// @param deadband	Deadband to use.
		/// @return Value with deadband applied.
		static float ApplyDeadband(float value, float deadband);

		/// @brief Calculates the inverse (conjugate) of a normalized quaternion.
		/// @param q Unit quaternion.
		/// @return Inverse quaternion.
		static Quaternion Inverse(const Quaternion& q);

		/// @brief Multiplies two quaternions (q1 * q2).
		/// @param q1 Left quaternion.
		/// @param q2 Right quaternion.
		/// @return Resulting quaternion.
		static Quaternion Multiply(const Quaternion& q1, const Quaternion& q2);

	private:
		// Private constructor
		Rotations() = delete;
};