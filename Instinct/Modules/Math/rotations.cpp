/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Math/rotations.cpp
 */

#include "rotations.hpp"

float Rotations::ExtractYawFromQuaternion(const Quaternion& q) {
	// Aerospace ZYX Euler extraction for Yaw
	float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
	float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
	float atan2 = 0;
	arm_atan2_f32(siny_cosp, cosy_cosp, &atan2);
	return atan2;
}

Quaternion Rotations::EulerToQuaternion(float roll, float pitch, float yaw) {
	// Converts Roll, Pitch, Yaw (in radians) to a Quaternion
	float cy = arm_cos_f32(yaw * 0.5f);
	float sy = arm_sin_f32(yaw * 0.5f);
	float cp = arm_cos_f32(pitch * 0.5f);
	float sp = arm_sin_f32(pitch * 0.5f);
	float cr = arm_cos_f32(roll * 0.5f);
	float sr = arm_sin_f32(roll * 0.5f);

	Quaternion q;
	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;
	return q;
}

// Standard Aerospace ZYX Euler extraction from a Quaternion
Vector3f Rotations::QuaternionToEuler(const Quaternion& q) {
	Vector3f angles;

	// Roll (x-axis rotation)
	float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
	float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
	arm_atan2_f32(sinr_cosp, cosr_cosp, &angles.x);

	// Pitch (y-axis rotation)
	float sinp = 2.0f * (q.w * q.y - q.z * q.x);
	if(fabsf(sinp) >= 1.0f) {
		// Use 90 degrees if out of range to prevent NaN
		angles.y = copysignf(3.14159265f / 2.0f, sinp); 
	}
	else {
		angles.y = arm_sin_f32(sinp);
	}

	// Yaw (z-axis rotation)
	float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
	float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
	arm_atan2_f32(siny_cosp, cosy_cosp, &angles.z);

	return angles;
}

float Rotations::WrapAngle(float angle) {
	// Keeps the angle bound strictly between -PI and +PI
	while(angle > 3.1415926535f) {
		angle -= 6.283185307f;
	}
	while (angle < -3.1415926535f) {
		angle += 6.283185307f;
	}
	return angle;
}

float Rotations::ApplyDeadband(float value, float deadband) {
	if(fabsf(value) < deadband) {
		return 0.0f;
	}
	// Re-scale so that moving past deadband starts smoothly at 0.0
	float sign = (value > 0.0f) ? 1.0f : -1.0f;
	return sign * ((fabsf(value) - deadband) / (1.0f - deadband));
}

Quaternion Rotations::Inverse(const Quaternion& q) {
	// For a normalized unit quaternion, the inverse is just the conjugate.
	Quaternion res;
	res.w = q.w;
	res.x = -q.x;
	res.y = -q.y;
	res.z = -q.z;
	return res;
}

Quaternion Rotations::Multiply(const Quaternion& q1, const Quaternion& q2) {
	// Standard Hamilton product of two quaternions
	Quaternion res;
	res.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	res.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
	res.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
	res.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
	return res;
}