/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:	Instinct/Modules/Math/matrix.hpp
 * Author:	NotBlackMagic
 * Brief:	Centralized matrix math utilities, some with ARM CMSIS and hardware accelerated.
 */

#pragma once

#include "arm_math.h"
#include <string.h>

enum class InitType { Uninitialized };

template <uint16_t Rows, uint16_t Cols>
struct Matrix {
	alignas(16) float data[Rows * Cols];
	arm_matrix_instance_f32 instance;

	// Constructor
	Matrix() {
		memset(data, 0, sizeof(data));
		instance = {Rows, Cols, data};
	}

	Matrix(InitType) {
		instance = {Rows, Cols, data};
	}

	Matrix(const Matrix& other) {
		memcpy(data, other.data, sizeof(data));
		instance = {Rows, Cols, data};
	}

	// Hardware unrolled fast math for small matrixes (Faster then CMSIS-DSP math)
	static void Multiply3x3(const Matrix<3, 3>& a, const Matrix<3, 3>& b, Matrix<3, 3>& result) {
		result.data[0] = a.data[0]*b.data[0] + a.data[1]*b.data[3] + a.data[2]*b.data[6];
		result.data[1] = a.data[0]*b.data[1] + a.data[1]*b.data[4] + a.data[2]*b.data[7];
		result.data[2] = a.data[0]*b.data[2] + a.data[1]*b.data[5] + a.data[2]*b.data[8];
		result.data[3] = a.data[3]*b.data[0] + a.data[4]*b.data[3] + a.data[5]*b.data[6];
		result.data[4] = a.data[3]*b.data[1] + a.data[4]*b.data[4] + a.data[5]*b.data[7];
		result.data[5] = a.data[3]*b.data[2] + a.data[4]*b.data[5] + a.data[5]*b.data[8];
		result.data[6] = a.data[6]*b.data[0] + a.data[7]*b.data[3] + a.data[8]*b.data[6];
		result.data[7] = a.data[6]*b.data[1] + a.data[7]*b.data[4] + a.data[8]*b.data[7];
		result.data[8] = a.data[6]*b.data[2] + a.data[7]*b.data[5] + a.data[8]*b.data[8];
	}

	// Hardware-accelerated Fused Multiply-Accumulate: result += a * b
	static void MultiplyAccumulate3x3(const Matrix<3, 3>& a, const Matrix<3, 3>& b, Matrix<3, 3>& result) {
		result.data[0] += a.data[0]*b.data[0] + a.data[1]*b.data[3] + a.data[2]*b.data[6];
		result.data[1] += a.data[0]*b.data[1] + a.data[1]*b.data[4] + a.data[2]*b.data[7];
		result.data[2] += a.data[0]*b.data[2] + a.data[1]*b.data[5] + a.data[2]*b.data[8];
		
		result.data[3] += a.data[3]*b.data[0] + a.data[4]*b.data[3] + a.data[5]*b.data[6];
		result.data[4] += a.data[3]*b.data[1] + a.data[4]*b.data[4] + a.data[5]*b.data[7];
		result.data[5] += a.data[3]*b.data[2] + a.data[4]*b.data[5] + a.data[5]*b.data[8];
		
		result.data[6] += a.data[6]*b.data[0] + a.data[7]*b.data[3] + a.data[8]*b.data[6];
		result.data[7] += a.data[6]*b.data[1] + a.data[7]*b.data[4] + a.data[8]*b.data[7];
		result.data[8] += a.data[6]*b.data[2] + a.data[7]*b.data[5] + a.data[8]*b.data[8];
	}

	// Hardware-accelerated Fused Multiply-Subtract: result -= a * b
	static void MultiplySubtract3x3(const Matrix<3, 3>& a, const Matrix<3, 3>& b, Matrix<3, 3>& result) {
		result.data[0] -= a.data[0]*b.data[0] + a.data[1]*b.data[3] + a.data[2]*b.data[6];
		result.data[1] -= a.data[0]*b.data[1] + a.data[1]*b.data[4] + a.data[2]*b.data[7];
		result.data[2] -= a.data[0]*b.data[2] + a.data[1]*b.data[5] + a.data[2]*b.data[8];
		
		result.data[3] -= a.data[3]*b.data[0] + a.data[4]*b.data[3] + a.data[5]*b.data[6];
		result.data[4] -= a.data[3]*b.data[1] + a.data[4]*b.data[4] + a.data[5]*b.data[7];
		result.data[5] -= a.data[3]*b.data[2] + a.data[4]*b.data[5] + a.data[5]*b.data[8];
		
		result.data[6] -= a.data[6]*b.data[0] + a.data[7]*b.data[3] + a.data[8]*b.data[6];
		result.data[7] -= a.data[6]*b.data[1] + a.data[7]*b.data[4] + a.data[8]*b.data[7];
		result.data[8] -= a.data[6]*b.data[2] + a.data[7]*b.data[5] + a.data[8]*b.data[8];
	}

	static void Multiply5x5(const Matrix<5, 5>& a, const Matrix<5, 5>& b, Matrix<5, 5>& result) {
		for(int r = 0; r < 5; r++) {
			for(int c = 0; c < 5; c++) {
				result.data[r * 5 + c] =	a.data[r * 5 + 0] * b.data[0 * 5 + c] +
											a.data[r * 5 + 1] * b.data[1 * 5 + c] +
											a.data[r * 5 + 2] * b.data[2 * 5 + c] +
											a.data[r * 5 + 3] * b.data[3 * 5 + c] +
											a.data[r * 5 + 4] * b.data[4 * 5 + c];
			}
		}
	}

	static void Multiply3x3Vector3(const Matrix<3, 3>& m, const Matrix<3, 1>& v, Matrix<3, 1>& res) {
		res.data[0] = m.data[0]*v.data[0] + m.data[1]*v.data[1] + m.data[2]*v.data[2];
		res.data[1] = m.data[3]*v.data[0] + m.data[4]*v.data[1] + m.data[5]*v.data[2];
		res.data[2] = m.data[6]*v.data[0] + m.data[7]*v.data[1] + m.data[8]*v.data[2];
	}

	static void MultiplyTransposed3x3Vector3(const Matrix<3, 3>& m, const Matrix<3, 1>& v, Matrix<3, 1>& res) {
		res.data[0] = m.data[0]*v.data[0] + m.data[3]*v.data[1] + m.data[6]*v.data[2];
		res.data[1] = m.data[1]*v.data[0] + m.data[4]*v.data[1] + m.data[7]*v.data[2];
		res.data[2] = m.data[2]*v.data[0] + m.data[5]*v.data[1] + m.data[8]*v.data[2];
	}

	static void Transpose3x3(const Matrix<3, 3>& src, Matrix<3, 3>& dst) {
		dst.data[0] = src.data[0]; dst.data[1] = src.data[3]; dst.data[2] = src.data[6];
		dst.data[3] = src.data[1]; dst.data[4] = src.data[4]; dst.data[5] = src.data[7];
		dst.data[6] = src.data[2]; dst.data[7] = src.data[5]; dst.data[8] = src.data[8];
	}

	static void Add3x3(const Matrix<3, 3>& a, const Matrix<3, 3>& b, Matrix<3, 3>& result) {
		for(int i = 0; i < 9; i++) {
			result.data[i] = a.data[i] + b.data[i];
		}
	}

	static void MultiplyScalar3x3(const Matrix<3, 3>& m, float scalar, Matrix<3, 3>& res) {
		for(int i = 0; i < 9; i++) {
			res.data[i] = m.data[i] * scalar;
		}
	}

	static void MultiplyScalarVector3(const Matrix<3, 1>& v, float scalar, Matrix<3, 1>& res) {
		res.data[0] = v.data[0] * scalar;
		res.data[1] = v.data[1] * scalar;
		res.data[2] = v.data[2] * scalar;
	}

	Matrix<3, 3> Inverse3x3() const {
		static_assert(Rows == 3 && Cols == 3, "Analytical inverse strictly requires a 3x3 matrix.");
		Matrix<3, 3> result(InitType::Uninitialized);
		
		float det = data[0] * (data[4] * data[8] - data[5] * data[7]) -
					data[1] * (data[3] * data[8] - data[5] * data[6]) +
					data[2] * (data[3] * data[7] - data[4] * data[6]);
					
		float invDet = 1.0f / det;

		// Adjugate multiplied by 1/Determinant
		result.data[0] = (data[4] * data[8] - data[5] * data[7]) * invDet;
		result.data[1] = (data[2] * data[7] - data[1] * data[8]) * invDet;
		result.data[2] = (data[1] * data[5] - data[2] * data[4]) * invDet;
		result.data[3] = (data[5] * data[6] - data[3] * data[8]) * invDet;
		result.data[4] = (data[0] * data[8] - data[2] * data[6]) * invDet;
		result.data[5] = (data[2] * data[3] - data[0] * data[5]) * invDet;
		result.data[6] = (data[3] * data[7] - data[4] * data[6]) * invDet;
		result.data[7] = (data[1] * data[6] - data[0] * data[7]) * invDet;
		result.data[8] = (data[0] * data[4] - data[1] * data[3]) * invDet;

		return result;
	}

	// Generic math (Auto-vectorized by compiler)
	static void Add(const Matrix<Rows, Cols>& a, const Matrix<Rows, Cols>& b, Matrix<Rows, Cols>& result) {
		// arm_mat_add_f32(&a.instance, &b.instance, &result.instance);
		for(uint16_t i = 0; i < Rows * Cols; i++) result.data[i] = a.data[i] + b.data[i];
	}

	static void Subtract(const Matrix<Rows, Cols>& a, const Matrix<Rows, Cols>& b, Matrix<Rows, Cols>& result) {
		// arm_mat_sub_f32(&a.instance, &b.instance, &result.instance);
		for(uint16_t i = 0; i < Rows * Cols; i++) result.data[i] = a.data[i] - b.data[i];
	}

	void SetIdentity() {
		memset(data, 0, sizeof(data));
		uint16_t minDim = (Rows < Cols) ? Rows : Cols;
		for(uint16_t i = 0; i < minDim; i++) {
			data[i * Cols + i] = 1.0f;
		}
	}

	// CMSIS-DSP wrappers (For Large Matrix Multiplication and Inversion)
	template <uint16_t ResCols>
	static void Multiply(const Matrix<Rows, Cols>& a, const Matrix<Cols, ResCols>& b, Matrix<Rows, ResCols>& result) {
		arm_mat_mult_f32(&a.instance, &b.instance, &result.instance);
	}

	static void Transpose(const Matrix<Rows, Cols>& src, Matrix<Cols, Rows>& dst) {
		arm_mat_trans_f32(&src.instance, &dst.instance);
	}

	// Hardware-Accelerated Matrix Inversion (Crucial for the Kalman Gain)
	Matrix<Rows, Cols> Inverse() const {
		static_assert(Rows == Cols, "Inverse only exists for square matrices.");
		Matrix<Rows, Cols> result(InitType::Uninitialized);
		arm_mat_inverse_f32(&this->instance, &result.instance);
		return result;
	}

	// Hardware-Accelerated Transpose
	Matrix<Cols, Rows> Transpose() const {
		Matrix<Cols, Rows> result(InitType::Uninitialized);
		arm_mat_trans_f32(&this->instance, &result.instance);
		return result;
	}

	// Operator overloads
	// Copy Assignment Operator
	Matrix& operator=(const Matrix& other) {
		if(this != &other) {
			memcpy(data, other.data, sizeof(data));
		}
		return *this;
	}

	// Addition Overload
	Matrix<Rows, Cols> operator+(const Matrix<Rows, Cols>& other) const {
		Matrix<Rows, Cols> result(InitType::Uninitialized);
		// arm_mat_add_f32(&this->instance, &other.instance, &result.instance);
		for(uint16_t i = 0; i < Rows * Cols; i++) result.data[i] = data[i] + other.data[i];
		return result;
	}

	// Subtraction Overload
	Matrix<Rows, Cols> operator-(const Matrix<Rows, Cols>& other) const {
		Matrix<Rows, Cols> result(InitType::Uninitialized);
		// arm_mat_sub_f32(&this->instance, &other.instance, &result.instance);
		for(uint16_t i = 0; i < Rows * Cols; i++) result.data[i] = data[i] - other.data[i];
		return result;
	}

	// Multiplication Overload (Handles inner dimension matching automatically)
	template <uint16_t ResCols>
	Matrix<Rows, ResCols> operator*(const Matrix<Cols, ResCols>& other) const {
		Matrix<Rows, ResCols> result(InitType::Uninitialized);
		arm_mat_mult_f32(&this->instance, &other.instance, &result.instance);
		return result;
	}

	// Scalar Multiplication
	Matrix<Rows, Cols> operator*(float scalar) const {
		Matrix<Rows, Cols> result(InitType::Uninitialized);
		// arm_mat_scale_f32(&this->instance, scalar, &result.instance);
		for(uint16_t i = 0; i < Rows * Cols; i++) result.data[i] = data[i] * scalar;
		return result;
	}
};

// Common Typedefs
using Matrix3f = Matrix<3, 3>;
using Matrix5f = Matrix<5, 5>;
using Matrix9f = Matrix<9, 9>;
using Matrix15f = Matrix<15, 15>;

// Column vectors
using Vector3fMat = Matrix<3, 1>;
using Vector9fMat = Matrix<9, 1>;
using Vector15fMat = Matrix<15, 1>;