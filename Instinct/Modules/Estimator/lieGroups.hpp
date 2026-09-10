/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Math/lieGroups.hpp
 * Brief:   SO(3) and SE_2(3) Manifold operations for the Invariant EKF.
 */

#pragma once

#include "matrix.hpp"
#include <math.h>

class LieGroups {
	public:
		// Skew-Symmetric Operator (w^): Converts a 3x1 vector into a 3x3 cross-product matrix.
		static Matrix3f Skew(const Vector3fMat& v) {
			Matrix3f S(InitType::Uninitialized);
			S.data[0] = 0.0f;
			S.data[1] = -v.data[2];
			S.data[2] = v.data[1];
			S.data[3] = v.data[2];
			S.data[4] = 0.0f;
			S.data[5] = -v.data[0];
			S.data[6] = -v.data[1];
			S.data[7] = v.data[0];
			S.data[8] = 0.0f;
			return S;
		}

		// SO(3) Exponential Map (Rodrigues' Formula): Maps a 3x1 angular velocity vector onto the SO(3) rotation manifold.
		static Matrix3f ExpSO3(const Vector3fMat& w) {
			float thetaSq = 	(w.data[0] * w.data[0]) + 
								(w.data[1] * w.data[1]) + 
								(w.data[2] * w.data[2]);
			
			float theta;
			arm_sqrt_f32(thetaSq, &theta);
			Matrix3f W = Skew(w);
			Matrix3f W2(InitType::Uninitialized);
			Matrix3f::Multiply3x3(W, W, W2);

			float aCoeff, bCoeff;

			// FPU Subnormal / Singularity Protection (Taylor Series Fallback)
			if(theta < 1e-4f) {
				aCoeff = 1.0f - (thetaSq / 6.0f);
				bCoeff = 0.5f - (thetaSq / 24.0f);
			}
			else {
				aCoeff = arm_sin_f32(theta) / theta;
				bCoeff = (1.0f - arm_cos_f32(theta)) / thetaSq;
			}

			Matrix3f result(InitType::Uninitialized);
			for(int i = 0; i < 9; i++) {
				// Inline Identity addition: 1.0f on the diagonal (indices 0, 4, 8)
				float identity = (i == 0 || i == 4 || i == 8) ? 1.0f : 0.0f;
				result.data[i] = identity + (W.data[i] * aCoeff) + (W2.data[i] * bCoeff);
			}
			return result;
		}

		static void CalculateKinematicsSO3(const Vector3fMat& w, Matrix3f& R, Matrix3f& JL) {
			float thetaSq = (w.data[0] * w.data[0]) + 
							(w.data[1] * w.data[1]) + 
							(w.data[2] * w.data[2]);
			float theta;
			arm_sqrt_f32(thetaSq, &theta);

			Matrix3f W = Skew(w);
			Matrix3f W2(InitType::Uninitialized);
			Matrix3f::Multiply3x3(W, W, W2);

			float aCoeff, bCoeff, cCoeff;
			if(theta < 1e-4f) {
				aCoeff = 1.0f - (thetaSq / 6.0f);
				bCoeff = 0.5f - (thetaSq / 24.0f);
				cCoeff = (1.0f / 6.0f) - (thetaSq / 120.0f);
			}
			else {
				aCoeff = arm_sin_f32(theta) / theta;
				bCoeff = (1.0f - arm_cos_f32(theta)) / thetaSq;
				cCoeff = (theta - arm_sin_f32(theta)) / (theta * thetaSq);
			}

			for(int i = 0; i < 9; i++) {
				float identity = (i == 0 || i == 4 || i == 8) ? 1.0f : 0.0f;
				R.data[i] = identity + (W.data[i] * aCoeff) + (W2.data[i] * bCoeff);
				// Note: The Left Jacobian's W coefficient is mathematically identical to R's W^2 coefficient (bCoeff)
				JL.data[i] = identity + (W.data[i] * bCoeff) + (W2.data[i] * cCoeff);
			}
		}

		// SO(3) Left Jacobian: Required to integrate Velocity and Position on the curved manifold.
		static Matrix3f LeftJacobianSO3(const Vector3fMat& w) {
			float thetaSq =	(w.data[0] * w.data[0]) + 
								(w.data[1] * w.data[1]) + 
								(w.data[2] * w.data[2]);
			
			float theta;
			arm_sqrt_f32(thetaSq, &theta);
			Matrix3f I;
			I.SetIdentity();
			Matrix3f W = Skew(w);
			Matrix3f W2;
			Matrix3f::Multiply3x3(W, W, W2);

			// FPU Subnormal / Singularity Protection (Taylor Series Fallback)
			float cCoeff, dCoeff;
			if(theta < 1e-4f) {
				cCoeff = 0.5f - (thetaSq / 24.0f);
				dCoeff = (1.0f / 6.0f) - (thetaSq / 120.0f);
			}
			else {
				cCoeff = (1.0f - arm_cos_f32(theta)) / thetaSq;
				dCoeff = (theta - arm_sin_f32(theta)) / (theta * thetaSq);
			}

			Matrix3f result(InitType::Uninitialized);
			for(int i = 0; i < 9; i++) {
				result.data[i] = I.data[i] + (W.data[i] * cCoeff) + (W2.data[i] * dCoeff);
			}
			return result;
		}

		// SE_2(3) Exponential Map: Maps a 9x1 error state [rotation, velocity, position] into the 5x5 Matrix.
		static void ExpSE23(const Vector9fMat& xi, Matrix5f& X) {
			Vector3fMat w(InitType::Uninitialized);
			Vector3fMat u(InitType::Uninitialized);
			Vector3fMat v(InitType::Uninitialized);

			// Extract the 3 sub-vectors
			for(int i = 0; i < 3; i++) {
				w.data[i] = xi.data[i];		// Angular error
				u.data[i] = xi.data[i+3];	// Velocity error
				v.data[i] = xi.data[i+6];	// Position error
			}

			Matrix3f R(InitType::Uninitialized);
			Matrix3f JL(InitType::Uninitialized);
			CalculateKinematicsSO3(w, R, JL);
			
			Vector3fMat Ju(InitType::Uninitialized);
			Vector3fMat Jv(InitType::Uninitialized);
			Matrix3f::Multiply3x3Vector3(JL, u, Ju);
			Matrix3f::Multiply3x3Vector3(JL, v, Jv);

			// Assemble the 5x5 Matrix
			X.SetIdentity();
			for(int r = 0; r < 3; r++) {
				for(int c = 0; c < 3; c++) {
					X.data[r * 5 + c] = R.data[r * 3 + c];
				}
				X.data[r * 5 + 3] = Ju.data[r];
				X.data[r * 5 + 4] = Jv.data[r];
			}
		}

		// SE_2(3) Adjoint Map: Maps vectors from the global frame into the local body frame through the manifold. Required to propagate the Error Covariance Matrix.
		static Matrix9f AdjointSE23(const Matrix5f& X) {
			Matrix9f Ad; // Constructor zeroes it out
			
			// Extract R, v, p from the 5x5 State Matrix
			Matrix3f R(InitType::Uninitialized);
			Vector3fMat v(InitType::Uninitialized);
			Vector3fMat p(InitType::Uninitialized);
			for(int r = 0; r < 3; r++) {
				for(int c = 0; c < 3; c++) {
					R.data[r * 3 + c] = X.data[r * 5 + c];
				}
				v.data[r] = X.data[r * 5 + 3];
				p.data[r] = X.data[r * 5 + 4];
			}

			Matrix3f vSkew = Skew(v);
			Matrix3f pSkew = Skew(p);

			Matrix3f vR(InitType::Uninitialized);
			Matrix3f pR(InitType::Uninitialized);
			Matrix3f::Multiply3x3(vSkew, R, vR);
			Matrix3f::Multiply3x3(pSkew, R, pR);

			// Assemble the 9x9 Adjoint matrix
			for(int r = 0; r < 3; r++) {
				for(int c = 0; c < 3; c++) {
					// Top-Left: R
					Ad.data[r * 9 + c] = R.data[r * 3 + c];
					
					// Mid-Left: v^ * R
					Ad.data[(r + 3) * 9 + c] = vR.data[r * 3 + c];
					// Mid-Mid: R
					Ad.data[(r + 3) * 9 + (c + 3)] = R.data[r * 3 + c];
					
					// Bottom-Left: p^ * R
					Ad.data[(r + 6) * 9 + c] = pR.data[r * 3 + c];
					// Bottom-Right: R
					Ad.data[(r + 6) * 9 + (c + 6)] = R.data[r * 3 + c];
				}
			}

			return Ad;
		}
};