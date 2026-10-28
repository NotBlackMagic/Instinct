/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Estimation/inEKF.cpp
 */

 #include "inEKF.hpp"

 InEKF::InEKF() : InEKF(Config{}) {}

 InEKF::InEKF(const Config& cfg) : 
 	config(cfg),
	lastTimestamp(0),
	isInitialized(false),
	isConverged(false),
	sampleCount(0),
	invBaroRef(1.0f / 101325.0f) 
{
	Reset();
}

void InEKF::Reset() {
	lastTimestamp = 0;
	isInitialized = false;
	isConverged = false;
	sampleCount = 0;

	R.SetIdentity();
	memset(v.data, 0, sizeof(v.data));
	memset(p.data, 0, sizeof(p.data));
	memset(gyroBias.data, 0, sizeof(gyroBias.data));
	memset(accelBias.data, 0, sizeof(accelBias.data));

	memset(magRefNed.data, 0, sizeof(magRefNed.data));
	magRefNed.data[0] = 0.5f;

	for(int r = 0; r < 5; r++) {
		for(int c = 0; c < 5; c++) {
			memset(P[r][c].data, 0, sizeof(P[r][c].data));
		}
	}

	const float initVars[5] = {
		config.initAttitudeVar,
		config.initVelocityVar,
		config.initPositionVar,
		config.initGyroBiasVar,
		config.initAccelBiasVar
	};

	for(int b = 0; b < 5; b++) {
		P[b][b].data[0] = P[b][b].data[4] = P[b][b].data[8] = initVars[b];
	}
}

void InEKF::InitializeState(uint64_t timestamp) {
	lastTimestamp = timestamp;
	isInitialized = true;
	isConverged = false;
	sampleCount = 0;
}

Status InEKF::InitializeMagReference(const float magInit[3]) {
	Vector3fMat magBody(InitType::Uninitialized);
	magBody.data[0] = magInit[0];
	magBody.data[1] = magInit[1];
	magBody.data[2] = magInit[2];

	// Project raw boot magnetometer reading into the Global frame
	Matrix3f::Multiply3x3Vector3(R, magBody, magRefNed);
	magRefNed.data[1] = 0.0f;	// Force East (Y) to 0, locking heading as North
	return Status::Ok;
}

Status InEKF::InitializeBaroReference(float baroInit) {
	invBaroRef = 1.0f / baroInit;
	return Status::Ok;
}

Status InEKF::Predict(const float accel[3], const float gyro[3], uint64_t timestamp) {
	if(isInitialized == false) {
		InitializeState(timestamp);
		return Status::Ok;
	}

	float dt = (float)(timestamp - lastTimestamp) / 1000000.0f;
	lastTimestamp = timestamp;

	// Guard against erratic timing loops or thread execution stalls
	if(dt <= 0.0f || dt > 0.1f) {
		return Status::Ok;
	}

	// Remove estimated sensor biases from raw hardware measurements w. continuous-time kinematic manifold integration
	Vector3fMat wStep(InitType::Uninitialized);
	wStep.data[0] = (gyro[0] - gyroBias.data[0]) * dt;
	wStep.data[1] = (gyro[1] - gyroBias.data[1]) * dt;
	wStep.data[2] = (gyro[2] - gyroBias.data[2]) * dt;

	Vector3fMat aUnbiased(InitType::Uninitialized);
	aUnbiased.data[0] = accel[0] - accelBias.data[0];
	aUnbiased.data[1] = accel[1] - accelBias.data[1];
	aUnbiased.data[2] = accel[2] - accelBias.data[2];

	Matrix3f expR = LieGroups::ExpSO3(wStep);

	// Rotate local acceleration vector into the global frame
	Vector3fMat aGlobal(InitType::Uninitialized);
	Matrix3f::Multiply3x3Vector3(R, aUnbiased, aGlobal);

	// Cache parameters for the upcoming covariance transition mapping
	Matrix3f oldR = R;
	Vector3fMat oldV = v;
	Vector3fMat oldP = p;

	// Cache specific force * dt before adding gravity for 3D Right-Invariant attitude-velocity coupling:
	// Phi_10 = -Skew(R * aUnbiased) * dt. In hover (aGlobal = [0, 0, -g]), this equals +Skew([0, 0, g]) * dt, and during horizontal accelerations it provides dynamic Yaw observability from GPS/Flow
	const float axDt = -aGlobal.data[0] * dt;
	const float ayDt = -aGlobal.data[1] * dt;
	const float azDt = -aGlobal.data[2] * dt; // +g * dt in hover

	// Update Kinematics
	Matrix3f::Multiply3x3(oldR, expR, R);

	float dtSqHalf = 0.5f * dt * dt;
	aGlobal.data[2] += this->gVal; // Add gravity along +Z (NED Down)
	for(int i = 0; i < 3; i++) {
		v.data[i] = oldV.data[i] + (aGlobal.data[i] * dt);
		p.data[i] = oldP.data[i] + (oldV.data[i] * dt) + (aGlobal.data[i] * dtSqHalf);
	}

	// Construct non-zero blocks of SE_2(3) Discrete Transition Matrix Phi:
	// vR = -dt * Skew(oldV) * oldR, negR_dt = -dt * oldR
	Matrix3f negR_dt(InitType::Uninitialized);
	Matrix3f::MultiplyScalar3x3(oldR, -dt, negR_dt);

	// Uninitialized array avoids 15 redundant memset calls (540 bytes) per IMU sample
	Matrix3f Y[3][5] = {
		{Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized)},
		{Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized)},
		{Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized)}
	};

	// Y = Phi * P (Only rows 0, 1, 2 change; rows 3 and 4 of Phi are Identity!)
	for(int c = 0; c < 5; c++) {
		// Row 0: P[0][c] + negR_dt * P[3][c]
		Y[0][c] = P[0][c];
		Matrix3f::MultiplyAccumulate3x3(negR_dt, P[3][c], Y[0][c]);

		// Row 1: P[1][c] + Skew([axDt, ayDt, azDt]) * P[0][c] + negR_dt * P[4][c]
		const float* p0 = P[0][c].data;
		const float* p1 = P[1][c].data;
		float* y1 = Y[1][c].data;

		y1[0] = p1[0] + (ayDt * p0[6] - azDt * p0[3]);
		y1[1] = p1[1] + (ayDt * p0[7] - azDt * p0[4]);
		y1[2] = p1[2] + (ayDt * p0[8] - azDt * p0[5]);

		y1[3] = p1[3] + (azDt * p0[0] - axDt * p0[6]);
		y1[4] = p1[4] + (azDt * p0[1] - axDt * p0[7]);
		y1[5] = p1[5] + (azDt * p0[2] - axDt * p0[8]);

		y1[6] = p1[6] + (axDt * p0[3] - ayDt * p0[0]);
		y1[7] = p1[7] + (axDt * p0[4] - ayDt * p0[1]);
		y1[8] = p1[8] + (axDt * p0[5] - ayDt * p0[2]);

		Matrix3f::MultiplyAccumulate3x3(negR_dt, P[4][c], Y[1][c]);

		// Row 2: P[2][c] + dt * P[1][c]
		const float* p2 = P[2][c].data;
		float* y2 = Y[2][c].data;
		for(int i = 0; i < 9; i++) {
			y2[i] = p2[i] + (p1[i] * dt);
		}
	}

	// Upper triangle of P_new = Y * Phi^T
	for(int r = 0; r < 3; r++) {
		// Col 0 (only for r == 0)
		if(r == 0) {
			P[0][0] = Y[0][0];
			Matrix3f::MultiplyAccumulateTransposedB3x3(Y[0][3], negR_dt, P[0][0]);
		}

		// Col 1 (for r <= 1)
		if(r <= 1) {
			P[r][1] = Y[r][1];
			// Y[r][0] * (Skew([axDt, ayDt, azDt]))^T
			const float* y0 = Y[r][0].data;
			float* pr1 = P[r][1].data;

			pr1[0] += (ayDt * y0[2] - azDt * y0[1]);
			pr1[1] += (azDt * y0[0] - axDt * y0[2]);
			pr1[2] += (axDt * y0[1] - ayDt * y0[0]);

			pr1[3] += (ayDt * y0[5] - azDt * y0[4]);
			pr1[4] += (azDt * y0[3] - axDt * y0[5]);
			pr1[5] += (axDt * y0[4] - ayDt * y0[3]);

			pr1[6] += (ayDt * y0[8] - azDt * y0[7]);
			pr1[7] += (azDt * y0[6] - axDt * y0[8]);
			pr1[8] += (axDt * y0[7] - ayDt * y0[6]);

			Matrix3f::MultiplyAccumulateTransposedB3x3(Y[r][4], negR_dt, P[r][1]);
		}

		// Col 2 (for r <= 2)
		const float* yr1 = Y[r][1].data;
		const float* yr2 = Y[r][2].data;
		float* pr2 = P[r][2].data;
		for(int i = 0; i < 9; i++) {
			pr2[i] = yr2[i] + (yr1[i] * dt);
		}

		// Col 3 & 4
		P[r][3] = Y[r][3];
		P[r][4] = Y[r][4];
	}
	// Rows 3 and 4 for Cols 3 and 4 remain unchanged prior to process noise Q!

	// Mirror upper triangle to lower triangle
	MirrorCovarianceLower();

	// Add Process Noise (Q) to diagonals
	float dtQ[5] = {
		this->config.gyroNoiseVar * dt,
		this->config.accelNoiseVar * dt,
		this->config.posNoiseVar * dt,
		this->config.gyroBiasWalkVar * dt,
		this->config.accelBiasWalkVar * dt
	};

	// UPDATE: Correct covariance clamping while preserving positive-definiteness
	for(int b = 0; b < 5; b++) {
		for(int a = 0; a < 3; a++) {
			int diagIdx = a * 3 + a;
			float diagVal = P[b][b].data[diagIdx] + dtQ[b];

			if(diagVal < this->minCovDiag) {
				// Adding positive variance to a diagonal always strengthens positive-definiteness
				P[b][b].data[diagIdx] = this->minCovDiag;
			}
			else if(diagVal > this->maxCovDiag) {
				// Scale entire row (b, a) and column (b, a) across the 15x15 matrix
				// P_new = D * P * D preserves correlation ratios and positive-definiteness!
				P[b][b].data[diagIdx] = diagVal;
				float scale;
				arm_sqrt_f32(this->maxCovDiag / diagVal, &scale);

				// Scale row 'a' across all 5 column blocks
				for(int c = 0; c < 5; c++) {
					P[b][c].data[a * 3 + 0] *= scale;
					P[b][c].data[a * 3 + 1] *= scale;
					P[b][c].data[a * 3 + 2] *= scale;
				}

				// Scale col 'a' across all 5 row blocks
				for(int r = 0; r < 5; r++) {
					P[r][b].data[0 * 3 + a] *= scale;
					P[r][b].data[1 * 3 + a] *= scale;
					P[r][b].data[2 * 3 + a] *= scale;
				}

				// Explicitly pin the diagonal to avoid float rounding drift
				P[b][b].data[diagIdx] = this->maxCovDiag;
			}
			else {
				P[b][b].data[diagIdx] = diagVal;
			}
		}
	}

	// for(int b = 0; b < 5; b++) {
	// 	P[b][b].data[0] = fmaxf(this->minCovDiag, P[b][b].data[0] + dtQ[b]);
	// 	P[b][b].data[4] = fmaxf(this->minCovDiag, P[b][b].data[4] + dtQ[b]);
	// 	P[b][b].data[8] = fmaxf(this->minCovDiag, P[b][b].data[8] + dtQ[b]);
	// 	// UPDATE: Added max covariance limit 
	// 	// P[b][b].data[0] = fminf(this->maxCovDiag, fmaxf(this->minCovDiag, P[b][b].data[0] + dtQ[b]));
	// 	// P[b][b].data[4] = fminf(this->maxCovDiag, fmaxf(this->minCovDiag, P[b][b].data[4] + dtQ[b]));
	// 	// P[b][b].data[8] = fminf(this->maxCovDiag, fmaxf(this->minCovDiag, P[b][b].data[8] + dtQ[b]));
	// }

	// Matrix Orthogonalization Hook: Periodically snap the manifold back to a perfect SO(3) rotation matrix
	if(sampleCount % 100 == 0) {
		this->NormalizeRotation();
	}

	// Establish a convergence threshold (approx. 1 second of stable updates)
	if(sampleCount < this->config.convergenceSamples) {
		sampleCount = sampleCount + 1;
	}
	else {
		isConverged = true;
	}

	return Status::Ok;
}

Status InEKF::UpdateGravity(const float accel[3]) {
	if(isInitialized == false) {
		return Status::Ok;
	}

	// 1G Magnitude Gate (0.8G to 1.2G)
	float u0 = accel[0] - accelBias.data[0];
	float u1 = accel[1] - accelBias.data[1];
	float u2 = accel[2] - accelBias.data[2];
	float normSq = u0 * u0 + u1 * u1 + u2 * u2;
	if(normSq < (gSq * 0.64f) || normSq > (gSq * 1.44f)) {
		return Status::Ok;
	}

	// Normalize measured specific force to gVal so Z-axis scale/bias errors never tilt the horizon when pitching/rolling
	float normU;
	arm_sqrt_f32(normSq, &normU);
	float scaleG = gVal / normU;
	u0 *= scaleG;
	u1 *= scaleG;
	u2 *= scaleG;

	// Expected specific force in body frame: R^T * [0, 0, -g]^T + accelBias
	// Since [0, 0, -g] only has Z, R^T * [0, 0, -g] is simply -g * Row 2 of R!
	Vector3fMat z(InitType::Uninitialized);
	z.data[0] = u0 - (-gVal * R.data[6]);
	z.data[1] = u1 - (-gVal * R.data[7]);
	z.data[2] = u2 - (-gVal * R.data[8]);

	// Right-Invariant hAttitude = R^T * Skew([0, 0, -g]): Col 2 is identically zero (Yaw is 100% decoupled!)
	const float h00 = -gVal * R.data[3], h01 = gVal * R.data[0];
	const float h10 = -gVal * R.data[4], h11 = gVal * R.data[1];
	const float h20 = -gVal * R.data[5], h21 = gVal * R.data[2];

	// Exploit 2-column sparsity of hAtt to compute HP[c] = hAtt * P[0][c]
	Matrix3f HP[5] = {
		Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized),
		Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized)
	};
	for(int c = 0; c < 5; c++) {
		const float* p0 = P[0][c].data;
		float* hp = HP[c].data;
		hp[0] = h00 * p0[0] + h01 * p0[3]; hp[1] = h00 * p0[1] + h01 * p0[4]; hp[2] = h00 * p0[2] + h01 * p0[5];
		hp[3] = h10 * p0[0] + h11 * p0[3]; hp[4] = h10 * p0[1] + h11 * p0[4]; hp[5] = h10 * p0[2] + h11 * p0[5];
		hp[6] = h20 * p0[0] + h21 * p0[3]; hp[7] = h20 * p0[1] + h21 * p0[4]; hp[8] = h20 * p0[2] + h21 * p0[5];
	}

	// S = HP[0] * hAtt^T + R_Noise (exploiting Col 2 of hAtt == 0)
	const float* hp0 = HP[0].data;
	Matrix3f S(InitType::Uninitialized);
	S.data[0] = hp0[0] * h00 + hp0[1] * h01 + this->config.gravityMeasVar;
	S.data[1] = hp0[0] * h10 + hp0[1] * h11;
	S.data[2] = hp0[0] * h20 + hp0[1] * h21;
	S.data[3] = S.data[1];
	S.data[4] = hp0[3] * h10 + hp0[4] * h11 + this->config.gravityMeasVar;
	S.data[5] = hp0[3] * h20 + hp0[4] * h21;
	S.data[6] = S.data[2];
	S.data[7] = S.data[5];
	S.data[8] = hp0[6] * h20 + hp0[7] * h21 + this->config.gravityMeasVar;

	Matrix3f sInv = S.Inverse3x3();

	Vector3fMat sInvZ(InitType::Uninitialized);
	Matrix3f::Multiply3x3Vector3(sInv, z, sInvZ);
	float mahalanobisSq = (z.data[0] * sInvZ.data[0]) + (z.data[1] * sInvZ.data[1]) + (z.data[2] * sInvZ.data[2]);
	if(!(mahalanobisSq >= 0.0f && mahalanobisSq <= this->gate3Sigma3D)) {
		return Status::Ok;
	}

	// Exact Schmidt-Kalman update for Attitude (r = 0) and Gyro Bias (r = 3)
	// Velocity (r = 1) and Position (r = 2) are untouched so attitude updates never move v or p!
	float delta[15] = {0.0f};
	const int activeRows[2] = {0, 3}; // Only Attitude (0) and Gyro Bias (3)
	for(int idx = 0; idx < 2; idx++) {
		int r = activeRows[idx];

		Matrix3f hpT(InitType::Uninitialized);
		Matrix3f::Transpose3x3(HP[r], hpT);

		Matrix3f Kr(InitType::Uninitialized);
		Matrix3f::Multiply3x3(hpT, sInv, Kr);

		Vector3fMat deltaBlk(InitType::Uninitialized);
		Matrix3f::MultiplyTransposed3x3Vector3(HP[r], sInvZ, deltaBlk);
		delta[r * 3 + 0] = deltaBlk.data[0];
		delta[r * 3 + 1] = deltaBlk.data[1];
		delta[r * 3 + 2] = deltaBlk.data[2];

		// Update entire row 'r' of P: P[r][c] -= Kr * HP[c]
		for(int c = 0; c < 5; c++) {
			Matrix3f::MultiplySubtract3x3(Kr, HP[c], P[r][c]);
		}
	}

	// Keep upper triangle in sync for row 3's off-diagonal columns (r = 1, 2 < 3)
	Matrix3f::Transpose3x3(P[3][1], P[1][3]);
	Matrix3f::Transpose3x3(P[3][2], P[2][3]);

	// Compute and Apply Error Correction
	ApplyErrorState(delta);

	// Mirror upper triangle to lower triangle
	MirrorCovarianceLower();

	return Status::Ok;
}

Status InEKF::UpdateMag(const float mag[3]) {
	if(isInitialized == false) {
		return Status::Ok;
	}

	// Compute expected magnetic reading in the body frame
	Vector3fMat yExpected(InitType::Uninitialized);
	Matrix3f::MultiplyTransposed3x3Vector3(R, magRefNed, yExpected);

	// Innovation calculation (Residual)
	Vector3fMat z(InitType::Uninitialized);
	z.data[0] = mag[0] - yExpected.data[0];
	z.data[1] = mag[1] - yExpected.data[1];
	z.data[2] = mag[2] - yExpected.data[2];

	// Direct formation of hMagAtt = R^T * Skew([mx, 0, mz])
	const float mx = magRefNed.data[0];
	const float mz = magRefNed.data[2];
	Matrix3f hMagAtt(InitType::Uninitialized);
	hMagAtt.data[0] = mz * R.data[3]; hMagAtt.data[1] = mx * R.data[6] - mz * R.data[0]; hMagAtt.data[2] = -mx * R.data[3];
	hMagAtt.data[3] = mz * R.data[4]; hMagAtt.data[4] = mx * R.data[7] - mz * R.data[1]; hMagAtt.data[5] = -mx * R.data[4];
	hMagAtt.data[6] = mz * R.data[5]; hMagAtt.data[7] = mx * R.data[8] - mz * R.data[2]; hMagAtt.data[8] = -mx * R.data[5];

	// Compute H * P: An array of five 3x3 blocks
	Matrix3f HP[5] = {
		Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized),
		Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized)
	};
	for(int c = 0; c < 5; c++) {
		Matrix3f::Multiply3x3(hMagAtt, P[0][c], HP[c]);
	}

	// Compute S = (H * P) * H^T + R_Noise
	Matrix3f S(InitType::Uninitialized);
	Matrix3f::MultiplyTransposedB3x3(HP[0], hMagAtt, S);
	S.data[0] += this->config.magMeasVar;
	S.data[4] += this->config.magMeasVar;
	S.data[8] += this->config.magMeasVar;

	Matrix3f sInv = S.Inverse3x3();

	// Innovation Gating: Mahalanobis distance for 3D vectors: z^T * S_inv * z
	Vector3fMat sInvZ(InitType::Uninitialized);
	Matrix3f::Multiply3x3Vector3(sInv, z, sInvZ);
	float mahalanobisSq = (z.data[0] * sInvZ.data[0]) + (z.data[1] * sInvZ.data[1]) + (z.data[2] * sInvZ.data[2]);
	if(!(mahalanobisSq >= 0.0f && mahalanobisSq <= this->gate3Sigma3D)) {
		return Status::Ok;
	}

	// Exact Schmidt-Kalman update for Attitude (r = 0) and Gyro Bias (r = 3)
	float delta[15] = {0.0f};
	const int activeRows[2] = {0, 3}; // Only Attitude (0) and Gyro Bias (3)
	for(int idx = 0; idx < 2; idx++) {
		int r = activeRows[idx];

		Matrix3f hpT(InitType::Uninitialized);
		Matrix3f::Transpose3x3(HP[r], hpT);

		Matrix3f Kr(InitType::Uninitialized);
		Matrix3f::Multiply3x3(hpT, sInv, Kr);

		Vector3fMat deltaBlk(InitType::Uninitialized);
		Matrix3f::MultiplyTransposed3x3Vector3(HP[r], sInvZ, deltaBlk);
		delta[r * 3 + 0] = deltaBlk.data[0];
		delta[r * 3 + 1] = deltaBlk.data[1];
		delta[r * 3 + 2] = deltaBlk.data[2];

		// Update entire row 'r' of P: P[r][c] -= Kr * HP[c]
		for(int c = 0; c < 5; c++) {
			Matrix3f::MultiplySubtract3x3(Kr, HP[c], P[r][c]);
		}
	}

	// Keep upper triangle in sync for row 3's off-diagonal columns (r = 1, 2 < 3)
	Matrix3f::Transpose3x3(P[3][1], P[1][3]);
	Matrix3f::Transpose3x3(P[3][2], P[2][3]);

	// Compute and Apply Error Correction
	ApplyErrorState(delta);

	// Mirror upper triangle to lower triangle
	MirrorCovarianceLower();

	return Status::Ok;
}

Status InEKF::UpdateVerticalPosition(float range, float rNoise) {
	// In the NED frame standard, Down is positive, meaning Altitude points along the negative axis
	float z = range - p.data[2];	// Innovation error calculation
	
	// Compute H * P (which evaluates to a 1x15 row vector)
	// H = 1.0 at index 8 (p.z) and 0.0 elsewhere
	float hpRow[15];
	for(int c = 0; c < 5; c++) {
		const float* p2 = P[2][c].data;
		hpRow[c * 3 + 0] = p2[6];
		hpRow[c * 3 + 1] = p2[7];
		hpRow[c * 3 + 2] = p2[8];
	}

	// Compute S = H * P * H^T + R_Noise
	float sVal = hpRow[8] + rNoise;
	if(!(sVal > 1e-7f) || (z * z) > (this->gate3Sigma1D * sVal)) {
		return Status::Ok;
	}

	float invS = 1.0f / sVal;
	float scaledZ = z * invS;

	// 1D Vertical Sensor Subspace Lock:
	// Only correct Vertical Velocity v.z (index 5) and Vertical Position p.z (index 8) so pitching/rolling
	// never pushes horizontal (p.x, p.y) or (v.x, v.y) via tilted covariance cross-terms!
	float kVz = hpRow[5] * invS;
	float kPz = hpRow[8] * invS;

	v.data[2] += hpRow[5] * scaledZ;
	p.data[2] += hpRow[8] * scaledZ;

	// Update Row 5 (r = 1, row 2) and Row 8 (r = 2, row 2) of P
	for(int c = 0; c < 5; c++) {
		float* p1c = P[1][c].data;
		float* p2c = P[2][c].data;
		p1c[6] -= kVz * hpRow[c * 3 + 0];
		p1c[7] -= kVz * hpRow[c * 3 + 1];
		p1c[8] -= kVz * hpRow[c * 3 + 2];

		p2c[6] -= kPz * hpRow[c * 3 + 0];
		p2c[7] -= kPz * hpRow[c * 3 + 1];
		p2c[8] -= kPz * hpRow[c * 3 + 2];
	}

	// Keep upper-triangle blocks P[0][1], P[0][2], P[1][2] consistent before MirrorCovarianceLower
	P[0][1].data[2] = P[1][0].data[6]; P[0][1].data[5] = P[1][0].data[7]; P[0][1].data[8] = P[1][0].data[8];
	P[0][2].data[2] = P[2][0].data[6]; P[0][2].data[5] = P[2][0].data[7]; P[0][2].data[8] = P[2][0].data[8];
	P[1][2].data[2] = P[2][1].data[6]; P[1][2].data[5] = P[2][1].data[7];

	// Mirror upper triangle to lower triangle
	MirrorCovarianceLower();

	return Status::Ok;
}

Status InEKF::UpdateBaro(float pressure) {
	if(isInitialized == false) {
		return Status::Ok;
	}

	float measuredAltitude = 44330.77f * (1.0f - powf(pressure * this->invBaroRef, 0.1902632f));
	return UpdateVerticalPosition(-measuredAltitude, this->config.baroMeasVar);
}

Status InEKF::UpdateRange(float range) {
	if(isInitialized == false) {
		return Status::Ok;
	}

	float cosTilt = R.data[8];
	if(range < this->config.minRange || range > this->config.maxRange || cosTilt < this->config.minTiltCos) {
		return Status::Ok;
	}

	float rangeTmp = -(range * cosTilt);
	float rNoise = this->config.rangeMeasVar / (cosTilt * cosTilt);
	return UpdateVerticalPosition(rangeTmp, rNoise);
}

Status InEKF::UpdateOpticalFlow(const float flowRate[2], const float gyro[3], float range, float quality) {
	if(isInitialized == false || quality <= 0.05f) {
		return Status::Ok;
	}

	// Require valid ground distance (use ToF if valid, otherwise fallback to EKF height -p.z / R[8])
	float cosTilt = R.data[8];
	if(cosTilt < this->config.minTiltCos) {
		return Status::Ok;
	}

	float d = range;
	if(d < this->config.minRange || d > this->config.maxRange) {
		float estHeight = -p.data[2];
		if(estHeight < this->config.minRange) {
			return Status::Ok;
		}
		d = estHeight / cosTilt;
	}

	// Derotate optical flow using unbiased gyro (in rad/s) and scale by range d to get body velocity [vx_b, vy_b]
	float wUnbiasedX = gyro[0] - gyroBias.data[0];
	float wUnbiasedY = gyro[1] - gyroBias.data[1];

	// Standard downward camera alignment:
	// Pitching nose down (-wy) / moving forward (+vx_b) produces flow along wy
	// Rolling right (+wx) / moving right (+vy_b) produces flow along wx
	float vxMeasBody = (flowRate[1] - wUnbiasedY) * d;
	float vyMeasBody = -(flowRate[0] - wUnbiasedX) * d;

	// Predicted body-frame velocity: v_body = R^T * v_ned
	// Only need X and Y components of R^T * v
	float vxPredBody = R.data[0] * v.data[0] + R.data[3] * v.data[1] + R.data[6] * v.data[2];
	float vyPredBody = R.data[1] * v.data[0] + R.data[4] * v.data[1] + R.data[7] * v.data[2];

	// 2D Innovation z = [vx_meas - vx_pred, vy_meas - vy_pred]
	float z0 = vxMeasBody - vxPredBody;
	float z1 = vyMeasBody - vyPredBody;

	// Right-Invariant Jacobian for Body Velocity v_b = R^T * v:
	// Attitude error cancels out! H is non-zero ONLY at Velocity block (Block 1):
	// Row 0 of H_vel = Row 0 of R^T = Col 0 of R = [R[0], R[3], R[6]]
	// Row 1 of H_vel = Row 1 of R^T = Col 1 of R = [R[1], R[4], R[7]]
	float h0x = R.data[0], h0y = R.data[3], h0z = R.data[6];
	float h1x = R.data[1], h1y = R.data[4], h1z = R.data[7];

	// Compute HP (2x15 matrix) using only Block 1 of P (P[1][c])
	float hp0[15];
	float hp1[15];
	for(int c = 0; c < 5; c++) {
		const float* pBlk = P[1][c].data;
		hp0[c * 3 + 0] = h0x * pBlk[0] + h0y * pBlk[3] + h0z * pBlk[6];
		hp0[c * 3 + 1] = h0x * pBlk[1] + h0y * pBlk[4] + h0z * pBlk[7];
		hp0[c * 3 + 2] = h0x * pBlk[2] + h0y * pBlk[5] + h0z * pBlk[8];

		hp1[c * 3 + 0] = h1x * pBlk[0] + h1y * pBlk[3] + h1z * pBlk[6];
		hp1[c * 3 + 1] = h1x * pBlk[1] + h1y * pBlk[4] + h1z * pBlk[7];
		hp1[c * 3 + 2] = h1x * pBlk[2] + h1y * pBlk[5] + h1z * pBlk[8];
	}

	// Scale measurement noise inversely with flow quality and proportionally with distance^2
	float baseVelVar = this->config.flowMeasVar * (1.0f + d * d);
	float rNoise = baseVelVar / fmaxf(0.1f, quality);

	// 2x2 Innovation Covariance S = H * P * H^T + R_noise (using Block 1 columns: indices 3, 4, 5)
	float s00 = (hp0[3] * h0x + hp0[4] * h0y + hp0[5] * h0z) + rNoise;
	float s01 = (hp0[3] * h1x + hp0[4] * h1y + hp0[5] * h1z);
	float s11 = (hp1[3] * h1x + hp1[4] * h1y + hp1[5] * h1z) + rNoise;

	float detS = (s00 * s11) - (s01 * s01);
	if(!(detS > 1e-7f)) {
		return Status::Ok;
	}
	float invDetS = 1.0f / detS;
	float sInv00 = s11 * invDetS;
	float sInv01 = -s01 * invDetS;
	float sInv11 = s00 * invDetS;

	// 2D Mahalanobis distance squared: z^T * S^-1 * z (95% / 3-sigma gate for 2 DOF = 9.21f)
	// Precompute sInvZ = S^-1 * z (2 floats)
	float sInvZ0 = sInv00 * z0 + sInv01 * z1;
	float sInvZ1 = sInv01 * z0 + sInv11 * z1;
	float mahalanobisSq = z0 * sInvZ0 + z1 * sInvZ1;
	if(!(mahalanobisSq >= 0.0f && mahalanobisSq <= this->gate3Sigma2D)) {
		return Status::Ok;
	}

	// Kalman Gain K (15x2): since P is symmetric, P * H^T = (H * P)^T = [hp0^T | hp1^T]
	float k0[15];
	float k1[15];
	float delta[15];
	for(int i = 0; i < 15; i++) {
		k0[i] = hp0[i] * sInv00 + hp1[i] * sInv01;
		k1[i] = hp0[i] * sInv01 + hp1[i] * sInv11;
		delta[i] = hp0[i] * sInvZ0 + hp1[i] * sInvZ1;
	}

	// Compute and Apply Error Correction
	ApplyErrorState(delta);

	// Covariance Update P = P - K * (H * P)
	// Calculate ONLY the upper triangle (c >= r).
	for(int c = 0; c < 5; c++) {
		for(int r = 0; r <= c; r++) {
			float* pBlk = P[r][c].data;
			for(int i = 0; i < 3; i++) {
				int rowIdx = r * 3 + i;
				float k0r = k0[rowIdx];
				float k1r = k1[rowIdx];
				pBlk[i * 3 + 0] -= (k0r * hp0[c * 3 + 0] + k1r * hp1[c * 3 + 0]);
				pBlk[i * 3 + 1] -= (k0r * hp0[c * 3 + 1] + k1r * hp1[c * 3 + 1]);
				pBlk[i * 3 + 2] -= (k0r * hp0[c * 3 + 2] + k1r * hp1[c * 3 + 2]);
			}
		}
	}

	// Mirror upper triangle to lower triangle
	MirrorCovarianceLower();

	return Status::Ok;
}

Status InEKF::UpdateGPSPosition(const float position[3]) {
	if(isInitialized == false) {
		return Status::Ok;
	}

	// 3D NED Position Residual: z = posMeas - p
	Vector3fMat z(InitType::Uninitialized);
	z.data[0] = position[0] - p.data[0];
	z.data[1] = position[1] - p.data[1];
	z.data[2] = position[2] - p.data[2];

	// Right-Invariant Jacobian Block 0: hPosAtt = -Skew(p)
	// H = [ 0 | 0 | I_3 | 0 | 0 ] -> HP[c] = P[2][c], S = P[2][2] + R_Noise
	Matrix3f S = P[2][2];
	S.data[0] += config.gpsHorizPosVar;
	S.data[4] += config.gpsHorizPosVar;
	S.data[8] += config.gpsVertPosVar;

	Matrix3f sInv = S.Inverse3x3();

	// 3D Mahalanobis Gate (3-sigma for 3 DOF = 11.3f)
	Vector3fMat sInvZ(InitType::Uninitialized);
	Matrix3f::Multiply3x3Vector3(sInv, z, sInvZ);
	float mahalanobisSq = (z.data[0] * sInvZ.data[0]) + (z.data[1] * sInvZ.data[1]) + (z.data[2] * sInvZ.data[2]);
	if(!(mahalanobisSq >= 0.0f && mahalanobisSq <= gate3Sigma3D)) {
		return Status::Ok;
	}

	// Cache HP[c] = P[2][c] before modifying P in-place
	Matrix3f HP[5] = {
		P[2][0], P[2][1], P[2][2], P[2][3], P[2][4]
	};

	float delta[15];
	Matrix3f K[5] = {
		Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized),
		Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized)
	};
	for(int r = 0; r < 5; r++) {
		// Since P is symmetric, (P * H^T)_r = P[r][2] = HP[r]^T
		Matrix3f::Multiply3x3(P[r][2], sInv, K[r]);

		Vector3fMat deltaBlk(InitType::Uninitialized);
		Matrix3f::Multiply3x3Vector3(P[r][2], sInvZ, deltaBlk);
		delta[r * 3 + 0] = deltaBlk.data[0];
		delta[r * 3 + 1] = deltaBlk.data[1];
		delta[r * 3 + 2] = deltaBlk.data[2];
	}

	ApplyErrorState(delta);

	for(int r = 0; r < 5; r++) {
		for(int c = r; c < 5; c++) {
			Matrix3f::MultiplySubtract3x3(K[r], HP[c], P[r][c]);
		}
	}

	MirrorCovarianceLower();
	return Status::Ok;
}

Status InEKF::UpdateGPSVelocity(const float velocity[3]) {
	if(isInitialized == false) {
		return Status::Ok;
	}

	// 3D NED Velocity Residual: z = velMeas - v
	Vector3fMat z(InitType::Uninitialized);
	z.data[0] = velocity[0] - v.data[0];
	z.data[1] = velocity[1] - v.data[1];
	z.data[2] = velocity[2] - v.data[2];

	Matrix3f S = P[1][1];
	S.data[0] += config.gpsVelVar;
	S.data[4] += config.gpsVelVar;
	S.data[8] += config.gpsVelVar;

	Matrix3f sInv = S.Inverse3x3();

	Vector3fMat sInvZ(InitType::Uninitialized);
	Matrix3f::Multiply3x3Vector3(sInv, z, sInvZ);
	float mahalanobisSq = (z.data[0] * sInvZ.data[0]) + (z.data[1] * sInvZ.data[1]) + (z.data[2] * sInvZ.data[2]);
	if(!(mahalanobisSq >= 0.0f && mahalanobisSq <= gate3Sigma3D)) {
		return Status::Ok;
	}

	Matrix3f HP[5] = {
		P[1][0], P[1][1], P[1][2], P[1][3], P[1][4]
	};

	float delta[15];
	Matrix3f K[5] = {
		Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized),
		Matrix3f(InitType::Uninitialized), Matrix3f(InitType::Uninitialized)
	};
	for(int r = 0; r < 5; r++) {
		Matrix3f::Multiply3x3(P[r][1], sInv, K[r]);

		Vector3fMat deltaBlk(InitType::Uninitialized);
		Matrix3f::Multiply3x3Vector3(P[r][1], sInvZ, deltaBlk);
		delta[r * 3 + 0] = deltaBlk.data[0];
		delta[r * 3 + 1] = deltaBlk.data[1];
		delta[r * 3 + 2] = deltaBlk.data[2];
	}

	ApplyErrorState(delta);

	for(int r = 0; r < 5; r++) {
		for(int c = r; c < 5; c++) {
			Matrix3f::MultiplySubtract3x3(K[r], HP[c], P[r][c]);
		}
	}

	MirrorCovarianceLower();
	return Status::Ok;
}

Quaternion InEKF::GetQuaternion() const {
	Quaternion q;
	float trace = R.data[0] + R.data[4] + R.data[8];
	if(trace > 0.0f) {
		float root;
		arm_sqrt_f32(fmaxf(0.0f, trace + 1.0f), &root);
		float s = 0.5f / root;
		q.w = 0.5f * root;
		q.x = (R.data[7] - R.data[5]) * s;
		q.y = (R.data[2] - R.data[6]) * s;
		q.z = (R.data[3] - R.data[1]) * s;
	}
	else if(R.data[0] > R.data[4] && R.data[0] > R.data[8]) {
		float root;
		arm_sqrt_f32(fmaxf(0.0f, 1.0f + R.data[0] - R.data[4] - R.data[8]), &root);
		float s = 2.0f * root;
		float invS = 1.0f / s;
		q.w = (R.data[7] - R.data[5]) * invS;
		q.x = 0.25f * s;
		q.y = (R.data[1] + R.data[3]) * invS; // Fixed bug: was R[1] + R[4]!
		q.z = (R.data[2] + R.data[6]) * invS; // Fixed bug: was R[2] + R[8]!
	}
	else if(R.data[4] > R.data[8]) {
		float root;
		arm_sqrt_f32(fmaxf(0.0f, 1.0f + R.data[4] - R.data[0] - R.data[8]), &root);
		float s = 2.0f * root;
		float invS = 1.0f / s;
		q.w = (R.data[2] - R.data[6]) * invS;
		q.x = (R.data[1] + R.data[3]) * invS; // Fixed bug: was R[1] + R[4]!
		q.y = 0.25f * s;
		q.z = (R.data[5] + R.data[7]) * invS;
	}
	else {
		float root;
		arm_sqrt_f32(fmaxf(0.0f, 1.0f + R.data[8] - R.data[0] - R.data[4]), &root);
		float s = 2.0f * root;
		float invS = 1.0f / s;
		q.w = (R.data[3] - R.data[1]) * invS;
		q.x = (R.data[2] + R.data[6]) * invS; // Fixed bug: was R[2] + R[8]!
		q.y = (R.data[5] + R.data[7]) * invS;
		q.z = 0.25f * s;
	}
	return q;
}

void InEKF::NormalizeRotation() {
	float x0 = R.data[0], x1 = R.data[3], x2 = R.data[6];
	float y0 = R.data[1], y1 = R.data[4], y2 = R.data[7];

	float xMag;
	arm_sqrt_f32(x0 * x0 + x1 * x1 + x2 * x2, &xMag);
	float invX = 1.0f / xMag;
	x0 *= invX; x1 *= invX; x2 *= invX;

	float dotXY = x0 * y0 + x1 * y1 + x2 * y2;
	y0 -= dotXY * x0;
	y1 -= dotXY * x1;
	y2 -= dotXY * x2;

	float yMag;
	arm_sqrt_f32(y0 * y0 + y1 * y1 + y2 * y2, &yMag);
	float invY = 1.0f / yMag;
	y0 *= invY; y1 *= invY; y2 *= invY;

	// Z = X cross Y
	float z0 = x1 * y2 - x2 * y1;
	float z1 = x2 * y0 - x0 * y2;
	float z2 = x0 * y1 - x1 * y0;

	R.data[0] = x0; R.data[1] = y0; R.data[2] = z0;
	R.data[3] = x1; R.data[4] = y1; R.data[5] = z1;
	R.data[6] = x2; R.data[7] = y2; R.data[8] = z2;
}

void InEKF::ApplyErrorState(const float delta[15]) {
	Vector3fMat w(InitType::Uninitialized);
	w.data[0] = delta[0];
	w.data[1] = delta[1];
	w.data[2] = delta[2];

	// Right-Invariant SO(3) retraction: R_new = ExpSO3(w) * R
	Matrix3f rDelta = LieGroups::ExpSO3(w);
	Matrix3f newR(InitType::Uninitialized);
	Matrix3f::Multiply3x3(rDelta, R, newR);
	R = newR;

	// Decoupled Euclidean retraction on R^3 (never rotates v or p around world origin!)
	for(int i = 0; i < 3; i++) {
		v.data[i] += delta[i + 3];
		p.data[i] += delta[i + 6];
		gyroBias.data[i] += delta[i + 9];
		accelBias.data[i] += delta[i + 12];
	}
}

void InEKF::MirrorCovarianceLower() {
	// UPDATE: Symmetrize the 5 diagonal 3x3 blocks (safety to ALWAYS make sure the covariance matrix is symmetric!)
	for(int b = 0; b < 5; b++) {
		float* d = P[b][b].data;
		d[1] = d[3] = 0.5f * (d[1] + d[3]);
		d[2] = d[6] = 0.5f * (d[2] + d[6]);
		d[5] = d[7] = 0.5f * (d[5] + d[7]);
	}

	// Mirror upper off-diagonal 3x3 blocks to lower triangle
	for(int r = 1; r < 5; r++) {
		for(int c = 0; c < r; c++) {
			Matrix3f::Transpose3x3(P[c][r], P[r][c]);
		}
	}
}