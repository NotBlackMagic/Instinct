/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/Estimation/inEKF.cpp
 */

 #include "inEKF.hpp"

 InEKF::InEKF() : 
	lastTimestampUs(0),
	isInitialized(false),
	isConverged(false),
	sampleCount(0),
	// Local Gravity in NED frame (Positive Z points straight down into the Earth)
	gravityNed([]() { Vector3fMat m; m.data[2] = 9.80665f; return m; }()),
	// Expected local Earth Magnetic field vector reference normalized to ~0.5 Gauss
	magRefNed([]() { Vector3fMat m; m.data[0] = 0.5f; return m; }()) 
{
	// Initialize state matrices to identities/zeroes
	R.SetIdentity();
	memset(v.data, 0, sizeof(v.data));
	memset(p.data, 0, sizeof(p.data));
	memset(gyroBias.data, 0, sizeof(gyroBias.data));
	memset(accelBias.data, 0, sizeof(accelBias.data));

	// Initialize Covariance Matrix with a conservative confidence baseline
	for(int r = 0; r < 5; r++) {
		for(int c = 0; c < 5; c++) {
			if(r == c) {
				P[r][c].SetIdentity();
			}
			else {
				memset(P[r][c].data, 0, sizeof(Matrix3f));
			}
		}
	}

	// Set initial diagonal uncertainties directly into the blocks
	P[0][0].data[0] = P[0][0].data[4] = P[0][0].data[8] = 0.01f;	// Attitude
	P[1][1].data[0] = P[1][1].data[4] = P[1][1].data[8] = 0.1f;		// Velocity
	P[2][2].data[0] = P[2][2].data[4] = P[2][2].data[8] = 0.5f;		// Position
	P[3][3].data[0] = P[3][3].data[4] = P[3][3].data[8] = 0.001f;	// Gyro bias
	P[4][4].data[0] = P[4][4].data[4] = P[4][4].data[8] = 0.01f;	// Accel bias

	// Define sensor noise variances (derived from device datasheets)
	gyroNoiseVar = 0.0001f;		// rad/s Noise Floor
	accelNoiseVar = 0.01f;		// m/s^2 Noise Floor
	gyroBiasWalkVar = 1e-6f;	// Gyro thermal stability drift rate
	accelBiasWalkVar = 1e-5f;	// Accel thermal stability drift rate
}

void InEKF::InitializeState(uint64_t timestampUs) {
	lastTimestampUs = timestampUs;
	isInitialized = true;
	isConverged = false;
	sampleCount = 0;
}

Status InEKF::InitializeMagReference(const float magInit[3]) {
	Vector3fMat magBody;
	magBody.data[0] = magInit[0];
	magBody.data[1] = magInit[1];
	magBody.data[2] = magInit[2];

	// Project raw boot magnetometer reading into the Global frame
	magRefNed = R * magBody;
	magRefNed.data[1] = 0.0f;	// Force East (Y) to 0, locking heading as North
	return Status::Ok;
}

void InEKF::NormalizeRotation() {
	Vector3fMat xCol(InitType::Uninitialized);
	Vector3fMat yCol(InitType::Uninitialized);
	Vector3fMat zCol(InitType::Uninitialized);

	// Extract columns
	xCol.data[0] = R.data[0];
	xCol.data[1] = R.data[3];
	xCol.data[2] = R.data[6];
	yCol.data[0] = R.data[1];
	yCol.data[1] = R.data[4];
	yCol.data[2] = R.data[7];

	// Normalize X
	float xMag = sqrtf(xCol.data[0]*xCol.data[0] + xCol.data[1]*xCol.data[1] + xCol.data[2]*xCol.data[2]);
	for(int i=0; i<3; i++) {
		xCol.data[i] = xCol.data[i] / xMag;
	}

	// Force Y orthogonal to X: y = y - (x dot y) * x
	float dotXY = (xCol.data[0]*yCol.data[0] + xCol.data[1]*yCol.data[1] + xCol.data[2]*yCol.data[2]);
	for(int i=0; i<3; i++) {
		yCol.data[i] = yCol.data[i] - (dotXY * xCol.data[i]);
	}

	// Normalize Y
	float yMag = sqrtf(yCol.data[0]*yCol.data[0] + yCol.data[1]*yCol.data[1] + yCol.data[2]*yCol.data[2]);
	for(int i=0; i<3; i++) {
		yCol.data[i] = yCol.data[i] / yMag;
	}

	// Z is exactly X cross Y
	zCol.data[0] = xCol.data[1]*yCol.data[2] - xCol.data[2]*yCol.data[1];
	zCol.data[1] = xCol.data[2]*yCol.data[0] - xCol.data[0]*yCol.data[2];
	zCol.data[2] = xCol.data[0]*yCol.data[1] - xCol.data[1]*yCol.data[0];

	// Write back to R
	R.data[0] = xCol.data[0]; R.data[1] = yCol.data[0]; R.data[2] = zCol.data[0];
	R.data[3] = xCol.data[1]; R.data[4] = yCol.data[1]; R.data[5] = zCol.data[1];
	R.data[6] = xCol.data[2]; R.data[7] = yCol.data[2]; R.data[8] = zCol.data[2];
}

Status InEKF::Predict(const float accel[3], const float gyro[3], uint64_t timestampUs) {
	if(isInitialized == false) {
		InitializeState(timestampUs);
		return Status::Ok;
	}

	float dt = (float)(timestampUs - lastTimestampUs) / 1000000.0f;
	lastTimestampUs = timestampUs;

	// Guard against erratic timing loops or thread execution stalls
	if(dt <= 0.0f || dt > 0.1f) {
		return Status::Ok;
	}

	// Remove estimated sensor biases from raw hardware measurements
	Vector3fMat wUnbiased(InitType::Uninitialized);
	wUnbiased.data[0] = gyro[0] - gyroBias.data[0];
	wUnbiased.data[1] = gyro[1] - gyroBias.data[1];
	wUnbiased.data[2] = gyro[2] - gyroBias.data[2];

	Vector3fMat aUnbiased(InitType::Uninitialized);
	aUnbiased.data[0] = accel[0] - accelBias.data[0];
	aUnbiased.data[1] = accel[1] - accelBias.data[1];
	aUnbiased.data[2] = accel[2] - accelBias.data[2];

	// Continuous-Time Kinematic Manifold Integration
	Vector3fMat wStep(InitType::Uninitialized);
	Matrix3f::MultiplyScalarVector3(wUnbiased, dt, wStep);
	Matrix3f expR = LieGroups::ExpSO3(wStep);

	// Rotate local acceleration vector into the global frame
	Vector3fMat aGlobal(InitType::Uninitialized);
	Matrix3f::Multiply3x3Vector3(R, aUnbiased, aGlobal);

	// Cache parameters for the upcoming covariance transition mapping
	Matrix3f oldR = R;
	Vector3fMat oldV = v;
	Vector3fMat oldP = p;

	// Update Kinematics
	Matrix3f newR(InitType::Uninitialized);
	Matrix3f::Multiply3x3(R, expR, newR);
	R = newR;

	float dtSqHalf = 0.5f * dt * dt;
	for(int i = 0; i < 3; i++) {
		float aNed = aGlobal.data[i] + gravityNed.data[i];
		v.data[i] = oldV.data[i] + (aNed * dt);
		p.data[i] = oldP.data[i] + (oldV.data[i] * dt) + (aNed * dtSqHalf);
	}

	// Construct the 15x15 Discrete Error State Transition Matrix (Phi)
	Matrix3f gSkew = LieGroups::Skew(gravityNed);
	Matrix3f vSkew = LieGroups::Skew(oldV);
	Matrix3f pSkew = LieGroups::Skew(oldP);

	Matrix3f gSkewDt(InitType::Uninitialized);
	Matrix3f vR_tmp(InitType::Uninitialized);
	Matrix3f vR(InitType::Uninitialized);
	Matrix3f pR_tmp(InitType::Uninitialized);
	Matrix3f pR(InitType::Uninitialized);
	Matrix3f negR_dt(InitType::Uninitialized);

	Matrix3f::MultiplyScalar3x3(gSkew, dt, gSkewDt);
	Matrix3f::Multiply3x3(vSkew, oldR, vR_tmp);
	Matrix3f::MultiplyScalar3x3(vR_tmp, -dt, vR);
	Matrix3f::Multiply3x3(pSkew, oldR, pR_tmp);
	Matrix3f::MultiplyScalar3x3(pR_tmp, -dt, pR);
	Matrix3f::MultiplyScalar3x3(oldR, -dt, negR_dt);

	// Propagate Covariance: P = (Phi * P * Phi^T) + Q
	// Memory is already laid out in blocks. Calculate Y = Phi * P directly.
	Matrix3f Y[5][5];
	for(int c = 0; c < 5; c++) {
		// Row 0
		Y[0][c] = P[0][c];
		Matrix3f::MultiplyAccumulate3x3(negR_dt, P[3][c], Y[0][c]);

		// Row 1
		Y[1][c] = P[1][c];
		Matrix3f::MultiplyAccumulate3x3(gSkewDt, P[0][c], Y[1][c]);
		Matrix3f::MultiplyAccumulate3x3(vR, P[3][c], Y[1][c]);
		Matrix3f::MultiplyAccumulate3x3(negR_dt, P[4][c], Y[1][c]);

		// Row 2
		// Since MultiplyScalar doesn't have a MAC version, we just do it manually or write to Y first
		Matrix3f::MultiplyScalar3x3(P[1][c], dt, Y[2][c]);
		Matrix3f::Add3x3(P[2][c], Y[2][c], Y[2][c]);
		Matrix3f::MultiplyAccumulate3x3(pR, P[3][c], Y[2][c]);

		// Row 3 & 4
		Y[3][c] = P[3][c];
		Y[4][c] = P[4][c];
	}

	// Compute Transposed Blocks for Phi^T
	Matrix3f negR_dt_T(InitType::Uninitialized), gSkewDt_T(InitType::Uninitialized);
	Matrix3f vR_T(InitType::Uninitialized), pR_T(InitType::Uninitialized);
	Matrix3f::Transpose3x3(negR_dt, negR_dt_T);
	Matrix3f::Transpose3x3(gSkewDt, gSkewDt_T);
	Matrix3f::Transpose3x3(vR, vR_T);
	Matrix3f::Transpose3x3(pR, pR_T);

	// Compute P_new = Y * Phi^T block-by-block directly into P
	for(int r = 0; r < 5; r++) {
		// Col 0
		if(0 >= r) {
			P[r][0] = Y[r][0];
			Matrix3f::MultiplyAccumulate3x3(Y[r][3], negR_dt_T, P[r][0]);
		}
		
		// Col 1
		if(1 >= r) {
			P[r][1] = Y[r][1];
			Matrix3f::MultiplyAccumulate3x3(Y[r][0], gSkewDt_T, P[r][1]);
			Matrix3f::MultiplyAccumulate3x3(Y[r][3], vR_T, P[r][1]);
			Matrix3f::MultiplyAccumulate3x3(Y[r][4], negR_dt_T, P[r][1]);
		}

		// Col 2
		if(2 >= r) {
			Matrix3f::MultiplyScalar3x3(Y[r][1], dt, P[r][2]);
			Matrix3f::Add3x3(Y[r][2], P[r][2], P[r][2]);
			Matrix3f::MultiplyAccumulate3x3(Y[r][3], pR_T, P[r][2]);
		}

		// Col 3 & 4
		if(3 >= r) {
			P[r][3] = Y[r][3];
		}
		if(4 >= r) {
			P[r][4] = Y[r][4];
		}
	}

	// Mirror upper triangle to lower triangle using hardware Transpose
	for(int r = 1; r < 5; r++) {
		for(int c = 0; c < r; c++) {
			Matrix3f::Transpose3x3(P[c][r], P[r][c]);
		}
	}

	// Process Noise (Q) applied to diagonal blocks
	float dtQ[5] = { gyroNoiseVar * dt, accelNoiseVar * dt, 1e-5f * dt, gyroBiasWalkVar * dt, accelBiasWalkVar * dt };
	static const float maxP[5] = {0.1f, 1.0f, 10.0f, 0.01f, 0.01f};

	for(int b = 0; b < 5; b++) {
		// Add Noise
		P[b][b].data[0] = P[b][b].data[0] + dtQ[b];
		P[b][b].data[4] = P[b][b].data[4] + dtQ[b];
		P[b][b].data[8] = P[b][b].data[8] + dtQ[b];
		
		// Clamp Diagonals to prevent unbounded float32 growth
		P[b][b].data[0] = fmaxf(1e-6f, P[b][b].data[0]);
		P[b][b].data[4] = fmaxf(1e-6f, P[b][b].data[4]);
		P[b][b].data[8] = fmaxf(1e-6f, P[b][b].data[8]);
	}

	// // In a Generic EKF: Only clamp states if they drift dangerously close to float limits.
	// // If you have no GPS, the Position Controller shouldn't be using these values anyway.
	// const float maxPos = 10000.0f;	// 10km
	// const float maxVel = 100.0f;		// 100m/s

	// // X and Y
	// if(fabsf(p.data[0]) > maxPos) p.data[0] = 0.0f; // Reset if drifting to infinity
	// if(fabsf(p.data[1]) > maxPos) p.data[1] = 0.0f;
	// if(fabsf(v.data[0]) > maxVel) v.data[0] = 0.0f;
	// if(fabsf(v.data[1]) > maxVel) v.data[1] = 0.0f;

	// // Z (In case Baro fails or is disabled)
	// if(fabsf(p.data[2]) > maxPos) p.data[2] = 0.0f;
	// if(fabsf(v.data[2]) > maxVel) v.data[2] = 0.0f;

	// Matrix Orthogonalization Hook: Periodically snap the manifold back to a perfect SO(3) rotation matrix
	if(sampleCount % 100 == 0) {
		this->NormalizeRotation();
	}

	// Establish a convergence threshold (approx. 1 second of stable updates)
	if(sampleCount < 8000) {
		sampleCount++;
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

	Vector3fMat y(InitType::Uninitialized);
	y.data[0] = accel[0];
	y.data[1] = accel[1];
	y.data[2] = accel[2];

	// Gating: Only fuse if drone is experiencing approx 1G (not violently maneuvering)
	float normSq = y.data[0]*y.data[0] + y.data[1]*y.data[1] + y.data[2]*y.data[2];
	float gSq = 9.80665f * 9.80665f;
	if(normSq < (gSq * 0.64f) || normSq > (gSq * 1.44f)) {
		return Status::Ok;
	}

	// Accelerometers measure Specific Force (f = a - g). Static expected measurement is -g.
	Vector3fMat expectedSpecificForce(InitType::Uninitialized);
	expectedSpecificForce.data[0] = -gravityNed.data[0];
	expectedSpecificForce.data[1] = -gravityNed.data[1];
	expectedSpecificForce.data[2] = -gravityNed.data[2];

	// Expected specific force vector mapped into the body frame
	Vector3fMat yExpected(InitType::Uninitialized);
	Matrix3f::MultiplyTransposed3x3Vector3(R, expectedSpecificForce, yExpected);

	// Add the current bias estimate to the predicted raw measurement
	yExpected.data[0] = yExpected.data[0] + accelBias.data[0];
	yExpected.data[1] = yExpected.data[1] + accelBias.data[1];
	yExpected.data[2] = yExpected.data[2] + accelBias.data[2];

	// Innovation (Residual)
	Vector3fMat z(InitType::Uninitialized);
	z.data[0] = y.data[0] - yExpected.data[0];
	z.data[1] = y.data[1] - yExpected.data[1];
	z.data[2] = y.data[2] - yExpected.data[2];

	// Construct Measurement Matrix block mapping
	// H = [hAttitude | 0 | 0 | 0 | I_3x3]
	Matrix3f rT(InitType::Uninitialized);
	Matrix3f::Transpose3x3(R, rT);
	Matrix3f hAttitude(InitType::Uninitialized);
	Matrix3f::Multiply3x3(rT, LieGroups::Skew(expectedSpecificForce), hAttitude);

	// Compute H * P: An array of five 3x3 blocks
	Matrix3f HP[5];
	for(int c = 0; c < 5; c++) {
		HP[c] = P[4][c]; 
		Matrix3f::MultiplyAccumulate3x3(hAttitude, P[0][c], HP[c]);
	}

	// Compute S = (H * P) * H^T + R_Noise
	// S = HP[0] * hAttitude^T + HP[4] * I^T + R_Noise
	Matrix3f hAttitudeT(InitType::Uninitialized);
	Matrix3f::Transpose3x3(hAttitude, hAttitudeT);

	Matrix3f S = HP[4];
	Matrix3f::MultiplyAccumulate3x3(HP[0], hAttitudeT, S);

	float rNoise = 0.5f; // Adjust this higher if the attitude is too twitchy
	S.data[0] = S.data[0] + rNoise;
	S.data[4] = S.data[4] + rNoise;
	S.data[8] = S.data[8] + rNoise;

	Matrix3f sInv = S.Inverse3x3();

	// Innovation Gating: Mahalanobis distance for 3D vectors: z^T * S_inv * z
	static Vector3fMat sInvZ(InitType::Uninitialized);
	Matrix3f::Multiply3x3Vector3(sInv, z, sInvZ);
	float mahalanobisSq =	(z.data[0] * sInvZ.data[0]) + 
							(z.data[1] * sInvZ.data[1]) + 
							(z.data[2] * sInvZ.data[2]);
	// A threshold of 11.3 means anything outside 3 standard deviations is rejected
	if(mahalanobisSq > 11.3f) {
		return Status::Ok; // Reject outlier
	}

	// Compute K = (P * H^T) * S_inv
	Matrix3f K[5];
	for(int r = 0; r < 5; r++) {
		Matrix3f tmpK = P[r][4];
		Matrix3f::MultiplyAccumulate3x3(P[r][0], hAttitudeT, tmpK);
		Matrix3f::Multiply3x3(tmpK, sInv, K[r]);
	}

	// Compute Error Correction Vector (Delta)
	static Vector15fMat delta(InitType::Uninitialized);
	for(int r = 0; r < 5; r++) {
		Vector3fMat delta_block(InitType::Uninitialized);
		Matrix3f::Multiply3x3Vector3(K[r], z, delta_block);
		
		delta.data[r * 3 + 0] = delta_block.data[0];
		delta.data[r * 3 + 1] = delta_block.data[1];
		delta.data[r * 3 + 2] = delta_block.data[2];
	}

	// Apply corrections to the manifold state space
	Vector9fMat kinematicDelta(InitType::Uninitialized);
	for(int i = 0; i < 9; i++) {
		kinematicDelta.data[i] = delta.data[i];
	}

	Matrix5f correctionX(InitType::Uninitialized);
	LieGroups::ExpSE23(kinematicDelta, correctionX);

	Matrix3f rDelta(InitType::Uninitialized);
	Vector3fMat ju(InitType::Uninitialized);
	Vector3fMat jv(InitType::Uninitialized);
	for(int r = 0; r < 3; r++) {
		for(int c = 0; c < 3; c++) {
			rDelta.data[r * 3 + c] = correctionX.data[r * 5 + c];
		}
		ju.data[r] = correctionX.data[r * 5 + 3];
		jv.data[r] = correctionX.data[r * 5 + 4];
	}

	Matrix3f newR(InitType::Uninitialized);
	Matrix3f::Multiply3x3(rDelta, R, newR);
	R = newR;

	Vector3fMat rV(InitType::Uninitialized);
	Vector3fMat rP(InitType::Uninitialized);
	Matrix3f::Multiply3x3Vector3(rDelta, v, rV);
	Matrix3f::Multiply3x3Vector3(rDelta, p, rP);

	for(int i = 0; i < 3; i++) {
		v.data[i] = rV.data[i] + ju.data[i];
		p.data[i] = rP.data[i] + jv.data[i];
		gyroBias.data[i] = gyroBias.data[i] + delta.data[i + 9];
		accelBias.data[i] = accelBias.data[i] + delta.data[i + 12];
	}

	// Covariance Update P = P - K*(H*P)
	// Calculate ONLY the upper triangle (c >= r)
	for(int r = 0; r < 5; r++) {
		for(int c = r; c < 5; c++) {
			Matrix3f::MultiplySubtract3x3(K[r], HP[c], P[r][c]);
		}
	}

	// Mirror upper triangle to lower triangle using hardware Transpose
	for(int r = 1; r < 5; r++) {
		for(int c = 0; c < r; c++) {
			Matrix3f::Transpose3x3(P[c][r], P[r][c]);
		}
	}

	return Status::Ok;
}

Status InEKF::UpdateMag(const float mag[3]) {
	if(isInitialized == false) {
		return Status::Ok;
	}

	Vector3fMat y(InitType::Uninitialized);
	y.data[0] = mag[0];
	y.data[1] = mag[1];
	y.data[2] = mag[2];

	// Compute expected magnetic reading in the body frame
	Vector3fMat yExpected(InitType::Uninitialized);
	Matrix3f::MultiplyTransposed3x3Vector3(R, magRefNed, yExpected);

	// Innovation calculation (Residual)
	Vector3fMat z(InitType::Uninitialized);
	z.data[0] = y.data[0] - yExpected.data[0];
	z.data[1] = y.data[1] - yExpected.data[1];
	z.data[2] = y.data[2] - yExpected.data[2];

	// Construct Measurement Matrix block mapping
	// H = [hMagAttitude | 0 | 0 | 0 | 0]
	Matrix3f rT(InitType::Uninitialized);
	Matrix3f::Transpose3x3(R, rT);
	Matrix3f hMagAttitude(InitType::Uninitialized);
	Matrix3f::Multiply3x3(rT, LieGroups::Skew(magRefNed), hMagAttitude);

	// Compute H * P: An array of five 3x3 blocks
	Matrix3f HP[5];
	for(int c = 0; c < 5; c++) {
		// Since H is zero everywhere except block 0, HP[c] = hMagAttitude * P[0][c]
		Matrix3f::Multiply3x3(hMagAttitude, P[0][c], HP[c]);
	}

	// Compute S = (H * P) * H^T + R_Noise
	Matrix3f hMagAttitudeT(InitType::Uninitialized);
	Matrix3f::Transpose3x3(hMagAttitude, hMagAttitudeT);

	Matrix3f S(InitType::Uninitialized);
	Matrix3f::Multiply3x3(HP[0], hMagAttitudeT, S);

	float rNoise = 0.04f;	// 0.2 Gauss standard deviation squared
	S.data[0] = S.data[0] + rNoise;
	S.data[4] = S.data[4] + rNoise;
	S.data[8] = S.data[8] + rNoise;

	Matrix3f sInv = S.Inverse3x3();

	// Innovation Gating: Mahalanobis distance for 3D vectors: z^T * S_inv * z
	static Vector3fMat sInvZ(InitType::Uninitialized);
	Matrix3f::Multiply3x3Vector3(sInv, z, sInvZ);
	float mahalanobisSq =	(z.data[0] * sInvZ.data[0]) + 
							(z.data[1] * sInvZ.data[1]) + 
							(z.data[2] * sInvZ.data[2]);
	// A threshold of 11.3 means anything outside 3 standard deviations is rejected
	if(mahalanobisSq > 11.3f) {
		return Status::Ok;	// Reject outlier
	}

	// Matrix<15, 3> K = P * H_T * S_inv;
	Matrix3f K[5];
	for(int r = 0; r < 5; r++) {
		Matrix3f tmpK(InitType::Uninitialized);
		Matrix3f::Multiply3x3(P[r][0], hMagAttitudeT, tmpK);
		Matrix3f::Multiply3x3(tmpK, sInv, K[r]);
	}

	// Compute Error Correction Vector (Delta)
	static Vector15fMat delta(InitType::Uninitialized);
	for(int r = 0; r < 5; r++) {
		Vector3fMat deltaBlock(InitType::Uninitialized);
		Matrix3f::Multiply3x3Vector3(K[r], z, deltaBlock);
		
		delta.data[r * 3 + 0] = deltaBlock.data[0];
		delta.data[r * 3 + 1] = deltaBlock.data[1];
		delta.data[r * 3 + 2] = deltaBlock.data[2];
	}

	// Apply corrections to the manifold state space
	Vector9fMat kinematicDelta(InitType::Uninitialized);
	for(int i = 0; i < 9; i++) {
		kinematicDelta.data[i] = delta.data[i];
	}

	Matrix5f correctionX(InitType::Uninitialized);
	LieGroups::ExpSE23(kinematicDelta, correctionX);

	Matrix3f rDelta(InitType::Uninitialized);
	Vector3fMat ju(InitType::Uninitialized);
	Vector3fMat jv(InitType::Uninitialized);
	for(int r = 0; r < 3; r++) {
		for(int c = 0; c < 3; c++) {
			rDelta.data[r * 3 + c] = correctionX.data[r * 5 + c];
		}
		ju.data[r] = correctionX.data[r * 5 + 3];
		jv.data[r] = correctionX.data[r * 5 + 4];
	}

	Matrix3f newR(InitType::Uninitialized);
	Matrix3f::Multiply3x3(rDelta, R, newR);
	R = newR;

	Vector3fMat rV(InitType::Uninitialized), rP(InitType::Uninitialized);
	Matrix3f::Multiply3x3Vector3(rDelta, v, rV);
	Matrix3f::Multiply3x3Vector3(rDelta, p, rP);

	for(int i = 0; i < 3; i++) {
		v.data[i] = rV.data[i] + ju.data[i];
		p.data[i] = rP.data[i] + jv.data[i];
		gyroBias.data[i] = gyroBias.data[i] + delta.data[i + 9];
		accelBias.data[i] = accelBias.data[i] + delta.data[i + 12];
	}

	// Covariance Update P = P - K*(H*P)
	// Calculate ONLY the upper triangle (c >= r)
	for(int r = 0; r < 5; r++) {
		for(int c = r; c < 5; c++) {
			Matrix3f::MultiplySubtract3x3(K[r], HP[c], P[r][c]);
		}
	}

	// Mirror upper triangle to lower triangle
	for(int r = 1; r < 5; r++) {
		for(int c = 0; c < r; c++) {
			Matrix3f::Transpose3x3(P[c][r], P[r][c]);
		}
	}

	return Status::Ok;
}

Status InEKF::UpdateBaro(float pressurePascal) {
	if(isInitialized == false) {
		return Status::Ok;
	}

	// Convert raw atmospheric pressure to a local altitude measurement
	// Standard ISA conversion benchmark: baseline 101325 Pa reference
	float measuredAltitude = 44330.77f * (1.0f - powf(pressurePascal / 101325.0f, 0.1902632f));

	// In the NED frame standard, Down is positive, meaning Altitude points along the negative axis
	float z = (-measuredAltitude) - p.data[2];	// Innovation error calculation
	float rNoise = 0.25f;						// 0.5 meter barometric noise deviation variance

	// Manifold Jacobian: H_att = -Skew(p). The 3rd row of -Skew(p) isolates the Z-axis attitude coupling
	float hy = p.data[1];
	float hx = -p.data[0];

	// Compute H * P (which evaluates to a 1x15 row vector)
	float hpRow[15];
	for(int c = 0; c < 5; c++) {
		hpRow[c * 3 + 0] = (hy * P[0][c].data[0]) + (hx * P[0][c].data[3]) + P[2][c].data[6];
		hpRow[c * 3 + 1] = (hy * P[0][c].data[1]) + (hx * P[0][c].data[4]) + P[2][c].data[7];
		hpRow[c * 3 + 2] = (hy * P[0][c].data[2]) + (hx * P[0][c].data[5]) + P[2][c].data[8];
	}

	// Compute S = H * P * H^T + R_Noise
	float sVal = (hpRow[0] * hy) + (hpRow[1] * hx) + hpRow[8] + rNoise;

	// Innovation Gating: z^2 / S gives the Mahalanobis distance squared
	// A threshold of 9.0 means anything outside 3 standard deviations is rejected
	if((z * z) / sVal > 9.0f) {
		return Status::Ok;	// Reject outlier
	}

	float invS = 1.0f / sVal;

	// K is the 8th column of P, scaled by invS.
	// Because P is mathematically symmetric, P * H^T is identical to (H * P)^T
	float kFlat[15];
	for(int i = 0; i < 15; i++) {
		kFlat[i] = hpRow[i] * invS;
	}

	// Compute Error Correction
	Vector15fMat delta(InitType::Uninitialized);
	for(int i = 0; i < 15; i++) {
		delta.data[i] = kFlat[i] * z;
	}

	// Extract and map updates across the SE_2(3) Manifold structures
	Vector9fMat kinematicDelta(InitType::Uninitialized);
	for(int i = 0; i < 9; i++) {
		kinematicDelta.data[i] = delta.data[i];
	}

	Matrix5f correctionX(InitType::Uninitialized);
	LieGroups::ExpSE23(kinematicDelta, correctionX);

	Matrix3f rDelta(InitType::Uninitialized);
	Vector3fMat ju(InitType::Uninitialized), jv(InitType::Uninitialized);
	for(int r = 0; r < 3; r++) {
		for(int c = 0; c < 3; c++) {
			rDelta.data[r * 3 + c] = correctionX.data[r * 5 + c];
		}
		ju.data[r] = correctionX.data[r * 5 + 3];
		jv.data[r] = correctionX.data[r * 5 + 4];
	}

	Matrix3f newR(InitType::Uninitialized);
	Matrix3f::Multiply3x3(rDelta, R, newR);
	R = newR;

	Vector3fMat rV(InitType::Uninitialized), rP(InitType::Uninitialized);
	Matrix3f::Multiply3x3Vector3(rDelta, v, rV);
	Matrix3f::Multiply3x3Vector3(rDelta, p, rP);

	for(int i = 0; i < 3; i++) {
		v.data[i] = rV.data[i] + ju.data[i];
		p.data[i] = rP.data[i] + jv.data[i];
		gyroBias.data[i] = gyroBias.data[i] + delta.data[i + 9];
		accelBias.data[i] = accelBias.data[i] + delta.data[i + 12];
	}

	// Covariance Update P = P - K * (H * P)
	// Calculate ONLY the upper triangle (c >= r).
	for(int c = 0; c < 5; c++) {
		for(int r = 0; r <= c; r++) {
			for(int i = 0; i < 3; i++) {
				for(int j = 0; j < 3; j++) {
					P[r][c].data[i * 3 + j] -= kFlat[r * 3 + i] * hpRow[c * 3 + j];
				}
			}
		}
	}

	// Mirror upper triangle to lower triangle
	for(int r = 1; r < 5; r++) {
		for(int c = 0; c < r; c++) {
			Matrix3f::Transpose3x3(P[c][r], P[r][c]);
		}
	}

	return Status::Ok;
}

Quaternion InEKF::GetQuaternion() const {
	Quaternion q;
	float trace = R.data[0] + R.data[4] + R.data[8];
	if(trace > 0.0f) {
		float s = 0.5f / sqrtf(fmaxf(0.0f, trace + 1.0f));
		q.w = 0.25f / s;
		q.x = (R.data[7] - R.data[5]) * s;
		q.y = (R.data[2] - R.data[6]) * s;
		q.z = (R.data[3] - R.data[1]) * s;
	}
	else {
		if(R.data[0] > R.data[4] && R.data[0] > R.data[8]) {
			float s = 2.0f * sqrtf(fmaxf(0.0f, 1.0f + R.data[0] - R.data[4] - R.data[8]));
			q.w = (R.data[7] - R.data[5]) / s;
			q.x = 0.25f * s;
			q.y = (R.data[1] + R.data[4]) / s;
			q.z = (R.data[2] + R.data[8]) / s;
		}
		else if(R.data[4] > R.data[8]) {
			float s = 2.0f * sqrtf(fmaxf(0.0f, 1.0f + R.data[4] - R.data[0] - R.data[8]));
			q.w = (R.data[2] - R.data[6]) / s;
			q.x = (R.data[1] + R.data[4]) / s;
			q.y = 0.25f * s;
			q.z = (R.data[5] + R.data[7]) / s;
		}
		else {
			float s = 2.0f * sqrtf(fmaxf(0.0f, 1.0f + R.data[8] - R.data[0] - R.data[4]));
			q.w = (R.data[3] - R.data[1]) / s;
			q.x = (R.data[2] + R.data[8]) / s;
			q.y = (R.data[5] + R.data[7]) / s;
			q.z = 0.25f * s;
		}
	}
	return q;
}

Vector3f InEKF::GetVelocity() const {
	return {v.data[0], v.data[1], v.data[2]};
}

Vector3f InEKF::GetPosition() const {
	return {p.data[0], p.data[1], p.data[2]};
}

Vector3f InEKF::GetGyroBias() const {
	return {gyroBias.data[0], gyroBias.data[1], gyroBias.data[2]};
}

Vector3f InEKF::GetAccelBias() const {
	return {accelBias.data[0], accelBias.data[1], accelBias.data[2]};
}