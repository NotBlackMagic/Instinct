/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Vision/opticalFlow.cpp
 */

#include "opticalFlow.hpp"

OpticalFlow::OpticalFlow() :
	pyrIdx(0),
	isInitialized(false),
	hasPrev(false),
	timestamp(0),
	invFocalX(1.0f / 143.7f),
	invFocalY(1.0f / 143.7f)
{
	memset(this->pyramids, 0, sizeof(this->pyramids));

	// Precompute 5x5 patch centers in L0 (128x128) coordinates.
	// Multiples of 4 ensure L1 (/2) and L2 (/4) land on exact integer pixels.
	static constexpr uint8_t coords[gridCols] = {24, 44, 64, 84, 104};
	uint8_t idx = 0;
	for(uint8_t r = 0; r < gridRows; r++) {
		for(uint8_t c = 0; c < gridCols; c++) {
			grid[idx++] = {coords[c], coords[r]};
		}
	}
}

Status OpticalFlow::Init(const Config& config) {
	if(config.fx <= 1.0f || config.fy <= 1.0f || config.minValid == 0) {
		return Status::Error;
	}

	this->config = config;
	this->invFocalX = 1.0f / config.fx;
	this->invFocalY = 1.0f / config.fy;

	this->Reset();
	this->ResetStatistics();
	this->isInitialized = true;

	return Status::Ok;
}

void OpticalFlow::Reset() {
	this->pyrIdx = 0;
	this->hasPrev = false;
	this->timestamp = 0;
	this->output = {};
}

Status OpticalFlow::Process(const VisionFrame& frame, const float gyro[3]) {
	if(this->isInitialized == false || frame.startAddress == nullptr || frame.width != rawWidth || frame.height != rawHeight || frame.format != PixelFormat::Grayscale) {
		return Status::Error;
	}

	this->stats.processedFrames = this->stats.processedFrames + 1;

	Pyramid& currPyr = this->pyramids[this->pyrIdx];
	Pyramid& prevPyr = this->pyramids[this->pyrIdx ^ 1U];

	// Stream 256x256 DMA buffer once to build the 3-level pyramid inside L1 D-Cache
	this->BuildPyramid(frame.startAddress, currPyr);

	// Bootstrap first frame
	if(this->hasPrev == false) {
		this->timestamp = frame.timestampUs;
		this->hasPrev = true;
		this->pyrIdx ^= 1U;
		this->output.isValid = false;
		return Status::Ok;
	}

	float dt = (float)(frame.timestampUs - this->timestamp) * 1e-6f;
	this->timestamp = frame.timestampUs;

	// Guard against timer wrap, zero dt, or dropped camera frames (> 100ms)
	if(dt <= 1e-5f || dt > 0.1f) {
		this->pyrIdx ^= 1U;
		this->output.isValid = false;
		this->stats.rejectedFrames = this->stats.rejectedFrames + 1;
		return Status::Ok;
	}

	// Predict initial L0 pixel displacement using gyro angular rates (+X right, +Y down, +Z optical axis)
	float uPrior = this->config.fx * gyro[1] * dt;
	float vPrior = -this->config.fy * gyro[0] * dt;

	float validU[numPatches];
	float validV[numPatches];
	uint8_t count = 0;

	// Track fixed 5x5 patch grid coarse-to-fine (L2 -> L1 -> L0)
	for(uint8_t i = 0; i < numPatches; i++) {
		float cx = (float)this->grid[i].x;
		float cy = (float)this->grid[i].y;

		// Seed L2 (1/4 scale) with scaled gyro prediction
		float u = uPrior * 0.25f;
		float v = vPrior * 0.25f;

		// Level 2 (32x32)
		if(this->TrackPatch(prevPyr.l2, currPyr.l2, l2Width, cx * 0.25f, cy * 0.25f, u, v) == false) {
			continue;
		}

		// Level 1 (64x64)
		u = u * 2.0f;
		v = v * 2.0f;
		if(this->TrackPatch(prevPyr.l1, currPyr.l1, l1Width, cx * 0.5f, cy * 0.5f, u, v) == false) {
			continue;
		}

		// Level 0 (128x128)
		u = u * 2.0f;
		v = v * 2.0f;
		if(this->TrackPatch(prevPyr.l0, currPyr.l0, l0Width, cx, cy, u, v) == false) {
			continue;
		}

		validU[count] = u;
		validV[count] = v;
		count = count + 1;
	}

	this->stats.lastTrackPatches = count;
	this->SolveVelocity(validU, validV, count, dt, gyro);

	if(this->output.isValid == true) {
		this->stats.validFrames = this->stats.validFrames + 1;
	}
	else {
		this->stats.rejectedFrames = this->stats.rejectedFrames + 1;
	}

	// Swap ping-pong pyramid index
	this->pyrIdx ^= 1U;

	return Status::Ok;
}

void OpticalFlow::BuildPyramid(const uint8_t* src, Pyramid& dst) {
	Bin2x2(src, dst.l0, rawWidth, rawHeight);			// 256x256 -> 128x128
	Bin2x2(dst.l0, dst.l1, l0Width, l0Height);	// 128x128 -> 64x64
	Bin2x2(dst.l1, dst.l2, l1Width, l1Height);	// 64x64 -> 32x32
}

void OpticalFlow::Bin2x2(const uint8_t* src, uint8_t* dst, uint16_t srcW, uint16_t srcH) {
	uint16_t dstW = srcW >> 1U;
	uint16_t dstH = srcH >> 1U;

	for(uint16_t y = 0; y < dstH; y++) {
		const uint8_t* row0 = src + ((y * 2U) * srcW);
		const uint8_t* row1 = row0 + srcW;
		uint8_t* dstRow = dst + (y * dstW);

		// Cortex-M55 Helium MVE: Processes 32x2 input pixels into 16 output pixels per iteration
		for(uint16_t x = 0; x < srcW; x += 32U) {
			uint8x16x2_t r0 = vld2q_u8(row0 + x);
			uint8x16x2_t r1 = vld2q_u8(row1 + x);

			uint8x16_t avgH0 = vrhaddq_u8(r0.val[0], r0.val[1]);
			uint8x16_t avgH1 = vrhaddq_u8(r1.val[0], r1.val[1]);
			uint8x16_t avg = vrhaddq_u8(avgH0, avgH1);

			vst1q_u8(dstRow + (x >> 1U), avg);
		}
	}
}

bool OpticalFlow::TrackPatch(const uint8_t* prev, const uint8_t* curr, uint16_t stride, float cx, float cy, float& u, float& v) const {
	constexpr int32_t halfPatch = patchSize >> 1; // 4
	int32_t tlX = (int32_t)cx - halfPatch;
	int32_t tlY = (int32_t)cy - halfPatch;

	// Ensure 1-pixel border for central-difference spatial gradients
	if(tlX < 1 || tlY < 1 || (tlX + patchSize) >= (int32_t)(stride - 1) || (tlY + patchSize) >= (int32_t)(stride - 1)) {
		return false;
	}

	// Precompute 8x8 template and spatial gradients (Ix, Iy) on prev frame
	__attribute__((aligned(16))) int16_t patchT[patchSize * patchSize];
	__attribute__((aligned(16))) int16_t gradX[patchSize * patchSize];
	__attribute__((aligned(16))) int16_t gradY[patchSize * patchSize];

	int32_t sumGxx = 0;
	int32_t sumGxy = 0;
	int32_t sumGyy = 0;

	for(int32_t py = 0; py < patchSize; py++) {
		const uint8_t* pRow = prev + (tlY + py) * stride + tlX;
		const uint8_t* pRowUp = pRow - stride;
		const uint8_t* pRowDown = pRow + stride;

		// Using MVE
		int32_t rowOffset = py * patchSize;

		// Load 8 uint8_t pixels and widen directly to 8x int16_t lanes
		int16x8_t rCenter = vreinterpretq_s16_u16(vldrbq_u16(pRow));
		int16x8_t rLeft = vreinterpretq_s16_u16(vldrbq_u16(pRow - 1));
		int16x8_t rRight = vreinterpretq_s16_u16(vldrbq_u16(pRow + 1));
		int16x8_t rUp = vreinterpretq_s16_u16(vldrbq_u16(pRowUp));
		int16x8_t rDown = vreinterpretq_s16_u16(vldrbq_u16(pRowDown));

		// Pre-shift template to Q6 (<< 6) once outside the Gauss-Newton loop
		vst1q_s16(patchT + rowOffset, vshlq_n_s16(rCenter, 6));

		// 2x central spatial gradients
		int16x8_t gx = vsubq_s16(rRight, rLeft);
		int16x8_t gy = vsubq_s16(rDown, rUp);

		vst1q_s16(gradX + rowOffset, gx);
		vst1q_s16(gradY + rowOffset, gy);

		// Single-instruction 8-lane multiply-accumulate across vector
		sumGxx = vmladavaq_s16(sumGxx, gx, gx);
		sumGxy = vmladavaq_s16(sumGxy, gx, gy);
		sumGyy = vmladavaq_s16(sumGyy, gy, gy);
	}

	// Scale structure tensor by 0.25f to compensate for 2x central gradient
	float aXX = (float)sumGxx * 0.25f;
	float aXY = (float)sumGxy * 0.25f;
	float aYY = (float)sumGyy * 0.25f;

	// Fast Shi-Tomasi minimum eigenvalue gate: At coarse levels (L2/L1), relax minEig slightly due to box-filter smoothing
	float levelMinEig = (stride == l0Width) ? this->config.minEig : (this->config.minEig * 0.5f);
	float trace = aXX + aYY;
	if(trace < (2.0f * levelMinEig)) {
		// Fast trace pre-reject avoids sqrtf on flat patches
		return false;
	}

	float det = (aXX * aYY) - (aXY * aXY);
	if(det < (levelMinEig * (trace - levelMinEig)) || det < 1e-4f) {
		return false;
	}

	float invDet = 1.0f / det;
	float invHxx = aYY * invDet;
	float invHxy = -aXY * invDet;
	float invHyy = aXX * invDet;

	// Coarse levels (L2/L1) only need 2 iterations and 0.1px convergence. L0 does full refinement
	uint8_t targetIter = (stride == l0Width) ? maxIter : 2U;
	float targetEpsSq = (stride == l0Width) ? this->config.epsSq : 0.01f;

	// Inverse-Compositional Gauss-Newton iterations
	float maxCoord = (float)(stride - patchSize - 2);
	int32_t maxAbsResQ6 = (int32_t)(this->config.maxRes * 4096.0f);
	int32_t absResSum = 0;
	for(uint8_t iter = 0; iter < maxIter; iter++) {
		float curTlX = (float)tlX + u;
		float curTlY = (float)tlY + v;

		if(curTlX < 0.0f || curTlY < 0.0f || curTlX >= maxCoord || curTlY >= maxCoord) {
			return false;
		}

		int32_t intX = (int32_t)curTlX;
		int32_t intY = (int32_t)curTlY;
		float fracX = curTlX - (float)intX;
		float fracY = curTlY - (float)intY;

		// Precompute Q8 fixed-point bilinear weights once for the 8x8 patch
		int32_t iFracX = (int32_t)(fracX * 16.0f + 0.5f);
		int32_t iFracY = (int32_t)(fracY * 16.0f + 0.5f);
		int32_t w00 = (16 - iFracX) * (16 - iFracY);
		int32_t w01 = iFracX * (16 - iFracY);
		int32_t w10 = (16 - iFracX) * iFracY;
		int32_t w11 = iFracX * iFracY;

		int32_t bXFixed = 0;
		int32_t bYFixed = 0;
		absResSum = 0;

		// Vectorized 8x8 Bilinear Warp + Residual + Gradient Dot-Product (8 pixels per iteration)
		for(int32_t py = 0; py < patchSize; py++) {
			const uint8_t* cRow0 = curr + (intY + py) * stride + intX;
			const uint8_t* cRow1 = cRow0 + stride;
			int32_t rowOffset = py * patchSize;

			uint16x8_t c00 = vldrbq_u16(cRow0);
			uint16x8_t c01 = vldrbq_u16(cRow0 + 1);
			uint16x8_t c10 = vldrbq_u16(cRow1);
			uint16x8_t c11 = vldrbq_u16(cRow1 + 1);

			// Q8 bilinear interpolation (max 255 * 256 = 65280, fits in uint16x8_t without overflow)
			uint16x8_t warpedQ8 = vmulq_n_u16(c00, w00);
			warpedQ8 = vmlaq_n_u16(warpedQ8, c01, w01);
			warpedQ8 = vmlaq_n_u16(warpedQ8, c10, w10);
			warpedQ8 = vmlaq_n_u16(warpedQ8, c11, w11);

			// Shift Q8 -> Q6 (>> 2) so signed residual fits in int16x8_t [-16320, +16320]
			int16x8_t warpedQ6 = vreinterpretq_s16_u16(vshrq_n_u16(warpedQ8, 2));
			int16x8_t tRowQ6 = vld1q_s16(patchT + rowOffset);
			int16x8_t diffQ6 = vsubq_s16(tRowQ6, warpedQ6);

			int16x8_t gxRow = vld1q_s16(gradX + rowOffset);
			int16x8_t gyRow = vld1q_s16(gradY + rowOffset);

			bXFixed = vmladavaq_s16(bXFixed, diffQ6, gxRow);
			bYFixed = vmladavaq_s16(bYFixed, diffQ6, gyRow);
			absResSum = vaddvaq_s16(absResSum, vabsq_s16(diffQ6));
		}

		// Scale back to float: 1 / (64 [Q6] * 2 [central diff]) = 1 / 128.0f
		constexpr float invScale = 1.0f / 128.0f;
		float bX = (float)bXFixed * invScale;
		float bY = (float)bYFixed * invScale;

		// Solve 2x2 normal equations: dU = H^-1 * b
		float dU = (invHxx * bX) + (invHxy * bY);
		float dV = (invHxy * bX) + (invHyy * bY);

		u = u + dU;
		v = v + dV;

		if((dU * dU + dV * dV) < targetEpsSq) {
			break;
		}
	}

	return (absResSum <= maxAbsResQ6);
}

void OpticalFlow::SolveVelocity(const float* uArr, const float* vArr, uint8_t count, float dt, const float gyro[3]) {
	if(count < this->config.minValid) {
		this->output.validCount = count;
		this->output.quality = 0.0f;
		this->output.isValid = false;
		return;
	}

	float scratchU[numPatches];
	float scratchV[numPatches];
	memcpy(scratchU, uArr, count * sizeof(float));
	memcpy(scratchV, vArr, count * sizeof(float));

	// Coordinate-wise Median
	float medU = Median(scratchU, count);
	float medV = Median(scratchV, count);

	// Compute median of squared deviations (avoids sqrt calls)
	float devSq[numPatches];
	for(uint8_t i = 0; i < count; i++) {
		float du = uArr[i] - medU;
		float dv = vArr[i] - medV;
		devSq[i] = (du * du) + (dv * dv);
	}
	float medDevSq = Median(devSq, count);

	// Floor gate at 0.5 px to preserve sub-pixel averaging during hover
	// (2.5 * 1.4826)^2 = 13.738f; floor gateSq at 0.5^2 = 0.25f
	float gateSq = fmaxf(0.25f, 13.738f * medDevSq);

	float sumU = 0.0f;
	float sumV = 0.0f;
	uint8_t inliers = 0;

	for(uint8_t i = 0; i < count; i++) {
		if(devSq[i] <= gateSq) {
			sumU = sumU + uArr[i];
			sumV = sumV + vArr[i];
			inliers = inliers + 1;
		}
	}

	if(inliers < this->config.minValid) {
		this->output.validCount = inliers;
		this->output.quality = 0.0f;
		this->output.isValid = false;
		return;
	}

	float invInliers = 1.0f / (float)inliers;
	float meanU = sumU * invInliers;
	float meanV = sumV * invInliers;
	float invDt = 1.0f / dt;

	// Convert L0 pixel shift to optical angular rates (rad/s)
	float flowWx = -(meanV * this->invFocalY) * invDt;
	float flowWy = (meanU * this->invFocalX) * invDt;

	this->output.pixelShift = {meanU, meanV, 0.0f};
	this->output.flowRate = {flowWx, flowWy, 0.0f};
	this->output.bodyRate = {flowWx - gyro[0], flowWy - gyro[1], 0.0f};
	this->output.validCount = inliers;
	this->output.quality = (float)inliers / (float)numPatches;
	this->output.isValid = true;
}

float OpticalFlow::Median(float* buf, uint8_t count) {
	// Insertion sort on <= 25 elements is faster and more branch-predictable than Quickselect on Cortex-M
	for(uint8_t i = 1; i < count; i++) {
		float key = buf[i];
		int8_t j = (int8_t)i - 1;
		while(j >= 0 && buf[j] > key) {
			buf[j + 1] = buf[j];
			j = j - 1;
		}
		buf[j + 1] = key;
	}

	uint8_t mid = count >> 1U;
	if((count & 1U) != 0U) {
		return buf[mid];
	}
	return 0.5f * (buf[mid - 1U] + buf[mid]);
}