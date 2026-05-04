/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Video/ov7670.cpp
 */

#include "ov7670.hpp"

Status OV7670::Init(const Config &config) {
	this->config = config;

	// Verify ID
	uint16_t manID = 0;
	Status status = this->ReadID(&manID);
	if(status != Status::Ok || manID != this->chipID) {
		return Status::Error;
	}
	sensorInfo.id = manID;

	// Reset device
	status = this->Reset();
	if(status != Status::Ok) {
		return status;
	}

	// Set Clock/frame rate
	status = this->SetFPS(this->config.fps);
	if(status != Status::Ok) {
		return status;
	}

	// Set Color Format/style
	status = this->SetFormat(this->config.format);
	if(status != Status::Ok) {
		return status;
	}

	// Set resolution
	status = this->SetResolution(this->config.width, this->config.height);
	if(status != Status::Ok) {
		return status;
	}
	sensorInfo.width = this->config.width;
	sensorInfo.height = this->config.height;

	// Set Test Pattern
	status = this->SetTestPattern(false);
	if(status != Status::Ok) {
		return status;
	}

	// WHY??? Magic color register...
	status = this->WriteRegister(OV7670::Register::RSVD, 0x84);
	if(status != Status::Ok) {
		return status;
	}

	// Decrease output drive capability to 1x (default is 2x). Improves horizontal banding and noise
	status = this->ModifyRegister(OV7670::Register::COM2, 0x03, 0x00);
	if(status != Status::Ok) {
		return status;
	}

	// Set PCLK clock edge. Improves horizontal banding and noise
	status = this->ModifyRegister(OV7670::Register::COM10, 0x30, 0x00);
	if(status != Status::Ok) {
		return status;
	}

	// Enable de-noise, and automatic white balance (AWB): 0x18
	// status = this->ModifyRegister(OV7670::Register::COM16, 0x38, 0x18);
	// if(status != Status::Ok) {
	// 	return status;
	// }

	// Enable 50Hz Banding filter
	status = this->SetBandingFilter(BandingFilter::Hz50);
	if(status != Status::Ok) {
		return status;
	}

	// Enable automatic black level calibration (ABLC)
	// status = this->ModifyRegister(OV7670::Register::ABLC1, 0x04, 0x04);
	// if(status != Status::Ok) {
	// 	return status;
	// }

	// Set Auto Exposure, Auto Gain, and Auto White Balance (enabled by default)
	// status = this->SetAutoExposure(true);
	// if(status != Status::Ok) {
	// 	return status;
	// }
	// status = this->SetAutoWhiteBalance(true);
	// if(status != Status::Ok) {
	// 	return status;
	// }

	// Load the magic tuning values to fix AWB color casts
	// status = this->ApplyAWBTuning();
	// if(status != Status::Ok) {
	// 	return status;
	// }
	
	return Status::Ok;
}

Status OV7670::Reset() {
	if(this->config.resetPin != nullptr) {
		this->config.resetPin->Write(0);
		tx_thread_sleep(10);
		this->config.resetPin->Write(1);
		tx_thread_sleep(10);
		return Status::Ok;
	}
	else {
		Status status = this->WriteRegister(OV7670::Register::COM7, 0x80);
		tx_thread_sleep(10);
		return status;
	}
}

Status OV7670::Start() {
	if(config.powerDownPin != nullptr) {
		config.powerDownPin->Write(0);
		tx_thread_sleep(5);
		return Status::Ok;
	} else {
		// Fallback to Clear Software Sleep
		return ModifyRegister(OV7670::Register::COM2, 0x10, 0x00);
	}
}

Status OV7670::Stop() {
	if(config.powerDownPin != nullptr) {
		config.powerDownPin->Write(1);
		return Status::Ok;
	}
	else {
		// Fallback to Software Sleep (COM2 bit 4)
		return this->ModifyRegister(OV7670::Register::COM2, 0x10, 0x10);
	}
}

Status OV7670::ReadID(uint16_t *id) {
	uint8_t pid, vid;
	if(this->ReadRegister(OV7670::Register::PID, pid) != Status::Ok) {
		return Status::Error;
	}
	if(this->ReadRegister(OV7670::Register::VER, vid) != Status::Ok) {
		return Status::Error;
	}
	*id = (pid << 8) + vid;
	return Status::Ok;
}

Status OV7670::SetResolution(uint16_t width, uint16_t height) {
	// Get closest resolution that can be used and is implemented
	OV7670::InternalResolution resolution = this->FindNearestResolution(width, height);

	if(resolution == OV7670::InternalResolution::UNKNOWN) {
		return Status::Error;
	}

	// Update config to used resolution
	switch (resolution) {
		case OV7670::InternalResolution::VGA:
			this->config.width = 640;
			this->config.height = 480;
			break;
		case OV7670::InternalResolution::QVGA:
			this->config.width = 320;
			this->config.height = 240;
			break;
		case OV7670::InternalResolution::QQVGA:
			this->config.width = 160;
			this->config.height = 120;
			break;
		case OV7670::InternalResolution::QQQVGA:
			this->config.width = 80;
			this->config.height = 60;
			break;
		default:
			break;
	}

	// Apply hardware scaling/resolution
	Status status = this->ApplyResolution(resolution);
	if(status != Status::Ok) {
		return status;
	}

	// Apply windowing
	status = this->ApplyWindow(0, 0, this->config.width, this->config.height);

	return status;
}

Status OV7670::SetFormat(PixelFormat format) {
	Status status = Status::Ok;
	switch(format) {
		case PixelFormat::BayerRaw8: {
			//Raw Bayer RGB
			status = this->ModifyRegister(OV7670::Register::COM7, 0x05, 0x01);
			if(status != Status::Ok) {
				return status;
			}
			status = this->ModifyRegister(OV7670::Register::COM15, 0x30, 0x00);
			break;
		}
		// case ColorFormat_ProcBayer: {
		// 	//Processed Bayer RGB
		// 	status = this->ModifyRegister(OV7670::Register::COM7, 0x05, 0x05);
		// 	status = this->ModifyRegister(OV7670::Register::COM15, 0x30, 0x00);
		// 	break;
		// }
		case PixelFormat::YUV422_YUYV: {
			// THIS FORMAT IS NOT WORKING, SEE COMMENT BELOW FOR COM13
			//YUV/YCbCr 4:2:2 (Default)
			status = this->ModifyRegister(OV7670::Register::COM7, 0x05, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			// Swap MSB-LSB order
			// status = this->ModifyRegister(OV7670::Register::COM3, 0x40, 0x40);
			// if(status != Status::Ok) {
			// 	return status;
			// }
			// UV swap: 00: YUYV; 01: YVYU; 10: UYVY; 11 VYUY
			uint8_t uvSwap = 0x00;	// YUYV
			status = this->ModifyRegister(OV7670::Register::TSLB, 0x08, (uvSwap & 0x02) << 2);
			if(status != Status::Ok) {
				return status;
			}
			// BUG: COM13 Bit 0 does nothing, does not flip U and V order... Bit 1 was also tested, same result. Always YVYU
			// status = this->ModifyRegister(OV7670::Register::COM13, 0x01, (uvSwap & 0x01));
			// if(status != Status::Ok) {
			// 	return status;
			// }
			status = this->ModifyRegister(OV7670::Register::COM15, 0x30, 0x00);
			break;
		}
		case PixelFormat::YUV422_YVYU: {
			//YUV/YCbCr 4:2:2 (Default)
			status = this->ModifyRegister(OV7670::Register::COM7, 0x05, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			// Swap MSB-LSB order
			// status = this->ModifyRegister(OV7670::Register::COM3, 0x40, 0x40);
			// if(status != Status::Ok) {
			// 	return status;
			// }
			// UV swap: 00: YUYV; 01: YVYU; 10: UYVY; 11 VYUY
			uint8_t uvSwap = 0x01;	// YVYU
			status = this->ModifyRegister(OV7670::Register::TSLB, 0x08, (uvSwap & 0x02) << 2);
			if(status != Status::Ok) {
				return status;
			}
			// BUG: COM13 Bit 0 does nothing, does not flip U and V order... Bit 1 was also tested, same result. Always YVYU
			// status = this->ModifyRegister(OV7670::Register::COM13, 0x01, (uvSwap & 0x01));
			// if(status != Status::Ok) {
			// 	return status;
			// }
			status = this->ModifyRegister(OV7670::Register::COM15, 0x30, 0x00);
			break;
		}
		// case ColorFormat_GRB422: {
		// 	//GRB 4:2:2
		// 	status = this->ModifyRegister(OV7670::Register::COM7, 0x05, 0x04);
		// 	status = this->ModifyRegister(OV7670::Register::COM15, 0x30, 0x00);
		// 	break;
		// }
		case PixelFormat::RGB565: {
			//RGB565
			status = this->ModifyRegister(OV7670::Register::COM7, 0x05, 0x04);
			if(status != Status::Ok) {
				return status;
			}
			status = this->ModifyRegister(OV7670::Register::COM15, 0x30, 0x10);
			if(status != Status::Ok) {
				return status;
			}
			status = this->ModifyRegister(OV7670::Register::RGB444, 0x02, 0x00); // RGB444: Clear Bit 1
			break;
		}
		// case ColorFormat_RGB555: {
		// 	//RGB555
		// 	status = this->ModifyRegister(OV7670::Register::COM7, 0x05, 0x04);
		// 	status = this->ModifyRegister(OV7670::Register::COM15, 0x30, 0x30);
		// 	break;
		// }
		default: {
			//YUV/YCbCr 4:2:2 (Default)
			status = this->ModifyRegister(OV7670::Register::COM7, 0x05, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->ModifyRegister(OV7670::Register::COM15, 0x30, 0x00);
			break;
		}
	}
	return status;
}

Status OV7670::SetFPS(uint32_t fps) {
	Status status = Status::Ok;
	// FPS = 1/tframe
	// tframe = 510 * tline
	// tline = 784 * tp
	// Raw data: tp = tpclk | YUV/RGB: tp = 2*tpclk
	// tpclk = 1 / PCLK

	// Config Internal Clock: Fclk = MCLK * PLL_MULT / (2 * (CLKRC + 1))

	// Below clocks for: VGA @ 30 fps
	// With MCLK = 16MHz: 16 * 6 / (2 * (1 + 1)) = 24MHz
	// this->WriteRegister(DBLV, 0x8A);		// PLL_MULT: x6
	// this->WriteRegister(CLKRC, 0x01);	// CLKRC: 1

	// With MCLK = 24MHz: Bypass PLL 
	// (OLD: 24 * 4 / (2 * (1 + 1)) = 24MHz) 
	status = this->WriteRegister(OV7670::Register::DBLV, 0x4A);	// PLL_MULT: x4
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(OV7670::Register::CLKRC, 0x01);	// CLKRC: 1
	return status;
}

Status OV7670::SetBrightness(int8_t value) {
	// Set Brightness (Default/normal: 0x00)
	return this->WriteRegister(OV7670::Register::BRIGHT, value);
}

Status OV7670::SetContrast(uint8_t value) {
	// Set Contrast (Default/normal: 0x40)
	return this->WriteRegister(OV7670::Register::CONTRAS, value);
}

Status OV7670::SetWhiteBalance(uint8_t redGain, uint8_t blueGain) {
	// Disable Auto White Balance (AWB)
	Status status = this->ModifyRegister(OV7670::Register::COM8, 0x02, 0x00);
	if(status != Status::Ok) {
		return status;
	}

	// Set the Red channel gain
	status = this->WriteRegister(OV7670::Register::RED, redGain);
	if(status != Status::Ok) {
		return status;
	}

	// Set the Blue channel gain
	return this->WriteRegister(OV7670::Register::BLUE, blueGain);
}

Status OV7670::SetManualExposure(uint16_t exposure) {
	// Disable Auto Exposure (AEC)
	Status status = this->ModifyRegister(Register::COM8, 0x01, 0x00);
	if(status != Status::Ok) {
		return status;
	}

	// The exposure value is mapped across 3 registers:
	// AECHH (0x16): AEC[15:10] mapped to bits 5:0
	// AECH (0x10): AEC[9:2] mapped to bits 7:0
	// COM1 (0x04): AEC[1:0] mapped to bits 1:0
	status = this->WriteRegister(Register::AECHH, (exposure >> 10) & 0x3F);
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(Register::AECH, (exposure >> 2) & 0xFF);
	if(status != Status::Ok) {
		return status;
	}
	return this->ModifyRegister(Register::COM1, 0x03, exposure & 0x03);
}

Status OV7670::SetMaxGain(GainCeiling ceiling) {
	// Enable Auto Gain (AGC) in COM8 (bit 2)
	Status status = this->ModifyRegister(Register::COM8, 0x04, 0x04);
	if(status != Status::Ok) {
		return status;
	}

	// Set the ceiling in COM9 bits [6:4]
	return this->ModifyRegister(Register::COM9, 0x70, static_cast<uint8_t>(ceiling));
}

Status OV7670::SetAutoExposure(bool enable) {
	// COM8 (0x13): Bit 0 is Auto Exposure (AEC), Bit 2 is Auto Gain (AGC).
	// Must always be set together for correct brightness.
	uint8_t val = enable ? 0x05 : 0x00;
	return this->ModifyRegister(OV7670::Register::COM8, 0x05, val);
}

Status OV7670::SetAutoWhiteBalance(bool enable) {
	uint8_t val = enable ? 0x02 : 0x00;
	return this->ModifyRegister(OV7670::Register::COM8, 0x02, val);
}

Status OV7670::SetBandingFilter(BandingFilter filter) {
	Status status = Status::Ok;
	switch (filter) {
		case BandingFilter::Off:
			status = this->ModifyRegister(OV7670::Register::COM8, 0x20, 0x00);
			break;
		case BandingFilter::Hz50:
			status = this->ModifyRegister(OV7670::Register::COM8, 0x20, 0x20);
			if(status != Status::Ok) {
				return status;
			}
			status = this->ModifyRegister(OV7670::Register::COM11, 0x18, 0x08);
			break;
		case BandingFilter::Hz60:
			status = this->ModifyRegister(OV7670::Register::COM8, 0x20, 0x20);
			if(status != Status::Ok) {
				return status;
			}
			status = this->ModifyRegister(OV7670::Register::COM11, 0x18, 0x00);
			break;
		case BandingFilter::Auto:
			status = this->ModifyRegister(OV7670::Register::COM8, 0x20, 0x20);
			if(status != Status::Ok) {
				return status;
			}
			status = this->ModifyRegister(OV7670::Register::COM11, 0x18, 0x10);
			break;
	}
	return status;
}

Status OV7670::SetOrientation(bool mirrorH, bool flipV) {
	uint8_t horizontal = mirrorH ? 0x01 : 0x00;
	uint8_t vertical = flipV ? 0x01 : 0x00;
	uint8_t mirror = ((vertical) << 4) + ((horizontal & 0x01) << 5);
	return this->ModifyRegister(OV7670::Register::MVFP, 0x30, mirror);
}

Status OV7670::SetTestPattern(bool enable) {
	Status status = Status::Ok;
	if(enable == true) {
		// Test pattern: DSP color bar
		status = this->ModifyRegister(OV7670::Register::SCALING_XSC, 0x80, 0x00);
		if(status != Status::Ok) {
			return status;
		}
		status = this->ModifyRegister(OV7670::Register::SCALING_YSC, 0x80, 0x00);
		if(status != Status::Ok) {
			return status;
		}
		status = this->ModifyRegister(OV7670::Register::COM17, 0x08, 0x08);
	}
	else {
		// No test pattern
		status = this->ModifyRegister(OV7670::Register::SCALING_XSC, 0x80, 0x00);
		if(status != Status::Ok) {
			return status;
		}
		status = this->ModifyRegister(OV7670::Register::SCALING_YSC, 0x80, 0x00);
		if(status != Status::Ok) {
			return status;
		}
		status = this->ModifyRegister(OV7670::Register::COM17, 0x08, 0x00);	
	}
	return status;
}

OV7670::InternalResolution OV7670::FindNearestResolution(uint16_t w, uint16_t h) {
	if(w >= 480) {
		return OV7670::InternalResolution::VGA;
	}
	else if(w >= 240) {
		return OV7670::InternalResolution::QVGA;
	}
	else if(w >= 120) {
		return OV7670::InternalResolution::QQVGA;
	}
	else if(w >= 60) {
		return OV7670::InternalResolution::QQQVGA;
	}
	return OV7670::InternalResolution::UNKNOWN;
}

Status OV7670::ApplyResolution(InternalResolution resolution) {
	Status status = Status::Ok;
	// Config Internal Clock: Fclk = MCLK * PLL_MULT / (2 * (CLKRC + 1))
	// With MCLK = 16MHz: 16 * 6 / (2 * (1 + 1)) = 24MHz (VGA @ 30 fps)
//	this->WriteRegister(OV7670::Register::DBLV, 0x8A);	// PLL_MULT: x6
//	this->WriteRegister(OV7670::Register::CLKRC, 0x01);	// CLKRC: 1

	// RGB565
//	this->ModifyRegister(OV7670::Register::COM7, 0x05, 0x04);
//	this->ModifyRegister(OV7670::Register::COM15, 0x30, 0x10);
	// YUV/YCbCr 4:2:2 (Default)
//	this->ModifyRegister(OV7670::Register::COM7, 0x05, 0x00);
//	this->ModifyRegister(OV7670::Register::COM15, 0x30, 0x00);

	uint8_t com3 = 0x00;
	uint8_t com14 = 0x00;
	uint8_t dcwctr = 0x00;
	uint8_t scaleXsc = 0x3A;
	uint8_t scaleYsc = 0x35;
	uint8_t pclkDiv = 0xF0;
	uint8_t pclkDly = 0x02;

	switch(resolution) {
		case OV7670::InternalResolution::VGA:
			// VGA (640x320)
			// Resolution is set by Down-Sampling and then Scaling (Zoom-Out)
			com3 = 0x00;	// Disable Down-Sampling; Disable Scaling (Zoom-Out)
			com14 = 0x00;	// Normal PCLK; Fixed Scaling Parameters; PCLK Div: by 1
			// Set Down-Sampling
			dcwctr = 0x00;		// Vertical Downsampling: by 1; Horizontal Downsampling: by 1
			// Set Scaling (Zoom-Out) not used, disabled
			scaleXsc = 0x3A;	// Horizontal Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			scaleYsc = 0x35;	// Vertical Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			// Re-adjust PCLK clock
			pclkDiv = 0xF0;	// Pixel Clock divider: When scaling = 1x to 1/2x -> 0; 1/2x to 1/4x -> 1; 1/4x to 1/8x -> 2; 1/8x to 1/16x -> 3
			pclkDly = 0x02;	// Pixel Clock Delay = (Original H size / Pixel Clock divider) - New H size (0x02 is default)
			break;
		case OV7670::InternalResolution::QVGA:
			// QVGA (320x240)
			// Resolution is set by Down-Sampling and then Scaling (Zoom-Out)
			com3 = 0x04;	// Enable Down-Sampling; Disable Scaling (Zoom-Out)
			com14 = 0x19;	// DCW and PCLK controlled; Manual Scaling Parameters; PCLK Div: by 2
			// Set Down-Sampling
			dcwctr = 0x11;		// Vertical Downsampling: by 2; Horizontal Downsampling: by 2
			// Set Scaling (Zoom-Out) not used, disabled
			scaleXsc = 0x3A;	// Horizontal Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			scaleYsc = 0x35;	// Vertical Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			// Re-adjust PCLK clock
			pclkDiv = 0xF1;	// Pixel Clock divider: When scaling = 1x to 1/2x -> 0; 1/2x to 1/4x -> 1; 1/4x to 1/8x -> 2; 1/8x to 1/16x -> 3
			pclkDly = 0x02;	// Pixel Clock Delay = (Original H size / Pixel Clock divider) - New H size (0x02 is default)
			break;
		case OV7670::InternalResolution::QQVGA:
			// QQVGA (160x120)
			// Resolution is set by Down-Sampling and then Scaling (Zoom-Out)
			com3 = 0x04;	// Enable Down-Sampling; Disable Scaling (Zoom-Out)
			com14 = 0x1A;	// DCW and PCLK controlled; Manual Scaling Parameters; PCLK Div: by 4
			// Set Down-Sampling
			dcwctr = 0x22;		// Vertical Downsampling: by 4; Horizontal Downsampling: by 4
			// Set Scaling (Zoom-Out) not used, disabled
			scaleXsc = 0x3A;	// Horizontal Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			scaleYsc = 0x35;	// Vertical Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			// Re-adjust PCLK clock
			pclkDiv = 0xF2;	// Pixel Clock divider: When scaling = 1x to 1/2x -> 0; 1/2x to 1/4x -> 1; 1/4x to 1/8x -> 2; 1/8x to 1/16x -> 3
			pclkDly = 0x02;	// Pixel Clock Delay = (Original H size / Pixel Clock divider) - New H size (0x02 is default)
			break;
		case OV7670::InternalResolution::QQQVGA:
			// QQQVGA (80x60)
			// Resolution is set by Down-Sampling and then Scaling (Zoom-Out)
			com3 = 0x04;	// Enable Down-Sampling; Disable Scaling (Zoom-Out)
			com14 = 0x1B;	// DCW and PCLK controlled; Manual Scaling Parameters; PCLK Div: by 8
			// Set Down-Sampling
			dcwctr = 0x33;		// Vertical Downsampling: by 8; Horizontal Downsampling: by 8
			// Set Scaling (Zoom-Out) not used, disabled
			scaleXsc = 0x3A;	// Horizontal Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			scaleYsc = 0x35;	// Vertical Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			// Re-adjust PCLK clock
			pclkDiv = 0xF3;	// Pixel Clock divider: When scaling = 1x to 1/2x -> 0; 1/2x to 1/4x -> 1; 1/4x to 1/8x -> 2; 1/8x to 1/16x -> 3
			pclkDly = 0x02;	// Pixel Clock Delay = (Original H size / Pixel Clock divider) - New H size (0x02 is default)
			break;
		case OV7670::InternalResolution::CIF:
			// CIF/SIF(PAL) (352x288)
			// Resolution is set by Down-Sampling and then Scaling (Zoom-Out)
			com3 = 0x08;	// Disable Down-Sampling; Enable Scaling (Zoom-Out)
			com14 = 0x11;	// DCW and PCLK controlled; Fixed Scaling Parameters; PCLK Div: by 2
			// Set Down-Sampling
//			this->WriteRegister(SCALING_DCWCTR, 0x00);		// Vertical Downsampling: by 1; Horizontal Downsampling: by 1
			// Set Scaling (Zoom-Out) not used, disabled
			scaleXsc = 0x3A;	// Horizontal Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			scaleYsc = 0x35;	// Vertical Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			// Re-adjust PCLK clock
			pclkDiv = 0xF1;	// Pixel Clock divider: When scaling = 1x to 1/2x -> 0; 1/2x to 1/4x -> 1; 1/4x to 1/8x -> 2; 1/8x to 1/16x -> 3
			pclkDly = 0x02;	// Pixel Clock Delay = (Original H size / Pixel Clock divider) - New H size (0x02 is default)
			break;
		case OV7670::InternalResolution::QCIF:
			// QCIF/QSIF(PAL) (176x144)
			// Resolution is set by Down-Sampling and then Scaling (Zoom-Out)
			com3 = 0x0C;	// Enable Down-Sampling; Enable Scaling (Zoom-Out)
			com14 = 0x11;	// DCW and PCLK controlled; Fixed Scaling Parameters; PCLK Div: by 2
			// Set Down-Sampling
			dcwctr = 0x11;		// Vertical Downsampling: by 2; Horizontal Downsampling: by 2
			// Set Scaling (Zoom-Out) not used, disabled
			scaleXsc = 0x3A;	// Horizontal Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			scaleYsc = 0x35;	// Vertical Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			// Re-adjust PCLK clock
			pclkDiv = 0xF1;	// Pixel Clock divider: When scaling = 1x to 1/2x -> 0; 1/2x to 1/4x -> 1; 1/4x to 1/8x -> 2; 1/8x to 1/16x -> 3
			pclkDly = 0x52;	// Pixel Clock Delay = (Original H size / Pixel Clock divider) - New H size (0x02 is default)
			break;
		case OV7670::InternalResolution::QQCIF:
			// QQCIF/QQSIF(PAL) (88x72)
			// Resolution is set by Down-Sampling and then Scaling (Zoom-Out)
			com3 = 0x0C;	// Enable Down-Sampling; Enable Scaling (Zoom-Out)
			com14 = 0x12;	// DCW and PCLK controlled; Fixed Scaling Parameters; PCLK Div: by 4
			// Set Down-Sampling
			dcwctr = 0x22;		// Vertical Downsampling: by 4; Horizontal Downsampling: by 4
			// Set Scaling (Zoom-Out) not used, disabled
			scaleXsc = 0x3A;	// Horizontal Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			scaleYsc = 0x35;	// Vertical Scaling Factor/Ratio = (0x20 / [6:0]); Between 1x and 0.5x
			// Re-adjust PCLK clock
			pclkDiv = 0xF2;	// Pixel Clock divider: When scaling = 1x to 1/2x -> 0; 1/2x to 1/4x -> 1; 1/4x to 1/8x -> 2; 1/8x to 1/16x -> 3
			pclkDly = 0x2A;	// Pixel Clock Delay = (Original H size / Pixel Clock divider) - New H size (0x02 is default)
			break;
		default:
			return Status::Error;
	}

	// Resolution is set by Down-Sampling and then Scaling (Zoom-Out)
	status = this->ModifyRegister(OV7670::Register::COM3, 0x0C, com3);
	if(status != Status::Ok) {
		return status;
	}
	status = this->ModifyRegister(OV7670::Register::COM14, 0x1F, com14);
	if(status != Status::Ok) {
		return status;
	}
	// Set Down-Sampling
	status = this->WriteRegister(OV7670::Register::SCALING_DCWCTR, dcwctr);
	if(status != Status::Ok) {
		return status;
	}
	// Set Scaling (Zoom-Out) not used, disabled
	status = this->ModifyRegister(OV7670::Register::SCALING_XSC, 0x7F, scaleXsc);
	if(status != Status::Ok) {
		return status;
	}
	status = this->ModifyRegister(OV7670::Register::SCALING_YSC, 0x7F, scaleYsc);
	if(status != Status::Ok) {
		return status;
	}
	// Re-adjust PCLK clock
	status = this->WriteRegister(OV7670::Register::SCALING_PCLK_DIV, pclkDiv);
	if(status != Status::Ok) {
		return status;
	}
	return this->WriteRegister(OV7670::Register::SCALING_PCLK_DELAY, pclkDly);
}

Status OV7670::ApplyWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h){
	Status status = Status::Ok;
	// OV7670 image sensor array is 656x488, but actually 784x510??, the of which 640x480 are real/active pixels
	// Offset values from: https://github.com/adafruit/Adafruit_OV7670/blob/master/src/ov7670.c
	// Also tested/adjsuted manaully and matches above code to
	uint16_t xOffset = 162;
	uint16_t yOffset = 9;
	
	// Calculate Hardware Coordinates with wrap-around
	uint16_t xStart = (x + xOffset);
	uint16_t xEnd = (x + xOffset + w) % 784;
	uint16_t yStart = (y + yOffset);
	uint16_t yEnd = (y + yOffset + h) % 510;

	// Set output window, not auto
	status = this->ModifyRegister(OV7670::Register::TSLB, 0x01, 0x00);
	if(status != Status::Ok) {
		return status;
	}

	// Set HSTART and HSTOP values
	status = this->WriteRegister(OV7670::Register::HSTART, (xStart >> 3));
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(OV7670::Register::HSTOP, (xEnd >> 3));
	if(status != Status::Ok) {
		return status;
	}
	status = this->ModifyRegister(OV7670::Register::HREF, 0x3F, ((xEnd & 0x07) << 3) | (xStart & 0x07));
	if(status != Status::Ok) {
		return status;
	}

	// Set VSTART and VSTOP values
	status = this->WriteRegister(OV7670::Register::VSTART, (yStart >> 2));
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(OV7670::Register::VSTOP, (yEnd >> 2));
	if(status != Status::Ok) {
		return status;
	}
	return this->ModifyRegister(OV7670::Register::VREF, 0x0F, ((yEnd & 0x03) << 2) | (yStart & 0x03));
}

Status OV7670::ApplyAWBTuning() {
	// These values are derived from the Linux kernel ov7670.c driver
	// They define the color space thresholds and matrices to prevent 
	// the AWB loop from settling on green or pink color casts.
	static constexpr RegisterValuePair awbConfig[] = {
		{ OV7670::Register::AWBC1, 0x0A },
		{ OV7670::Register::AWBC2, 0xF0 },
		{ OV7670::Register::AWBC3, 0x34 },
		{ OV7670::Register::AWBC4, 0x58 },
		{ OV7670::Register::AWBC5, 0x28 },
		{ OV7670::Register::AWBC6, 0x3A },
		{ OV7670::Register::AWBC7, 0x88 },
		{ OV7670::Register::AWBC8, 0x88 },
		{ OV7670::Register::AWBC9, 0x44 },
		{ OV7670::Register::AWBC10, 0x67 },
		{ OV7670::Register::AWBC11, 0x49 },
		{ OV7670::Register::AWBC12, 0x0E },
		{ OV7670::Register::AWBCTR3, 0x0A },
		{ OV7670::Register::AWBCTR2, 0x55 },
		{ OV7670::Register::AWBCTR1, 0x11 },
		{ OV7670::Register::AWBCTR0, 0x9F }
	};

	uint8_t arraySize = sizeof(awbConfig) / sizeof(awbConfig[0]);
	for (uint8_t i = 0; i < arraySize; i++) {
		Status status = this->WriteRegister(awbConfig[i].reg, awbConfig[i].value);
		if (status != Status::Ok) {
			return status;
		}
	}

	return Status::Ok;
}

Status OV7670::WriteRegister(Register reg, uint8_t value) {
	const uint8_t maxRetries = 3;
	Status status = Status::Error;

	// Do a few retires, due to using SCCB and not I2C there are issues with the ACK (Do-Not-Care in SCCB)
	for(uint8_t attempt = 0; attempt < maxRetries; attempt++) {
		this->buffer[0] = static_cast<uint8_t>(reg);
		this->buffer[1] = value;

		status = this->bus.TransferAsync(this->addr, I3C::TargetType::I2C, this->buffer, 2, nullptr, 0);
		if(status == Status::Ok) {
			status = this->bus.TransferWait(1000);
			if(status == Status::Ok) {
				return Status::Ok;
			}
		}

		tx_thread_sleep(1);
	}

	return status;
}

Status OV7670::ReadRegister(Register reg, uint8_t& value) {
	const uint8_t maxRetries = 3;
	Status status = Status::Error;

	// Do a few retires, due to using SCCB and not I2C there are issues with the ACK (Do-Not-Care in SCCB)
	for(uint8_t attempt = 0; attempt < maxRetries; attempt++) {
		this->buffer[0] = static_cast<uint8_t>(reg);
		status = this->bus.TransferAsync(this->addr, I3C::TargetType::I2C, this->buffer, 1, this->buffer, 0);
		if(status == Status::Ok) {
			status = this->bus.TransferWait(1000);
		}

		if(status != Status::Ok) {
			// Failed in write regsiter, loop around and retry
			tx_thread_sleep(1);
			continue;
		}

		status = this->bus.TransferAsync(this->addr, I3C::TargetType::I2C, nullptr, 0, this->buffer, 1);
		if(status == Status::Ok) {
			status = this->bus.TransferWait(1000);
			if(status == Status::Ok) {
				value = this->buffer[0];
				return Status::Ok;
			}
		}

		tx_thread_sleep(1);
	}
	
	return status;
}

Status OV7670::ModifyRegister(Register reg, uint8_t mask, uint8_t value) {
	uint8_t regVal;
	Status status = this->ReadRegister(reg, regVal);
	if(status != Status::Ok) {
		return status;
	}

	regVal &= ~mask;
	regVal |= (value & mask);

	return this->WriteRegister(reg, regVal);
}