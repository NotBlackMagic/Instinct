/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	SDK/Drivers/Video/ov9281.cpp
 */

#include "ov9281.hpp"

constexpr uint32_t xclk = 24000000;
constexpr uint32_t sysClk = 64000000;	// sys_clk: XCLK (24 MHz) / 1.5 * 40 (VCO = 640 MHz) / 5 / 2 = 64 MHz

static constexpr OV9281::RegisterValuePair ov9281GlobalInit[] = {
	{OV9281::Register::SC_SOFTWARE_RESET,	0x01},
	// Clock config/PLL config: SYS: 640MHz
																		// 0x0300: PLL1 PRE DIV (Default: 0x01) (0: /1, 1: /1.5, 2: /2, 3: /2.5, 4: /3, 5: /4, 6: /6, 8: /8)
	{static_cast<OV9281::Register>(0x0301),	0x00},	// Bit[1:0]: PLL1 Multiplier [9:8]
	{static_cast<OV9281::Register>(0x0302),	0x28},	// Bit [7:0]: PLL1 Multiplier [7:0]
	{static_cast<OV9281::Register>(0x0303),	0x00},	// Bit [3:0] PLL1 M DIV (Default: 0x00) (/(1 + [3:0]))
																		// 0x0304: PLL1 MIPI DIV (Default: 0x03)
																		// 0x0305: PLL1 PRE SYS DIV (Default: 0x02)
																		// 0x0306: PLL1 SYS DIV (Default: 0x01)
																		// 0x0307: PLL1 DIV RST SYNC (Default: 0x00)
																		// 0x0308: PLL1 Bypass (Default: 0x00)
																		// 0x0309: PLL1 CP (Default: 0x01)
																		// 0x030A: PLL1 PRE DIV0 (Default: 0x00) (0: /1, 1: /2)
																		// 0x030B: PLL2 PRE DIV (Default: 0x04)
																		// 0x030C: PLL2 Multiplier[9:8] (Default: 0x00)
	{static_cast<OV9281::Register>(0x030D),	0x50},	// Bit [7:0]: PLL2 Multiplier [7:0] (Default: 0x50)
	{static_cast<OV9281::Register>(0x030E),	0x02},	// Bit [2:0]: PLL2 SYS DIV (Default: 0x02)

	{static_cast<OV9281::Register>(0x3001),	0x00},
	{static_cast<OV9281::Register>(0x3004),	0x00},
	{static_cast<OV9281::Register>(0x3005),	0x00},
	{OV9281::Register::SC_CTRL_06,			0x04},
	{static_cast<OV9281::Register>(0x3013),	0x18},
	{static_cast<OV9281::Register>(0x3022),	0x01},
	{static_cast<OV9281::Register>(0x3023),	0x00},
	{static_cast<OV9281::Register>(0x302C),	0x00},
	{static_cast<OV9281::Register>(0x302F),	0x00},
	{static_cast<OV9281::Register>(0x3030),	0x04},
	{static_cast<OV9281::Register>(0x303A),	0x00},
	{static_cast<OV9281::Register>(0x303F),	0x01},

	{static_cast<OV9281::Register>(0x3503),	0x08},
	{static_cast<OV9281::Register>(0x3505),	0x8C},
	{static_cast<OV9281::Register>(0x3507),	0x03},

	{static_cast<OV9281::Register>(0x3610),	0x80},
	{static_cast<OV9281::Register>(0x3611),	0xA0},
	{static_cast<OV9281::Register>(0x3620),	0x6F},
	{static_cast<OV9281::Register>(0x3632),	0x56},
	{static_cast<OV9281::Register>(0x3633),	0x78},
	{static_cast<OV9281::Register>(0x3662),	0x05},
	{static_cast<OV9281::Register>(0x3666),	0x00},
	{static_cast<OV9281::Register>(0x366F),	0x5A},
	{static_cast<OV9281::Register>(0x3680),	0x84},

	{static_cast<OV9281::Register>(0x3712),	0x80},
	{static_cast<OV9281::Register>(0x372D),	0x22},
	{static_cast<OV9281::Register>(0x3731),	0x80},
	{static_cast<OV9281::Register>(0x3732),	0x30},
	{static_cast<OV9281::Register>(0x377D),	0x22},
	{static_cast<OV9281::Register>(0x3788),	0x02},
	{static_cast<OV9281::Register>(0x3789),	0xA4},
	{static_cast<OV9281::Register>(0x378A),	0x00},
	{static_cast<OV9281::Register>(0x378B),	0x4A},
	{static_cast<OV9281::Register>(0x3799),	0x20},

	{static_cast<OV9281::Register>(0x3881),	0x42},
	{static_cast<OV9281::Register>(0x38B1),	0x00},
	{static_cast<OV9281::Register>(0x3920),	0xFF},

	{static_cast<OV9281::Register>(0x4010),	0x40},
	{static_cast<OV9281::Register>(0x4043),	0x40},

	{static_cast<OV9281::Register>(0x4307),	0x30},

	{static_cast<OV9281::Register>(0x4501),	0x00},
	{static_cast<OV9281::Register>(0x450A),	0x08},
	{static_cast<OV9281::Register>(0x4601),	0x04},

	{static_cast<OV9281::Register>(0x470F),	0x00},
	{static_cast<OV9281::Register>(0x4F07),	0x00},
	{static_cast<OV9281::Register>(0x4800),	0x00},

	{static_cast<OV9281::Register>(0x5000),	0x9F},
	{static_cast<OV9281::Register>(0x5001),	0x00},
	{OV9281::Register::PRE_CTRL00,			0x00},

	{static_cast<OV9281::Register>(0x5D00),	0x07},
	{static_cast<OV9281::Register>(0x5D01),	0x00}
};

Status OV9281::Init(const Config &config) {
	this->config = config;

	// Verify ID
	uint16_t manID = 0;
	Status status = this->ReadID(&manID);
	if(status != Status::Ok || manID != this->chipID) {
		return Status::Error;
	}
	this->sensorInfo.id = manID;

	// Reset device
	status = this->Reset();
	if(status != Status::Ok) {
		return status;
	}

	// Load Magic Global Initialization Blob
	uint16_t size = sizeof(ov9281GlobalInit) / sizeof(ov9281GlobalInit[0]);
	status = this->WriteRegisterArray(ov9281GlobalInit, size);
	if(status != Status::Ok) {
		return status;
	}
	tx_thread_sleep(300);

	// Configure trigger mode (External FSIN or Continuous)
	if(this->config.triggerMode == TriggerMode::External) {
		status = this->WriteRegister(Register::SC_CTRL_06, 0x00);
		if(status != Status::Ok) {
			return status;
		}
		status = this->WriteRegister(Register::TIMING_REG23, 0x30);
	}
	else {
		status = this->WriteRegister(Register::SC_CTRL_06, 0x02);
		if(status != Status::Ok) {
			return status;
		}
		status = this->WriteRegister(Register::TIMING_REG23, 0x00);
	}

	// Apply target resolution and frame geometry
	status = this->SetResolution(this->config.width, this->config.height);
	if(status != Status::Ok) {
		return status;
	}

	status = this->SetFormat(config.format);
	if(status != Status::Ok) {
		return status;
	}

	status = this->SetAnalogGain(0x10);
	if(status != Status::Ok) {
		return status;
	}

	// Apply frame rate
	if(this->config.fps > 0) {
		status = this->SetFPS(this->config.fps);
		if(status != Status::Ok) {
			return status;
		}
	}

	// Force DVP output and disable the MIPI PHY.
	// Enable IO output pads
	// Bit[1:0]: IO 17:16
	status = this->WriteRegister(static_cast<Register>(0x3004), 0x03);
	if(status != Status::Ok) {
		return status;
	}
	// Bit[7:0]: IO 15:8
	status = this->WriteRegister(static_cast<Register>(0x3005), 0xFF);
	if(status != Status::Ok) {
		return status;
	}
	// Bit[7:0]: IO 7:0
	status = this->WriteRegister(Register::SC_CTRL_06, 0xFF);
	if(status != Status::Ok) {
		return status;
	}
	// Bit[3]: MIPI Pad
	status = this->WriteRegister(static_cast<Register>(0x3011), 0x02);
	if(status != Status::Ok) {
		return status;
	}
	// Bit[4]: 0 DVP enable, 1 MIPI enable
	status = this->WriteRegister(static_cast<Register>(0x3039), 0x02);
	if(status != Status::Ok) {
		return status;
	}
	// Bit[2]: 0 DVP enable, 1 MIPI enable (and power down MIPI stuff)
	status = this->WriteRegister(Register::SC_MIPI_SC_CTRL0, 0x18);
	if(status != Status::Ok) {
		return status;
	}
	// Bit[0]: 1 DVP enable
	status = this->WriteRegister(Register::DVP_MODE, 0x01);
	if(status != Status::Ok) {
		return status;
	}
	// DVP Polarities
	status = this->WriteRegister(Register::POLARITY_CTRL, 0x09);
	if(status != Status::Ok) {
		return status;
	}

	return Status::Ok;
}

Status OV9281::Reset() {
	if(this->config.shutdownPin != nullptr) {
		this->config.shutdownPin->Write(0);
		tx_thread_sleep(2);
		this->config.shutdownPin->Write(1);
		tx_thread_sleep(1);
		return Status::Ok;
	}

	// Bit[0]: Software resset: 0 Off, 1 On
	Status status = this->WriteRegister(Register::SC_SOFTWARE_RESET, 0x01);
	tx_thread_sleep(10);
	return status;
}

Status OV9281::Start() {
	// Bit[0]: Mode select: 0 Standby, 1 Streaming
	return this->WriteRegister(Register::MODE_SELECT, 0x01);
}

Status OV9281::Stop() {
	return this->WriteRegister(Register::MODE_SELECT, 0x00);
}

Status OV9281::ReadID(uint16_t *id) {
	if(id == nullptr) {
		return Status::Error;
	}

	uint8_t idHigh = 0;
	uint8_t idLow = 0;

	Status status = this->ReadRegister(Register::SC_CHIP_ID_H, idHigh);
	if(status != Status::Ok) {
		return status;
	}
	status = this->ReadRegister(Register::SC_CHIP_ID_L, idLow);
	if(status != Status::Ok) {
		return status;
	}

	*id = (static_cast<uint16_t>(idHigh) << 8) | idLow;
	return Status::Ok;
}

Status OV9281::SetResolution(uint16_t width, uint16_t height) {
	InternalResolution res = this->FindNearestResolution(width, height);
	if(res == InternalResolution::UNKNOWN) {
		return Status::Error;
	}
	return this->ApplyResolution(res);
}

Status OV9281::SetFormat(PixelFormat format) {
	this->config.format = format;
	return Status::Ok;
}

Status OV9281::SetFPS(uint32_t fps) {
	if(fps == 0) {
		return Status::Error;
	}

	// FPS = fsys / (HTS * VTS)
	// VTS = fsys / (HTS * fps)

	uint16_t minVTS = 0x0208;
	switch(this->sensorInfo.height) {
		case 800:
		case 720:
			minVTS = 0x038E;	// 910 lines (800 active + 110 blanking)
			break;
		case 400:
		case 256:
			minVTS = 0x0208;	// 520 lines (400 active + 120 blanking)
			break;
		default:
			return Status::Error;
	}

	// Read the active HTS
	uint8_t htsHigh = 0;
	uint8_t htsLow = 0;
	Status status = this->ReadRegister(Register::HTS_H, htsHigh);
	if(status != Status::Ok) {
		return status;
	}
	status = this->ReadRegister(Register::HTS_L, htsLow);
	if(status != Status::Ok) {
		return status;
	}

	uint32_t hts = (static_cast<uint32_t>(htsHigh) << 8) | htsLow;
	if(hts == 0) {
		return Status::Error;
	}

	// Calculate target VTS = round(sysClk / (HTS * FPS))
	uint32_t vts = (sysClk + ((hts * fps) / 2)) / (hts * fps);

	// Clamp VTS
	if(vts < minVTS) {
		vts = minVTS;
	}
	if(vts > 0x7FFF) {
		vts = 0x7FFF;
	}

	status = this->WriteRegister(Register::VTS_H, static_cast<uint8_t>(vts >> 8));
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(Register::VTS_L, static_cast<uint8_t>(vts));
	if(status != Status::Ok) {
		return status;
	}
	
	// Record the actual achieved frame rate
	this->config.fps = sysClk / (hts * vts);

	return status;
}

Status OV9281::SetExposure(uint32_t exposure) {
	uint8_t vtsHigh;
	uint8_t vtsLow;
	Status status = this->ReadRegister(Register::VTS_H, vtsHigh);
	if(status != Status::Ok) {
		return status;
	}

	status = this->ReadRegister(Register::VTS_L, vtsLow);
	if(status != Status::Ok) {
		return status;
	}

	uint32_t vts = (static_cast<uint32_t>(vtsHigh) << 8) | static_cast<uint32_t>(vtsLow);
	if(exposure < 4) {
		exposure = 4;
	}

	uint32_t maximum = vts > 25 ? vts - 25 : 4;
	if(exposure > maximum) {
		exposure = maximum;
	}

	uint32_t value = exposure << 4;

	status = this->WriteRegister(Register::EXPO_H, static_cast<uint8_t>((value >> 16) & 0x0F));
	if(status != Status::Ok) {
		return status;
	}

	status = this->WriteRegister(Register::EXPO_M, static_cast<uint8_t>(value >> 8));
	if(status != Status::Ok) {
		return status;
	}

	return this->WriteRegister(Register::EXPO_L, static_cast<uint8_t>(value));
}

Status OV9281::SetAnalogGain(uint16_t gain) {
	// Clamp to valid sensor analog gain limits
	if(gain < 0x0010) {
		gain = 0x0010;
	}
	else if(gain > 0x00F8) {
		gain = 0x00F8;
	}

	// Bit [9:8] in DEBUG, Bit [7:0] in GAIN. Format is Q4 i.e. fixed-point with 4 fractional bits
	Status status = this->WriteRegister(Register::DEBUG, static_cast<uint8_t>((gain >> 8) & 0x07));
	if(status != Status::Ok) {
		return status;
	}
	return this->WriteRegister(Register::GAIN, static_cast<uint8_t>(gain & 0xFF));
}

Status OV9281::SetOrientation(bool mirrorH, bool flipV) {
	// TIMING_FORMAT1 (0x3820): Bit[2:1] controls vertical flip
	uint8_t vVal = flipV ? 0x04 : 0x00;
	Status status = this->ModifyRegister(Register::TIMING_FORMAT1, 0x04, vVal);
	if(status != Status::Ok) {
		return status;
	}

	// TIMING_FORMAT2 (0x3821): Bit[2] controls horizontal mirror
	uint8_t hVal = mirrorH ? 0x04 : 0x00;
	return this->ModifyRegister(Register::TIMING_FORMAT2, 0x04, hVal);
}

Status OV9281::SetTestPattern(bool enable) {
	// PRE_CTRL00: 0x80 enables 8-bar test pattern, 0x00 disables
	return this->WriteRegister(Register::PRE_CTRL00, enable ? 0x80 : 0x00);
}

OV9281::InternalResolution OV9281::FindNearestResolution(uint16_t w, uint16_t h) {
	if(w >= 1280 && h >= 800) {
		return InternalResolution::WXGA;
	}
	if(w >= 1280 && h >= 720) {
		return InternalResolution::HD;
	}
	if(w >= 640 && h >= 400) {
		return InternalResolution::VGA;
	}
	if(w >= 256 && h >= 256) {
		return InternalResolution::P256;
	}
	return InternalResolution::UNKNOWN;
}

Status OV9281::ApplyResolution(InternalResolution resolution) {
	Status status = this->Stop();
	if(status != Status::Ok) {
		return status;
	}

	switch(resolution) {
		case InternalResolution::WXGA:
			// Full array (1296 x 816)
			status = this->ApplyWindow(0, 0, 1296, 816);
			if(status != Status::Ok) {
				return status;
			}

			// Output dimensions: 1280 x 800
			status = this->WriteRegister(Register::X_OUTPUT_H, 0x05);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::X_OUTPUT_L, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_OUTPUT_H, 0x03);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_OUTPUT_L, 0x20);
			if(status != Status::Ok) {
				return status;
			}

			// Pixel binning off
			status = this->WriteRegister(static_cast<Register>(0x3778), 0x00);
			if(status != Status::Ok) {
				return status;
			}

			// DVP Line timing: HTS = 1456 (0x05B0)
			status = this->WriteRegister(Register::HTS_H, 0x05);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::HTS_L, 0xB0);
			if(status != Status::Ok) {
				return status;
			}

			// Base VTS = 910 (0x038E)
			status = this->WriteRegister(Register::VTS_H, 0x03);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::VTS_L, 0x8E);
			if(status != Status::Ok) {
				return status;
			}

			// Crop offsets (8 px margins: 1296-16=1280, 816-16=800)
			status = this->WriteRegister(Register::X_WIN_H, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::X_WIN_L, 0x08);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_WIN_H, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_WIN_L, 0x08);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::X_INC, 0x11);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_INC, 0x11);
			if(status != Status::Ok) {
				return status;
			}

			// Timing format but retain mirror/flip bits [2:1]
			status = this->ModifyRegister(Register::TIMING_FORMAT1, 0xF9, 0x40);
			if(status != Status::Ok) {
				return status;
			}
			status = this->ModifyRegister(Register::TIMING_FORMAT2, 0xFB, 0x00);
			if(status != Status::Ok) {
				return status;
			}

			// Full-resolution BLC windows
			status = this->WriteRegister(static_cast<Register>(0x4003), 0x40);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4008), 0x04);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4009), 0x0B);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x400C), 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x400D), 0x07);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4507), 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4509), 0x00);
			if(status != Status::Ok) {
				return status;
			}

			this->sensorInfo.width = 1280;
			this->sensorInfo.height = 800;
			break;

		case InternalResolution::HD:
			// Centered 720p vertical crop on sensor active array
			status = this->ApplyWindow(0, 40, 1280, 736);
			if(status != Status::Ok) {
				return status;
			}

			// Output dimensions: 1280 x 720
			status = this->WriteRegister(Register::X_OUTPUT_H, 0x05);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::X_OUTPUT_L, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_OUTPUT_H, 0x02);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_OUTPUT_L, 0xD0);
			if(status != Status::Ok) {
				return status;
			}

			// Pixel binning off
			status = this->WriteRegister(static_cast<Register>(0x3778), 0x00);
			if(status != Status::Ok) {
				return status;
			}

			// DVP Line timing: HTS = 1456 (0x05B0)
			status = this->WriteRegister(Register::HTS_H, 0x02);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::HTS_L, 0xD8);
			if(status != Status::Ok) {
				return status;
			}

			// Base VTS = 910 (0x038E)
			status = this->WriteRegister(Register::VTS_H, 0x03);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::VTS_L, 0x8E);
			if(status != Status::Ok) {
				return status;
			}

			// Crop offsets (8 px margins: 1296-16=1280, 736-16=720)
			status = this->WriteRegister(Register::X_WIN_H, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::X_WIN_L, 0x08);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_WIN_H, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_WIN_L, 0x08);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::X_INC, 0x11);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_INC, 0x11);
			if(status != Status::Ok) {
				return status;
			}

			// Timing format but retain mirror/flip bits [2:1]
			status = this->ModifyRegister(Register::TIMING_FORMAT1, 0xF9, 0x40);
			if(status != Status::Ok) {
				return status;
			}
			status = this->ModifyRegister(Register::TIMING_FORMAT2, 0xFB, 0x00);
			if(status != Status::Ok) {
				return status;
			}

			// Full-resolution BLC windows
			status = this->WriteRegister(static_cast<Register>(0x4003), 0x40);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4008), 0x04);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4009), 0x0B);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x400C), 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x400D), 0x07);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4507), 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4509), 0x00);
			if(status != Status::Ok) {
				return status;
			}

			this->sensorInfo.width = 1280;
			this->sensorInfo.height = 720;
			break;

		case InternalResolution::VGA:
			// 640x400 is a 2x horizontal / 2x vertical subsampled mode.
			status = this->ApplyWindow(0, 0, 1296, 816);
			if(status != Status::Ok) {
				return status;
			}

			// Output dimensions: 640 x 400
			status = this->WriteRegister(Register::X_OUTPUT_H, 0x02);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::X_OUTPUT_L, 0x80);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_OUTPUT_H, 0x01);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_OUTPUT_L, 0x90);
			if(status != Status::Ok) {
				return status;
			}

			// Pixel binning enabled
			status = this->WriteRegister(static_cast<Register>(0x3778), 0x10);
			if(status != Status::Ok) {
				return status;
			}

			// DVP Line timing: HTS = 728 (0x02D8)
			status = this->WriteRegister(Register::HTS_H, 0x02);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::HTS_L, 0xD8);
			if(status != Status::Ok) {
				return status;
			}

			// Base VTS = 520 (0x0208)
			status = this->WriteRegister(Register::VTS_H, 0x02);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::VTS_L, 0x08);
			if(status != Status::Ok) {
				return status;
			}

			// Crop offsets
			status = this->WriteRegister(Register::X_WIN_H, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::X_WIN_L, 0x04);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_WIN_H, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_WIN_L, 0x04);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::X_INC, 0x31);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_INC, 0x22);
			if(status != Status::Ok) {
				return status;
			}

			// Vertical bining
			status = this->ModifyRegister(Register::TIMING_FORMAT1, 0xF9, 0x60);
			if(status != Status::Ok) {
				return status;
			}
			// Horizontal bining
			status = this->ModifyRegister(Register::TIMING_FORMAT2, 0xFB, 0x01);
			if(status != Status::Ok) {
				return status;
			}

			// Scaled BLC windows for binned output
			status = this->WriteRegister(static_cast<Register>(0x4008), 0x02);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4009), 0x05);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x400C), 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x400D), 0x03);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4507), 0x03);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4509), 0x80);
			if(status != Status::Ok) {
				return status;
			}

			this->sensorInfo.width = 640;
			this->sensorInfo.height = 400;
			break;

		case InternalResolution::P256:
			// 256x256 is a 2x horizontal / 2x vertical subsampled mode and windowing
			status = this->ApplyWindow(0, 0, 1296, 816);
			if(status != Status::Ok) {
				return status;
			}

			// Output dimensions: 256 x 256
			status = this->WriteRegister(Register::X_OUTPUT_H, 0x01);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::X_OUTPUT_L, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_OUTPUT_H, 0x01);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_OUTPUT_L, 0x00);
			if(status != Status::Ok) {
				return status;
			}

			// Pixel binning enabled
			status = this->WriteRegister(static_cast<Register>(0x3778), 0x10);
			if(status != Status::Ok) {
				return status;
			}

			// DVP Line timing: HTS = 728 (0x02D8)
			status = this->WriteRegister(Register::HTS_H, 0x02);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::HTS_L, 0xD8);
			if(status != Status::Ok) {
				return status;
			}

			// Base VTS = 520 (0x0208)
			status = this->WriteRegister(Register::VTS_H, 0x02);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::VTS_L, 0x08);
			if(status != Status::Ok) {
				return status;
			}

			// Crop offsets: Crop center from 648x408 pinned stream to 256x256
			// X_WIN = (648 - 256) / 2 = 196 (0x00C4)
			// Y_WIN = (408 - 256) / 2 = 76 (0x004C)
			status = this->WriteRegister(Register::X_WIN_H, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::X_WIN_L, 0xC4);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_WIN_H, 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_WIN_L, 0x4C);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::X_INC, 0x31);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(Register::Y_INC, 0x22);
			if(status != Status::Ok) {
				return status;
			}

			// Vertical bining
			status = this->ModifyRegister(Register::TIMING_FORMAT1, 0xF9, 0x60);
			if(status != Status::Ok) {
				return status;
			}
			// Horizontal bining
			status = this->ModifyRegister(Register::TIMING_FORMAT2, 0xFB, 0x01);
			if(status != Status::Ok) {
				return status;
			}

			// Scaled BLC windows for binned output
			status = this->WriteRegister(static_cast<Register>(0x4008), 0x02);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4009), 0x05);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x400C), 0x00);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x400D), 0x03);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4507), 0x03);
			if(status != Status::Ok) {
				return status;
			}
			status = this->WriteRegister(static_cast<Register>(0x4509), 0x80);
			if(status != Status::Ok) {
				return status;
			}

			this->sensorInfo.width = 256;
			this->sensorInfo.height = 256;
			break;

		default:
			return Status::Error;
	}

	return this->SetExposure(128);
}

Status OV9281::ApplyWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
	if(w == 0 || h == 0) {
		return Status::Error;
	}

	uint32_t xEnd = static_cast<uint32_t>(x) + w - 1;
	uint32_t yEnd = static_cast<uint32_t>(y) + h - 1;
	// Active array size: 1296 x 816
	if(xEnd >= 1296 || yEnd >= 816) {
		return Status::Error;
	}

	Status status = this->WriteRegister(Register::X_START_H, static_cast<uint8_t>((x >> 8) & 0xFF));
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(Register::X_START_L, static_cast<uint8_t>(x & 0xFF));
	if(status != Status::Ok) {
		return status;
	}

	status = this->WriteRegister(Register::Y_START_H, static_cast<uint8_t>((y >> 8) & 0xFF));
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(Register::Y_START_L, static_cast<uint8_t>(y & 0xFF));
	if(status != Status::Ok) {
		return status;
	}

	status = this->WriteRegister(Register::X_END_H, static_cast<uint8_t>((xEnd >> 8) & 0xFF));
	if(status != Status::Ok) {
		return status;
	}

	status = this->WriteRegister(Register::X_END_L, static_cast<uint8_t>(xEnd & 0xFF));
	if(status != Status::Ok) {
		return status;
	}

	status = this->WriteRegister(Register::Y_END_H, static_cast<uint8_t>((yEnd >> 8) & 0xFF));
	if(status != Status::Ok) {
		return status;
	}

	status = this->WriteRegister(Register::Y_END_L,	static_cast<uint8_t>(yEnd & 0xFF));
	if(status != Status::Ok) {
		return status;
	}

	return Status::Ok;
}

Status OV9281::WriteRegisterArray(const RegisterValuePair* array, uint16_t size) {
	if(array == nullptr || size == 0) {
		return Status::Error;
	}

	for(uint16_t i = 0; i < size; i++) {
		Status status = this->WriteRegister(array[i].reg, array[i].value);
		if(status != Status::Ok) {
			return status; 
		}
	}
	return Status::Ok;
}

Status OV9281::WriteRegister(Register reg, uint8_t value) {
	const uint8_t maxRetries = 3;
	Status status = Status::Error;

	// Do a few retires, due to using SCCB and not I2C there are issues with the ACK (Do-Not-Care in SCCB)
	for(uint8_t attempt = 0; attempt < maxRetries; attempt++) {
		uint16_t regAddr = static_cast<uint16_t>(reg);
		this->buffer[0] = static_cast<uint8_t>(regAddr >> 8);
		this->buffer[1] = static_cast<uint8_t>(regAddr & 0xFF);
		this->buffer[2] = value;

		status = this->bus.TransferAsync(this->addr, I3C::TargetType::I2C, this->buffer, 3, nullptr, 0);
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

Status OV9281::ReadRegister(Register reg, uint8_t& value) {
	const uint8_t maxRetries = 3;
	Status status = Status::Error;

	// Do a few retires, due to using SCCB and not I2C there are issues with the ACK (Do-Not-Care in SCCB)
	for(uint8_t attempt = 0; attempt < maxRetries; attempt++) {
		uint16_t regAddr = static_cast<uint16_t>(reg);
		this->buffer[0] = static_cast<uint8_t>(regAddr >> 8);
		this->buffer[1] = static_cast<uint8_t>(regAddr & 0xFF);
		
		status = this->bus.TransferAsync(this->addr, I3C::TargetType::I2C, this->buffer, 2, nullptr, 0);
		if(status == Status::Ok) {
			status = this->bus.TransferWait(1000);
		}

		if(status != Status::Ok) {
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

Status OV9281::ModifyRegister(Register reg, uint8_t mask, uint8_t value) {
	uint8_t regVal;
	Status status = this->ReadRegister(reg, regVal);
	if(status != Status::Ok) {
		return status;
	}

	regVal &= ~mask;
	regVal |= (value & mask);

	return this->WriteRegister(reg, regVal);
}