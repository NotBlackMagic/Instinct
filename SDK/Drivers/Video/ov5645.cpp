/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Video/ov5645.cpp
 */

#include "ov5645.hpp"

// OV5645 Magic Initialization Blobs
// From Linux driver: https://github.com/torvalds/linux/blob/master/drivers/media/i2c/ov5645.c
static constexpr OV5645::RegisterValuePair ov5645GlobalInit[] = {
	{ static_cast<OV5645::Register>(0x3103), 0x11 },
	{ static_cast<OV5645::Register>(0x3008), 0x42 },
	{ static_cast<OV5645::Register>(0x3103), 0x03 },
	{ static_cast<OV5645::Register>(0x3503), 0x07 },
	{ static_cast<OV5645::Register>(0x3002), 0x1c },
	{ static_cast<OV5645::Register>(0x3006), 0xc3 },
	{ static_cast<OV5645::Register>(0x3017), 0x00 },
	{ static_cast<OV5645::Register>(0x3018), 0x00 },
	{ static_cast<OV5645::Register>(0x302e), 0x0b },
	{ static_cast<OV5645::Register>(0x3037), 0x13 },	// PLL root divider: DIV2, PLL pre-divider: 3
	{ static_cast<OV5645::Register>(0x3108), 0x01 },
	{ static_cast<OV5645::Register>(0x3611), 0x06 },
	{ static_cast<OV5645::Register>(0x3500), 0x00 },
	{ static_cast<OV5645::Register>(0x3501), 0x01 },
	{ static_cast<OV5645::Register>(0x3502), 0x00 },
	{ static_cast<OV5645::Register>(0x350a), 0x00 },
	{ static_cast<OV5645::Register>(0x350b), 0x3f },
	{ static_cast<OV5645::Register>(0x3620), 0x33 },
	{ static_cast<OV5645::Register>(0x3621), 0xe0 },
	{ static_cast<OV5645::Register>(0x3622), 0x01 },
	{ static_cast<OV5645::Register>(0x3630), 0x2e },
	{ static_cast<OV5645::Register>(0x3631), 0x00 },
	{ static_cast<OV5645::Register>(0x3632), 0x32 },
	{ static_cast<OV5645::Register>(0x3633), 0x52 },
	{ static_cast<OV5645::Register>(0x3634), 0x70 },
	{ static_cast<OV5645::Register>(0x3635), 0x13 },
	{ static_cast<OV5645::Register>(0x3636), 0x03 },
	{ static_cast<OV5645::Register>(0x3703), 0x5a },
	{ static_cast<OV5645::Register>(0x3704), 0xa0 },
	{ static_cast<OV5645::Register>(0x3705), 0x1a },
	{ static_cast<OV5645::Register>(0x3709), 0x12 },
	{ static_cast<OV5645::Register>(0x370b), 0x61 },
	{ static_cast<OV5645::Register>(0x370f), 0x10 },
	{ static_cast<OV5645::Register>(0x3715), 0x78 },
	{ static_cast<OV5645::Register>(0x3717), 0x01 },
	{ static_cast<OV5645::Register>(0x371b), 0x20 },
	{ static_cast<OV5645::Register>(0x3731), 0x12 },
	{ static_cast<OV5645::Register>(0x3901), 0x0a },
	{ static_cast<OV5645::Register>(0x3905), 0x02 },
	{ static_cast<OV5645::Register>(0x3906), 0x10 },
	{ static_cast<OV5645::Register>(0x3719), 0x86 },
	{ static_cast<OV5645::Register>(0x3810), 0x00 },
	{ static_cast<OV5645::Register>(0x3811), 0x10 },
	{ static_cast<OV5645::Register>(0x3812), 0x00 },
	{ static_cast<OV5645::Register>(0x3821), 0x01 },
	{ static_cast<OV5645::Register>(0x3824), 0x01 },
	{ static_cast<OV5645::Register>(0x3826), 0x03 },
	{ static_cast<OV5645::Register>(0x3828), 0x08 },
	{ static_cast<OV5645::Register>(0x3a19), 0xf8 },
	{ static_cast<OV5645::Register>(0x3c01), 0x34 },
	{ static_cast<OV5645::Register>(0x3c04), 0x28 },
	{ static_cast<OV5645::Register>(0x3c05), 0x98 },
	{ static_cast<OV5645::Register>(0x3c07), 0x07 },
	{ static_cast<OV5645::Register>(0x3c09), 0xc2 },
	{ static_cast<OV5645::Register>(0x3c0a), 0x9c },
	{ static_cast<OV5645::Register>(0x3c0b), 0x40 },
	{ static_cast<OV5645::Register>(0x3c01), 0x34 },
	{ static_cast<OV5645::Register>(0x4001), 0x02 },
	{ static_cast<OV5645::Register>(0x4514), 0x00 },
	{ static_cast<OV5645::Register>(0x4520), 0xb0 },
	{ static_cast<OV5645::Register>(0x460b), 0x37 },
	{ static_cast<OV5645::Register>(0x460c), 0x20 },
	{ static_cast<OV5645::Register>(0x4818), 0x01 },
	{ static_cast<OV5645::Register>(0x481d), 0xf0 },
	{ static_cast<OV5645::Register>(0x481f), 0x50 },
	{ static_cast<OV5645::Register>(0x4823), 0x70 },
	{ static_cast<OV5645::Register>(0x4831), 0x14 },
	{ static_cast<OV5645::Register>(0x5000), 0xa7 },
	{ static_cast<OV5645::Register>(0x5001), 0x83 },
	{ static_cast<OV5645::Register>(0x501d), 0x00 },
	{ static_cast<OV5645::Register>(0x501f), 0x00 },
	{ static_cast<OV5645::Register>(0x503d), 0x00 },
	{ static_cast<OV5645::Register>(0x505c), 0x30 },
	{ static_cast<OV5645::Register>(0x5181), 0x59 },
	{ static_cast<OV5645::Register>(0x5183), 0x00 },
	{ static_cast<OV5645::Register>(0x5191), 0xf0 },
	{ static_cast<OV5645::Register>(0x5192), 0x03 },
	{ static_cast<OV5645::Register>(0x5684), 0x10 },
	{ static_cast<OV5645::Register>(0x5685), 0xa0 },
	{ static_cast<OV5645::Register>(0x5686), 0x0c },
	{ static_cast<OV5645::Register>(0x5687), 0x78 },
	{ static_cast<OV5645::Register>(0x5a00), 0x08 },
	{ static_cast<OV5645::Register>(0x5a21), 0x00 },
	{ static_cast<OV5645::Register>(0x5a24), 0x00 },
	{ static_cast<OV5645::Register>(0x3008), 0x02 },
	{ static_cast<OV5645::Register>(0x3503), 0x00 },
	{ static_cast<OV5645::Register>(0x5180), 0xff },
	{ static_cast<OV5645::Register>(0x5181), 0xf2 },
	{ static_cast<OV5645::Register>(0x5182), 0x00 },
	{ static_cast<OV5645::Register>(0x5183), 0x14 },
	{ static_cast<OV5645::Register>(0x5184), 0x25 },
	{ static_cast<OV5645::Register>(0x5185), 0x24 },
	{ static_cast<OV5645::Register>(0x5186), 0x09 },
	{ static_cast<OV5645::Register>(0x5187), 0x09 },
	{ static_cast<OV5645::Register>(0x5188), 0x0a },
	{ static_cast<OV5645::Register>(0x5189), 0x75 },
	{ static_cast<OV5645::Register>(0x518a), 0x52 },
	{ static_cast<OV5645::Register>(0x518b), 0xea },
	{ static_cast<OV5645::Register>(0x518c), 0xa8 },
	{ static_cast<OV5645::Register>(0x518d), 0x42 },
	{ static_cast<OV5645::Register>(0x518e), 0x38 },
	{ static_cast<OV5645::Register>(0x518f), 0x56 },
	{ static_cast<OV5645::Register>(0x5190), 0x42 },
	{ static_cast<OV5645::Register>(0x5191), 0xf8 },
	{ static_cast<OV5645::Register>(0x5192), 0x04 },
	{ static_cast<OV5645::Register>(0x5193), 0x70 },
	{ static_cast<OV5645::Register>(0x5194), 0xf0 },
	{ static_cast<OV5645::Register>(0x5195), 0xf0 },
	{ static_cast<OV5645::Register>(0x5196), 0x03 },
	{ static_cast<OV5645::Register>(0x5197), 0x01 },
	{ static_cast<OV5645::Register>(0x5198), 0x04 },
	{ static_cast<OV5645::Register>(0x5199), 0x12 },
	{ static_cast<OV5645::Register>(0x519a), 0x04 },
	{ static_cast<OV5645::Register>(0x519b), 0x00 },
	{ static_cast<OV5645::Register>(0x519c), 0x06 },
	{ static_cast<OV5645::Register>(0x519d), 0x82 },
	{ static_cast<OV5645::Register>(0x519e), 0x38 },
	{ static_cast<OV5645::Register>(0x5381), 0x1e },
	{ static_cast<OV5645::Register>(0x5382), 0x5b },
	{ static_cast<OV5645::Register>(0x5383), 0x08 },
	{ static_cast<OV5645::Register>(0x5384), 0x0a },
	{ static_cast<OV5645::Register>(0x5385), 0x7e },
	{ static_cast<OV5645::Register>(0x5386), 0x88 },
	{ static_cast<OV5645::Register>(0x5387), 0x7c },
	{ static_cast<OV5645::Register>(0x5388), 0x6c },
	{ static_cast<OV5645::Register>(0x5389), 0x10 },
	{ static_cast<OV5645::Register>(0x538a), 0x01 },
	{ static_cast<OV5645::Register>(0x538b), 0x98 },
	{ static_cast<OV5645::Register>(0x5300), 0x08 },
	{ static_cast<OV5645::Register>(0x5301), 0x30 },
	{ static_cast<OV5645::Register>(0x5302), 0x10 },
	{ static_cast<OV5645::Register>(0x5303), 0x00 },
	{ static_cast<OV5645::Register>(0x5304), 0x08 },
	{ static_cast<OV5645::Register>(0x5305), 0x30 },
	{ static_cast<OV5645::Register>(0x5306), 0x08 },
	{ static_cast<OV5645::Register>(0x5307), 0x16 },
	{ static_cast<OV5645::Register>(0x5309), 0x08 },
	{ static_cast<OV5645::Register>(0x530a), 0x30 },
	{ static_cast<OV5645::Register>(0x530b), 0x04 },
	{ static_cast<OV5645::Register>(0x530c), 0x06 },
	{ static_cast<OV5645::Register>(0x5480), 0x01 },
	{ static_cast<OV5645::Register>(0x5481), 0x08 },
	{ static_cast<OV5645::Register>(0x5482), 0x14 },
	{ static_cast<OV5645::Register>(0x5483), 0x28 },
	{ static_cast<OV5645::Register>(0x5484), 0x51 },
	{ static_cast<OV5645::Register>(0x5485), 0x65 },
	{ static_cast<OV5645::Register>(0x5486), 0x71 },
	{ static_cast<OV5645::Register>(0x5487), 0x7d },
	{ static_cast<OV5645::Register>(0x5488), 0x87 },
	{ static_cast<OV5645::Register>(0x5489), 0x91 },
	{ static_cast<OV5645::Register>(0x548a), 0x9a },
	{ static_cast<OV5645::Register>(0x548b), 0xaa },
	{ static_cast<OV5645::Register>(0x548c), 0xb8 },
	{ static_cast<OV5645::Register>(0x548d), 0xcd },
	{ static_cast<OV5645::Register>(0x548e), 0xdd },
	{ static_cast<OV5645::Register>(0x548f), 0xea },
	{ static_cast<OV5645::Register>(0x5490), 0x1d },
	{ static_cast<OV5645::Register>(0x5580), 0x02 },
	{ static_cast<OV5645::Register>(0x5583), 0x40 },
	{ static_cast<OV5645::Register>(0x5584), 0x10 },
	{ static_cast<OV5645::Register>(0x5589), 0x10 },
	{ static_cast<OV5645::Register>(0x558a), 0x00 },
	{ static_cast<OV5645::Register>(0x558b), 0xf8 },
	{ static_cast<OV5645::Register>(0x5800), 0x3f },
	{ static_cast<OV5645::Register>(0x5801), 0x16 },
	{ static_cast<OV5645::Register>(0x5802), 0x0e },
	{ static_cast<OV5645::Register>(0x5803), 0x0d },
	{ static_cast<OV5645::Register>(0x5804), 0x17 },
	{ static_cast<OV5645::Register>(0x5805), 0x3f },
	{ static_cast<OV5645::Register>(0x5806), 0x0b },
	{ static_cast<OV5645::Register>(0x5807), 0x06 },
	{ static_cast<OV5645::Register>(0x5808), 0x04 },
	{ static_cast<OV5645::Register>(0x5809), 0x04 },
	{ static_cast<OV5645::Register>(0x580a), 0x06 },
	{ static_cast<OV5645::Register>(0x580b), 0x0b },
	{ static_cast<OV5645::Register>(0x580c), 0x09 },
	{ static_cast<OV5645::Register>(0x580d), 0x03 },
	{ static_cast<OV5645::Register>(0x580e), 0x00 },
	{ static_cast<OV5645::Register>(0x580f), 0x00 },
	{ static_cast<OV5645::Register>(0x5810), 0x03 },
	{ static_cast<OV5645::Register>(0x5811), 0x08 },
	{ static_cast<OV5645::Register>(0x5812), 0x0a },
	{ static_cast<OV5645::Register>(0x5813), 0x03 },
	{ static_cast<OV5645::Register>(0x5814), 0x00 },
	{ static_cast<OV5645::Register>(0x5815), 0x00 },
	{ static_cast<OV5645::Register>(0x5816), 0x04 },
	{ static_cast<OV5645::Register>(0x5817), 0x09 },
	{ static_cast<OV5645::Register>(0x5818), 0x0f },
	{ static_cast<OV5645::Register>(0x5819), 0x08 },
	{ static_cast<OV5645::Register>(0x581a), 0x06 },
	{ static_cast<OV5645::Register>(0x581b), 0x06 },
	{ static_cast<OV5645::Register>(0x581c), 0x08 },
	{ static_cast<OV5645::Register>(0x581d), 0x0c },
	{ static_cast<OV5645::Register>(0x581e), 0x3f },
	{ static_cast<OV5645::Register>(0x581f), 0x1e },
	{ static_cast<OV5645::Register>(0x5820), 0x12 },
	{ static_cast<OV5645::Register>(0x5821), 0x13 },
	{ static_cast<OV5645::Register>(0x5822), 0x21 },
	{ static_cast<OV5645::Register>(0x5823), 0x3f },
	{ static_cast<OV5645::Register>(0x5824), 0x68 },
	{ static_cast<OV5645::Register>(0x5825), 0x28 },
	{ static_cast<OV5645::Register>(0x5826), 0x2c },
	{ static_cast<OV5645::Register>(0x5827), 0x28 },
	{ static_cast<OV5645::Register>(0x5828), 0x08 },
	{ static_cast<OV5645::Register>(0x5829), 0x48 },
	{ static_cast<OV5645::Register>(0x582a), 0x64 },
	{ static_cast<OV5645::Register>(0x582b), 0x62 },
	{ static_cast<OV5645::Register>(0x582c), 0x64 },
	{ static_cast<OV5645::Register>(0x582d), 0x28 },
	{ static_cast<OV5645::Register>(0x582e), 0x46 },
	{ static_cast<OV5645::Register>(0x582f), 0x62 },
	{ static_cast<OV5645::Register>(0x5830), 0x60 },
	{ static_cast<OV5645::Register>(0x5831), 0x62 },
	{ static_cast<OV5645::Register>(0x5832), 0x26 },
	{ static_cast<OV5645::Register>(0x5833), 0x48 },
	{ static_cast<OV5645::Register>(0x5834), 0x66 },
	{ static_cast<OV5645::Register>(0x5835), 0x44 },
	{ static_cast<OV5645::Register>(0x5836), 0x64 },
	{ static_cast<OV5645::Register>(0x5837), 0x28 },
	{ static_cast<OV5645::Register>(0x5838), 0x66 },
	{ static_cast<OV5645::Register>(0x5839), 0x48 },
	{ static_cast<OV5645::Register>(0x583a), 0x2c },
	{ static_cast<OV5645::Register>(0x583b), 0x28 },
	{ static_cast<OV5645::Register>(0x583c), 0x26 },
	{ static_cast<OV5645::Register>(0x583d), 0xae },
	{ static_cast<OV5645::Register>(0x5025), 0x00 },
	{ static_cast<OV5645::Register>(0x3a0f), 0x30 },
	{ static_cast<OV5645::Register>(0x3a10), 0x28 },
	{ static_cast<OV5645::Register>(0x3a1b), 0x30 },
	{ static_cast<OV5645::Register>(0x3a1e), 0x26 },
	{ static_cast<OV5645::Register>(0x3a11), 0x60 },
	{ static_cast<OV5645::Register>(0x3a1f), 0x14 },
	{ static_cast<OV5645::Register>(0x0601), 0x02 },
	{ static_cast<OV5645::Register>(0x3008), 0x42 },
	{ static_cast<OV5645::Register>(0x3008), 0x02 },
	{ OV5645::Register::IO_MIPI_CTRL00, 0x40 },	// Set to two lane mode, MIPI disabled
	{ OV5645::Register::MIPI_CTRL00, 0x24 },		// Set gated clock lane, LP11 as no packet
	{ OV5645::Register::PAD_OUT_VAL00, 0x70 }
};

static constexpr OV5645::RegisterValuePair ov5645VGA[] = {
	{ static_cast<OV5645::Register>(0x3612), 0xab },
	{ static_cast<OV5645::Register>(0x3614), 0x50 },
	{ static_cast<OV5645::Register>(0x3618), 0x00 },
	{ static_cast<OV5645::Register>(0x3034), 0x18 },	// PLL charge pump control, MIPI bit more: 8-bit mode
	{ static_cast<OV5645::Register>(0x3035), 0x21 },	// ?? (ESP-32: 0x12) System clock divider: 2, Scale divider for MIPI: 1
	{ static_cast<OV5645::Register>(0x3036), 0x70 },	// ?? PLL multiplier: x112
	{ static_cast<OV5645::Register>(0x3600), 0x09 },
	{ static_cast<OV5645::Register>(0x3601), 0x43 },
	{ static_cast<OV5645::Register>(0x3708), 0x66 },
	{ static_cast<OV5645::Register>(0x370c), 0xc3 },
	{ static_cast<OV5645::Register>(0x3800), 0x00 },	// X address start
	{ static_cast<OV5645::Register>(0x3801), 0x00 },
	{ static_cast<OV5645::Register>(0x3802), 0x00 },	// Y address start
	{ static_cast<OV5645::Register>(0x3803), 0x06 },
	{ static_cast<OV5645::Register>(0x3804), 0x0a },	// X address end
	{ static_cast<OV5645::Register>(0x3805), 0x3f },
	{ static_cast<OV5645::Register>(0x3806), 0x07 },	// Y address end
	{ static_cast<OV5645::Register>(0x3807), 0x9d },
	{ static_cast<OV5645::Register>(0x3808), 0x02 },	// Horizontal width, DVPHO = 640
	{ static_cast<OV5645::Register>(0x3809), 0x80 },
	{ static_cast<OV5645::Register>(0x380a), 0x01 },	// Vertical height, DVPVO = 480
	{ static_cast<OV5645::Register>(0x380b), 0xe0 },
	{ static_cast<OV5645::Register>(0x380c), 0x07 },	// Total horizontal size
	{ static_cast<OV5645::Register>(0x380d), 0x68 },
	{ static_cast<OV5645::Register>(0x380e), 0x03 },	// Total vertical size
	{ static_cast<OV5645::Register>(0x380f), 0xd8 },
	{ static_cast<OV5645::Register>(0x3813), 0x06 },
	{ static_cast<OV5645::Register>(0x3814), 0x31 },
	{ static_cast<OV5645::Register>(0x3815), 0x31 },
	{ static_cast<OV5645::Register>(0x3820), 0x47 },
	{ static_cast<OV5645::Register>(0x3a02), 0x03 },
	{ static_cast<OV5645::Register>(0x3a03), 0xd8 },
	{ static_cast<OV5645::Register>(0x3a08), 0x01 },
	{ static_cast<OV5645::Register>(0x3a09), 0xf8 },
	{ static_cast<OV5645::Register>(0x3a0a), 0x01 },
	{ static_cast<OV5645::Register>(0x3a0b), 0xa4 },
	{ static_cast<OV5645::Register>(0x3a0e), 0x02 },
	{ static_cast<OV5645::Register>(0x3a0d), 0x02 },
	{ static_cast<OV5645::Register>(0x3a14), 0x03 },
	{ static_cast<OV5645::Register>(0x3a15), 0xd8 },
	{ static_cast<OV5645::Register>(0x3a18), 0x00 },
	{ static_cast<OV5645::Register>(0x4004), 0x02 },
	{ static_cast<OV5645::Register>(0x4005), 0x18 },
	{ static_cast<OV5645::Register>(0x4300), 0x32 },
	{ static_cast<OV5645::Register>(0x4202), 0x00 },
	{ static_cast<OV5645::Register>(0x5001), 0xA3 },	// Enable scaling
	{ static_cast<OV5645::Register>(0x5601), 0x22 },	// Bit[6:4] HDIV, BIT[2:0] VDIV (Both x4 times)
};

// From Linux driver: https://github.com/torvalds/linux/blob/master/drivers/media/i2c/ov5645.c
static constexpr OV5645::RegisterValuePair ov5645SXGA[] = {
	{ static_cast<OV5645::Register>(0x3612), 0xa9 },
	{ static_cast<OV5645::Register>(0x3614), 0x50 },
	{ static_cast<OV5645::Register>(0x3618), 0x00 },
	{ static_cast<OV5645::Register>(0x3034), 0x18 },	// PLL charge pump control, MIPI bit more: 8-bit mode
	{ static_cast<OV5645::Register>(0x3035), 0x21 },	// System clock divider: XX, Scale divider for MIPI: YY
	{ static_cast<OV5645::Register>(0x3036), 0x70 },	// PLL multiplier: x112
	{ static_cast<OV5645::Register>(0x3600), 0x09 },
	{ static_cast<OV5645::Register>(0x3601), 0x43 },
	{ static_cast<OV5645::Register>(0x3708), 0x66 },
	{ static_cast<OV5645::Register>(0x370c), 0xc3 },
	{ static_cast<OV5645::Register>(0x3800), 0x00 },
	{ static_cast<OV5645::Register>(0x3801), 0x00 },
	{ static_cast<OV5645::Register>(0x3802), 0x00 },
	{ static_cast<OV5645::Register>(0x3803), 0x04 },	// ESP		TIMING VS: Y address start[7:0]
	{ static_cast<OV5645::Register>(0x3804), 0x0a },
	{ static_cast<OV5645::Register>(0x3805), 0x3f },
	{ static_cast<OV5645::Register>(0x3806), 0x07 },
	{ static_cast<OV5645::Register>(0x3807), 0x9b },	// ESP		TIMING VH: Y address end[7:0]
	{ static_cast<OV5645::Register>(0x3808), 0x02 },	// ESP		TIMING DVPHO: DVP output horizontal width[11:8]
	{ static_cast<OV5645::Register>(0x3809), 0x80 },	// ESP		TIMING DVPHO: DVP output horizontal width[7:0]
	{ static_cast<OV5645::Register>(0x380a), 0x01 },	// ESP		TIMING DVPVO: DVP output vertical height[10:8]
	{ static_cast<OV5645::Register>(0x380b), 0xe0 },	// ESP		TIMING DVPVO: DVP output vertical height[7:0]
	{ static_cast<OV5645::Register>(0x380c), 0x09 },	// ESP		TIMING HTS: Total horizontal size[11:8]
	{ static_cast<OV5645::Register>(0x380d), 0x00 },	// ESP		TIMING HTS: Total horizontal size[7:0]
	{ static_cast<OV5645::Register>(0x380e), 0x05 },	// ESP		TIMING VTS: Total vertical size[15:8]
	{ static_cast<OV5645::Register>(0x380f), 0x10 },	// ESP		TIMING VTS: Total vertical size[7:0]
	{ static_cast<OV5645::Register>(0x3813), 0x06 },
	{ static_cast<OV5645::Register>(0x3814), 0x31 },
	{ static_cast<OV5645::Register>(0x3815), 0x31 },
	{ static_cast<OV5645::Register>(0x3820), 0x47 },	// TIMING TC REG20: Timing Control: Blackline vflip, ISP vflip, Sensor vflip, Vertical binning enable
	{ static_cast<OV5645::Register>(0x3a02), 0x03 },
	{ static_cast<OV5645::Register>(0x3a03), 0xb0 },	// AEC MAX EXPO (60HZ): 60Hz Maximum Exposure Output Limit
	{ static_cast<OV5645::Register>(0x3a08), 0x01 },
	{ static_cast<OV5645::Register>(0x3a09), 0x50 },	// AEC B50 STEP: 50Hz Band Width
	{ static_cast<OV5645::Register>(0x3a0a), 0x00 },	// AEC B60 STEP: 60Hz Band Width
	{ static_cast<OV5645::Register>(0x3a0b), 0xf6 },	// AEC B60 STEP: 60Hz Band Width
	{ static_cast<OV5645::Register>(0x3a0e), 0x02 },	// AEC CTRL0E: 50Hz Max Bands in One Frame
	{ static_cast<OV5645::Register>(0x3a0d), 0x02 },	// AEC CTRL0D: 60Hz Max Bands in One Frame
	{ static_cast<OV5645::Register>(0x3a14), 0x03 },	// AEC MAX EXPO (50Hz): 50Hz Maximum Exposure Output Limit
	{ static_cast<OV5645::Register>(0x3a15), 0xd8 },	// AEC MAX EXPO (50Hz): 50Hz Maximum Exposure Output Limit
	{ static_cast<OV5645::Register>(0x3a18), 0x00 },
	{ static_cast<OV5645::Register>(0x4004), 0x02 },
	{ static_cast<OV5645::Register>(0x4005), 0x18 },
	{ static_cast<OV5645::Register>(0x4300), 0x32 },
	{ static_cast<OV5645::Register>(0x4202), 0x00 }
};

// From Linux driver: https://github.com/torvalds/linux/blob/master/drivers/media/i2c/ov5645.c
static constexpr OV5645::RegisterValuePair ov5645FHD[] = {
	{ static_cast<OV5645::Register>(0x3612), 0xab },
	{ static_cast<OV5645::Register>(0x3614), 0x50 },
	{ static_cast<OV5645::Register>(0x3618), 0x04 },
	{ static_cast<OV5645::Register>(0x3034), 0x18 },
	{ static_cast<OV5645::Register>(0x3035), 0x11 },
	{ static_cast<OV5645::Register>(0x3036), 0x54 },
	{ static_cast<OV5645::Register>(0x3600), 0x08 },
	{ static_cast<OV5645::Register>(0x3601), 0x33 },
	{ static_cast<OV5645::Register>(0x3708), 0x63 },
	{ static_cast<OV5645::Register>(0x370c), 0xc0 },
	{ static_cast<OV5645::Register>(0x3800), 0x01 },
	{ static_cast<OV5645::Register>(0x3801), 0x50 },
	{ static_cast<OV5645::Register>(0x3802), 0x01 },
	{ static_cast<OV5645::Register>(0x3803), 0xb2 },
	{ static_cast<OV5645::Register>(0x3804), 0x08 },
	{ static_cast<OV5645::Register>(0x3805), 0xef },
	{ static_cast<OV5645::Register>(0x3806), 0x05 },
	{ static_cast<OV5645::Register>(0x3807), 0xf1 },
	{ static_cast<OV5645::Register>(0x3808), 0x07 },
	{ static_cast<OV5645::Register>(0x3809), 0x80 },
	{ static_cast<OV5645::Register>(0x380a), 0x04 },
	{ static_cast<OV5645::Register>(0x380b), 0x38 },
	{ static_cast<OV5645::Register>(0x380c), 0x09 },
	{ static_cast<OV5645::Register>(0x380d), 0xc4 },
	{ static_cast<OV5645::Register>(0x380e), 0x04 },
	{ static_cast<OV5645::Register>(0x380f), 0x60 },
	{ static_cast<OV5645::Register>(0x3813), 0x04 },
	{ static_cast<OV5645::Register>(0x3814), 0x11 },
	{ static_cast<OV5645::Register>(0x3815), 0x11 },
	{ static_cast<OV5645::Register>(0x3820), 0x47 },
	{ static_cast<OV5645::Register>(0x4514), 0x88 },
	{ static_cast<OV5645::Register>(0x3a02), 0x04 },
	{ static_cast<OV5645::Register>(0x3a03), 0x60 },
	{ static_cast<OV5645::Register>(0x3a08), 0x01 },
	{ static_cast<OV5645::Register>(0x3a09), 0x50 },
	{ static_cast<OV5645::Register>(0x3a0a), 0x01 },
	{ static_cast<OV5645::Register>(0x3a0b), 0x18 },
	{ static_cast<OV5645::Register>(0x3a0e), 0x03 },
	{ static_cast<OV5645::Register>(0x3a0d), 0x04 },
	{ static_cast<OV5645::Register>(0x3a14), 0x04 },
	{ static_cast<OV5645::Register>(0x3a15), 0x60 },
	{ static_cast<OV5645::Register>(0x3a18), 0x00 },
	{ static_cast<OV5645::Register>(0x4004), 0x06 },
	{ static_cast<OV5645::Register>(0x4005), 0x18 },
	{ static_cast<OV5645::Register>(0x4300), 0x32 },
	{ static_cast<OV5645::Register>(0x4202), 0x00 },
	{ static_cast<OV5645::Register>(0x4837), 0x0b }
};

// From Linux driver: https://github.com/torvalds/linux/blob/master/drivers/media/i2c/ov5645.c
static constexpr OV5645::RegisterValuePair ov5645QSXGA[] = {
	{ static_cast<OV5645::Register>(0x3612), 0xab },
	{ static_cast<OV5645::Register>(0x3614), 0x50 },
	{ static_cast<OV5645::Register>(0x3618), 0x04 },
	{ static_cast<OV5645::Register>(0x3034), 0x18 },
	{ static_cast<OV5645::Register>(0x3035), 0x11 },
	{ static_cast<OV5645::Register>(0x3036), 0x54 },
	{ static_cast<OV5645::Register>(0x3600), 0x08 },
	{ static_cast<OV5645::Register>(0x3601), 0x33 },
	{ static_cast<OV5645::Register>(0x3708), 0x63 },
	{ static_cast<OV5645::Register>(0x370c), 0xc0 },
	{ static_cast<OV5645::Register>(0x3800), 0x00 },
	{ static_cast<OV5645::Register>(0x3801), 0x00 },
	{ static_cast<OV5645::Register>(0x3802), 0x00 },
	{ static_cast<OV5645::Register>(0x3803), 0x00 },
	{ static_cast<OV5645::Register>(0x3804), 0x0a },
	{ static_cast<OV5645::Register>(0x3805), 0x3f },
	{ static_cast<OV5645::Register>(0x3806), 0x07 },
	{ static_cast<OV5645::Register>(0x3807), 0x9f },
	{ static_cast<OV5645::Register>(0x3808), 0x0a },
	{ static_cast<OV5645::Register>(0x3809), 0x20 },
	{ static_cast<OV5645::Register>(0x380a), 0x07 },
	{ static_cast<OV5645::Register>(0x380b), 0x98 },
	{ static_cast<OV5645::Register>(0x380c), 0x0b },
	{ static_cast<OV5645::Register>(0x380d), 0x1c },
	{ static_cast<OV5645::Register>(0x380e), 0x07 },
	{ static_cast<OV5645::Register>(0x380f), 0xb0 },
	{ static_cast<OV5645::Register>(0x3813), 0x06 },
	{ static_cast<OV5645::Register>(0x3814), 0x11 },
	{ static_cast<OV5645::Register>(0x3815), 0x11 },
	{ static_cast<OV5645::Register>(0x3820), 0x47 },
	{ static_cast<OV5645::Register>(0x4514), 0x88 },
	{ static_cast<OV5645::Register>(0x3a02), 0x07 },
	{ static_cast<OV5645::Register>(0x3a03), 0xb0 },
	{ static_cast<OV5645::Register>(0x3a08), 0x01 },
	{ static_cast<OV5645::Register>(0x3a09), 0x27 },
	{ static_cast<OV5645::Register>(0x3a0a), 0x00 },
	{ static_cast<OV5645::Register>(0x3a0b), 0xf6 },
	{ static_cast<OV5645::Register>(0x3a0e), 0x06 },
	{ static_cast<OV5645::Register>(0x3a0d), 0x08 },
	{ static_cast<OV5645::Register>(0x3a14), 0x07 },
	{ static_cast<OV5645::Register>(0x3a15), 0xb0 },
	{ static_cast<OV5645::Register>(0x3a18), 0x01 },
	{ static_cast<OV5645::Register>(0x4004), 0x06 },
	{ static_cast<OV5645::Register>(0x4005), 0x18 },
	{ static_cast<OV5645::Register>(0x4300), 0x32 },
	{ static_cast<OV5645::Register>(0x4837), 0x0b },
	{ static_cast<OV5645::Register>(0x4202), 0x00 }
};

Status OV5645::Init(const Config &config) {
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

	// Load Magic Global Initialization Blob
	uint16_t size = sizeof(ov5645GlobalInit) / sizeof(ov5645GlobalInit[0]);
	status = this->WriteRegisterArray(ov5645GlobalInit, size);
	if(status != Status::Ok) {
		return status;
	}
	tx_thread_sleep(300);

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

	// Set Power Off
	// this->WriteRegister(OV5645::Register::IO_MIPI_CTRL00, 0x58);

	return Status::Ok;
}

Status OV5645::Reset() {
	if(this->config.resetPin != nullptr) {
		this->config.resetPin->Write(0);
		tx_thread_sleep(10);
		this->config.resetPin->Write(1);
		tx_thread_sleep(20);
		return Status::Ok;
	}
	else {
		Status status = this->WriteRegister(OV5645::Register::SYSTEM_CTRL0, 0x82);
		tx_thread_sleep(20);
		return status;
	}
}

Status OV5645::Start() {
	if(config.powerDownPin != nullptr) {
		config.powerDownPin->Write(0);
		tx_thread_sleep(5);
	}
	Status status = this->WriteRegister(OV5645::Register::IO_MIPI_CTRL00, 0x45);
	if(status != Status::Ok) {
		return status;
	}
	return this->WriteRegister(OV5645::Register::SYSTEM_CTRL0, 0x02);
}

Status OV5645::Stop() {
	Status status = this->WriteRegister(OV5645::Register::IO_MIPI_CTRL00, 0x40);
	if(status != Status::Ok) {
		return status;
	}
	status = this->WriteRegister(OV5645::Register::SYSTEM_CTRL0, 0x42);
	if(config.powerDownPin != nullptr) {
		config.powerDownPin->Write(1);
	}
	return status;
}

Status OV5645::ReadID(uint16_t *id) {
	uint8_t pid, vid;
	if(this->ReadRegister(OV5645::Register::CHIP_ID_HIGH, pid) != Status::Ok) {
		return Status::Error;
	}
	if(this->ReadRegister(OV5645::Register::CHIP_ID_LOW, vid) != Status::Ok) {
		return Status::Error;
	}
	*id = (pid << 8) + vid;
	return Status::Ok;
}

Status OV5645::SetResolution(uint16_t width, uint16_t height) {
	// Get closest resolution that can be used and is implemented
	OV5645::InternalResolution resolution = this->FindNearestResolution(width, height);

	if(resolution == OV5645::InternalResolution::UNKNOWN) {
		return Status::Error;
	}

	// Update config to used resolution
	switch (resolution) {
		case InternalResolution::QSXGA:
			this->config.width = 2592;
			this->config.height = 1944;
			break;
		case InternalResolution::FHD:
			this->config.width = 1920;
			this->config.height = 1080;
			break;
		case InternalResolution::SXGA:
			this->config.width = 1280;
			this->config.height = 1024;
			break;
		case InternalResolution::VGA:
			this->config.width = 640;
			this->config.height = 480;
			break;
		default:
			break;;
	}

	// Apply hardware scaling/resolution
	Status status = this->ApplyResolution(resolution);
	if(status != Status::Ok) {
		return status;
	}

	return Status::Ok;
}

Status OV5645::SetFormat(PixelFormat format) {
	Status status = Status::Ok;
	switch(format) {
		case PixelFormat::YUV422_YUYV:
			// YUV/YCbCr 4:2:2
			// Output format of formatter module (Bit[7:4]): 0x0: RAW; 0x1: Y8; 0x2: UV444/RGB888; 0x3: YUV422; 0x4: YUV420; 0x5: YUV420 (MIPI only); 0x6: RGB565; 0x7: RGB555; 0x8: RGB555; 0x9: RGB444; 0xA: RGB444
			// Output sequence (Bit[3:0]) YUV422: 0x0: YUYV; 0x1: YVYU; 0x2: UYVY; 0x3: VYUY;
			status = this->WriteRegister(OV5645::Register::FORMAT_CTRL00, 0x30);
			if(status == Status::Ok) {
				//Format select: 000: ISP YUV422; 001: ISP RGB; 010: ISP dither; 011: ISP RAW (DPC); 100: SNR RAW; 101: ISP RAW (CIP)
				status = this->WriteRegister(OV5645::Register::FORMAT_MUX_CTRL, 0x00);
			}
			break;
		case PixelFormat::YUV422_YVYU: {
			// YUV/YCbCr 4:2:2
			status = this->WriteRegister(OV5645::Register::FORMAT_CTRL00, 0x31);
			if(status == Status::Ok) {
				status = this->WriteRegister(OV5645::Register::FORMAT_MUX_CTRL, 0x00);
			}
			break;
		}
		case PixelFormat::YUV422_UYVY:
			// UYVY 4:2:2
			status = this->WriteRegister(OV5645::Register::FORMAT_CTRL00, 0x32);
			if(status == Status::Ok) {
				status = this->WriteRegister(OV5645::Register::FORMAT_MUX_CTRL, 0x00);
			}
			break;
		case PixelFormat::RGB565:
			// RGB565
			// Output format of formatter module (Bit[7:4]): 
			// Output sequence (Bit[3:0]) RAW: 0x0: BGBG../GRGR..; 0x1: GBGB../RGRG..; 0x2: GRGR../BGBG..; 0x3: RGRG../GBGB..
			status = this->WriteRegister(OV5645::Register::FORMAT_CTRL00, 0x60);
			if(status == Status::Ok) {
				status = this->WriteRegister(OV5645::Register::FORMAT_MUX_CTRL, 0x01);
			}
			break;
		default:
			return Status::Error; // Unsupported format
	}
	return status;
}

Status OV5645::SetFPS(uint32_t fps) {
	return Status::Ok;
}

Status OV5645::SetTestPattern(bool enable) {
	Status status = Status::Ok;
	if(enable == true) {
		// Test pattern: DSP color bar (pre ISP)
		status = this->ModifyRegister(static_cast<OV5645::Register>(0x503D), 0x83, 0x80);
		// Incremental test pattern in VFIFO
		// status = this->ModifyRegister(static_cast<OV5645::Register>(0x4600), 0x10, 0x10);
	}
	else {
		// No test pattern (pre ISP)
		status = this->ModifyRegister(static_cast<OV5645::Register>(0x503D), 0x83, 0x00);
		// Incremental test pattern in VFIFO
		// status = this->ModifyRegister(static_cast<OV5645::Register>(0x4600), 0x10, 0x00);
	}
	return status;
}

OV5645::InternalResolution OV5645::FindNearestResolution(uint16_t w, uint16_t h) {
	if(w >= 2592) {
		return InternalResolution::QSXGA;
	}
	if(w >= 1920) {
		return InternalResolution::FHD;
	}
	else if(w >= 1280) {
		return InternalResolution::SXGA;
	}
	else if(w >= 640) {
		return InternalResolution::VGA;
	}
	return InternalResolution::UNKNOWN;
}

Status OV5645::ApplyResolution(InternalResolution resolution) {
	switch(resolution) {
		case InternalResolution::QSXGA: {
			// QSXGA/Full resolution (2592x1944)
			uint16_t size = sizeof(ov5645QSXGA) / sizeof(ov5645QSXGA[0]);
			return this->WriteRegisterArray(ov5645QSXGA, size);
		}
		case InternalResolution::FHD: {
			// Full HD (1920x1080)
			uint16_t size = sizeof(ov5645FHD) / sizeof(ov5645FHD[0]);
			return this->WriteRegisterArray(ov5645FHD, size);
		}
		case InternalResolution::SXGA: {
			// SXGA (1280x1024)
			uint16_t size = sizeof(ov5645SXGA) / sizeof(ov5645SXGA[0]);
			return this->WriteRegisterArray(ov5645SXGA, size);
		}
		case InternalResolution::VGA: {
			// VGA (640x480)
			uint16_t size = sizeof(ov5645VGA) / sizeof(ov5645VGA[0]);
			return this->WriteRegisterArray(ov5645VGA, size);
		}
		default:
			return Status::Error;
	}
	return Status::Ok;
}

Status OV5645::ApplyWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
	return Status::Ok;
}

Status OV5645::WriteRegisterArray(const RegisterValuePair* array, uint16_t size) {
	if(array == nullptr || size == 0) {
		return Status::Error;
	}

	for(uint16_t i = 0; i < size; i++) {
		Status status = this->WriteRegister(array[i].reg, array[i].value);
		if(status != Status::Ok) {
			return status; 
		}

		if(array[i].reg == OV5645::Register::SYSTEM_CTRL0 && array[i].value == 0x42) {
			tx_thread_sleep(2);
		}
	}
	return Status::Ok;
}

Status OV5645::WriteRegister(Register reg, uint8_t value) {
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

Status OV5645::ReadRegister(Register reg, uint8_t& value) {
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

Status OV5645::ModifyRegister(Register reg, uint8_t mask, uint8_t value) {
	uint8_t regVal;
	Status status = this->ReadRegister(reg, regVal);
	if(status != Status::Ok) {
		return status;
	}

	regVal &= ~mask;
	regVal |= (value & mask);

	return this->WriteRegister(reg, regVal);
}