/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Sensors/icm45686.hpp
 * Author:  NotBlackMagic
 * Brief:   ICM-45686 6-Axis IMU driver class for STM32N6.
 */

#pragma once

#include <stdint.h>

#include "spi.hpp"
#include "status.hpp"
#include "system.hpp"

class ICM45686 {
	public:
		// Standard Chip identifications
		static constexpr uint8_t i2cAddrPrimary = 0x68;
		static constexpr uint8_t i2cAddrSecondary = 0x69;
		static constexpr uint8_t chipID = 0xE9;

		/// @brief Device register map.
		enum class Register : uint8_t {
			ACCEL_DATA_X1_UI = 0x00,
			ACCEL_DATA_X0_UI = 0x01,
			ACCEL_DATA_Y1_UI = 0x02,
			ACCEL_DATA_Y0_UI = 0x03,
			ACCEL_DATA_Z1_UI = 0x04,
			ACCEL_DATA_Z0_UI = 0x05,
			GYRO_DATA_X1_UI = 0x06,
			GYRO_DATA_X0_UI = 0x07,
			GYRO_DATA_Y1_UI = 0x08,
			GYRO_DATA_Y0_UI = 0x09,
			GYRO_DATA_Z1_UI = 0x0A,
			GYRO_DATA_Z0_UI = 0x0B,
			TEMP_DATA1_UI = 0x0C,
			TEMP_DATA0_UI = 0x0D,
			TMST_FSYNCH = 0x0E,
			TMST_FSYNCL = 0x0F,
			PWR_MGMT0 = 0x10,
			FIFO_COUNT_0 = 0x12,
			FIFO_COUNT_1 = 0x13,
			FIFO_DATA = 0x14,
			INT1_CONFIG0 = 0x16,
			INT1_CONFIG1 = 0x17,
			INT1_CONFIG2 = 0x18,
			INT1_STATUS0 = 0x19,
			INT1_STATUS1 = 0x1A,
			ACCEL_CONFIG0 = 0x1B,
			GYRO_CONFIG0 = 0x1C,
			FIFO_CONFIG0 = 0x1D,
			FIFO_CONFIG1_0 = 0x1E,
			FIFO_CONFIG1_1 = 0x1F,
			FIFO_CONFIG2 = 0x20,
			FIFO_CONFIG3 = 0x21,
			FIFO_CONFIG4 = 0x22,
			TMST_WOM_CONFIG = 0x23,
			FSYNC_CONFIG0 = 0x24,
			FSYNC_CONFIG1 = 0x25,
			RTC_CONFIG = 0x26,
			DMP_EXT_SEN_ODR_CFG = 0x27,
			ODR_DECIMATE_CONFIG = 0x28,
			EDMP_APEX_EN0 = 0x29,
			EDMP_APEX_EN1 = 0x2A,
			APEX_BUFFER_MGMT = 0x2B,
			INTF_CONFIG0 = 0x2C,
			INTF_CONFIG1_OVRD = 0x2D,
			INTF_AUX_CONFIG = 0x2E,
			IOC_PAD_SCENARIO = 0x2F,
			IOC_PAD_SCENARIO_AUX_OVRD = 0x30,
			DRIVE_CONFIG0 = 0x32,
			DRIVE_CONFIG1 = 0x33,
			DRIVE_CONFIG2 = 0x34,
			INT_APEX_CONFIG0 = 0x39,
			INT_APEX_CONFIG1 = 0x3A,
			INT_APEX_STATUS0 = 0x3B,
			INT_APEX_STATUS1 = 0x3C,
			ACCEL_DATA_X1_AUX1 = 0x44,
			ACCEL_DATA_X0_AUX1 = 0x45,
			ACCEL_DATA_Y1_AUX1 = 0x46,
			ACCEL_DATA_Y0_AUX1 = 0x47,
			ACCEL_DATA_Z1_AUX1 = 0x48,
			ACCEL_DATA_Z0_AUX1 = 0x49,
			GYRO_DATA_X1_AUX1 = 0x4A,
			GYRO_DATA_X0_AUX1 = 0x4B,
			GYRO_DATA_Y1_AUX1 = 0x4C,
			GYRO_DATA_Y0_AUX1 = 0x4D,
			GYRO_DATA_Z1_AUX1 = 0x4E,
			GYRO_DATA_Z0_AUX1 = 0x4F,
			TEMP_DATA1_AUX1 = 0x50,
			TEMP_DATA0_AUX1 = 0x51,
			TMST_FSYNCH_AUX1 = 0x52,
			TMST_FSYNCL_AUX1 = 0x53,
			PWR_MGMT_AUX1 = 0x54,
			FS_SEL_AUX1 = 0x55,
			INT2_CONFIG0 = 0x56,
			INT2_CONFIG1 = 0x57,
			INT2_CONFIG2 = 0x58,
			INT2_STATUS0 = 0x59,
			INT2_STATUS1 = 0x5A,
			WHO_AM_I = 0x72,
			REG_HOST_MSG = 0x73,
			IREG_ADDR_15_8 = 0x7C,
			IREG_ADDR_7_0 = 0x7D,
			IREG_DATA = 0x7E,
			REG_MISC2 = 0x7F
		};

		/// @brief Indirect registers (IREG) address offsets.
		enum class IREGBase : uint8_t {
			SRAM = 0x00,
			BAR = 0xA0,
			SYS1 = 0xA4,
			SYS2 = 0xA5,
			TOP1 = 0xA2
		};

		/// @brief Device user bank IMEM register map.
		enum class IMEMSRAM : uint16_t {
			IMEM_SRAM_REG_0 = 0x00,
			IMEM_SRAM_REG_1 = 0x01,
			IMEM_SRAM_REG_2 = 0x02,
			IMEM_SRAM_REG_3 = 0x03,
			IMEM_SRAM_REG_4 = 0x04,
			IMEM_SRAM_REG_5 = 0x05,
			IMEM_SRAM_REG_6 = 0x06,
			IMEM_SRAM_REG_7 = 0x07,
			IMEM_SRAM_REG_8 = 0x08,
			IMEM_SRAM_REG_9 = 0x09,
			IMEM_SRAM_REG_10 = 0x0A,
			IMEM_SRAM_REG_11 = 0x0B,
			IMEM_SRAM_REG_56 = 0x38,
			IMEM_SRAM_REG_57 = 0x39,
			IMEM_SRAM_REG_64 = 0x40,
			IMEM_SRAM_REG_68 = 0x44,
			IMEM_SRAM_REG_92 = 0x5C,
			IMEM_SRAM_REG_96 = 0x60,
			IMEM_SRAM_REG_97 = 0x61,
			IMEM_SRAM_REG_98 = 0x62,
			IMEM_SRAM_REG_99 = 0x63,
			IMEM_SRAM_REG_100 = 0x64,
			IMEM_SRAM_REG_101 = 0x65,
			IMEM_SRAM_REG_102 = 0x66,
			IMEM_SRAM_REG_103 = 0x67,
			IMEM_SRAM_REG_104 = 0x68,
			IMEM_SRAM_REG_105 = 0x69,
			IMEM_SRAM_REG_106 = 0x6A,
			IMEM_SRAM_REG_107 = 0x6B,
			IMEM_SRAM_REG_136 = 0x88,
			IMEM_SRAM_REG_137 = 0x89,
			IMEM_SRAM_REG_138 = 0x8A,
			IMEM_SRAM_REG_139 = 0x8B,
			IMEM_SRAM_REG_141 = 0x8D,
			IMEM_SRAM_REG_142 = 0x8E,
			IMEM_SRAM_REG_143 = 0x8F,
			IMEM_SRAM_REG_144 = 0x90,
			IMEM_SRAM_REG_146 = 0x92,
			IMEM_SRAM_REG_154 = 0x9A,
			IMEM_SRAM_REG_155 = 0x9B,
			IMEM_SRAM_REG_156 = 0x9C,
			IMEM_SRAM_REG_157 = 0x9D,
			IMEM_SRAM_REG_159 = 0x9F,
			IMEM_SRAM_REG_160 = 0xA0,
			IMEM_SRAM_REG_182 = 0xB6,
			IMEM_SRAM_REG_185 = 0xB9,
			IMEM_SRAM_REG_186 = 0xBA,
			IMEM_SRAM_REG_196 = 0xC4,
			IMEM_SRAM_REG_197 = 0xC5,
			IMEM_SRAM_REG_198 = 0xC6,
			IMEM_SRAM_REG_199 = 0xC7,
			IMEM_SRAM_REG_288 = 0x120,
			IMEM_SRAM_REG_289 = 0x121,
			IMEM_SRAM_REG_290 = 0x122,
			IMEM_SRAM_REG_291 = 0x123,
			IMEM_SRAM_REG_292 = 0x124,
			IMEM_SRAM_REG_293 = 0x125,
			IMEM_SRAM_REG_294 = 0x126,
			IMEM_SRAM_REG_295 = 0x127,
			IMEM_SRAM_REG_296 = 0x128,
			IMEM_SRAM_REG_297 = 0x129,
			IMEM_SRAM_REG_298 = 0x12A,
			IMEM_SRAM_REG_299 = 0x12B,
			IMEM_SRAM_REG_304 = 0x130,
			IMEM_SRAM_REG_305 = 0x131,
			IMEM_SRAM_REG_306 = 0x132,
			IMEM_SRAM_REG_307 = 0x133,
			IMEM_SRAM_REG_308 = 0x134,
			IMEM_SRAM_REG_309 = 0x135,
			IMEM_SRAM_REG_316 = 0x13C,
			IMEM_SRAM_REG_317 = 0x13D,
			IMEM_SRAM_REG_318 = 0x13E,
			IMEM_SRAM_REG_319 = 0x13F,
			IMEM_SRAM_REG_320 = 0x140,
			IMEM_SRAM_REG_321 = 0x141,
			IMEM_SRAM_REG_392 = 0x188,
			IMEM_SRAM_REG_393 = 0x189,

			IMEM_SRAM_REG_400 = 0x190,
			IMEM_SRAM_REG_401 = 0x191,
			IMEM_SRAM_REG_402 = 0x192,
			IMEM_SRAM_REG_403 = 0x193,
			IMEM_SRAM_REG_404 = 0x194,
			IMEM_SRAM_REG_405 = 0x195,
			IMEM_SRAM_REG_406 = 0x196,
			IMEM_SRAM_REG_540 = 0x21C,
			IMEM_SRAM_REG_541 = 0x21D,
			IMEM_SRAM_REG_542 = 0x21E,
			IMEM_SRAM_REG_543 = 0x21F,
			IMEM_SRAM_REG_544 = 0x220,
			IMEM_SRAM_REG_545 = 0x221,
			IMEM_SRAM_REG_546 = 0x222,
			IMEM_SRAM_REG_547 = 0x223,
			IMEM_SRAM_REG_548 = 0x224,
			IMEM_SRAM_REG_549 = 0x225,
			IMEM_SRAM_REG_550 = 0x226,
			IMEM_SRAM_REG_551 = 0x227,
			IMEM_SRAM_REG_556 = 0x22C,
			IMEM_SRAM_REG_557 = 0x22D,
			IMEM_SRAM_REG_558 = 0x22E,
			IMEM_SRAM_REG_559 = 0x22F,
			IMEM_SRAM_REG_560 = 0x230,
			IMEM_SRAM_REG_561 = 0x231,
			IMEM_SRAM_REG_562 = 0x232,
			IMEM_SRAM_REG_563 = 0x233,
			IMEM_SRAM_REG_564 = 0x234,
			IMEM_SRAM_REG_565 = 0x235,
			IMEM_SRAM_REG_566 = 0x236,
			IMEM_SRAM_REG_567 = 0x237,
			IMEM_SRAM_REG_568 = 0x238,
			IMEM_SRAM_REG_569 = 0x239,
			IMEM_SRAM_REG_570 = 0x23A,
			IMEM_SRAM_REG_571 = 0x23B,
			IMEM_SRAM_REG_572 = 0x23C,
			IMEM_SRAM_REG_573 = 0x23D,
			IMEM_SRAM_REG_574 = 0x23E,
			IMEM_SRAM_REG_575 = 0x23F,
			IMEM_SRAM_REG_576 = 0x240,
			IMEM_SRAM_REG_577 = 0x241,
			IMEM_SRAM_REG_578 = 0x242,
			IMEM_SRAM_REG_579 = 0x243,
			IMEM_SRAM_REG_580 = 0x244,
			IMEM_SRAM_REG_581 = 0x245,
			IMEM_SRAM_REG_582 = 0x246,
			IMEM_SRAM_REG_583 = 0x247,
			IMEM_SRAM_REG_584 = 0x248,
			IMEM_SRAM_REG_585 = 0x249,
			IMEM_SRAM_REG_586 = 0x24A,
			IMEM_SRAM_REG_587 = 0x24B,
			IMEM_SRAM_REG_988 = 0x3DC,
			IMEM_SRAM_REG_989 = 0x3DD,
			IMEM_SRAM_REG_990 = 0x3DE,
			IMEM_SRAM_REG_991 = 0x3DF,
			IMEM_SRAM_REG_994 = 0x3E2,
			IMEM_SRAM_REG_995 = 0x3E3,
			IMEM_SRAM_REG_1000 = 0x3E8,
			IMEM_SRAM_REG_1001 = 0x3E9,
			IMEM_SRAM_REG_1002 = 0x3EA,
			IMEM_SRAM_REG_1003 = 0x3EB,
			IMEM_SRAM_REG_1004 = 0x3EC,
			IMEM_SRAM_REG_1008 = 0x3F0,
			IMEM_SRAM_REG_1009 = 0x3F1,
			IMEM_SRAM_REG_1010 = 0x3F2,
			IMEM_SRAM_REG_1011 = 0x3F3,
			IMEM_SRAM_REG_1016 = 0x3F8,
			IMEM_SRAM_REG_1017 = 0x3F9,
			IMEM_SRAM_REG_1018 = 0x3FA,
			IMEM_SRAM_REG_1019 = 0x3FB,
			IMEM_SRAM_REG_1042 = 0x412,
			IMEM_SRAM_REG_1168 = 0x490,
			//1168 to 1203
			IMEM_SRAM_REG_1203 = 0x4B3
		};

		/// @brief Device user bank IPREG_BAR register map.
		enum class IPREGBAR : uint8_t {
			IPREG_BAR_REG_57 = 0x39,
			IPREG_BAR_REG_58 = 0x3A,
			IPREG_BAR_REG_59 = 0x3B,
			IPREG_BAR_REG_60 = 0x3C,
			IPREG_BAR_REG_61 = 0x3D,
			IPREG_BAR_REG_62 = 0x3E
		};

		/// @brief Device user bank IPREG_TOP1 register map.
		enum class IPREGTOP1 : uint8_t {
			I2CM_COMMAND_0 = 0x06,
			I2CM_COMMAND_1 = 0x07,
			I2CM_COMMAND_2 = 0x08,
			I2CM_COMMAND_3 = 0x09,
			I2CM_DEV_PROFILE0 = 0x0E,
			I2CM_DEV_PROFILE1 = 0x0F,
			I2CM_DEV_PROFILE2 = 0x10,
			I2CM_DEV_PROFILE3 = 0x11,
			I2CM_CONTROL = 0x16,
			I2CM_STATUS = 0x18,
			I2CM_EXT_DEV_STATUS = 0x1A,
			I2CM_RD_DATA0 = 0x1B,
			I2CM_RD_DATA1 = 0x1C,
			I2CM_RD_DATA2 = 0x1D,
			I2CM_RD_DATA3= 0x1E,
			I2CM_RD_DATA4 = 0x1F,
			I2CM_RD_DATA5 = 0x20,
			I2CM_RD_DATA6 = 0x21,
			I2CM_RD_DATA7 = 0x22,
			I2CM_RD_DATA8 = 0x23,
			I2CM_RD_DATA9 = 0x24,
			I2CM_RD_DATA10 = 0x25,
			I2CM_RD_DATA11 = 0x26,
			I2CM_RD_DATA12 = 0x27,
			I2CM_RD_DATA13 = 0x28,
			I2CM_RD_DATA14 = 0x29,
			I2CM_RD_DATA15 = 0x2A,
			I2CM_RD_DATA16 = 0x2B,
			I2CM_RD_DATA17 = 0x2C,
			I2CM_RD_DATA18 = 0x2D,
			I2CM_RD_DATA19 = 0x2E,
			I2CM_RD_DATA20 = 0x2F,
			I2CM_WR_DATA0 = 0x33,
			I2CM_WR_DATA1 = 0x34,
			I2CM_WR_DATA2 = 0x35,
			I2CM_WR_DATA3 = 0x36,
			I2CM_WR_DATA4 = 0x37,
			I2CM_WR_DATA5 = 0x38,
			SIFS_IXC_ERROR_STATUS = 0x4B,
			EDMP_PRGRM_IRQ0_0 = 0x4F,
			EDMP_PRGRM_IRQ0_1 = 0x50,
			EDMP_PRGRM_IRQ1_0 = 0x51,
			EDMP_PRGRM_IRQ1_1 = 0x52,
			EDMP_PRGRM_IRQ2_0 = 0x53,
			EDMP_PRGRM_IRQ2_1 = 0x54,
			EDMP_SP_START_ADDR = 0x55,
			SMC_CONTROL_0 = 0x58,
			SMC_CONTROL_1 = 0x59,
			STC_CONFIG = 0x63,
			SREG_CTRL = 0x67,
			SIFS_I3C_STC_CFG = 0x68,
			INT_PULSE_MIN_ON_INTF0 = 0x69,
			INT_PULSE_MIN_ON_INTF1 = 0x6A,
			INT_PULSE_MIN_OFF_INTF0 = 0x6B,
			INT_PULSE_MIN_OFF_INTF1 = 0x6C,
			ISR_0_7 = 0x6E,
			ISR_8_15 = 0x6F,
			ISR_16_23 = 0x70,
			STATUS_MASK_PIN_0_7 = 0x71,
			STATUS_MASK_PIN_8_15 = 0x72,
			STATUS_MASK_PIN_16_23 = 0x73,
			INT_I2CM_SOURCE = 0x74,
			ACCEL_WOM_X_THR = 0x7E,
			ACCEL_WOM_Y_THR = 0x7F,
			ACCEL_WOM_Z_THR = 0x80,
			SELFTEST = 0x90,
			IPREG_MISC = 0x97,
			SW_PLL1_TRIM = 0xA2,
			FIFO_SRAM_SLEEP = 0xA7
		};

		/// @brief Device user bank IPREG_SYS1 register map.
		enum class IPREGSYS1 : uint8_t {
			IPREG_SYS1_REG_42 = 0x2A,
			IPREG_SYS1_REG_43 = 0x2B,
			IPREG_SYS1_REG_56 = 0x38,
			IPREG_SYS1_REG_57 = 0x39,
			IPREG_SYS1_REG_56n = 0x46,
			IPREG_SYS1_REG_57n = 0x47,
			IPREG_SYS1_REG_166 = 0xA6,
			IPREG_SYS1_REG_168 = 0xA8,
			IPREG_SYS1_REG_170 = 0xAA,
			IPREG_SYS1_REG_171 = 0xAB,
			IPREG_SYS1_REG_172 = 0xAC
		};

		/// @brief Device user bank IPREG_SYS2 register map.
		enum class IPREGSYS2 : uint8_t {
			IPREG_SYS2_REG_24 = 0x18,
			IPREG_SYS2_REG_25 = 0x19,
			IPREG_SYS2_REG_32 = 0x20,
			IPREG_SYS2_REG_33 = 0x21,
			IPREG_SYS2_REG_40 = 0x28,
			IPREG_SYS2_REG_41 = 0x29,
			IPREG_SYS2_REG_123 = 0x7B,
			IPREG_SYS2_REG_129 = 0x81,
			IPREG_SYS2_REG_130 = 0x82,
			IPREG_SYS2_REG_131 = 0x83,
			IPREG_SYS2_REG_132 = 0x84,
		};

		/// @brief Supported output data rates (ODR).
		enum class OutputDataRate : uint8_t {
			Hz6400 = 0x03,
			Hz3200 = 0x04,
			Hz1600 = 0x05,
			Hz800 = 0x06,
			Hz400 = 0x07,
			Hz200 = 0x08,
			Hz100 = 0x09,
			Hz50 = 0x0A,
			Hz25 = 0x0B,
			Hz12_5 = 0x0C,
			Hz6_25 = 0x0D,
			Hz3_125 = 0x0E,
			Hz1_5625 = 0x0F
		};

		/// @brief Supported accelerometer output full scale (FS).
		enum class AccelScale : uint8_t {
			G32 = 0x00,
			G16 = 0x01,
			G8 = 0x02,
			G4 = 0x03,
			G2 = 0x04
		};

		/// @brief Supported gyroscope output full scale (FS).
		enum class GyroScale : uint8_t {
			DPS4000 = 0x00,
			DPS2000 = 0x01,
			DPS1000 = 0x02,
			DPS500 = 0x03,
			DPS250 = 0x04,
			DPS125 = 0x05,
			DPS62_5 = 0x06,
			DPS31_25 = 0x07,
			DPS15_625 = 0x08
		};

		/// @brief ICM-45686 sensor configuration structure.
		struct Config {
			AccelScale accelScale;
			GyroScale gyroScale;
			OutputDataRate accelOdr;
			OutputDataRate gyroOdr;
		};

		/// @brief Constructor.
		/// @param spi	Reference to the low-level bus driver.
		ICM45686(SPI& spi) : bus(spi) {};

		/// @brief Initializes the ICM-45686 magnetometer.
		/// @param config ICM-45686 magnetometer configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid or failed.
		Status Init(const Config& config);

		/// @brief Resets the sensor device, using software reset.
		/// @return Status::Ok if reset succeeded cleanly.
		Status Reset();

		/// @brief Reads the Manufacturer and Device IDs.
		/// @param id Device ID, or manufacturer ID.
		/// @return Status::Ok if read succeeded, or Status::Error if failed.
		Status ReadID(uint8_t& id);

		/// @brief Sets the sensors data output full scale.
		/// @param accelScale	Accelerometer output full scale.
		/// @param gyroScale	Gyroscope output full scale.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetScales(AccelScale accelScale, GyroScale gyroScale);

		/// @brief Sets the accelerometer offset value, to be added to the read value.
		/// @param offsetX Offset for x-axis.
		/// @param offsetY Offset for y-axis.
		/// @param offsetZ Offset for z-axis.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetAccelOffsets(float offsetX, float offsetY, float offsetZ);

		/// @brief Sets the gyroscope offset value, to be added to the read value.
		/// @param offsetX Offset for x-axis.
		/// @param offsetY Offset for y-axis.
		/// @param offsetZ Offset for z-axis.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status SetGyroOffsets(float offsetX, float offsetY, float offsetZ);

		/// @brief Performs a hardware self test.
		/// @return Status::Ok if self test passed, Status::Error if failed.
		Status RunHardwareSelfTest();

		/// @brief Start a non-blocking data transfer request (calls TransferAsync of the underlying bus).
		/// @details Returns immediately. Use TransferWait() to synchronize completion.
		/// @return Status::Ok if the transfer started, or Status::Busy if the bus is locked by another thread.
		Status RequestData();

		/// @brief Blocks the current thread until data transfer completes.
		/// @param accel	Pointer to array be filled with received accelerometer data (x, y, z).
		/// @param gyro		Pointer to array be filled with received gyroscope data (x, y, z).
		/// @param temp		Pointer to be filled with received temperature readings.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status GetData(float* accel, float* gyro, float* temp);

	private:
		SPI& bus;
		Config config;

		static constexpr uint16_t transferSize = 32;
		__attribute__((aligned(32))) uint8_t txBuffer[transferSize];
		__attribute__((aligned(32))) uint8_t rxBuffer[transferSize];

		static constexpr float accelSens[] = {
			32.0f / 32768.0f,
			16.0f / 32768.0f,
			8.0f / 32768.0f,
			4.0f / 32768.0f,
			2.0f / 32768.0f
		};

		static constexpr float gyroSens[] = {
			4000.0f / 32768.0f,
			2000.0f / 32768.0f,
			1000.0f / 32768.0f,
			500.0f / 32768.0f,
			250.0f / 32768.0f,
			125.0f / 32768.0f,
			62.5f / 32768.0f,
			31.25f / 32768.0f,
			15.625f / 32768.0f
		};

		static constexpr float tempSens = (1.0f/256);
		
		float accelOffset[3];
		float gyroOffset[3];
		static constexpr float tempOffset = 25.0f;

		Status WriteIREG(IREGBase base, uint8_t reg, uint8_t value);
		Status ReadIREG(IREGBase base, uint8_t reg, uint8_t& value);

		Status WriteRegister(Register reg, uint8_t value);
		Status ReadRegister(Register reg, uint8_t& value);
		Status ModifyRegister(Register reg, uint8_t mask, uint8_t value);
		float ParseAxis(uint8_t msb, uint8_t lsb, float scaleFactor, float offset);
};