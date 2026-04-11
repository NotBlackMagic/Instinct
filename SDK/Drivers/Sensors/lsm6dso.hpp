/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/Sensors/lsm6dso.hpp
 * Author:  NotBlackMagic
 * Brief:   LSM6DSO 6-Axis IMU driver class for STM32N6.
 */

#pragma once

#include <stdint.h>

#include "spi.hpp"
#include "status.hpp"

#include "tx_api.h"

class LSM6DSO {
	public:
		// Standard Chip identifications
		static constexpr uint8_t i2cAddrPrimary = 0x6A;
		static constexpr uint8_t i2cAddrSecondary = 0x6B;
		static constexpr uint8_t chipID = 0xE9;

		enum class Register : uint8_t {
			FUNC_CFG_ACCESS = 0x01,
			PIN_CTRL = 0x02,
			FIFO_CTRL1 = 0x07,
			FIFO_CTRL2 = 0x08,
			FIFO_CTRL3 = 0x09,
			FIFO_CTRL4 = 0x0A,
			COUNTER_BDR_REG1 = 0x0B,
			COUNTER_BDR_REG2 = 0x0C,
			INT1_CTRL = 0x0D,
			INT2_CTRL = 0x0E,
			WHO_AM_I = 0x0F,
			CTRL1_XL = 0x10,
			CTRL2_G = 0x11,
			CTRL3_C = 0x12,
			CTRL4_C = 0x13,
			CTRL5_C = 0x14,
			CTRL6_C = 0x15,
			CTRL7_G = 0x16,
			CTRL8_XL = 0x17,
			CTRL9_XL = 0x18,
			CTRL10_C = 0x19,
			ALL_INT_SRC = 0x1A,
			WAKE_UP_SRC = 0x1B,
			TAP_SRC = 0x1C,
			D6D_SRC = 0x1D,
			STATUS_REG = 0x1E,
			STATUS_SPIAUX = 0x1E,
			OUT_TEMP_L = 0x20,
			OUT_TEMP_H = 0x21,
			OUTX_L_G = 0x22,
			OUTX_H_G = 0x23,
			OUTY_L_G = 0x24,
			OUTY_H_G = 0x25,
			OUTZ_L_G = 0x26,
			OUTZ_H_G = 0x27,
			OUTX_L_A = 0x28,
			OUTX_H_A = 0x29,
			OUTY_L_A = 0x2A,
			OUTY_H_A = 0x2B,
			OUTZ_L_A = 0x2C,
			OUTZ_H_A = 0x2D,
			EMB_FUNC_STATUS_MAINPAGE = 0x35,
			FSM_STATUS_A_MAINPAGE = 0x36,
			FSM_STATUS_B_MAINPAGE = 0x37,
			STATUS_MASTER_MAINPAGE = 0x39,
			FIFO_STATUS1 = 0x3A,
			FIFO_STATUS2 = 0x3B,
			TIMESTAMP0 = 0x40,
			TIMESTAMP1 = 0x41,
			TIMESTAMP2 = 0x42,
			TIMESTAMP3 = 0x43,
			TAP_CFG0 = 0x56,
			TAP_CFG1 = 0x57,
			TAP_CFG2 = 0x58,
			TAP_THS_6D = 0x59,
			INT_DUR2 = 0x5A,
			WAKE_UP_THS = 0x5B,
			WAKE_UP_DUR = 0x5C,
			FREE_FALL = 0x5D,
			MD1_CFG = 0x5E,
			MD2_CFG = 0x5F,
			I3C_BUS_AVB = 0x62,
			INTERNAL_FREQ_FINE = 0x63,
			INT_OIS = 0x6F,
			CTRL1_OIS = 0x70,
			CTRL2_OIS = 0x71,
			CTRL3_OIS = 0x72,
			X_OFS_USR = 0x73,
			Y_OFS_USR = 0x74,
			Z_OFS_USR = 0x75,
			FIFO_DATA_OUT_TAG = 0x78,
			FIFO_DATA_OUT_X_L = 0x79,
			FIFO_DATA_OUT_X_H = 0x7A,
			FIFO_DATA_OUT_Y_L = 0x7B,
			FIFO_DATA_OUT_Y_H = 0x7C,
			FIFO_DATA_OUT_Z_L = 0x7D,
			FIFO_DATA_OUT_Z_H = 0x7E
		};

		enum class RegisterEmbFunc : uint8_t {
			PAGE_SEL = 0x02,
			EMB_FUNC_EN_A = 0x04,
			EMB_FUNC_EN_B = 0x05,
			PAGE_ADDRESS = 0x08,
			PAGE_VALUE = 0x09,
			EMB_FUNC_INT1 = 0x0A,
			FSM_INT1_A = 0x0B,
			FSM_INT1_B = 0x0C,
			EMB_FUNC_INT2 = 0x0E,
			FSM_INT2_A = 0x0F,
			FSM_INT2_B = 0x10,
			EMB_FUNC_STATUS = 0x12,
			FSM_STATUS_A = 0x13,
			FSM_STATUS_B = 0x14,
			PAGE_RW = 0x17,
			EMB_FUNC_FIFO_CFG = 0x44,
			FSM_ENABLE_A = 0x46,
			FSM_ENABLE_B = 0x47,
			FSM_LONG_COUNTER_L = 0x48,
			FSM_LONG_COUNTER_H = 0x49,
			FSM_LONG_COUNTER_CLEAR = 0x4A,
			FSM_OUTS1 = 0x4C,
			FSM_OUTS2 = 0x4D,
			FSM_OUTS3 = 0x4E,
			FSM_OUTS4 = 0x4F,
			FSM_OUTS5 = 0x50,
			FSM_OUTS6 = 0x51,
			FSM_OUTS7 = 0x52,
			FSM_OUTS8 = 0x53,
			FSM_OUTS9 = 0x54,
			FSM_OUTS10 = 0x55,
			FSM_OUTS11 = 0x56,
			FSM_OUTS12 = 0x57,
			FSM_OUTS13 = 0x58,
			FSM_OUTS14 = 0x59,
			FSM_OUTS15 = 0x5A,
			FSM_OUTS16 = 0x5B,
			EMB_FUNC_ODR_CFG_B = 0x5F,
			STEP_COUNTER_L = 0x62,
			STEP_COUNTER_H = 0x63,
			EMB_FUNC_SRC = 0x64,
			EMB_FUNC_INIT_A = 0x66,
			EMB_FUNC_INIT_B = 0x67
		};

		enum class RegisterAdvPG0 : uint8_t {
			MAG_SENSITIVITY_L = 0xBA,
			MAG_SENSITIVITY_H = 0xBB,
			MAG_OFFX_L = 0xC0,
			MAG_OFFX_H = 0xC1,
			MAG_OFFY_L = 0xC2,
			MAG_OFFY_H = 0xC3,
			MAG_OFFZ_L = 0xC4,
			MAG_OFFZ_H = 0xC5,
			MAG_SI_XX_L = 0xC6,
			MAG_SI_XX_H = 0xC7,
			MAG_SI_XY_L = 0xC8,
			MAG_SI_XY_H = 0xC9,
			MAG_SI_XZ_L = 0xCA,
			MAG_SI_XZ_H = 0xCB,
			MAG_SI_YY_L = 0xCC,
			MAG_SI_YY_H = 0xCD,
			MAG_SI_YZ_L = 0xCE,
			MAG_SI_YZ_H = 0xCF,
			MAG_SI_ZZ_L = 0xD0,
			MAG_SI_ZZ_H = 0xD1,
			MAG_CFG_A = 0xD4,
			MAG_CFG_B = 0xD5
		};

		enum class RegisterAdvPG1 : uint8_t {
			FSM_LC_TIMEOUT_L = 0x7A,
			FSM_LC_TIMEOUT_H = 0x7B,
			FSM_PROGRAMS = 0x7C,
			FSM_START_ADD_L = 0x7E,
			FSM_START_ADD_H = 0x7F,
			PEDO_CMD_REG = 0x83,
			PEDO_DEB_STEPS_CONF = 0x84,
			PEDO_SC_DELTAT_L = 0xD0,
			PEDO_SC_DELTAT_H = 0xD1
		};

		enum class RegisterSensHub : uint8_t {
			SENSOR_HUB_1 = 0x02,
			SENSOR_HUB_2 = 0x03,
			SENSOR_HUB_3 = 0x04,
			SENSOR_HUB_4 = 0x05,
			SENSOR_HUB_5 = 0x06,
			SENSOR_HUB_6 = 0x07,
			SENSOR_HUB_7 = 0x08,
			SENSOR_HUB_8 = 0x09,
			SENSOR_HUB_9 = 0x0A,
			SENSOR_HUB_10 = 0x0B,
			SENSOR_HUB_11 = 0x0C,
			SENSOR_HUB_12 = 0x0D,
			SENSOR_HUB_13 = 0x0E,
			SENSOR_HUB_14 = 0x0F,
			SENSOR_HUB_15 = 0x10,
			SENSOR_HUB_16 = 0x11,
			SENSOR_HUB_17 = 0x12,
			SENSOR_HUB_18 = 0x13,
			MASTER_CONFIG = 0x14,
			SLV0_ADD = 0x15,
			SLV0_SUBADD = 0x16,
			SLV0_CONFIG = 0x17,
			SLV1_ADD = 0x18,
			SLV1_SUBADD = 0x19,
			SLV1_CONFIG = 0x1A,
			SLV2_ADD = 0x1B,
			SLV2_SUBADD = 0x1C,
			SLV2_CONFIG = 0x1D,
			SLV3_ADD = 0x1E,
			SLV3_SUBADD = 0x1F,
			SLV3_CONFIG = 0x20,
			DATAWRITE_SLV0 = 0x21,
			STATUS_MASTER = 0x22
		};

		enum class SampleRate : uint8_t {
			Off = 0x00,
			Hz12_5 = 0x01,
			Hz26 = 0x02,
			Hz52 = 0x03,
			Hz104 = 0x04,
			Hz208 = 0x05,
			Hz416 = 0x06,
			Hz833 = 0x07,
			Hz1666 = 0x08,
			Hz3333 = 0x09,
			Hz6666 = 0x0A
		};

		enum class AccelScale : uint8_t {
			G2 = 0x00,		//0.061 mg/LSB
			G16 = 0x01,		//0.488 mg/LSB
			G4 = 0x02,		//0.122 mg/LSB
			G8 = 0x03		//0.244 mg/LSB
		};
		
		enum class GyroScale : uint8_t {
			DPS250 = 0x00,	//8.75 mdps/LSB
			DPS500 = 0x01,	//17.50 mdps/LSB
			DPS1000 = 0x02,	//35 mdps/LSB
			DPS2000 = 0x03	//70 mdps/LSB
		};

		enum class AccelLPF : uint8_t {
			LPF_ODR2 = 0x00,
			LPF_ODR4 = 0x01,
			LPF_ODR10 = 0x02,
			LPF_ODR20 = 0x03,
			LPF_ODR45 = 0x04,
			LPF_ODR100 = 0x05,
			LPF_ODR200 = 0x06,
			LPF_ODR400 = 0x07,
			LPF_ODR800 = 0x08
		};

		enum class GyroLPF : uint8_t {	//12.5Hz	26Hz	52Hz	104Hz	208Hz	416Hz	833Hz	1.67kHz	3.33kHz	6.67kHz	//
			LPF_Off = 0x00,				//----------------------------------------------------------------------------------//
			LPFType_0 = 0x01,			//4.2		8.3		16.6	33.0	67.0	136.6	239.2	304.2	328.5	335.5	//
			LPFType_1 = 0x02,			//4.2		8.3		16.6	33.0	67.0	130.5	192.4	220.7	229.6	232.0	//
			LPFType_2 = 0x03,			//4.2		8.3		16.6	33.0	67.0	120.3	154.2	166.6	170.1	171.1	//
			LPFType_3 = 0x04,			//4.2		8.3		16.6	33.0	67.0	137.1	281.8	453.2	559.2	609.0	//
			LPFType_4 = 0x05,			//4.2		8.3		16.7	33.0	62.4	86.7	96.6	99.6	NA		NA		//
			LPFType_5 = 0x06,			//4.2		8.3		16.8	31.0	43.2	48.0	49.4	49.8	NA		NA		//
			LPFType_6 = 0x07,			//4.1		7.8		13.4	19.0	23.1	24.6	25.0	25.1	NA		NA		//
			LPFType_7 = 0x08			//3.9		6.7		9.7		11.5	12.2	12.4	12.5	12.5	NA		NA		//
		};

		enum class GyroHPF : uint8_t {
			HPF_Off = 0x00,
			HPF_16mHz = 0x01,
			HPF_65mHz = 0x02,
			HPF_260mHz = 0x03,
			HPF_1Hz04 = 0x04
		};

		struct Config {
			AccelScale accelScale;
			GyroScale gyroScale;
			SampleRate accelOdr;
			SampleRate gyroOdr;
		};

		LSM6DSO(SPI& spi) : bus(spi) {};

		Status Init(const Config& config);
		Status Reset();
		Status ReadID(uint8_t& id);
		Status ReadStatus(uint8_t& status);

		Status SetScales(AccelScale accelScale, GyroScale gyroScale);
		Status SetAccelOffsets(float offsetX, float offsetY, float offsetZ);
		Status SetGyroOffsets(float offsetX, float offsetY, float offsetZ);
		Status RunHardwareSelfTest();

		Status RequestData();
		Status GetData(float* accel, float* gyro, float* temp);

	private:
		SPI& bus;
		Config config;

		static constexpr uint16_t transferSize = 32;
		__attribute__((aligned(32))) uint8_t txBuffer[transferSize];
		__attribute__((aligned(32))) uint8_t rxBuffer[transferSize];

		static constexpr float accelSens[] = {
			0.061f,
			0.488f,
			0.122f,
			0.244f
		};

		static constexpr float gyroSens[] = {
			8.75f,
			17.5f,
			35.f,
			70.f
		};

		static constexpr float tempSens = (1.0f/256);

		float accelOffset[3];
		float gyroOffset[3];
		static constexpr float tempOffset = 25.0f;

		Status WriteRegister(Register reg, uint8_t value);
		Status ReadRegister(Register reg, uint8_t& value);
		Status ModifyRegister(Register reg, uint8_t mask, uint8_t value);
		float ParseAxis(uint8_t msb, uint8_t lsb, float scaleFactor, float offset);
};