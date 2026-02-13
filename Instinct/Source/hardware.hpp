#pragma once

#include "gpio.hpp"
#include "hyperbus.hpp"
#include "i2c.hpp"
#include "i3c.hpp"
#include "sdmmc.hpp"
#include "status.hpp"
#include "spi.hpp"
#include "system.hpp"
#include "uart.hpp"

#include "hyperFlash.hpp"
#include "hyperRAM.hpp"

#include "bmm350.hpp"
#include "bmp581.hpp"
#include "ina700.hpp"
#include "icm45686.hpp"
#include "lis2mdl.hpp"
#include "lsm6dso.hpp"

#include "logger.hpp"

extern GPIO ledRed;
extern GPIO ledBlue;
extern GPIO ledGreen;
extern GPIO userButton;

extern GPIO camInt0;
extern GPIO camInt1;
extern GPIO camPwdn;

extern GPIO csiRst;
extern GPIO csiPwdn;
extern GPIO csiIO2;
extern GPIO csiIO3;
extern GPIO csiIO4;
extern GPIO csiIO5;

extern GPIO imuIPWEn;
extern GPIO imuIInt;
extern GPIO imuEPWEn0;
extern GPIO imuEPWEn1;
extern GPIO imuEHeatEn;
extern GPIO imuEInt0;
extern GPIO imuEInt1;

extern GPIO ospiInt;
extern GPIO ospiRst;
extern GPIO hspiRst;

extern GPIO ethRxErr;
extern GPIO ethRst;

extern GPIO sdDet;
extern GPIO sdVioSel;

extern GPIO hdRadioEn;

const extern HyperRAM::Config extRAMConfig;
extern HyperBus hyperBus1;
extern HyperRAM externalPSRAM;

const extern HyperFlash::Config extFlashConfig;
extern HyperBus hyperBus2;
extern HyperFlash externalFlash;

extern UART uart4;

extern SDMMC sdmmc1;
extern SDMMC sdmmc2;

extern SPI spi1;
extern SPI spi2;
extern SPI spi4;

extern I2C i2c1;
extern I2C i2c2;
extern I2C i2c4;

extern I3C i3c1;
extern I3C i3c2;

extern INA700 ina700;

extern LIS2MDL lis2mdl;
extern LSM6DSO lsm6dso;

extern ICM45686 icm45686;
extern BMM350 bmm350;
extern BMP581 bmp581;

void HardwareInit();