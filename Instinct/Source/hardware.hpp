#pragma once

#include "dcmi.hpp"
#include "dmaChannel.hpp"
#include "exti.hpp"
#include "gpio.hpp"
#include "hyperbus.hpp"
#include "i2c.hpp"
#include "i3c.hpp"
#include "jpeg.hpp"
#include "pwm.hpp"
#include "sdmmc.hpp"
#include "status.hpp"
#include "spi.hpp"
#include "system.hpp"
#include "timer.hpp"
#include "uart.hpp"
#include "usb.hpp"

#include "hyperFlash.hpp"
#include "hyperRAM.hpp"
#include "sd.hpp"

#include "bmm350.hpp"
#include "bmp581.hpp"
#include "ina700.hpp"
#include "icm45686.hpp"
#include "icp20100.hpp"
#include "lis2mdl.hpp"
#include "lsm6dso.hpp"

#include "ov7670.hpp"
#include "cameraDCMI.hpp"
#include "jpegEncoder.hpp"

#include "logger.hpp"

#include "fx_api.h"

#include "usbClassCDC.hpp"
#include "usbClassUVC.hpp"
#include "usbDevice.hpp"

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

extern GPIO onboardIMUPwEn;
extern GPIO onboardIMUInt;
extern GPIO ext1IMUPwEn;
extern GPIO ext2IMUPwEn;
extern GPIO extIMUHeater;
extern GPIO ext1IMUInt;
extern GPIO ext2IMUInt;

extern GPIO ospiInt;
extern GPIO ospiRst;
extern GPIO hspiRst;

extern GPIO ethRxErr;
extern GPIO ethRst;

extern GPIO sdDet;
extern GPIO sdVioSel;

extern GPIO hdRadioEn;

extern Dcmi dcmi;
extern Jpeg jpeg;

extern HyperRAM::Config extRAMConfig;
extern HyperBus hyperBus1;
extern HyperRAM externalPSRAM;

extern HyperFlash::Config extFlashConfig;
extern HyperBus hyperBus2;
extern HyperFlash externalFlash;

extern I2C i2c1;
extern I2C i2c2;
extern I2C i2c4;

// extern I3C i3c1;
extern I3C i3c2;

extern SD::Config sdConfig;
extern SDMMC sdmmc1;
extern SD sdCard;
extern FX_MEDIA sdMedia;

extern SDMMC sdmmc2;

extern SPI spi1;
extern SPI spi2;
extern SPI spi4;

extern Timer tim2;
extern PWM pwm1Ch1;
extern PWM pwm1Ch2;
extern PWM pwm1Ch3;
extern PWM pwm1Ch4;

extern Timer tim8;
extern PWM pwm2Ch1;
extern PWM pwm2Ch2;
extern PWM pwm2Ch3;
extern PWM pwm2Ch4;

extern UART uart4;
extern UART hdrUART;

extern DMAChannel dcmiDMAChannel;
extern DMAChannel jpegEncInDMAChannel;
extern DMAChannel jpegEncOutDMAChannel;

extern INA700 ina700;

extern LSM6DSO onboardIMU;
extern LIS2MDL onboardMag;
extern ICP20100 onboardBaro;

extern ICM45686 ext2IMU;
extern BMM350 extMag;
extern BMP581 extBaro;

extern OV7670 ov7670;

class CameraDCMI;
extern CameraDCMI cameraSD;

class JPEGEncoder;
extern JPEGEncoder jpegEncoder;

extern USB usbHardware;
extern USBDevice usbDevice;
extern USBClassCDC usbCDC;
extern USBClassUVC usbUVC;

void HardwareInit();