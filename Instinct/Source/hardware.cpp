#include "hardware.hpp"
#include "../../Board/PlumaN6.hpp"

GPIO ledRed(GPIOB, LL_GPIO_PIN_13);
GPIO ledBlue(GPIOB, LL_GPIO_PIN_2);
GPIO ledGreen(GPIOQ, LL_GPIO_PIN_1);
GPIO userButton(GPIOO, LL_GPIO_PIN_5);

GPIO camInt0(GPIOG, LL_GPIO_PIN_0);
GPIO camInt1(GPIOA, LL_GPIO_PIN_3);
GPIO camPwdn(GPIOE, LL_GPIO_PIN_9);

GPIO csiRst(GPIOA, LL_GPIO_PIN_2);
GPIO csiPwdn(GPIOG, LL_GPIO_PIN_6);
GPIO csiIO2(GPIOG, LL_GPIO_PIN_8);
GPIO csiIO3(GPIOG, LL_GPIO_PIN_9);
GPIO csiIO4(GPIOG, LL_GPIO_PIN_11);
GPIO csiIO5(GPIOG, LL_GPIO_PIN_12);

GPIO onboardIMUPwEn(GPIOD, LL_GPIO_PIN_13);
GPIO onboardIMUInt(GPIOG, LL_GPIO_PIN_15);
GPIO ext1IMUPwEn(GPIOD, LL_GPIO_PIN_10);
GPIO ext2IMUPwEn(GPIOC, LL_GPIO_PIN_13);
GPIO extIMUHeater(GPIOE, LL_GPIO_PIN_3);
GPIO ext1IMUInt(GPIOF, LL_GPIO_PIN_6);
GPIO ext2IMUInt(GPIOQ, LL_GPIO_PIN_2);

GPIO ospiInt(GPION, LL_GPIO_PIN_7);
GPIO ospiRst(GPION, LL_GPIO_PIN_12);
GPIO hspiRst(GPIOO, LL_GPIO_PIN_1);

GPIO ethRxErr(GPIOG, LL_GPIO_PIN_5);
GPIO ethRst(GPIOH, LL_GPIO_PIN_4);

GPIO sdDet(GPIOQ, LL_GPIO_PIN_6);
GPIO sdVioSel(GPIOQ, LL_GPIO_PIN_4);

GPIO hdRadioEn(GPIOQ, LL_GPIO_PIN_0);

Csi csi(CSI);
extern "C" void CSI_IRQHandler(void) { csi.InterruptHandler(); }

Dcmipp dcmipp(DCMIPP);
extern "C" void DCMIPP_IRQHandler(void) { dcmipp.InterruptHandler(); }

Dcmi dcmi(DCMI);
extern "C" void DCMI_PSSI_IRQHandler(void) { dcmi.InterruptHandler(); }

Jpeg jpeg(JPEG);
extern "C" void JPEG_IRQHandler(void) { jpeg.InterruptHandler(); }

// Default for Rev. A is: Cypress S80KS2564
HyperRAM::Config extRAMConfig = {	
	.deviceName = "Cypress S80KS2564",
	.expectedID = 0x06,
	.sizeBytes = 32768 * 1024,	// 32 MByte
	.pageSize = 32 * 8 * 4,		// 1 kByte
	.sourceClockHz = 0,
	.frequencyHz = 200000000,	// 100 MHz
	.initialLatency = 7,		// 7 Cycles
	.fixedLatency = true,
	.rwRecoveryTime = 7,		// 7 Cycles @ 200MHz
	.refreshRateUs = 4,			// 4us or 4000ns
	.writeZeroLatency = false,
	.configReg0 = 0xBF2F,		// CFG0: Set drive strength to 46 Ohm
	.configReg1 = 0				// CFG1: Do not change
};
HyperBus hyperBus1 = HyperBus(XSPI1);
extern "C" void XSPI1_IRQHandler(void) { hyperBus1.InterruptHandler(); }
HyperRAM externalPSRAM = HyperRAM(hyperBus1);

// Default for Rev. A is: Cypress S26HS512T with Uniform Sector Flash Configuration
HyperFlash::Config extFlashConfig = {	
	.deviceName = "Cypress S26HS512T",
	.expectedID = 0x0034,
	.expectedDeviceID = 0x0090,
	.sizeBytes = 256 * 256 * 1024,	// 64 MByte
	.sectorSize = 256 * 1024,		// 256 kByte
	.pageSize = 256,				// 512 bytes or 256 bytes (default: 256)
	.sourceClockHz = 0,
	.frequencyHz = 100000000,		// 100 MHz
	.initialLatency = 16,			// 16 Cycles
	.fixedLatency = false,
	.rwRecoveryTime = 0,
	.configReg0 = 0x8EBF,			// CFG0: Set drive strength to 46 Ohm
	.configReg1 = 0					// CFG1: Do not change
};
HyperBus hyperBus2 = HyperBus(XSPI2);
extern "C" void XSPI2_IRQHandler(void) { hyperBus2.InterruptHandler(); }
HyperFlash externalFlash = HyperFlash(hyperBus2);

// To on-board IMU
I2C i2c1(I2C1);
extern "C" void I2C1_EV_IRQHandler(void) { i2c1.InterruptHandler(); }
// To power header and on-board power monitor IC
I2C i2c2(I2C2);
extern "C" void I2C2_EV_IRQHandler(void) { i2c2.InterruptHandler(); }
// To IMU header
I2C i2c4(I2C4);
extern "C" void I2C4_EV_IRQHandler(void) { i2c4.InterruptHandler(); }

I3C i3c1(I3C1);
extern "C" void I3C1_EV_IRQHandler(void) { i3c1.InterruptHandler(); }
I3C i3c2(I3C2);
extern "C" void I3C2_EV_IRQHandler(void) { i3c2.InterruptHandler(); }

SD::Config sdConfig = {
	.use4BitMode = true,
	.use1V8Level = false,
	.useHighSpeed = true,
	.useUHS = false,
	.vioSelectPin = &sdVioSel
};
SD sdCard = SD(sdmmc1);
SDMMC sdmmc1(SDMMC1);
extern "C" void SDMMC1_IRQHandler(void) { sdmmc1.InterruptHandler(); }
SDMMC sdmmc2(SDMMC2);
extern "C" void SDMMC2_IRQHandler(void) { sdmmc2.InterruptHandler(); }

// To On-Board IMU
SPI spi1(SPI1);
extern "C" void SPI1_IRQHandler(void) { spi1.InterruptHandler(); }
// To IMU header
SPI spi2(SPI2);
extern "C" void SPI2_IRQHandler(void) { spi2.InterruptHandler(); }
// To IMU header
SPI spi4(SPI4);
extern "C" void SPI4_IRQHandler(void) { spi4.InterruptHandler(); }

Timer timer2(TIM2);
PWM pwm1Ch1(timer2, PWM::Channel::Ch1);
PWM pwm1Ch2(timer2, PWM::Channel::Ch2);
PWM pwm1Ch3(timer2, PWM::Channel::Ch3);
PWM pwm1Ch4(timer2, PWM::Channel::Ch4);

Timer timer8(TIM8);
PWM pwm2Ch1(timer8, PWM::Channel::Ch1);
PWM pwm2Ch2(timer8, PWM::Channel::Ch2);
PWM pwm2Ch3(timer8, PWM::Channel::Ch3);
PWM pwm2Ch4(timer8, PWM::Channel::Ch4);

UART uart4(UART4);
extern "C" void UART4_IRQHandler(void) { uart4.InterruptHandler(); }

UART ldrUART(USART6);
extern "C" void USART6_IRQHandler(void) { ldrUART.InterruptHandler(); }

UART hdrUART(UART7);
extern "C" void UART7_IRQHandler(void) { hdrUART.InterruptHandler(); }

USB usbHardware(USB1_OTG_HS);
extern "C" void USB1_OTG_HS_IRQHandler(void) { usbHardware.InterruptHandler(); }
USBDevice usbDevice(usbHardware);
USBClassCDC usbCDC;
USBClassUVC usbUVC;

DMAChannel csiDMAChannel(HPDMA1, LL_DMA_CHANNEL_3);
extern "C" void HPDMA1_Channel3_IRQHandler(void) { csiDMAChannel.InterruptHandler(); }

DMAChannel dcmiDMAChannel(HPDMA1, LL_DMA_CHANNEL_2);
extern "C" void HPDMA1_Channel2_IRQHandler(void) { dcmiDMAChannel.InterruptHandler(); }

DMAChannel jpegEncInDMAChannel(HPDMA1, LL_DMA_CHANNEL_0);
extern "C" void HPDMA1_Channel0_IRQHandler(void) { jpegEncInDMAChannel.InterruptHandler(); }

DMAChannel jpegEncOutDMAChannel(HPDMA1, LL_DMA_CHANNEL_1);
extern "C" void HPDMA1_Channel1_IRQHandler(void) { jpegEncOutDMAChannel.InterruptHandler(); }

// EXTI Interrupt Routing
extern "C" void EXTI0_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_0) == 0x01) {
		EXTIManager::Dispatch(0, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_0);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_0) == 0x01) {
		EXTIManager::Dispatch(0, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_0);
	}
}

extern "C" void EXTI1_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_1) == 0x01) {
		EXTIManager::Dispatch(1, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_1);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_1) == 0x01) {
		EXTIManager::Dispatch(1, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_1);
	}
}

extern "C" extern "C" void EXTI2_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_2) == 0x01) {
		EXTIManager::Dispatch(2, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_2);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_2) == 0x01) {
		EXTIManager::Dispatch(2, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_2);
	}
}

extern "C" void EXTI3_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_3) == 0x01) {
		EXTIManager::Dispatch(3, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_3);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_3) == 0x01) {
		EXTIManager::Dispatch(3, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_3);
	}
}

extern "C" void EXTI4_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_4) == 0x01) {
		EXTIManager::Dispatch(4, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_4);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_4) == 0x01) {
		EXTIManager::Dispatch(4, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_4);
	}
}

extern "C" void EXTI5_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_5) == 0x01) {
		EXTIManager::Dispatch(5, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_5);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_5) == 0x01) {
		EXTIManager::Dispatch(5, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_5);
	}
}

extern "C" void EXTI6_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_6) == 0x01) {
		EXTIManager::Dispatch(6, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_6);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_6) == 0x01) {
		EXTIManager::Dispatch(6, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_6);
	}
}

extern "C" void EXTI7_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_7) == 0x01) {
		EXTIManager::Dispatch(7, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_7);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_7) == 0x01) {
		EXTIManager::Dispatch(7, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_7);
	}
}

extern "C" void EXTI8_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_8) == 0x01) {
		EXTIManager::Dispatch(8, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_8);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_8) == 0x01) {
		EXTIManager::Dispatch(8, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_8);
	}
}

extern "C" void EXTI9_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_9) == 0x01) {
		EXTIManager::Dispatch(9, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_9);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_9) == 0x01) {
		EXTIManager::Dispatch(9, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_9);
	}
}

extern "C" void EXTI10_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_10) == 0x01) {
		EXTIManager::Dispatch(10, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_10);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_10) == 0x01) {
		EXTIManager::Dispatch(10, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_10);
	}
}

extern "C" void EXTI11_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_11) == 0x01) {
		EXTIManager::Dispatch(11, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_11);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_11) == 0x01) {
		EXTIManager::Dispatch(11, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_11);
	}
}

extern "C" void EXTI12_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_12) == 0x01) {
		EXTIManager::Dispatch(12, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_12);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_12) == 0x01) {
		EXTIManager::Dispatch(12, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_12);
	}
}

extern "C" void EXTI13_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_13) == 0x01) {
		EXTIManager::Dispatch(13, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_13);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_13) == 0x01) {
		EXTIManager::Dispatch(13, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_13);
	}
}

extern "C" void EXTI14_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_14) == 0x01) {
		EXTIManager::Dispatch(14, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_14);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_14) == 0x01) {
		EXTIManager::Dispatch(14, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_14);
	}
}

extern "C" void EXTI15_IRQHandler(void) {
	// Check rising edge
	if(LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_15) == 0x01) {
		EXTIManager::Dispatch(15, EXTIManager::Edge::Rising);
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_15);
	}
	// Check falling edge
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_15) == 0x01) {
		EXTIManager::Dispatch(15, EXTIManager::Edge::Falling);
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_15);
	}
}

// On-board sensors
INA700 ina700(i2c2, 0x44);
LSM6DSO onboardIMU(spi1);
LIS2MDL onboardMag(i2c1, 0x1E);
ICP20100 onboardBaro(i2c1, 0x63);

// External IMU sensors
ICM45686 ext2IMU(spi2);
BMM350 extMag(i2c4, 0x14);
BMP581 extBaro(i2c4, 0x47);

OV5645 ov5645(i3c1, 0x3C);
OV7670 ov7670(i3c1, 0x21);
OV9281 ov9281(i3c1, 0x00);

// Radio Links
RCReceiver mainRC(ldrUART);

CameraDCMI cameraSD(dcmi, ov7670, dcmiDMAChannel);
CameraMIPI cameraHD(csi, dcmipp, ov5645, csiDMAChannel);

JPEGEncoder jpegEncoder(jpeg, jpegEncInDMAChannel, jpegEncOutDMAChannel);

void HardwareInit() {
	// Initialize Physical Layer

	// Initialize Cortex-M DWT Cycle Counter
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;	// Enable Trace system
	DWT->CYCCNT = 0;								// Reset the counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;			// Start the counter

	// Initialize general purpose GPIOs
	BoardGPIOInit();
	// Initialize peripheral specific IO pins
	// DCMI peripherals
	BoardDCMIInit();
	// I2C peripherals
	BoardI2C1Init();
	BoardI2C2Init();
	// BoardI2C3Init();
	BoardI2C4Init();
	// I3C peripherals
	BoardI3C1Init();
	BoardI3C2Init();
	// PWM peripherals
	BoardPWM2Init();
	BoardPWM8Init();
	// SDMMC peripherals
	BoardSDMMC1Init();
	// BoardSDMMC2Init();
	// SPI peripherals
	BoardSPI1Init();
	BoardSPI2Init();
	BoardSPI4Init();
	// BoardSPI5Init();
	// UART peripherals
	// BoardUART3Init();
	BoardUART4Init();
	BoardUART6Init();
	BoardUART7Init();
	// BoardUART8Init();
	// XPSI/HyperBus peripherals
	BoardXSPI1Init();
	BoardXSPI2Init();
	LOG_INFO("Board IO Init OK.");

	// Initialize GPIOs
	// Outputs
	ledRed.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	ledBlue.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	ledGreen.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	sdVioSel.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	camPwdn.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	csiPwdn.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	csiRst.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	onboardIMUPwEn.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::OpenDrain, .pull = GPIO::Pull::NoPull});
	ext1IMUPwEn.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::OpenDrain, .pull = GPIO::Pull::NoPull});
	ext2IMUPwEn.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::OpenDrain, .pull = GPIO::Pull::NoPull});
	extIMUHeater.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::OpenDrain, .pull = GPIO::Pull::NoPull});

	// Inputs
	userButton.Init({.mode = GPIO::Mode::Input, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::PullUp});
	sdDet.Init({.mode = GPIO::Mode::Input, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::PullUp});
	onboardIMUInt.Init({.mode = GPIO::Mode::Input, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::PullDown});
	ext1IMUInt.Init({.mode = GPIO::Mode::Input, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::PullDown});
	ext2IMUInt.Init({.mode = GPIO::Mode::Input, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::PullDown});

	ledRed.Write(1);
	ledGreen.Write(1);
	ledBlue.Write(1);
	extIMUHeater.Write(1);
	sdVioSel.Write(0);		//SD VIO Selection: 0 -> 3V3, 1 -> 1V8
	camPwdn.Write(0);
	csiPwdn.Write(1);
	csiRst.Write(0);
	onboardIMUPwEn.Write(0);	// Power down/disable LDO
	ext1IMUPwEn.Write(0);		// Power down/disable LDO
	ext2IMUPwEn.Write(0);		// Power down/disable LDO
	LOG_INFO("GPIO Init OK.");

	// Initialize UARTs
	uart4.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC9), .baudrate = 115200, .dataBits = UART::DataBits::DataBits_8, .stopBits = UART::StopBits::StopBits_1, .parity = UART::Parity::None, .hwFlowControl = false});
	ldrUART.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC9), .baudrate = 115200, .dataBits = UART::DataBits::DataBits_8, .stopBits = UART::StopBits::StopBits_1, .parity = UART::Parity::None, .hwFlowControl = false});
	hdrUART.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC9), .baudrate = 115200, .dataBits = UART::DataBits::DataBits_8, .stopBits = UART::StopBits::StopBits_1, .parity = UART::Parity::None, .hwFlowControl = false});
	LOG_INFO("UARTs Init OK.");

	// Initialize I2Cs
	i2c1.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC10), .mode = I2C::Mode::Fast});
	i2c2.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC10), .mode = I2C::Mode::Fast});
	i2c4.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC10), .mode = I2C::Mode::Fast});
	LOG_INFO("I2Cs Init OK.");

	// Initialize I3Cs
	i3c1.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC10), .mode = I3C::Mode::Mixed_Fast});
	i3c2.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC10), .mode = I3C::Mode::Mixed_Fast});
	LOG_INFO("I3Cs Init OK.");

	// Initialize SPIs
	spi1.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC9), .baudrate = 1000000, .polarity = SPI::ClockPolarity::High, .phase = SPI::ClockPhase::SecondEdge, .bitOrder = SPI::BitOrder::MSBFirst});
	spi2.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC9), .baudrate = 1000000, .polarity = SPI::ClockPolarity::High, .phase = SPI::ClockPhase::SecondEdge, .bitOrder = SPI::BitOrder::MSBFirst});
	LOG_INFO("SPIs Init OK.");

	// Initialize Timers
	timer2.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::AXI), .frequencyHz = 50});
	timer2.Start();
    timer8.Init({.sourceClockHz = System::GetNodeFrequency(System::ClockNode::AXI), .frequencyHz = 50});
	timer8.Start();
	LOG_INFO("TIMs Init OK.");

	// Initialize PWMs
	pwm1Ch1.Init({.polarity = PWM::Polarity::High});
	pwm1Ch2.Init({.polarity = PWM::Polarity::High});
	pwm1Ch3.Init({.polarity = PWM::Polarity::High});
	pwm1Ch4.Init({.polarity = PWM::Polarity::High});
	pwm2Ch1.Init({.polarity = PWM::Polarity::High});
	pwm2Ch2.Init({.polarity = PWM::Polarity::High});
	pwm2Ch3.Init({.polarity = PWM::Polarity::High});
	pwm2Ch4.Init({.polarity = PWM::Polarity::High});
	LOG_INFO("PWMs Init OK.");

	// Configure XSPI clock (needs the System stuff to be initialized!!)
	extRAMConfig.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC3);	// From IC3
	extFlashConfig.sourceClockHz = System::GetNodeFrequency(System::ClockNode::IC3);	// From IC3
}