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

GPIO imuIPWEn(GPIOD, LL_GPIO_PIN_13);
GPIO imuIInt(GPIOG, LL_GPIO_PIN_15);
GPIO imuEPWEn0(GPIOD, LL_GPIO_PIN_10);
GPIO imuEPWEn1(GPIOC, LL_GPIO_PIN_13);
GPIO imuEHeatEn(GPIOE, LL_GPIO_PIN_3);
GPIO imuEInt0(GPIOF, LL_GPIO_PIN_6);
GPIO imuEInt1(GPIOQ, LL_GPIO_PIN_2);

GPIO ospiInt(GPION, LL_GPIO_PIN_7);
GPIO ospiRst(GPION, LL_GPIO_PIN_12);
GPIO hspiRst(GPIOO, LL_GPIO_PIN_1);

GPIO ethRxErr(GPIOG, LL_GPIO_PIN_5);
GPIO ethRst(GPIOH, LL_GPIO_PIN_4);

GPIO sdDet(GPIOQ, LL_GPIO_PIN_6);
GPIO sdVioSel(GPIOQ, LL_GPIO_PIN_4);

GPIO hdRadioEn(GPIOQ, LL_GPIO_PIN_0);

// Default for Rev. A is: Cypress S80KS2564
const HyperRAM::Config extRAMConfig = {	
	.deviceName = "Cypress S80KS2564",
	.expectedID = 0x06,
	.sizeBytes = 32768 * 1024,	// 32 MByte
	.pageSize = 32 * 8 * 4,		// 1 kByte
	.frequencyHz = 100000000,	// 100 MHz
	.initalLatency = 7,			// 7 Cycles
	.fixedLatency = false,
	.rwRecoveryTime = 7,		// 7 Cycles
	.refreshRateUs = 4,			// 4us or 4000ns
	.writeZeroLatency = false
};
HyperBus hyperBus1 = HyperBus(XSPI1);
HyperRAM externalPSRAM = HyperRAM(hyperBus1);

// Default for Rev. A is: Cypress S26HS512T with Uniform Sector Flash Configuration
const HyperFlash::Config extFlashConfig = {	
	.deviceName = "Cypress S26HS512T",
	.expectedID = 0x0034,
	.expectedDeviceID = 0x0090,
	.sizeBytes = 256 * 256 * 1024,	// 64 MByte
	.sectorSize = 256 * 1024,		// 256 kByte
	.pageSize = 256,				// 512 bytes or 256 bytes (default: 256)
	.frequencyHz = 100000000,		// 100 MHz
	.initialLatency = 16,			// 16 Cycles
	.fixedLatency = false,
	.rwRecoveryTime = 0
};
HyperBus hyperBus2 = HyperBus(XSPI2);
HyperFlash externalFlash = HyperFlash(hyperBus2);

UART uart4(UART4);
extern "C" void UART4_IRQHandler(void) { uart4.InterruptHandler(); }

I2C i2c1(I2C1);
extern "C" void I2C1_EV_IRQHandler(void) { i2c1.InterruptHandler(); }
I2C i2c2(I2C2);
extern "C" void I2C2_EV_IRQHandler(void) { i2c2.InterruptHandler(); }
I2C i2c4(I2C4);
extern "C" void I2C4_EV_IRQHandler(void) { i2c4.InterruptHandler(); }

I3C i3c1(I3C1);
extern "C" void I3C1_EV_IRQHandler(void) { i3c1.InterruptHandler(); }
I3C i3c2(I3C2);
extern "C" void I3C2_EV_IRQHandler(void) { i3c2.InterruptHandler(); }

SDMMC sdmmc1(SDMMC1);
extern "C" void SDMMC1_IRQHandler(void) { sdmmc1.InterruptHandler(); }
SDMMC sdmmc2(SDMMC2);
extern "C" void SDMMC2_IRQHandler(void) { sdmmc2.InterruptHandler(); }

SPI spi1(SPI1);
extern "C" void SPI1_IRQHandler(void) { spi1.InterruptHandler(); }
SPI spi2(SPI2);
extern "C" void SPI2_IRQHandler(void) { spi2.InterruptHandler(); }
SPI spi4(SPI4);
extern "C" void SPI4_IRQHandler(void) { spi4.InterruptHandler(); }

INA700 ina700(i2c2, 0x44);
LIS2MDL lis2mdl(i2c1, 0x1E);
LSM6DSO lsm6dso(spi1);

ICM45686 icm45686(spi2);
BMM350 bmm350(i2c4, 0x14);
BMP581 bmp581(i2c4, 0x47);

void HardwareInit() {
	//Initialize Physical Layer

	//Initialize general purpose GPIOs
	BoardGPIOInit();
	//Initialize peripheral specific IO pins
	//I2C peripherals
	BoardI2C1Init();
	BoardI2C2Init();
	// BoardI2C3Init();
	BoardI2C4Init();
	//I3C peripherals
	BoardI3C1Init();
	BoardI3C2Init();
	//PWM peripherals
	// BoardPWM2Init();
	// BoardPWM8Init();
	//SDMMC peripherals
	BoardSDMMC1Init();
	// BoardSDMMC2Init();
	//SPI peripherals
	BoardSPI1Init();
	BoardSPI2Init();
	BoardSPI4Init();
	// BoardSPI5Init();
	//UART peripherals
	// BoardUART3Init();
	BoardUART4Init();
	// BoardUART6Init();
	// BoardUART7Init();
	// BoardUART8Init();
	//XPSI/HyperBus peripherals
	BoardXSPI1Init();
	BoardXSPI2Init();
	LOG_INFO("Board IO Init OK.");

	//Initialize GPIOs
	//Outputs
	ledRed.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	ledBlue.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	ledGreen.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	imuEHeatEn.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::OpenDrain, .pull = GPIO::Pull::NoPull});
	sdVioSel.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});

	//Inputs
	userButton.Init({.mode = GPIO::Mode::Input, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::PullUp});
	sdDet.Init({.mode = GPIO::Mode::Input, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::PullUp});

	ledRed.Write(1);
	ledGreen.Write(1);
	ledBlue.Write(1);
	imuEHeatEn.Write(1);
	sdVioSel.Write(0);		//SD VIO Selection: 0 -> 3V3, 1 -> 1V8
	LOG_INFO("GPIO Init OK.");

	//Initialize UARTs
	uart4.Init({.baudrate = 115200, .dataBits = UART::DataBits::DataBits_8, .stopBits = UART::StopBits::StopBits_1, .parity = UART::Parity::None, .hwFlowControl = false});
	LOG_INFO("UARTs Init OK.");

	//Initialize I2Cs
	i2c1.Init({.mode = I2C::Mode::Fast});
	i2c2.Init({.mode = I2C::Mode::Fast});
	i2c4.Init({.mode = I2C::Mode::Fast});
	LOG_INFO("I2Cs Init OK.");

	//Initialize I3Cs
	i3c1.Init({.mode = I3C::Mode::Mixed_Fast});
	i3c2.Init({.mode = I3C::Mode::Mixed_Fast});
	LOG_INFO("I3Cs Init OK.");

	//Initialize SPIs
	spi1.Init({.baudrate = 1000000, .polarity = SPI::ClockPolarity::High, .phase = SPI::ClockPhase::SecondEdge, .bitOrder = SPI::BitOrder::MSBFirst});
	spi2.Init({.baudrate = 1000000, .polarity = SPI::ClockPolarity::High, .phase = SPI::ClockPhase::SecondEdge, .bitOrder = SPI::BitOrder::MSBFirst});
	LOG_INFO("SPIs Init OK.");
}