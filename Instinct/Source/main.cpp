#include <cstring>
#include <stdio.h>

#include "actuatorThread.hpp"
#include "auxiliaryThread.hpp"
#include "commanderThread.hpp"
#include "controllerThread.hpp"
#include "estimatorThread.hpp"
#include "inertialThread.hpp"
#include "loggerThread.hpp"
#include "monitorThread.hpp"
#include "radioThread.hpp"
#include "sensorHubThread.hpp"
#include "storageThread.hpp"
#include "mavlinkThread.hpp"
#include "visionThread.hpp"

#include "blackboxConfig.hpp"
#include "mavlinkConfig.hpp"

#include "console.hpp"
#include "hardware.hpp"
#include "pubSub.hpp"

#include "tx_api.h"

#include "zenoh-pico.h"

#define THREADX_BUFFER_POOL_SIZE				16384
alignas(32) static UCHAR tx_byte_pool_buffer[THREADX_BUFFER_POOL_SIZE];
static TX_BYTE_POOL threadBytePool;
static TX_THREAD testThread;

extern void InitZenohSerialTransport(z_owned_session_t* session);

extern "C" {
	// This gives the Zenoh C-library access to your ThreadX byte pool
	TX_BYTE_POOL* pthreadx_byte_pool = &threadBytePool;
}

void TestThread(ULONG thread_input) {
	(void)thread_input;

	Status status = Status::Ok;
	// ZENOH-PICO STUFF!!!!!
	// Allocate the config struct and pass its pointer to be initialized

	// z_owned_config_t config;
	// z_result_t cfg_status = z_config_default(&config);
	// if(cfg_status == 0) {
	// 	// Tell Zenoh to establish a connection using the Serial subsystem.
	// 	// This command natively triggers our _z_open_serial_from_dev function.
	// 	zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, "serial/any#baudrate=115200");

	// 	z_owned_session_t session;
	// 	z_result_t open_status = z_open(&session, z_move(config), nullptr);

	// 	if(open_status == 0) {
	// 		LOG_INFO("Zenoh Session Opened on UART7!");
	// 	}
	// 	else {
	// 		LOG_ERR("Zenoh Session Open Failed!");
	// 	}
	// }
	// else {
	// 	LOG_ERR("Zenoh Config Init Failed!");
	// }

	// while(1) {
	// 	ledBlue.Toggle();
	// 	tx_thread_sleep(100);
	// }

	// USB-CDC STUFF!!!!!
	// uint8_t rxBuffer[64];
	// while(1) {
	// 	// Check for incoming data
	// 	uint32_t bytesAvailable = usbCDC.Available();
	// 	if(bytesAvailable > 0) {
	// 		// Read data out of the ring buffer
	// 		uint32_t bytesToRead = (bytesAvailable > sizeof(rxBuffer)) ? sizeof(rxBuffer) : bytesAvailable;
	// 		uint32_t bytesRead = usbCDC.Read(rxBuffer, bytesToRead);

	// 		// Echo the received data back to the host
	// 		if(bytesRead > 0) {
	// 			usbCDC.Write(rxBuffer, bytesRead);
	// 		}
	// 	}

	// 	// Sleep for 10 ticks (~10ms) to keep the echo responsive without pegging the CPU
	// 	tx_thread_sleep(10);
	// }

	while(1) {
		ledBlue.Toggle();
		tx_thread_sleep(100);
	}
}

void tx_application_define(void *first_unused_memory) {
	(void)first_unused_memory;
	
	uint32_t status = TX_SUCCESS;
	char *pointer;

	Logger::Instance().Init();
	Logger::Instance().RegisterConsole(&debugUART);

	LOG_INFO("--------------------------------");
	LOG_INFO("System Booting...");
	LOG_INFO("Logger Initialized.");

	// Start Hardware stuff here, uses RTOS objects
	HardwareInit();
	LOG_INFO("Peripherals Initialized.");
	ledGreen.Write(0);

	// Configure and initialize USB-HS peripheral
	USBDevice::Config usbCfg;
	usbCfg.vid = 0x1234;					// Dummy VID for testing
	usbCfg.pid = 0x5678;					// Dummy PID for testing
	usbCfg.version = 0x0100;				// v1.00
	usbCfg.manufacturer = "PlumaLabs";
	usbCfg.product = "PlumaN6 HD";
	usbCfg.serialNumber = "00000001";
	usbCfg.maxPower = 50;					// 100mA (value * 2mA)
	usbCfg.selfPowered = false;

	// Define UVC Capabilties/formats
	static const FrameFormat formats[] = {
		// {
		// 	.width = 640,
		// 	.height = 480,
		// 	.frameInterval = 2000000, // 5 FPS in 100ns units
		// 	.format = PixelFormat::YUV422_YVYU,
		// 	.codec = VisionCodec::None
		// }
		{
			.width = 640,
			.height = 480,
			.frameInterval = 2000000, // 5 FPS in 100ns units
			.format = PixelFormat::Unknown,
			.codec = VisionCodec::H264
		}
	};

	// Register UVC Controls (camera controls over the UVC protocol)
	USBClassUVC::ControlConfig uvcCtrlConfig;
	uvcCtrlConfig.puControls = 0;	//USBClassUVC::PUControl::Brightness | USBClassUVC::PUControl::WhiteBalanceTempAuto | USBClassUVC::PUControl::WhiteBalanceTemp;	// Add brightness, more can be added by ORing
	uvcCtrlConfig.itControls = 0;	//USBClassUVC::ITControl::ExposureTimeAbsolute;
	uvcCtrlConfig.puControlRegistry = VisionThread::puControlRegistry;
	uvcCtrlConfig.numPUControls = VisionThread::numPUControls;
	uvcCtrlConfig.itControlRegistry = VisionThread::itControlRegistry;
	uvcCtrlConfig.numITControls = VisionThread::numITControls;
	usbUVC.SetControls(uvcCtrlConfig);

	// Initialize and Connect
	if(usbDevice.Init(usbCfg) == Status::Ok) {
		LOG_INFO("USB Init OK");

		// Register the CDC Class before starting the bus
		if(usbDevice.RegisterClass(&usbCDC) == Status::Ok) {
			LOG_INFO("USB CDC Registered OK");
		}
		else {
			LOG_INFO("USB CDC Registration Failed!");
		}

		// Register the UVC Class
		usbUVC.RegisterFormats(formats, 1);
		if(usbDevice.RegisterClass(&usbUVC) == Status::Ok) {
			LOG_INFO("USB UVC Registered OK");
		}
		else {
			LOG_INFO("USB UVC Registration Failed!");
		}

		if(usbDevice.Start() == Status::Ok) {
			LOG_INFO("USB Start OK");
		}
		else {
			LOG_INFO("USB Start Failed!");
		}
	}
	else {
		LOG_INFO("USB Init Failed!");
	}

	// Start system threads
	Console::Init(&debugUART);

	// Start application threads
	MonitorThread::Init();
	StorageThread::Init();
	VisionThread::Init();
	LoggerThread::Init();
	// InertialThread::Init();
	// AuxiliaryThread::Init();
	// RadioThread::Init();
	// MavlinkThread::Init();
	// SensorHubThread::Init();
	// EstimatorThread::Init();
	// ControllerThread::Init();
	// CommanderThread::Init();
	// ActuatorThread::Init();

	// Configure blackbox (data logging to SD card) logging profile
	// BlackboxConfig::ApplyTuningProfile();
	// Configure Mavlink stream (telemetry) rates
	// MavlinkConfig::ApplyStandardProfile();

	// Create a byte memory pool from which to allocate the thread stacks
	status = tx_byte_pool_create(&threadBytePool, const_cast<char*>("Static Thread Byte Pool"), tx_byte_pool_buffer, THREADX_BUFFER_POOL_SIZE);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Create Byte Pool Failed.");
	}

	// Create the TestThread
	// Allocate the stack
	// status = tx_byte_allocate(&threadBytePool, (VOID**) &pointer, 8192, TX_NO_WAIT);
	// if(status != TX_SUCCESS) {
	// 	LOG_ERR("ThreadX Stack 0 Allocate Failed.");
	// }
	// Create thread
	// status = tx_thread_create(&testThread, const_cast<char*>("Test Thread"),
	// 										TestThread, 0,
	// 										pointer, 8192,
	// 										0, 0,
	// 										TX_NO_TIME_SLICE, TX_AUTO_START);
	// if(status != TX_SUCCESS) {
	// 	LOG_ERR("ThreadX Test Thread Create Failed.");
	// }
}

int main(void) {
	// Enable debugger in flash run mode
	System::EnableDebug();
	
	// Debug Blue LED to see Application loaded
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_ALL);
	ledBlue.Init({.mode = GPIO::Mode::Output, .type = GPIO::Output::PushPull, .pull = GPIO::Pull::NoPull});
	ledBlue.Write(0);
	Time::DelayNOP(100000);
	ledBlue.Write(1);
	Time::DelayNOP(100000);
	ledBlue.Write(0);
	Time::DelayNOP(100000);
	ledBlue.Write(1);
	Time::DelayNOP(100000);
	ledBlue.Write(0);
	Time::DelayNOP(100000);

	// MCU Configuration
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	// HAL_Init();
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);
	System::EnableCache();

	SystemCoreClockUpdate();
	// For when running in Dev mode from internal SRAM use:
	System::InitClock();
	// For when running in XIP mode from external HyperFlash/XSPI2 use:
	// Note: This becase clock must be initiazlied in the Boot/FSBL (calling System::InitClock() there) to have the HyperFlash/XSPI2 already correcly running and can't be changed!
	// System::InitFWClock();

	System::EnableAXISRAM();
	System::EnableVENCSRAM();
	Time::Init();

	ledBlue.Write(1);

	System::InitSysTick();
	tx_kernel_enter();

	// We should never get here as control is now taken by the scheduler
	while (1) {
		ledRed.Toggle();
		Time::Delay(200);
	}
}

/// @brief	This function is executed in case of error occurrence.
/// @retval	None
void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}
