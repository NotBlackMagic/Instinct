// #include "main.h"

#include <cstring>
#include <stdio.h>

#include "i3c.hpp"
#include "status.hpp"

#include "stm32n6xx_ll_dma.h"

#include "auxiliaryThread.hpp"
#include "estimatorThread.hpp"
#include "inertialThread.hpp"
#include "loggerThread.hpp"
#include "monitorThread.hpp"
#include "sensorHubThread.hpp"
#include "storageThread.hpp"
#include "visionThread.hpp"

#include "usbClassCDC.hpp"
#include "usbClassUVC.hpp"
#include "usbDevice.hpp"

#include "console.hpp"
#include "hardware.hpp"
#include "pubSub.hpp"
#include "system.hpp"

#include "logger.hpp"

#include "tx_api.h"

#define THREADX_BUFFER_POOL_SIZE				12288
alignas(32) static UCHAR tx_byte_pool_buffer[THREADX_BUFFER_POOL_SIZE];
static TX_BYTE_POOL threadBytePool;
static TX_THREAD testThread;

// extern void InitZenohSerialTransport(z_owned_session_t* session);

// extern "C" {
//     // This gives the Zenoh C-library access to your ThreadX byte pool
//     TX_BYTE_POOL* pthreadx_byte_pool = &threadBytePool;
// }

const uint32_t buffLen = 32 * 1024;
alignas(32) uint8_t dataW[buffLen];
alignas(32) uint8_t dataR[buffLen];

void TestThread(ULONG thread_input) {
	// ZENOH-PICO STUFF!!!!!
	// 1. Allocate the config struct and pass its pointer to be initialized
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

	uint32_t i;
	volatile uint32_t errCnt = 0;
	volatile uint64_t timestamp = Time::GetUs();
	volatile uint64_t deltaTime = Time::GetUs() - timestamp;
	float speed = 0;
	uint8_t repeats = 1;

	// Init External PSRAM
	if(externalPSRAM.Init(extRAMConfig) == Status::Ok) {
		LOG_INFO("HyperRAM Init OK: %s.", extRAMConfig.deviceName);
	}
	else {
		LOG_ERR("HyperRAM Init Failed!");
		while(1);
	}

	// Init External Flash
	if(externalFlash.Init(extFlashConfig) == Status::Ok) {
		LOG_INFO("HyperFlash Init OK: %s.", extFlashConfig.deviceName);
	}
	else {
		LOG_ERR("HyperFlash Init Failed!");
		while(1);
	}

	// RAM Test
	for(i = 0; i < buffLen; i++) {
		dataW[i] = (uint8_t)i;
	}
	System::CleanCache((uint32_t*)dataW, buffLen);

	timestamp = Time::GetUs();
	externalPSRAM.Write(0, dataW, buffLen);
	deltaTime = Time::GetUs() - timestamp;
	speed = (buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM Write: %d Bytes in %d us (%.2f MByte/s)", buffLen, deltaTime, speed);

	memset(dataR, 0x55, buffLen);
	timestamp = Time::GetUs();
	externalPSRAM.Read(0, dataR, buffLen);
	deltaTime = Time::GetUs() - timestamp;

	errCnt = 0;
	for(i = 0; i < buffLen; i++) {
		if(dataR[i] != dataW[i]) {
			volatile uint8_t readError = dataR[i];
			errCnt += 1;
		}
	}

	speed = (buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM Read: %d Bytes in %d us (%.2f MByte/s), Err %d", buffLen, deltaTime, speed, errCnt);

	// RAM Memory Mapped Test
	externalPSRAM.EnterMemoryMappedMode();
	void *extRAMPtr = (void*)hyperBus1.GetBaseAddr();

	timestamp = Time::GetUs();
	for(i = 0; i < repeats; i++) {
		memcpy(extRAMPtr, dataW, buffLen);
	}
	deltaTime = Time::GetUs() - timestamp;

	speed = (repeats * buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM MM Write: %d Bytes in %d us (%.2f MByte/s)", (repeats * buffLen), deltaTime, speed);
	
	memset(dataR, 0x55, buffLen);
	timestamp = Time::GetUs();
	for(i = 0; i < repeats; i++) {
		memcpy(dataR, extRAMPtr, buffLen);
	}
	deltaTime = Time::GetUs() - timestamp;

	errCnt = 0;
	for(i = 0; i < buffLen; i++) {
		if(dataR[i] != dataW[i]) {
			volatile uint8_t readError = dataR[i];
			errCnt += 1;
		}
	}

	speed = (repeats * buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM MM Read: %d Bytes in %d us (%.2f MByte/s), Err %d", (repeats * buffLen), deltaTime, speed, errCnt);

	// HPDMA Test (PSRAM to SRAM)
	memset(dataR, 0x55, buffLen);

	System::CleanCache((uint32_t*)hyperBus1.GetBaseAddr(), buffLen);
	System::CleanCache((uint32_t*)dataR, buffLen);

	LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_HPDMA1);
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RISAF);
	// NVIC_SetPriority(HPDMA1_Channel0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	// NVIC_EnableIRQ(HPDMA1_Channel0_IRQn);

	// RIF configuration (AXISRAM2)
	const uint32_t RIF_CID = 0x0F;	// Allow ALL i.e RW for everyone
	const uint32_t RIF_ATTRIBUTE_SEC = 0x00000001U;
	const uint32_t RIF_CID_NONE = 0x00000000U;
	RISAF3->REG[0].STARTR = 0x0;
	RISAF3->REG[0].ENDR = 0xFFFFFFFFU;		// Full region
	RISAF3->REG[0].CIDCFGR = (RIF_CID | (RIF_CID << RISAF_REGx_CIDCFGR_WRENC0_Pos));
	RISAF3->REG[0].CFGR = (RISAF_REGx_CFGR_BREN | (RIF_ATTRIBUTE_SEC << RISAF_REGx_CFGR_SEC_Pos)
							| (RIF_CID_NONE << RISAF_REGx_CFGR_PRIVC0_Pos));

	// RIF configuration (XSPI1)
	RISAF11->REG[0].STARTR = 0x0;
	RISAF11->REG[0].ENDR = 0xFFFFFFFFU;		// Full region
	RISAF11->REG[0].CIDCFGR = (RIF_CID | (RIF_CID << RISAF_REGx_CIDCFGR_WRENC0_Pos));
	RISAF11->REG[0].CFGR = (RISAF_REGx_CFGR_BREN | (RIF_ATTRIBUTE_SEC << RISAF_REGx_CFGR_SEC_Pos)
							| (RIF_CID_NONE << RISAF_REGx_CFGR_PRIVC0_Pos));

	// Access configuration
	LL_DMA_EnableChannelSecure(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_EnableChannelPrivilege(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_EnableChannelSrcSecure(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_EnableChannelDestSecure(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_SetStaticIsolation(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_CHANNEL_STATIC_CID_2);

	LL_DMA_SetSrcAddress(HPDMA1, LL_DMA_CHANNEL_12, (uint32_t)hyperBus1.GetBaseAddr());
	LL_DMA_SetDestAddress(HPDMA1, LL_DMA_CHANNEL_12, (uint32_t)dataR);

	LL_DMA_SetDataTransferDirection(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_DIRECTION_MEMORY_TO_MEMORY);
	LL_DMA_SetBlkHWRequest(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_HWREQUEST_SINGLEBURST);

	LL_DMA_SetSrcDataWidth(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_SRC_DATAWIDTH_WORD);
	LL_DMA_SetDestDataWidth(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_DEST_DATAWIDTH_WORD);
	LL_DMA_SetDataAlignment(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_DATA_ALIGN_ZEROPADD);

	LL_DMA_SetSrcIncMode(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_SRC_INCREMENT);
	LL_DMA_SetDestIncMode(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_DEST_INCREMENT);

	LL_DMA_SetSrcBurstLength(HPDMA1, LL_DMA_CHANNEL_12, 2);
	LL_DMA_SetDestBurstLength(HPDMA1, LL_DMA_CHANNEL_12, 2);
	LL_DMA_SetChannelPriorityLevel(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_HIGH_PRIORITY);

	LL_DMA_SetBlkDataLength(HPDMA1, LL_DMA_CHANNEL_12, buffLen);

	LL_DMA_SetSrcAllocatedPort(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_SRC_ALLOCATED_PORT0);
	LL_DMA_SetDestAllocatedPort(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_DEST_ALLOCATED_PORT0);
	
	LL_DMA_SetTransferEventMode(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_TCEM_BLK_TRANSFER);
	LL_DMA_SetTransferMode(HPDMA1, LL_DMA_CHANNEL_12, LL_DMA_NORMAL);

	LL_DMA_ClearFlag_TC(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_ClearFlag_HT(HPDMA1, LL_DMA_CHANNEL_12);
	LL_DMA_ClearFlag_DTE(HPDMA1, LL_DMA_CHANNEL_12);

	timestamp = Time::GetUs();
	LL_DMA_EnableChannel(HPDMA1, LL_DMA_CHANNEL_12);
	DMA_Channel_TypeDef* dmaChannel = ((DMA_Channel_TypeDef *)((uint32_t)HPDMA1 + LL_DMA_CH_OFFSET_TAB[LL_DMA_CHANNEL_12]));
	uint32_t dmaStatus = dmaChannel->CSR;
	do {
		dmaStatus = dmaChannel->CSR;
	}
	while((dmaStatus & DMA_CSR_TCF) != DMA_CSR_TCF);
	deltaTime = Time::GetUs() - timestamp;

	System::InvalidateCache((uint32_t*)dataR, buffLen);

	errCnt = 0;
	for(i = 0; i < buffLen; i++) {
		if(dataR[i] != dataW[i]) {
			volatile uint8_t readError = dataR[i];
			errCnt += 1;
		}
	}

	speed = (buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	LOG_INFO("PSRAM DMA Read: %d Bytes in %d us (%.2f MByte/s), Err %d", (buffLen), deltaTime, speed, errCnt);
	
	// // Flash Test
	// uint32_t flashAddr = 0;
	// for(i = 0; i < buffLen; i++) {
	// 	dataW[i] = (uint8_t)i;
	// }
	// externalFlash.SectorErase(flashAddr);

	// timestamp = Time::GetUs();
	// externalFlash.Program(flashAddr, dataW, buffLen);
	// deltaTime = Time::GetUs() - timestamp;
	// speed = (buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	// LOG_INFO("Flash Write: %d Bytes in %d us (%.2f MByte/s)", buffLen, deltaTime, speed);

	// memset(dataR, 0x55, buffLen);
	// timestamp = Time::GetUs();
	// externalFlash.Read(flashAddr, dataR, buffLen);
	// deltaTime = Time::GetUs() - timestamp;

	// errCnt = 0;
	// for(i = 0; i < buffLen; i++) {
	// 	if(dataR[i] != dataW[i]) {
	// 		errCnt += 1;
	// 	}
	// }

	// speed = (buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	// LOG_INFO("Flash Read: %d Bytes in %d us (%.2f MByte/s), Err %d", buffLen, deltaTime, speed, errCnt);

	// // Flash Memory Mapped Test
	// externalFlash.EnterMemoryMappedMode();
	// void *extFlashPtr = (void*)hyperBus2.GetBaseAddr();

	// memset(dataR, 0x55, buffLen);
	// timestamp = Time::GetUs();
	// for(i = 0; i < repeats; i++) {
	// 	memcpy(dataR, extFlashPtr, buffLen);
	// }
	// deltaTime = Time::GetUs() - timestamp;

	// errCnt = 0;
	// for(i = 0; i < buffLen; i++) {
	// 	if(dataR[i] != dataW[i]) {
	// 		errCnt += 1;
	// 	}
	// }

	// speed = (repeats * buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	// LOG_INFO("Flash MM Read: %d Bytes in %d us (%.2f MByte/s), Err %d", (repeats * buffLen), deltaTime, speed, errCnt);

	// // HPDMA Test (HyperFlash to SRAM)
	// memset(dataR, 0x55, buffLen);
	// System::CleanCache((uint32_t*)hyperBus2.GetBaseAddr(), buffLen);
	// System::CleanCache((uint32_t*)dataR, buffLen);

	// // RIF configuration (XSPI2)
	// RISAF12->REG[0].STARTR = 0x0;
	// RISAF12->REG[0].ENDR = 0x00FFFFFFU;		// 256 MByte area
	// RISAF12->REG[0].CIDCFGR = (RIF_CID_2 | (RIF_CID_2 << RISAF_REGx_CIDCFGR_WRENC0_Pos));
	// RISAF12->REG[0].CFGR = (RISAF_REGx_CFGR_BREN | (RIF_ATTRIBUTE_SEC << RISAF_REGx_CFGR_SEC_Pos)
	// 						| (RIF_CID_NONE << RISAF_REGx_CFGR_PRIVC0_Pos));

	// // Re-setup HPDMA
	// LL_DMA_SetDestAddress(HPDMA1, LL_DMA_CHANNEL_12, (uint32_t)dataR);
	// LL_DMA_SetSrcAddress(HPDMA1, LL_DMA_CHANNEL_12, (uint32_t)hyperBus2.GetBaseAddr());
	// LL_DMA_SetBlkDataLength(HPDMA1, LL_DMA_CHANNEL_12, buffLen);

	// LL_DMA_ClearFlag_TC(HPDMA1, LL_DMA_CHANNEL_12);
	// LL_DMA_ClearFlag_HT(HPDMA1, LL_DMA_CHANNEL_12);
	// LL_DMA_ClearFlag_DTE(HPDMA1, LL_DMA_CHANNEL_12);

	// timestamp = Time::GetUs();
	// LL_DMA_EnableChannel(HPDMA1, LL_DMA_CHANNEL_12);
	// dmaChannel = ((DMA_Channel_TypeDef *)((uint32_t)HPDMA1 + LL_DMA_CH_OFFSET_TAB[LL_DMA_CHANNEL_12]));
	// dmaStatus = dmaChannel->CSR;
	// do {
	// 	dmaStatus = dmaChannel->CSR;
	// }
	// while((dmaStatus & DMA_CSR_TCF) != DMA_CSR_TCF);
	// deltaTime = Time::GetUs() - timestamp;

	// System::InvalidateCache((uint32_t*)dataR, buffLen);

	// errCnt = 0;
	// for(i = 0; i < buffLen; i++) {
	// 	if(dataR[i] != dataW[i]) {
	// 		errCnt += 1;
	// 	}
	// }

	// speed = (buffLen) * (1.0f/1024.f * 1.0f/1024.f) * (1.0f/(deltaTime * 0.001f * 0.001f));
	// LOG_INFO("Flash DMA Read: %d Bytes in %d us (%.2f MByte/s), Err %d", (buffLen), deltaTime, speed, errCnt);

	while(1) {
		ledBlue.Toggle();
		tx_thread_sleep(100);
	}
}

void tx_application_define(void *first_unused_memory) {
	uint32_t status = TX_SUCCESS;
	char *pointer;

	Logger::Instance().Init();
	Logger::Instance().RegisterConsole(&uart4);

	LOG_INFO("--------------------------------");
	LOG_INFO("System Booting...");
	LOG_INFO("Logger Initialized.");

	// Start Hardware stuff here, uses RTOS objects
	HardwareInit();
	LOG_INFO("Peripherals Initialized.");

	// Define your test configuration
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
			.codec = VisionCodec::Jpeg
		}
	};

	// Register UVC Controls (camera controls over the UVC protocol)
	USBClassUVC::ControlConfig uvcCtrlConfig;
	uvcCtrlConfig.puControls = USBClassUVC::PUControl::Brightness | USBClassUVC::PUControl::WhiteBalanceTempAuto | USBClassUVC::PUControl::WhiteBalanceTemp;	// Add brightness, more can be added by ORing
	uvcCtrlConfig.itControls = USBClassUVC::ITControl::ExposureTimeAbsolute;
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

	// Start system threads
	Console::Init(&uart4);

	// Start application threads
	MonitorThread::Init();
	StorageThread::Init();
	// VisionThread::Init();
	InertialThread::Init();
	AuxiliaryThread::Init();
	LoggerThread::Init();
	// SensorHubThread::Init();
	// EstimatorThread::Init();

	// Create a byte memory pool from which to allocate the thread stacks
	status = tx_byte_pool_create(&threadBytePool, const_cast<char*>("Static Thread Byte Pool"), tx_byte_pool_buffer, THREADX_BUFFER_POOL_SIZE);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Create Byte Pool Failed.");
	}

	// Create the TestThread
	// Allocate the stack
	status = tx_byte_allocate(&threadBytePool, (VOID**) &pointer, 8192, TX_NO_WAIT);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Stack 0 Allocate Failed.");
	}
	// Create thread
	status = tx_thread_create(&testThread, const_cast<char*>("Test Thread"),
											TestThread, 0,
											pointer, 8192,
											0, 0,
											TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Test Thread Create Failed.");
	}
}

int main(void) {
	// MCU Configuration
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	// HAL_Init();
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);
	System::EnableCache();

	SystemCoreClockUpdate();
	System::InitSysTick();

	System::InitClock();
	System::InitSysTick();
	Time::Init();

	// Enable debugger in flash run mode
	System::EnableDebug();

	tx_kernel_enter();

	// We should never get here as control is now taken by the scheduler
	while (1) {
		ledRed.Toggle();
		Time::Delay(200);
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}
