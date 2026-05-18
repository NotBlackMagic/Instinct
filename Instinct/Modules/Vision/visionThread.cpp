#include "visionThread.hpp"

TX_THREAD VisionThread::threadPtr;
uint8_t VisionThread::threadStack[8196];

static VisionFrame rawFrameBuffer;
static VisionFrame processedFrameBuffer;
static VisionFrame jpegFrameBuffer[2];

// Wrapper for Manual Exposure
void VisionThread::ExposureTimeControl(int16_t value) { cameraSD.GetSensor().SetManualExposure(static_cast<uint16_t>(value)); }

// Wrappers for camera controls
void VisionThread::BrightnessControl(int16_t value) { cameraSD.GetSensor().SetBrightness(value); }

// Wrapper for Auto White Balance (Boolean: 1 = Auto, 0 = Manual)
void VisionThread::AutoWhiteBalanceControl(int16_t value) { cameraSD.GetSensor().SetAutoWhiteBalance(value != 0); }

// Wrapper for Manual White Balance (Temperature in Kelvin)
void VisionThread::WhiteBalanceTemperatureControl(int16_t value) {
	// Standard UVC range is often ~2800K (warm/red) to ~6500K (cool/blue). OV7670 gains are 0x00 to 0xFF (baseline 0x40).
	if(value < 2800) {
		value = 2800;
	}
	if(value > 6500) {
		value = 6500;
	}
	// Simple linear interpolation:
	// 2800K: More red, less blue
	// 6500K: Less red, more Blue
	// 4650K: Equilibrium/balanced
	uint8_t redGain = 0x60 - ((value - 2800) * 0x30) / 3700;
	uint8_t blueGain = 0x30 + ((value - 2800) * 0x30) / 3700;
	cameraSD.GetSensor().SetWhiteBalance(redGain, blueGain);
}

// Register all camera controls to be exposed over UVC
USBClassUVC::UVCControlState VisionThread::puControlRegistry[] = {
	// Selector | Len | Min | Max | Res | Def | Cur | Tgt | Dirty | Callback
	// Brightness
	{ USBClassUVC::PUSelector::Brightness, 2, -127, 127, 1, 0, 0, 0, false, BrightnessControl },
	// Auto White Balance (1 Byte, Boolean)
	{ USBClassUVC::PUSelector::WhiteBalanceTempAuto, 1, 0, 1, 1, 1, 1, 1, false, AutoWhiteBalanceControl },
	// Manual White Balance Temperature (2 Bytes, Kelvin)
	{ USBClassUVC::PUSelector::WhiteBalanceTemp, 2, 2800, 6500, 10, 4600, 4600, 4600, false, WhiteBalanceTemperatureControl }
};
const uint8_t VisionThread::numPUControls = 3;

USBClassUVC::UVCControlState VisionThread::itControlRegistry[] = {
	// Selector | Len | Min | Max | Res | Def | Cur | Tgt | Dirty | Callback
	{ USBClassUVC::ITSelector::ExposureTimeAbsolute, 4, 1, 1000, 1, 100, 100, 100, false, ExposureTimeControl}
};
const uint8_t VisionThread::numITControls = 1; 

void VisionThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr,	const_cast<char*>("Vision"),
													VisionThread::Run, 0,
													threadStack, sizeof(threadStack),
													5, 0,
													TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Vision Thread Create Failed.");
	}
}

void VisionThread::Run(ULONG input) {
	(void)input;
	
	LOG_INFO("Vision Thread Initialized.");

	// Initialize
	// SD-CAM Initialization
	CameraDCMI::Config configSD;
	configSD.width = 640;
	configSD.height = 480;
	configSD.fps = 30;
	configSD.format = PixelFormat::YUV422_YVYU;
	Status status = cameraSD.Init(configSD);
	if(status != Status::Ok) {
		LOG_ERR("SD-CAM Init Failed.");
		while(1) {
			tx_thread_sleep(1000);
		}
	}
	LOG_INFO("SD-CAM Initialized.");
	tx_thread_sleep(1000);

	// cameraSD.GetSensor().SetManualExposure(0);
	cameraSD.GetSensor().SetMaxGain(OV7670::GainCeiling::x4);
	// cameraSD.GetSensor().SetTestPattern(true);

	// HD-CAM Initialization
	CameraMIPI::Config configHD;
	configHD.width = 640;
	configHD.height = 480;
	configHD.fps = 30;
	configHD.format = PixelFormat::YUV422_YVYU;
	status = cameraHD.Init(configHD);
	if(status != Status::Ok) {
		LOG_ERR("HD-CAM Init Failed.");
		while(1) {
			tx_thread_sleep(1000);
		}
	}
	LOG_INFO("HD-CAM Initialized.");
	tx_thread_sleep(1000);

	// cameraHD.GetSensor().SetTestPattern(true);

	status = jpegEncoder.Init();
	if(status != Status::Ok) {
		LOG_ERR("JPEG Encoder Init Failed.");
		while(1) {
			tx_thread_sleep(1000);
		}
	}
	// LOG_INFO("JPEG Encoder Initialized.");

	// Setup frame buffer
	rawFrameBuffer.startAddress = (uint8_t*)hyperBus1.GetBaseAddr();	// (uint8_t*)0x34200000
	rawFrameBuffer.width = 640;
	rawFrameBuffer.height = 480;
	rawFrameBuffer.payloadSize = 640 * 480 * 2;
	rawFrameBuffer.allocatedSize = 640 * 480 * 2;
	rawFrameBuffer.format = PixelFormat::YUV422_YVYU;

	processedFrameBuffer.startAddress = (uint8_t*)(hyperBus1.GetBaseAddr() + 0x450000); // Offset past raw buffer
	processedFrameBuffer.width = 640;
	processedFrameBuffer.height = 480;
	processedFrameBuffer.payloadSize = 640 * 480 * 2;
	processedFrameBuffer.allocatedSize = 640 * 480 * 2;
	processedFrameBuffer.format = PixelFormat::Unknown;	// Handled by processor

	jpegFrameBuffer[0].startAddress = (uint8_t*)(hyperBus1.GetBaseAddr() + 0x900000); // Offset past raw buffer
	jpegFrameBuffer[0].width = 640;
	jpegFrameBuffer[0].height = 480;
	jpegFrameBuffer[0].allocatedSize = 640 * 480 * 2;	// 200 KB worst-case
	jpegFrameBuffer[0].format = PixelFormat::Unknown;	// Handled by codec
	jpegFrameBuffer[0].codec = VisionCodec::Jpeg;

	jpegFrameBuffer[1].startAddress = (uint8_t*)(hyperBus1.GetBaseAddr() + 0xA00000); // Offset past raw buffer
	jpegFrameBuffer[1].width = 640;
	jpegFrameBuffer[1].height = 480;
	jpegFrameBuffer[1].allocatedSize = 640 * 480 * 2;	// 200 KB worst-case
	jpegFrameBuffer[1].format = PixelFormat::Unknown;	// Handled by codec
	jpegFrameBuffer[1].codec = VisionCodec::Jpeg;

	uint32_t frameCount = 0;
	uint32_t uvcDropedCount = 0;
	uint64_t timestamp = Time::GetMs();
	uint64_t deltaTime = 0;
	uint32_t timeCap, timeConvMCU, timeEncJpeg;

	uint8_t jpegBufIx = 0;	// The active index for hardware to write to

	bool saveSnapshot = false;
	bool lastBtnState = false;
	while(1) {
		// Process pending UVC camera control updates
		for(uint8_t i = 0; i < numPUControls; i++) {
			if(puControlRegistry[i].toUpdate) {
				// Safely grab the target
				int16_t newTarget = puControlRegistry[i].target;
				puControlRegistry[i].toUpdate = false;
				
				// Execute the I2C callback attached to this control
				if(puControlRegistry[i].ApplyControl != nullptr) {
					puControlRegistry[i].ApplyControl(newTarget);
				}
				
				// Update the cache so GET_CUR is accurate
				puControlRegistry[i].current = newTarget;
				
				LOG_INFO("SD-CAM PU Ctrl %d: %d", puControlRegistry[i].selector, newTarget);
			}
		}

		for(uint8_t i = 0; i < numITControls; i++) {
			if(itControlRegistry[i].toUpdate) {
				// Safely grab the target
				int16_t newTarget = itControlRegistry[i].target;
				itControlRegistry[i].toUpdate = false;
				
				// Execute the I2C callback attached to this control
				if(itControlRegistry[i].ApplyControl != nullptr) {
					itControlRegistry[i].ApplyControl(newTarget);
				}
				
				// Update the cache so GET_CUR is accurate
				itControlRegistry[i].current = newTarget;
				
				LOG_INFO("SD-CAM IT Ctrl %d: %d", itControlRegistry[i].selector, newTarget);
			}
		}

		// Grab references to the active buffer
		VisionFrame& jpegBuf = jpegFrameBuffer[jpegBufIx];

		timestamp = Time::GetUs();
		
		// status = cameraSD.CaptureAsync(rawFrameBuffer);
		status = cameraHD.CaptureAsync(rawFrameBuffer);
		if(status == Status::Ok) {
			// status = cameraSD.CaptureWait(1000);	// Capture takes about 43ms
			status = cameraHD.CaptureWait(1000);

			rawFrameBuffer.timestampUs = Time::GetUs();

			timeCap = Time::GetUs() - timestamp;
			timestamp = Time::GetUs();

			// status = PatternGenerator::Checkerboard(rawFrameBuffer);
			if(status == Status::Ok) {
				// Pass through image processor to transfor to MCU blocks
				status = ImageProcessor::ConvertToMCU(rawFrameBuffer, processedFrameBuffer);	// Conversion takes about 85ms
				timeConvMCU = Time::GetUs() - timestamp;
				timestamp = Time::GetUs();

				// Compress Raw Frame to JPEG
				status = jpegEncoder.EncodeAsync(processedFrameBuffer, jpegBuf, 80);
				if(status == Status::Ok) {
					status = jpegEncoder.EncodeWait(1000);	// Encoding takes about 6ms
					jpegBuf.timestampUs = Time::GetUs();

					timeEncJpeg = Time::GetUs() - timestamp;
					timestamp = Time::GetUs();

					if(status == Status::Ok) {
						// USB UVC Stuff
						status = usbUVC.SubmitFrame(&jpegBuf);
						if(status == Status::Ok) {
							// tx_thread_sleep(50);
							jpegBufIx = 1 - jpegBufIx;	// Ping-pong buffer indexing
						}
						else {
							uvcDropedCount += 1;
							// LOG_WARN("UVC Frame Drop.");
						}

						if(saveSnapshot == true) {
							if(StorageThread::IsReady() == true) {
								// Dynamic file name
								static uint32_t snapCounter = 0;
								char fileName[16];
								snprintf(fileName, sizeof(fileName), "snap%04lu.bin", snapCounter++);
								// ImageWriter::SaveBMP(rawFrameBuffer, *StorageThread::GetMedia(), fileName);
								ImageWriter::SaveBinary(rawFrameBuffer, *StorageThread::GetMedia(), fileName);
								snprintf(fileName, sizeof(fileName), "snap%04lu.jpeg", snapCounter++);
								ImageWriter::SaveBinary(jpegBuf, *StorageThread::GetMedia(), fileName);
							}
							saveSnapshot = false;
						}

						frameCount += 1;
						if(deltaTime < Time::GetMs()) {
							// Only calculate and report FPS every 1s
							float fps = frameCount / 5.0f;
							// LOG_INFO("SD-CAM: %.1f FPS (UVC Drop %d/%d)", fps, uvcDropedCount, frameCount);
							// LOG_INFO("SD-CAM: DCMI %d us, MCU Blk %d us, JPEG %d us", timeCap, timeConvMCU, timeEncJpeg);	// SD-CAM: Cap 32228 us, To MCU 86573 us, Enc 6525 us
							frameCount = 0;
							uvcDropedCount = 0;
							deltaTime = Time::GetMs() + 5000;
						}
					}
				}
			}
			else {
				// LOG_WARN("SD-CAM Frame Drop.");
			}
		}
		else {
			// LOG_WARN("SD-CAM Frame Drop.");
		}

		uint8_t usrBut = userButton.Read();
		if(lastBtnState == true && usrBut == 0x00) {
			saveSnapshot = true;
			lastBtnState = false;
		}
		else if(userButton.Read() == 0x01) {
			lastBtnState = true;
		}
	}
}