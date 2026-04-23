#include "visionThread.hpp"

TX_THREAD VisionThread::threadPtr;
uint8_t VisionThread::threadStack[4096];

static VisionFrame rawFrameBuffer;
static VisionFrame processedFrameBuffer;
static VisionFrame jpegFrameBuffer;

// Wrappers for camera controls
void VisionThread::BrightnessControl(int16_t value) { cameraSD.GetSensor().SetBrightness(value); };

// Register all camera controls to be exposed over UVC
USBClassUVC::UVCControlState VisionThread::puControlRegistry[] = {
	// Selector | Len | Min | Max | Res | Def | Cur | Tgt | Dirty | Callback
	{ USBClassUVC::PUSelector::Brightness, 2, -127, 127, 1, 0, 0, 0, false, BrightnessControl }
};
const uint8_t VisionThread::numPUControls = 1;

USBClassUVC::UVCControlState VisionThread::itControlRegistry[] = {
	// Selector | Len | Min | Max | Res | Def | Cur | Tgt | Dirty | Callback
	{ USBClassUVC::ITSelector::ExposureTimeAbsolute, 10, 1000, 10, 100, 0, 0, 0, false, nullptr }
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
	LOG_INFO("Vision Thread Initialized.");

	// Initialize
	CameraDCMI::Config config;
	config.width = 640;
	config.height = 480;
	config.fps = 30;
	config.format = PixelFormat::YUV422_YVYU;
	Status status = cameraSD.Init(config);
	if(status != Status::Ok) {
		LOG_ERR("SD-CAM Init Failed.");
		while(1) {
			tx_thread_sleep(1000);
		}
	}
	LOG_INFO("SD-CAM Initialized.");
	tx_thread_sleep(1000);

	status = jpegEncoder.Init();
	if(status != Status::Ok) {
		LOG_ERR("JPEG Encoder Init Failed.");
		while(1) {
			tx_thread_sleep(1000);
		}
	}
	// LOG_INFO("JPEG Encoder Initialized.");

	// Setup frame buffer
	rawFrameBuffer.startAddress = (uint8_t*)hyperBus1.GetBaseAddr();
	rawFrameBuffer.width = 640;
	rawFrameBuffer.height = 480;
	rawFrameBuffer.payloadSize = 640 * 480 * 2;
	rawFrameBuffer.allocatedSize = 640 * 480 * 2;
	rawFrameBuffer.format = PixelFormat::YUV422_YVYU;

	processedFrameBuffer.startAddress = (uint8_t*)(hyperBus1.GetBaseAddr() + 0x100000); // Offset past raw buffer
	processedFrameBuffer.width = 640;
	processedFrameBuffer.height = 480;
	processedFrameBuffer.payloadSize = 640 * 480 * 2;
	processedFrameBuffer.allocatedSize = 640 * 480 * 2;
	processedFrameBuffer.format = PixelFormat::Unknown;	// Handled by processor

	jpegFrameBuffer.startAddress = (uint8_t*)(hyperBus1.GetBaseAddr() + 0x200000); // Offset past raw buffer
	jpegFrameBuffer.width = 640;
	jpegFrameBuffer.height = 480;
	jpegFrameBuffer.allocatedSize = 200 * 1024;		// 200 KB worst-case
	jpegFrameBuffer.format = PixelFormat::Unknown;	// Handled by codec
	jpegFrameBuffer.codec = VisionCodec::Jpeg;

	uint32_t frameCount = 0;
	uint32_t uvcDropedCount = 0;
	uint32_t timestamp = Time::GetMs();
	uint32_t deltaTime = 0;

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

		uint32_t timeCap, timeConvMCU, timeEncJpeg;
		uint64_t timestamp = Time::GetUs();
		status = cameraSD.CaptureAsync(rawFrameBuffer);
		status = Status::Ok;
		if(status == Status::Ok) {
			status = cameraSD.CaptureWait(1000);	// Capture takes about 43ms
			timeCap = Time::GetUs() - timestamp;
			timestamp = Time::GetUs();

			// status = PatternGenerator::ColorBar(rawFrameBuffer);
			if(status == Status::Ok) {
				// Pass through image processor to transfor to MCU blocks
				status = ImageProcessor::ConvertToMCU(rawFrameBuffer, processedFrameBuffer);	// Conversion takes about 85ms
				timeConvMCU = Time::GetUs() - timestamp;
				timestamp = Time::GetUs();

				// Compress Raw Frame to JPEG
				status = jpegEncoder.EncodeAsync(processedFrameBuffer, jpegFrameBuffer, 80);
				if(status == Status::Ok) {
					status = jpegEncoder.EncodeWait(1000);	// Encoding takes about 6ms
					timeEncJpeg = Time::GetUs() - timestamp;
					timestamp = Time::GetUs();

					// LOG_INFO("SD-CAM: Cap %d us, To MCU %d us, Enc %d us", timeCap, timeConvMCU, timeEncJpeg);	// SD-CAM: Cap 42998 us, To MCU 84620 us, Enc 5596 us

					if(status == Status::Ok) {
						// USB UVC Stuff
						status = usbUVC.SubmitFrame(&rawFrameBuffer);
						if(status == Status::Ok) {
							tx_thread_sleep(50);
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
								ImageWriter::SaveBinary(jpegFrameBuffer, *StorageThread::GetMedia(), fileName);
							}
							saveSnapshot = false;
						}

						frameCount += 1;
						deltaTime = Time::GetMs() - timestamp;
						if(deltaTime > 1000) {
							// Only calculate and report FPS every 1s
							// LOG_INFO("SD-CAM: %d FPS (UVC Dropped %d)", frameCount, uvcDropedCount);
							frameCount = 0;
							uvcDropedCount = 0;
							timestamp = Time::GetMs();
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