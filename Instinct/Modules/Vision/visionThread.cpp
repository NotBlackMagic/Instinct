#include "visionThread.hpp"

TX_THREAD VisionThread::threadPtr;
uint8_t VisionThread::threadStack[8196];

static VisionFrame rawFrameBuffer;
static VisionFrame processedFrameBuffer;
static VisionFrame jpegFrameBuffer[2];

// Wrapper for Manual Exposure
void VisionThread::ExposureTimeControl(int16_t value) { cameraHD.GetSensor().SetManualExposure(static_cast<uint16_t>(value)); }

// Wrappers for camera controls
void VisionThread::BrightnessControl(int16_t value) { cameraHD.GetSensor().SetBrightness(value); }

// Wrapper for Auto White Balance (Boolean: 1 = Auto, 0 = Manual)
void VisionThread::AutoWhiteBalanceControl(int16_t value) { cameraHD.GetSensor().SetAutoWhiteBalance(value != 0); }

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
	cameraHD.GetSensor().SetWhiteBalance(redGain, blueGain);
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

static volatile bool uvcStreamRequested = false;
static volatile bool uvcFormatChanged = false;
// USB UVC stream start callback function
void VisionThread::OnUVCStreamState(USBClassUVC::StreamEvent event) {
	if(event == USBClassUVC::StreamEvent::Start) {
		uvcStreamRequested = true;
	}
	else if(event == USBClassUVC::StreamEvent::FormatChange) {
		uvcFormatChanged = true;
	}
}

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
	// USB UVC stuff
	usbUVC.RegisterStreamEventCallback(VisionThread::OnUVCStreamState);

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
	LOG_INFO("JPEG Encoder Initialized.");

	// VENC Initialization in JPEG mode
	Venc::Config vencConfig = {};
	vencConfig.codec = Venc::Codec::H264;
	vencConfig.imageParams.width = 640;
	vencConfig.imageParams.height = 480;
	vencConfig.imageParams.frameRate = 30;
	vencConfig.imageParams.inputFormat = Venc::InputFormat::YUV422InterleavedYUYV;
	// For JPEG Encoding mode
	vencConfig.jpegQuality = 8;
	// H.264 Rate Control (Targeting 2 Mbps)
	vencConfig.rateControl.enablePictureRc = true;
	vencConfig.rateControl.bitPerSecond = 2000000; 
	vencConfig.rateControl.gopLen = 30; // One I-Frame every 30 frames (1 second)
	vencConfig.rateControl.qpMin = 10;
	vencConfig.rateControl.qpMax = 51;
	vencConfig.rateControl.qpHdr = 26;
	// H.264 Coding Control
	vencConfig.codingControl.sliceSize = 0; // 0 = encode whole frame as one slice
	vencConfig.codingControl.enableCabac = true;
	vencConfig.codingControl.enableTransform8x8 = true;
	vencConfig.codingControl.insertIdrHeader = true;
	vencConfig.codingControl.disableDeblockingFilter = false;
	// Ideal pool size can be found: VC8000NanoE Video Encoder Software Integration Guide, Section H264 4.3.1-4.3.4
	vencConfig.poolAddress = (uint8_t*)(hyperBus1.GetBaseAddr() + 0x400000);
	vencConfig.poolSize = 2 * 1024 * 1024;
	status = vencEncoder.Init(vencConfig);
	if(status != Status::Ok) {
		LOG_ERR("VENC H264 Encoder Init Failed.");
		while(1) {
			tx_thread_sleep(1000);
		}
	}
	LOG_INFO("VENC H264 Encoder Initialized.");

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

	jpegFrameBuffer[0].startAddress = (uint8_t*)(hyperBus1.GetBaseAddr() + 0x200000); // Offset past raw buffer
	jpegFrameBuffer[0].width = 640;
	jpegFrameBuffer[0].height = 480;
	jpegFrameBuffer[0].allocatedSize = 640 * 480 * 2;
	jpegFrameBuffer[0].format = PixelFormat::Unknown;	// Handled by codec
	jpegFrameBuffer[0].codec = VisionCodec::H264;

	jpegFrameBuffer[1].startAddress = (uint8_t*)(hyperBus1.GetBaseAddr() + 0x300000); // Offset past raw buffer
	jpegFrameBuffer[1].width = 640;
	jpegFrameBuffer[1].height = 480;
	jpegFrameBuffer[1].allocatedSize = 640 * 480 * 2;
	jpegFrameBuffer[1].format = PixelFormat::Unknown;	// Handled by codec
	jpegFrameBuffer[1].codec = VisionCodec::H264;

	// H.264 Stream Start (SPS/PPS Generation)
	static uint8_t spsPpsStorage[128];          // Dedicated buffer for SPS/PPS headers
	static VisionFrame spsPpsFrame = {};
	VisionFrame& headerBuf = jpegFrameBuffer[0];
	status = vencEncoder.EncodeStart(headerBuf);
	if(status == Status::Ok && headerBuf.payloadSize > 0) {
		// Copy the generated headers into dedicated SPS/PPS buffer
		spsPpsFrame.startAddress = spsPpsStorage;
		spsPpsFrame.allocatedSize = sizeof(spsPpsStorage);
		spsPpsFrame.payloadSize = headerBuf.payloadSize;
		if(headerBuf.payloadSize <= sizeof(spsPpsStorage)) {
			memcpy(spsPpsStorage, headerBuf.startAddress, headerBuf.payloadSize);
		}
		else {
			LOG_ERR("Failed to save SPS/PPS headers: Too large");
		}
	}

	uint32_t frameCount = 0;
	uint32_t uvcDropedCount = 0;
	uint32_t startupBurstCounter = 3;
	uint64_t timestamp = Time::GetMs();
	uint64_t deltaTime = 0;
	uint32_t timeCap, timeConvMCU, timeEncJpeg;

	uint8_t jpegBufIx = 0;	// The active index for hardware to write to

	bool isRecording = false;
	uint32_t recordedFrames = 0;
	uint32_t recordedSize = 0;
	FX_FILE videoFile;

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
				
				LOG_INFO("HD-CAM PU Ctrl %d: %d", puControlRegistry[i].selector, newTarget);
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
				
				LOG_INFO("HD-CAM IT Ctrl %d: %d", itControlRegistry[i].selector, newTarget);
			}
		}

		if(uvcStreamRequested == true) {
			uvcStreamRequested = false;
			startupBurstCounter = 3;
			LOG_INFO("UVC Host connected! Forcing IDR keyframes.");
		}

		if(uvcFormatChanged == true) {
			uvcFormatChanged = false;
			const FrameFormat* newFormat = usbUVC.GetActiveFormat();
			if(newFormat != nullptr) {
				LOG_INFO("UVC Host requested format change: %dx%d", newFormat->width, newFormat->height);
				// Execute the format change
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
				// status = ImageProcessor::ConvertToMCU(rawFrameBuffer, processedFrameBuffer);
				status = ImageProcessor::ConvertFormat(rawFrameBuffer, processedFrameBuffer);
				timeConvMCU = Time::GetUs() - timestamp;
				timestamp = Time::GetUs();

				// Request a keyframe for the first 3 frames
                bool requestIFrame = (startupBurstCounter > 0);

				// Compress Raw Frame to JPEG
				// status = jpegEncoder.EncodeAsync(processedFrameBuffer, jpegBuf, 80);
				status = vencEncoder.EncodeAsync(processedFrameBuffer, jpegBuf, requestIFrame);
				if(status == Status::Ok) {
					// status = jpegEncoder.EncodeWait(1000);	// Encoding takes about 6ms
					status = vencEncoder.EncodeWait(1000);
					jpegBuf.timestampUs = Time::GetUs();

					timeEncJpeg = Time::GetUs() - timestamp;
					timestamp = Time::GetUs();

					if(status == Status::Ok) {
						if(startupBurstCounter > 0) {
							startupBurstCounter = startupBurstCounter - 1;
						}
						
						// USB UVC Stuff
						// For H264, the SPS/PPS header must be injected at the start of a stream (as with saving to a file/SD card). This is done here, adding the cached header ("spsPpsFrame") at the beginning of the first frames.
						status = usbUVC.SubmitFrame(&jpegBuf, requestIFrame ? &spsPpsFrame : nullptr);
						if(status == Status::Ok) {
							// tx_thread_sleep(50);
							// jpegBufIx = 1 - jpegBufIx;	// Ping-pong buffer indexing
						}
						else {
							uvcDropedCount += 1;
							// LOG_WARN("UVC Frame Drop.");
						}

						if(saveSnapshot == true) {
							// For stream/video recording mode
							if(jpegBuf.isKeyframe == true && isRecording == false) {
								if(StorageThread::IsReady() == true) {
									Status res = ImageWriter::OpenStream(videoFile, *StorageThread::GetMedia(), "vid.264", spsPpsFrame, 10 * 1024 * 1024);
									if(res == Status::Ok) {
										isRecording = true;
										recordedFrames = 0;
										LOG_INFO("Recording started...");
									}
									saveSnapshot = false;
								}
								// // Open the file when recording starts
								// fx_file_delete(StorageThread::GetMedia(), const_cast<char*>("vid.264"));
								// if(fx_file_create(StorageThread::GetMedia(), const_cast<char*>("vid.264")) == FX_SUCCESS) {
								// 	if(fx_file_open(StorageThread::GetMedia(), &videoFile, const_cast<char*>("vid.264"), FX_OPEN_FOR_WRITE) == FX_SUCCESS) {
								// 		// Pre-allocate a contiguous space upfront, preventing FileX from searching the FAT table during write burst.
								// 		UINT allocStatus = fx_file_allocate(&videoFile, 10 * 1024 * 1024);
								// 		if(allocStatus == FX_SUCCESS) {
								// 			// Write the SPS/PPS header
								// 			UINT writeStatus = fx_file_write(&videoFile, spsPpsFrame.startAddress, spsPpsFrame.payloadSize);
								// 			if(writeStatus == FX_SUCCESS) {
								// 				isRecording = true;
								// 				recordedFrames = 0;
								// 				recordedSize = spsPpsFrame.payloadSize;
								// 				LOG_INFO("Recording started");
								// 			}
								// 			else {
								// 				LOG_ERR("Failed to write headers: %u", writeStatus);
								// 				fx_file_close(&videoFile);
								// 			}
								// 		}
								// 		else {
								// 			LOG_ERR("Failed to pre-allocate file space: %u", allocStatus);
								// 			fx_file_close(&videoFile);
								// 		}
								// 	}
								// }
								// saveSnapshot = false;
							}
							// if(StorageThread::IsReady() == true) {
							// 	// Dynamic file name
							// 	static uint32_t snapCounter = 0;
							// 	char fileName[16];
							// 	// snprintf(fileName, sizeof(fileName), "snap%04lu.bin", snapCounter++);
							// 	// ImageWriter::SaveBMP(rawFrameBuffer, *StorageThread::GetMedia(), fileName);
							// 	// ImageWriter::SaveBinary(rawFrameBuffer, *StorageThread::GetMedia(), fileName);
							// 	snprintf(fileName, sizeof(fileName), "snap%04lu.jpeg", snapCounter++);
							// 	ImageWriter::SaveBinary(jpegBuf, *StorageThread::GetMedia(), fileName);
							// 	LOG_INFO("Saved to SD card. Size: %d kB", (jpegBuf.payloadSize >> 10));
							// }
							// saveSnapshot = false;
						}
						if(isRecording == true) {
							Status res = ImageWriter::AppendStream(videoFile, jpegBuf);
							if(res == Status::Ok) {
								recordedFrames++;
							}
							else {
								ImageWriter::CloseStream(videoFile, *StorageThread::GetMedia());
								isRecording = false;
							}

							if(recordedFrames >= 60) {
								ImageWriter::CloseStream(videoFile, *StorageThread::GetMedia());
								isRecording = false;
								LOG_INFO("Recording finished! Saved %d frames, %d kB", recordedFrames, (recordedSize >> 10));
							}
							// if(StorageThread::IsReady() == true) {
							// 	// Write frames sequentially without closing the file
							// 	UINT writeStatus = fx_file_write(&videoFile, jpegBuf.startAddress, jpegBuf.payloadSize);
							// 	if(writeStatus == FX_SUCCESS) {
							// 		recordedFrames = recordedFrames + 1;
							// 		recordedSize = recordedSize + jpegBuf.payloadSize;
							// 	}
							// 	else {
							// 		LOG_ERR("Recording write failed, stopping recording. Error: %u", writeStatus);
							// 		fx_file_close(&videoFile);
							// 		isRecording = false;
							// 	}

							// 	// Stop and close after 60 frames
							// 	if(recordedFrames >= 60) {
							// 		fx_file_close(&videoFile);
							// 		fx_media_flush(StorageThread::GetMedia());
							// 		isRecording = false;
							// 		LOG_INFO("Recording finished! Saved %d frames, %d kB", recordedFrames, (recordedSize >> 10));
							// 	}
							// }
						}

						frameCount += 1;
						if(deltaTime < Time::GetMs()) {
							// Only calculate and report FPS every 1s
							float fps = frameCount / 5.0f;
							LOG_INFO("HD-CAM: %.1f FPS (UVC Drop %d/%d)", fps, uvcDropedCount, frameCount);
							// Performance values for VGA 640x480 on "Legacy" JPEG Encoder:
							// From AN4996 (STM32H7+SDRAM@100M): To MCU 58 ms, Enc 4 ms, TOTAL 62 ms

							// SD-CAM & HyperRAM@100M & Code in SRAM:			Cap 32228 us, To MCU  86573 us, Enc 6525 us

							// HD-CAM & HyperRAM@50M & Code in SRAM:			Cap 61834 us, To MCU 166303 us, Enc 9266 us
							// HD-CAM & HyperRAM@100M & Code in SRAM:			Cap 40924 us, To MCU  85077 us, Enc 7130 us
							// HD-CAM & HyperRAM@200M & Code in SRAM:			Cap 44978 us, To MCU  48642 us, Enc 6347 us

							// HD-CAM & HyperRAM@50M & Code in HyperFlash/XIP:	Cap 61809 us, To MCU 159115 us, Enc 9392 us
							// HD-CAM & HyperRAM@200M & Code in HyperFlash/XIP:	Cap 44390 us, To MCU  49146 us, Enc 6441 us

							// Performance values for VGA 640x480 on VENC JPEG Encoder (all with HyperRAM@200M):
							// HD-CAM & All Buf in HyperRAM & Code in SRAM:										Cap 46174 us, To YUYV  15704 us, Enc 4743 us
							// HD-CAM & Venc Pool in SRAM, rest in HyperRAM & Code in SRAM:						Cap 46298 us, To YUYV  15700 us, Enc 4649 us
							// HD-CAM & Venc Pool, Proc. Buffer in SRAM, rest in HyperRAM & Code in SRAM:		Cap 43988 us, To YUYV  18947 us, Enc 3711 us
							// HD-CAM & Venc Pool, Proc., Out Buffer in SRAM, rest in HyperRAM & Code in SRAM:	Cap 43979 us, To YUYV  18889 us, Enc 3684 us

							//Note: Removing all Clean/Invalidate cache from VENCEncoder and VENC reduces Encoding by ~1000us
							//Alternative (better), optimized cache clean/invaldiate calls to minimum reduced about 500us. Using full cache clean on large buffers (like the video buffer) reduced it a further 400us:
							// Performance values for VGA 640x480 on VENC JPEG Encoder (all with HyperRAM@200M) after cache handling optimization:
							// HD-CAM & All Buf in HyperRAM & Code in SRAM:										Cap 47166 us, To YUYV  15704 us, Enc 3763 us
							// HD-CAM & Venc Pool in SRAM, rest in HyperRAM & Code in SRAM:						Cap 47329 us, To YUYV  15705 us, Enc 3602 us
							// HD-CAM & Venc Pool, Proc. Buffer in SRAM, rest in HyperRAM & Code in SRAM:		Cap 45085 us, To YUYV  18887 us, Enc 2661 us
							
							LOG_INFO("HD-CAM: DCMI %d us, MCU Blk %d us, JPEG %d us", timeCap, timeConvMCU, timeEncJpeg);	
							frameCount = 0;
							uvcDropedCount = 0;
							deltaTime = Time::GetMs() + 5000;
						}
					}
				}
			}
			else {
				// LOG_WARN("HD-CAM Frame Drop.");
			}
		}
		else {
			// LOG_WARN("HD-CAM Frame Drop.");
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