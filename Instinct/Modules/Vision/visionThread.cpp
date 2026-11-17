/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/Vision/visionThread.cpp
 */

#include "visionThread.hpp"

TX_THREAD VisionThread::threadPtr;
__attribute__((aligned(32))) uint8_t VisionThread::threadStack[VisionThread::threadStackSize];

VisionThread::Statistics VisionThread::stats = {};
uint32_t VisionThread::pipelineRestartCounter = 0;

const uint32_t rawBufSize = 1920 * 64 * 2;
__attribute__((section(".axisram1"), aligned(32))) uint8_t rawFrameBuf[rawBufSize];

static VisionFrame rawFrameBuffer[2];
static VisionFrame processedFrameBuffer;
static VisionFrame jpegFrameBuffer[2];

// Static state definitions
volatile bool VisionThread::captureEnabled = true;
volatile bool VisionThread::recordingRequested = false;
volatile bool VisionThread::isRecording = false;

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

void VisionThread::SetCapture(bool enable) {
	captureEnabled = enable;
}

bool VisionThread::IsCapturing() {
	return captureEnabled;
}

void VisionThread::SetRecording(bool enable) {
	recordingRequested = enable;
}

bool VisionThread::IsRecording() {
	return isRecording;
}

void VisionThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr,	const_cast<char*>("Vision"),
													VisionThread::Run, 0,
													threadStack, sizeof(threadStack),
													10, 10,
													TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Vision Thread Create Failed.");
	}
}

void VisionThread::Run(ULONG input) {
	(void)input;
	Status status;
	
	LOG_INFO("Vision Thread Initialized.");

	// General camera configurations
	uint32_t camWidth = 1920;
	uint32_t camHeight = 1088;
	uint8_t camFPS = 20;
	PixelFormat camFormat = PixelFormat::YUV422_YUYV;

	// Line mode parameters
#if USE_LINE_MODE
	constexpr Dcmipp::LineCount lineWrap = Dcmipp::LineCount::Lines64;
	constexpr Dcmipp::LineCount lineTrigger = Dcmipp::LineCount::Lines32;
	// vv For statistics/metering only vv
	constexpr uint32_t wrapLines = 1U << static_cast<uint32_t>(lineWrap);
	constexpr uint32_t triggerLines = 1U << static_cast<uint32_t>(lineTrigger);
	const float numSlices = static_cast<float>(camHeight) / static_cast<float>(triggerLines);
#endif

	// Initialize
	// USB UVC stuff
	usbUVC.RegisterStreamEventCallback(VisionThread::OnUVCStreamState);

	// HD-CAM Initialization
	CameraMIPI::Config configHD;
	configHD.width = camWidth;
	configHD.height = camHeight;
	configHD.fps = camFPS;
	configHD.format = camFormat;
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

	// Dedicated (legacy) JPEG Encoder Initialization
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
#if USE_LINE_MODE
	vencConfig.enableStreamingMode = true;
	vencConfig.lineWrap = static_cast<Venc::LineCount>(lineWrap);
	vencConfig.lineTrigger = static_cast<Venc::LineCount>(lineTrigger);
	// 64 lines = 4 MB rows per slice. Or 0 = encode whole frame as one slice
	vencConfig.codingControl.sliceSize = 0;
#else
	vencConfig.enableStreamingMode = false;
	vencConfig.codingControl.sliceSize = 0;	// 0 = encode whole frame as one slice
#endif
	vencConfig.codec = Venc::Codec::H264;
	vencConfig.imageParams.width = camWidth;
	vencConfig.imageParams.height = 1080;
	vencConfig.imageParams.frameRate = camFPS;
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
	vencConfig.codingControl.profile = Venc::H264Profile::High;
	vencConfig.codingControl.insertIdrHeader = true;
	vencConfig.codingControl.disableDeblockingFilter = false;

	// Required pool size can be found in "VC8000NanoE Video Encoder Software Integration Guide", Section H264 4.3.1-4.3.4
	// Note: VENC always stores internal reference frames in 4:2:0 (1.5 B/px), regardless of input format.
	//	Reference frames: W * H * 3.0 B (Double-buffered: 2 Luma + 2 Chroma = 6.27 MB @ 1920x1088)
	//	Shared + SW bufs: W * H * 0.16 B + 54 KB (MV, NAL, CABAC tables = 0.38 MB @ 1920x1088)
	// Minimum pool size = (W * H * 3.16) + 64 KB  (~6.7 MB for 1080p)
	vencConfig.poolSize = 8 * 1024 * 1024;
	vencConfig.poolAddress = externalPSRAM.Allocate(vencConfig.poolSize, 32);
	status = vencEncoder.Init(vencConfig);
	if(status != Status::Ok) {
		LOG_ERR("VENC H264 Encoder Init Failed.");
		while(1) {
			tx_thread_sleep(1000);
		}
	}
	LOG_INFO("VENC H264 Encoder Initialized.");

	// Buffer sizing
#if USE_LINE_MODE
	// uint32_t rawBufSize = camWidth * wrapLines * 2;
#else
	uint32_t rawBufSize = camWidth * camHeight * 2;
#endif
	uint32_t comprBufSize = (camWidth * camHeight * 2) / 8;

	// Setup frame buffer
	rawFrameBuffer[0].width = camWidth;
	rawFrameBuffer[0].height = camHeight;
	rawFrameBuffer[0].format = camFormat;
	rawFrameBuffer[0].payloadSize = rawBufSize;
	rawFrameBuffer[0].allocatedSize = rawBufSize;
	rawFrameBuffer[0].startAddress = rawFrameBuf;	// externalPSRAM.Allocate(rawFrameBuffer[0].allocatedSize, 32);

	rawFrameBuffer[1].width = camWidth;
	rawFrameBuffer[1].height = camHeight;
	rawFrameBuffer[1].format = camFormat;
	rawFrameBuffer[1].payloadSize = rawBufSize;
	rawFrameBuffer[1].allocatedSize = rawBufSize;
	rawFrameBuffer[1].startAddress = externalPSRAM.Allocate(rawFrameBuffer[1].allocatedSize, 32);

	// processedFrameBuffer.format = PixelFormat::Unknown;	// Handled by processor
	// processedFrameBuffer.width = camWidth;
	// processedFrameBuffer.height = camHeight;
	// processedFrameBuffer.payloadSize = camWidth * camHeight * 2;
	// processedFrameBuffer.allocatedSize = camWidth * camHeight * 2;
	// processedFrameBuffer.startAddress = externalPSRAM.Allocate(processedFrameBuffer.allocatedSize, 32);

	jpegFrameBuffer[0].format = PixelFormat::Unknown;	// Handled by codec
	jpegFrameBuffer[0].codec = VisionCodec::H264;
	jpegFrameBuffer[0].width = camWidth;
	jpegFrameBuffer[0].height = camHeight;
	jpegFrameBuffer[0].allocatedSize = comprBufSize;
	jpegFrameBuffer[0].startAddress = externalPSRAM.Allocate(jpegFrameBuffer[0].allocatedSize, 32);

	jpegFrameBuffer[1].format = PixelFormat::Unknown;	// Handled by codec
	jpegFrameBuffer[1].codec = VisionCodec::H264;
	jpegFrameBuffer[1].width = camWidth;
	jpegFrameBuffer[1].height = camHeight;
	jpegFrameBuffer[1].allocatedSize = comprBufSize;
	jpegFrameBuffer[1].startAddress = externalPSRAM.Allocate(jpegFrameBuffer[1].allocatedSize, 32);

	// H.264 Stream Start (SPS/PPS Generation)
	static uint8_t spsPpsStorage[128];		// Dedicated buffer for SPS/PPS headers
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

	uint32_t epochFrameCount = 0;
	uint32_t startupBurstCounter = 3;
	uint64_t epochStartMs = Time::GetMs();
	uint64_t timestamp = Time::GetUs();

	uint32_t timeCap = 0;
	uint32_t timeConvMCU = 0;
	uint32_t timeEnc = 0;
	uint32_t timeIO = 0;

#if USE_LINE_MODE
	uint32_t timeSliceCap = 0;
	uint32_t timeSliceEnc = 0;
#endif

	uint8_t rawBufIx = 0;	// The active index for MCU to read from
	uint8_t jpegBufIx = 0;	// The active index for hardware to write to

	// Hardware Tracking Variables
	uint32_t encRawBytes = 0;
	uint32_t encOutBytes = 0;
	uint64_t epochEncBytesSum = 0;

	uint32_t recordedFrames = 0;
	uint32_t recordedSize = 0;
	FX_FILE videoFile;

	bool saveSnapshot = false;
	bool lastBtnState = false;

	bool hwCapturing = false;
	if(captureEnabled == true) {
#if USE_LINE_MODE
		// Start continuous hardware line-wrapping capture
		status = cameraHD.CaptureAsync(rawFrameBuffer[0], lineTrigger, lineWrap);
#else
		// Start hardware continuous capture
		status = cameraHD.CaptureAsync(rawFrameBuffer[0], rawFrameBuffer[1]);
#endif
		if(status != Status::Ok) {
			LOG_ERR("Failed to start HD-CAM capture.");
		}
		else {
			hwCapturing = true;
		}
	}

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

		// Hardware Capture Start/Stop Gate
		if(captureEnabled == false) {
			if(hwCapturing == true) {
				cameraHD.CaptureAbort();
#if USE_LINE_MODE
				vencEncoder.EncodeAbort();
#endif
				if(isRecording == true) {
					ImageWriter::CloseStream(videoFile, *StorageThread::GetMedia());
					isRecording = false;
					recordingRequested = false;
				}
				hwCapturing = false;
				LOG_INFO("HD-CAM Capture Suspended.");
			}
			tx_thread_sleep(10);
		}
		else {
			if(hwCapturing == false) {
#if USE_LINE_MODE
				status = cameraHD.CaptureAsync(rawFrameBuffer[0], lineTrigger, lineWrap);
#else
				status = cameraHD.CaptureAsync(rawFrameBuffer[0], rawFrameBuffer[1]);
#endif
				if(status == Status::Ok) {
					hwCapturing = true;
					rawBufIx = 0;
					jpegBufIx = 0;
					startupBurstCounter = 3;
					timestamp = Time::GetUs();
					LOG_INFO("HD-CAM Capture Resumed.");
				}
				else {
					LOG_ERR("Failed to resume HD-CAM capture.");
					tx_thread_sleep(10);
				}
			}

			if(hwCapturing == true) {
				// Measure IO/Idle Overhead
				uint64_t waitStartUs = Time::GetUs();
				timeIO = static_cast<uint32_t>(waitStartUs - timestamp);
				stats.ioTime = timeIO;
				
				// status = cameraHD.CaptureAsync(rawFrameBuffer);
				status = Status::Ok;
				if(status == Status::Ok) {
					status = cameraHD.CaptureWait(1000);
					// status = PatternGenerator::Checkerboard(rawFrameBuffer);
					if(status == Status::Ok) {
#if USE_LINE_MODE
						VisionFrame& currentRaw = rawFrameBuffer[0];
						VisionFrame& currentJpeg = jpegFrameBuffer[jpegBufIx];
						jpegBufIx = 1 - jpegBufIx;
#else
						VisionFrame& currentRaw = rawFrameBuffer[rawBufIx];
						VisionFrame& currentJpeg = jpegFrameBuffer[jpegBufIx];
						rawBufIx = 1 - rawBufIx;
						jpegBufIx = 1 - jpegBufIx;

						uint64_t waitEndUs = Time::GetUs();
						timeCap = static_cast<uint32_t>(waitEndUs - waitStartUs);
						stats.capTime = timeCap;
#endif

						currentRaw.timestampUs = Time::GetUs();

						// Request a keyframe on startup burst or when video recording is requested
						bool requestIFrame = (startupBurstCounter > 0) || ((recordingRequested || saveSnapshot) && !isRecording);

						// Pass through image processor to transfor to MCU blocks
						// status = ImageProcessor::ConvertToMCU(rawFrameBuffer, processedFrameBuffer);
						// status = ImageProcessor::ConvertFormat(rawFrameBuffer, processedFrameBuffer);
						timeConvMCU = 0;	//Time::GetUs() - timestamp;
						// timestamp = Time::GetUs();

						// Compress Raw Frame to JPEG
						// status = jpegEncoder.EncodeAsync(processedFrameBuffer, jpegBuf, 80);
						uint64_t encStartUs = Time::GetUs();
						status = vencEncoder.EncodeAsync(currentRaw, currentJpeg, requestIFrame);
						if(status == Status::Ok) {
							// status = jpegEncoder.EncodeWait(1000);	// Encoding takes about 6ms
							status = vencEncoder.EncodeWait(1000);

							uint64_t encEndUs = Time::GetUs();
							currentJpeg.timestampUs = encEndUs;
#if USE_LINE_MODE
							// Exact HW time: DCMIPP and VENC frame complete timestamps. The delta is exactly how long VENC took to chew the final lines.
							uint64_t dcmippStart = cameraHD.GetPipeStatistics(Dcmipp::PipeID::Main).lastVsyncTimestamp;
							uint64_t dcmippDone = cameraHD.GetPipeStatistics(Dcmipp::PipeID::Main).lastFrameTimestamp;
							uint64_t vencDone = vencEncoder.GetStatistics().lastFrameEndTimestamp;

							// Camera active frame readout time (VSYNC to last line DMA complete)
							timeCap = static_cast<uint32_t>(dcmippDone - dcmippStart);
							stats.capTime = timeCap;

							// HW encode time for one 32-line slice (measured on the final slice after DCMIPP finishes)
							timeSliceEnc = static_cast<uint32_t>(vencDone - dcmippDone);
							stats.sliceEnc = timeSliceEnc;
							if(timeSliceEnc > stats.maxSliceEnc) {
								stats.maxSliceEnc = timeSliceEnc;
							}

							// Estimated pure VENC compute time for the full frame (excluding camera wait time)
							timeEnc = timeSliceEnc * numSlices;
							stats.encTime = timeEnc;
							if(timeEnc > stats.maxEncTime) {
								stats.maxEncTime = timeEnc;
							}

							// Camera readout time per 32-line slice (must be > timeSliceEnc to avoid overrun)
							timeSliceCap = timeCap / numSlices;

							// Fetch byte counts
							encOutBytes = vencEncoder.GetStatistics().lastFrameBytes;					
#else
							// VENC encode time for the full frame
							timeEnc = static_cast<uint32_t>(encEndUs - encStartUs);
							stats.encTime = timeEnc;
							if(timeEnc > stats.maxEncTime) {
								stats.maxEncTime = timeEnc;
							}

							// Fetch byte counts
							encOutBytes = vencEncoder.GetStatistics().lastFrameBytes;
#endif
							timestamp = Time::GetUs();

							// Update payload and compression ratio
							stats.lastFrameBytes = encOutBytes;
							encRawBytes = (currentRaw.format == PixelFormat::YUV420_NV12) ? ((currentRaw.width * currentRaw.height * 3) / 2) : (currentRaw.width * currentRaw.height * 2);
							if(encOutBytes > 0) {
								stats.compression = static_cast<float>(encRawBytes) / static_cast<float>(encOutBytes);
								if(stats.compressionMin > stats.compression) {
									stats.compressionMin = stats.compression;
								}
								epochEncBytesSum += encOutBytes;
							}

							if(status == Status::Ok) {
								if(startupBurstCounter > 0) {
									startupBurstCounter = startupBurstCounter - 1;
								}
								
								// USB UVC Stuff
								// For H264, the SPS/PPS header must be injected at the start of a stream (as with saving to a file/SD card). This is done here, adding the cached header ("spsPpsFrame") at the beginning of the first frames.
								status = usbUVC.SubmitFrame(&currentJpeg, currentJpeg.isKeyframe ? &spsPpsFrame : nullptr);
								if(status == Status::Ok) {
									// tx_thread_sleep(50);
									// jpegBufIx = 1 - jpegBufIx;	// Ping-pong buffer indexing
								}
								else {
									stats.uvcDroppedCount = stats.uvcDroppedCount + 1;
									// LOG_WARN("UVC Frame Drop.");
								}

								// Integrated Recording Start Trigger (Keyframe aligned)
								if((recordingRequested == true || saveSnapshot == true) && isRecording == false) {
									// For stream/video recording mode
									if(currentJpeg.isKeyframe == true) {
										if(StorageThread::IsReady() == true) {
											Status res = ImageWriter::OpenStream(videoFile, *StorageThread::GetMedia(), "vid.264", spsPpsFrame, 10 * 1024 * 1024);
											if(res == Status::Ok) {
												isRecording = true;
												recordingRequested = true;
												recordedFrames = 0;
												recordedSize = spsPpsFrame.payloadSize;
												LOG_INFO("Recording started...");
											}
											else {
												LOG_WARN("Failed to open stream file.");
												recordingRequested = false;
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
								// Integrated Recording Stop Trigger (Requested stop from Shell/API)
								else if(recordingRequested == false && isRecording == true) {
									ImageWriter::CloseStream(videoFile, *StorageThread::GetMedia());
									isRecording = false;
									LOG_INFO("Recording stopped. Total frames: %lu, %lu kB", recordedFrames, (recordedSize >> 10));
								}
								if(isRecording == true) {
									Status res = ImageWriter::AppendStream(videoFile, currentJpeg);
									if(res == Status::Ok) {
										recordedFrames = recordedFrames + 1;
										recordedSize = recordedSize + currentJpeg.payloadSize;
									}
									else {
										ImageWriter::CloseStream(videoFile, *StorageThread::GetMedia());
										isRecording = false;
										recordingRequested = false;
									}

									// if(recordedFrames >= 60) {
									// 	ImageWriter::CloseStream(videoFile, *StorageThread::GetMedia());
									// 	isRecording = false;
									// 	recordingRequested = false;
									// 	LOG_INFO("Recording finished! Saved %d frames, %d kB", recordedFrames, (recordedSize >> 10));
									// }
									
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

								stats.totalFrames = stats.totalFrames + 1;
								epochFrameCount = epochFrameCount + 1;

								// Periodic Snapshot Publication (Every 1000ms)
								uint64_t currentMs = Time::GetMs();
								uint64_t elapsedMs = currentMs - epochStartMs;
								if(elapsedMs >= 1000) {
									// Performance values for VGA 640x480 on "Legacy" JPEG Encoder:
									// From AN4996 (STM32H7+SDRAM@100M): To MCU 58 ms, Enc 4 ms, TOTAL 62 ms

									// SD-CAM & HyperRAM@100M & Code in SRAM:			Cap 32228 us, To MCU 86573 us, Enc 6525 us

									// HD-CAM & HyperRAM@50M & Code in SRAM:			Cap 61834 us, To MCU 166303 us, Enc 9266 us
									// HD-CAM & HyperRAM@100M & Code in SRAM:			Cap 40924 us, To MCU 85077 us, Enc 7130 us
									// HD-CAM & HyperRAM@200M & Code in SRAM:			Cap 44978 us, To MCU 48642 us, Enc 6347 us

									// HD-CAM & HyperRAM@50M & Code in HyperFlash/XIP:	Cap 61809 us, To MCU 159115 us, Enc 9392 us
									// HD-CAM & HyperRAM@200M & Code in HyperFlash/XIP:	Cap 44390 us, To MCU 49146 us, Enc 6441 us

									// Performance values for VGA 640x480 on VENC JPEG Encoder (all with HyperRAM@200M):
									// HD-CAM & All Buf in HyperRAM & Code in SRAM:										Cap 46174 us, To YUYV 15704 us, Enc 4743 us
									// HD-CAM & Venc Pool in SRAM, rest in HyperRAM & Code in SRAM:						Cap 46298 us, To YUYV 15700 us, Enc 4649 us
									// HD-CAM & Venc Pool, Proc. Buffer in SRAM, rest in HyperRAM & Code in SRAM:		Cap 43988 us, To YUYV 18947 us, Enc 3711 us
									// HD-CAM & Venc Pool, Proc., Out Buffer in SRAM, rest in HyperRAM & Code in SRAM:	Cap 43979 us, To YUYV 18889 us, Enc 3684 us

									// Note: Removing all Clean/Invalidate cache from VENCEncoder and VENC reduces Encoding by ~1000us
									// Alternative (better), optimized cache clean/invalidates calls to minimum reduced about 500us. Using full cache clean on large buffers (like the video buffer) reduced it a further 400us:
									// Performance values for VGA 640x480 on VENC JPEG Encoder (all with HyperRAM@200M) after cache handling optimization:
									// HD-CAM & All Buf in HyperRAM & Code in SRAM:										Cap 47166 us, To YUYV 15704 us, Enc 3763 us
									// HD-CAM & Venc Pool in SRAM, rest in HyperRAM & Code in SRAM:						Cap 47329 us, To YUYV 15705 us, Enc 3602 us
									// HD-CAM & Venc Pool, Proc. Buffer in SRAM, rest in HyperRAM & Code in SRAM:		Cap 45085 us, To YUYV 18887 us, Enc 2661 us

									// Newest performance analysis for 1920x1080 H264. Code in HyperFlash/XIP. Full frame mode:
									// HD-CAM & All Buf in HyperRAM:			Cap 60985 us, Full Enc 39028 (Max: 39969) us
									// Out Buf in SRAM, rest in HyperRAM:		Cap 61032 us, Full Enc 38878 (Max: 39532) us
									// -- Same but in Line mode (32 lines): --
									// HD-CAM & All Buf in HyperRAM:			Full Enc 33966 (Max: 34170) us, Slice 996 (Max: 1005) us
									// Raw buffer in SRAM, rest in HyperRAM:	Full Enc 26690 (Max: 28798) us, Slice 785 (Max: 847) us
									// -- Same but for JPEG: --
									// HD-CAM & All Buf in HyperRAM:			Cap 77097 us, Full Enc 23266 (Max: 24000) us
									// Out Buf in SRAM, rest in HyperRAM:		Cap 77003 us, Full Enc 23510 (Max: 23731) us

									// Newest performance analysis for 1280x720. Code in HyperFlash/XIP. Full frame mode:
									// HD-CAM & All Buf in HyperRAM:			Cap 82342 us, Full Enc 17688 (Max: 18396) us

									stats.fps = (epochFrameCount * 1000.0f) / static_cast<float>(elapsedMs);
									stats.bitrateBps = static_cast<uint32_t>((epochEncBytesSum * 8000ULL) / elapsedMs);
									stats.pipelineRestarts = pipelineRestartCounter;

									// Hardware driver statistics snapshot
									stats.csi = cameraHD.GetCSIStatistics();
									stats.dcmipp = cameraHD.GetPipeStatistics(Dcmipp::PipeID::Main);
									stats.venc = vencEncoder.GetStatistics();

									// Reset epoch window
									epochFrameCount = 0;
									epochEncBytesSum = 0;
									epochStartMs = currentMs;
								}
							}
						}
						else {
							LOG_WARN("VENC Encode Error! Resetting pipeline...");
						}
					}
					else {
						// Pipeline error handling
						pipelineRestartCounter = pipelineRestartCounter + 1;
						stats.pipelineRestarts = pipelineRestartCounter;
						LOG_WARN("HD-CAM Frame Drop / Error. Restarting Pipeline...");

#if USE_LINE_MODE
						cameraHD.CaptureAbort();
						vencEncoder.EncodeAbort();
						status = cameraHD.CaptureAsync(rawFrameBuffer[0], lineTrigger, lineWrap);
#else
						// The driver caught a glitch and automatically aborted into Idle mode. Restart the continuous hardware pipeline.
						status = cameraHD.CaptureAsync(rawFrameBuffer[0], rawFrameBuffer[1]);
#endif

						// Reset application ping-pong indices.
						rawBufIx = 0;
						jpegBufIx = 0;

						// Give the MIPI PHY a tiny moment to stabilize
						tx_thread_sleep(5);
					}
				}
				else {
					// LOG_WARN("HD-CAM Frame Drop.");
				}
			}
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