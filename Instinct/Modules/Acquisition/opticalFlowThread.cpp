/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	SDK/Acquisition/opticalFlowThread.cpp
 * Author:	NotBlackMagic
 */

#include "opticalFlowThread.hpp"

// Allocate the static variables
TX_THREAD OpticalFlowThread::threadPtr;
__attribute__((aligned(32))) uint8_t OpticalFlowThread::threadStack[OpticalFlowThread::threadStackSize];
TX_EVENT_FLAGS_GROUP OpticalFlowThread::event;

// __attribute__((section(".axisram1"), aligned(32)))

__attribute__((aligned(32))) uint8_t frameBufferSRAM1[65536];
__attribute__((aligned(32))) uint8_t frameBufferSRAM2[65536];
static VisionFrame rawFrameBuffer[2];
static uint8_t captureIndex = 0;
OpticalFlow OpticalFlowThread::flow;

// Output Publisher
Topic<FlowMsg> OpticalFlowThread::topicFlow = Topic<FlowMsg>("flow", static_cast<uint8_t>(TopicID::Flow), 0);
Topic<RangeMsg> OpticalFlowThread::topicRange = Topic<RangeMsg>("range", static_cast<uint8_t>(TopicID::Range), 0);

Topic<IMUMsg>* OpticalFlowThread::topicVirtualIMU = nullptr;

void OpticalFlowThread::ToFIntCallback(void* ctx, EXTIManager::Edge edge) {
	(void)ctx;
	(void)edge;
	tx_event_flags_set(&event, EVT_TOF_DRDY, TX_OR);
}

void OpticalFlowThread::DCMIFrameCallback(void* ctx, CameraDCMI::Event evt) {
	(void)ctx;
	if(evt == CameraDCMI::Event::FrameComplete) {
		tx_event_flags_set(&event, EVT_DCMI_FRAME, TX_OR);
	}
	else {
		tx_event_flags_set(&event, EVT_DCMI_ERR, TX_OR);
	}
}

void OpticalFlowThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("Optical Flow"),
										OpticalFlowThread::Run, 0,
										threadStack, sizeof(threadStack),
										7, 7,
										TX_NO_TIME_SLICE, TX_AUTO_START);
	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Optical Flow Thread Create Failed.");
	}

	Broker::RegisterTopic(&topicFlow);
	Broker::RegisterTopic(&topicRange);
}

void OpticalFlowThread::Run(ULONG input) {
	(void)input;
	LOG_INFO("Optical Flow Thread Initialized.");

	// FlowMsg flowMsg;
	float distanceMatrix[64];
	uint8_t targetStatus[64];
	uint64_t lastCaptureTime = 0;

	// General camera configurations
	// OV9281 Native Resolution: 1280x800 -> Hardware binning 2x2: 640x400 -> Hardware windowing: 256x256
	// OV9281 Native FoV: 120 deg -> Hardware windowing: ~48 deg
	uint32_t camWidth = 256;
	uint32_t camHeight = 256;
	uint8_t camFPS = 30;
	PixelFormat camFormat = PixelFormat::Grayscale;

	// Initialize OpticalFlow solver
	OpticalFlow::Config flowCfg;
	flowCfg.fx = 143.7f; // 256x256 crop (~48 deg FOV) -> 128x128 L0
	flowCfg.fy = 143.7f;
	flow.Init(flowCfg);

	// Create RTOS objects
	if(tx_event_flags_create(&event, const_cast<char*>("Flow_event")) != TX_SUCCESS) {
		return;
	}

	volatile uint8_t probeI3C1 = i3c1.Probe(0x52);

	// External/offboard downward ToF Sensor (VL53L8CH) Init
	if(i3c1.Probe(0x52) == 0x01) {
		VL53L8CH::Config tofCfg = {
			.resolution = VL53L8CH::Resolution::Grid8x8,
			.odr = VL53L8CH::OutputDataRate::Hz15,
			.rangingMode = VL53L8CH::RangingMode::Continuous,
			.targetOrder = VL53L8CH::TargetOrder::Closest,
			.integTime = 20
		};
		if(extVL53ToF.Init(tofCfg) == Status::Ok) {
			if(EXTIManager::RegisterCallback(camInt0.GetPinIndex(), ToFIntCallback, nullptr) == Status::Ok) {
				// Falling edge: VL53L8CH GPIO1 pulls low on DRDY
				camInt0.EnableIRQ(GPIO::Interrupt::Falling, 0x0E);
				LOG_INFO("Ext ToF (VL53L8CH) Init & EXTI Routed OK");
			}
			else {
				LOG_WARN("Ext ToF EXTI Routing Failed!");
			}
		}
		else {
			LOG_WARN("Ext ToF (VL53L8CH) Init Failed!");
		}
	}
	else {
		LOG_WARN("Ext ToF (VL53L8CH) not found on I3C1!");
	}

	// Setup frame buffer
	rawFrameBuffer[0].width = camWidth;
	rawFrameBuffer[0].height = camHeight;
	rawFrameBuffer[0].format = camFormat;
	rawFrameBuffer[0].payloadSize = camWidth * camHeight * 1;
	rawFrameBuffer[0].allocatedSize = camWidth * camHeight * 1;
	rawFrameBuffer[0].startAddress = frameBufferSRAM1;

	rawFrameBuffer[1].width = camWidth;
	rawFrameBuffer[1].height = camHeight;
	rawFrameBuffer[1].format = camFormat;
	rawFrameBuffer[1].payloadSize = camWidth * camHeight * 1;
	rawFrameBuffer[1].allocatedSize = camWidth * camHeight * 1;
	rawFrameBuffer[1].startAddress = frameBufferSRAM2;

	// SD-CAM Initialization
	CameraDCMI::Config configSD;
	configSD.width = camWidth;
	configSD.height = camHeight;
	configSD.fps = camFPS;
	configSD.format = camFormat;
	configSD.EventCallback = OpticalFlowThread::DCMIFrameCallback;
	configSD.callbackContext = nullptr;
	Status status = cameraSD.Init(configSD);
	if(status != Status::Ok) {
		LOG_ERR("SD-CAM Init Failed.");
	}
	else {
		LOG_INFO("SD-CAM Initialized.");
	}
	tx_thread_sleep(100);
	cameraSD.GetSensor().Start();
	cameraSD.GetSensor().SetAnalogGain(0x10);
	cameraSD.GetSensor().SetExposure(800);

	// Prime the pipeline: begin capturing into the first buffer
	captureIndex = 0;
	status = cameraSD.CaptureAsync(rawFrameBuffer[captureIndex]);
	if(status != Status::Ok) {
		LOG_ERR("Initial Camera CaptureAsync Failed.");
	}
	lastCaptureTime = Time::GetUs();

	IMUMsg imuMsg{};

	uint64_t timestamp = Time::GetUs();
	uint64_t deltaTime = 0;
	uint32_t timeProc;

	ULONG events = 0;
	UINT txStatus = TX_SUCCESS;
	while(1) {
		// Wait for ToF DRDY, Camera Frame, or Camera Error
		txStatus = tx_event_flags_get(&event, EVT_TOF_DRDY | EVT_DCMI_FRAME | EVT_DCMI_ERR, TX_OR_CLEAR, &events, 10);
		if(txStatus != TX_SUCCESS) {
			// Guard against stale event flags on timeout
			events = 0;
		}	
		uint64_t currentTime = Time::GetUs();

		// Handle Camera New Frame
		if((events & EVT_DCMI_FRAME) == EVT_DCMI_FRAME) {
			// Ping-pong frame buffers
			uint8_t completedIndex = captureIndex;
			captureIndex ^= 1;

			status = cameraSD.CaptureAsync(rawFrameBuffer[captureIndex]);
			lastCaptureTime = Time::GetUs();

			if(status != Status::Ok) {
				// LOG_WARN("SD-CAM Frame Drop / Error. Restarting Pipeline...");

				cameraSD.CaptureAbort();
				cameraSD.CaptureAsync(rawFrameBuffer[captureIndex]);
				lastCaptureTime = Time::GetUs();
			}

			// Work on the completed frame
			VisionFrame& completedFrame = rawFrameBuffer[completedIndex];

			// Non-blocking Peek of latest IMU gyro
			float gyro[3] = {0.0f, 0.0f, 0.0f};
			if(topicVirtualIMU == nullptr) {
				topicVirtualIMU = Broker::GetTopic<IMUMsg>(static_cast<uint8_t>(TopicID::Imu), virtualInstanceID);
			}
			if(topicVirtualIMU != nullptr && topicVirtualIMU->Peek(imuMsg) == true) {
				gyro[0] = imuMsg.gyro[0];
				gyro[1] = imuMsg.gyro[1];
				gyro[2] = imuMsg.gyro[2];
			}

			// Optical Flow Calculation
			timestamp = Time::GetUs();
			if(flow.Process(completedFrame, gyro) == Status::Ok) {
				const OpticalFlow::Output& flowOutput = flow.GetOutput();
				if(flowOutput.isValid == true) {
					FlowMsg flowMsg{};
					flowMsg.timestamp = completedFrame.timestampUs;
					flowMsg.flowRate[0] = flowOutput.flowRate.x;
					flowMsg.flowRate[1] = flowOutput.flowRate.y;
					flowMsg.quality = flowOutput.quality;
					topicFlow.Publish(flowMsg);
				}
				// LOG_INFO("Optical Flow: x: %.2f, y: %.2f, q: %.2f", flowOutput.flowRate.x, flowOutput.flowRate.y, flowOutput.quality);
			}

			timeProc = Time::GetUs() - timestamp;
			if(deltaTime < Time::GetMs()) {
				// Original, first version: "PROC 3316 us"
				// 1st Opt. (moving more to MVE extensions and reducing certain iterations in L2 and L1): "PRE 63 PROC 1530 us"
				// 2nd Opt. (move frame buffer from PSRAM to SRAM): "PRE 64 PROC 1290 us"
				// LOG_INFO("Optical Flow Thread: PRE %d PROC %d us", timePre, timeProc);
				deltaTime = Time::GetMs() + 5000;
			}

			// Debug USB stream: submit the frame without blocking the capture loop
			// status = usbUVC.SubmitFrame(&completedFrame, nullptr);
			// if(status != Status::Ok) {
			// 	// USB endpoint busy: drop the preview frame without affecting flow calculations
			// }
		}
		else {
			// Check for transfer failure or camera hang (> 150 ms without a completed frame)
			if((events & EVT_DCMI_ERR) != 0 || (Time::GetUs() - lastCaptureTime > 150000)) {
				// LOG_WARN("SD-CAM Frame Drop / Error. Restarting Pipeline...");

				cameraSD.CaptureAbort();

				// Clear any stale flags that may have latched during abort
				ULONG staleEvents = 0;
				tx_event_flags_get(&event, EVT_DCMI_FRAME | EVT_DCMI_ERR, TX_OR_CLEAR, &staleEvents, TX_NO_WAIT);
				
				cameraSD.CaptureAsync(rawFrameBuffer[captureIndex]);
				lastCaptureTime = Time::GetUs();
			}
		}

		// Handle ToF Ground Distance
		if((events & EVT_TOF_DRDY) == EVT_TOF_DRDY) {
			extVL53ToF.RequestData();
			if(extVL53ToF.GetData(distanceMatrix, targetStatus) == Status::Ok) {
				// Average the 4 central zones (zones 27, 28, 35, 36 in an 8x8 matrix) for normal incidence height
				float sumDistanceMm = 0.0f;
				uint8_t validCount = 0;
				const uint8_t centerZones[4] = {27, 28, 35, 36};

				for(uint8_t i = 0; i < 4; i++) {
					uint8_t idx = centerZones[i];
					// Target status 5 and 9 indicate valid ranging targets in ST ULD
					if(targetStatus[idx] == 5 || targetStatus[idx] == 9) {
						sumDistanceMm += distanceMatrix[idx];
						validCount++;
					}
				}

				if(validCount > 0) {
					RangeMsg rangeMsg{};
					rangeMsg.timestamp = currentTime;
					rangeMsg.distance = (sumDistanceMm / static_cast<float>(validCount)) * 0.001f;
					topicRange.Publish(rangeMsg);
				}
			}
		}
	}
}