/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/USB/usbClassUVC.hpp
 * Author:  NotBlackMagic
 * Brief:   UVC (USB Video Class) Class Driver
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "logger.hpp"
#include "status.hpp"
#include "usbClass.hpp"

#include "visionTypes.hpp"

class USBClassUVC : public USBClass {
	public:
		/// @brief Processing Unit Controls (Image Adjustments)
		struct PUControl {
			static constexpr uint32_t None					= 0;
			static constexpr uint32_t Brightness			= (1 << 0);
			static constexpr uint32_t Contrast				= (1 << 1);
			static constexpr uint32_t Hue					= (1 << 2);
			static constexpr uint32_t Saturation			= (1 << 3);
			static constexpr uint32_t Sharpness				= (1 << 4);
			static constexpr uint32_t Gamma					= (1 << 5);
			static constexpr uint32_t WhiteBalanceTemp		= (1 << 6);
			static constexpr uint32_t WhiteBalanceComponent	= (1 << 7);
			static constexpr uint32_t BacklightCompensation	= (1 << 8);
			static constexpr uint32_t Gain					= (1 << 9);
			static constexpr uint32_t PowerLineFrequency	= (1 << 10);
			static constexpr uint32_t HueAuto				= (1 << 11);
			static constexpr uint32_t WhiteBalanceTempAuto	= (1 << 12);
			static constexpr uint32_t WhiteBalanceCompAuto	= (1 << 13);
			static constexpr uint32_t DigitalMultiplier		= (1 << 14);
			static constexpr uint32_t DigitalMultiplierLimit = (1 << 15);
			static constexpr uint32_t AnalogVideoStandard	= (1 << 16);
			static constexpr uint32_t AnalogVideoLockStatus	= (1 << 17);
			static constexpr uint32_t ContrastAuto			= (1 << 18);
		};

		/// @brief Processing Unit Control Selectors (Used in Setup Packets)
		struct PUSelector {
			static constexpr uint8_t BacklightCompensation  = 0x01;
			static constexpr uint8_t Brightness             = 0x02;
			static constexpr uint8_t Contrast               = 0x03;
			static constexpr uint8_t Gain                   = 0x04;
			static constexpr uint8_t PowerLineFrequency     = 0x05;
			static constexpr uint8_t Hue                    = 0x06;
			static constexpr uint8_t Saturation             = 0x07;
			static constexpr uint8_t Sharpness              = 0x08;
			static constexpr uint8_t Gamma                  = 0x09;
			static constexpr uint8_t WhiteBalanceTemp       = 0x0A;
			static constexpr uint8_t WhiteBalanceComponent  = 0x0B;
			static constexpr uint8_t DigitalMultiplier      = 0x0C;
			static constexpr uint8_t DigitalMultiplierLimit = 0x0D;
			static constexpr uint8_t HueAuto                = 0x0E;
			static constexpr uint8_t WhiteBalanceTempAuto   = 0x0F;
			static constexpr uint8_t WhiteBalanceCompAuto   = 0x10;
			static constexpr uint8_t ContrastAuto           = 0x11;
		};

		/// @brief Input Terminal Controls (Sensor/Lens Adjustments)
		struct ITControl {
			static constexpr uint32_t None					= 0;
			static constexpr uint32_t ScanningMode			= (1 << 0);
			static constexpr uint32_t AutoExposureMode		= (1 << 1);
			static constexpr uint32_t AutoExposurePriority	= (1 << 2);
			static constexpr uint32_t ExposureTimeAbsolute	= (1 << 3);
			static constexpr uint32_t ExposureTimeRelative	= (1 << 4);
			static constexpr uint32_t FocusAbsolute			= (1 << 5);
			static constexpr uint32_t FocusRelative			= (1 << 6);
			static constexpr uint32_t IrisAbsolute			= (1 << 7);
			static constexpr uint32_t IrisRelative			= (1 << 8);
			static constexpr uint32_t ZoomAbsolute			= (1 << 9);
			static constexpr uint32_t ZoomRelative			= (1 << 10);
			static constexpr uint32_t PanTiltAbsolute		= (1 << 11);
			static constexpr uint32_t PanTiltRelative		= (1 << 12);
			static constexpr uint32_t RollAbsolute			= (1 << 13);
			static constexpr uint32_t RollRelative			= (1 << 14);
			static constexpr uint32_t FocusAuto				= (1 << 17);
			static constexpr uint32_t Privacy				= (1 << 18);
			static constexpr uint32_t FocusSimple			= (1 << 19);
			static constexpr uint32_t Window				= (1 << 20);
			static constexpr uint32_t RegionOfInterest		= (1 << 21);
		};

		/// @brief Input Terminal Control Selectors (Used in Setup Packets)
		struct ITSelector {
			static constexpr uint8_t ScanningMode           = 0x01;
			static constexpr uint8_t AutoExposureMode       = 0x02;
			static constexpr uint8_t AutoExposurePriority   = 0x03;
			static constexpr uint8_t ExposureTimeAbsolute   = 0x04;
			static constexpr uint8_t ExposureTimeRelative   = 0x05;
			static constexpr uint8_t FocusAbsolute          = 0x06;
			static constexpr uint8_t FocusRelative          = 0x07;
			static constexpr uint8_t IrisAbsolute           = 0x08;
			static constexpr uint8_t IrisRelative           = 0x09;
			static constexpr uint8_t ZoomAbsolute           = 0x0A;
			static constexpr uint8_t ZoomRelative           = 0x0B;
			static constexpr uint8_t PanTiltAbsolute        = 0x0C;
			static constexpr uint8_t PanTiltRelative        = 0x0D;
			static constexpr uint8_t RollAbsolute           = 0x0E;
			static constexpr uint8_t RollRelative           = 0x0F;
			static constexpr uint8_t FocusAuto              = 0x11;
			static constexpr uint8_t Privacy                = 0x12;
			static constexpr uint8_t FocusSimple            = 0x13;
			static constexpr uint8_t Window                 = 0x14;
			static constexpr uint8_t RegionOfInterest       = 0x15;
		};

		/// @brief UVC Probe/Commit Negotiation Structure.
		struct ProbeCommitControl {
			uint16_t hint;						///< Bitfield indicating which fields the host wants to lock/keep constant during negotiation.
			uint8_t formatIndex;				///< The index of the Video Format Descriptor (e.g., 1 for Uncompressed YUY2, 2 for MJPEG).
			uint8_t frameIndex;					///< The index of the Video Frame Descriptor (e.g., 1 for 640x480, 2 for 1920x1080).
			uint32_t frameInterval;				///< Time between frames in 100ns units. (e.g., 333333 = ~30fps).
			uint16_t keyFrameRate;				///< Key-frame rate for compressed streams. (Usually 0 for uncompressed).
			uint16_t pFrameRate;				///< P-frame rate for compressed streams. (Usually 0 for uncompressed).
			uint16_t compQuality;				///< Target compression quality from 0 to 10000. (Usually 0 for uncompressed).
			uint16_t compWindowSize;			///< Window size for average bit rate control. (Usually 0).
			uint16_t delay;						///< Internal device delay from image capture to USB transmission, in milliseconds.
			uint32_t maxVideoFrameSize;			///< Maximum size of a single video frame in bytes (e.g., Width * Height * BytesPerPixel).
			uint32_t maxPayloadTransferSize;	///< Maximum bytes the device will transmit in a single USB payload/transfer.
		};

		/// @brief UVC 1.0 Class-Specific Request Codes.
		enum UVCRequest {
			SetCurrent = 0x01,
			GetCurrent = 0x81,
			GetMinimum = 0x82,
			GetMaximum = 0x83,
			GetResolution = 0x84,
			GetLength = 0x85,
			GetInformation = 0x86,
			GetDefault = 0x87
		};

		/// @brief Data passed to the application layer when the host requests a camera control.
		struct ControlEvent {
			uint8_t entityId;			///< The target unit (e.g., 0x01 for Input Terminal, 0x02 for Processing Unit)
			uint8_t controlSelector;	///< The control being targeted (e.g., PU_BRIGHTNESS_CONTROL is 0x02)
			UVCRequest requestType;		///< GetCurrent, SetCurrent, GetMinimum, etc.
			uint8_t* payload;			///< Pointer to the driver's internal buffer (read from or write to this)
			uint16_t length;			///< Size of the payload expected by the host
		};

		/// @brief Generic/universal callback function signature for applying requested camera controls, to be handled OUTSIDE the ISR/USB callback
		typedef void (*ApplicationApplyCtrl)(int16_t value);

		/// @brief Generic/universal camera control struct, with all required fields for the UVC Control Handler
		struct UVCControlState {
			const uint8_t selector;		// e.g., 0x02 for Brightness, 0x03 for Contrast
			const uint8_t length;		// Usually 2 bytes for PU controls
			const int16_t min;			// GET_MIN
			const int16_t max;			// GET_MAX
			const int16_t res;			// GET_RES (resolution)
			const int16_t def;			// GET_DEF (default)

			int16_t current;			// GET_CUR (cached current value)
			volatile int16_t target;	// SET_CUR (set in ISR, applied in ApplyControl later)
			volatile bool toUpdate;		// Flag to apply target with ApplyControl outside ISR

			const ApplicationApplyCtrl ApplyControl;	// Apply function to handle/set target value of this control
		};

		/// @brief UVC Control Configuration
		struct ControlConfig {
			uint32_t puControls;		///< Supported/implemented Processing Unit Controls
			uint32_t itControls;		///< Supported/implemented Input Terminal Controls

			UVCControlState* puControlRegistry;		///< Pointer to the application's array of PU controls
			uint8_t numPUControls;       			///< Number of controls in the array

			UVCControlState* itControlRegistry;		///< Pointer to the application's array of IT controls
			uint8_t numITControls;					///< Number of controls in the array
		};

		USBClassUVC();
		virtual ~USBClassUVC() = default;

		/// @brief Initializes the USB UVC Class. Called when the device is configured.
		/// @param bus Passed reference of the low-level bus driver.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid.
		Status Init(USB& usb) override;

		/// @brief Removes the USB UVC Class endpoints.
		/// @param bus Passed reference of the low-level bus driver.
		/// @return Status::Ok if was successful.
		Status DeInit(USB& usb) override;

		/// @brief Called by the core device during registration to allocate interfaces and endpoints.
		/// @param nextInterfaceID	Reference to the device's running interface counter, value updated in this function.
		/// @param nextInEp			Reference to the device's running IN endpoint counter, value updated in this function.
		/// @param nextOutEp		Reference to the device's running OUT endpoint counter, value updated in this function.
		/// @return Status::Ok if was successful.
		Status AssignResource(uint8_t& nextInterfaceID, uint8_t& nextInEp, uint8_t& nextOutEp) override;
		
		/// @brief Checks if this class instance manages the given endpoint address.
		/// @param epAddr Endpoint address/number.
		/// @return True if has endpoint address, False otherwise.
		bool HasEndpoint(uint8_t epAddr) const override;

		/// @brief Checks if this interface ID corresponds to this class instance.
		/// @param interfaceID Interface ID.
		/// @return True if is this interface ID address, False otherwise.
		bool HasInterface(uint8_t interfaceID) const override;
		
		/// @brief Returns the USB Class configuration descriptor, used in the USB Device driver when asked by Host.
		/// @param speed	Passed USB Bus speed by the USB Device class.
		/// @param len		Pointer to be filled with the configuration size, number of bytes.
		/// @return Pointer to the configuration descriptor.
		const uint8_t* GetConfigDescriptor(USB::BusSpeed speed, uint16_t* len) override;
		
		/// @brief Event handler for USB setup.
		/// @param bus		Passed reference of the low-level bus driver.
		/// @param setup	Passed setup package, received from the host.
		/// @return Status::Ok if was successful.
		Status OnSetup(USB& usb, const USB::SetupPacket& setup) override;

		/// @brief Event handler for USB Data In (Isochronous write/transfer finished callback).
		/// @param bus		Passed reference of the low-level bus driver.
		/// @param epNum	Endpoint number.
		/// @return Status::Ok if was successful.
		Status OnDataIn(USB& usb, uint8_t epNum) override;

		/// @brief Event handler for USB Data Out (Isochronous receive callback).
		/// @param bus		Passed reference of the low-level bus driver.
		/// @param epNum	Endpoint number.
		/// @param len		Number of bytes received.
		/// @return Status::Ok if was successful.
		Status OnDataOut(USB& usb, uint8_t epNum, uint32_t len) override;

		/// @brief Configures which camera controls are supported by the hardware.
		/// @param config The control configuration.
		void SetControls(const ControlConfig& config);

		/// @brief Register available video frame formats, to be sent to Host.
		/// @param formatArray	Array of valid frame formats.
		/// @param count		Array size.
		/// @return Status::Ok if was successful.
		Status RegisterFormats(const FrameFormat* formatArray, uint8_t count);

		/// @brief Submit a new video frame to be transmitted.
		/// @param frame	Pointer to the frame.
		/// @return Status::Ok if successfully submitted, Status::Busy if still transmitting previous frame.
		Status SubmitFrame(const VisionFrame* frame);

	private:
		// Dynamically assigned resources
		uint8_t commInterface;
		uint8_t dataInterface;
		uint8_t videoInEp;

		// Device state
		bool isStreaming;
		bool txBusy;
		uint8_t pendingControlSelector;
		uint8_t pendingRequestInterface;
		uint8_t pendingEntityID;
		uint8_t pendingRequestType;
		__attribute__((aligned(32))) uint8_t controlPayload[32]; // Buffer for getting/setting camera control data
		uint8_t frameIdToggle;
		// 26-byte UVC 1.0 Probe/Commit control structure
		__attribute__((aligned(32))) uint8_t probeCommitBytes[64];
		ProbeCommitControl probeCommitControl;

		// Store the negotiated packet size for runtime use
		uint16_t isoMaxPacketSize;		

		// Video configurations
		ControlConfig controls;
		const FrameFormat* frameFormats;
		uint8_t formatCount;

		// Transaction Context
		static constexpr uint32_t maxPacketSize = 512;
		
		const uint8_t* frameBuffer;
		uint32_t frameBufferRemaining;
		__attribute__((aligned(32))) uint8_t activeUVCDescriptor[maxPacketSize];

		// Hardware staging buffer for the payload (Header + Data)
		__attribute__((aligned(32))) uint8_t epInBuffer[maxPacketSize];

		// Internal helpers
		void StartTX();
		void PackProbeCommit();
		void UnpackProbeCommit();
		Status ProcessControlRequest(ControlEvent& event);
};