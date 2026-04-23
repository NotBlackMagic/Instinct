/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/USB/usbClassUVC.cpp
 */

#include "usbClassUVC.hpp"

USBClassUVC::USBClassUVC() {
	this->commInterface = 0xFF;
	this->dataInterface = 0xFF;
	this->videoInEp = 0xFF;

	// Initialize with default 640x480 Uncompressed values
	this->probeCommitControl.hint = 0x0000;
	this->probeCommitControl.formatIndex = 1;
	this->probeCommitControl.frameIndex = 1;
	this->probeCommitControl.frameInterval = 333333;		// 30 fps in 100ns units (0x00051615)
	this->probeCommitControl.keyFrameRate = 0;
	this->probeCommitControl.pFrameRate = 0;
	this->probeCommitControl.compQuality = 0;
	this->probeCommitControl.compWindowSize = 0;
	this->probeCommitControl.delay = 0;
	this->probeCommitControl.maxVideoFrameSize = 614400;	// 640 * 480 * 2 (YUY2 is 16 bpp)
	this->probeCommitControl.maxPayloadTransferSize = 0;

	// Prime the byte array
	this->PackProbeCommit();

	this->isStreaming = false;
	this->txBusy = false;
	this->frameIdToggle = 0;
	this->frameBuffer = nullptr;
	this->frameBufferRemaining = 0;
}

Status USBClassUVC::AssignResource(uint8_t& nextInterfaceID, uint8_t& nextInEp, uint8_t& nextOutEp) {
	(void)nextOutEp;

	commInterface = nextInterfaceID++;
	dataInterface = nextInterfaceID++;

	videoInEp = 0x80 | nextInEp++; 

	return Status::Ok;
}

bool USBClassUVC::HasEndpoint(uint8_t epAddr) const {
	if(epAddr == this->videoInEp) {
		return true;
	}
	return false;
}

bool USBClassUVC::HasInterface(uint8_t interfaceID) const {
	if(interfaceID == this->commInterface || interfaceID == this->dataInterface) {
		return true;
	}
	return false;
}

Status USBClassUVC::Init(USB& usb) {
	this->bus = &usb;

	// Reset buffer state
	this->txBusy = false;
	this->isStreaming = false;
	this->frameIdToggle = 0;

	this->isoMaxPacketSize = (this->bus->GetBusSpeed() == USB::BusSpeed::High) ? 512 : 64;

	return Status::Ok;
}

Status USBClassUVC::DeInit(USB& usb) {
	(void)usb;
	if(this->bus != nullptr) {
		this->bus->CloseEndpoint(this->videoInEp);
		this->bus = nullptr;
	}
	this->isStreaming = false;
	return Status::Ok;
}

const uint8_t* USBClassUVC::GetConfigDescriptor(USB::BusSpeed speed, uint16_t* len) {
	uint16_t offset = 0;

	// Calculate dynamic length of the VS section
	VisionCodec activeCodec = VisionCodec::None;
	PixelFormat activePixelFormat = PixelFormat::Unknown;
	if(this->frameFormats != nullptr && this->formatCount > 0) {
		activeCodec = this->frameFormats[0].codec;
		activePixelFormat = this->frameFormats[0].format;
	}

	// Determine descriptor lengths based on the active codec
	uint8_t formatDescLength = 0;
	uint8_t frameDescLength = 0;
	if(activeCodec == VisionCodec::Jpeg) {
		formatDescLength = 11;
		frameDescLength = 30; // VS_FRAME_MJPEG
	}
	else if(activeCodec == VisionCodec::H264) {
		formatDescLength = 28;	// VS_FORMAT_FRAME_BASED
		frameDescLength = 38;	// VS_FRAME_FRAME_BASED (Larger than standard frames)
	}
	else {
		formatDescLength = 27;	// VS_FORMAT_UNCOMPRESSED
		frameDescLength = 30;	// VS_FRAME_UNCOMPRESSED
	}

	// VS Header (14) + Format + (Frames * Num) + ColorMatching (6)
	uint16_t vsTotalLength = 14 + formatDescLength + (frameDescLength * this->formatCount) + 6;

	// Base UVC Topology (IAD, VC Interface, Terminals)
	// Interface Association Descriptor (IAD)
	this->activeUVCDescriptor[offset++] = 0x08;		// bLength
	this->activeUVCDescriptor[offset++] = 0x0B;		// bDescriptorType (IAD)
	this->activeUVCDescriptor[offset++] = this->commInterface;	// bFirstInterface
	this->activeUVCDescriptor[offset++] = 0x02;		// bInterfaceCount
	this->activeUVCDescriptor[offset++] = 0x0E;		// bFunctionClass (Video)
	this->activeUVCDescriptor[offset++] = 0x03;		// bFunctionSubClass (Video Interface Collection)
	this->activeUVCDescriptor[offset++] = 0x00;		// bFunctionProtocol
	this->activeUVCDescriptor[offset++] = 0x00;		// iFunction

	// Standard VideoControl Interface Descriptor
	this->activeUVCDescriptor[offset++] = 0x09;		// bLength
	this->activeUVCDescriptor[offset++] = 0x04;		// bDescriptorType (Interface)
	this->activeUVCDescriptor[offset++] = this->commInterface;	// bInterfaceNumber
	this->activeUVCDescriptor[offset++] = 0x00;		// bAlternateSetting
	this->activeUVCDescriptor[offset++] = 0x00;		// bNumEndpoints
	this->activeUVCDescriptor[offset++] = 0x0E;		// bInterfaceClass (Video)
	this->activeUVCDescriptor[offset++] = 0x01;		// bInterfaceSubClass (VideoControl)
	this->activeUVCDescriptor[offset++] = 0x00;		// bInterfaceProtocol
	this->activeUVCDescriptor[offset++] = 0x00;		// iInterface

	// Class-specific VideoControl Interface Header Descriptor
	this->activeUVCDescriptor[offset++] = 0x0D;		// bLength
	this->activeUVCDescriptor[offset++] = 0x24;		// bDescriptorType (CS_INTERFACE)
	this->activeUVCDescriptor[offset++] = 0x01;		// bDescriptorSubtype (VC_HEADER)
	this->activeUVCDescriptor[offset++] = 0x10;		// bcdUVC LSB
	this->activeUVCDescriptor[offset++] = 0x01;		// bcdUVC MSB (1.10)
	this->activeUVCDescriptor[offset++] = 0x34;		// wTotalLength LSB
	this->activeUVCDescriptor[offset++] = 0x00;		// wTotalLength MSB
	this->activeUVCDescriptor[offset++] = 0x80;		// dwClockFrequency (Byte 0)
	this->activeUVCDescriptor[offset++] = 0x8D;		// dwClockFrequency (Byte 1)
	this->activeUVCDescriptor[offset++] = 0x5B;		// dwClockFrequency (Byte 2)
	this->activeUVCDescriptor[offset++] = 0x00;		// dwClockFrequency (Byte 3)
	this->activeUVCDescriptor[offset++] = 0x01;		// bInCollection (1 streaming interface)
	this->activeUVCDescriptor[offset++] = this->dataInterface;	// baInterfaceNr(1)

	// Input Terminal Descriptor (Camera Sensor)
	this->activeUVCDescriptor[offset++] = 0x12;		// bLength
	this->activeUVCDescriptor[offset++] = 0x24;		// bDescriptorType (CS_INTERFACE)
	this->activeUVCDescriptor[offset++] = 0x02;		// bDescriptorSubtype (VC_INPUT_TERMINAL)
	this->activeUVCDescriptor[offset++] = 0x01;		// bTerminalID
	this->activeUVCDescriptor[offset++] = 0x01;		// wTerminalType LSB
	this->activeUVCDescriptor[offset++] = 0x02;		// wTerminalType MSB (ITT_CAMERA)
	this->activeUVCDescriptor[offset++] = 0x00;		// bAssocTerminal
	this->activeUVCDescriptor[offset++] = 0x00;		// iTerminal
	this->activeUVCDescriptor[offset++] = 0x00;		// wObjectiveFocalLengthMin LSB
	this->activeUVCDescriptor[offset++] = 0x00;		// wObjectiveFocalLengthMin MSB
	this->activeUVCDescriptor[offset++] = 0x00;		// wObjectiveFocalLengthMax LSB
	this->activeUVCDescriptor[offset++] = 0x00;		// wObjectiveFocalLengthMax MSB
	this->activeUVCDescriptor[offset++] = 0x00;		// wOcularFocalLength LSB
	this->activeUVCDescriptor[offset++] = 0x00;		// wOcularFocalLength MSB
	this->activeUVCDescriptor[offset++] = 0x03;		// bControlSize
	this->activeUVCDescriptor[offset++] = static_cast<uint8_t>(this->controls.itControls & 0xFF);
	this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((this->controls.itControls >> 8) & 0xFF);
	this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((this->controls.itControls >> 16) & 0xFF);

	// Processing Unit Descriptor (Camera ISP)
	this->activeUVCDescriptor[offset++] = 0x0C;		// bLength (12 bytes for UVC 1.1)
	this->activeUVCDescriptor[offset++] = 0x24;		// bDescriptorType (CS_INTERFACE)
	this->activeUVCDescriptor[offset++] = 0x05;		// bDescriptorSubtype (VC_PROCESSING_UNIT)
	this->activeUVCDescriptor[offset++] = 0x02;		// bUnitID (ID 2)
	this->activeUVCDescriptor[offset++] = 0x01;		// bSourceID (Wired to Input Terminal 1)
	this->activeUVCDescriptor[offset++] = 0x00;		// wMaxMultiplier LSB
	this->activeUVCDescriptor[offset++] = 0x00;		// wMaxMultiplier MSB
	this->activeUVCDescriptor[offset++] = 0x02;		// bControlSize (2 bytes)
	this->activeUVCDescriptor[offset++] = static_cast<uint8_t>(this->controls.puControls & 0xFF);
	this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((this->controls.puControls >> 8) & 0xFF);
	this->activeUVCDescriptor[offset++] = 0x00;		// iProcessing (0)
	this->activeUVCDescriptor[offset++] = 0x00;		// bmVideoStandards (0)

	// Output Terminal Descriptor (USB Host)
	this->activeUVCDescriptor[offset++] = 0x09;		// bLength
	this->activeUVCDescriptor[offset++] = 0x24;		// bDescriptorType (CS_INTERFACE)
	this->activeUVCDescriptor[offset++] = 0x03;		// bDescriptorSubtype (VC_OUTPUT_TERMINAL)
	this->activeUVCDescriptor[offset++] = 0x03;		// bTerminalID
	this->activeUVCDescriptor[offset++] = 0x01;		// wTerminalType LSB
	this->activeUVCDescriptor[offset++] = 0x01;		// wTerminalType MSB (TT_STREAMING)
	this->activeUVCDescriptor[offset++] = 0x00;		// bAssocTerminal
	this->activeUVCDescriptor[offset++] = 0x02;		// bSourceID (Wired to Processing Unit 2)
	this->activeUVCDescriptor[offset++] = 0x00;		// iTerminal

	// Standard VideoStreaming Interface Descriptor
	this->activeUVCDescriptor[offset++] = 0x09;		// bLength
	this->activeUVCDescriptor[offset++] = 0x04;		// bDescriptorType (Interface)
	this->activeUVCDescriptor[offset++] = this->dataInterface;	// bInterfaceNumber
	this->activeUVCDescriptor[offset++] = 0x00;		// bAlternateSetting
	this->activeUVCDescriptor[offset++] = 0x00;		// bNumEndpoints (0 for Isochronous)
	this->activeUVCDescriptor[offset++] = 0x0E;		// bInterfaceClass (Video)
	this->activeUVCDescriptor[offset++] = 0x02;		// bInterfaceSubClass (VideoStreaming)
	this->activeUVCDescriptor[offset++] = 0x00;		// bInterfaceProtocol
	this->activeUVCDescriptor[offset++] = 0x00;		// iInterface

	// Video Streaming Format Header
	this->activeUVCDescriptor[offset++] = 0x0E;		// bLength
	this->activeUVCDescriptor[offset++] = 0x24;		// bDescriptorType (CS_INTERFACE)
	this->activeUVCDescriptor[offset++] = 0x01;		// bDescriptorSubtype (VS_INPUT_HEADER)
	this->activeUVCDescriptor[offset++] = 0x01;		// bNumFormats
	this->activeUVCDescriptor[offset++] = static_cast<uint8_t>(vsTotalLength & 0xFF);			// wTotalLength LSB
	this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((vsTotalLength >> 8) & 0xFF);	// wTotalLength MSB
	this->activeUVCDescriptor[offset++] = this->videoInEp;	// bEndpointAddress
	this->activeUVCDescriptor[offset++] = 0x00;		// bmInfo
	this->activeUVCDescriptor[offset++] = 0x03;		// bTerminalLink (To Output Terminal 2)
	this->activeUVCDescriptor[offset++] = 0x00;		// bStillCaptureMethod
	this->activeUVCDescriptor[offset++] = 0x00;		// bTriggerSupport
	this->activeUVCDescriptor[offset++] = 0x00;		// bTriggerUsage
	this->activeUVCDescriptor[offset++] = 0x01;		// bControlSize
	this->activeUVCDescriptor[offset++] = 0x00;		// bmaControls(1)

	// Dynamic Format Descriptor
	if(activeCodec == VisionCodec::Jpeg) {
		this->activeUVCDescriptor[offset++] = 11;		// bLength
		this->activeUVCDescriptor[offset++] = 0x24;		// bDescriptorType (CS_INTERFACE)
		this->activeUVCDescriptor[offset++] = 0x06;		// bDescriptorSubtype (VS_FORMAT_MJPEG)
		this->activeUVCDescriptor[offset++] = 0x01;		// bFormatIndex
		this->activeUVCDescriptor[offset++] = this->formatCount; // bNumFrameDescriptors
		this->activeUVCDescriptor[offset++] = 0x01;		// bmFlags (Fixed size samples)
		this->activeUVCDescriptor[offset++] = 0x01;		// bDefaultFrameIndex
		this->activeUVCDescriptor[offset++] = 0x00;		// bAspectRatioX
		this->activeUVCDescriptor[offset++] = 0x00;		// bAspectRatioY
		this->activeUVCDescriptor[offset++] = 0x00;		// bmInterlaceFlags
		this->activeUVCDescriptor[offset++] = 0x00;		// bCopyProtect
	} 
	else if(activeCodec == VisionCodec::H264) {
		// UVC 1.1 Frame-Based Format (Typical for custom H.264 streams)
		this->activeUVCDescriptor[offset++] = 28;		// bLength
		this->activeUVCDescriptor[offset++] = 0x24;		// bDescriptorType
		this->activeUVCDescriptor[offset++] = 0x10;		// bDescriptorSubtype (VS_FORMAT_FRAME_BASED)
		this->activeUVCDescriptor[offset++] = 0x01;		// bFormatIndex
		this->activeUVCDescriptor[offset++] = this->formatCount;

		// guidFormat (H264 GUID: "H264" followed by standard suffix)
		this->activeUVCDescriptor[offset++] = 0x48;
		this->activeUVCDescriptor[offset++] = 0x32;
		this->activeUVCDescriptor[offset++] = 0x36;
		this->activeUVCDescriptor[offset++] = 0x34;
		this->activeUVCDescriptor[offset++] = 0x00;
		this->activeUVCDescriptor[offset++] = 0x00;
		this->activeUVCDescriptor[offset++] = 0x10;
		this->activeUVCDescriptor[offset++] = 0x00;
		this->activeUVCDescriptor[offset++] = 0x80;
		this->activeUVCDescriptor[offset++] = 0x00;
		this->activeUVCDescriptor[offset++] = 0x00;
		this->activeUVCDescriptor[offset++] = 0xAA;
		this->activeUVCDescriptor[offset++] = 0x00;
		this->activeUVCDescriptor[offset++] = 0x38;
		this->activeUVCDescriptor[offset++] = 0x9B;
		this->activeUVCDescriptor[offset++] = 0x71;

		this->activeUVCDescriptor[offset++] = 16;		// bBitsPerPixel (Used for bandwidth calculation)
		this->activeUVCDescriptor[offset++] = 0x01;		// bDefaultFrameIndex
		this->activeUVCDescriptor[offset++] = 0x00;		// bAspectRatioX
		this->activeUVCDescriptor[offset++] = 0x00;		// bAspectRatioY
		this->activeUVCDescriptor[offset++] = 0x00;		// bmInterlaceFlags
		this->activeUVCDescriptor[offset++] = 0x00;		// bCopyProtect
		this->activeUVCDescriptor[offset++] = 0x00;		// bVariableSize (0 = fixed frame interval)
	} 
	else {
		// Uncompressed Format (YUY2, NV12)
		this->activeUVCDescriptor[offset++] = 27;		// bLength
		this->activeUVCDescriptor[offset++] = 0x24;		// bDescriptorType
		this->activeUVCDescriptor[offset++] = 0x04;		// bDescriptorSubtype (VS_FORMAT_UNCOMPRESSED)
		this->activeUVCDescriptor[offset++] = 0x01;		// bFormatIndex
		this->activeUVCDescriptor[offset++] = this->formatCount;

		if(activePixelFormat == PixelFormat::YUV422_YUYV) {
			this->activeUVCDescriptor[offset++] = 0x4E;
			this->activeUVCDescriptor[offset++] = 0x56;
			this->activeUVCDescriptor[offset++] = 0x31;
			this->activeUVCDescriptor[offset++] = 0x32;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x10;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x80;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0xAA;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x38;
			this->activeUVCDescriptor[offset++] = 0x9B;
			this->activeUVCDescriptor[offset++] = 0x71;
		}
		else if(activePixelFormat == PixelFormat::YUV422_YVYU) {
			this->activeUVCDescriptor[offset++] = 0x59;	// 'Y'
			this->activeUVCDescriptor[offset++] = 0x56;	// 'V'
			this->activeUVCDescriptor[offset++] = 0x59;	// 'Y'
			this->activeUVCDescriptor[offset++] = 0x55;	// 'U'
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x10;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x80;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0xAA;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x38;
			this->activeUVCDescriptor[offset++] = 0x9B;
			this->activeUVCDescriptor[offset++] = 0x71;
		}
		else {
			// Default to YUY2
			this->activeUVCDescriptor[offset++] = 0x59;	// 'Y'
			this->activeUVCDescriptor[offset++] = 0x55;	// 'U'
			this->activeUVCDescriptor[offset++] = 0x59;	// 'Y'
			this->activeUVCDescriptor[offset++] = 0x32;	// '2'
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x10;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x80;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0xAA;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x38;
			this->activeUVCDescriptor[offset++] = 0x9B;
			this->activeUVCDescriptor[offset++] = 0x71;
		}

		this->activeUVCDescriptor[offset++] = (activePixelFormat == PixelFormat::YUV420_NV12) ? 12 : 16; // bBitsPerPixel
		this->activeUVCDescriptor[offset++] = 0x01;		// bDefaultFrameIndex
		this->activeUVCDescriptor[offset++] = 0x00;		// bAspectRatioX
		this->activeUVCDescriptor[offset++] = 0x00;		// bAspectRatioY
		this->activeUVCDescriptor[offset++] = 0x00;		// bmInterlaceFlags
		this->activeUVCDescriptor[offset++] = 0x00;		// bCopyProtect
	}

	// Dynamically Append User Frames inline
	if(this->frameFormats != nullptr) {
		for(uint8_t i = 0; i < this->formatCount; i++) {
			// Prevent buffer overflow! (30 bytes per frame + 13 bytes for footer)
			if((offset + 30 + 13) >= sizeof(this->activeUVCDescriptor)) {
				break; 
			}

			uint8_t frameIndex = i + 1; // UVC uses 1-based indexing
			const FrameFormat& frame = this->frameFormats[i];

			// Determine correct Subtype
			uint8_t frameSubtype = 0x05; // VS_FRAME_UNCOMPRESSED
			if(activeCodec == VisionCodec::Jpeg) {
				frameSubtype = 0x07;	// VS_FRAME_MJPEG
			}
			else if(activeCodec == VisionCodec::H264) {
				frameSubtype = 0x11;	// VS_FRAME_FRAME_BASED
			}
			
			// Assume 16 bits per pixel (YUY2) for max frame size calculation
			uint8_t bytesPerPixel = (frame.format == PixelFormat::YUV420_NV12) ? 1 : 2;
			uint32_t maxFrameSize = frame.width * frame.height * bytesPerPixel;

			this->activeUVCDescriptor[offset++] = 30;			// bLength
			this->activeUVCDescriptor[offset++] = 0x24;			// bDescriptorType
			this->activeUVCDescriptor[offset++] = frameSubtype;	// bDescriptorSubtype
			this->activeUVCDescriptor[offset++] = frameIndex;	// bFrameIndex
			this->activeUVCDescriptor[offset++] = 0x00;			// bmCapabilities
			
			// wWidth
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>(frame.width & 0xFF);
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((frame.width >> 8) & 0xFF);
			// wHeight
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>(frame.height & 0xFF);
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((frame.height >> 8) & 0xFF);
			
			// dwMinBitRate (Dummy for uncompressed)
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x94;
			this->activeUVCDescriptor[offset++] = 0x11;
			// dwMaxBitRate (Dummy for uncompressed)
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x00;
			this->activeUVCDescriptor[offset++] = 0x94;
			this->activeUVCDescriptor[offset++] = 0x11;
			
			// dwMaxVideoFrameBufferSize
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>(maxFrameSize & 0xFF);
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((maxFrameSize >> 8) & 0xFF);
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((maxFrameSize >> 16) & 0xFF);
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((maxFrameSize >> 24) & 0xFF);
			
			// dwDefaultFrameInterval
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>(frame.frameInterval & 0xFF);
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((frame.frameInterval >> 8) & 0xFF);
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((frame.frameInterval >> 16) & 0xFF);
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((frame.frameInterval >> 24) & 0xFF);
			
			this->activeUVCDescriptor[offset++] = 0x01;	// bFrameIntervalType (1 discrete rate)

			if(activeCodec == VisionCodec::H264) {
				// Frame-Based requires dwBytesPerLine (dummy value for H264)
				this->activeUVCDescriptor[offset++] = 0x00; this->activeUVCDescriptor[offset++] = 0x00;
				this->activeUVCDescriptor[offset++] = 0x00; this->activeUVCDescriptor[offset++] = 0x00;
			}
			
			// dwFrameInterval
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>(frame.frameInterval & 0xFF);
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((frame.frameInterval >> 8) & 0xFF);
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((frame.frameInterval >> 16) & 0xFF);
			this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((frame.frameInterval >> 24) & 0xFF);
		}
	}

	// Endpoint Footer

	// Color matching descriptor
	this->activeUVCDescriptor[offset++] = 0x06;		// bLength
	this->activeUVCDescriptor[offset++] = 0x24;		// bDescriptorType (CS_INTERFACE)
	this->activeUVCDescriptor[offset++] = 0x0D;		// bDescriptorSubtype (VS_COLORFORMAT)
	this->activeUVCDescriptor[offset++] = 0x01;		// bColorPrimaries
	this->activeUVCDescriptor[offset++] = 0x01;		// bTransferCharacteristics
	this->activeUVCDescriptor[offset++] = 0x04;		// bMatrixCoefficients

	// Standard VideoStreaming Interface Descriptor (Alt Setting 1 - Active Streaming)
	this->activeUVCDescriptor[offset++] = 0x09;
	this->activeUVCDescriptor[offset++] = 0x04;
	this->activeUVCDescriptor[offset++] = this->dataInterface;
	this->activeUVCDescriptor[offset++] = 0x01;		// bAlternateSetting 1
	this->activeUVCDescriptor[offset++] = 0x01;		// bNumEndpoints (1 for Isochronous)
	this->activeUVCDescriptor[offset++] = 0x0E;
	this->activeUVCDescriptor[offset++] = 0x02;
	this->activeUVCDescriptor[offset++] = 0x00;
	this->activeUVCDescriptor[offset++] = 0x00;

	// Standard Isochronous Endpoint Descriptor
	this->activeUVCDescriptor[offset++] = 0x07;
	this->activeUVCDescriptor[offset++] = 0x05;
	this->activeUVCDescriptor[offset++] = this->videoInEp;
	this->activeUVCDescriptor[offset++] = 0x05;		// bmAttributes (0x01 Isochronous | 0x04 Asynchronous)

	// wMaxPacketSize
	this->isoMaxPacketSize = (speed == USB::BusSpeed::High) ? 512 : 64;
	this->activeUVCDescriptor[offset++] = static_cast<uint8_t>(this->isoMaxPacketSize & 0xFF);
	this->activeUVCDescriptor[offset++] = static_cast<uint8_t>((this->isoMaxPacketSize >> 8) & 0xFF);

	this->activeUVCDescriptor[offset++] = 0x01;		// bInterval (1 microframe for High Speed)

	// Assign length if requested by the core USB driver
	if(len != nullptr) {
		*len = offset; 
	}

	return this->activeUVCDescriptor;
}

Status USBClassUVC::OnSetup(USB& usb, const USB::SetupPacket& setup) {
	(void)usb;

	LOG_INFO("SETUP: RT:%02X REQ:%02X VAL:%04X IDX:%04X LEN:%d", setup.requestType, setup.request, setup.value, setup.index, setup.length);

	uint8_t requestTypeMask = setup.requestType & 0x60;
	uint8_t recipientMask = setup.requestType & 0x1F;
	if(requestTypeMask == 0x00 && recipientMask == 0x01) {
		// Handle Standard Requests (Type 0x00)
		if(setup.request == 0x0B && (setup.index & 0xFF) == this->dataInterface) {
			// SET_INTERFACE handling
			uint8_t altSetting = setup.value & 0xFF;
			if(altSetting == 1) {
				// The host switched to Alt 1: Start pumping data
				this->isStreaming = true;
				this->txBusy = false;
				this->frameIdToggle = 0;
				this->bus->OpenEndpoint(this->videoInEp, USB::EndpointType::Isochronous, this->isoMaxPacketSize);

				// Push the first empty header to start the interrupts
				this->StartTX();
			} 
			else if(altSetting == 0) {
				// The host switched to Alt 0: Stop the stream
				this->isStreaming = false;
				this->txBusy = false;
				this->frameBufferRemaining = 0;
				this->frameBuffer = nullptr;
				this->frameIdToggle = 0;
				this->bus->CloseEndpoint(this->videoInEp);
			}
			// Acknowledge with ZLP
			return this->bus->Transmit(0x80, nullptr, 0);
		}
		else if(setup.request == 0x0A) {
			// GET_INTERFACE handling
			uint8_t targetInterface = setup.index & 0xFF;
			if(targetInterface == this->dataInterface) {
				// Return 1 if streaming, 0 if halted
				this->probeCommitBytes[0] = this->isStreaming ? 1 : 0;
				this->bus->Receive(0x00, nullptr, 0);
				return this->bus->Transmit(0x80, this->probeCommitBytes, 1);
			}
			else if(targetInterface == this->commInterface) {
				// Video Control Interface only ever operates on AltSetting 0
				this->probeCommitBytes[0] = 0; 
				this->bus->Receive(0x00, nullptr, 0);
				return this->bus->Transmit(0x80, this->probeCommitBytes, 1);
			}
		}
	}
	else if(requestTypeMask == 0x20 && recipientMask == 0x01) {
		// Handle Class-Specific Requests (Type 0x20)
		uint8_t interfaceNum = setup.index & 0xFF;
		uint8_t entityId = (setup.index >> 8) & 0xFF;
		uint8_t controlSelector = (setup.value >> 8) & 0xFF;

		// Store routing info for potential DataOut phase
		this->pendingRequestInterface = interfaceNum;
		this->pendingEntityID = entityId;
		this->pendingControlSelector = controlSelector;
		this->pendingRequestType = setup.request;

		if(interfaceNum == this->dataInterface) {
			if(controlSelector == 0x01 || controlSelector == 0x02) {
				// Video streaming controls/setup (Probe/Commit Negotiation)
				switch(setup.request) {
					case UVCRequest::SetCurrent:
						// Prepare Ep0 to receive host configuration into our byte array
						return this->bus->Receive(0x00, this->probeCommitBytes, setup.length);
					case UVCRequest::GetCurrent:
					case UVCRequest::GetMinimum:
					case UVCRequest::GetMaximum:
					case UVCRequest::GetDefault:
						// Make sure the byte array is up-to-date with our struct, then send
						this->PackProbeCommit(); 
						this->bus->Receive(0x00, nullptr, 0); 
						return this->bus->Transmit(0x80, this->probeCommitBytes, 34);
					case UVCRequest::GetInformation:
						// Windows asks: "What can I do with this control?"
						// Bit 0 = Supports GET, Bit 1 = Supports SET. (0x03 means both).
						this->probeCommitBytes[0] = 0x03;
						this->bus->Receive(0x00, nullptr, 0); 
						return this->bus->Transmit(0x80, this->probeCommitBytes, 1);
					case UVCRequest::GetLength:
						// Windows asks: "How many bytes is your Probe/Commit structure?"
						// We must reply with exactly 34 (0x0022 in little-endian).
						this->probeCommitBytes[0] = 34;		// 0x22
						this->probeCommitBytes[1] = 0x00;	// 0x00
						this->bus->Receive(0x00, nullptr, 0); 
						return this->bus->Transmit(0x80, this->probeCommitBytes, 2);
					default:
						break;
				}
			}
		}
		else if(interfaceNum == this->commInterface) {
			// Video control/setup (camera adjustments)
			if(setup.request == static_cast<uint8_t>(UVCRequest::SetCurrent)) {
				// Set request, get request data/payload from host
				return this->bus->Receive(0x00, this->controlPayload, setup.length);
			} 
			else {
				// Get request, get requested data from the camera layer through the callback
				ControlEvent event;
				event.entityId = entityId;
				event.controlSelector = controlSelector;
				event.requestType = static_cast<UVCRequest>(setup.request);
				event.payload = this->controlPayload;
				event.length = setup.length;

				Status status = this->ProcessControlRequest(event);
				if(status == Status::Ok) {
					this->bus->Receive(0x00, nullptr, 0); 
					return this->bus->Transmit(0x80, this->controlPayload, setup.length);
				}
			}
		}
	}

	return Status::Error; // Unknown or unhandled request
}

Status USBClassUVC::OnDataIn(USB& usb, uint8_t epNum) {
	(void)usb;

	if(epNum == (videoInEp & 0x7F)) {
		this->txBusy = false;
		
		// Push the next chunk of the current frame
		this->StartTX();
	}

	return Status::Ok;
}

Status USBClassUVC::OnDataOut(USB& usb, uint8_t epNum, uint32_t len) {
	(void)usb;

	if(epNum == 0x00) {
		if(this->pendingRequestInterface == this->dataInterface) {
			if(len == 34) {
				// Video streaming controls/setup (Probe/Commit Negotiation). Unpack the received bytes into local struct
				this->UnpackProbeCommit();

				if(this->pendingControlSelector == 0x01) { 
					// Probe phase: validation and negotiation

					// Validate Format Index
					if(this->probeCommitControl.formatIndex != 1) {
						this->probeCommitControl.formatIndex = 1; 
					}

					// Validate Frame Index
					uint8_t reqFrameIndex = this->probeCommitControl.frameIndex;
					
					// If the host asks for an index out of bounds, forcefully set it to the first resolution
					if(reqFrameIndex < 1 || reqFrameIndex > this->formatCount) {
						reqFrameIndex = 1;
						this->probeCommitControl.frameIndex = reqFrameIndex;
					}

					// Fetch the matched capability from your registered vision types
					const FrameFormat& activeCap = this->frameFormats[reqFrameIndex - 1];

					// Validate Frame Interval (Framerate): Force the negotiated interval to match what you registered
					this->probeCommitControl.frameInterval = activeCap.frameInterval;

					// Calculate hardware requirements dynamically based on the matched setting
					uint8_t bytesPerPixel = (activeCap.format == PixelFormat::YUV420_NV12) ? 1 : 2;
					this->probeCommitControl.maxVideoFrameSize = activeCap.width * activeCap.height * bytesPerPixel; 
					
					// Tell the host what size of payload chunks it should expect.
					this->probeCommitControl.maxPayloadTransferSize = this->isoMaxPacketSize;

				} 
				else if(this->pendingControlSelector == 0x02) { 
					// Commit phase: The host accepted Probe modifications, settings are locked in
					this->frameIdToggle = 0;
					this->txBusy = false;
				}

				// Acknowledge the settings 
				this->bus->Transmit(0x80, nullptr, 0);
			}
		}
		else if(this->pendingRequestInterface == this->commInterface) {
			// Video control/setup (camera adjustments)
			// ONLY process if the original Setup packet was a SetCurrent command
			if(this->pendingRequestType == static_cast<uint8_t>(UVCRequest::SetCurrent)) {
				ControlEvent event;
				event.entityId = this->pendingEntityID;
				event.controlSelector = this->pendingControlSelector;
				event.requestType = UVCRequest::SetCurrent;
				event.payload = this->controlPayload;
				event.length = len;

				Status status = this->ProcessControlRequest(event);
				if(status == Status::Ok) {
					// Acknowledge the successful control update
					return this->bus->Transmit(0x80, nullptr, 0); 
				}
				
				// If the application rejected the value or callback is null, return Error to stall
				return Status::Error;
			}
			else {
				// Was just a ZLP for ACK of a GET request, ignore.
				return Status::Ok;
			}
		}
	}

	return Status::Ok;
}

void USBClassUVC::SetControls(const ControlConfig& config) {
	this->controls = config;
}

Status USBClassUVC::RegisterFormats(const FrameFormat* formatArray, uint8_t count) {
	if(formatArray == nullptr || count == 0) {
		return Status::Error;
	}
	this->frameFormats = formatArray;
	this->formatCount = count;
	return Status::Ok;
}

Status USBClassUVC::SubmitFrame(const VisionFrame* frame) {
	if(this->bus == nullptr || this->isStreaming == false) {
		return Status::Error;
	}

	// Prevent overwriting a frame that is currently in progress
	if(this->frameBufferRemaining > 0) {
		return Status::Busy;
	}

	// Validate the incoming frame token
	if(frame == nullptr || frame->startAddress == nullptr || frame->payloadSize == 0) {
		return Status::Error;
	}

	// Set up the tracking variables for the new frame
	this->frameBuffer = frame->startAddress;
	this->frameBufferRemaining = frame->payloadSize;

	// Kickstart the transmission pipeline
	this->StartTX();

	return Status::Ok;
}

void USBClassUVC::StartTX() {
	if(this->bus == nullptr || this->txBusy == true || this->isStreaming == false) {
		return;
	}

	// Minimum UVC Payload Header length
	uint8_t headerLength = 2; 

	// Header Bitfield Configuration
	// Bit 0: FID (Frame ID) - Toggles per frame
	// Bit 1: EOF (End of Frame) - Set on the very last packet
	// Bit 7: EOH (End of Header) - Always set to 1 for standard compliance
	uint8_t headerInfo = 0x80; 
	if(this->frameIdToggle != 0) {
		headerInfo |= 0x01; // Set the FID bit
	}

	// Handle the Idle / Starved state (in Isochronous mode, something has to be sent always!)
	if(this->frameBufferRemaining == 0) {
		this->epInBuffer[0] = headerLength;
		this->epInBuffer[1] = headerInfo;
		this->txBusy = true;
		this->bus->Transmit(this->videoInEp, this->epInBuffer, headerLength);
		return;
	}

	// Calculate how much payload data we can fit in this single hardware packet
	uint32_t maxPayloadSize = this->isoMaxPacketSize - headerLength;
	uint32_t bytesToSend = this->frameBufferRemaining;
	bool isLastPacket = false;

	if(bytesToSend > maxPayloadSize) {
		bytesToSend = maxPayloadSize;
	} 
	else {
		// This packet will finish the current frame
		isLastPacket = true;
		headerInfo |= 0x02; // Set the EOF bit
	}

	// Pack UVC Payload Header
	this->epInBuffer[0] = headerLength;
	this->epInBuffer[1] = headerInfo;

	// Copy video payload
	memcpy(&this->epInBuffer[headerLength], this->frameBuffer, bytesToSend);

	// Advance the frame tracking pointers
	this->frameBuffer += bytesToSend;
	this->frameBufferRemaining -= bytesToSend;

	// If last packet, toggle FID for the next frame
	if(isLastPacket == true) {
		this->frameIdToggle = (this->frameIdToggle == 0) ? 1 : 0;
	}

	// Lock the pipeline and hand the buffer to driver
	this->txBusy = true;
	uint32_t totalBytes = bytesToSend + headerLength;
	this->bus->Transmit(this->videoInEp, this->epInBuffer, totalBytes);
}

Status USBClassUVC::ProcessControlRequest(USBClassUVC::ControlEvent& event) {
	// Handle Controls
	if(event.entityId == 0x00) {
		// VC_REQUEST_ERROR_CODE_CONTROL (Windows asking "Why did you stall?")
		if(event.requestType == USBClassUVC::UVCRequest::GetCurrent) {
			event.payload[0] = 0x00; // 0x00 = "No Error"
			return Status::Ok;
		}
		return Status::Error;
	}

	// Select the correct registry based on Entity ID
	UVCControlState* activeRegistry = nullptr;
	uint8_t registrySize = 0;
	if(event.entityId == 0x01) {
		// Entity 1: Input Terminal
		activeRegistry = this->controls.itControlRegistry;
		registrySize = this->controls.numITControls;
	}
	else if(event.entityId == 0x02) {
		// Entity 2: Processing Unit
		activeRegistry = this->controls.puControlRegistry;
		registrySize = this->controls.numPUControls;
	}
	else {
		return Status::Error;
	}

	if(activeRegistry == nullptr || registrySize == 0) {
		return Status::Error;
	}

	// Find the requested control in the registry
	UVCControlState* activeCtrl = nullptr;
	for(uint8_t i = 0; i < registrySize; i++) {
		if(activeRegistry[i].selector == event.controlSelector) {
			activeCtrl = &activeRegistry[i];
			break;
		}
	}

	// If selector isn't in our array, reject it (STALL)
	if(activeCtrl == nullptr) {
		return Status::Error;
	}

	int16_t* value = reinterpret_cast<int16_t*>(event.payload);

	switch(event.requestType) {
		case USBClassUVC::UVCRequest::SetCurrent:
			// Set new value command
			activeCtrl->target = *value;
			activeCtrl->toUpdate = true;
			return Status::Ok;
		case USBClassUVC::UVCRequest::GetCurrent:
			// Get current value
			*value = activeCtrl->current;
			return Status::Ok;
		case USBClassUVC::UVCRequest::GetMinimum:
			// Get lower control limit (minimum allowed value)
			*value = activeCtrl->min;
			return Status::Ok;
		case USBClassUVC::UVCRequest::GetMaximum:
			// Get higher control limit (maximum allowed value)
			*value = activeCtrl->max;
			return Status::Ok;
		case USBClassUVC::UVCRequest::GetResolution:
			// Get control step size
			*value = activeCtrl->res;
			return Status::Ok;
		case USBClassUVC::UVCRequest::GetLength:
			// Get size/length of control field
			*value = activeCtrl->length;
			return Status::Ok;
		case USBClassUVC::UVCRequest::GetDefault:
			// Get default control value
			*value = activeCtrl->def;
			return Status::Ok;
		case USBClassUVC::UVCRequest::GetInformation:
			// Get what is supported (getter, setter, both)
			event.payload[0] = 0x03;
			return Status::Ok;
		default:
			return Status::Error;
	}

	// If reached here, unsupported control
	return Status::Error;
}

void USBClassUVC::PackProbeCommit() {
	this->probeCommitBytes[0] = static_cast<uint8_t>(this->probeCommitControl.hint & 0xFF);
	this->probeCommitBytes[1] = static_cast<uint8_t>((this->probeCommitControl.hint >> 8) & 0xFF);

	this->probeCommitBytes[2] = this->probeCommitControl.formatIndex;
	this->probeCommitBytes[3] = this->probeCommitControl.frameIndex;

	this->probeCommitBytes[4] = static_cast<uint8_t>(this->probeCommitControl.frameInterval & 0xFF);
	this->probeCommitBytes[5] = static_cast<uint8_t>((this->probeCommitControl.frameInterval >> 8) & 0xFF);
	this->probeCommitBytes[6] = static_cast<uint8_t>((this->probeCommitControl.frameInterval >> 16) & 0xFF);
	this->probeCommitBytes[7] = static_cast<uint8_t>((this->probeCommitControl.frameInterval >> 24) & 0xFF);

	this->probeCommitBytes[8] = static_cast<uint8_t>(this->probeCommitControl.keyFrameRate & 0xFF);
	this->probeCommitBytes[9] = static_cast<uint8_t>((this->probeCommitControl.keyFrameRate >> 8) & 0xFF);

	this->probeCommitBytes[10] = static_cast<uint8_t>(this->probeCommitControl.pFrameRate & 0xFF);
	this->probeCommitBytes[11] = static_cast<uint8_t>((this->probeCommitControl.pFrameRate >> 8) & 0xFF);

	this->probeCommitBytes[12] = static_cast<uint8_t>(this->probeCommitControl.compQuality & 0xFF);
	this->probeCommitBytes[13] = static_cast<uint8_t>((this->probeCommitControl.compQuality >> 8) & 0xFF);

	this->probeCommitBytes[14] = static_cast<uint8_t>(this->probeCommitControl.compWindowSize & 0xFF);
	this->probeCommitBytes[15] = static_cast<uint8_t>((this->probeCommitControl.compWindowSize >> 8) & 0xFF);

	this->probeCommitBytes[16] = static_cast<uint8_t>(this->probeCommitControl.delay & 0xFF);
	this->probeCommitBytes[17] = static_cast<uint8_t>((this->probeCommitControl.delay >> 8) & 0xFF);

	this->probeCommitBytes[18] = static_cast<uint8_t>(this->probeCommitControl.maxVideoFrameSize & 0xFF);
	this->probeCommitBytes[19] = static_cast<uint8_t>((this->probeCommitControl.maxVideoFrameSize >> 8) & 0xFF);
	this->probeCommitBytes[20] = static_cast<uint8_t>((this->probeCommitControl.maxVideoFrameSize >> 16) & 0xFF);
	this->probeCommitBytes[21] = static_cast<uint8_t>((this->probeCommitControl.maxVideoFrameSize >> 24) & 0xFF);

	this->probeCommitBytes[22] = static_cast<uint8_t>(this->probeCommitControl.maxPayloadTransferSize & 0xFF);
	this->probeCommitBytes[23] = static_cast<uint8_t>((this->probeCommitControl.maxPayloadTransferSize >> 8) & 0xFF);
	this->probeCommitBytes[24] = static_cast<uint8_t>((this->probeCommitControl.maxPayloadTransferSize >> 16) & 0xFF);
	this->probeCommitBytes[25] = static_cast<uint8_t>((this->probeCommitControl.maxPayloadTransferSize >> 24) & 0xFF);

	this->probeCommitBytes[26] = 0x00; // bmFramingInfo
	this->probeCommitBytes[27] = 0x00; // bPreferedVersion
	this->probeCommitBytes[28] = 0x00; // bMinVersion
	this->probeCommitBytes[29] = 0x00; // bMaxVersion
	this->probeCommitBytes[30] = 0x00; // bUsage
	this->probeCommitBytes[31] = 0x00; // bBitDepthLuma
	this->probeCommitBytes[32] = 0x00; // bmSettings
	this->probeCommitBytes[33] = 0x00; // bMaxNumberOfRefFramesPlus1
}

void USBClassUVC::UnpackProbeCommit() {
	this->probeCommitControl.hint = static_cast<uint16_t>(this->probeCommitBytes[0]) | 
								(static_cast<uint16_t>(this->probeCommitBytes[1]) << 8);
								
	this->probeCommitControl.formatIndex = this->probeCommitBytes[2];
	this->probeCommitControl.frameIndex = this->probeCommitBytes[3];

	this->probeCommitControl.frameInterval = static_cast<uint32_t>(this->probeCommitBytes[4]) | 
										(static_cast<uint32_t>(this->probeCommitBytes[5]) << 8) | 
										(static_cast<uint32_t>(this->probeCommitBytes[6]) << 16) | 
										(static_cast<uint32_t>(this->probeCommitBytes[7]) << 24);
										
	this->probeCommitControl.keyFrameRate = static_cast<uint16_t>(this->probeCommitBytes[8]) | 
										(static_cast<uint16_t>(this->probeCommitBytes[9]) << 8);
										
	this->probeCommitControl.pFrameRate = static_cast<uint16_t>(this->probeCommitBytes[10]) | 
									(static_cast<uint16_t>(this->probeCommitBytes[11]) << 8);
									
	this->probeCommitControl.compQuality = static_cast<uint16_t>(this->probeCommitBytes[12]) | 
										(static_cast<uint16_t>(this->probeCommitBytes[13]) << 8);
										
	this->probeCommitControl.compWindowSize = static_cast<uint16_t>(this->probeCommitBytes[14]) | 
										(static_cast<uint16_t>(this->probeCommitBytes[15]) << 8);
										
	this->probeCommitControl.delay = static_cast<uint16_t>(this->probeCommitBytes[16]) | 
								(static_cast<uint16_t>(this->probeCommitBytes[17]) << 8);
								
	this->probeCommitControl.maxVideoFrameSize = static_cast<uint32_t>(this->probeCommitBytes[18]) | 
											(static_cast<uint32_t>(this->probeCommitBytes[19]) << 8) | 
											(static_cast<uint32_t>(this->probeCommitBytes[20]) << 16) | 
											(static_cast<uint32_t>(this->probeCommitBytes[21]) << 24);
											
	this->probeCommitControl.maxPayloadTransferSize = static_cast<uint32_t>(this->probeCommitBytes[22]) | 
												(static_cast<uint32_t>(this->probeCommitBytes[23]) << 8) | 
												(static_cast<uint32_t>(this->probeCommitBytes[24]) << 16) | 
												(static_cast<uint32_t>(this->probeCommitBytes[25]) << 24);
}