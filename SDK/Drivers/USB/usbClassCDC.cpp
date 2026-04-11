/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/USB/usbClassCDC.cpp
 */

#include "usbClassCDC.hpp"

// CDC ACM Descriptor Template
// Total size: 8 + 9 + 5 + 5 + 4 + 5 + 7 + 9 + 7 + 7 = 66 bytes
// Note: Excludes the main Configuration Descriptor header (handled by USBDevice)
static uint8_t cdcDescriptorTemplate[] = {
	// Interface Association Descriptor (IAD)
	0x08,	// bLength
	0x0B,	// bDescriptorType (IAD)
	0x00,	// bFirstInterface (PATCHED LATER)
	0x02,	// bInterfaceCount
	0x02,	// bFunctionClass (CDC)
	0x02,	// bFunctionSubClass (ACM)
	0x01,	// bFunctionProtocol (AT Commands)
	0x00,	// iFunction

	// Communication Class Interface (CCI)
	0x09,	// bLength
	0x04,	// bDescriptorType (Interface)
	0x00,	// bInterfaceNumber (PATCHED LATER)
	0x00,	// bAlternateSetting
	0x01,	// bNumEndpoints (1 Interrupt IN)
	0x02,	// bInterfaceClass (CDC)
	0x02,	// bInterfaceSubClass (ACM)
	0x01,	// bInterfaceProtocol
	0x00,	// iInterface

	// CDC Functional Descriptors
	// Header Functional Descriptor
	0x05, 0x24, 0x00, 0x10, 0x01, 
	// Call Management Functional Descriptor
	0x05, 0x24, 0x01, 0x00, 0x01,	// Data interface is 1 (PATCHED LATER at index 26)
	// ACM Functional Descriptor
	0x04, 0x24, 0x02, 0x02,
	// Union Functional Descriptor
	0x05, 0x24, 0x06, 0x00, 0x01,	// Comm interface 0, Data interface 1 (PATCHED LATER at index 34, 35)

	// Interrupt IN Endpoint
	0x07,	// bLength
	0x05,	// bDescriptorType (Endpoint)
	0x81,	// bEndpointAddress (PATCHED LATER)
	0x03,	// bmAttributes (Interrupt)
	0x08, 0x00,	// wMaxPacketSize (8 bytes)
	0x10,	// bInterval (16 frames)

	// Data Class Interface (DCI)
	0x09,	// bLength
	0x04,	// bDescriptorType (Interface)
	0x01,	// bInterfaceNumber (PATCHED LATER)
	0x00,	// bAlternateSetting
	0x02,	// bNumEndpoints (2 Bulk)
	0x0A,	// bInterfaceClass (Data)
	0x00,	// bInterfaceSubClass
	0x00,	// bInterfaceProtocol
	0x00,	// iInterface

	// Bulk OUT Endpoint
	0x07,	// bLength
	0x05,	// bDescriptorType (Endpoint)
	0x01,	// bEndpointAddress (PATCHED LATER)
	0x02,	// bmAttributes (Bulk)
	0x00, 0x02,	// wMaxPacketSize (512 bytes for High-Speed)
	0x00,	// bInterval

	// Bulk IN Endpoint
	0x07,	// bLength
	0x05,	// bDescriptorType (Endpoint)
	0x82,	// bEndpointAddress (PATCHED LATER)
	0x02,	// bmAttributes (Bulk)
	0x00, 0x02,	// wMaxPacketSize (512 bytes for High-Speed)
	0x00	// bInterval
};

static uint8_t activeCDCDescriptor[sizeof(cdcDescriptorTemplate)];

USBClassCDC::USBClassCDC() {
	this->lineCoding.baudRate = 115200;
	this->lineCoding.stopBits = StopBits::StopBits_1;
	this->lineCoding.parity = Parity::None;
	this->lineCoding.dataBits = DataBits::DataBits_8;

	this->PackLineCoding();

	this->commInterface = 0xFF;
	this->dataInterface = 0xFF;
	this->intInEp = 0xFF;
	this->bulkInEp = 0xFF;
	this->bulkOutEp = 0xFF;

	this->rxBufHead = 0;
	this->rxBufTail = 0;
	this->txBufHead = 0;
	this->txBufTail = 0;
	this->txBusy = false;
}

Status USBClassCDC::AssignResource(uint8_t& nextInterfaceId, uint8_t& nextInEp, uint8_t& nextOutEp) {
	this->commInterface = nextInterfaceId++;
	this->dataInterface = nextInterfaceId++;

	this->intInEp = 0x80 | nextInEp++;
	this->bulkInEp = 0x80 | nextInEp++;
	this->bulkOutEp = nextOutEp++;

	return Status::Ok;
}

bool USBClassCDC::HasEndpoint(uint8_t epAddr) const {
	if(epAddr == this->intInEp || epAddr == this->bulkInEp || epAddr == this->bulkOutEp) {
		return true;
	}
	return false;
}

bool USBClassCDC::HasInterface(uint8_t interfaceId) const {
	if(interfaceId == this->commInterface || interfaceId == this->dataInterface) {
		return true;
	}
	return false;
}

Status USBClassCDC::Init(USB& usb) {
	this->bus = &usb;

	// Reset buffer state
	this->rxBufHead = 0;
	this->rxBufTail = 0;
	this->txBufHead = 0;
	this->txBufTail = 0;
	this->txBusy = false;

	this->bulkMaxPacketSize = (this->bus->GetBusSpeed() == USB::BusSpeed::High) ? 512 : 64;

	// Open endpoints in the hardware
	Status status = this->bus->OpenEndpoint(this->intInEp, USB::EndpointType::Interrupt, 8);
	if(status != Status::Ok) {
		return status;
	}
	status = this->bus->OpenEndpoint(this->bulkInEp, USB::EndpointType::Bulk, this->bulkMaxPacketSize);
	if(status != Status::Ok) {
		return status;
	}
	status = this->bus->OpenEndpoint(this->bulkOutEp, USB::EndpointType::Bulk, this->bulkMaxPacketSize);
	if(status != Status::Ok) {
		return status;
	}

	// Prime the OUT endpoint to receive data
	return this->bus->Receive(this->bulkOutEp, this->epOutBuffer, this->bulkMaxPacketSize);
}

Status USBClassCDC::DeInit(USB& usb) {
	(void)usb;
	if(this->bus != nullptr) {
		// Close endpoints
		this->bus->CloseEndpoint(this->intInEp);
		this->bus->CloseEndpoint(this->bulkInEp);
		this->bus->CloseEndpoint(this->bulkOutEp);
		this->bus = nullptr;
	}
	return Status::Ok;
}

const uint8_t* USBClassCDC::GetConfigDescriptor(USB::BusSpeed speed, uint16_t* len) {
	memcpy(activeCDCDescriptor, cdcDescriptorTemplate, sizeof(cdcDescriptorTemplate));

	// Patch Interface IDs
	activeCDCDescriptor[2] = this->commInterface;	// IAD First Interface
	activeCDCDescriptor[10] = this->commInterface;	// CCI Interface Number
	activeCDCDescriptor[26] = this->dataInterface;	// Call Management Data Interface
	activeCDCDescriptor[34] = this->commInterface;	// Union Comm Interface
	activeCDCDescriptor[35] = this->dataInterface;	// Union Data Interface
	activeCDCDescriptor[45] = this->dataInterface;	// DCI Interface Number

	// Patch Endpoint IDs
	activeCDCDescriptor[38] = this->intInEp;	// Interrupt IN
	activeCDCDescriptor[54] = this->bulkOutEp;	// Bulk OUT
	activeCDCDescriptor[61] = this->bulkInEp;	// Bulk IN

	this->bulkMaxPacketSize = (speed == USB::BusSpeed::High) ? 512 : 64;
	// Bulk OUT wMaxPacketSize (indices 56 & 57)
	activeCDCDescriptor[56] = static_cast<uint8_t>(this->bulkMaxPacketSize & 0xFF);
	activeCDCDescriptor[57] = static_cast<uint8_t>((this->bulkMaxPacketSize >> 8) & 0xFF);
	// Bulk IN wMaxPacketSize (indices 63 & 64)
	activeCDCDescriptor[63] = static_cast<uint8_t>(this->bulkMaxPacketSize & 0xFF);
	activeCDCDescriptor[64] = static_cast<uint8_t>((this->bulkMaxPacketSize >> 8) & 0xFF);


	if(len != nullptr) {
		*len = sizeof(activeCDCDescriptor);
	}
	return activeCDCDescriptor;
}

Status USBClassCDC::OnSetup(USB& usb, const USB::SetupPacket& setup) {
	(void)usb;
	// Check if the request is directed to our communication interface
	if((setup.requestType & 0x1F) == 0x01 && setup.index == commInterface) {
		switch(setup.request) {
			case CDCRequest::SetLineCoding:
				// Host sends LineCoding config. Prepare Ep0 to receive setup.wLength bytes
				return this->bus->Receive(0x00, this->lineCodingBytes, 7);
			case CDCRequest::GetLineCoding:
				// Host asks for LineCoding config. Send our lineCoding struct
				this->PackLineCoding();
				this->bus->Receive(0x00, nullptr, 0);
				return this->bus->Transmit(0x80, this->lineCodingBytes, 7);
			case CDCRequest::SetControlLineState:
				// Host asserts/deasserts DTR and RTS. Handled in setup phase, no data stage.
				// bool dtr = (setup.value & 0x0001) != 0;
				// bool rts = (setup.value & 0x0002) != 0;
				return this->bus->Transmit(0x80, nullptr, 0);
			case CDCRequest::SendBreak:
				// Acknowledge the break signal
				return this->bus->Transmit(0x80, nullptr, 0);
			default:
				break;
		}
	}
	return Status::Error; // Unknown or unhandled request
}

Status USBClassCDC::OnDataIn(USB& usb, uint8_t epNum) {
	(void)usb;
	if(epNum == (bulkInEp & 0x7F)) {
		// Bulk IN transfer complete. The hardware FIFO is ready for the next Transmit().
		this->txBusy = false;
		
		// Start the next batch of data if the application queued more while this transfer was happening.
		this->StartTX();
	}
	return Status::Ok;
}

Status USBClassCDC::OnDataOut(USB& usb, uint8_t epNum, uint32_t len) {
	(void)usb;
	if(epNum == 0x00) {
		if(len == 7) {
			// Convert 7-byte little-endian array back to struct
			this->UnpackLineCoding();

			// Send the Status Phase IN (ZLP) to acknowledge the settings
			this->bus->Transmit(0x80, nullptr, 0);
		}
	}
	else if(epNum == this->bulkOutEp) {
		// Copy the newly received data from the hardware catch buffer into our ring buffer
		for(uint32_t i = 0; i < len; i++) {
			if(Available() < rxBufferSize) {
				this->rxBuffer[this->rxBufHead] = this->epOutBuffer[i];
				this->rxBufHead = (this->rxBufHead + 1) % this->rxBufferSize;
			}
		}

		// Re-prime the endpoint to receive the next packet into the catch buffer
		this->bus->Receive(this->bulkOutEp, this->epOutBuffer, this->bulkMaxPacketSize);
	}
	return Status::Ok;
}

Status USBClassCDC::Write(const uint8_t* buf, uint32_t len) {
	if(this->bus == nullptr) {
		return Status::Error;
	}

	uint32_t itemsInQueue = 0;
	if(this->txBufHead >= this->txBufTail) {
		itemsInQueue = this->txBufHead - this->txBufTail;
	}
	else {
		itemsInQueue = this->txBufferSize - this->txBufTail + this->txBufHead;
	}

	uint32_t freeSpace = this->txBufferSize - itemsInQueue - 1; // -1 to distinguish full from empty

	uint32_t bytesToWrite = (len > freeSpace) ? freeSpace : len;

	// Push application data into the TX ring buffer
	for(uint32_t i = 0; i < bytesToWrite; i++) {
		this->txBuffer[this->txBufHead] = buf[i];
		this->txBufHead = (this->txBufHead + 1) % this->txBufferSize;
	}

	// Kickstart the transmission pipeline
	if(bytesToWrite > 0) {
		this->StartTX();
	}

	return Status::Ok;
}

uint32_t USBClassCDC::Read(uint8_t* buf, uint32_t maxLen) {
	uint32_t bytesRead = 0;

	// Pop data from the ring buffer up to the requested length, stopping if we catch up to the head.
	while((bytesRead < maxLen) && (this->rxBufTail != this->rxBufHead)) {
		buf[bytesRead] = this->rxBuffer[this->rxBufTail];
		this->rxBufTail = (this->rxBufTail + 1) % this->rxBufferSize;
		bytesRead = bytesRead + 1;
	}

	return bytesRead;
}

uint32_t USBClassCDC::Available() {
	if(this->rxBufHead >= this->rxBufTail) {
		return this->rxBufHead - this->rxBufTail;
	}
	else {
		return this->rxBufferSize - this->rxBufTail + this->rxBufHead;
	}
}

void USBClassCDC::StartTX() {
	// If the hardware is busy or we have no bus, do nothing
	if(this->bus == nullptr || this->txBusy == true) {
		return;
	}

	uint32_t bytesAvailable = 0;
	if(this->txBufHead >= this->txBufTail) {
		bytesAvailable = this->txBufHead - this->txBufTail;
	}
	else {
		bytesAvailable = this->txBufferSize - this->txBufTail + this->txBufHead;
	}
	// If there is no data to send, exit
	if(bytesAvailable == 0) {
		return;
	}

	// Calculate how much we can send in this single hardware packet
	uint32_t bytesToSend = (bytesAvailable > this->bulkMaxPacketSize) ? this->bulkMaxPacketSize : bytesAvailable;

	// Move data from the ring buffer into the 32-byte aligned hardware staging buffer.
	// This perfectly handles ring buffer wrap-around before passing it to the hardware.
	for(uint32_t i = 0; i < bytesToSend; i++) {
		this->epInBuffer[i] = this->txBuffer[this->txBufTail];
		this->txBufTail = (this->txBufTail + 1) % this->txBufferSize;
	}

	// Lock the pipeline and hand the buffer to driver
	this->txBusy = true;
	this->bus->Transmit(this->bulkInEp, this->epInBuffer, bytesToSend);
}

void USBClassCDC::PackLineCoding() {
	this->lineCodingBytes[0] = static_cast<uint8_t>(this->lineCoding.baudRate & 0xFF);
	this->lineCodingBytes[1] = static_cast<uint8_t>((this->lineCoding.baudRate >> 8) & 0xFF);
	this->lineCodingBytes[2] = static_cast<uint8_t>((this->lineCoding.baudRate >> 16) & 0xFF);
	this->lineCodingBytes[3] = static_cast<uint8_t>((this->lineCoding.baudRate >> 24) & 0xFF);
	this->lineCodingBytes[4] = static_cast<uint8_t>(this->lineCoding.stopBits);
	this->lineCodingBytes[5] = static_cast<uint8_t>(this->lineCoding.parity);
	this->lineCodingBytes[6] = static_cast<uint8_t>(this->lineCoding.dataBits);
}

void USBClassCDC::UnpackLineCoding() {
	this->lineCoding.baudRate = static_cast<uint32_t>(this->lineCodingBytes[0]) |
								(static_cast<uint32_t>(this->lineCodingBytes[1]) << 8) |
								(static_cast<uint32_t>(this->lineCodingBytes[2]) << 16) |
								(static_cast<uint32_t>(this->lineCodingBytes[3]) << 24);
	this->lineCoding.stopBits = (StopBits)this->lineCodingBytes[4];
	this->lineCoding.parity = (Parity)this->lineCodingBytes[5];
	this->lineCoding.dataBits = (DataBits)this->lineCodingBytes[6];
}