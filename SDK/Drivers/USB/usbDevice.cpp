/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/Drivers/USB/usbDevice.cpp
 */

#include "usbDevice.hpp"

USBDevice::USBDevice(USB& usb) : bus(usb) {
	this->isInitialized = false;
	this->classCount = 0;
	this->nextInterfaceID = 0;
	this->nextInEp = 0x81;
	this->nextOutEp = 0x01;
}

Status USBDevice::Init(const Config& config) {
	this->config = config;
	this->state = State::Default;

	// Configure the Bus
	USB::Config hwConfig;
	hwConfig.speed = USB::BusSpeed::High;
	hwConfig.useDMA = true;
	hwConfig.EventCallback = &USBDevice::EventCallbackWrapper;
	hwConfig.callbackContext = this;

	Status status = this->bus.Init(hwConfig);

	this->isInitialized = true;
	return status;
}

Status USBDevice::Start() {
	return this->bus.Connect();
}

Status USBDevice::Stop() {
	return this->bus.Disconnect();
}

Status USBDevice::RegisterClass(USBClass* usbClass) {
	if(usbClass == nullptr|| this->classCount >= maxClasses) {
		return Status::Error;
	}

	// Assign endpoints to the USB class
	usbClass->AssignResource(this->nextInterfaceID, this->nextInEp, this->nextOutEp);

	this->classes[this->classCount] = usbClass;
	this->classCount++;
	
	return Status::Ok;
}

void USBDevice::EventCallbackWrapper(void* ctx, USB::Event event, uint8_t epAddr, uint32_t len) {
	USBDevice* device = static_cast<USBDevice*>(ctx);
	if(device != nullptr) {
		device->EventHandler(event, epAddr, len);
	}
}

void USBDevice::EventHandler(USB::Event event, uint8_t epAddr, uint32_t len) {
	switch(event) {
		case USB::Event::Reset:
			this->state = State::Default;
			this->deviceAddress = 0;
			for(uint8_t i = 0; i < this->classCount; i++) {
				this->classes[i]->DeInit(this->bus);
			}
			break;
		case USB::Event::Suspend:
			this->state = State::Suspended;
			break;
		case USB::Event::Resume:
			// TODO: Optionally restore state based on address
			break;
		case USB::Event::Sof:
			for(uint8_t i = 0; i < this->classCount; i++) {
				this->classes[i]->OnSoF(this->bus);
			}
			break;
		case USB::Event::Setup: {
			this->HandleSetup(epAddr & 0x7F);
			break;
		}
		case USB::Event::TransferComplete:
			// RX Done
			this->HandleTransferComplete(epAddr, len);
			break;
		case USB::Event::Error:
			for(uint8_t i = 0; i < this->classCount; i++) {
				if(this->classes[i]->HasEndpoint(epAddr) == true) {
					this->classes[i]->OnError(this->bus, epAddr & 0x7F); 
				}
			}
			break;
		default:
			break;
	}
}

void USBDevice::HandleSetup(uint8_t epNum) {
	(void)epNum;

	// Get setup packet from peripheral driver
	USB::SetupPacket setup = *this->bus.GetSetupPacket();

	// Decode setup packet
	// Bits 5-6: Type (0 = Standard, 1 = Class, 2 = Vendor)
	// Bits 0-4: Recipient (0 = Device, 1 = Interface, 2 = Endpoint)
	uint8_t reqType = (setup.requestType & 0x60) >> 5;
	uint8_t recipient = (setup.requestType & 0x1F);

	// Handle the request type
	if(reqType == 0x00) {
		// Standard request type
		switch(recipient) {
			case 0x00:
				// Device
				this->StandardDeviceRequest(setup);
				break;
			case 0x01:
				// Interface
				this->StandardInterfaceRequest(setup);
				break;
			case 0x02:
				// Endpoint
				this->StandardEndpointRequest(setup);
				break;
			default:
				// Invalid
				this->ControlStall();
				break;
		}
	}
	else if(reqType == 0x01 || reqType == 0x02) {
		if(recipient == 0x01) {
			// Interface Recipient
			uint8_t targetInterface = setup.index & 0xFF;
			for(uint8_t i = 0; i < this->classCount; i++) {
				if(this->classes[i]->HasInterface(targetInterface) == true) {
					if(this->classes[i]->OnSetup(this->bus, setup) != Status::Ok) {
						this->ControlStall();
					}
					return;
				}
			}
		}
		else if(recipient == 0x02) { // Endpoint Recipient
			uint8_t targetEp = setup.index & 0xFF;
			for(uint8_t i = 0; i < this->classCount; i++) {
				if(this->classes[i]->HasEndpoint(targetEp) == true) {
					if(this->classes[i]->OnSetup(this->bus, setup) != Status::Ok) {
						this->ControlStall();
					}
					return;
				}
			}
		}

		// No class has been registers, stall
		this->ControlStall();
	}
	else {
		// Unknown or reserved type
		this->ControlStall(); 
	}
}

void USBDevice::HandleTransferComplete(uint8_t epAddr, uint32_t len) {
	uint8_t epNum = epAddr & 0x7F;
	bool isCmdIn = ((epAddr & 0x80) == 0x80);

	if(epNum == 0) {
		// Control endpoint (EP0)
		if(isCmdIn == true) {
			//Endpoint 0 IN Transmission finished
			if(this->ep0TxRemaining > 0) {
				// Send the next chunk
				uint16_t chunk = (this->ep0TxRemaining > 64) ? 64 : this->ep0TxRemaining;
				this->bus.Transmit(0x80, this->ep0TxData, chunk);
				
				this->ep0TxData += chunk;
				this->ep0TxRemaining -= chunk;

				// Arm the OUT Status Phase if this is the final chunk
				if(this->ep0TxRemaining == 0) {
					this->bus.Receive(0x00, nullptr, 0);
				}
			}
			else if(len == 0) {
				this->bus.StartEp0Setup();
			}
		}
		else {
			//Endpoint 0 OUT Reception finished
			for(uint8_t i = 0; i < this->classCount; i++) {
				this->classes[i]->OnDataOut(this->bus, epNum, len);
			}

			if(len == 0) {
				this->bus.StartEp0Setup();
			}
		}
	}
	else {
		// Standard class endpoint
		for(uint8_t i = 0; i < this->classCount; i++) {
			if(this->classes[i]->HasEndpoint(epAddr) == true) {
				if(isCmdIn == true) {
					this->classes[i]->OnDataIn(this->bus, epNum);
				}
				else {
					this->classes[i]->OnDataOut(this->bus, epNum, len);
				}
				return; // Event handled
			}
		}
	}
}

void USBDevice::StandardDeviceRequest(const USB::SetupPacket& setup) {
	switch((USBDevice::Code)setup.request) {
		case USBDevice::Code::GetStatus: {
			// Get device status: If self-powered and if remote wakeup enabled
			// Bit 0 = Self Powered, Bit 1 = Remote Wakeup.
			uint16_t status = 0x0000;
			if(this->config.selfPowered == true) {
				status |= 0x0001;
			}
			this->ControlTransmit((uint8_t*)&status, 2, setup.length);
			break;
		}
		case USBDevice::Code::ClearFeature:
			// Usually used for Remote Wakeup or Test Modes.
			this->ControlACK();
			break;
		case USBDevice::Code::SetFeature:
			// Usually used for Remote Wakeup or Test Modes.
			this->ControlACK();
			break;
		case USBDevice::Code::SetAddress:
			this->SetAddress(setup);
			break;
		case USBDevice::Code::GetDescriptor:
			this->GetDescriptor(setup);
			break;
		case USBDevice::Code::SetDescriptor:
			break;
		case USBDevice::Code::GetConfiguration: {
			// Get current configuration
			// Most devices only have one so return 1 (or 0 if not configured)
			uint8_t curConfig = (this->state == State::Configured) ? 1: 0;
			this->ControlTransmit(&curConfig, 1, setup.length);
			break;
		}
		case USBDevice::Code::SetConfiguration:
			this->SetConfiguration(setup);
			break;
		// Below request are handled in StandardInterfaceRequest()
		// case USBDevice::Code::GetInterface:
		// 	break;
		// case USBDevice::Code::SetInterface:
		// 	break;
		default:
			// Unsupported standard request
			this->ControlStall();
			break;
	}
}

void USBDevice::StandardInterfaceRequest(const USB::SetupPacket& setup) {
	uint8_t targetInterface = setup.index & 0xFF;
	switch((USBDevice::Code)setup.request) {
		case USBDevice::Code::GetInterface: {
			// Return current alternate setting for the interface (Usually 0)
			uint8_t altSetting = 0;
			this->ControlTransmit(&altSetting, 1, setup.length);
			break;
		}
		case USBDevice::Code::SetInterface:
			// Acknowledge the alternate setting change. 
			for(uint8_t i = 0; i < this->classCount; i++) {
				if(this->classes[i]->HasInterface(targetInterface) == true) {
					if(this->classes[i]->OnSetup(this->bus, setup) == Status::Ok) {
						return; // Class successfully handled it and sent the ACK
					}
				}
			}
			this->ControlStall();
			break;
		default:
			// Unsupported standard request
			this->ControlStall();
			break;
	}
}

void USBDevice::StandardEndpointRequest(const USB::SetupPacket& setup) {
	uint8_t epAddr = (uint8_t)(setup.index & 0xFF);
	switch((USBDevice::Code)setup.request) {
		case USBDevice::Code::GetStatus: {
			// Get stall status
			uint16_t epStatus = 0x0000;
			if(this->bus.IsStalled(epAddr) == true) {
				epStatus = 0x0001;	// Bit 0 = Halt
			}
			this->ControlTransmit((uint8_t*)&epStatus, 2, setup.length);
			break;
		}
		case USBDevice::Code::ClearFeature:
			// Clear stalled endpoint
			if(setup.value == 0x00) {
				this->bus.ClearStall(epAddr);
				
				// Route the clear event to the specific class that owns this endpoint
				for(uint8_t i = 0; i < this->classCount; i++) {
					if(this->classes[i]->HasEndpoint(epAddr) == true) {
						this->classes[i]->OnEndpointClear(this->bus, epAddr);
						break;
					}
				}

				this->ControlACK();
			} 
			else {
				this->ControlStall();
			}
			break;
		case USBDevice::Code::SetFeature:
			// The only standard endpoint feature is ENDPOINT_HALT (0x00)
			if(setup.value == 0x00) {
				this->bus.StallEndpoint(epAddr);
				this->ControlACK();
			} 
			else {
				this->ControlStall();
			}
			break;
		default:
			// Unsupported standard request
			this->ControlStall();
			break;
	}
}

void USBDevice::GetDescriptor(const USB::SetupPacket& setup) {
	USBDevice::DescriptorType descType = (USBDevice::DescriptorType)((setup.value >> 8) & 0xFF);
	uint8_t descIndex = (setup.value) & 0xFF;

	switch(descType) {
		case USBDevice::DescriptorType::Device: {
			// Build device descriptor (18-bytes)
			uint8_t devDesc[18] = {
				18,											// bLength
				(uint8_t)USBDevice::DescriptorType::Device,	// bDescriptorType
				0x00, 0x02,								// bcdUSB (USB 2.0)
				0xEF, // bDeviceClass (Miscellaneous)
				0x02, // bDeviceSubClass
				0x01, // bDeviceProtocol
				// 0x00,										// bDeviceClass (0 = Defined at Interface level)
				// 0x00,										// bDeviceSubClass
				// 0x00,										// bDeviceProtocol
				64,											// bMaxPacketSize0
				(uint8_t)(this->config.vid & 0xFF),			// idVendor LSB
				(uint8_t)(this->config.vid >> 8),			// idVendor MSB
				(uint8_t)(this->config.pid & 0xFF),		// idProduct LSB
				(uint8_t)(this->config.pid >> 8),			// idProduct MSB
				(uint8_t)(this->config.version & 0xFF), 
				(uint8_t)(this->config.version >> 8), 
				1,											// iManufacturer (String Index)
				2,											// iProduct (String Index)
				3,											// iSerialNumber (String Index)
				1											// bNumConfigurations
			};
			this->ControlTransmit(devDesc, sizeof(devDesc), setup.length);
			break;
		}
		case USBDevice::DescriptorType::Config: {
			// Build the 9-byte Configuration Header
			uint16_t totalLen = 9;	// Start with header length

			// Calculate total length of all class descriptors
			for(uint8_t i = 0; i < this->classCount; i++) {
				uint16_t classDescLen = 0;
				this->classes[i]->GetConfigDescriptor(this->bus.GetBusSpeed(), &classDescLen);
				totalLen += classDescLen;
			}

			// Limit total length to our buffer size to prevent overflow
			if(totalLen > sizeof(this->ep0TxBuffer)) {
				totalLen = sizeof(this->ep0TxBuffer);
			}

			uint8_t configHdr[9] = {
				9,											// bLength
				(uint8_t)USBDevice::DescriptorType::Config,	// bDescriptorType
				(uint8_t)(totalLen & 0xFF),					// wTotalLength LSB
				(uint8_t)(totalLen >> 8),					// wTotalLength MSB
				this->nextInterfaceID,						// bNumInterfaces
				1,											// bConfigurationValue
				0,											// iConfiguration (String Index)
				(uint8_t)(0x80 | (this->config.selfPowered ? 0x40 : 0x00)), // bmAttributes
				this->config.maxPower						// bMaxPower (in 2mA units)
			};

			// Assemble the full descriptor in the TX buffer
			for(uint8_t i = 0; i < 9; i++) {
				this->ep0TxBuffer[i] = configHdr[i];
			}

			// Append all class descriptors
			uint16_t currentOffset = 9;
			for(uint8_t i = 0; i < this->classCount; i++) {
				uint16_t classDescLen = 0;
				const uint8_t* classDesc = this->classes[i]->GetConfigDescriptor(this->bus.GetBusSpeed(), &classDescLen);
				
				for(uint16_t j = 0; j < classDescLen; j++) {
					if(currentOffset < sizeof(this->ep0TxBuffer)) {
						this->ep0TxBuffer[currentOffset++] = classDesc[j];
					}
				}
			}

			this->ControlTransmit(this->ep0TxBuffer, totalLen, setup.length);
			break;
		}
		case USBDevice::DescriptorType::String: {
			this->SendStringDescriptor(descIndex, setup.length);
			break;
		}
		case USBDevice::DescriptorType::Qualifier: {
			// Build a standard 10-byte Device Qualifier Descriptor
			uint8_t qualDesc[10] = {
				10,												// bLength
				(uint8_t)USBDevice::DescriptorType::Qualifier,	// bDescriptorType
				0x00, 0x02,									// bcdUSB (USB 2.0)
				0xEF,											// bDeviceClass
				0x02,											// bDeviceSubClass
				0x01,											// bDeviceProtocol
				64,												// bMaxPacketSize0
				1,												// bNumConfigurations
				0x00											// bReserved
			};
			this->ControlTransmit(qualDesc, sizeof(qualDesc), setup.length);
			break;
		}
		default:
			// Unknown descriptor requested
			this->ControlStall();
			break;
	}
}

void USBDevice::SetAddress(const USB::SetupPacket& setup) {
	// Save new address
	this->deviceAddress = (uint8_t)(setup.value & 0x7F);

	if(this->deviceAddress != 0) {
		this->state = State::Addressed;
	}
	else {
		this->state = State::Default;
	}

	this->bus.SetAddress(this->deviceAddress);

	// Send ACK back (zero length packet or ZLP)
	this->ControlACK();
}

void USBDevice::SetConfiguration(const USB::SetupPacket& setup) {
	uint8_t configValue = (uint8_t)(setup.value & 0xFF);

	if(configValue == 0) {
		// Un-configure device command/configuration
		this->state = State::Addressed;

		// De-Initialize ALL class driver
		for(uint8_t i = 0; i < this->classCount; i++) {
			this->classes[i]->DeInit(this->bus);
		}

		this->ControlACK();
	}
	else {
		// Activate configuration 1
		this->state = State::Configured;

		// Initialize ALL class driver
		bool allInitSuccess = true;
		for(uint8_t i = 0; i < this->classCount; i++) {
			if(this->classes[i]->Init(this->bus) != Status::Ok) {
				allInitSuccess = false;
				break;
			}
		}

		if(allInitSuccess == true) {
			this->ControlACK();
		}
		else {
			// Class initialization failed...
			this->ControlStall();
		}
	}
}

void USBDevice::SendStringDescriptor(uint8_t index, uint16_t reqLength) {
	// Attention that USB string encoding is UTF-16LE (16-bit Unicode) but C/C++ is 8-bit ASCII/UTF-8. Add 0x00 after each character to "convert"
	if(index == 0) {
		// String 0: Supported Language Codes (0x0409 = English US)
		uint8_t langDesc[4] = {4, (uint8_t)USBDevice::DescriptorType::String, 0x09, 0x04};
		this->ControlTransmit(langDesc, 4, reqLength);
		return;
	}

	const char* strPtr = nullptr;
	switch(index) {
		case 1:
			strPtr = this->config.manufacturer;
			break;
		case 2:
			strPtr = this->config.product;
			break;
		case 3:
			strPtr = this->config.serialNumber;
			break;
		default:
			// Allow classes to handle custom strings (like 0xEE for MS OS 2.0)
			for(uint8_t i = 0; i < this->classCount; i++) {
				if(this->classes[i]->HandleStringDescriptor(this->bus, index, reqLength)) {
					return; // Class handled it
				}
			}
			break;
	}

	if(strPtr == nullptr) {
		// Something wrong... Stall
		this->ControlStall();
		return;
	}

	// Convert from ASCII to UTF-16LE
	// First find string length (use strlen(x)??)
	uint16_t strLen = 0;
	// Calculate the maximum characters that can fit: (BufferSize - 2 bytes for header) / 2 bytes per char
	uint16_t maxChars = (sizeof(this->ep0TxBuffer) - 2) / 2;
	while(strPtr[strLen] != '\0' && strLen < maxChars) {
		strLen += 1;
	}

	uint16_t descLen = 2 + (strLen * 2);
	this->ep0TxBuffer[0] = descLen;
	this->ep0TxBuffer[1] = (uint8_t)USBDevice::DescriptorType::String;

	for(uint16_t i = 0; i < strLen; i++) {
		this->ep0TxBuffer[2 + (i * 2)] = strPtr[i];
		this->ep0TxBuffer[3 + (i * 2)] = 0x00; // UTF-16LE padding
	}

	this->ControlTransmit(this->ep0TxBuffer, descLen, reqLength);
}

void USBDevice::ControlTransmit(const uint8_t* data, uint16_t len, uint16_t reqLength) {
	// Enforce Host requested length limit
	uint16_t transferLength = (len < reqLength) ? len : reqLength;

	// Buffer overflow check and limiting
	if(transferLength > sizeof(ep0TxBuffer)) {
		transferLength = sizeof(ep0TxBuffer);
	}

	// Copy data to local Endpoint 0 buffer
	if(data != this->ep0TxBuffer) {
		for(uint16_t i = 0; i < transferLength; i++) {
			this->ep0TxBuffer[i] = data[i];
		}
	}

	// Software chunking to handle EP0 packet size limit (64 bytes)
	this->ep0TxData = this->ep0TxBuffer;
	this->ep0TxRemaining = transferLength;

	// Grab the first 64 bytes (or less)
	uint16_t chunk = (this->ep0TxRemaining > 64) ? 64 : this->ep0TxRemaining;
	this->bus.Transmit(0x80, this->ep0TxData, chunk);
	// Advance the pointers
	this->ep0TxData += chunk;
	this->ep0TxRemaining -= chunk;

	// Only arm the OUT Status Phase if this was the final chunk
	if(this->ep0TxRemaining == 0) {
		this->bus.Receive(0x00, nullptr, 0); 
	}

	// this->bus.Transmit(0x80, this->ep0TxBuffer, transferLength);

	// this->bus.Receive(0x00, nullptr, 0);
}

void USBDevice::ControlACK() {
	this->bus.Transmit(0x80, nullptr, 0);
}

void USBDevice::ControlStall() {
	this->bus.StallEndpoint(0x80); // Stall EP0 IN
	this->bus.StallEndpoint(0x00); // Stall EP0 OUT

	this->bus.StartEp0Setup();
}