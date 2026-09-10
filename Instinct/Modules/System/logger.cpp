/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/logger.cpp
 */

#include "logger.hpp"

static constexpr uint16_t maxLogLines = 128;

// Singleton access
Logger& Logger::Instance() {
	static Logger instance;
	return instance;
}

Logger::Logger()
	: 	consolePort(nullptr),
		consoleMinLevel(Logger::LogLevel::Info), 
		sdMinLevel(Logger::LogLevel::Info),
		telemLevel(Logger::LogLevel::Warn),
		initialized(false) {}

void Logger::Init() {
	if(initialized == false) {
		// Initialize the mutex
		UINT status = tx_mutex_create(&mutex, const_cast<char*>("log mutex"), TX_NO_INHERIT);

		if(status == TX_SUCCESS) {
			initialized = true;
		}
	}
}

void Logger::SetConsoleLevel(Logger::LogLevel level) {
	consoleMinLevel = level;
}

void Logger::SetSDLevel(Logger::LogLevel level) {
	sdMinLevel = level;
}

void Logger::SetTelemLevel(Logger::LogLevel level) {
	telemLevel = level;
}

void Logger::RegisterConsole(UART* uart) {
	consolePort = uart;
}

void Logger::LogData(uint16_t topicId, const uint8_t* payload, uint8_t len) {
	if(len == 0 || sdMinLevel == Logger::LogLevel::Off) {
		return;
	}

	// Lock Logger
	bool useLock = false;
	if(initialized == true) {
		if(tx_thread_identify() == nullptr) {
			return; // Prevent ISR corruption
		}
		useLock = true;
	}
	if(useLock == true) {
		if(tx_mutex_get(&mutex, TX_NO_WAIT) != TX_SUCCESS) {
			return; 
		}
	}

	uint16_t syncWord = 0x55AA;
	uint16_t packetSize = 5 + len; // 2(sync) + 2(topic) + 1(len) + payload

	if((sdCount + packetSize) <= sdBufferSize) {
		auto writeByte = [&](uint8_t b) {
			sdBuffer[sdHead] = b;
			sdHead = (sdHead + 1) % sdBufferSize;
		};

		writeByte(syncWord & 0xFF);
		writeByte(syncWord >> 8);
		writeByte(topicId & 0xFF);
		writeByte(topicId >> 8);
		writeByte(len);

		for(uint8_t i = 0; i < len; i++) {
			writeByte(payload[i]);
		}
		sdCount += packetSize;
	}

	if(useLock == true) {
		tx_mutex_put(&mutex);
	}
}

void Logger::Log(Logger::LogLevel level, const char* file, int line, const char* fmt, ...) {
	// Check used/set log levels
	if(level < consoleMinLevel && level < sdMinLevel) {
		return;
	}

	// Lock Logger
	bool useLock = false;
	if(initialized == true) {
		if(tx_thread_identify() == nullptr) {
			return; // Prevent ISR corruption
		}
		useLock = true;
	}
	if(useLock == true) {
		if(tx_mutex_get(&mutex, TX_NO_WAIT) != TX_SUCCESS) {
			return; 
		}
	}

	// Allocate extra space for leading (5) and trailing (5) color codes
	char buffer[maxLogLines + 13];
	uint16_t index;

	static const uint16_t colorLen = 5;
	const char* colorCode = "\033[39m"; // Default Reset (4 bytes, padded to 5)
	const char* tagPtr = "";

	switch(level) {
		case LogLevel::Trace: {
			colorCode = "\033[39m";		// Color: Default
			tagPtr = "[T] ";
			break;
		}
		case LogLevel::Debug: {
			colorCode = "\033[39m";		// Color: Default
			tagPtr = "[D] ";
			break;
		}
		case LogLevel::Info: {
			colorCode = "\033[39m";		// Color: Default
			tagPtr = "[I] ";
			break;
		}
		case LogLevel::Warn: {
			colorCode = "\033[33m";		// Color: Yellow
			tagPtr = "[W] ";
			break;
		}
		case LogLevel::Error: {
			colorCode = "\033[31m";		// Color: Red
			tagPtr = "[E] ";
			break;
		}
		case LogLevel::Fatal: {
			colorCode = "\033[35m";		// Color: Magenta
			tagPtr = "[!] ";
			break;
		}
		default:
			return;
	}

	// Skip color code spacing at beginning (only used for console/terminal/shell)
	index = colorLen;	

	// Add timestamp
	float timeSec = (float)tx_time_get() / TX_TIMER_TICKS_PER_SECOND;
	index += snprintf(&buffer[index], maxLogLines - index, "[%8.3f] ", timeSec);

	// Add tag
	index += snprintf(&buffer[index], maxLogLines - index, "%s", tagPtr);

	// Add code file and line
	if(file != nullptr) {
		const char* filename = strrchr(file, '/');
		const char* filenameWin = strrchr(file, '\\');

		if(filenameWin > filename) {
			filename = filenameWin;
		}

		if(filename) {
			filename++;
		} 
		else {
			filename = file;
		}
		
		index += snprintf(&buffer[index], maxLogLines - index, "%s:%d ", filename, line);
	}

	// Add message
	va_list args;
	va_start(args, fmt);
	index += vsnprintf(&buffer[index], maxLogLines - index, fmt, args);
	va_end(args);

	// Add newline and reset
	if(index < maxLogLines + colorLen - 2) {
		buffer[index++] = '\r';
		buffer[index++] = '\n';
		buffer[index] = '\0';
	}
	else {
		index = maxLogLines + colorLen - 2;
		buffer[index++] = '\r';
		buffer[index++] = '\n';
		buffer[index] = '\0';
	}

	// Send to all log message handlers
	if(level >= consoleMinLevel && consolePort != nullptr) {
		// Insert the active color code at the very beginning (index 0)
		memcpy(&buffer[0], colorCode, colorLen);
		
		// Add color reset code to end
		memcpy(&buffer[index], "\033[39m", colorLen);

		consolePort->Transmit((uint8_t*)buffer, (index + colorLen));
	}

	if(level >= sdMinLevel) {
		// // Copy the formatted string into the circular buffer
		// for(uint16_t i = colorLen; i < index; i++) {
		// 	if(sdCount < sdBufferSize) {
		// 		sdBuffer[sdHead] = buffer[i];
		// 		sdHead = (sdHead + 1) % sdBufferSize;
		// 		sdCount++;
		// 	}
		// 	else {
		// 		// Buffer overflow!
		// 		break;
		// 	}
		// }

		// Pack ASCII text into the binary stream using Topic ID 0x0000
		uint16_t syncWord = 0x55AA;
		uint16_t topicId = 0x0000;
		uint8_t payloadLen = (index - colorLen > 255) ? 255 : (index - colorLen);
		uint16_t packetSize = 5 + payloadLen;

		if((sdCount + packetSize) <= sdBufferSize) {
			sdBuffer[sdHead] = (syncWord & 0xFF);
			sdHead = (sdHead + 1) % sdBufferSize;
			
			sdBuffer[sdHead] = (syncWord >> 8);
			sdHead = (sdHead + 1) % sdBufferSize;

			sdBuffer[sdHead] = (topicId & 0xFF);
			sdHead = (sdHead + 1) % sdBufferSize;

			sdBuffer[sdHead] = (topicId >> 8);
			sdHead = (sdHead + 1) % sdBufferSize;

			sdBuffer[sdHead] = (payloadLen);
			sdHead = (sdHead + 1) % sdBufferSize;

			for(uint16_t i = colorLen; i < colorLen + payloadLen; i++) {
				sdBuffer[sdHead] = (buffer[i]);
				sdHead = (sdHead + 1) % sdBufferSize;
			}
			sdCount += packetSize;
		}
	}

	if(useLock == true) {
		tx_mutex_put(&mutex);
	}
}

uint16_t Logger::ReadSDBuffer(uint8_t* outBuffer, uint16_t maxLen) {
	// Lock Logger
	bool useLock = false;
	if(initialized == true) {
		if(tx_thread_identify() == nullptr) {
			return 0; // Prevent ISR corruption
		}
		useLock = true;
	}
	if(useLock == true) {
		if(tx_mutex_get(&mutex, TX_NO_WAIT) != TX_SUCCESS) {
			return 0;
		}
	}

	uint16_t bytesRead = 0;

	// For high speed SDMMC, force reads in multiples of 512 bytes.
	uint16_t available = (sdCount > maxLen) ? maxLen : sdCount;
	available = (available / 512) * 512;

	while(bytesRead < available) {
		outBuffer[bytesRead] = sdBuffer[sdTail];
		sdTail = (sdTail + 1) % sdBufferSize;
		sdCount--;
		bytesRead++;
	}

	if(useLock == true) {
		tx_mutex_put(&mutex);
	}

	return bytesRead;
}

void Logger::Printf(const char* fmt, ...) {
	// Lock Logger
	bool useLock = false;
	if(initialized == true) {
		if(tx_thread_identify() == nullptr) {
			return; // Prevent ISR corruption
		}
		useLock = true;
	}
	if(useLock == true) {
		if(tx_mutex_get(&mutex, TX_NO_WAIT) != TX_SUCCESS) {
			return; 
		}
	}

	char buffer[maxLogLines];

	va_list args;
	va_start(args, fmt);
	int offset = vsnprintf(buffer, maxLogLines, fmt, args);
	va_end(args);

	if(consolePort != nullptr) {
		consolePort->Transmit((uint8_t*)buffer, offset);
	}

	if(useLock == true) {
		tx_mutex_put(&mutex);
	}
}

void Logger::Write(const char* str) {
	if(str == nullptr) {
		return;
	}

	Write(str, strlen(str));
}

void Logger::Write(const char* data, uint16_t len) {
	if(len == 0) {
		return;
	}

	// Lock Logger
	bool useLock = false;
	if(initialized == true) {
		if(tx_thread_identify() == nullptr) {
			return; // Prevent ISR corruption
		}
		useLock = true;
	}
	if(useLock == true) {
		if(tx_mutex_get(&mutex, TX_NO_WAIT) != TX_SUCCESS) {
			return; 
		}
	}

	if(consolePort != nullptr) {
		consolePort->Transmit((uint8_t*)data, len);
	}

	if(useLock == true) {
		tx_mutex_put(&mutex);
	}
}