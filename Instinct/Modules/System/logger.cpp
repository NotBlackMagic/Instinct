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
		sdMinLevel(Logger::LogLevel::Debug),
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

void Logger::Log(Logger::LogLevel level, const char* file, int line, const char* fmt, ...) {
	// Check used/set log levels
	if(level < consoleMinLevel && level < sdMinLevel) {
		return;
	}

	// Lock Logger
	bool useLock = initialized && (tx_thread_identify() != nullptr);
	if(useLock == true) {
		if (tx_mutex_get(&mutex, TX_WAIT_FOREVER) != TX_SUCCESS) {
			return;
		}
	}

	char buffer[maxLogLines + 5];
	uint16_t index;

	static const size_t colorLen = 5;
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

	// Write color header
	memcpy(buffer, colorCode, colorLen);
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
	if(index < maxLogLines - 3) {
		buffer[index++] = '\r';
		buffer[index++] = '\n';
		buffer[index] = '\0';
	}
	else {
		buffer[maxLogLines-3] = '\r';
		buffer[maxLogLines-2] = '\n';
		buffer[maxLogLines-1] = '\0';
	}

	// Send to all log message handlers
	if(level >= consoleMinLevel && consolePort != nullptr) {
		// Add color reset code to end
		memcpy(&buffer[index], "\033[39m", colorLen);
		index += colorLen;

		consolePort->Transmit((uint8_t*)buffer, index);
	}

	if(level >= sdMinLevel) {
		// RingBufWrite(&_sdBuf, buffer + ColorLen, offset - ColorLen);
	}

	if(useLock == true) {
		tx_mutex_put(&mutex);
	}
}

void Logger::Printf(const char* fmt, ...) {
	// Lock Logger
	bool useLock = initialized && (tx_thread_identify() != nullptr);
	if(useLock == true) {
		if(tx_mutex_get(&mutex, TX_WAIT_FOREVER) != TX_SUCCESS) {
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

void Logger::Write(const char* data, size_t len) {
	if(len == 0) {
		return;
	}

	// Lock Logger
	bool useLock = initialized && (tx_thread_identify() != nullptr);
	if(useLock == true) {
		if (tx_mutex_get(&mutex, TX_WAIT_FOREVER) != TX_SUCCESS) {
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