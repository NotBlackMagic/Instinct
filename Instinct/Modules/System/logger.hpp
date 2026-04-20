/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/logger.hpp
 * Author:  NotBlackMagic
 * Brief:   System logger, write log messages to console, SD card and telemetry.
 */

#pragma once

#include <stdarg.h>
#include <stdio.h>

#include "uart.hpp"

#include "tx_api.h"

class Logger {
	public:
		/// @brief Logging levels.
		enum class LogLevel : uint8_t {
			Trace = 0,		// Noise (Register dumps, etc.)
			Debug = 1,		// Dev info
			Info  = 2,		// Nominal events
			Warn  = 3,		// Watch out
			Error = 4,		// Functionality lost
			Fatal = 5,		// System dead/Unsafe
			Off   = 6  
		};


		// Delete copy constructors.
		Logger(const Logger&) = delete;
		void operator=(const Logger&) = delete;

		/// @brief Constructor.
		static Logger& Instance();

		void Init();
		void Log(Logger::LogLevel level, const char* file, int line, const char* fmt, ...);
		void Printf(const char* fmt, ...);
		void Write(const char* str);
		void Write(const char* data, uint16_t len);

		void SetConsoleLevel(Logger::LogLevel level);
		void SetSDLevel(Logger::LogLevel level);
		void SetTelemLevel(Logger::LogLevel level);

		void RegisterConsole(UART* uart);
		void RegisterSD();
		void RegisterTelemetry();

		uint16_t ReadSDBuffer(uint8_t* outBuffer, uint16_t maxLen);

	private:
		Logger();

		UART* consolePort;

		Logger::LogLevel consoleMinLevel;
		Logger::LogLevel sdMinLevel;
		Logger::LogLevel telemLevel;

		bool initialized;
		TX_MUTEX mutex;

		// RAM Buffer for SD Card text logging
		static constexpr uint16_t sdBufferSize = 8192; // 8KB buffer
		__attribute__((aligned(32))) uint8_t sdBuffer[sdBufferSize];
		uint16_t sdHead{0};
		uint16_t sdTail{0};
		uint16_t sdCount{0};
};

// Helpers/shortcodes for logging specific message types
#define LOG_TRACE(...)		Logger::Instance().Log(Logger::LogLevel::Trace, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_DEBUG(...)		Logger::Instance().Log(Logger::LogLevel::Debug, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_INFO(...) 		Logger::Instance().Log(Logger::LogLevel::Info,  nullptr, 0, __VA_ARGS__)
#define LOG_WARN(...) 		Logger::Instance().Log(Logger::LogLevel::Warn,  __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERR(...) 		Logger::Instance().Log(Logger::LogLevel::Error, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_FATAL(...)		Logger::Instance().Log(Logger::LogLevel::Fatal, __FILE__, __LINE__, __VA_ARGS__)