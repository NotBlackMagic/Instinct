/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/Modules/System/logger.hpp
 * Author:	NotBlackMagic
 * Brief:	System logger, write log messages to console, SD card and telemetry.
 */

#pragma once

#include <stdarg.h>
#include <stdio.h>

#include "uart.hpp"
#include "system.hpp"

#if defined (USE_RTOS)
#include "tx_api.h"
#endif

class Logger {
	public:
		/// @brief Logging levels.
		enum class LogLevel : uint8_t {
			Trace = 0,		// Noise (Register dumps, etc.)
			Debug = 1,		// Dev info
			Info = 2,		// Nominal events
			Warn = 3,		// Watch out
			Error = 4,		// Functionality lost
			Fatal = 5,		// System dead/Unsafe
			Off = 6
		};

		// Delete copy constructors.
		Logger(const Logger&) = delete;
		void operator=(const Logger&) = delete;

		/// @brief Constructor.
		static Logger& Instance();

		/// @brief Initializes the logger, creates the mutex.
		void Init();

		/// @brief Writes a log message, both to the regsitered console, telemetry and SD card. Formatted with added color, timestamp, end-of-line.
		/// @note Not intended to be used directly but through the helper shortcodes LOG_TRACE, LOG_INFO, etc...
		/// @param level	Logging level
		/// @param file		Name of the file calling the logging message
		/// @param line		Code line of the logging message call
		/// @param fmt		String format template, can have subsequent arguments
		void Log(Logger::LogLevel level, const char* file, int line, const char* fmt, ...);
		
		/// @brief Writes a string with possible arguments to the registered console only. No added special formatting.
		/// @param fmt	String format template, can have subsequent arguments (equivalent to a printf)
		void Printf(const char* fmt, ...);

		/// @brief Writes a simple string to the registered console only. No added special formatting.
		/// @param str	String to be written
		void Write(const char* str);

		/// @brief Writes raw bytes to the registered console only.
		/// @param data	Byte array to write
		/// @param len	Length of the byte array
		void Write(const char* data, uint16_t len);

		/// @brief Writes raw bytes to the registered SD card only. Used for binary logging of PubSub messages.
		/// @param topicID	PubSub message topic ID
		/// @param payload	Byte array to write
		/// @param len		Length of the payload
		void LogData(uint16_t topicID, const uint8_t* payload, uint8_t len);

		/// @brief These functions set the Logging Level, level below messages are written to the respective part.
		/// @param level	Log level, lower level higher priority/importance
		void SetConsoleLevel(Logger::LogLevel level);
		void SetSDLevel(Logger::LogLevel level);
		void SetTelemLevel(Logger::LogLevel level);

		/// @brief These functions register a logging part i.e. to where logging messages will be written.
		void RegisterConsole(UART* uart);
		void RegisterSD();
		void RegisterTelemetry();

		/// @brief Helper function to read bytes from the SD card, used internally to the logger.
		/// @param outBuffer	Pointer to write bytes to
		/// @param maxLen		Maximum number of bytes that can be read to outBuffer
		/// @return Number of bytes read to outBuffer.
		uint16_t ReadSDBuffer(uint8_t* outBuffer, uint16_t maxLen);

	private:
		Logger();

		UART* consolePort;

		Logger::LogLevel consoleMinLevel;
		Logger::LogLevel sdMinLevel;
		Logger::LogLevel telemLevel;

		bool initialized;
#if defined (USE_RTOS)
		TX_MUTEX mutex;
#endif

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
#define LOG_INFO(...) 		Logger::Instance().Log(Logger::LogLevel::Info, nullptr, 0, __VA_ARGS__)
#define LOG_WARN(...) 		Logger::Instance().Log(Logger::LogLevel::Warn, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERR(...) 		Logger::Instance().Log(Logger::LogLevel::Error, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_FATAL(...)		Logger::Instance().Log(Logger::LogLevel::Fatal, __FILE__, __LINE__, __VA_ARGS__)