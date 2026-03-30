#pragma once

#include <stdarg.h>
#include <stdio.h>

#include "hardware.hpp"

#include "tx_api.h"

class Logger {
	public:
		enum class LogLevel : uint8_t {
			Trace = 0,		// Noise (Register dumps, etc.)
			Debug = 1,		// Dev info
			Info  = 2,		// Nominal events
			Warn  = 3,		// Watch out
			Error = 4,		// Functionality lost
			Fatal = 5,		// System dead/Unsafe
			Off   = 6  
		};

		static Logger& Instance();

		void Init();
		void Log(Logger::LogLevel level, const char* file, int line, const char* fmt, ...);
		void Printf(const char* fmt, ...);
		void Write(const char* str);
		void Write(const char* data, size_t len);

		void SetConsoleLevel(Logger::LogLevel level);
		void SetSDLevel(Logger::LogLevel level);
		void SetTelemLevel(Logger::LogLevel level);

		void RegisterConsole(UART* uart);
		void RegisterSD();
		void RegisterTelemetry();

		//Prevent copying
		Logger(const Logger&) = delete;
		void operator=(const Logger&) = delete;

	private:
		Logger();

		UART* consolePort;

		Logger::LogLevel consoleMinLevel;
		Logger::LogLevel sdMinLevel;
		Logger::LogLevel telemLevel;

		bool initialized;
		TX_MUTEX mutex;
};

// Helpers/shortcodes for logging specific message types
#define LOG_TRACE(...)		Logger::Instance().Log(Logger::LogLevel::Trace, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_DEBUG(...)		Logger::Instance().Log(Logger::LogLevel::Debug, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_INFO(...) 		Logger::Instance().Log(Logger::LogLevel::Info,  nullptr, 0, __VA_ARGS__)
#define LOG_WARN(...) 		Logger::Instance().Log(Logger::LogLevel::Warn,  __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERR(...) 		Logger::Instance().Log(Logger::LogLevel::Error, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_FATAL(...)		Logger::Instance().Log(Logger::LogLevel::Fatal, __FILE__, __LINE__, __VA_ARGS__)