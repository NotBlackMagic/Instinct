/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    Instinct/Modules/System/loggerThread.cpp
 */

#include "loggerThread.hpp"

TX_THREAD LoggerThread::threadPtr;
uint8_t LoggerThread::threadStack[4096];

__attribute__((aligned(32))) uint8_t LoggerThread::writeBuffer[512];

void LoggerThread::Init() {
	uint32_t status = tx_thread_create(&threadPtr, const_cast<char*>("Logger"),
										LoggerThread::Run,
										0,
										threadStack,
										sizeof(threadStack),
										16,
										16,
										TX_NO_TIME_SLICE,
										TX_AUTO_START);

	if(status != TX_SUCCESS) {
		LOG_ERR("ThreadX Logger Thread Create Failed.");
	}
}

void LoggerThread::Run(ULONG input) {
	LOG_INFO("Logger Thread Initialized.");

	FX_FILE logFile;
	UINT status;
	bool fileIsOpen = false;
	uint16_t bytesToSync;

	uint32_t timestamp = Time::GetMs();
	uint32_t deltaTime = 0;
	
	while(1) {
		// Wait for StorageThread to mount the SD card
		if(StorageThread::IsReady() == false) {
			if(fileIsOpen == true) {
				fx_file_close(&logFile);
				fileIsOpen = false;
			}
			tx_thread_sleep(100);
			continue; 
		}

		// Open or Create the log file
		if(fileIsOpen == false) {
			status = fx_file_open(StorageThread::GetMedia(), &logFile, const_cast<char*>("syslog.txt"), FX_OPEN_FOR_WRITE);
			
			if(status == FX_NOT_FOUND) {
				fx_file_create(StorageThread::GetMedia(), const_cast<char*>("syslog.txt"));
				status = fx_file_open(StorageThread::GetMedia(), &logFile, const_cast<char*>("syslog.txt"), FX_OPEN_FOR_WRITE);
			}

			if(status == FX_SUCCESS) {
				fx_file_seek(&logFile, logFile.fx_file_current_file_size);
				fileIsOpen = true;
			} 
			else {
				LOG_INFO("SD Logging open/create log file Failed.");
				tx_thread_sleep(100);
				continue;
			}
		}

		// Read from the Logger RAM buffer into the temporary chunk buffer
		bytesToSync = Logger::Instance().ReadSDBuffer(writeBuffer, sizeof(writeBuffer));
		
		if(bytesToSync > 0) {
			status = fx_file_write(&logFile, writeBuffer, bytesToSync);
			if(status == FX_SUCCESS) {
				// Periodic flush: Flush every 2 seconds
				deltaTime = Time::GetMs() - timestamp;
				if(deltaTime > 2000) {
					fx_media_flush(StorageThread::GetMedia());
					timestamp = Time::GetMs();
				}
			} 
			else {
				LOG_INFO("SD Logging writing to log file Failed.");
				fx_file_close(&logFile);
				fileIsOpen = false;
			}
		} 
		else {
			tx_thread_sleep(50);
		}
	}
}