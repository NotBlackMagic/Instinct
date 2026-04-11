#pragma once

#include <stdint.h>

#include "i2c.hpp"

class INA700 {
	public:
		// Standard Chip identifications
		static constexpr uint8_t i2cAddrPrimary = 0x44;
		static constexpr uint8_t i2cAddrSecondary = 0x45;
		static constexpr uint16_t chipID = 0x5449;

		/// @brief Device register map.
		enum class Register : uint8_t {
			CONFIG = 0x00,			//Configuration, 16 Bits
			ADC_CONFIG = 0x01,		//ADC Configuration, 16 Bits
			VBUS = 0x05,			//Bus Voltage Measurement, 16 Bits
			DIETEMP = 0x06,			//Temperature Measurement, 16 Bits
			CURRENT = 0x07,			//Current Result, 16 Bits
			POWER = 0x08,			//Power Result, 24 Bits
			ENERGY = 0x09,			//Energy Result, 40 Bits
			CHARGE = 0x0A,			//Charge Result, 40 Bits
			ALERT_DIAG = 0x0B,		//Diagnostic Flags and Alert, 16 Bits
			COL = 0x0C,				//Current Over-Limit Threshold, 16 Bits
			CUL = 0x0D,				//Current Under-Limit Threshold, 16 Bits
			BOVL = 0x0E,			//Bus Overvoltage Threshold, 16 Bits
			BUVL = 0x0F,			//Bus Undervoltage Threshold, 16 Bits
			TEMP_LIMIT = 0x10,		//Temperature Over-Limit Threshold, 16 Bits
			PWR_LIMIT = 0x11,		//Power Over-Limit Threshold, 16 Bits
			MANUFACTURER_ID = 0x3E	//Manufacturer ID, 16 Bits
		};

		/// @brief INA700 sensor configuration structure.
		struct Config {
		};

		/// @brief Constructor.
		/// @param i2c	Reference to the low-level bus driver.
		/// @param addr	Bus address.
		INA700(I2C& i2c, uint8_t addr) : bus(i2c), addr(addr) {};

		/// @brief Initializes the INA700 power monitor.
		/// @param config INA700 configuration.
		/// @return Status::Ok if initialization succeeded, or Status::Error if the config was invalid or failed.
		Status Init(const Config& config);

		/// @brief Resets the sensor device, using software reset.
		/// @return Status::Ok if reset succeeded cleanly.
		Status Reset();

		/// @brief Reads the Manufacturer and Device IDs.
		/// @param id Device ID, or manufacturer ID.
		/// @return Status::Ok if read succeeded, or Status::Error if failed.
		Status ReadID(uint16_t& id);

		/// @brief Start a non-blocking data transfer request (calls TransferAsync of the underlying bus).
		/// @details Returns immediately. Use TransferWait() to synchronize completion.
		/// @return Status::Ok if the transfer started, or Status::Busy if the bus is locked by another thread.
		// Status RequestData();

		/// @brief Blocks the current thread until data transfer completes.
		/// @param volt	Measured bus voltage.
		/// @param curr	Measured bus current.
		/// @param temp	Measured chip die temperature. 
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		// Status GetData(float& volt, float& curr, float& temp);

		/// @brief Blocking read of current bus voltage.
		/// @param value Measured bus voltage.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status ReadVoltage(float& value);

		/// @brief Blocking read of current bus current.
		/// @param value Measured bus current.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status ReadCurrent(float& value);

		/// @brief Blocking read of current chip die temperature.
		/// @param value Measured chip die temperature.
		/// @return Status::Ok if set succeeded, or Status::Error if failed.
		Status ReadTemperature(float& value);

	private:
		I2C& bus;
		const uint8_t addr;
		Config config;

		static constexpr uint16_t transferSize = 32;
		__attribute__((aligned(32))) uint8_t buffer[transferSize];

		static constexpr float voltSens = 0.003125f;	// 3.125mV/LSB
		static constexpr float currSens = 0.000480f;	// 480μA/LSB
		static constexpr float tempSens = 0.0078125f;	// 125 m°C/LSB or 7.8125m°C/LSB (0.0078125) ??
		static constexpr float powerSens = 0.000096f;	// 96 μW/LSB
		static constexpr float energySens = 0.001536f;	// 1.536 mJ/LSB
		static constexpr float chargeSens = 0.00003f;	// 30 μC/LSB
		
		Status WriteRegister(Register reg, uint16_t value);
		Status ReadRegister(Register reg, uint16_t& value);
		Status ModifyRegister(Register reg, uint16_t mask, uint16_t value);
};