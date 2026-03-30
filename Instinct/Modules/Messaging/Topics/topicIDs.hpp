#pragma once

#include <stdint.h>

enum class TopicID : uint8_t {
	// Inertial Sensors
	Accel = 1,
	Gyro = 2,
	Mag = 3,
	Baro = 4,
	Imu = 5,

	// GNSS Sensors
	NavSat = 10,
	NavRTCM = 11,

	// Environment/Ranging Sensors
	Range = 20,
	Tacho = 21,

	// Power Sensors
	Power = 30,
	Battery = 31,

	// Inputs
	RC = 40
};