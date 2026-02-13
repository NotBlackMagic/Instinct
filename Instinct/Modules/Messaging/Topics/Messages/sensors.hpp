#pragma once

#include "Common.hpp"

struct AccelMsg {
	Timestamp timestamp;	//Microseconds
	float values[3];		//m/s^2
	float temperature;		//degrees Celsius
};

struct GyroMsg {
	Timestamp timestamp;	//Microseconds
	float values[3];		//rad/s
	float temperature;		//degrees Celsius
};

struct MagMsg {
	Timestamp timestamp;	//Microseconds
	float values[3];		//Gauss
	float temperature;		//degrees Celsius
};

struct BaroMsg {
	Timestamp timestamp;	//Microseconds
	float pressure;			//Pascal
	float temperature;		//degrees Celsius
};

struct ImuMsg {
	Timestamp timestamp;	//Microseconds
	float accel[3];			//m/s^2
	float gyro[3];			//rad/s
	float temperature;		//degrees Celsius
};

struct NavSatMsg {
	Timestamp timestamp;	//Microseconds
	int8_t fixType;			//GNSS fix status, > 0 has fix
	uint8_t service;		//Used services, used with bitmask
	uint8_t numSats;		//Number of satellites used
	double lat;				//In degrees, positive is north of equator; negative is south.
	double lon;				//In degrees, positive is east of prime meridian; negative is west.
	float alt;				//In meters, positive is above the WGS 84 ellipsoid
	float hdop;				//Horizontal dilution
};