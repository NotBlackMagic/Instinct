#pragma once

#include <stdint.h>

struct Vector3f {
	float x, y, z;
};

struct Quaternion {
	float x, y, z, w;
};

using Timestamp = uint64_t;