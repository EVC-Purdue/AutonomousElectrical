#include "util.h"

#include <stdint.h>


int16_t clamp_i16(int16_t value, int16_t min, int16_t max) {
	if (value < min) {
		return min;
	} else if (value > max) {
		return max;
	} else {
		return value;
	}
}

uint32_t clamp_u32(uint32_t value, uint32_t min, uint32_t max) {
	if (value < min) {
		return min;
	} else if (value > max) {
		return max;
	} else {
		return value;
	}
}