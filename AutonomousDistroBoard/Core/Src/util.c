#include "util.h"

#include <stdint.h>
#include <stdbool.h>

//----------------------------------------------------------------------------//
bool util_has_elapsed(uint32_t now, uint32_t start, uint32_t duration_ms) {
	return (uint32_t)(now - start) >= duration_ms;
}
uint16_t clamp_u16(uint16_t value, uint16_t min, uint16_t max) {
	if (value < min) {
		return min;
	} else if (value > max) {
		return max;
	} else {
		return value;
	}
}

uint16_t clamp_i32(int32_t value, int32_t min, int32_t max) {
	if (value < min) {
		return (uint16_t)min;
	} else if (value > max) {
		return (uint16_t)max;
	} else {
		return (uint16_t)value;
	}
}

uint16_t map_u16(uint16_t value, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
	if (in_max == in_min) {
		// Avoid division by zero; return the midpoint of the output range
		return (out_min + out_max) / 2;
	}

	// Saturate value to input range to avoid underflow/overflow in arithmetic
	if (value <= in_min) {
		return out_min;
	}
	if (value >= in_max) {
		return out_max;
	}

	uint32_t range_in  = (uint32_t)in_max  - (uint32_t)in_min;
	uint32_t range_out = (uint32_t)out_max - (uint32_t)out_min;
	uint32_t num       = (uint32_t)value  - (uint32_t)in_min;
	uint32_t scaled    = num * range_out;

	return (uint16_t)((scaled + range_in / 2) / range_in + out_min);
}

int32_t map_i32(int32_t value, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
	if (in_max == in_min) {
		// Avoid division by zero; return the midpoint of the output range
		return (out_min + out_max) / 2;
	}

	// Saturate value to input range to avoid underflow/overflow in arithmetic
	if (in_min < in_max) {
		if (value <= in_min) return out_min;
		if (value >= in_max) return out_max;
	} else {
		if (value >= in_min) return out_min;
		if (value <= in_max) return out_max;
	}

	int64_t range_in  = (int64_t)in_max  - (int64_t)in_min;
	int64_t range_out = (int64_t)out_max - (int64_t)out_min;
	int64_t num       = (int64_t)value  - (int64_t)in_min;
	int64_t scaled    = num * range_out;

	int64_t offset;
	if ((scaled >= 0) == (range_in >= 0)) {
		offset = range_in / 2;
	} else {
		offset = -(range_in / 2);
	}

	return (int32_t)((scaled + offset) / range_in + out_min);
}
//----------------------------------------------------------------------------//


// option_u32_t --------------------------------------------------------------//
option_u32_t option_u32_some(uint32_t value) {
	option_u32_t opt = {
		.is_some = true,
		.value = value
	};
	return opt;
}
option_u32_t option_u32_none(void) {
	option_u32_t opt = {
		.is_some = false,
		.value = 0 // value is irrelevant when is_some is false
	};
	return opt;
}
bool option_u32_is_some(option_u32_t opt) {
	return opt.is_some;
}
bool option_u32_is_none(option_u32_t opt) {
	return !opt.is_some;
}
uint32_t option_u32_unwrap(option_u32_t opt) {
	// Only call this if opt is known to be Some, otherwise behavior is undefined
	return opt.value;
}
//----------------------------------------------------------------------------//


// debounce_controller_t -----------------------------------------------------//
void debounce_controller_init(
	debounce_controller_t* controller,
	bool initial_state,
	uint32_t rising_debounce_ms,
	uint32_t falling_debounce_ms,
	uint32_t accumulating_debounce_ms
) {
	controller->stable_state = initial_state;
	controller->transition_start_time = option_u32_none();
	controller->interruption_start_time = option_u32_none();
	controller->rising_debounce_ms = rising_debounce_ms;
	controller->falling_debounce_ms = falling_debounce_ms;
	controller->accumulating_debounce_ms = accumulating_debounce_ms;
}

void debounce_controller_reset(
	debounce_controller_t* controller,
	bool state
) {
	controller->stable_state = state;
	controller->transition_start_time = option_u32_none();
	controller->interruption_start_time = option_u32_none();
}

bool debounce_controller_update(
	debounce_controller_t* controller,
	bool raw_state,
	uint32_t now
) {
	if (raw_state == controller->stable_state) {
		if (option_u32_is_some(controller->transition_start_time)) {
			if (option_u32_is_none(controller->interruption_start_time)) {
				controller->interruption_start_time = option_u32_some(now);
			} else if (util_has_elapsed(now, option_u32_unwrap(controller->interruption_start_time), controller->accumulating_debounce_ms)) {
				controller->transition_start_time = option_u32_none();
				controller->interruption_start_time = option_u32_none();
			}
		}

		return controller->stable_state;
	}

	if (option_u32_is_none(controller->transition_start_time)) {
		controller->transition_start_time = option_u32_some(now);
	}

	controller->interruption_start_time = option_u32_none();

	uint32_t debounce_ms = raw_state
		? controller->rising_debounce_ms
		: controller->falling_debounce_ms;

	if (util_has_elapsed(now, option_u32_unwrap(controller->transition_start_time), debounce_ms)) {
		controller->stable_state = raw_state;
		controller->transition_start_time = option_u32_none();
		controller->interruption_start_time = option_u32_none();
	}

	return controller->stable_state;
}

bool debounce_controller_get_state(const debounce_controller_t* controller) {
	return controller->stable_state;
}
//----------------------------------------------------------------------------//