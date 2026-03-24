#include "util.h"

#include <stdint.h>
#include <stdbool.h>

bool util_has_elapsed(uint32_t now, uint32_t start, uint32_t duration_ms) {
	return (uint32_t)(now - start) >= duration_ms;
}

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