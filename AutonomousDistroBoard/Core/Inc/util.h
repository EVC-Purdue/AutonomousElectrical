#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include <stdbool.h>


bool has_elapsed(uint32_t now, uint32_t start, uint32_t duration_ms);


// Option<uint32_t> type -----------------------------------------------------//
typedef struct {
	bool is_some; // true if the option contains a value, false if it is None
	uint32_t value;
} option_u32_t;

option_u32_t option_u32_some(uint32_t value);
option_u32_t option_u32_none(void);
bool option_u32_is_some(option_u32_t opt);
bool option_u32_is_none(option_u32_t opt);
uint32_t option_u32_unwrap(option_u32_t opt); // only call if opt is known to be Some!!
//----------------------------------------------------------------------------//


// Debounce controller -------------------------------------------------------//
typedef struct {
	bool stable_state;
	option_u32_t transition_start_time;
	option_u32_t interruption_start_time;
	uint32_t rising_debounce_ms;
	uint32_t falling_debounce_ms;
	uint32_t accumulating_debounce_ms;
} debounce_controller_t;

void debounce_controller_init(
	debounce_controller_t* controller,
	bool initial_state,
	uint32_t rising_debounce_ms,
	uint32_t falling_debounce_ms,
	uint32_t accumulating_debounce_ms
);

void debounce_controller_reset(
	debounce_controller_t* controller,
	bool state
);

bool debounce_controller_update(
	debounce_controller_t* controller,
	bool raw_state,
	uint32_t now
);

bool debounce_controller_get_state(const debounce_controller_t* controller);
//----------------------------------------------------------------------------//


#endif // UTIL_H