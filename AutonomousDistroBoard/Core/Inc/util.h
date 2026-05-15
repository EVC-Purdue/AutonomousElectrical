#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include <stdbool.h>


bool util_has_elapsed(uint32_t now, uint32_t start, uint32_t duration_ms);
uint16_t clamp_u16(uint16_t value, uint16_t min, uint16_t max);
int32_t clamp_i32(int32_t value, int32_t min, int32_t max);
uint16_t map_u16(uint16_t value, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
int32_t map_i32(int32_t value, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int32_t max_i32(int32_t a, int32_t b);
int32_t min_i32(int32_t a, int32_t b);


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
// Noise-tolerant debounce controller for signals, with separate debounce
// times for rising and falling edges and an optional accumulating debounce feature
// to help filter out noise that causes multiple rapid transitions
typedef int debounce_state_t;
typedef struct {
	debounce_state_t stable_state; // current accepted state
	option_u32_t transition_start_time; // when a possible transition began
	option_u32_t interruption_start_time; // when signal temporarily flipped back
	uint32_t debounce_ms; // configuration parameter: how long the signal needs to be continuous in the state before accepting the transition
	uint32_t accumulating_debounce_ms; // If a transition toward a new candidate state is in progress and the
									   // signal temporarily returns to the currently stable state, require it
									   // to remain in the stable state for this duration before abandoning the
									   // in-progress transition. Helps reject noisy oscillations.
} debounce_controller_t;

void debounce_controller_init(
	debounce_controller_t* controller,
	debounce_state_t initial_state,
	uint32_t debounce_ms,
	uint32_t accumulating_debounce_ms
);

void debounce_controller_reset(
	debounce_controller_t* controller,
	debounce_state_t state
);

debounce_state_t debounce_controller_update(
	debounce_controller_t* controller,
	debounce_state_t raw_state,
	uint32_t now
);

debounce_state_t debounce_controller_get_state(const debounce_controller_t* controller);
//----------------------------------------------------------------------------//


#endif // UTIL_H