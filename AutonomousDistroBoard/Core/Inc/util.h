#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include <stdbool.h>


// Option<uint32_t> type
typedef struct {
	bool is_some; // true if the option contains a value, false if it is None
	uint32_t value;
} option_u32_t;

option_u32_t option_u32_some(uint32_t value);
option_u32_t option_u32_none(void);
bool option_u32_is_some(option_u32_t opt);
bool option_u32_is_none(option_u32_t opt);
uint32_t option_u32_unwrap(option_u32_t opt); // only call if opt is known to be Some!!



#endif // UTIL_H