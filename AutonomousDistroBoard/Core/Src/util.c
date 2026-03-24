#include "util.h"

#include <stdint.h>
#include <stdbool.h>

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