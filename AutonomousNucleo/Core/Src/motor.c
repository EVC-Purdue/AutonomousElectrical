#include "motor.h"

#include <stdint.h>

// 0-(2^16-1) = 0-100% speed = 1000-2000 PWM pulse
uint32_t motor_unscaled_to_pwm(uint16_t motor_unscaled) {
	uint32_t pulse_range = MOTOR_PWM_PULSE_MAX - MOTOR_PWM_PULSE_MIN;
	uint32_t raw = (uint32_t)motor_unscaled;
	return MOTOR_PWM_PULSE_MIN + ((raw * (pulse_range)) >> 16);
}