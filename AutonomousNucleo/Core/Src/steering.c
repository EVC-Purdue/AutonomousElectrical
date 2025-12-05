#include "steering.h"

#include <stdint.h>

// 0-(2^16-1) = -100% to 100% = 1000-2000 PWM pulse
uint32_t steering_unscaled_to_pwm(uint16_t motor_unscaled) {
    uint32_t pulse_range = STEERING_PWM_PULSE_MAX - STEERING_PWM_PULSE_MIN;
    uint32_t raw = (uint32_t)motor_unscaled;
    return STEERING_PWM_PULSE_MIN + ((raw * pulse_range) >> 16);
}

