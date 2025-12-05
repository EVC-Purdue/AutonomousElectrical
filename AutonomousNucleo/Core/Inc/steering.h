#ifndef STEERING_H
#define STEERING_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define STEERING_PWM_PULSE_MIN (1000)
#define STEERING_PWM_PULSE_MAX (2000)

// 0-(2^16-1) = -100% to 100% = 1000-2000 PWM pulse
uint32_t steering_unscaled_to_pwm(uint16_t motor_unscaled);


#endif