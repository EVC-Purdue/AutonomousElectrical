#ifndef STEERING_H
#define STEERING_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

// #define STEERING_PWM_PULSE_MIN  (500) // at 0 degrees
// #define STEERING_PWM_PULSE_MAX  (2500) // at 360 degrees
#define STEERING_PWM_PULSE_MIN  (1000) // at 90 degrees
#define STEERING_PWM_PULSE_MAX  (2000) // at 270 degrees
// #define STEERING_MIN_ANGLE_DEG  (90)
// #define STEERING_MAX_ANGLE_DEG  (270)

// 0-(2^16-1) = 90-270 degrees servo angle = 1000-2000 PWM pulse
uint32_t steering_unscaled_to_pwm(uint16_t motor_unscaled);


#endif