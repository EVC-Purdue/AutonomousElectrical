#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_PWM_MIN (1000)
#define MOTOR_PWM_MAX (2000)


// 0-(2^16-1) = 0-100% speed = 1000-2000 PWM pulse
uint32_t motor_unscaled_to_pwm(uint16_t motor_unscaled);



#endif