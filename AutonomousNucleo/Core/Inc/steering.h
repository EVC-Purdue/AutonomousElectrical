#ifndef STEERING_H
#define STEERING_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define STEERING_MIN_PULSE_US  (500)
#define STEERING_MAX_PULSE_US  (2500)
#define STEERING_MIN_ANGLE_DEG (0)
#define STEERING_MAX_ANGLE_DEG (360)

void steering_set_pulse(TIM_HandleTypeDef* htim2, uint16_t us);
uint16_t steering_angle_to_pulse(int16_t angle);
void steering_set_servo_angle(TIM_HandleTypeDef* htim2, int16_t angle);

#endif