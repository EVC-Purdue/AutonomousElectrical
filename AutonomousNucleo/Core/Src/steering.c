#include "steering.h"

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "util.h"

void steering_set_pulse(TIM_HandleTypeDef* htim2, uint16_t us) {
    uint32_t pulse = clamp_u32(us, STEERING_MIN_PULSE_US, STEERING_MAX_PULSE_US);
    __HAL_TIM_SET_COMPARE(htim2, TIM_CHANNEL_1, pulse);
}

uint16_t steering_angle_to_pulse(int16_t angle) {
    int16_t clamped_angle     = clamp_i16(angle, STEERING_MIN_ANGLE_DEG, STEERING_MAX_ANGLE_DEG);
    uint16_t pulse_width_diff = STEERING_MAX_PULSE_US - STEERING_MIN_PULSE_US;
    uint16_t angle_range      = STEERING_MAX_ANGLE_DEG - STEERING_MIN_ANGLE_DEG;
    return STEERING_MIN_PULSE_US
        + (pulse_width_diff * (clamped_angle - STEERING_MIN_ANGLE_DEG)) / angle_range;
}

void steering_set_servo_angle(TIM_HandleTypeDef* htim2, int16_t angle) {
    uint16_t pulse = steering_angle_to_pulse(angle);
    steering_set_pulse(htim2, pulse);
}