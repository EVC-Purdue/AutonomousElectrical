#ifndef LOGIC_H
#define LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define CONTACTER_SET_DELAY (1000) // 1 sec after boot, set contacter state
#define SPI_MSG_SIZE        (4)    // 2 bytes for motor speed, 2 bytes for steering angle
#define SPI_TIMEOUT_MS      (100)  // If no message in this time, set speed to 0

void logic_init(void);
void logic_run(
	SPI_HandleTypeDef* hspi2, // Rubik Pi 3 <-> STM32 SPI handle
	TIM_HandleTypeDef* htim2_motor, // Motor PWM TIM handle
	TIM_HandleTypeDef* htim3_steering // Steering servo PWM TIM handle
);

#endif