#include "logic.h"

#include <stdbool.h>
#include <stdint.h>

#include "motor.h"

#include "stm32f4xx_hal.h"

static uint32_t boot_time   = 0;
static bool contacter_state = false;

static uint8_t rx_buff[SPI_MSG_SIZE] = {0};
static uint8_t tx_buff[SPI_MSG_SIZE] = {0};

void logic_init() {
    boot_time = HAL_GetTick();
}

void logic_run(
	SPI_HandleTypeDef* hspi2, // Rubik Pi 3 <-> STM32 SPI handle
	TIM_HandleTypeDef* htim2_motor // Motor PWM TIM handle
) {
    uint32_t now = HAL_GetTick();
    if (now - boot_time >= CONTACTER_SET_DELAY) {
        contacter_state = true;
    }

    // Set contacter
    // TODO

    // SPI communication
    HAL_StatusTypeDef spi_stat =
        HAL_SPI_TransmitReceive(hspi2, tx_buff, rx_buff, SPI_MSG_SIZE, SPI_TIMEOUT_MS);

	if (spi_stat != HAL_OK) { // timeout hit
		// Force set speed to 0
		tx_buff[0] = 0;
		tx_buff[1] = 0;
	}

	// Process received data
	// 0-(2^16-1) = 0-100% speed = 1000-2000 PWM pulse
	uint16_t motor_unscaled = ((uint16_t)rx_buff[0] << 8) | (uint16_t)rx_buff[1];
	// 0-(2^16-1) = -90 to 90 degrees steering angle = 90-270 degrees servo angle
	uint16_t steering_unscaled = ((uint16_t)rx_buff[2] << 8) | (uint16_t)rx_buff[3];

	// Set motor PWM
	uint32_t motor_pwm = motor_unscaled_to_pwm(motor_unscaled);
	__HAL_TIM_SET_COMPARE(htim2_motor, TIM_CHANNEL_1, motor_pwm);

	// Set steering servo PWM
	// TODO
}
