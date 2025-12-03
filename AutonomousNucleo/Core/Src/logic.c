#include "logic.h"

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "motor.h"
#include "steering.h"
#include "stm32f4xx_hal.h"

static uint8_t gs_rx_buff[SPI_MSG_SIZE] = {0};
static uint8_t gs_tx_buff[SPI_MSG_SIZE] = {0};

void logic_init() {
    // Turn off LED initially
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void logic_run(SPI_HandleTypeDef* hspi2, // Rubik Pi 3 <-> STM32 SPI handle
    TIM_HandleTypeDef* htim2_motor,      // Motor PWM TIM handle
    TIM_HandleTypeDef* htim3_steering    // Steering servo PWM TIM handle
) {
    // SPI communication
    HAL_StatusTypeDef spi_stat =
        HAL_SPI_TransmitReceive(hspi2, gs_tx_buff, gs_rx_buff, SPI_MSG_SIZE, SPI_TIMEOUT_MS);

    bool spi_okay = (spi_stat == HAL_OK);

    // Set LED state
    if (spi_okay) {
        // Loop rate is bound/delayed by blocking SPI call
        // Toggle basically on SPI message
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    } else {
        // Solid green to show ON but no SPI
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    }

    if (spi_okay) {
        // Process received data
        // 0-(2^16-1) = 0-100% speed = 1000-2000 PWM pulse
        uint16_t motor_unscaled = ((uint16_t)gs_rx_buff[0] << 8) | (uint16_t)gs_rx_buff[1];
        // 0-(2^16-1) = -90 to 90 degrees steering angle = 90-270 degrees servo angle
        uint16_t steering_unscaled = ((uint16_t)gs_rx_buff[2] << 8) | (uint16_t)gs_rx_buff[3];

        // Set motor PWM
        uint32_t motor_pwm = motor_unscaled_to_pwm(motor_unscaled);
        __HAL_TIM_SET_COMPARE(htim2_motor, TIM_CHANNEL_1, motor_pwm);
        // Set rx buffer with pwm value for confirmation
        gs_tx_buff[0] = (motor_pwm >> 8) & 0xFF;
        gs_tx_buff[1] = motor_pwm & 0xFF;

        // Set steering servo PWM
        uint32_t steering_pwm = steering_unscaled_to_pwm(steering_unscaled);
        __HAL_TIM_SET_COMPARE(htim3_steering, TIM_CHANNEL_1, steering_pwm);
        // Set rx buffer with pwm value for confirmation
        gs_tx_buff[2] = (steering_pwm >> 8) & 0xFF;
        gs_tx_buff[3] = steering_pwm & 0xFF;
    } else {
        // Attempt to stop the vechile on SPI error
        __HAL_TIM_SET_COMPARE(htim2_motor, TIM_CHANNEL_1, MOTOR_PWM_PULSE_MIN);
        __HAL_TIM_SET_COMPARE(htim3_steering, TIM_CHANNEL_1, 0); // Stop?

        // Set tx buffer to 0xFF (never possible to achieve normally) on SPI error
        for (size_t i = 0; i < SPI_MSG_SIZE; i++) {
            gs_tx_buff[i] = 0xFF;
        }
    }
}
