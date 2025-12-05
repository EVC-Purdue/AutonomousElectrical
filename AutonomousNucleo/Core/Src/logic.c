#include "logic.h"

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "motor.h"
#include "steering.h"
#include "stm32f4xx_hal.h"

// SPI communication ---------------------------------------------------------//
static uint8_t gs_rx_buff[SPI_MSG_SIZE] = {0};
static uint8_t gs_tx_buff[SPI_MSG_SIZE] = {0xAB, 0xCD, 0xEF, 0x01};

// RC E-stop PWM interupt ----------------------------------------------------//
static volatile uint32_t gs_tim5_ic_rising      = 0;
static volatile uint32_t gs_tim5_pulse_width_us = 0;
static volatile bool gs_tim5_capturing          = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        if (!gs_tim5_capturing) { // waiting for rising edge
            gs_tim5_ic_rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
            gs_tim5_capturing = true;
        } else { // waiting for falling edge
            uint32_t ic_falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            // Handle counter rollover
            if (ic_falling >= gs_tim5_ic_rising) {
                gs_tim5_pulse_width_us = ic_falling - gs_tim5_ic_rising;
            } else {
                gs_tim5_pulse_width_us = (0xFFFF - gs_tim5_ic_rising) + ic_falling + 1;
            }
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            gs_tim5_capturing = false;
        }
    }
}

// logic_init() --------------------------------------------------------------//
void logic_init() {
    // Turn off LED initially
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    // TODO: set contactor pin low
}

// logic_run() ---------------------------------------------------------------//
void logic_run(SPI_HandleTypeDef* hspi2, // Rubik Pi 3 <-> STM32 SPI handle
    TIM_HandleTypeDef* htim2_motor,      // Motor PWM TIM handle
    TIM_HandleTypeDef* htim3_steering    // Steering servo PWM TIM handle
) {
    // SPI communication
    HAL_StatusTypeDef spi_stat =
        HAL_SPI_TransmitReceive(hspi2, gs_tx_buff, gs_rx_buff, SPI_MSG_SIZE, SPI_TIMEOUT_MS);

    bool spi_okay = (spi_stat == HAL_OK);
    bool contactor_on = (gs_tim5_pulse_width_us >= ESTOP_PULSE_WIDTH);

    // Set LED state
    if (spi_okay) {
        // Loop rate is bound/delayed by blocking SPI call
        // Toggle basically on SPI message
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    } else {
        // Solid green to show ON but no SPI
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    }

    // Set contactor state
    // TODO

    if (spi_okay) {
        // Process received data
        // 0-(2^16-1) = 0-100% speed = 1000-2000 PWM pulse
        uint16_t motor_unscaled = ((uint16_t)gs_rx_buff[0] << 8) | (uint16_t)gs_rx_buff[1];
        // 0-(2^16-1) = -100% to 100% = 1000-2000 PWM pulse
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
