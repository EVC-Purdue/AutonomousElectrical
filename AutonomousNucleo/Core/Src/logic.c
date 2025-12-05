#include "logic.h"

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "motor.h"
#include "steering.h"
#include "stm32f4xx_hal.h"

// Kart control --------------------------------------------------------------//
static volatile bool gs_contactor_on                   = false;
static volatile uint32_t gs_contactor_last_rx = 0;

// SPI communication ---------------------------------------------------------//
static uint8_t gs_rx_buff[SPI_MSG_SIZE] = {0};
static uint8_t gs_tx_buff[SPI_MSG_SIZE] = {0};

// RC E-stop PWM interupt ----------------------------------------------------//
static volatile uint32_t gs_tim5_ic_rising      = 0;
static volatile uint32_t gs_tim5_pulse_width_us = 0;
static volatile bool gs_tim5_capturing          = false;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    if ((htim->Instance == TIM5) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)) {
        if (!gs_tim5_capturing) { // waiting for rising edge
            gs_tim5_ic_rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            // Switch to capture falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
            gs_tim5_capturing = true;
        } else { // waiting for falling edge
            uint32_t ic_falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            // Handle counter rollover
            uint32_t pulse_width_us;
            if (ic_falling >= gs_tim5_ic_rising) {
                pulse_width_us = ic_falling - gs_tim5_ic_rising;
            } else {
                pulse_width_us = (0xFFFF - gs_tim5_ic_rising) + ic_falling + 1;
            }

            // If the the contactor switch state has changed, update GPIO
            // Don't update it every time to avoid unnecessary writes
            bool new_contactor_on = (pulse_width_us >= ESTOP_PULSE_WIDTH);
            if (new_contactor_on != gs_contactor_on) {
                gs_contactor_on = new_contactor_on;
                HAL_GPIO_WritePin(CONTACTOR_GPIO_Port,
                    CONTACTOR_Pin,
                    gs_contactor_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }

            // Switch back to capture rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            gs_tim5_capturing = false;

            // Update last rx time
            gs_contactor_last_rx = HAL_GetTick();
        }
    }
}

// logic_init() --------------------------------------------------------------//
void logic_init() {
    // Turn off LED initially
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    // Set contactor pin low
    HAL_GPIO_WritePin(CONTACTOR_GPIO_Port, CONTACTOR_Pin, GPIO_PIN_RESET);
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

    // Set LED state
    if (gs_contactor_on && spi_okay) {
        // Loop rate is bound/delayed by blocking SPI call
        // Toggle basically on SPI message if all is good
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    } else if (gs_contactor_on) {
        // Solid green to show contactor is on but SPI error
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    } else {
        // Contactor off, SPI is either
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }

    // If no valid E-stop pulse recently, open contactor
    uint32_t now_ms = HAL_GetTick();
    if ((now_ms - gs_contactor_last_rx) > ESTOP_TIMEOUT_MS) {
        gs_contactor_on   = false;
        gs_tim5_capturing = false; // reset capturing state
    }
    // Always set contactor GPIO based on current state for consistency
    HAL_GPIO_WritePin(CONTACTOR_GPIO_Port,
        CONTACTOR_Pin,
        gs_contactor_on ? GPIO_PIN_SET : GPIO_PIN_RESET);

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

        // Set tx buffer to 0xFF (never possible to achieve normally) on SPI error
        for (size_t i = 0; i < SPI_MSG_SIZE; i++) {
            gs_tx_buff[i] = 0xFF;
        }
    }
}
