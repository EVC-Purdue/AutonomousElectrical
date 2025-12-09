#include "logic.h"

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "motor.h"
#include "steering.h"
#include "stm32f4xx_hal.h"

// Kart control --------------------------------------------------------------//
static bool gs_contactor_on          = false;
// static volatile uint32_t gs_contactor_last_rx = 0;
static uint32_t gs_boot_time = 0;

// SPI communication ---------------------------------------------------------//
static uint8_t gs_rx_buff[SPI_MSG_SIZE] = {0};
static uint8_t gs_tx_buff[SPI_MSG_SIZE] = {0};

// RC E-stop PWM interrupt ---------------------------------------------------//
// static volatile uint32_t gs_tim5_ic_rising = 0;
// static volatile bool gs_tim5_capturing     = false;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    // if ((htim->Instance == TIM5) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)) {
    //     if (!gs_tim5_capturing) { // waiting for rising edge
    //         gs_tim5_ic_rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    //         // Switch to capture falling edge
    //         __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    //         gs_tim5_capturing = true;
    //     } else { // waiting for falling edge
    //         uint32_t ic_falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    //         // Handle counter rollover
    //         uint32_t pulse_width_us;
    //         if (ic_falling >= gs_tim5_ic_rising) {
    //             pulse_width_us = ic_falling - gs_tim5_ic_rising;
    //         } else {
    //             pulse_width_us = (0xFFFFFFFF - gs_tim5_ic_rising) + ic_falling + 1;
    //         }

    //         // If the contactor switch state has changed, update GPIO
    //         // Don't update it every time to avoid unnecessary writes
    //         bool new_contactor_on = (pulse_width_us >= ESTOP_PULSE_WIDTH);
    //         if (new_contactor_on != gs_contactor_on) {
    //             gs_contactor_on = new_contactor_on;
    //             HAL_GPIO_WritePin(CONTACTOR_GPIO_Port,
    //                 CONTACTOR_Pin,
    //                 gs_contactor_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    //         }

    //         // Switch back to capture rising edge
    //         __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    //         gs_tim5_capturing = false;

    //         // Update last rx time
    //         gs_contactor_last_rx = HAL_GetTick();
    //     }
    // }
}

// logic_init() --------------------------------------------------------------//
void logic_init() {
    // Turn off LED initially
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    // Set contactor pin low
    HAL_GPIO_WritePin(CONTACTOR_GPIO_Port, CONTACTOR_Pin, GPIO_PIN_RESET);
    gs_boot_time = HAL_GetTick();
    // Initialize last E-stop pulse timestamp to current tick
    // gs_contactor_last_rx = HAL_GetTick();
}

// logic_run() ---------------------------------------------------------------//
// uint16_t pwm_values[]    = {1000, 1200};
// const size_t pwm_values_len = 2;
// uint8_t pwm_values_idx   = 0;
// uint8_t b1_last_state    = 0;
// uint32_t last_press_tick = 0;

uint32_t last_motor_pwm = MOTOR_PWM_PULSE_MIN;

void logic_run(SPI_HandleTypeDef* hspi2, // Rubik Pi 3 <-> STM32 SPI handle
    TIM_HandleTypeDef* htim2_motor,      // Motor PWM TIM handle
    TIM_HandleTypeDef* htim3_steering    // Steering servo PWM TIM handle
) {
    // uint8_t button_state = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

    // uint32_t now = HAL_GetTick();
    // if (button_state == GPIO_PIN_SET && b1_last_state == GPIO_PIN_RESET) {
    //     if (now - last_press_tick > 200) // debounce
    //     {
    //         pwm_values_idx = (pwm_values_idx + 1) % pwm_values_len;

    //         __HAL_TIM_SET_COMPARE(htim2_motor, TIM_CHANNEL_1, pwm_values[pwm_values_idx]);
    //         HAL_GPIO_WritePin(LD2_GPIO_Port,
    //                             LD2_Pin,
    //                             pwm_values_idx == 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    //         last_press_tick = now;
    //     }
    // }

    // b1_last_state = button_state;


    // SPI communication
    HAL_StatusTypeDef spi_stat =
        HAL_SPI_TransmitReceive(hspi2, gs_tx_buff, gs_rx_buff, SPI_MSG_SIZE, SPI_TIMEOUT_MS);

    bool spi_okay = (spi_stat == HAL_OK);

    // Read contactor state in a critical section
    // __disable_irq();
    // bool contactor_on_local = gs_contactor_on;
    // __enable_irq();

    uint32_t now_ms = HAL_GetTick();
    if (now_ms - gs_boot_time < 2000) {
        // During first 2s after boot, force contactor off
        gs_contactor_on = false;
    } else {
        gs_contactor_on = true;
    }

    // HAL_GPIO_WritePin(LD2_GPIO_Port,
    //     LD2_Pin,
    //     gs_contactor_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONTACTOR_GPIO_Port,
        CONTACTOR_Pin,
        gs_contactor_on ? GPIO_PIN_SET : GPIO_PIN_RESET);

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

    if (spi_okay && gs_contactor_on) {
        // Process received data
        // 0-(2^16-1) = 0-100% speed = 1000-2000 PWM pulse
        uint16_t motor_unscaled = ((uint16_t)gs_rx_buff[0] << 8) | (uint16_t)gs_rx_buff[1];
        // 0-(2^16-1) = -100% to 100% = 1000-2000 PWM pulse
        uint16_t steering_unscaled = ((uint16_t)gs_rx_buff[2] << 8) | (uint16_t)gs_rx_buff[3];

        // Set motor PWM
        uint32_t motor_pwm = motor_unscaled_to_pwm(motor_unscaled);

        // No REGEN RN
        uint32_t allowed_min = last_motor_pwm - 20;
        if (allowed_min < MOTOR_PWM_PULSE_MIN) {
            allowed_min = MOTOR_PWM_PULSE_MIN;
        }
        if (motor_pwm < allowed_min) {
            motor_pwm = allowed_min;
        }
        last_motor_pwm = motor_pwm;
        __HAL_TIM_SET_COMPARE(htim2_motor, TIM_CHANNEL_1, motor_pwm);
        // Set rx buffer with pwm value for confirmation
        gs_tx_buff[0] = (motor_pwm >> 8) & 0xFF;
        gs_tx_buff[1] = motor_pwm & 0xFF;

        // Set steering servo PWM
        uint32_t steering_pwm = steering_unscaled_to_pwm(steering_unscaled);
        // __HAL_TIM_SET_COMPARE(htim3_steering, TIM_CHANNEL_1, steering_pwm);
        // NO STEERINT TODAY :(
        __HAL_TIM_SET_COMPARE(htim3_steering, TIM_CHANNEL_1, 1500);
        // Set rx buffer with pwm value for confirmation
        gs_tx_buff[2] = (steering_pwm >> 8) & 0xFF;
        gs_tx_buff[3] = steering_pwm & 0xFF;
    } else {
        // Attempt to stop the vehicle on SPI error
        // No force stop on b/c no REGEN
        // __HAL_TIM_SET_COMPARE(htim2_motor, TIM_CHANNEL_1, MOTOR_PWM_PULSE_MIN);

        // Set tx buffer to 0xFF (never possible to achieve normally) on SPI error
        for (size_t i = 0; i < SPI_MSG_SIZE; i++) {
            gs_tx_buff[i] = 0xFF;
        }
    }
}
