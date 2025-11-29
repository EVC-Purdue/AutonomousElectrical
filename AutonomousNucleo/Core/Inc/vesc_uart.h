#if 0

#ifndef VESC_UART_H
#define VESC_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

// VESC COMM_PACKET_ID (command IDs)
typedef enum {
    COMM_FW_VERSION                = 0,
    COMM_JUMP_TO_BOOTLOADER        = 1,
    COMM_ERASE_NEW_APP             = 2,
    COMM_WRITE_NEW_APP_DATA        = 3,
    COMM_GET_VALUES                = 4,
    COMM_SET_DUTY                  = 5,
    COMM_SET_CURRENT               = 6,
    COMM_SET_CURRENT_BRAKE         = 7,
    COMM_SET_RPM                   = 8,
    COMM_SET_POS                   = 9,
    COMM_SET_HANDBRAKE             = 10,
    COMM_SET_DETECT                = 11,
    COMM_SET_SERVO_POS             = 12,
    COMM_SET_MCCONF                = 13,
    COMM_GET_MCCONF                = 14,
    COMM_GET_MCCONF_DEFAULT        = 15,
    COMM_SET_APPCONF               = 16,
    COMM_GET_APPCONF               = 17,
    COMM_GET_APPCONF_DEFAULT       = 18,
    COMM_SAMPLE_PRINT              = 19,
    COMM_TERMINAL_CMD              = 20,
    COMM_PRINT                     = 21,
    COMM_ROTOR_POSITION            = 22,
    COMM_EXPERIMENT_SAMPLE         = 23,
    COMM_DETECT_MOTOR_PARAM        = 24,
    COMM_DETECT_MOTOR_R_L          = 25,
    COMM_DETECT_MOTOR_FLUX_LINKAGE = 26,
    COMM_DETECT_ENCODER            = 27,
    COMM_DETECT_HALL_FOC           = 28,
    COMM_REBOOT                    = 29,
    COMM_ALIVE                     = 30,
    COMM_GET_DECODED_PPM           = 31,
    COMM_GET_DECODED_ADC           = 32,
    COMM_GET_DECODED_CHUK          = 33,
    COMM_FORWARD_CAN               = 34,
    COMM_SET_CHUCK_DATA            = 35,
    COMM_CUSTOM_APP_DATA           = 36,
    COMM_NRF_START_PAIRING         = 37
} COMM_PACKET_ID;

// VESC motor values structure
typedef struct {
    float temp_fet;
    float temp_motor;
    float avg_motor_current;
    float avg_input_current;
    float avg_id;
    float avg_iq;
    float duty_now;
    float rpm;
    float v_in;
    float amp_hours;
    float amp_hours_charged;
    float watt_hours;
    float watt_hours_charged;
    int32_t tachometer;
    int32_t tachometer_abs;
    uint8_t fault_code;
    float pid_pos_now;
    uint8_t vesc_id;
} vesc_values_t;

// Function prototypes
void vesc_uart_init(UART_HandleTypeDef* huart);
void vesc_set_duty(float duty);
void vesc_set_current(float current);
void vesc_set_current_brake(float current);
void vesc_set_rpm(int32_t rpm);
void vesc_set_pos(float pos);
void vesc_get_values(void);
void vesc_uart_process(void (*vesc_values_received_cb)(vesc_values_t*));

#endif

#endif