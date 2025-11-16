#include "vesc_uart.h"
#include <string.h>

static UART_HandleTypeDef *vesc_huart;
static uint8_t rx_buffer[512];
static uint16_t rx_index = 0;
static vesc_values_t motor_values;

// CRC16 calculation
static uint16_t crc16(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

// Buffer functions for unpacking data
static int32_t buffer_get_int32(uint8_t *buffer, int32_t *index)
{
    int32_t res = ((uint32_t)buffer[*index]) << 24 |
                  ((uint32_t)buffer[*index + 1]) << 16 |
                  ((uint32_t)buffer[*index + 2]) << 8 |
                  ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}

static int16_t buffer_get_int16(uint8_t *buffer, int32_t *index)
{
    int16_t res = ((uint16_t)buffer[*index]) << 8 |
                  ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}

static float buffer_get_float16(uint8_t *buffer, float scale, int32_t *index)
{
    return (float)buffer_get_int16(buffer, index) / scale;
}

static float buffer_get_float32(uint8_t *buffer, float scale, int32_t *index)
{
    return (float)buffer_get_int32(buffer, index) / scale;
}

// Send packet
static void send_packet(uint8_t *data, uint16_t len)
{
    uint8_t packet[len + 7];
    uint16_t index = 0;

    if (len <= 256)
    {
        packet[index++] = 2;
        packet[index++] = len;
    }
    else
    {
        packet[index++] = 3;
        packet[index++] = (len >> 8) & 0xFF;
        packet[index++] = len & 0xFF;
    }

    memcpy(&packet[index], data, len);
    index += len;

    uint16_t crc = crc16(data, len);
    packet[index++] = (crc >> 8) & 0xFF;
    packet[index++] = crc & 0xFF;
    packet[index++] = 3;

    HAL_UART_Transmit(vesc_huart, packet, index, 100);
}

// Initialize VESC UART
void vesc_uart_init(UART_HandleTypeDef *huart)
{
    vesc_huart = huart;
    rx_index = 0;
}

// Set duty cycle (-1.0 to 1.0)
void vesc_set_duty(float duty)
{
    uint8_t buffer[5];
    int32_t duty_int = (int32_t)(duty * 100000.0f);

    buffer[0] = COMM_SET_DUTY;
    buffer[1] = (duty_int >> 24) & 0xFF;
    buffer[2] = (duty_int >> 16) & 0xFF;
    buffer[3] = (duty_int >> 8) & 0xFF;
    buffer[4] = duty_int & 0xFF;

    send_packet(buffer, 5);
}

// Set current in amps
void vesc_set_current(float current)
{
    uint8_t buffer[5];
    int32_t current_ma = (int32_t)(current * 1000.0f);

    buffer[0] = COMM_SET_CURRENT;
    buffer[1] = (current_ma >> 24) & 0xFF;
    buffer[2] = (current_ma >> 16) & 0xFF;
    buffer[3] = (current_ma >> 8) & 0xFF;
    buffer[4] = current_ma & 0xFF;

    send_packet(buffer, 5);
}

// Set brake current
void vesc_set_current_brake(float current)
{
    uint8_t buffer[5];
    int32_t current_ma = (int32_t)(current * 1000.0f);

    buffer[0] = COMM_SET_CURRENT_BRAKE;
    buffer[1] = (current_ma >> 24) & 0xFF;
    buffer[2] = (current_ma >> 16) & 0xFF;
    buffer[3] = (current_ma >> 8) & 0xFF;
    buffer[4] = current_ma & 0xFF;

    send_packet(buffer, 5);
}

// Set RPM
void vesc_set_rpm(int32_t rpm)
{
    uint8_t buffer[5];

    buffer[0] = COMM_SET_RPM;
    buffer[1] = (rpm >> 24) & 0xFF;
    buffer[2] = (rpm >> 16) & 0xFF;
    buffer[3] = (rpm >> 8) & 0xFF;
    buffer[4] = rpm & 0xFF;

    send_packet(buffer, 5);
}

// Set position
void vesc_set_pos(float pos)
{
    uint8_t buffer[5];
    int32_t pos_int = (int32_t)(pos * 1000000.0f);

    buffer[0] = COMM_SET_POS;
    buffer[1] = (pos_int >> 24) & 0xFF;
    buffer[2] = (pos_int >> 16) & 0xFF;
    buffer[3] = (pos_int >> 8) & 0xFF;
    buffer[4] = pos_int & 0xFF;

    send_packet(buffer, 5);
}

// Request motor values
void vesc_get_values(void)
{
    uint8_t buffer[1];
    buffer[0] = COMM_GET_VALUES;
    send_packet(buffer, 1);
}

// Process received packet
static void process_packet(
    uint8_t *data,
    uint16_t len,
    void (*vesc_values_received_cb)(vesc_values_t *))
{
    if (len < 1)
        return;

    COMM_PACKET_ID cmd = data[0];

    if (cmd == COMM_GET_VALUES)
    {
        int32_t ind = 1;

        motor_values.temp_fet = buffer_get_float16(data, 10.0, &ind);
        motor_values.temp_motor = buffer_get_float16(data, 10.0, &ind);
        motor_values.avg_motor_current = buffer_get_float32(data, 100.0, &ind);
        motor_values.avg_input_current = buffer_get_float32(data, 100.0, &ind);
        motor_values.avg_id = buffer_get_float32(data, 100.0, &ind);
        motor_values.avg_iq = buffer_get_float32(data, 100.0, &ind);
        motor_values.duty_now = buffer_get_float16(data, 1000.0, &ind);
        motor_values.rpm = buffer_get_float32(data, 1.0, &ind);
        motor_values.v_in = buffer_get_float16(data, 10.0, &ind);
        motor_values.amp_hours = buffer_get_float32(data, 10000.0, &ind);
        motor_values.amp_hours_charged = buffer_get_float32(data, 10000.0, &ind);
        motor_values.watt_hours = buffer_get_float32(data, 10000.0, &ind);
        motor_values.watt_hours_charged = buffer_get_float32(data, 10000.0, &ind);
        motor_values.tachometer = buffer_get_int32(data, &ind);
        motor_values.tachometer_abs = buffer_get_int32(data, &ind);
        motor_values.fault_code = data[ind++];
        motor_values.pid_pos_now = buffer_get_float32(data, 1000000.0, &ind);
        motor_values.vesc_id = data[ind++];

        vesc_values_received_cb(&motor_values);
    }
}

// UART receive processing (call this in main loop or use DMA)
void vesc_uart_process(void (*vesc_values_received_cb)(vesc_values_t *))
{
    uint8_t byte;

    if (HAL_UART_Receive(vesc_huart, &byte, 1, 10) == HAL_OK)
    {
        rx_buffer[rx_index++] = byte;

        // Check for complete packet
        if (rx_index >= 2)
        {
            uint16_t payload_len = 0;
            uint16_t packet_start = 0;

            if (rx_buffer[0] == 2)
            {
                payload_len = rx_buffer[1];
                packet_start = 2;
            }
            else if (rx_buffer[0] == 3 && rx_index >= 3)
            {
                payload_len = (rx_buffer[1] << 8) | rx_buffer[2];
                packet_start = 3;
            }

            uint16_t packet_len = packet_start + payload_len + 3; // +3 for CRC and end byte

            if (rx_index >= packet_len && rx_buffer[packet_len - 1] == 3)
            {
                // Verify CRC
                uint16_t crc_calc = crc16(&rx_buffer[packet_start], payload_len);
                uint16_t crc_recv = (rx_buffer[packet_start + payload_len] << 8) |
                                    rx_buffer[packet_start + payload_len + 1];

                if (crc_calc == crc_recv)
                {
                    process_packet(&rx_buffer[packet_start], payload_len, vesc_values_received_cb);
                }

                rx_index = 0;
            }
        }

        if (rx_index >= 512)
            rx_index = 0; // Overflow protection
    }
}