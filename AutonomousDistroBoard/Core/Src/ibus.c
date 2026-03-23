#include "ibus.h"
#include <stdint.h>


volatile uint16_t g_dma_last_pos = 0;

uint8_t g_ibus_frame[IBUS_FRAME_SIZE] = {0};
uint8_t g_ibus_dma_buffer[IBUS_DMA_BUFFER_SIZE] = {0};
uint8_t g_ibus_index = 0;
uint16_t g_ibus_channels[IBUS_CHANNEL_COUNT] = {0};


void ibus_process(UART_HandleTypeDef *sbus_huart) {
    uint16_t dma_pos = IBUS_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(sbus_huart->hdmarx);

    while (g_dma_last_pos != dma_pos) {
        uint8_t byte = g_ibus_dma_buffer[g_dma_last_pos++];

        if (g_dma_last_pos >= IBUS_DMA_BUFFER_SIZE) {
            g_dma_last_pos = 0;
		}

        ibus_parse_byte(byte);
    }
}



void ibus_parse_byte(uint8_t byte) {
	// Byte 0:     Length = 0x20 (32)
	// Byte 1:     Command = 0x40
	// Byte 2-29:  14 channels, little-endian uint16
	// Byte 30-31: checksum (little-endian)

    if (g_ibus_index == 0 && byte != 0x20) {
        return; // wait for frame start
	}

    g_ibus_frame[g_ibus_index++] = byte;
    if (g_ibus_index == IBUS_FRAME_SIZE) {
        g_ibus_index = 0;
        if (g_ibus_frame[1] == 0x40) {
            uint16_t checksum = 0xFFFF;
            for (int i = 0; i < 30; i++) {
                checksum -= g_ibus_frame[i];
			}

            uint16_t received = g_ibus_frame[30] | (g_ibus_frame[31] << 8);
            if (checksum == received) {
                for (int i = 0; i < IBUS_CHANNEL_COUNT; i++) {
                    g_ibus_channels[i] = g_ibus_frame[2 + i*2] | (g_ibus_frame[3 + i*2] << 8);
				}
            }
        }
    }
}